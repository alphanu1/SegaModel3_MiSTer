// SPDX-License-Identifier: GPL-3.0-or-later
//
// ppc_dcache bench. Random CPU-side traffic in the core's pipelined protocol
// (a request in any cycle d_rdy is high, mostly back to back, answers in
// order) against an exact reference: the cache's contents (tag, dirty, data per line, PLRU) and
// memory, updated in program order as each request is issued. Checked:
//   - every load's lanes;
//   - every line allocation (set, way, tag), in order, against the
//     reference's victim choice;
//   - every single access the port makes (cache-inhibited, disabled, locked
//     miss, write-through, the bypass page) against the reference's list, in
//     order, with lanes and store data;
//   - after a dcbf of every line, the whole memory image.
// With snoops on (arg 6: 1 reads, 2 reads and writes; study D-41) a DMA
// master snoops lines, mostly ones the CPU side has just stored to, at random
// gaps (often back to back), and reads memory as soon as sn_ack comes. Each
// byte it reads must be the line's value after every request answered by the
// cycle sn_req rose, or a value one of the requests still unanswered then,
// or made before sn_ack, gave it. Each snoop is applied to the reference as
// it starts (a modified line written back; a read leaves it clean, a write
// (sn_inv = 1) invalidates it). Where the order matters, the CPU side keeps
// out of the way: for a read, no store, dcbz or dcbi to the line is
// unanswered as it starts or made before sn_ack (they decide whether the
// line is modified afterwards, which dcbi and DCFI expose), nor any
// inhibited access (on a modified line that is 603e 3.6.4.1's boundedly
// undefined case, and it would see memory before or after the push); for a
// write, nothing to its set, except an lwarx made as an inhibited load (it
// never looks at the arrays) to a line that was not modified, for the
// reservation rule; the master's write then leaves memory as it was. A
// write snoop of a clean line is sometimes started with a flash invalidate
// (HID0[DCFI]), which may start before or after it. At sn_ack the master writes some of the
// line's words, in memory and in the reference. Some word loads are made as
// lwarx (d_rsv), and res_kill is checked every cycle against a model of the
// rule in ppc_dcache.sv. SNOOP_HOSTILE=1 adds long acknowledge delays and
// beat gaps on the memory side.
// The address space is a few sets and many pages, so lines are evicted and
// written back all the time. Some accesses to cacheable pages are made with
// I = 1 or W = 1 while their line is resident, so an access that wrongly
// looks at the arrays reads or writes a different value than the reference.
//
// usage: Vppc_dcache [ops] [seed] [lat_lo lat_hi] [stream] [snoop]

#include "Vppc_dcache.h"
#ifdef SN_DEBUG
#include "Vppc_dcache___024root.h"
#endif
#include "verilated.h"
#include "cache_mem.h"
#include <algorithm>
#include <deque>
#include <cstdlib>
#include <array>
#include <map>
#include <set>

static const uint32_t RSV = 0xF00FF;

// ------------------------------------------------------------- reference
struct RefLine { bool v = false; uint32_t tag = 0; bool d = false; uint32_t w[8]; };
struct Alloc { unsigned set, way; uint32_t tag; };

struct Ref {
  RefLine c[128][4];
  uint8_t plru[128] = {};
  Memory mem;
  std::vector<Single> singles;
  std::vector<Alloc> allocs;
  bool dce = true, dlock = false, noopti = false;
  uint64_t hits = 0, misses = 0, wbs = 0, byp = 0;
  uint32_t last_evict = 0;                    // the line most recently cast out (or flushed)

  static unsigned victim_plru(uint8_t p) { return (p & 1) ? 2 + ((p >> 2) & 1) : ((p >> 1) & 1); }
  void touch(unsigned s, unsigned w) {
    uint8_t p = plru[s];
    p = (p & ~1) | ((w >> 1) ? 0 : 1);
    if (!(w >> 1)) p = (p & ~2) | ((w & 1) ? 0 : 2);
    else           p = (p & ~4) | ((w & 1) ? 0 : 4);
    plru[s] = p;
  }
  int find(uint32_t a) {
    unsigned s = (a >> 5) & 127; uint32_t t = a >> 12;
    for (int w = 0; w < 4; w++) if (c[s][w].v && c[s][w].tag == t) return w;
    return -1;
  }
  void writeback(unsigned s, unsigned w) {
    RefLine &L = c[s][w];
#ifdef SN_DEBUG
    { static long wa = getenv("WATCH") ? strtol(getenv("WATCH"), 0, 16) : -1;
      if (((L.tag << 12) | (s << 5)) == (uint32_t(wa) & ~31u)) std::printf("   REF writeback %08X word %08X dirty %d\n", (L.tag << 12) | (s << 5), L.w[(wa >> 2) & 7], int(L.d)); }
#endif
    for (int k = 0; k < 8; k++) mem.wr((L.tag << 12) | (s << 5) | (k << 2), L.w[k], 0xF);
    last_evict = (L.tag << 12) | (s << 5);
    wbs++;
  }
  unsigned alloc(uint32_t a, bool fill) {
    unsigned s = (a >> 5) & 127;
    int v = -1;
    for (int w = 0; w < 4 && v < 0; w++) if (!c[s][w].v) v = w;
    if (v < 0) v = victim_plru(plru[s]);
    RefLine &L = c[s][v];
    if (L.v && L.d) writeback(s, v);
    L.v = true; L.tag = a >> 12; L.d = false;
    for (int k = 0; k < 8; k++) L.w[k] = fill ? mem.rd((a & ~31u) | (k << 2)) : 0;
    touch(s, v);
    allocs.push_back({s, (unsigned)v, a >> 12});
    return v;
  }
  static uint32_t merge(uint32_t o, uint32_t d, unsigned be) {
    uint32_t m = 0;
    for (int l = 0; l < 4; l++) if (be & (8 >> l)) m |= 0xFFu << (24 - 8 * l);
    return (o & ~m) | (d & m);
  }
  bool bypass(uint32_t a, unsigned wimg) const { return (wimg & 4) || !dce || (a >> 12) == RSV; }

  uint32_t load(uint32_t a, unsigned be, unsigned wimg) {
    if (bypass(a, wimg)) { byp++; singles.push_back({false, a, be, 0}); return mem.rd(a); }
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) { hits++; touch(s, w); return c[s][w].w[(a >> 2) & 7]; }
    misses++;
    if (dlock) { singles.push_back({false, a, be, 0}); return mem.rd(a); }
    w = alloc(a, true);
    return c[s][w].w[(a >> 2) & 7];
  }
  void store(uint32_t a, unsigned be, uint32_t d, unsigned wimg) {
    bool wt = wimg & 8;
    if (bypass(a, wimg)) { byp++; singles.push_back({true, a, be, d}); mem.wr(a, d, be); return; }
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) {
      hits++; touch(s, w);
      RefLine &L = c[s][w];
      L.w[(a >> 2) & 7] = merge(L.w[(a >> 2) & 7], d, be);
      if (wt) { singles.push_back({true, a, be, d}); mem.wr(a, d, be); }
      else L.d = true;
      return;
    }
    misses++;
    if (wt || dlock) { singles.push_back({true, a, be, d}); mem.wr(a, d, be); return; }
    w = alloc(a, true);
    RefLine &L = c[s][w];
    L.w[(a >> 2) & 7] = merge(L.w[(a >> 2) & 7], d, be);
    L.d = true;
  }
  void dcbt(uint32_t a, unsigned wimg) {
    if (noopti || !dce || dlock || bypass(a, wimg) || (wimg & 8) || (wimg & 1)) return;
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) { touch(s, w); return; }
    alloc(a, true);
  }
  void dcbz(uint32_t a, unsigned wimg) {
    if ((wimg & 4) || (wimg & 8)) return;
    bool rsv = (a >> 12) == RSV;
    int w = rsv ? -1 : find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) {
      touch(s, w); for (auto &x : c[s][w].w) x = 0; c[s][w].d = true; return;
    }
    if (!dlock && !rsv) { w = alloc(a, false); c[s][w].d = true; return; }
    for (int k = 0; k < 8; k++) mem.wr((a & ~31u) | (k << 2), 0, 0xF);
  }
  void dcbst(uint32_t a) {
    if ((a >> 12) == RSV) return;
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0 && c[s][w].d) { writeback(s, w); c[s][w].d = false; }
  }
  void dcbf(uint32_t a) {
    if ((a >> 12) == RSV) return;
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) { if (c[s][w].d) writeback(s, w); c[s][w].v = false; c[s][w].d = false; }
  }
  void dcbi(uint32_t a) {
    if ((a >> 12) == RSV) return;
    int w = find(a); unsigned s = (a >> 5) & 127;
    if (w >= 0) { c[s][w].v = false; c[s][w].d = false; }
  }
  // the line as another master sees it through the snoop: the cache's copy if
  // modified, else memory (a clean copy differs from memory only after an
  // inhibited store to a resident line, 603e 3.6.4.1's boundedly undefined
  // case, and a snoop does not push it)
  void view(uint32_t line, uint32_t v[8]) {
    int w = find(line); unsigned s = (line >> 5) & 127;
    bool d = w >= 0 && c[s][w].d;
    for (int k = 0; k < 8; k++) v[k] = d ? c[s][w].w[k] : mem.rd(line | (k << 2));
  }
  // a snooped read (the caching-inhibited read of 603e table 3-6): written
  // back if modified, then clean
  void snoop_rd(uint32_t line) {
    if ((line >> 12) == RSV || !dce) return;
    int w = find(line); unsigned s = (line >> 5) & 127;
    if (w >= 0 && c[s][w].d) { writeback(s, w); c[s][w].d = false; }
  }
  // a snooped write (603e table 3-6): written back if modified, invalidated
  void snoop_inv(uint32_t line) {
    if ((line >> 12) == RSV || !dce) return;
    int w = find(line); unsigned s = (line >> 5) & 127;
    if (w >= 0) { if (c[s][w].d) writeback(s, w); c[s][w].v = false; c[s][w].d = false; }
  }
  void flash() { for (auto &s : c) for (auto &l : s) { l.v = false; l.d = false; } for (auto &p : plru) p = 0; }
};

// ------------------------------------------------------------- the bench
static std::mt19937 rng;
static int rnd(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }

enum { OP_LD, OP_ST, OP_TCH, OP_DCBZ, OP_DCBST, OP_DCBF, OP_DCBI };
struct Op { int k; uint32_t a; unsigned be; uint32_t d; unsigned wimg; uint32_t exp; uint64_t t0; bool ref_hit; bool rsv = false; };

// pages: cacheable, write-through, inhibited, the bypass page
static const uint32_t PG_C[]  = { 0x00000, 0x00001, 0x00002, 0x00003, 0x00004, 0x00005, 0x00006, 0x00007,
                                  0x00010, 0x00011, 0x00012, 0x00013, 0xFF800, 0xFFFFF, 0x7FFFF, 0x80000 };
static const uint32_t PG_W[]  = { 0x00020, 0x00021 };
static const uint32_t PG_I[]  = { 0xF0040, 0xF0041 };
static const unsigned SETS[]  = { 0, 1, 2, 127 };

static Ref *gref;
static uint32_t pick_addr(unsigned &wimg, int &cls) {
  // an inhibited access to the line just cast out: it must reach memory after
  // the write-back (the order the port keeps)
  if (gref && gref->last_evict && rnd(0, 99) < 3) {
    wimg = 4; cls = 0;
    return gref->last_evict | (rnd(0, 7) << 2);
  }
  int r = rnd(0, 99);
  uint32_t pg;
  if (r < 40)      { pg = PG_C[rnd(0, 3)];  wimg = 0; cls = 0; }   // hot: four pages
  else if (r < 76) { pg = PG_C[rnd(0, 15)]; wimg = 0; cls = 0; }
  else if (r < 84) { pg = PG_W[rnd(0, 1)];  wimg = 8; cls = 1; }
  else if (r < 94) { pg = PG_I[rnd(0, 1)];  wimg = 4 | 1; cls = 2; }
  else             { pg = RSV;              wimg = 0; cls = 3; }
  unsigned set = rnd(0, 9) < 8 ? SETS[rnd(0, 3)] : rnd(0, 127);
  uint32_t a = (pg << 12) | (set << 5) | (rnd(0, 7) << 2);
  // the paradox cases: a resident cacheable line accessed inhibited or write-through
  if (cls == 0 && rnd(0, 99) < 4) wimg = rnd(0, 1) ? 4 : 8;
  if (cls == 0 && rnd(0, 99) < 3) wimg |= 1;        // guarded: only dcbt cares
  return a;
}
static void pick_size(uint32_t &a, unsigned &be) {
  int s = rnd(0, 2);
  if (s == 0) { unsigned b = rnd(0, 3); be = 8 >> b; a |= b; }
  else if (s == 1) { unsigned h = rnd(0, 1); be = h ? 3 : 12; a |= h * 2; }
  else be = 15;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long nops = argc > 1 ? atol(argv[1]) : 200000;
  unsigned seed = argc > 2 ? atoi(argv[2]) : 1;
  int lat_lo = argc > 3 ? atoi(argv[3]) : 12, lat_hi = argc > 4 ? atoi(argv[4]) : 15;
  bool stream_only = argc > 5 && atoi(argv[5]);
  int snoop_mode = argc > 6 ? atoi(argv[6]) : 0;
  rng.seed(seed);
  std::mt19937 srng(seed ^ 0x3C6EF372u);          // the DMA master's own choices
  auto srnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(srng); };

  Vppc_dcache *t = new Vppc_dcache;
  Ref ref;
  gref = &ref;
  Memory dmem;
  MemAgent ma; ma.rng = &rng; ma.mem = &dmem; ma.lat_lo = lat_lo; ma.lat_hi = lat_hi;
  if (getenv("CACHE_FIXED")) { ma.ack_max = 0; ma.lat_wide = 0; ma.gap_max = 0; }   // deterministic memory: ack at once, beats every cycle
  if (snoop_mode && getenv("SNOOP_HOSTILE")) { ma.ack_max = 24; ma.gap_max = 6; ma.lat_wide = 30; }

  uint64_t cyc = 0;
  int errors = 0;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++; };

  t->rst = 1; t->d_req = 0; t->hid_dce = 1; t->hid_dlock = 0; t->hid_dcfi = 0; t->hid_noopti = 0;
  t->m_ack = 0; t->m_wnext = 0; t->m_rvalid = 0; t->m_rdata = 0;
  t->sn_req = 0; t->sn_addr = 0; t->sn_inv = 0; t->d_rsv = 0;
  for (int i = 0; i < 4; i++) tick();
  t->rst = 0;

  std::deque<Op> outq;                          // requests taken, not yet answered, in order
  Op cur{};
  std::vector<Alloc> dut_allocs;
  uint64_t n_ld = 0, n_ldchk = 0, n_st = 0, n_ops[7] = {}, n_dce = 0, n_dlock = 0, n_noopti = 0, n_dcfi = 0;
  uint64_t lat_sum[4] = {}, lat_n[4] = {}, lat_min[4] = {99, 99, 99, 99}, lat_max[4] = {};
  std::map<int, uint64_t> hist_ldhit;
  long issued = 0;
  int gap = 0;
  enum { RUN, HID_WAIT, DCFI_WAIT, FLUSH, DONE } phase = RUN;
  int hid_kind = 0, dcfi_hold = 0;
  uint32_t flush_i = 0;
  std::vector<uint32_t> flush_list;
  for (uint32_t pg : PG_C) for (int s = 0; s < 128; s++) flush_list.push_back((pg << 12) | (s << 5));
  for (uint32_t pg : PG_W) for (int s = 0; s < 128; s++) flush_list.push_back((pg << 12) | (s << 5));
  for (uint32_t pg : PG_I) for (int s = 0; s < 128; s++) flush_list.push_back((pg << 12) | (s << 5));

  // directed hit streams first: one line warmed, then back-to-back loads, then stores
  std::vector<Op> script;
  {
    uint32_t a = 0x00003040;
    script.push_back({OP_LD, a, 15, 0, 0, 0, 0, false});
    for (int i = 0; i < 64; i++) script.push_back({OP_LD, a | ((i & 7) << 2), 15, 0, 0, 0, 0, false});
    for (int i = 0; i < 64; i++) script.push_back({OP_ST, a | ((i & 7) << 2), 15, (uint32_t)(0x1000 + i), 0, 0, 0, false});
    for (int i = 0; i < 64; i++)                                  // store then load of the same word
      script.push_back({(i & 1) ? OP_LD : OP_ST, a | (((i >> 1) & 7) << 2), 15, (uint32_t)(0x2000 + i), 0, 0, 0, false});
  }
  size_t script_i = 0;

  // ---- the DMA master (snoop_mode)
  struct ViewLog { uint64_t seq; uint32_t line; uint32_t vb[8], va[8]; };
  std::deque<ViewLog> vlog;                     // the latest requests that changed a line's view
  uint64_t n_seq = 0, n_ans = 0;                // requests made, answered
  struct Snoop {
    bool act = false, inv = false; uint32_t line = 0; uint64_t t0 = 0;
    uint32_t v0[8]; std::vector<std::array<uint32_t, 8>> maybe; int kind = 0; bool same = false;
  } sn;
  uint32_t force_line = 0; bool force = false;   // a write snoop to start with a flash invalidate
  int dcfi_pend = -1;                            // cycles until that flash invalidate fires
  uint64_t sn_dcfi = 0, sn_rsv_in = 0;
  int sn_gap = 50;
  std::deque<uint32_t> recent_st, recent_rq;   // lines the CPU side stored to, or touched, lately
  uint64_t sn_n[2] = {}, sn_kind[2][3] = {}, sn_lat_sum[2][3] = {}, sn_lat_max[2][3] = {}, sn_bytes = 0;
  uint64_t sn_race_out = 0, sn_race_mem = 0, sn_race_win = 0, sn_race_st = 0, sn_b2b = 0, sn_dmaw = 0;
  uint64_t n_rsv = 0, n_kill_cyc = 0, n_kill_set = 0;
  uint64_t sn_race_sb = 0, sn_race_fill = 0, sn_race_evict = 0, sn_race_miss = 0;
  std::map<uint32_t, uint64_t> last_st_cyc, last_alloc_cyc;   // by line: the latest store made, allocation decided
  uint64_t last_miss_cyc = 0; uint32_t last_evict_seen = 0; uint64_t last_evict_cyc = 0;
  // the reservation rule's model (ppc_dcache.sv, "the snoop")
  bool m_rsvq = false, m_rvkill = false; uint32_t m_rvpa = 0, m_rsvline = 0;
  uint64_t last_ack = 0;
  auto log_view = [&](uint32_t line, const uint32_t *vb) {
    ViewLog e; e.seq = n_seq; e.line = line;
    for (int k = 0; k < 8; k++) e.vb[k] = vb[k];
    ref.view(line, e.va);
    if (memcmp(e.vb, e.va, sizeof e.va)) {
      vlog.push_back(e); if (vlog.size() > 64) vlog.pop_front();
      if (sn.act && line == sn.line) { std::array<uint32_t, 8> v; for (int k = 0; k < 8; k++) v[k] = e.va[k]; sn.maybe.push_back(v); sn_race_win++; }
    }
  };
  uint64_t s_t0 = 0, s_mark[4] = {};

  long max_cyc = nops * 200 + 200000;
  while (phase != DONE && (long)cyc < max_cyc) {
    // ---------------------------------------------------- outputs of the last edge
    bool dv = t->d_valid; uint32_t drd = t->d_rdata;
    if (t->t_alloc) {
      dut_allocs.push_back({t->t_alloc_set, t->t_alloc_way, t->t_alloc_tag});
      if (snoop_mode) { last_alloc_cyc[(t->t_alloc_tag << 12) | (t->t_alloc_set << 5)] = cyc; last_miss_cyc = cyc; }
    }
    if (snoop_mode && ref.last_evict != last_evict_seen) { last_evict_seen = ref.last_evict; last_evict_cyc = cyc; }

    t->d_req = 0; t->d_rsv = 0;
    if (dv && !outq.empty()) {
      cur = outq.front(); outq.pop_front(); n_ans++;
      unsigned lat = cyc - cur.t0;
      int cls = (cur.k == OP_LD) ? (cur.ref_hit ? 0 : 1) : (cur.k == OP_ST) ? (cur.ref_hit ? 2 : 3) : -1;
      if (cls >= 0) { lat_sum[cls] += lat; lat_n[cls]++; lat_min[cls] = std::min<uint64_t>(lat_min[cls], lat); lat_max[cls] = std::max<uint64_t>(lat_max[cls], lat); }
      if (cls == 0) hist_ldhit[lat]++;
      if (cur.k == OP_LD) {
        uint32_t m = 0;
        for (int l = 0; l < 4; l++) if (cur.be & (8 >> l)) m |= 0xFFu << (24 - 8 * l);
        n_ldchk++;
        if ((drd & m) != (cur.exp & m)) {
          if (errors < 10) std::printf("LOAD MISMATCH cyc %lu addr %08X be %X wimg %X got %08X exp %08X\n",
                                       (unsigned long)cyc, cur.a, cur.be, cur.wimg, drd, cur.exp);
          errors++;
        }
      }
    } else if (dv) { std::printf("spurious d_valid at %lu\n", (unsigned long)cyc); errors++; }

    // ---------------------------------------------------- CPU side
    // a HID0 change or the end waits for every answer
    bool quiet_cpu = outq.empty();
    {
      if (phase == RUN && script_i >= script.size() && issued >= nops) { phase = FLUSH; }
      if (phase == RUN && script_i >= script.size() && !stream_only && rnd(0, 999) < 3) { phase = HID_WAIT; hid_kind = rnd(0, 9); }
      if (stream_only && script_i >= script.size() && phase == RUN) phase = FLUSH;
      if (phase == HID_WAIT && dcfi_pend > 0) dcfi_pend--;
      else if (phase == HID_WAIT && dcfi_pend == 0) {
        t->hid_dcfi = 1; dcfi_hold = 0; phase = DCFI_WAIT; ref.flash(); n_dcfi++; dcfi_pend = -1;
      } else if (phase == HID_WAIT) {
        if (t->d_idle && ma.idle() && quiet_cpu && !sn.act) {
          if (hid_kind < 3) { ref.dce = !ref.dce; t->hid_dce = ref.dce; n_dce++; phase = RUN; }
          else if (hid_kind < 6) { ref.dlock = !ref.dlock; t->hid_dlock = ref.dlock; n_dlock++; phase = RUN; }
          else if (hid_kind < 7) { ref.noopti = !ref.noopti; t->hid_noopti = ref.noopti; n_noopti++; phase = RUN; }
          else if (ref.dce) {
            if (snoop_mode >= 2 && srnd(0, 1))           // a clean resident line, snooped (written) as DCFI starts
              for (int k = 0; k < 64 && !force; k++) {
                unsigned st = k < 16 ? SETS[k & 3] : srnd(0, 127), w = srnd(0, 3);
                if (ref.c[st][w].v && !ref.c[st][w].d && ref.c[st][w].tag != RSV) { force = true; force_line = (ref.c[st][w].tag << 12) | (st << 5); }
              }
            if (force) dcfi_pend = srnd(0, 4);           // the snoop starts now, DCFI 0-4 cycles later
            else { t->hid_dcfi = 1; dcfi_hold = 0; phase = DCFI_WAIT; ref.flash(); n_dcfi++; }
          }
          else phase = RUN;
          // keep the cache mostly enabled and unlocked
          if (phase == RUN && !ref.dce && rnd(0, 1)) { ref.dce = true; t->hid_dce = 1; }
          if (phase == RUN && ref.dlock && rnd(0, 1)) { ref.dlock = false; t->hid_dlock = 0; }
        }
      } else if (phase == DCFI_WAIT) {
        if (t->dcfi_ack) dcfi_hold = 1;
        if (dcfi_hold && ++dcfi_hold > 2) { t->hid_dcfi = 0; phase = RUN; }
      } else if (phase == FLUSH) {
        if (!t->d_rdy) ;
        else if (flush_i < flush_list.size()) {
          cur = {OP_DCBF, flush_list[flush_i++], 15, 0, 0, 0, cyc, false};
          ref.dcbf(cur.a);
          t->d_req = 1; t->d_we = 0; t->d_addr = cur.a; t->d_be = 15; t->d_wdata = 0; t->d_wimg = 0; t->d_op = 4;
          outq.push_back(cur); n_seq++;
        } else if (t->d_idle && ma.idle() && quiet_cpu && !sn.act) phase = DONE;
      } else if (phase == RUN && gap > 0) {
        gap--;
      } else if (phase == RUN && script_i == 1 && !(quiet_cpu && t->d_idle)) {
        ;                                       // the streams start once the warming fill is done
      } else if (phase == RUN && t->d_rdy) {
        if (script_i == script.size() && s_t0 == 0) s_t0 = cyc;
        Op o{};
        if (script_i < script.size()) {
          o = script[script_i];
          if (script_i == 1) s_mark[0] = cyc;
          if (script_i == 65) s_mark[1] = cyc;
          if (script_i == 129) s_mark[2] = cyc;
          if (script_i == 193) s_mark[3] = cyc;
          script_i++;
        } else {
          int r = rnd(0, 99);
          int cls;
          o.a = pick_addr(o.wimg, cls);
          for (int tries = 0; sn.act && sn.inv && ((o.a >> 5) & 127) == ((sn.line >> 5) & 127) && tries < 20; tries++)
            o.a = pick_addr(o.wimg, cls);
          if (sn.act && sn.inv && ((o.a >> 5) & 127) == ((sn.line >> 5) & 127)) o.a ^= 1u << 5;   // the next set
          o.k = r < 45 ? OP_LD : r < 85 ? OP_ST : r < 88 ? OP_TCH : r < 91 ? OP_DCBZ : r < 94 ? OP_DCBST :
                r < 97 ? OP_DCBF : OP_DCBI;
          if (sn.act && !sn.inv && (o.a & ~31u) == sn.line) {      // (a read snoop's line: see the header)
            if (o.k == OP_ST || o.k == OP_DCBZ || o.k == OP_DCBI) o.k = OP_LD;
            if (cls == 0) o.wimg &= ~4u;
          }
          bool rsv_in = sn.act && sn.inv && sn.kind != 2 && srnd(0, 3) == 0;   // an lwarx in a write snoop's window, to its line
          if (rsv_in) { o.k = OP_LD; o.a = sn.line | (srnd(0, 7) << 2); o.wimg = 4; sn.same = true; sn_rsv_in++; }
          if (rsv_in) o.be = 15;
          else if (o.k == OP_LD || o.k == OP_ST) pick_size(o.a, o.be); else o.be = 15;
          o.d = rng();
          if (snoop_mode && o.k == OP_LD && o.be == 15 && (rsv_in || srnd(0, 99) < 10)) o.rsv = true;   // an lwarx
          issued++;
        }
        o.t0 = cyc;
        uint32_t wa = o.a & ~3u;
        o.ref_hit = (o.k == OP_LD || o.k == OP_ST) && !ref.bypass(wa, o.wimg) && ref.find(wa) >= 0;
        // the store's data sits in its lanes, as the core places it
        uint32_t sd = o.d;
        if (o.k == OP_ST) {
          unsigned b = o.a & 3;
          if (o.be == 15) sd = o.d;
          else if (o.be == 3 || o.be == 12) sd = (o.d & 0xFFFF) * 0x10001u;
          else sd = (o.d & 0xFF) * 0x01010101u;
          (void)b;
          o.d = sd;
        }
        uint32_t vb[8];
        ref.view(wa & ~31u, vb);
        switch (o.k) {
          case OP_LD:    o.exp = ref.load(wa, o.be, o.wimg); n_ld++; break;
          case OP_ST:    ref.store(wa, o.be, o.d, o.wimg); n_st++; break;
          case OP_TCH:   ref.dcbt(wa, o.wimg); break;
          case OP_DCBZ:  ref.dcbz(wa, o.wimg); break;
          case OP_DCBST: ref.dcbst(wa); break;
          case OP_DCBF:  ref.dcbf(wa); break;
          case OP_DCBI:  ref.dcbi(wa); break;
        }
        n_ops[o.k]++;
#ifdef SN_DEBUG
        { static long w = getenv("WATCH") ? strtol(getenv("WATCH"), 0, 16) : -1;
          if (w >= 0 && (wa & ~31u) == (uint32_t(w) & ~31u))
            std::printf("   REQ c%lu k %d a %08X be %X d %08X wimg %X seq %lu\n", (unsigned long)cyc, o.k, o.a, o.be, o.d, o.wimg, (unsigned long)n_seq); }
#endif
        log_view(wa & ~31u, vb);
        n_seq++;
        if (snoop_mode) {
          if (o.k == OP_ST) { recent_st.push_back(wa & ~31u); if (recent_st.size() > 8) recent_st.pop_front(); last_st_cyc[wa & ~31u] = cyc; }
          recent_rq.push_back(wa & ~31u); if (recent_rq.size() > 16) recent_rq.pop_front();
        }
        t->d_rsv = o.rsv; n_rsv += o.rsv;
        t->d_req = 1; t->d_we = (o.k == OP_ST); t->d_addr = wa; t->d_be = o.be; t->d_wdata = o.d;
        t->d_wimg = o.wimg;
        t->d_op = (o.k == OP_LD || o.k == OP_ST) ? 0 : (o.k == OP_TCH) ? 1 : (o.k == OP_DCBZ) ? 2 :
                  (o.k == OP_DCBST) ? 3 : (o.k == OP_DCBF) ? 4 : 5;
        outq.push_back(o);
        gap = (script_i < script.size()) ? 0 : (rnd(0, 3) == 0 ? rnd(1, 3) : 0);
      }
    }

    if (getenv("DC_TRACE") && cyc < 400)
      std::printf("  cyc %lu: rdy %d req %d %s %08X | valid %d %08X | q %zu\n", (unsigned long)cyc, int(t->d_rdy), int(t->d_req),
                  t->d_we ? "st" : "ld", t->d_addr, int(dv), drd, outq.size());
    // ---------------------------------------------------- the DMA master
    if (snoop_mode) {
      bool sack = t->sn_ack;                    // (from the last edge)
      bool inW  = sn.act && sn.inv && cyc > sn.t0 && !sack;   // the snoop in progress in the cache
      if (sack && !sn.act) { std::printf("SNOOP: sn_ack with no snoop at cycle %lu\n", (unsigned long)cyc); errors++; }
      if (sn.act && sack) {
        // the master reads the line from memory now
        unsigned lat = cyc - sn.t0;
        sn_lat_sum[sn.inv][sn.kind] += lat; sn_lat_max[sn.inv][sn.kind] = std::max<uint64_t>(sn_lat_max[sn.inv][sn.kind], lat);
        for (int k = 0; k < 8; k++) {
          uint32_t m = dmem.rd(sn.line | (k << 2));
          for (int b = 0; b < 4; b++) {
            auto by = [&](uint32_t x) { return (x >> (24 - 8 * b)) & 0xFF; };
            bool ok = by(m) == by(sn.v0[k]);
            for (auto &v : sn.maybe) ok = ok || by(m) == by(v[k]);
            sn_bytes++;
            if (!ok) {
              if (errors < 10) std::printf("SNOOP %s line %08X word %d byte %d: memory %02X, expected %02X (%zu later values) at cycle %lu, snoop from %lu\n",
                                           sn.inv ? "write" : "read", sn.line, k, b, by(m), by(sn.v0[k]), sn.maybe.size(),
                                           (unsigned long)cyc, (unsigned long)sn.t0);
              errors++;
            }
          }
        }
        if (sn.inv && !sn.same) {               // the master's write: some words of the line
          int mask = srnd(1, 255);
          for (int k = 0; k < 8; k++) if (mask & (1 << k)) {
            uint32_t d = srng();
            dmem.wr(sn.line | (k << 2), d, 0xF); ref.mem.wr(sn.line | (k << 2), d, 0xF);
          }
          sn_dmaw++;
        }
        sn.act = false; last_ack = cyc;
        int g = srnd(0, 9);
        sn_gap = g < 4 ? 0 : g < 8 ? srnd(1, 20) : srnd(20, 200);
      } else if (sn.act && cyc - sn.t0 > 20000) {
        std::printf("SNOOP: no sn_ack for line %08X since cycle %lu\n", sn.line, (unsigned long)sn.t0); errors++; sn.act = false;
      } else if (!sn.act && ((phase == RUN && script_i >= script.size()) || force) && (ref.dce || force)) {
        // eager: straight after a store, an allocation or a cast-out, at that line
        int eager = 0;
        if (!recent_st.empty() && cyc - last_st_cyc[recent_st.back()] <= 2 && srnd(0, 3) == 0) eager = 1;
        else if (cyc - last_miss_cyc <= 3 && srnd(0, 7) == 0) eager = 2;
        else if (last_evict_seen && cyc - last_evict_cyc <= 3 && srnd(0, 3) == 0) eager = 3;
        if (force) eager = 4;
        if (sn_gap > 0 && !eager) sn_gap--;
        else {
          int r = srnd(0, 99);
          uint32_t line;
          if (eager == 4) { line = force_line; force = false; sn_dcfi++; }
          else if (eager == 1) line = recent_st.back();
          else if (eager == 2) { line = recent_rq.empty() ? 0 : recent_rq.back(); for (auto &kv : last_alloc_cyc) if (kv.second == last_miss_cyc) line = kv.first; }
          else if (eager == 3) line = last_evict_seen & ~31u;
          else if (r >= 90 && r < 95 && n_rsv) line = m_rsvq ? m_rsvline : m_rvpa;   // the reservation's line
          else if (r < 45 && !recent_st.empty()) line = recent_st[srnd(0, recent_st.size() - 1)];
          else if (r < 70 && !recent_rq.empty()) line = recent_rq[srnd(0, recent_rq.size() - 1)];
          else if (r < 85) line = (PG_C[srnd(0, 3)] << 12) | (SETS[srnd(0, 3)] << 5);
          else if (r < 90) line = (PG_C[srnd(0, 15)] << 12) | (srnd(0, 127) << 5);
          else line = ((srnd(0, 1) ? PG_I[srnd(0, 1)] : RSV) << 12) | (srnd(0, 127) << 5);
          bool inv = snoop_mode >= 2 && (eager == 4 || srnd(0, 99) < (r >= 90 && r < 95 && n_rsv ? 80 : 35)), rd_ok = true;
          for (auto &o : outq) {
            if (((o.a >> 5) & 127) == ((line >> 5) & 127)) inv = false;
            if ((o.a & ~31u) == line && (o.k == OP_ST || o.k == OP_DCBZ || o.k == OP_DCBI || (o.wimg & 4))) rd_ok = false;
          }
          if (!inv && !rd_ok) { sn_gap = 1; goto sn_drive; }
          sn.act = true; sn.inv = inv; sn.line = line; sn.t0 = cyc; sn.maybe.clear(); sn.same = false;
          ref.view(line, sn.v0);
          bool oldest = true;                   // undo what requests not yet answered did to the line
          for (auto it = vlog.begin(); it != vlog.end(); ++it)
            if (it->seq >= n_ans && it->line == line) {
              if (oldest) { memcpy(sn.v0, it->vb, sizeof sn.v0); oldest = false; }
              std::array<uint32_t, 8> v; for (int k = 0; k < 8; k++) v[k] = it->va[k]; sn.maybe.push_back(v);
            }
          int w = ref.find(line); unsigned st = (line >> 5) & 127;
          sn.kind = w < 0 ? 0 : ref.c[st][w].d ? 2 : 1;
          sn_n[inv]++; sn_kind[inv][sn.kind]++;
          sn_race_out += !outq.empty(); sn_race_mem += !ma.idle(); sn_race_st += !sn.maybe.empty();
          sn_b2b += (cyc == last_ack + 1);
          { auto i = last_st_cyc.find(line); sn_race_sb += i != last_st_cyc.end() && cyc - i->second <= 3; }
          { auto i = last_alloc_cyc.find(line); sn_race_fill += i != last_alloc_cyc.end() && cyc - i->second <= 24; }
          sn_race_evict += line == last_evict_seen && cyc - last_evict_cyc <= 24;
          sn_race_miss  += cyc - last_miss_cyc <= 24 && !(last_alloc_cyc.count(line) && cyc - last_alloc_cyc[line] <= 24);
          if (inv) ref.snoop_inv(line); else ref.snoop_rd(line);
        }
      }
    sn_drive:
      t->sn_req = sn.act; t->sn_addr = sn.line >> 5; t->sn_inv = sn.inv;
      // the reservation rule, as res_kill must show it this cycle
      t->clk = 0; t->eval();
      bool exp_kill = m_rvkill || (m_rsvq && inW && m_rsvline == sn.line);
      if (bool(t->res_kill) != exp_kill) {
        if (errors < 10) std::printf("RESERVATION res_kill %d, expected %d at cycle %lu (lwarx line %08X, snoop line %08X, window %d)\n",
                                     int(t->res_kill), int(exp_kill), (unsigned long)cyc, m_rsvq ? m_rsvline : m_rvpa, sn.line, int(inW));
        errors++;
      }
      n_kill_cyc += exp_kill;
      bool kill_n;
      if (m_rsvq) { kill_n = inW && m_rsvline == sn.line; m_rvpa = m_rsvline; }
      else kill_n = m_rvkill || (inW && m_rvpa == sn.line);
      n_kill_set += kill_n && !m_rvkill;
      m_rvkill = kill_n;
      m_rsvq = t->d_req && t->d_rsv;
      if (m_rsvq) m_rsvline = t->d_addr & ~31u;
    }

#ifdef SN_DEBUG
    { static long wa = getenv("WATCH") ? strtol(getenv("WATCH"), 0, 16) : -1;
      static uint32_t pd = 0, pr = 0, pv = 0; static bool first = true;
      if (wa >= 0) {
        uint32_t v[8]; ref.view(uint32_t(wa) & ~31u, v);
        uint32_t d = dmem.rd(wa), rm = ref.mem.rd(wa), vw = v[(wa >> 2) & 7];
        if (first || d != pd || rm != pr || vw != pv)
          std::printf("   WATCH c%lu dmem %08X ref.mem %08X ref view %08X (resident %d)\n", (unsigned long)cyc, d, rm, vw, ref.find(wa) >= 0);
        pd = d; pr = rm; pv = vw; first = false;
      }
    }
    {
      static long t_lo = getenv("SN_T0") ? atol(getenv("SN_T0")) : -1, t_hi = getenv("SN_T1") ? atol(getenv("SN_T1")) : -1;
      if ((long)cyc >= t_lo && (long)cyc <= t_hi) {
        auto *r = t->rootp;
        std::printf("  c%lu sn_st %d req %d %08X inv %d ack %d | quiet %d cp %d wbp %d mp %d dq %d mq %d f %d sb %d s %d uq %d | r_v %d x_v %d rdsel %d | dreq %d %s %08X be %X | dv %d\n",
          (unsigned long)cyc, int(r->ppc_dcache__DOT__sn_st), int(t->sn_req), t->sn_addr << 5, int(t->sn_inv), int(t->sn_ack),
          int(r->ppc_dcache__DOT__quiet), int(r->ppc_dcache__DOT__cp_act), int(r->ppc_dcache__DOT__wb_pend), int(r->ppc_dcache__DOT__mp),
          int(r->ppc_dcache__DOT__dq_v), int(r->ppc_dcache__DOT__mq_v), int(r->ppc_dcache__DOT__f_act), int(r->ppc_dcache__DOT__sb_v),
          int(r->ppc_dcache__DOT__s_v), int(r->ppc_dcache__DOT__uq_v), int(r->ppc_dcache__DOT__r_v), int(r->ppc_dcache__DOT__x_v),
          int(r->ppc_dcache__DOT__rd_sel), int(t->d_req), t->d_we ? "st" : "ld", t->d_addr, int(t->d_be), int(t->d_valid));
        if (r->ppc_dcache__DOT__sn_st == 4)
          std::printf("     C: hv %X hit %d hw %d dty %d st_sn %03X dq_a %08X tg %05X %05X %05X %05X | ref kind %d\n",
            int(r->ppc_dcache__DOT__sn_hv), int(r->ppc_dcache__DOT__sn_hit), int(r->ppc_dcache__DOT__sn_hw), int(r->ppc_dcache__DOT__sn_dty),
            int(r->ppc_dcache__DOT__st_sn), uint32_t(r->ppc_dcache__DOT__dq_a) << 2,
            int(r->ppc_dcache__DOT__dq_tg[0]), int(r->ppc_dcache__DOT__dq_tg[1]), int(r->ppc_dcache__DOT__dq_tg[2]), int(r->ppc_dcache__DOT__dq_tg[3]), sn.kind);
      }
    }
#endif
    // ---------------------------------------------------- memory side
    ma.step(t->m_req, t->m_cmd, t->m_addr, t->m_be, t->m_wdata);
    t->m_ack = ma.ack; t->m_rvalid = ma.rvalid; t->m_wnext = ma.wnext; t->m_rdata = ma.rdata;
    tick();
    if (ma.err) { errors += ma.err; ma.err = 0; }
    if (Verilated::gotFinish()) { std::printf("$finish / assertion\n"); errors++; break; }
  }

  if (phase != DONE) { std::printf("TIMEOUT at cycle %lu (phase %d, outstanding %zu)\n", (unsigned long)cyc, phase, outq.size()); errors++; }

  // ---------------------------------------------------- final checks
  if (dut_allocs.size() != ref.allocs.size()) {
    std::printf("ALLOC COUNT dut %zu ref %zu\n", dut_allocs.size(), ref.allocs.size()); errors++;
  }
  for (size_t i = 0; i < std::min(dut_allocs.size(), ref.allocs.size()); i++) {
    auto &a = dut_allocs[i]; auto &b = ref.allocs[i];
    if (a.set != b.set || a.way != b.way || a.tag != b.tag) {
      std::printf("ALLOC %zu dut set %u way %u tag %05X ref set %u way %u tag %05X\n", i, a.set, a.way, a.tag, b.set, b.way, b.tag);
      errors++; break;
    }
  }
  if (ma.log.size() != ref.singles.size()) {
    std::printf("SINGLE COUNT dut %zu ref %zu\n", ma.log.size(), ref.singles.size()); errors++;
  }
  for (size_t i = 0; i < std::min(ma.log.size(), ref.singles.size()); i++) {
    auto &a = ma.log[i]; auto &b = ref.singles[i];
    uint32_t m = 0;
    for (int l = 0; l < 4; l++) if (b.be & (8 >> l)) m |= 0xFFu << (24 - 8 * l);
    if (a.we != b.we || a.addr != b.addr || a.be != b.be || (a.we && ((a.data & m) != (b.data & m)))) {
      std::printf("SINGLE %zu dut %c %08X be %X %08X ref %c %08X be %X %08X\n", i, a.we ? 'W' : 'R', a.addr, a.be, a.data,
                  b.we ? 'W' : 'R', b.addr, b.be, b.data);
      errors++; break;
    }
  }
  std::set<uint32_t> keys;
  for (auto &kv : dmem.w) keys.insert(kv.first);
  for (auto &kv : ref.mem.w) keys.insert(kv.first);
  uint64_t memdiff = 0;
  for (uint32_t k : keys) if (dmem.rd(k << 2) != ref.mem.rd(k << 2)) {
    if (memdiff < 5) std::printf("MEMORY %08X dut %08X ref %08X\n", k << 2, dmem.rd(k << 2), ref.mem.rd(k << 2));
    memdiff++;
  }
  errors += memdiff ? 1 : 0;

  std::printf("seed %u: %lu ops, %lu cycles; loads %lu (checked %lu) stores %lu dcbt %lu dcbz %lu dcbst %lu dcbf %lu dcbi %lu\n",
              seed, (unsigned long)issued, (unsigned long)cyc, (unsigned long)n_ld, (unsigned long)n_ldchk, (unsigned long)n_st,
              (unsigned long)n_ops[2], (unsigned long)n_ops[3], (unsigned long)n_ops[4], (unsigned long)n_ops[5], (unsigned long)n_ops[6]);
  std::printf("  ref hits %lu misses %lu write-backs %lu bypassed %lu; allocations %zu; singles %zu; memory words %zu\n",
              (unsigned long)ref.hits, (unsigned long)ref.misses, (unsigned long)ref.wbs, (unsigned long)ref.byp,
              ref.allocs.size(), ref.singles.size(), keys.size());
  std::printf("  port: fills %lu cast-outs %lu single reads %lu single writes %lu; HID0 DCE %lu DLOCK %lu NOOPTI %lu DCFI %lu\n",
              (unsigned long)ma.n_fill, (unsigned long)ma.n_wb, (unsigned long)ma.n_rd, (unsigned long)ma.n_wr,
              (unsigned long)n_dce, (unsigned long)n_dlock, (unsigned long)n_noopti, (unsigned long)n_dcfi);
  const char *cn[4] = {"load hit", "load miss", "store hit", "store miss"};
  for (int c = 0; c < 4; c++) if (lat_n[c])
    std::printf("  %-10s request to d_valid: min %lu mean %.2f max %lu cycles (%lu)\n", cn[c], (unsigned long)lat_min[c],
                (double)lat_sum[c] / lat_n[c], (unsigned long)lat_max[c], (unsigned long)lat_n[c]);
  std::printf("  hit streams, cycles per access: 64 loads %.2f, 64 stores %.2f, 32 store+load pairs %.2f per pair\n",
              (s_mark[1] - s_mark[0]) / 64.0, (s_mark[2] - s_mark[1]) / 64.0, (s_t0 - s_mark[2]) / 32.0);
  if (snoop_mode) {
    const char *kn[3] = {"miss", "clean", "modified"};
    for (int i = 0; i < 2; i++) {
      std::printf("  snoops (%s) %lu:", i ? "write" : "read", (unsigned long)sn_n[i]);
      for (int k = 0; k < 3; k++) if (sn_kind[i][k])
        std::printf(" %s %lu (mean %.1f, max %lu cycles)", kn[k], (unsigned long)sn_kind[i][k],
                    (double)sn_lat_sum[i][k] / sn_kind[i][k], (unsigned long)sn_lat_max[i][k]);
      std::printf("\n");
    }
    std::printf("  snoop races: CPU requests outstanding %lu, memory port busy %lu, line changed by an unanswered request %lu,"
                " changed during the snoop %lu, back to back %lu; DMA writes %lu; bytes checked %lu\n",
                (unsigned long)sn_race_out, (unsigned long)sn_race_mem, (unsigned long)sn_race_st, (unsigned long)sn_race_win,
                (unsigned long)sn_b2b, (unsigned long)sn_dmaw, (unsigned long)sn_bytes);
    std::printf("  snoop races: a store to the line in the 3 cycles before %lu, its allocation (fill) in the 24 before %lu,"
                " its cast-out in the 24 before %lu, a CPU miss to another line in the 24 before %lu\n",
                (unsigned long)sn_race_sb, (unsigned long)sn_race_fill, (unsigned long)sn_race_evict, (unsigned long)sn_race_miss);
    std::printf("  write snoops started with DCFI %lu, lwarx to a write snoop's line in its window %lu\n", (unsigned long)sn_dcfi, (unsigned long)sn_rsv_in);
    std::printf("  lwarx %lu, reservations cleared %lu, cycles cleared %lu\n", (unsigned long)n_rsv, (unsigned long)n_kill_set,
                (unsigned long)n_kill_cyc);
  }
  std::printf("%s: %d error(s)\n", errors ? "FAIL" : "PASS", errors);
  delete t;
  return errors ? 1 : 0;
}
