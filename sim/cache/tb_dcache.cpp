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
// The address space is a few sets and many pages, so lines are evicted and
// written back all the time. Some accesses to cacheable pages are made with
// I = 1 or W = 1 while their line is resident, so an access that wrongly
// looks at the arrays reads or writes a different value than the reference.
//
// usage: Vppc_dcache [ops] [seed] [lat_lo lat_hi] [stream]

#include "Vppc_dcache.h"
#include "verilated.h"
#include "cache_mem.h"
#include <algorithm>
#include <deque>
#include <cstdlib>
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
  void flash() { for (auto &s : c) for (auto &l : s) { l.v = false; l.d = false; } for (auto &p : plru) p = 0; }
};

// ------------------------------------------------------------- the bench
static std::mt19937 rng;
static int rnd(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }

enum { OP_LD, OP_ST, OP_TCH, OP_DCBZ, OP_DCBST, OP_DCBF, OP_DCBI };
struct Op { int k; uint32_t a; unsigned be; uint32_t d; unsigned wimg; uint32_t exp; uint64_t t0; bool ref_hit; };

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
  rng.seed(seed);

  Vppc_dcache *t = new Vppc_dcache;
  Ref ref;
  gref = &ref;
  Memory dmem;
  MemAgent ma; ma.rng = &rng; ma.mem = &dmem; ma.lat_lo = lat_lo; ma.lat_hi = lat_hi;
  if (getenv("CACHE_FIXED")) { ma.ack_max = 0; ma.lat_wide = 0; ma.gap_max = 0; }   // deterministic memory: ack at once, beats every cycle

  uint64_t cyc = 0;
  int errors = 0;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++; };

  t->rst = 1; t->d_req = 0; t->hid_dce = 1; t->hid_dlock = 0; t->hid_dcfi = 0; t->hid_noopti = 0;
  t->m_ack = 0; t->m_wnext = 0; t->m_rvalid = 0; t->m_rdata = 0;
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
  uint64_t s_t0 = 0, s_mark[4] = {};

  long max_cyc = nops * 200 + 200000;
  while (phase != DONE && (long)cyc < max_cyc) {
    // ---------------------------------------------------- outputs of the last edge
    bool dv = t->d_valid; uint32_t drd = t->d_rdata;
    if (t->t_alloc) dut_allocs.push_back({t->t_alloc_set, t->t_alloc_way, t->t_alloc_tag});

    t->d_req = 0;
    if (dv && !outq.empty()) {
      cur = outq.front(); outq.pop_front();
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
      if (phase == HID_WAIT) {
        if (t->d_idle && ma.idle() && quiet_cpu) {
          if (hid_kind < 3) { ref.dce = !ref.dce; t->hid_dce = ref.dce; n_dce++; phase = RUN; }
          else if (hid_kind < 6) { ref.dlock = !ref.dlock; t->hid_dlock = ref.dlock; n_dlock++; phase = RUN; }
          else if (hid_kind < 7) { ref.noopti = !ref.noopti; t->hid_noopti = ref.noopti; n_noopti++; phase = RUN; }
          else if (ref.dce) { t->hid_dcfi = 1; dcfi_hold = 0; phase = DCFI_WAIT; ref.flash(); n_dcfi++; }
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
          outq.push_back(cur);
        } else if (t->d_idle && ma.idle() && quiet_cpu) phase = DONE;
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
          o.k = r < 45 ? OP_LD : r < 85 ? OP_ST : r < 88 ? OP_TCH : r < 91 ? OP_DCBZ : r < 94 ? OP_DCBST :
                r < 97 ? OP_DCBF : OP_DCBI;
          if (o.k == OP_LD || o.k == OP_ST) pick_size(o.a, o.be); else o.be = 15;
          o.d = rng();
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
  std::printf("%s: %d error(s)\n", errors ? "FAIL" : "PASS", errors);
  delete t;
  return errors ? 1 : 0;
}
