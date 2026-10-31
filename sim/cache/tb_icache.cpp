// SPDX-License-Identifier: GPL-3.0-or-later
//
// ppc_icache bench. Fetch streams in the core's protocol (one request
// outstanding, the next issued in the cycle the answer is seen, i_seq on
// sequential requests) against an exact reference of the cache's contents
// and PLRU. Checked: every fetched word; every line allocation (set, way,
// tag) in order; every single read (cache-inhibited page, ICE = 0, a miss
// under ILOCK, the bypass page) in order. Code is changed in memory between
// fetches, so a stale line the reference still holds must be returned, and
// one it no longer holds must not. icbi, ICFI, ICE and ILOCK change while
// fetching. Some fetches from cacheable pages are made with I = 1 while their
// line is resident: the cache must read memory, which differs by then.
//
// usage: Vppc_icache [fetches] [seed] [lat_lo lat_hi] [no_seq]

#include "Vppc_icache.h"
#include "verilated.h"
#include "cache_mem.h"
#include <algorithm>
#include <cstdlib>
#include <map>

static const uint32_t RSV = 0xF00FF;

struct RefLine { bool v = false; uint32_t tag = 0; uint32_t w[8]; };
struct Alloc { unsigned set, way; uint32_t tag; };

struct Ref {
  RefLine c[128][4];
  uint8_t plru[128] = {};
  Memory mem;
  std::vector<Single> singles;
  std::vector<Alloc> allocs;
  bool ice = true, ilock = false;
  uint64_t hits = 0, misses = 0, byp = 0;
  static unsigned victim_plru(uint8_t p) { return (p & 1) ? 2 + ((p >> 2) & 1) : ((p >> 1) & 1); }
  void touch(unsigned s, unsigned w) {
    uint8_t p = plru[s];
    p = (p & ~1) | ((w >> 1) ? 0 : 1);
    if (!(w >> 1)) p = (p & ~2) | ((w & 1) ? 0 : 2);
    else           p = (p & ~4) | ((w & 1) ? 0 : 4);
    plru[s] = p;
  }
  bool bypass(uint32_t a, unsigned wimg) const { return (wimg & 4) || !ice || (a >> 12) == RSV; }
  bool resident(uint32_t a) const {
    unsigned s = (a >> 5) & 127;
    for (int w = 0; w < 4; w++) if (c[s][w].v && c[s][w].tag == (a >> 12)) return true;
    return false;
  }
  uint32_t fetch(uint32_t a, unsigned wimg, bool &hit) {
    hit = false;
    if (bypass(a, wimg)) { byp++; singles.push_back({false, a, 15, 0}); return mem.rd(a); }
    unsigned s = (a >> 5) & 127;
    for (int w = 0; w < 4; w++) if (c[s][w].v && c[s][w].tag == (a >> 12)) {
      hits++; hit = true; touch(s, w); return c[s][w].w[(a >> 2) & 7];
    }
    misses++;
    if (ilock) { singles.push_back({false, a, 15, 0}); return mem.rd(a); }
    int v = -1;
    for (int w = 0; w < 4 && v < 0; w++) if (!c[s][w].v) v = w;
    if (v < 0) v = victim_plru(plru[s]);
    RefLine &L = c[s][v];
    L.v = true; L.tag = a >> 12;
    for (int k = 0; k < 8; k++) L.w[k] = mem.rd((a & ~31u) | (k << 2));
    touch(s, v);
    allocs.push_back({s, (unsigned)v, a >> 12});
    return L.w[(a >> 2) & 7];
  }
  void icbi(unsigned s) { for (auto &l : c[s]) l.v = false; }
  void flash() { for (auto &s : c) for (auto &l : s) l.v = false; for (auto &p : plru) p = 0; }
};

static std::mt19937 rng;
static int rnd(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }

static const uint32_t PG_C[] = { 0x00000, 0x00001, 0x00002, 0x00003, 0x00004, 0x00005, 0x00006, 0x00007,
                                 0xFFF00, 0xFFF01, 0xFFFFF, 0x7FFFF };
static const uint32_t PG_I[] = { 0xF0040 };
static unsigned page_wimg(uint32_t pg) { return pg == 0xF0040 ? 4 : 0; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long nf = argc > 1 ? atol(argv[1]) : 200000;
  unsigned seed = argc > 2 ? atoi(argv[2]) : 1;
  int lat_lo = argc > 3 ? atoi(argv[3]) : 12, lat_hi = argc > 4 ? atoi(argv[4]) : 15;
  bool no_seq = argc > 5 && atoi(argv[5]);
  rng.seed(seed);

  Vppc_icache *t = new Vppc_icache;
  Ref ref;
  Memory imem;
  MemAgent ma; ma.rng = &rng; ma.mem = &imem; ma.lat_lo = lat_lo; ma.lat_hi = lat_hi;
  if (getenv("CACHE_FIXED")) { ma.ack_max = 0; ma.lat_wide = 0; ma.gap_max = 0; }   // deterministic memory: ack at once, beats every cycle
  uint64_t cyc = 0; int errors = 0;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++; };

  t->rst = 1; t->i_req = 0; t->hid_ice = 1; t->hid_ilock = 0; t->hid_icfi = 0; t->icbi_req = 0;
  t->m_ack = 0; t->m_rvalid = 0;
  for (int i = 0; i < 4; i++) tick();
  t->rst = 0;

  // directed timing streams: 64 sequential words fetched twice (miss, then hit),
  // a loop inside one line, a jump between two lines of different sets
  struct F { uint32_t a; bool seq; };
  std::vector<F> script;
  for (int r = 0; r < 2; r++) for (int i = 0; i < 64; i++) script.push_back({0x00001000u + 4u * i, i != 0});
  for (int r = 0; r < 16; r++) for (int i = 0; i < 4; i++) script.push_back({0x00001000u + 4u * i, i != 0});
  for (int r = 0; r < 16; r++) for (int i = 0; i < 4; i++) script.push_back({(r & 1 ? 0x00001040u : 0x00001020u) + 4u * i, i != 0});
  size_t si = 0;
  uint64_t mark[5] = {};

  bool out = false; uint32_t cur_a = 0, cur_exp = 0; bool cur_hit = false; uint64_t t0 = 0;
  uint32_t pc = 0x00000000; bool have_pc = false;
  std::vector<Alloc> dut_allocs;
  uint64_t n_f = 0, n_seq = 0, n_jump = 0, n_icbi = 0, n_icfi = 0, n_ice = 0, n_ilock = 0, n_mod = 0, n_para = 0;
  uint64_t lat_n[3] = {}, lat_s[3] = {}, lat_mx[3] = {}, lat_mn[3] = {999, 999, 999};
  long issued = 0; int gap = 0;
  enum { RUN, EVT, ICFI_WAIT, ICBI_WAIT, DONE } ph = RUN;
  int ev_kind = 0, hold = 0;
  long max_cyc = nf * 120 + 200000;

  while (ph != DONE && (long)cyc < max_cyc) {
    bool iv = t->i_valid; uint32_t idata = t->i_data;
    if (t->t_alloc) dut_allocs.push_back({t->t_alloc_set, t->t_alloc_way, t->t_alloc_tag});
    t->i_req = 0;
    if (out && iv) {
      out = false;
      if (idata != cur_exp) {
        if (errors < 10) std::printf("FETCH MISMATCH cyc %lu addr %08X got %08X exp %08X\n", (unsigned long)cyc, cur_a, idata, cur_exp);
        errors++;
      }
      unsigned lat = cyc - t0; int c = cur_hit ? 0 : 1;
      lat_n[c]++; lat_s[c] += lat; lat_mx[c] = std::max<uint64_t>(lat_mx[c], lat); lat_mn[c] = std::min<uint64_t>(lat_mn[c], lat);
      gap = si < script.size() ? 0 : (rnd(0, 7) == 0 ? rnd(1, 3) : 0);
    } else if (iv) { std::printf("spurious i_valid at %lu\n", (unsigned long)cyc); errors++; }

    if (t->icbi_ack && ph == ICBI_WAIT) { t->icbi_req = 0; ph = RUN; }

    if (!out) {
      if (ph == RUN && si >= script.size() && issued >= nf) ph = DONE;
      if (ph == RUN && si >= script.size() && rnd(0, 999) < 8) { ph = EVT; ev_kind = rnd(0, 9); }
      if (ph == EVT) {
        if (ev_kind < 4) {                                   // icbi of a set: no fetch outstanding
          unsigned s = rnd(0, 3) ? (pc >> 5) & 127 : rnd(0, 127);
          t->icbi_req = 1; t->icbi_set = s; ref.icbi(s); n_icbi++; ph = ICBI_WAIT;
        } else if (t->i_idle && ma.idle()) {
          if (ev_kind < 7) {                                 // change code under the cache
            for (int k = rnd(1, 8); k > 0; k--) {
              uint32_t pg = rnd(0, 3) ? PG_C[rnd(0, 2)] : PG_C[rnd(0, 11)];
              uint32_t a = (pg << 12) | (rnd(0, 127) << 5) | (rnd(0, 7) << 2);
              if (rnd(0, 1)) a = (pc & ~31u) | (rnd(0, 7) << 2);
              uint32_t d = rng();
              imem.wr(a, d, 15); ref.mem.wr(a, d, 15);
            }
            n_mod++; ph = RUN;
          } else if (ev_kind == 7) { ref.ice = !ref.ice; t->hid_ice = ref.ice; n_ice++; ph = RUN;
            if (!ref.ice && rnd(0, 1)) { ref.ice = true; t->hid_ice = 1; } }
          else if (ev_kind == 8) { ref.ilock = !ref.ilock; t->hid_ilock = ref.ilock; n_ilock++; ph = RUN; }
          else if (ref.ice) { t->hid_icfi = 1; hold = 0; ref.flash(); n_icfi++; ph = ICFI_WAIT; }
          else ph = RUN;
          if (ph == RUN && ref.ilock && rnd(0, 2) == 0) { ref.ilock = false; t->hid_ilock = 0; }
        }
      } else if (ph == ICFI_WAIT) {
        if (t->icfi_ack) hold = 1;
        if (hold && ++hold > 2) { t->hid_icfi = 0; ph = RUN; }
      } else if (ph == RUN && gap > 0) {
        gap--;
      } else if (ph == RUN) {
        uint32_t a; bool seq;
        if (si < script.size()) {
          if (si == 64) mark[0] = cyc;
          if (si == 128) mark[1] = cyc;
          if (si == 192) mark[2] = cyc;
          a = script[si].a; seq = script[si].seq && !no_seq; si++;
          if (si == script.size()) mark[3] = 1;
        } else {
          if (mark[3] == 1) mark[3] = cyc;
          int r = rnd(0, 99);
          if (have_pc && r < 82) { a = pc + 4; seq = !no_seq && (((a & 0xFFF) != 0) || rnd(0, 1)); if (rnd(0, 19) == 0) seq = false; n_seq++; }
          else {
            n_jump++;
            int j = rnd(0, 99);
            if (j < 30 && have_pc) a = (pc & ~31u) | (rnd(0, 7) << 2);                     // same line
            else if (j < 75) a = (PG_C[rnd(0, 1)] << 12) | (rnd(0, 15) << 5) | (rnd(0, 7) << 2);   // hot
            else if (j < 88) a = (PG_C[rnd(0, 11)] << 12) | (rnd(0, 3) << 5) | (rnd(0, 7) << 2);   // aliasing sets
            else if (j < 95) a = (PG_C[rnd(0, 11)] << 12) | (rnd(126, 127) << 5) | (rnd(0, 7) << 2);  // page end
            else if (j < 97) a = (PG_I[0] << 12) | (rnd(0, 127) << 5) | (rnd(0, 7) << 2);
            else a = (RSV << 12) | (rnd(0, 127) << 5) | (rnd(0, 7) << 2);
            seq = false;
          }
          issued++;
        }
        unsigned wimg = page_wimg(a >> 12);
        // (only on a non-sequential fetch: i_seq means the same translation, so the same WIMG)
        if (si >= script.size() && !seq && wimg == 0 && rnd(0, 99) < 8 && ref.resident(a)) { wimg = 4; n_para++; }
        pc = a; have_pc = true;
        cur_a = a; cur_exp = ref.fetch(a, wimg, cur_hit); t0 = cyc; n_f++;
        t->i_req = 1; t->i_addr = a; t->i_seq = seq; t->i_wimg = wimg;
        out = true;
      }
    }

    ma.step(t->m_req, t->m_cmd, t->m_addr, t->m_be, 0);
    t->m_ack = ma.ack; t->m_rvalid = ma.rvalid; t->m_rdata = ma.rdata;
    tick();
    if (ma.err) { errors += ma.err; ma.err = 0; }
    if (Verilated::gotFinish()) { std::printf("$finish / assertion\n"); errors++; break; }
  }
  if (ph != DONE) { std::printf("TIMEOUT at cycle %lu\n", (unsigned long)cyc); errors++; }

  if (dut_allocs.size() != ref.allocs.size()) { std::printf("ALLOC COUNT dut %zu ref %zu\n", dut_allocs.size(), ref.allocs.size()); errors++; }
  for (size_t i = 0; i < std::min(dut_allocs.size(), ref.allocs.size()); i++) {
    auto &a = dut_allocs[i]; auto &b = ref.allocs[i];
    if (a.set != b.set || a.way != b.way || a.tag != b.tag) {
      std::printf("ALLOC %zu dut set %u way %u tag %05X ref set %u way %u tag %05X\n", i, a.set, a.way, a.tag, b.set, b.way, b.tag);
      errors++; break;
    }
  }
  if (ma.log.size() != ref.singles.size()) { std::printf("SINGLE COUNT dut %zu ref %zu\n", ma.log.size(), ref.singles.size()); errors++; }
  for (size_t i = 0; i < std::min(ma.log.size(), ref.singles.size()); i++)
    if (ma.log[i].addr != ref.singles[i].addr || ma.log[i].we || ma.log[i].be != 15) {
      std::printf("SINGLE %zu dut %08X ref %08X\n", i, ma.log[i].addr, ref.singles[i].addr); errors++; break;
    }

  std::printf("seed %u: %lu fetches (%lu sequential, %lu jumps), %lu cycles; icbi %lu ICFI %lu ICE %lu ILOCK %lu code changes %lu inhibited-resident %lu\n",
              seed, (unsigned long)n_f, (unsigned long)n_seq, (unsigned long)n_jump, (unsigned long)cyc, (unsigned long)n_icbi,
              (unsigned long)n_icfi, (unsigned long)n_ice, (unsigned long)n_ilock, (unsigned long)n_mod, (unsigned long)n_para);
  std::printf("  ref hits %lu misses %lu bypassed %lu; allocations %zu; singles %zu; port fills %lu single reads %lu\n",
              (unsigned long)ref.hits, (unsigned long)ref.misses, (unsigned long)ref.byp, ref.allocs.size(), ref.singles.size(),
              (unsigned long)ma.n_fill, (unsigned long)ma.n_rd);
  for (int c = 0; c < 2; c++) if (lat_n[c])
    std::printf("  %s request to i_valid: min %lu mean %.2f max %lu cycles (%lu)\n", c ? "miss/bypass" : "hit        ",
                (unsigned long)lat_mn[c], (double)lat_s[c] / lat_n[c], (unsigned long)lat_mx[c], (unsigned long)lat_n[c]);
  std::printf("  cycles per fetch: 64 sequential misses %.2f, the same 64 hitting %.2f, 4-word loop in a line %.2f, "
              "alternating two lines %.2f\n", (mark[0] - 0) / 64.0, (mark[1] - mark[0]) / 64.0, (mark[2] - mark[1]) / 64.0,
              (mark[3] - mark[2]) / 64.0);
  std::printf("%s: %d error(s)\n", errors ? "FAIL" : "PASS", errors);
  delete t;
  return errors ? 1 : 0;
}
