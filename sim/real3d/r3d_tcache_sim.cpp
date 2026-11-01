// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Texture cache measurement for the rasteriser at 100 MHz (D-28): the texel
// addresses of every textured Z-passing pixel in the RTL's order (8-row bands,
// passes, draw order), four corners a quad, banks by (A[11], A[0]); lines of
// 4 x 4 texels. A pixel is one quad at level 0 or the nearest mip level, and
// one or two quads (two lookups, two cycles at four texels a clock) when
// trilinear. For each cache shape: line fills, and cycles of an in-order
// non-blocking cache (lookup ahead by D quads, F fills in flight) against one
// quad a clock, on one of these memories:
//
//   ideal   a 64-bit port, a line every 4 cycles, latency 24 (R66's figure)
//   sdram   a 16-bit SDRAM behind Model 2's m2_sdram: one transfer at a time,
//           3 cycles of arbitration and dispatch, a CAS a word (16 a line),
//           rows left open, PRE tRP 2, ACT tRCD 2, tRAS 5, tRC 7, CL 2 plus
//           4 cycles of capture, refresh every 700 cycles (tRP + tRC). Map 0 is
//           the line address straight into {row, column} (8 MiB = one bank);
//           map 1 puts 8 x 4 lines in a row and spreads neighbours over banks.
//           Texture-only: nothing else on the chip.
//   ddr3    MiSTer's 64-bit f2sdram port at 100 MHz: a beat a cycle, reads
//           pipelined, data in order, a fixed read latency, shared with the 3D
//           layer's writes, its scan-out and polygon RAM at R64's measured
//           rates and burst sizes (43.8 MB/s in 248-beat writes, 43.8 MB/s in
//           256-beat reads, 28.4 MB/s in 32-beat reads). Latency 30 or 60, and
//           13 (m2_ddr3's board figure, R351 of Model 2), pipelined or one
//           request at a time as m2_ddr3 is built.
//
//   r3d_tcache_sim --tris DIR --dumps DIR [--frames N] [--mip 0|1|2] [--lod 0|1] [--all-shapes] [--geo-lod VROM]
//
// --geo-lod: with lod 1, each polygon's B is the geometry's LOD word (the model
// r3d_geo with lod_word, MAME's frustum, every node walked so its polygon
// numbers are the list's) from the record the geometry sends: the inverted
// copy's -B for a double-sided polygon facing away.

#include "r3d_rast.h"
#include "r3d_geo_lod.h"
#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <zlib.h>

using namespace r3d;
namespace fs = std::filesystem;

struct quad { uint32_t a[4]; };

static int ev(char const *n, int d) { char const *e = std::getenv(n); return e ? std::atoi(e) : d; }
static uint32_t line_of(uint32_t a) { return ((a >> 13) << 9) | ((a >> 2) & 511); }   // {y tile, x tile}

struct hook : d25_hook {
  rframe const *f; std::vector<quad> bucket[4][48];
  uint64_t npx = 0, quads = 0, minified = 0, lvn[12] = {};
  void px_(rtri const &t, uint32_t u, uint32_t v, int L, int y) {
    quad q; texel_addr4(t, u, v, L, q.a);
    bucket[t.pass][y / 8].push_back(q);
    quads++;
  }
  void px(d25_px const &p) override {
    if (!p.zpass || p.k < K_TEX) return;
    rtri const &t = f->tris[size_t(p.tri)];
    npx++;
    lvn[std::min(p.lv0, 11)]++;
    minified += p.lv0 > 0 || p.lfr > 0;
    px_(t, p.u, p.v, p.lv0, p.y);
    if (p.lv1 >= 0) px_(t, p.u, p.v, p.lv1, p.y);
  }
};

struct shape { char const *name; int sets, ways; };
static shape const SH_ALL[] = {{"16K 4w", 128, 4}, {"32K dm", 1024, 1}, {"32K 2w", 512, 2}, {"32K 4w", 256, 4}, {"64K 4w", 512, 4}, {"128K 4w", 1024, 4}};
static shape const SH_MAIN[] = {{"32K 4w", 256, 4}, {"64K 4w", 512, 4}};

static uint32_t setof(uint32_t ln, int sets) {
  uint32_t const xt = ln & 511, yt = ln >> 9;
  int const b = __builtin_ctz(uint32_t(sets));
  uint32_t const xb = uint32_t(b + 1) / 2, yb = uint32_t(b) - xb;   // interleave x and y tile bits, as the RTL
  return ((yt & ((1u << yb) - 1)) << xb) | (xt & ((1u << xb) - 1));
}

// ---- memories: serve(t, line) for fills in issue order, t non-decreasing;
// returns the cycle the line is in the cache (data in, four write cycles done)
struct mem {
  virtual ~mem() = default;
  virtual uint64_t serve(uint64_t t, uint32_t line) = 0;
  uint64_t busy = 0;   // cycles the texture traffic held the memory
};

struct mem_ideal : mem {
  int L, B; uint64_t fr = 0;
  mem_ideal(int l, int b) : L(l), B(b) {}
  uint64_t serve(uint64_t t, uint32_t) override {
    uint64_t const s = std::max(t, fr);
    fr = s + uint64_t(B); busy += uint64_t(B);
    return s + uint64_t(L) + uint64_t(B);
  }
};

struct mem_sdram : mem {
  static constexpr int T_RCD = 2, T_RP = 2, T_RC = 7, T_RAS = 5, CAP = 6, ARB = 3, REFI = 700, WORDS = 16;
  int map;
  uint64_t fr = 0, next_ref = REFI;
  bool open[4] = {}; uint32_t row[4] = {}; uint64_t act[4] = {};
  uint64_t hits = 0, closed = 0, conflicts = 0;
  explicit mem_sdram(int m) : map(m) {}
  uint64_t serve(uint64_t t, uint32_t ln) override {
    uint32_t const xt = ln & 511, yt = ln >> 9;
    uint32_t bank, rw;
    if (map == 0) { bank = 0; rw = ln >> 5; }                                    // word = line x 16: col = line[4:0] x 16
    else { bank = ((yt >> 2) & 1) << 1 | ((xt >> 3) & 1); rw = ((yt >> 3) << 5) | (xt >> 4); }
    uint64_t s = std::max(t, fr);
    while (next_ref <= s) {                                                      // precharge all, refresh
      s = std::max(s, next_ref) + T_RP + T_RC;
      for (bool &o : open) o = false;
      next_ref += REFI;
    }
    uint64_t const s0 = s;
    s += ARB;
    uint64_t cas;
    if (open[bank] && row[bank] == rw) { cas = s; hits++; }
    else {
      uint64_t a;
      if (!open[bank]) { a = std::max(s + 1, act[bank] + T_RC); closed++; }
      else { uint64_t const p = std::max(s + 1, act[bank] + T_RAS); a = std::max(p + T_RP, act[bank] + T_RC); conflicts++; }
      act[bank] = a; open[bank] = true; row[bank] = rw;
      cas = a + T_RCD;
    }
    fr = cas + WORDS;
    busy += fr - s0;
    return cas + WORDS - 1 + CAP + 1 + 4;
  }
};

struct mem_ddr3 : mem {
  int L; bool serial;                     // serial: one request at a time, as m2_ddr3 is built
  uint64_t last = 0;
  struct client { uint64_t period, beats, next; bool write; };
  std::vector<client> cl;
  uint64_t cmd = 0, rd = 0;
  mem_ddr3(int l, double scale, bool ser = false) : L(l), serial(ser) {
    // R64, 57.52 Hz, 8 B a beat at 100 MHz: period = burst bytes / rate
    auto per = [&](double mbs, double beats) { return uint64_t(beats * 8.0 / (mbs * scale * 1e6) * 1e8); };
    cl.push_back({per(43.8, 248), 248, 1000, true});    // 3D layer, row writes
    cl.push_back({per(43.8, 256), 256, 2300, false});   // scan-out, a line a burst
    cl.push_back({per(28.4, 32), 32, 400, false});      // polygon RAM
  }
  void issue(uint64_t t, uint64_t beats, bool write) {
    uint64_t const i = std::max(t, cmd);
    if (write) cmd = i + beats;
    else { cmd = i + 1; rd = std::max(i + uint64_t(L), rd) + beats; }
  }
  uint64_t serve(uint64_t t, uint32_t) override {
    for (;;) {   // background requests up to t, in time order
      client *n = nullptr;
      for (auto &c : cl) if (c.next <= t && (!n || c.next < n->next)) n = &c;
      if (!n) break;
      issue(n->next, n->beats, n->write);
      n->next += n->period;
    }
    uint64_t const i = std::max({t, cmd, serial ? last + 1 : 0});
    cmd = i + 1;
    uint64_t const d = std::max(i + uint64_t(L), rd);
    rd = d + 4;
    busy += 4;
    last = d + 4;
    return d + 4 + 4;
  }
};

struct memspec { char const *name; int kind, a; double b; };
static memspec const MEMS[] = {
  {"ideal64", 0, 24, 4}, {"sdram m0", 1, 0, 0}, {"sdram m1", 1, 1, 0},
  {"ddr3 L30", 2, 30, 1.0}, {"ddr3 L60", 2, 60, 1.0}, {"ddr3 L30 D-20", 2, 30, 180.0 / 116.0},
  {"ddr3 L13", 2, 13, 1.0}, {"ddr3 L13 serial", 3, 13, 1.0},
};
constexpr int NMEM = 8;
static std::unique_ptr<mem> make_mem(memspec const &m) {
  if (m.kind == 0) return std::make_unique<mem_ideal>(m.a, int(m.b));
  if (m.kind == 1) return std::make_unique<mem_sdram>(m.a);
  return std::make_unique<mem_ddr3>(m.a, m.b, m.kind == 3);
}

constexpr int MAXSH = 6;
struct res { bool ok = false; uint64_t px = 0, quads = 0, minified = 0, lvn[12] = {}, miss[MAXSH] = {}, cyc[MAXSH][NMEM] = {}; };

// Line fills of one cache shape (the RTL's victim: a global round-robin way,
// never one this pixel's lines already hold in the set; TC_REPL=1 true LRU),
// and the cycles on each memory.
static void sim(std::vector<quad> const &st, shape const *SH, int nsh, res &r) {
  int const D = ev("TC_D", 64), F = ev("TC_F", 8);
  bool const lru = ev("TC_REPL", 0) == 1;
  for (int s = 0; s < nsh; s++) {
    int const sets = SH[s].sets, ways = SH[s].ways;
    for (int mi = 0; mi < NMEM; mi++) {
      auto m = make_mem(MEMS[mi]);
      std::vector<uint32_t> tag(size_t(sets * ways), 0xffffffffu);
      std::vector<uint64_t> age(size_t(sets * ways), 0), lready(size_t(sets * ways), 0);
      uint64_t clk = 0, rr = 0, miss = 0, t_out = 0, t_look = 0;
      std::deque<uint64_t> fills, look;
      for (size_t i = 0; i < st.size(); i++) {
        uint64_t tl = t_look + 1;
        if (i >= size_t(D)) tl = std::max(tl, look.front());
        uint64_t need = 0;
        uint32_t done[4]; int nd = 0;
        size_t used[4]; int nu = 0;
        for (int k = 0; k < 4; k++) {
          uint32_t const ln = line_of(st[i].a[k]);
          bool dup = false;
          for (int j = 0; j < nd; j++) dup |= done[j] == ln;
          if (dup) continue;
          done[nd++] = ln;
          size_t const base = size_t(setof(ln, sets)) * size_t(ways);
          int hit = -1;
          for (int w = 0; w < ways; w++) if (tag[base + size_t(w)] == ln) hit = w;
          size_t e;
          if (hit >= 0) e = base + size_t(hit);
          else {
            int vic = 0;
            if (lru) { for (int w = 1; w < ways; w++) if (age[base + size_t(w)] < age[base + size_t(vic)]) vic = w; }
            else {
              vic = int(rr % uint64_t(ways));
              for (int kk = ways - 1; kk >= 0; kk--) {
                int const w = int((rr + uint64_t(kk)) % uint64_t(ways));
                bool busy = false;
                for (int j = 0; j < nu; j++) busy |= used[j] == base + size_t(w);
                if (!busy) vic = w;
              }
              rr = uint64_t(vic) + 1;
            }
            e = base + size_t(vic);
            tag[e] = ln; miss++;
            while (!fills.empty() && fills.front() <= tl) fills.pop_front();
            while (fills.size() >= size_t(F)) { tl = std::max(tl, fills.front()); fills.pop_front(); }
            uint64_t const c = m->serve(tl, ln);
            fills.push_back(c);
            lready[e] = c;
          }
          used[nu++] = e;
          age[e] = ++clk;
          need = std::max(need, lready[e]);
        }
        t_look = tl;
        t_out = std::max({t_out + 1, need, tl + 2});
        look.push_back(t_out);
        if (look.size() > size_t(D)) look.pop_front();
      }
      r.cyc[s][mi] = t_out;
      r.miss[s] = miss;
    }
  }
}

int main(int argc, char **argv) {
  std::string tris, dumps, geo_vrom;
  int frames = 1 << 30, threads = ev("TC_THREADS", 12);
  bool all = false;
  d25_opts d; d.band = 8;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx(); else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--mip") d.mip = std::atoi(nx().c_str()); else if (a == "--lod") d.lod = std::atoi(nx().c_str());
    else if (a == "--all-shapes") all = true;
    else if (a == "--geo-lod") geo_vrom = nx();
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  shape const *SH = all ? SH_ALL : SH_MAIN;
  int const nsh = all ? 6 : 2;
  std::vector<std::string> names;
  for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
  std::sort(names.begin(), names.end());
  if (int(names.size()) > frames) names.resize(size_t(frames));
  std::vector<res> rr(names.size());
  std::atomic<size_t> nxt{0};
  std::vector<uint32_t> vrom;
  if (!geo_vrom.empty()) {
    vrom.resize(0x1000000);
    std::FILE *fv = std::fopen(geo_vrom.c_str(), "rb");
    if (!fv) { std::fprintf(stderr, "no VROM %s\n", geo_vrom.c_str()); return 2; }
    size_t n = std::fread(vrom.data(), 4, vrom.size(), fv); (void)n; std::fclose(fv);
  }
  std::atomic<uint64_t> geo_given{0}, geo_tris{0};
  auto wk = [&]() {
    texram tex; std::vector<uint32_t> dl(0x40000);
    std::unique_ptr<r3d_mem> gm;
    if (!vrom.empty()) { gm = std::make_unique<r3d_mem>(); gm->vrom = vrom; }
    for (size_t i; (i = nxt++) < names.size();) {
      rframe f;
      if (!read_rframe(tris + "/" + names[i] + ".tri", f)) continue;
      gzFile gz = gzopen((dumps + "/" + names[i] + ".bin.gz").c_str(), "rb");
      bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
                gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
      if (gz) gzclose(gz);
      if (!ok) continue;
      for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      rtl_opts o; o.oow = true;
      auto h = std::make_unique<hook>(); h->f = &f;
      d25_opts dd = d; dd.hook = h.get();
      std::vector<uint32_t> lodb;
      if (gm && geo_lod_load(dumps + "/" + names[i] + ".bin.gz", *gm)) {
        static bool const own = ev("TC_GEO_OWN", 0) != 0;   // measurement: the polygon's own B even when its copy is sent
        lodb = geo_lod_words(*gm, own);
        dd.lodb = &lodb;
        uint64_t gv = 0; for (auto const &t : f.tris) gv += t.poly < lodb.size() && lodb[t.poly] != 0xffffffffu;
        geo_given += gv; geo_tris += f.tris.size();
      }
      rimage img; d25_stats st;
      render_d25(f, tex, o, dd, img, st);
      std::vector<quad> s;
      for (int b = 0; b < 48; b++) for (int p = 0; p < 4; p++) s.insert(s.end(), h->bucket[p][b].begin(), h->bucket[p][b].end());
      res &r = rr[i];
      r.px = h->npx; r.quads = h->quads; r.minified = h->minified;
      for (int k = 0; k < 12; k++) r.lvn[k] = h->lvn[k];
      sim(s, SH, nsh, r);
      r.ok = true;
    }
  };
  std::vector<std::thread> th;
  for (int k = 0; k < threads; k++) th.emplace_back(wk);
  for (auto &t : th) t.join();
  std::vector<res> ok;
  for (auto &r : rr) if (r.ok) ok.push_back(r);
  auto pct = [&](auto get) {
    std::vector<uint64_t> v; for (auto &r : ok) v.push_back(get(r));
    std::sort(v.begin(), v.end());
    return std::make_pair(v[v.size() / 2], v[std::min(v.size() - 1, v.size() * 99 / 100)]);
  };
  uint64_t tp = 0, tq = 0, tm = 0, tl[12] = {};
  for (auto &r : ok) { tp += r.px; tq += r.quads; tm += r.minified; for (int k = 0; k < 12; k++) tl[k] += r.lvn[k]; }
  auto px = pct([](res const &r) { return r.px; });
  auto qd = pct([](res const &r) { return r.quads; });
  std::printf("mip %d lod %d%s, %zu frames, D %d F %d\n", d.mip, d.lod, vrom.empty() ? "" : " (B from the geometry model)", ok.size(), ev("TC_D", 64), ev("TC_F", 8));
  if (!vrom.empty()) std::printf("triangles with the geometry's B: %" PRIu64 " of %" PRIu64 "\n", uint64_t(geo_given), uint64_t(geo_tris));
  std::printf("textured passing pixels a frame p50 / p99: %" PRIu64 " / %" PRIu64 "; quads p50 / p99 %" PRIu64 " / %" PRIu64 " (%.3f a pixel); minified %.1f%%; level 0..5:",
              px.first, px.second, qd.first, qd.second, double(tq) / double(tp), 100.0 * double(tm) / double(tp));
  for (int k = 0; k < 6; k++) std::printf(" %.1f%%", 100.0 * double(tl[k]) / double(tp));
  std::printf("\n");
  for (int s = 0; s < nsh; s++) {
    auto m = pct([&](res const &r) { return r.miss[s]; });
    std::printf("%-7s line fills p50 / p99 %7" PRIu64 " / %7" PRIu64 " (p50 %.1f, p99 %.1f MB/s)\n", SH[s].name, m.first, m.second,
                double(m.first) * 32 * 57.52 / 1e6, double(m.second) * 32 * 57.52 / 1e6);
    for (int mi = 0; mi < NMEM; mi++) {
      auto c = pct([&](res const &r) { return r.cyc[s][mi]; });
      std::printf("    %-14s textured cycles p50 / p99 %8" PRIu64 " / %8" PRIu64 "  (p50 %.1f%%, p99 %.1f%% of a frame)\n", MEMS[mi].name, c.first, c.second,
                  100.0 * double(c.first) / 1738530.0, 100.0 * double(c.second) / 1738530.0);
    }
  }
  return 0;
}
