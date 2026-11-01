// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// ---------------------------------------------------------------------------
//
// r3d_rast_zband against render_d25: every Z-tested pixel of the frames,
// replayed band by band (8 rows) and pass by pass in the model's draw order,
// each passing pixel and its order checked, contour verdicts returned from the
// model's alpha test after a random shading delay. Random input gaps, output
// stalls and RAM contents at power-up. --synthetic: random overlapping spans
// in one band, ties, contour runs and short generations against a plain Z
// buffer, for the hazards the frames rarely reach.
//
//   Vr3d_rast_zband --tris DIR --dumps DIR [--frames N] [--trans 0|1]
//   Vr3d_rast_zband --synthetic N [--trans 0|1]
//   common: [--gap PCT] [--stall PCT] [--delay MAX]

#include "Vr3d_rast_zband.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <zlib.h>
#include <cstdlib>

using namespace r3d;
namespace fs = std::filesystem;

struct pair_in {
  bool nw; int y, x, on; uint32_t z0, z1; int zk, tag; bool stip; int tr, pinv;
  int ver[2];   // contour verdict a lane (-1 none)
};
struct px_exp { int x, y, tag; };

struct stream {
  std::vector<pair_in> in;
  std::vector<px_exp> out;
  std::vector<int> verdicts;   // in order of passing contour pixels
  uint64_t px = 0, pass = 0, gens = 0;
};

// ---- from the model ---------------------------------------------------------------
struct ev { int32_t tri; int x, y; uint32_t z; bool stip, zpass, ver; };
struct triinfo { rkind k; uint32_t trans, pinv; };

struct hook : d25_hook {
  rframe const *f = nullptr;
  int trans = 0;
  std::vector<ev> bucket[4][48];
  std::vector<triinfo> ti;
  void tri(int32_t idx, int, rtl_tri const &, rkind k, bool) override {
    rtri const &t = f->tris[size_t(idx)];
    ti[size_t(idx)] = {k, t.trans, (t.hdr[6] >> 17) & 1};
  }
  void px(d25_px const &p) override {
    rtri const &t = f->tris[size_t(p.tri)];
    bucket[t.pass][p.y / 8].push_back({p.tri, p.x, p.y, p.z, p.stip_out, p.zpass, p.ok && p.zw});
  }
};

static int zk_of(rkind k) {
  switch (k) {
  case K_SOLID: case K_TEX: case K_TEX_COLORMOD: return 1;
  case K_CONTOUR: return 2;
  default: return 0;
  }
}

static void build(hook &h, rframe const &f, int trans, stream &s) {
  int tag = 0;
  for (int b = 0; b < 48; b++)
    for (int p = 0; p < 4; p++) {
      auto const &v = h.bucket[p][b];
      if (v.empty()) continue;
      bool nw = true;
      s.gens++;
      int32_t last = -1;
      for (size_t i = 0; i < v.size();) {
        ev const &e = v[i];
        if (e.tri != last) { tag = (tag + 1) & 255; last = e.tri; }
        triinfo const &t = h.ti[size_t(e.tri)];
        rtri const &rt = f.tris[size_t(e.tri)];
        pair_in q{};
        q.nw = nw; nw = false;
        q.y = e.y; q.x = e.x; q.z0 = e.z; q.on = 1; q.zk = zk_of(t.k); q.tag = tag;
        q.stip = trans == 1 && rt.trans < 32; q.tr = int(rt.trans & 31); q.pinv = int(t.pinv);
        q.ver[0] = q.ver[1] = -1;
        bool two = i + 1 < v.size() && v[i + 1].tri == e.tri && v[i + 1].y == e.y && v[i + 1].x == e.x + 1;
        int n = two ? 2 : 1;
        if (two) { q.on = 3; q.z1 = v[i + 1].z; }
        for (int l = 0; l < n; l++) {
          ev const &g = v[i + size_t(l)];
          s.px++;
          if (g.zpass) {
            s.pass++;
            s.out.push_back({g.x, g.y, tag});
            if (q.zk == 2) s.verdicts.push_back(g.ver);
          }
        }
        s.in.push_back(q);
        i += size_t(n);
        // a span is a run of x + 1; the next pair starts where this one ended
        (void)rt;
      }
    }
}

// ---- synthetic ----------------------------------------------------------------------
static void synth(int n, int trans, uint32_t seed, stream &s) {
  std::mt19937 g(seed);
  std::vector<uint32_t> zb(8 * 496), zv(8 * 496, 0);
  uint32_t gen = 0;
  int tag = 0;
  constexpr uint8_t BY[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  int band = 0, sparse = 0;
  while (int(s.gens) < n) {
    gen++; s.gens++;
    if (g() % 4 == 0) band = int(g() % 48);
    // a sparse phase: 60-90 generations on one row's first pixels, so the rest
    // of the band goes untouched for longer than the stamp's period
    if (!sparse && g() % 300 == 0) sparse = 60 + int(g() % 31);
    if (sparse) sparse--;
    int const ntri = sparse ? 1 : 1 + int(g() % ((g() & 1) ? 4 : 40));
    bool nw = true;
    for (int t = 0; t < ntri; t++) {
      tag = (tag + 1) & 255;
      int const zk = int(g() % 3) == 0 ? 2 : int(g() % 3);
      bool const stip = trans == 1 && (g() & 1);
      int const tr = int(g() % 32), pinv = int(g() & 1);
      // half the triangles stack on the last one's rows and x: back-to-back hazards
      static int rows = 1, r0 = 0, xc = 0;
      if (g() & 1) { rows = 1 + int(g() % 8); r0 = int(g() % 8); xc = int(g() % 496); }
      bool const micro = sparse || g() % 4 == 0;   // one row of one or two pixels
      if (sparse) { rows = 1; r0 = 0; xc = int(g() % 16); }
      for (int r = 0; r < rows && r0 + r < 8; r++) {
        int const y = band * 8 + r0 + r;
        int const half = int(g() % ((g() & 3) ? 6 : 60));
        int x0 = std::max(0, xc - half + int(g() % 3)), x1 = std::min(496, xc + half + 1 + int(g() % 3));
        if (micro) { x0 = xc; x1 = std::min(496, xc + 1 + int(g() & 1)); if (r > 0) break; }
        if (x1 <= x0) continue;
        for (int x = x0; x < x1; x += 2) {
          pair_in q{};
          q.nw = nw; nw = false;
          q.y = y; q.x = x; q.on = (x + 1 < x1) ? 3 : 1; q.zk = zk; q.tag = tag; q.stip = stip; q.tr = tr; q.pinv = pinv;
          for (int l = 0; l < 2; l++) {
            q.ver[l] = -1;
            uint32_t z = (g() % 4 == 0) ? uint32_t(g() & 0xffffff) : uint32_t(0x400000 + (g() % 8) * 0x1000);
            if (micro && (g() & 1)) z = 0x400000 + uint32_t(tag) * 0x100;
            (l ? q.z1 : q.z0) = z;
            if (!(q.on >> l & 1)) continue;
            int const px = x + l;
            s.px++;
            if (stip) {
              uint32_t bb = BY[y & 3][px & 3];
              if (pinv) bb = 15 - bb;
              if (!(bb < uint32_t(tr) / 2)) continue;
            }
            size_t const a = size_t(y & 7) * 496 + size_t(px);
            uint32_t const ref = zv[a] == gen ? zb[a] : 0;
            if (!(z >= ref)) continue;
            s.pass++;
            s.out.push_back({px, y, tag});
            bool wr = zk == 1;
            if (zk == 2) { int const v = int(g() & 1); s.verdicts.push_back(v); wr = v; }
            if (wr) { zb[a] = z; zv[a] = gen; }
            if (std::getenv("ZB_DBG") && a == size_t(std::atoi(std::getenv("ZB_DBG"))))
              std::printf("dbg gen %u tag %d px %d,%d z %06x ref %06x zk %d wr %d pass# %zu\n", gen, tag, px, y, z, ref, zk, int(wr), s.out.size() - 1);
          }
          s.in.push_back(q);
        }
      }
    }
    if (nw) { s.gens--; gen--; }   // no pixels: no generation
  }
}

// ---- the run ---------------------------------------------------------------------------
static uint64_t run(Vr3d_rast_zband &top, stream const &s, int gap, int stall, int dmax, uint64_t &cyc, uint64_t seed) {
  std::mt19937 rng{uint32_t(seed)};
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  size_t si = 0, oi = 0, vi = 0;
  std::deque<uint64_t> vq;   // cycle each verdict is due
  uint64_t bad = 0, idle = 0;
  while ((si < s.in.size() || oi < s.out.size() || !vq.empty()) && idle < 200000) {
    bool const offer = si < s.in.size() && !pct(gap);
    if (offer) {
      pair_in const &q = s.in[si];
      top.s_new = q.nw; top.s_y = uint32_t(q.y); top.s_x = uint32_t(q.x); top.s_on = uint32_t(q.on);
      top.s_z0 = q.z0; top.s_z1 = q.z1; top.s_p0 = uint64_t(si); top.s_p1 = uint64_t(si) ^ 0x2aaaaaaaaaaULL;
      top.s_zk = uint32_t(q.zk); top.s_tag = uint32_t(q.tag);
      top.s_stip = q.stip; top.s_trans = uint32_t(q.tr); top.s_pinv = uint32_t(q.pinv);
    }
    top.s_valid = offer;
    top.o_ready = !pct(stall);
    bool const vdue = !vq.empty() && vq.front() <= cyc;
    top.c_valid = vdue;
    top.c_zw = vdue ? s.verdicts[vi] : 0;
    top.clk = 0; top.eval();
    bool const took = top.s_valid && top.s_ready;
    bool const out = top.o_valid && top.o_ready;
    int const on = int(top.o_on), oy = int(top.o_y), ox = int(top.o_x), ot = int(top.o_tag), octr = top.o_ctr;
    top.clk = 1; top.eval(); cyc++;
    if (took) si++;
    if (vdue) { vq.pop_front(); vi++; }
    if (!out) { idle++; continue; }
    idle = 0;
    for (int l = 0; l < 2; l++) {
      if (!(on >> l & 1)) continue;
      if (oi >= s.out.size()) { if (bad++ < 10) std::printf("  extra pixel (%d,%d)\n", ox + l, oy); continue; }
      px_exp const &e = s.out[oi++];
      if (e.x != ox + l || e.y != oy || e.tag != ot)
        if (bad++ < 10) std::printf("  pass %zu: rtl (%d,%d) tag %d, model (%d,%d) tag %d\n", oi - 1, ox + l, oy, ot, e.x, e.y, e.tag);
      if (octr) vq.push_back(cyc + uint64_t(dmax ? rng() % uint32_t(dmax + 1) : 0));
    }
    // verdicts are in order: a later one is never due before an earlier one
    for (size_t k = 1; k < vq.size(); k++) vq[k] = std::max(vq[k], vq[k - 1] + 1);
  }
  if (oi < s.out.size()) { bad += s.out.size() - oi; std::printf("  %zu passing pixels missing\n", s.out.size() - oi); }
  if (si < s.in.size()) { bad++; std::printf("  stuck at input pair %zu of %zu\n", si, s.in.size()); }
  return bad;
}

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, gap = 20, stall = 20, dmax = 40, nsyn = 0, trans = 0;
  uint32_t seed = 7;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--gap") gap = std::atoi(nx().c_str());
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--delay") dmax = std::atoi(nx().c_str());
    else if (a == "--synthetic") nsyn = std::atoi(nx().c_str());
    else if (a == "--trans") trans = std::atoi(nx().c_str());
    else if (a == "--seed") seed = uint32_t(std::atoi(nx().c_str()));
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_zband top;
  top.rst = 1; top.s_valid = 0; top.o_ready = 0; top.c_valid = 0;
  for (int k = 0; k < 2; k++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;
  uint64_t bad = 0, cyc = 0, npx = 0, npass = 0, npairs = 0, ngen = 0, nver = 0;
  size_t nfr = 0;
  if (nsyn) {
    stream s;
    synth(nsyn, trans, seed, s);
    bad += run(top, s, gap, stall, dmax, cyc, 1);
    npx = s.px; npass = s.pass; npairs = s.in.size(); ngen = s.gens; nver = s.verdicts.size();
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    nfr = names.size();
    // the model renders frames on worker threads, a few ahead of the RTL
    std::vector<stream> ss(names.size());
    std::vector<uint64_t> fcyc;
    std::vector<std::atomic<int>> ready(names.size());
    for (auto &r : ready) r = 0;
    std::atomic<size_t> nxt{0}, used{0};
    auto wk = [&]() {
      texram tex;
      std::vector<uint32_t> dl(0x40000);
      for (size_t i; (i = nxt++) < names.size();) {
        while (i > used + 16) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        rframe f;
        bool ok = read_rframe(tris + "/" + names[i] + ".tri", f);
        gzFile gz = ok ? gzopen((dumps + "/" + names[i] + ".bin.gz").c_str(), "rb") : nullptr;
        ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
             gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
        if (gz) gzclose(gz);
        if (ok) {
          for (auto &v : f.vps)
            for (int k = 0; k < 0x30; k++) v.node[k] = dl[((v.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
          rtl_opts o; o.oow = true;
          d25_opts d; d.trans = trans; d.band = 8;
          hook h; h.f = &f; h.trans = trans; h.ti.resize(f.tris.size());
          d.hook = &h;
          rimage img; d25_stats st;
          render_d25(f, tex, o, d, img, st);
          build(h, f, trans, ss[i]);
        }
        ready[i] = ok ? 1 : 2;
      }
    };
    std::vector<std::thread> th;
    for (int k = 0; k < 12; k++) th.emplace_back(wk);
    for (size_t i = 0; i < names.size(); i++) {
      while (!ready[i]) std::this_thread::yield();
      if (ready[i] == 2) { std::printf("  %s: no data\n", names[i].c_str()); bad++; continue; }
      stream &s = ss[i];
      uint64_t const c0 = cyc;
      uint64_t const b = run(top, s, gap, stall, dmax, cyc, i + 1);
      fcyc.push_back(cyc - c0);
      if (b) std::printf("  %s: %" PRIu64 " wrong\n", names[i].c_str(), b);
      bad += b;
      npx += s.px; npass += s.pass; npairs += s.in.size(); ngen += s.gens; nver += s.verdicts.size();
      s = stream();
      used = i + 1;
    }
    for (auto &t : th) t.join();
    std::sort(fcyc.begin(), fcyc.end());
    if (!fcyc.empty())
      std::printf("  cycles a frame p50 / p99 / max: %" PRIu64 " / %" PRIu64 " / %" PRIu64 " (%.1f%% of a 100 MHz frame at p99)\n",
                  fcyc[fcyc.size() / 2], fcyc[std::min(fcyc.size() - 1, fcyc.size() * 99 / 100)], fcyc.back(),
                  100.0 * double(fcyc[std::min(fcyc.size() - 1, fcyc.size() * 99 / 100)]) / 1738530.0);
  }
  std::printf("r3d_rast_zband: %zu frames, %" PRIu64 " generations, %" PRIu64 " pairs, %" PRIu64 " pixels, %" PRIu64
              " passing, %" PRIu64 " contour verdicts, %" PRIu64 " wrong; %.3f cycles a pair (gap %d%%, stall %d%%, delay %d)\n",
              nfr, ngen, npairs, npx, npass, nver, bad, double(cyc) / double(std::max<uint64_t>(npairs, 1)), gap, stall, dmax);
  return bad ? 1 : 0;
}
