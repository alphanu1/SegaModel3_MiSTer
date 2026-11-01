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
// r3d_rast_fogp against render_d25: every pixel fog applies to, its P against
// the model's; --random: 1/w values of either sign, exponents and K over
// their range against fog_p. Random stalls.
//
//   Vr3d_rast_fogp --tris DIR --dumps DIR [--frames N] | --random N

#include "Vr3d_rast_fogp.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>
#include <zlib.h>

using namespace r3d;
namespace fs = std::filesystem;
using i128 = __int128;

struct fin { uint32_t r, n, eo, pos, k, p; };
static int msb128(unsigned __int128 v) { int n = -1; while (v) { v >>= 1; n++; } return n; }
static fin mk(i128 o2, int eo, float K) {
  fin f{};
  f.eo = uint32_t(eo) & 0x1ff; f.pos = o2 > 0;
  std::memcpy(&f.k, &K, 4);
  if (o2 != 0) {
    unsigned __int128 const a = (unsigned __int128)(o2 < 0 ? -o2 : o2);
    int const n = msb128(a);
    uint64_t const m = uint64_t(n >= 23 ? (a >> (n - 23)) : (a << (23 - n)));
    f.n = uint32_t(n); f.r = uint32_t(((uint64_t(1) << 47) + m / 2) / m);
  }
  f.p = uint32_t(d25_fog_p(o2, eo, K));
  return f;
}
static i128 at2(rtl_tri::plane const &p, int x, int y) { return i128(p.c) * 2 + i128(p.dx) * (2 * x + 1) + i128(p.dy) * (2 * y + 1); }

struct hook : d25_hook {
  std::vector<rtl_tri> tr; std::vector<fin> *v;
  void tri(int32_t idx, int, rtl_tri const &r, rkind, bool) override { tr[size_t(idx)] = r; }
  void px(d25_px const &p) override {
    if (!p.fog) return;
    rtl_tri const &r = tr[size_t(p.tri)];
    fin f = mk(at2(r.pl[0], p.x, p.y), r.pl[0].e, p.K);
    if (int32_t(f.p) != p.P) std::printf("  bench: model P %d against fog_p %u\n", p.P, f.p);
    v->push_back(f);
  }
};

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, nrand = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--random") nrand = std::atoi(nx().c_str());
  }
  std::vector<fin> v;
  size_t nfr = 0;
  if (nrand) {
    std::mt19937 g(9);
    for (int i = 0; i < nrand; i++) {
      int64_t o = int64_t(uint64_t(g()) << 32 | g()) >> (22 + g() % 40);
      if (g() % 16 == 0) o = -o;
      if (g() % 64 == 0) o = 0;
      int const eo = int(g() % 120) - 50;
      uint32_t kb = (g() & 0x7fffff) | (uint32_t(90 + g() % 60) << 23);
      if (g() % 32 == 0) kb = g();
      if (g() % 64 == 0) kb = 0;
      float K; std::memcpy(&K, &kb, 4);
      v.push_back(mk(i128(o), eo, K));
    }
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    nfr = names.size();
    texram tex; std::vector<uint32_t> dl(0x40000);
    for (auto const &nm : names) {
      rframe f;
      if (!read_rframe(tris + "/" + nm + ".tri", f)) continue;
      gzFile gz = gzopen((dumps + "/" + nm + ".bin.gz").c_str(), "rb");
      bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
                gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
      if (gz) gzclose(gz);
      if (!ok) continue;
      for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
      hook h; h.tr.resize(f.tris.size()); h.v = &v; d.hook = &h;
      rimage img; d25_stats st;
      render_d25(f, tex, o, d, img, st);
    }
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_fogp t;
  std::mt19937 rng(2);
  uint64_t bad = 0, got = 0;
  size_t si = 0;
  std::vector<int> inflight;   // index fed on each enabled edge
  while (got < v.size()) {
    bool const en = rng() % 4 != 0;
    t.en = en;
    bool const feed = si < v.size();
    if (feed) { fin const &f = v[si]; t.i_r = f.r; t.i_n = f.n; t.i_eo = f.eo; t.i_pos = f.pos; t.i_k = f.k; }
    t.clk = 0; t.eval(); t.clk = 1; t.eval();
    if (!en) continue;
    inflight.push_back(feed ? int(si++) : -1);
    if (inflight.size() > 2) {
      int const j = inflight[inflight.size() - 3];
      if (j >= 0) { got++; if (t.o_p != v[size_t(j)].p && bad++ < 10) std::printf("  %d: rtl %u model %u (r %07x n %u eo %d pos %u k %08x)\n", j, t.o_p, v[size_t(j)].p, v[size_t(j)].r, v[size_t(j)].n, int(int16_t(v[size_t(j)].eo << 7) >> 7), v[size_t(j)].pos, v[size_t(j)].k); }
      else if (si >= v.size()) break;
    }
  }
  std::printf("r3d_rast_fogp: %zu frames, %zu pixels (%d random), %" PRIu64 " wrong\n", nfr, v.size(), nrand, bad);
  return bad ? 1 : 0;
}
