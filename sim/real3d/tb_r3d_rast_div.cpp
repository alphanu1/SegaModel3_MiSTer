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
// r3d_rast_div against the model's slopes (r3d_rast_rtl.cpp rtl_tri::setup),
// every finite triangle of the lists, then random vertex sets over the whole
// 32-bit range.
//
//   Vr3d_rast_div --tris DIR [--frames N] [--random N]

#include "Vr3d_rast_div.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

int main(int argc, char **argv) {
  std::string tris;
  int frames = 1 << 30, nrand = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--random") nrand = std::atoi(nx().c_str());
  }
  // the inputs: y-sorted fixed vertices and the model's slopes
  struct job { int64_t x[3], y[3]; __int128 s[3]; };
  std::vector<job> jobs;
  rtl_opts const o;
  if (!tris.empty()) {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    for (auto const &fn : names) {
      rframe f;
      if (!read_rframe(fn, f)) continue;
      for (auto const &t : f.tris) {
        bool fin = true;
        for (int k = 0; k < 3; k++) fin = fin && std::isfinite(t.x[k]) && std::isfinite(t.y[k]) && std::isfinite(t.w[k]);
        if (!fin) continue;
        rtl_tri r;
        r.setup(t, kind_of(t), o);
        jobs.push_back({{r.vx[0], r.vx[1], r.vx[2]}, {r.vy[0], r.vy[1], r.vy[2]}, {r.s12, r.s13, r.s23}});
      }
    }
  }
  std::mt19937_64 g(5);
  for (int i = 0; i < nrand; i++) {
    job j;
    int const sh = int(g() % 32);   // spread of magnitudes
    for (int k = 0; k < 3; k++) {
      j.x[k] = int64_t(int32_t(uint32_t(g()))) >> sh;
      j.y[k] = int64_t(int32_t(uint32_t(g()))) >> int(g() % 32);
    }
    std::sort(std::begin(j.y), std::end(j.y));
    auto sl = [&](int a, int b) -> __int128 {
      if (j.y[b] == j.y[a]) return 0;
      return (__int128(j.x[b] - j.x[a]) << 20) / __int128(j.y[b] - j.y[a]);
    };
    j.s[0] = sl(0, 1); j.s[1] = sl(0, 2); j.s[2] = sl(1, 2);
    jobs.push_back(j);
  }

  Verilated::commandArgs(argc, argv);
  Vr3d_rast_div top;
  auto tick = [&]() { top.clk = 0; top.eval(); top.clk = 1; top.eval(); };
  top.rst = 1; top.i_valid = 0; top.o_ready = 1;
  tick(); tick();
  top.rst = 0;
  size_t next = 0, done = 0;
  uint64_t bad = 0, cyc = 0;
  std::mt19937 rr(3);
  auto s54 = [](uint64_t v) { return __int128(int64_t(v << 10) >> 10); };
  while (done < jobs.size() && cyc < 400000000ull) {
    if (next < jobs.size()) {
      auto const &j = jobs[next];
      top.i_x1 = uint32_t(int32_t(j.x[0])); top.i_y1 = uint32_t(int32_t(j.y[0]));
      top.i_x2 = uint32_t(int32_t(j.x[1])); top.i_y2 = uint32_t(int32_t(j.y[1]));
      top.i_x3 = uint32_t(int32_t(j.x[2])); top.i_y3 = uint32_t(int32_t(j.y[2]));
    }
    top.i_valid = next < jobs.size() && rr() % 5 != 0;
    top.o_ready = rr() % 5 != 0;
    top.clk = 0; top.eval();
    bool const took = top.i_valid && top.i_ready, out = top.o_valid && top.o_ready;
    __int128 const g0 = s54(top.o_s12), g1 = s54(top.o_s13), g2 = s54(top.o_s23);
    top.clk = 1; top.eval(); cyc++;
    if (took) next++;
    if (out) {
      auto const &j = jobs[done++];
      if (g0 != j.s[0] || g1 != j.s[1] || g2 != j.s[2]) {
        if (bad < 8) std::printf("  job %zu: rtl %lld %lld %lld model %lld %lld %lld\n", done - 1, (long long)g0, (long long)g1, (long long)g2,
                                 (long long)j.s[0], (long long)j.s[1], (long long)j.s[2]);
        bad++;
      }
    }
  }
  std::printf("r3d_rast_div: %zu triangles, %zu done, %" PRIu64 " wrong; %.1f cycles a triangle (20%% gaps and stalls)\n",
              jobs.size(), done, bad, double(cyc) / double(std::max<size_t>(done, 1)));
  return bad || done != jobs.size() ? 1 : 0;
}
