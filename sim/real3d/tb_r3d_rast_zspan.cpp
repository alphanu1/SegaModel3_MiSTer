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
// r3d_rast_zspan against the model: every span of every triangle of the
// lists, with the depth plane (plane 0: 1/w with --oow, else w), each pixel's
// Z word against z_word and its plane value against at2. Random input gaps
// and output stalls; then random planes at the bounds make_plane allows.
//
//   Vr3d_rast_zspan --tris DIR [--frames N] [--gap PCT] [--stall PCT] [--oow] [--random N]

#include "Vr3d_rast_zspan.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;
using i128 = __int128;


struct span_in { int y, x0, x1; int64_t c, dx, dy; int e; };
struct px_out { int y, x; uint32_t z; uint64_t v; };

static int msb128(unsigned __int128 v) { int n = -1; while (v) { v >>= 1; n++; } return n; }
static uint32_t zword(i128 v2, int e) {
  if (v2 <= 0) return 0;
  int const n = msb128((unsigned __int128)v2);
  int const ex = n + (e - 41) + 127;
  if (ex <= 0) return 0;
  if (ex >= 255) return 0xffffff;
  unsigned __int128 const u = (unsigned __int128)v2;
  uint32_t const man = uint32_t((n >= 16 ? (u >> (n - 16)) : (u << (16 - n))) & 0xffff);
  return (uint32_t(ex) << 16) | man;
}

int main(int argc, char **argv) {
  std::string tris;
  int frames = 1 << 30, gap = 20, stall = 20, nrand = 0;
  rtl_opts o;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--gap") gap = std::atoi(nx().c_str());
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--oow") o.oow = true;
    else if (a == "--random") nrand = std::atoi(nx().c_str());
  }
  std::vector<span_in> spans;
  std::deque<px_out> expect;
  uint64_t big = 0;
  uint64_t const M42 = (uint64_t(1) << 42) - 1;
  auto add = [&](span_in const &s) {
    spans.push_back(s);
    for (int x = s.x0; x < s.x1; x++) {
      i128 const v = i128(s.c) * 2 + i128(s.dx) * (2 * x + 1) + i128(s.dy) * (2 * s.y + 1);
      if (v >= (i128(1) << 41) || v < -(i128(1) << 41)) big++;
      expect.push_back({s.y, x, zword(v, s.e), uint64_t(int64_t(v)) & M42});
    }
  };
  uint64_t ntri = 0;
  if (!tris.empty()) {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    for (auto const &fn : names) {
      rframe f;
      if (!read_rframe(fn, f)) continue;
      for (auto const &t : f.tris) {
        rtl_tri r;
        if (!r.setup(t, kind_of(t), o)) continue;
        ntri++;
        auto const &p = r.pl[0];
        for (int32_t y = r.y0; y < r.y1; y++) {
          int32_t a, b;
          if (r.span(y, a, b)) add({y, a, b, p.c, p.dx, p.dy, p.e});
        }
      }
    }
  }
  std::mt19937 g(3);
  for (int i = 0; i < nrand; i++) {
    span_in s;
    s.y = int(g() % 384);
    s.x0 = int(g() % 496);
    s.x1 = s.x0 + 1 + int(g() % std::min<uint32_t>(uint32_t(496 - s.x0), (g() & 1) ? 4u : 496u));
    auto r = [&](int bits) { int64_t m = (int64_t(1) << bits) - 1; int64_t v = int64_t(uint64_t(g()) << 32 | g()) & m; return (g() & 1) ? -v : v; };
    s.c = r(38); s.dx = r(29 - int(g() % 20)); s.dy = r(29 - int(g() % 20));
    s.e = int(g() % 300) - 150;
    if (g() % 64 == 0) { s.c = 0; s.dx = 0; s.dy = 0; }   // a zero plane: v2 = 0 exactly
    if (g() % 64 == 1) { s.dx = 0; s.dy = 0; s.c = (g() & 1) ? 0 : -1; }
    add(s);
  }

  Verilated::commandArgs(argc, argv);
  Vr3d_rast_zspan top;
  std::mt19937 rng(1);
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  top.rst = 1; top.s_valid = 0; top.p_ready = 0;
  for (int k = 0; k < 2; k++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;
  size_t si = 0;
  uint64_t bad = 0, got = 0, cyc = 0, idle = 0;
  uint64_t const want = expect.size();
  uint64_t const M40 = (uint64_t(1) << 40) - 1;
  while ((si < spans.size() || !expect.empty()) && idle < 100000) {
    bool const offer = si < spans.size() && !pct(gap);
    if (offer) {
      span_in const &s = spans[si];
      top.s_y = uint32_t(s.y); top.s_x0 = uint32_t(s.x0); top.s_x1 = uint32_t(s.x1);
      top.s_c = uint64_t(s.c) & M40; top.s_dx = uint64_t(s.dx) & M40; top.s_dy = uint64_t(s.dy) & M40;
      top.s_e = uint32_t(s.e) & 0x1ff;
    }
    top.s_valid = offer;
    top.p_ready = !pct(stall);
    top.clk = 0; top.eval();
    bool const took = top.s_valid && top.s_ready;
    bool const out = top.p_valid && top.p_ready;
    px_out lane[2] = {{int(top.p_y), int(top.p_x), top.p_z0, uint64_t(top.p_v0) & M42},
                      {int(top.p_y), int(top.p_x) + 1, top.p_z1, uint64_t(top.p_v1) & M42}};
    int const on = int(top.p_on);
    top.clk = 1; top.eval(); cyc++;
    if (took) si++;
    if (!out) { idle++; continue; }
    idle = 0;
    for (int l = 0; l < 2; l++) {
      if (!(on >> l & 1)) continue;
      got++;
      if (expect.empty()) { if (bad++ < 10) std::printf("  extra pixel\n"); continue; }
      px_out const e = expect.front(); expect.pop_front();
      px_out const &q = lane[l];
      if (e.y != q.y || e.x != q.x || e.z != q.z || e.v != q.v)
        if (bad++ < 10) std::printf("  pixel %" PRIu64 ": rtl (%d,%d) z %06x v %011" PRIx64 ", model (%d,%d) z %06x v %011" PRIx64 "\n",
                                    got, q.x, q.y, q.z, q.v, e.x, e.y, e.z, e.v);
    }
  }
  bad += expect.size();
  std::printf("r3d_rast_zspan: %" PRIu64 " triangles, %zu spans (%d random), %" PRIu64 " pixels, %" PRIu64 " wrong; plane values past 42 bits %" PRIu64 "; %.2f cycles a span (gap %d%%, stall %d%%)\n",
              ntri, spans.size(), nrand, want, bad, big, double(cyc) / double(std::max<size_t>(spans.size(), 1)), gap, stall);
  return bad ? 1 : 0;
}
