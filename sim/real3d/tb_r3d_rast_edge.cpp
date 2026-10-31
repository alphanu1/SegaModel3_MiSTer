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
// r3d_rast_edge (or, built with -DFRONT, r3d_rast_front: the slope divider and
// the walker) against the model's spans (r3d_rast_rtl.cpp rtl_tri::span),
// every triangle of every frame, in draw order, with random gaps on the input
// and random stalls on the output.
//
// Each triangle goes in once per band it touches (--band N rows; 0: one band,
// the whole screen), and the expected spans are the model's rows in that band.
//
//   Vr3d_rast_edge --tris DIR [--frames N] [--gap PCT] [--stall PCT] [--seed N] [--band N]

#ifdef FRONT
#include "Vr3d_rast_front.h"
using top_t = Vr3d_rast_front;
#else
#include "Vr3d_rast_edge.h"
using top_t = Vr3d_rast_edge;
#endif
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

struct sp { int y, x0, x1; };

int main(int argc, char **argv) {
  std::string tris;
  int frames = 1 << 30, gap = 20, stall = 20;
  unsigned seed = 1;
  int synth = 0, band = 16;   // N random triangles on a 1/4 px grid (ties in x and y), instead of the lists
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--gap") gap = std::atoi(nx().c_str());
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--synthetic") synth = std::atoi(nx().c_str());
    else if (a == "--band") band = std::atoi(nx().c_str());
  }
  std::vector<std::string> names;
  if (synth > 0) names.push_back("synthetic");
  else for (auto const &e : fs::directory_iterator(tris))
    if (e.path().extension() == ".tri") names.push_back(e.path().string());
  std::sort(names.begin(), names.end());
  if (int(names.size()) > frames) names.resize(size_t(frames));

  Verilated::commandArgs(argc, argv);
  top_t top;
  std::mt19937 rng(seed);
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  uint64_t cyc = 0;
  auto tick = [&]() { top.clk = 0; top.eval(); top.clk = 1; top.eval(); cyc++; };
  top.rst = 1; top.t_valid = 0; top.s_ready = 0;
  tick(); tick();
  top.rst = 0;

  rtl_opts const o;
  uint64_t ntri = 0, nspan = 0, bad = 0, frames_ok = 0;
  std::vector<uint64_t> fcyc;
  for (auto const &fn : names) {
    rframe f;
    if (synth > 0) {
      // vertices on a quarter-pixel grid, some far off screen, some flat or tiny:
      // exact half-pixel ties in rows and spans
      std::mt19937 g(seed * 7919u);
      for (int i = 0; i < synth; i++) {
        rtri t{};
        int const sc = (g() % 4 == 0) ? 64 : 1;
        for (int k = 0; k < 3; k++) {
          t.x[k] = float(int(g() % 2400) - 600) * 0.25f * float(sc == 64 && k == 0 ? 1 : 1);
          t.y[k] = float(int(g() % 1900) - 200) * 0.25f;
          if (g() % 8 == 0) t.y[k] = t.y[(k + 1) % 3];          // flat edges
          if (g() % 16 == 0) t.x[k] = float(int(g() % 4000) - 2000) * 2.0f;  // long slopes
          t.w[k] = 1.0f;
        }
        t.buf = uint32_t(i);
        f.tris.push_back(t);
      }
    } else if (!read_rframe(fn, f)) continue;
    // draw order: pass by pass, opaque forwards, alpha backwards
    std::vector<int> order;
    for (uint32_t pass = 0; pass < 4; pass++) {
      std::vector<int> op, al;
      for (size_t i = 0; i < f.tris.size(); i++)
        if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int(i));
      std::sort(op.begin(), op.end(), [&](int a, int b) { return f.tris[a].buf < f.tris[b].buf; });
      std::sort(al.begin(), al.end(), [&](int a, int b) { return f.tris[a].buf > f.tris[b].buf; });
      order.insert(order.end(), op.begin(), op.end());
      order.insert(order.end(), al.begin(), al.end());
    }
    struct in_t { rtl_tri r; int b0, b1; };
    std::vector<in_t> ins;
    std::deque<sp> expect;
    int const bh = band > 0 ? band : SCR_H;
    for (int idx : order) {
      rtri const &t = f.tris[size_t(idx)];
      bool fin = true;
      for (int k = 0; k < 3; k++) fin = fin && std::isfinite(t.x[k]) && std::isfinite(t.y[k]) && std::isfinite(t.w[k]);
      if (!fin) continue;   // dropped upstream
      in_t it;
      if (!it.r.setup(t, kind_of(t), o)) continue;   // no rows: not binned
      for (int b = it.r.y0 / bh; b <= (it.r.y1 - 1) / bh; b++) {
        it.b0 = b * bh; it.b1 = std::min((b + 1) * bh, SCR_H);
        ins.push_back(it);
        for (int32_t y = std::max(it.r.y0, it.b0); y < std::min(it.r.y1, it.b1); y++) {
          int32_t a, c;
          if (it.r.span(y, a, c)) expect.push_back({y, a, c});
        }
      }
    }
    uint64_t const c0 = cyc;
    size_t next = 0;
    uint64_t got = 0, fbad = 0;
    uint64_t const want = expect.size();
    uint64_t idle = 0;
    while ((next < ins.size() || top.busy || !expect.empty()) && idle < 2000000) {
      // drive
      bool const offer = next < ins.size() && !pct(gap);
      if (offer) {
        rtl_tri const &r = ins[next].r;
        top.t_x1 = uint32_t(int32_t(r.vx[0])); top.t_y1 = uint32_t(int32_t(r.vy[0]));
        top.t_x2 = uint32_t(int32_t(r.vx[1])); top.t_y2 = uint32_t(int32_t(r.vy[1]));
        top.t_x3 = uint32_t(int32_t(r.vx[2])); top.t_y3 = uint32_t(int32_t(r.vy[2]));
        top.t_b0 = uint32_t(ins[next].b0); top.t_b1 = uint32_t(ins[next].b1);
#ifndef FRONT
        top.t_s12 = uint64_t(int64_t(r.s12)) & ((uint64_t(1) << 54) - 1);
        top.t_s13 = uint64_t(int64_t(r.s13)) & ((uint64_t(1) << 54) - 1);
        top.t_s23 = uint64_t(int64_t(r.s23)) & ((uint64_t(1) << 54) - 1);
#endif
      }
      top.t_valid = offer;
      top.s_ready = !pct(stall);
      top.clk = 0; top.eval();
      bool const took = top.t_valid && top.t_ready;
      bool const out = top.s_valid && top.s_ready;
      sp const g{int(top.s_y), int(top.s_x0), int(top.s_x1)};
      top.clk = 1; top.eval(); cyc++;
      if (took) next++;
      if (out) {
        got++;
        if (expect.empty()) { fbad++; if (bad + fbad < 10) std::printf("  %s: extra span y %d %d..%d\n", fn.c_str(), g.y, g.x0, g.x1); }
        else {
          sp const e = expect.front();
          expect.pop_front();
          if (e.y != g.y || e.x0 != g.x0 || e.x1 != g.x1) {
            fbad++;
            if (bad + fbad < 10) std::printf("  %s: span %" PRIu64 " rtl y %d %d..%d model y %d %d..%d\n", fn.c_str(), got, g.y, g.x0, g.x1, e.y, e.x0, e.x1);
          }
        }
        idle = 0;
      } else idle++;
    }
    if (!expect.empty()) { fbad += expect.size(); if (bad + fbad < 10) std::printf("  %s: %zu spans missing\n", fn.c_str(), expect.size()); }
    ntri += ins.size();
    nspan += want;
    bad += fbad;
    frames_ok += fbad == 0;
    fcyc.push_back(cyc - c0);
  }
  std::sort(fcyc.begin(), fcyc.end());
  auto q = [&](double p) { return fcyc.empty() ? 0 : fcyc[std::min(fcyc.size() - 1, size_t(p * double(fcyc.size())))]; };
  std::printf("%s: %zu frames, %" PRIu64 " match; %" PRIu64 " (triangle, band) pairs, %" PRIu64 " spans, %" PRIu64 " wrong; gap %d%%, stall %d%%, band %d\n",
#ifdef FRONT
              "r3d_rast_front (divider + walker)",
#else
              "r3d_rast_edge",
#endif
              fcyc.size(), frames_ok, ntri, nspan, bad, gap, stall, band);
  std::printf("cycles per frame (one pair at a time) p50 %" PRIu64 " p99 %" PRIu64 " max %" PRIu64 "\n", q(0.5), q(0.99), q(1.0));
  return bad ? 1 : 0;
}
