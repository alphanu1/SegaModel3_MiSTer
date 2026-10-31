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
// r3d_rast_intake against the model: every triangle of the lists as the
// geometry's set-up record (m2-notes.md section 14) built from
// rtl_tri::setup's FP32 results, and each output field against rtl_tri's
// fixed-point vertices and planes. Random input gaps and output stalls; then
// random records of random FP32 values for the converter's whole range.
//
//   Vr3d_rast_intake --tris DIR [--frames N] [--gap PCT] [--stall PCT] [--random N] [--oow]

#include "Vr3d_rast_intake.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

struct field { int tag; uint64_t v; };

// the model's conversions, written out for the random set
static int64_t to_fix16(float f) {
  double const d = std::ldexp(double(f), 16);
  if (!(d > -2147483648.0 && d < 2147483647.0)) return d > 0 ? 2147483647 : -2147483647 - 1;
  return std::llround(d);
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
    else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--oow") o.oow = true;
  }
  std::vector<std::vector<uint32_t>> recs;   // record words
  std::deque<field> expect;
  uint64_t const M40 = (uint64_t(1) << 40) - 1;
  auto add_tri = [&](rtl_tri const &r, uint32_t w0) {
    std::vector<uint32_t> w = {w0 | (r.tex ? 0x10u : 0u), 0x12345u, 0xabcdefu};
    for (int k = 0; k < 3; k++) { w.push_back(fb(r.fx[k])); w.push_back(fb(r.fy[k])); }
    w.push_back(fb(r.det));
    for (int p = 0; p < r.np; p++) { w.push_back(fb(r.pdx[p])); w.push_back(fb(r.pdy[p])); w.push_back(fb(r.ps[p])); }
    recs.push_back(w);
    for (int k = 0; k < 3; k++) expect.push_back({15, w[size_t(k)]});
    for (int k = 0; k < 3; k++) { expect.push_back({2 * k, uint64_t(uint32_t(int32_t(r.vx[k])))}); expect.push_back({2 * k + 1, uint64_t(uint32_t(int32_t(r.vy[k])))}); }
    expect.push_back({6, fb(r.det)});
    for (int p = 0; p < r.np; p++) {
      expect.push_back({8 + 4 * p, uint64_t(int64_t(r.pl[p].e)) & M40});
      expect.push_back({9 + 4 * p, uint64_t(r.pl[p].c) & M40});
      expect.push_back({10 + 4 * p, uint64_t(r.pl[p].dx) & M40});
      expect.push_back({11 + 4 * p, uint64_t(r.pl[p].dy) & M40});
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
        add_tri(r, 0);
        ntri++;
      }
    }
  }
  // random records: vertices over the whole FP32 range (saturation, ties) and
  // coefficients at random exponents, the model's make_plane rule
  std::mt19937 g(7);
  for (int i = 0; i < nrand; i++) {
    rtl_tri r;
    r.tex = g() & 1; r.np = r.tex ? 5 : 2;
    auto rf = [&](int emin, int emax) {
      uint32_t const E = uint32_t(emin + int(g() % uint32_t(emax - emin + 1)));
      uint32_t u = (g() & 0x80000000u) | (E << 23) | (g() & 0x7fffff);
      if (g() % 16 == 0) u &= 0xff800000u | (g() & 0x7f0000u);   // short mantissas: exact ties
      if (g() % 32 == 0) u = g() & 0x80000000u;                   // zeros
      float f; std::memcpy(&f, &u, 4); return f;
    };
    for (int k = 0; k < 3; k++) {
      r.fx[k] = rf(100, 160); r.fy[k] = rf(100, 160);
      if (g() % 64 == 0) { uint32_t u = 0x7f800000u | (g() % 3 ? 0 : 1) | (g() & 0x80000000u); std::memcpy(&r.fx[k], &u, 4); }
      r.vx[k] = to_fix16(r.fx[k]); r.vy[k] = to_fix16(r.fy[k]);
    }
    r.det = rf(100, 150);
    for (int p = 0; p < r.np; p++) {
      r.ps[p] = rf(60, 200); r.pdx[p] = rf(60, 190); r.pdy[p] = rf(60, 190);
      // make_plane as the model does it
      long double const m = std::max({std::fabs((long double)r.ps[p]), std::fabs((long double)r.pdx[p]) * 512.0L, std::fabs((long double)r.pdy[p]) * 512.0L});
      int ex = 0;
      if (m > 0) std::frexp(m, &ex);
      r.pl[p].e = ex + 2;
      auto q = [&](float f) { return std::llround(std::ldexp((long double)f, 40 - r.pl[p].e)); };
      r.pl[p].c = q(r.ps[p]); r.pl[p].dx = q(r.pdx[p]); r.pl[p].dy = q(r.pdy[p]);
    }
    add_tri(r, 0);
  }

  Verilated::commandArgs(argc, argv);
  Vr3d_rast_intake top;
  std::mt19937 rng(1);
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  top.rst = 1; top.i_valid = 0; top.o_ready = 0;
  for (int k = 0; k < 2; k++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;
  size_t ri = 0, wi = 0;
  uint64_t got = 0, bad = 0, cyc = 0, idle = 0;
  uint64_t const want = expect.size();
  while ((ri < recs.size() || !expect.empty()) && idle < 100000) {
    bool const offer = ri < recs.size() && !pct(gap);
    if (offer) { top.i_data = recs[ri][wi]; top.i_last = wi + 1 == recs[ri].size(); }
    top.i_valid = offer;
    top.o_ready = !pct(stall);
    top.clk = 0; top.eval();
    bool const took = top.i_valid && top.i_ready;
    bool const out = top.o_valid && top.o_ready;
    field const gf{int(top.o_tag), uint64_t(top.o_data) & M40};
    top.clk = 1; top.eval(); cyc++;
    if (took && ++wi == recs[ri].size()) { wi = 0; ri++; }
    if (out) {
      got++; idle = 0;
      if (expect.empty()) { if (bad++ < 10) std::printf("  extra field tag %d\n", gf.tag); continue; }
      field const e = expect.front(); expect.pop_front();
      uint64_t const ev = (e.tag <= 6 || e.tag == 15) ? (e.v & 0xffffffffu) : e.v;
      uint64_t const gv = (e.tag <= 6 || e.tag == 15) ? (gf.v & 0xffffffffu) : gf.v;
      if (e.tag != gf.tag || ev != gv)
        if (bad++ < 10) std::printf("  field %" PRIu64 ": rtl tag %d %010" PRIx64 ", model tag %d %010" PRIx64 "\n", got, gf.tag, gv, e.tag, ev);
    } else idle++;
  }
  bad += expect.size();
  std::printf("r3d_rast_intake: %" PRIu64 " list triangles, %d random records, %" PRIu64 " fields, %" PRIu64 " wrong; %.1f cycles a record (gap %d%%, stall %d%%)\n",
              ntri, nrand, want, bad, double(cyc) / double(std::max<size_t>(recs.size(), 1)), gap, stall);
  return bad ? 1 : 0;
}
