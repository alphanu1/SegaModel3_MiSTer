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
// r3d_face against the model's FP32 back-face test (r3d_geo.cpp), random
// matrices and polygons, with random gaps on both input ports and the unit's
// write-port holds honoured. Built with -DFACE_LOD (LOD = 1; FACE_KZ2 the
// KZ2 parameter's word): each polygon's NP word too, B = d x (NP x C) against
// the model in order, and C (-cota, random) changed while the unit is idle.
//
//   Vr3d_face [polygons] [seed]

#include "Vr3d_face.h"
#include "Vr3d_face___024root.h"
#include "verilated.h"
#include "r3d_fp.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

using namespace r3d;

struct poly { int32_t v[3], n[3]; uint32_t np; int mb; bool step15; };
#ifndef FACE_KZ2
#define FACE_KZ2 0x3f800000u
#endif

static std::deque<uint32_t> g_sums;   // debug: V0..2 N0..2 d per polygon, in write-back order
static float g_c = 0;                 // LOD: the viewport's C
static uint32_t g_b = 0;              // LOD: the last polygon's B
static bool g_step15 = false;         // FACE_STEP15=1: 13.11 positions and u10.14 NP (a run's step is fixed)
static void model(float const T[4][3], poly const &p, bool &gt, bool &lt) {
  fp_unit f; f.ftz = true;
  float const sc = p.step15 ? float(1.0 / 2048.0) : float(1.0 / 128.0);
  float x = float(p.v[0]) * sc, y = float(p.v[1]) * sc, z = float(p.v[2]) * sc;
  float nf[3]; for (int k = 0; k < 3; k++) nf[k] = float(p.n[k]) * float(1.0 / 4194304.0);
  float V[3], N[3];
  for (int j = 0; j < 3; j++) {
    float a = f.mul(x, T[0][j]); a = f.add(f.mul(y, T[1][j]), a); a = f.add(f.mul(z, T[2][j]), a);
    V[j] = f.add(f.mul(1.0f, T[3][j]), a);
    float b = f.mul(nf[0], T[0][j]); b = f.add(f.mul(nf[1], T[1][j]), b); N[j] = f.add(f.mul(nf[2], T[2][j]), b);
  }
  float d = f.mul(V[0], N[0]); d = f.add(f.mul(V[1], N[1]), d); d = f.add(f.mul(V[2], N[2]), d);
  gt = d > 0.0f; lt = d < 0.0f;
  g_b = fbits(f.mul(d, f.mul(float(p.np) * float(p.step15 ? 1.0 / 16384.0 : 1.0 / 4096.0), g_c)));   // u10.14 from Step 1.5
  for (int j = 0; j < 3; j++) g_sums.push_back(fbits(N[j]));
  for (int j = 0; j < 3; j++) g_sums.push_back(fbits(V[j]));
  g_sums.push_back(fbits(d));
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  g_step15 = std::getenv("FACE_STEP15") && std::atoi(std::getenv("FACE_STEP15"));
  long const npoly = argc > 1 ? std::atol(argv[1]) : 200000;
  std::mt19937 rng(argc > 2 ? unsigned(std::atoi(argv[2])) : 1u);
  auto rf = [&](int lo, int hi) {   // a random float with exponent in [lo, hi], a zero sometimes
    if (rng() % 16 == 0) return 0.0f;
    int e = lo + int(rng() % unsigned(hi - lo + 1));
    return std::ldexp(1.0f + float(rng() % 8388608) / 8388608.0f, e) * ((rng() & 1) ? -1.0f : 1.0f);
  };
  auto ri = [&](int bits) { int32_t m = int32_t(rng() % (1u << bits)); return (rng() & 1) ? -m : m; };
  Vr3d_face *t = new Vr3d_face;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; t->t_we = 0; t->t_clr = 0; t->t_set = 0; t->in_we = 0; t->in_commit = 0; t->step15 = g_step15;
  tick(); tick(); t->rst = 0;
  float T[2][4][3] = {};
  std::deque<std::pair<bool, bool>> expect;
  std::deque<uint32_t> bexp;
  uint64_t bad = 0, got = 0, nt = 0, ngt = 0, nlt = 0, bgot = 0, bbad = 0, cloads = 0;
  int const nwords = 6;
#ifdef FACE_LOD
  int const nw = 7;
  t->c_we = 0;
#else
  int const nw = nwords;
#endif
  long sent = 0;
  int mb = 0;              // buffer of the polygons being sent
  int left = 0;            // polygons left in this model
  // input state machines
  int istate = 0, iword = 0; poly cur{};
  int tstate = 0, tidx = 0;  // T load of buffer 1 - mb, then a switch
  bool pending_switch = false;
  bool drain = false;   // LOD: waiting for the unit to empty, then a new C
  uint64_t cyc = 0;
  // first matrix
  auto newT = [&](float (&m)[4][3]) {
    int s = int(rng() % 40) - 20;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i][j] = rf(s - 12, s);
    for (int j = 0; j < 3; j++) m[3][j] = rf(s - 10, s + 20);
  };
  // load buffer 0 directly through the port, then set it
  newT(T[0]);
  for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) {
    t->t_we = 1; t->t_buf = 0; t->t_row = i; t->t_col = j; t->t_d = fbits(T[0][i][j]); tick();
  }
  t->t_we = 0; t->t_set = 1; t->t_buf = 0; tick(); t->t_set = 0;
#ifdef FACE_LOD
  {
    fp_unit f; f.ftz = true;
    float const nc = -1.5f;
    t->c_we = 1; t->c_d = fbits(nc); tick(); t->c_we = 0;
    g_c = (FACE_KZ2 == 0x3f800000u) ? nc : f.mul(bitsf(FACE_KZ2), nc);
    for (int k = 0; k < 20; k++) tick();
  }
#endif
  left = 1 + int(rng() % 12);
  while ((sent < npoly || !expect.empty()) && cyc < uint64_t(npoly) * 200 + 100000) {
    cyc++;
    // drive inputs (combinational from state, before the edge)
    t->in_we = 0; t->in_commit = 0; t->t_we = 0; t->t_clr = 0; t->t_set = 0;
    t->eval();
    // matrix: load the other buffer while its polygons are gone, then switch models
    if (tstate == 0 && left == 0 && istate == 0) {
      int nb = 1 - mb;
      if (!((t->mb_busy >> nb) & 1)) { t->t_clr = 1; t->t_buf = nb; newT(T[nb]); tstate = 1; tidx = 0; }
    } else if (tstate == 1) {
      if (rng() % 4) {
        int nb = 1 - mb;
        t->t_we = 1; t->t_buf = nb; t->t_row = tidx / 3; t->t_col = tidx % 3; t->t_d = fbits(T[nb][tidx / 3][tidx % 3]);
        t->eval();
        if (!t->t_hold) { tidx++; if (tidx == 12) tstate = 2; }
      }
    } else if (tstate == 2) {
      t->t_set = 1; t->t_buf = 1 - mb; mb = 1 - mb; left = 1 + int(rng() % 12); tstate = 0;
#ifdef FACE_LOD
      drain = rng() % 3 == 0;
#endif
    }
    // polygon words (none while draining for a new C)
    if (istate == 0 && sent < npoly && left > 0 && tstate == 0 && !drain) {
      if (t->in_ready && rng() % 3 == 0) {
        cur.step15 = g_step15;
        int sc = int(rng() % 23);
        for (int k = 0; k < 3; k++) { cur.v[k] = ri(1 + sc); cur.n[k] = ri(23); }
        if (rng() % 8 == 0) cur.n[rng() % 3] = 0;
        cur.np = rng() % 6 == 0 ? 0u : (rng() & 0xffffffu) >> (rng() % 24);
        cur.mb = mb; istate = 1; iword = 0;
      }
    } else if (istate == 1) {
      if (rng() % 4) {
        t->in_we = 1; t->in_word = iword < nwords ? iword : 7;
        t->in_d = uint32_t(iword < 3 ? cur.v[iword] : iword < 6 ? cur.n[iword - 3] : int32_t(cur.np)) & 0xffffff;
        t->eval();
        if (!t->in_hold) { iword++; if (iword == nw) istate = 2; }
      }
    } else if (istate == 2) {
      t->in_commit = 1; t->in_mb = cur.mb;
      bool g, l; model(T[cur.mb], cur, g, l); expect.push_back({g, l}); ngt += g; nlt += l;
      bexp.push_back(g_b);
      sent++; left--; istate = 0;
    }
#ifdef FACE_LOD
    t->c_we = 0;
    // a new viewport: C changes while nothing is in the unit
    if (drain && istate == 0 && t->idle) {
      drain = false;
      fp_unit f; f.ftz = true;
      float const nc = -rf(-6, 6);
      t->c_we = 1; t->c_d = fbits(nc);
      g_c = (FACE_KZ2 == 0x3f800000u) ? nc : f.mul(bitsf(FACE_KZ2), nc);
      cloads++;
    }
#endif
    t->eval();
    tick();
    if (std::getenv("FACE_DBG")) {
      static std::deque<std::pair<char, uint32_t>> rs;
      auto *rp = t->rootp;
      if (rp->r3d_face__DOT__wn) rs.push_back({'N', rp->r3d_face__DOT__sum});
      if (rp->r3d_face__DOT__wv) rs.push_back({'V', rp->r3d_face__DOT__sum});
      if (rp->r3d_face__DOT__wd) rs.push_back({'D', rp->r3d_face__DOT__sum});
      (void)rs;
    }
    if (t->v_valid) {
      got++;
      if (expect.empty()) { bad++; continue; }
      auto e = expect.front(); expect.pop_front();
      if (e.first != bool(t->v_gt) || e.second != bool(t->v_lt)) { if (bad < 10) std::printf("poly %lu: model %d%d rtl %d%d\n", (unsigned long)got, e.first, e.second, t->v_gt, t->v_lt); bad++; }
    }
#ifdef FACE_LOD
    if (t->b_valid) {
      bgot++;
      if (bexp.empty()) { bbad++; continue; }
      uint32_t const e = bexp.front(); bexp.pop_front();
      if (e != t->b_word) { if (bbad < 10) std::printf("poly %lu: B model %08X rtl %08X\n", (unsigned long)bgot, e, t->b_word); bbad++; }
    }
#endif
    nt++;
  }
  std::printf("r3d_face: %ld polygons, %lu verdicts, %lu wrong, %lu left over; %lu facing away, %lu towards; %.1f cycles a polygon; subnormal events %u\n",
              sent, (unsigned long)got, (unsigned long)bad, (unsigned long)expect.size(), (unsigned long)ngt, (unsigned long)nlt,
              double(cyc) / double(sent ? sent : 1), t->rootp->r3d_face__DOT__subn_cnt);
#ifdef FACE_LOD
  // the last B products after the last verdict
  for (int k = 0; k < 200 && !bexp.empty(); k++) {
    tick();
    if (t->b_valid) {
      bgot++;
      uint32_t const e = bexp.front(); bexp.pop_front();
      if (e != t->b_word) { if (bbad < 10) std::printf("poly %lu: B model %08X rtl %08X\n", (unsigned long)bgot, e, t->b_word); bbad++; }
    }
  }
  std::printf("  LOD (KZ2 %08X): %lu B words, %lu wrong, %lu left over; %lu C loads\n", FACE_KZ2, (unsigned long)bgot, (unsigned long)bbad,
              (unsigned long)bexp.size(), (unsigned long)cloads);
  bad += bbad + bexp.size();
  if (bgot != uint64_t(sent)) bad++;
#endif
  delete t;
  return (bad == 0 && expect.empty() && got == uint64_t(sent)) ? 0 : 1;
}
