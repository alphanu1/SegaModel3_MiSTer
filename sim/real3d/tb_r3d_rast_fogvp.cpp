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
// r3d_rast_fogvp against make_fog (r3d_rast_rtl.cpp) on random viewport
// records: the density and kz over the whole FP32 range (zeros, subnormals,
// infinities, NaN, 99999 and its neighbours, kz 1.0 and MAME's values), any
// colour, ambient and start. K is compared as bits, S after the clamp to
// [-4096, 1024] (which lin cannot tell from the full value), fc exactly.
// Records whose density, kz or quotient is subnormal are counted apart (the
// RTL flushes them; fog's P is 0 either way).
//
//   Vr3d_rast_fogvp [N] [SEED]

#include "Vr3d_rast_fogvp.h"
#include "verilated.h"
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

static float u2f(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
static uint32_t f2u(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char **argv) {
  long n = argc > 1 ? std::atol(argv[1]) : 1000000;
  unsigned seed = argc > 2 ? unsigned(std::atoi(argv[2])) : 1;
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_fogvp t;
  std::mt19937 g(seed);
  auto rf = [&]() -> uint32_t {
    switch (g() % 12) {
    case 0: return 0;
    case 1: return 0x80000000u;
    case 2: return (g() & 0x80000000u) | (g() & 0x7fffff);           // subnormal
    case 3: return (g() & 0x80000000u) | 0x7f800000u | (g() % 3 ? 0 : (g() & 0x7fffff));
    case 4: return 0x47C34F80u + (g() % 5) - 2;                        // 99999 and around
    case 5: return 0x3f800000u;                                        // 1.0
    case 6: return 0x408000a8u;                                        // MAME's kz squared, a typical value
    case 7: return 0x3c000000u + (g() & 0x3ffffff);                    // small densities
    default: return g();
    }
  };
  t.rst = 1; t.i_valid = 0;
  for (int i = 0; i < 4; i++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
  t.rst = 0;
  uint64_t bad = 0, sub = 0, nz = 0;
  for (long i = 0; i < n; i++) {
    uint32_t w[6] = {0, g(), g() & 0xffffff, rf(), g(), rf()};
    if (g() % 4 == 0) w[4] = (w[4] & 0xffff0000u) | (g() % 2 ? (g() & 0x7ff) : uint32_t(-int(g() & 0x7ff)) & 0xffff);
    // the model (make_fog)
    uint32_t const amb = (w[4] >> 16) & 255;
    uint32_t fc = 0;
    for (int c = 0; c < 3; c++) {
      uint32_t const ch = (w[2] >> (16 - 8 * c)) & 255, p = ch * amb;
      fc |= ((p + (p >> 8) + 128) >> 8) << (16 - 8 * c);
    }
    float dens = std::fabs(u2f(w[3]));
    if (!(dens < 99999.0f)) dens = 0.0f;
    int32_t const st16 = int16_t(w[4] & 0xffff);
    int32_t S = (st16 * 16448 + 2048) >> 12;
    S = S < -4096 ? -4096 : S > 1024 ? 1024 : S;
    float const kz = u2f(w[5]);
    float K = (kz > 0.0f && std::isfinite(kz)) ? dens / kz : 0.0f;
    if (!std::isfinite(K)) K = 0.0f;
    bool const subn = std::fpclassify(dens) == FP_SUBNORMAL || std::fpclassify(kz) == FP_SUBNORMAL || std::fpclassify(K) == FP_SUBNORMAL;
    // the RTL
    for (int k = 0; k < 6; k++) {
      t.i_valid = 1; t.i_data = w[k]; t.i_last = k == 5;
      for (;;) { t.clk = 0; t.eval(); bool const took = t.i_ready; t.clk = 1; t.eval(); if (took) break; }
    }
    t.i_valid = 0;
    int guard = 0;
    while (!t.o_valid && guard++ < 100) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
    int32_t const s = int32_t(int16_t(uint16_t(t.o_s << 2))) >> 2;
    bool const ok = t.o_k == f2u(K) && s == S && t.o_fc == fc;
    nz += K != 0.0f;
    if (!ok) {
      if (subn) sub++;
      else if (bad++ < 10) std::printf("  dens %08X kz %08X n25 %08X: rtl K %08X S %d fc %06X, model K %08X S %d fc %06X\n",
                                      w[3], w[5], w[4], t.o_k, s, t.o_fc, f2u(K), S, fc);
    }
  }
  std::printf("r3d_rast_fogvp: %ld records (%" PRIu64 " with K > 0), %" PRIu64 " wrong, %" PRIu64 " differ only with a subnormal operand or K\n", n, nz, bad, sub);
  return bad ? 1 : 0;
}
