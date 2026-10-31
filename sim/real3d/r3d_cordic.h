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
// Integer model of rtl/real3d/r3d_setup_cr.sv: MAME's frustum half-angle
// tangent, tanf((float)atan2(y, x) * 0.5f), from fixed-point CORDIC.
//
//   1. x, y aligned to a common exponent, the larger at bit 60 (Q2.61).
//   2. Vectoring CORDIC, 62 steps: z = atan2(y, x), Q3.60 radians
//      (x < 0 starts from +-pi with x, y negated).
//   3. z rounded to FP32 (nearest even, the bits below as sticky); halved.
//   4. Rotation CORDIC, 62 steps, from 1/K: cos and sin of the half angle.
//   5. tan = sin / cos by 64 restoring steps (Q2.62); rounded to FP32.
//
// Every step is integer, as the RTL does it. Steps 3 and 5 are correctly
// rounded whenever the ~2^-54 error of the iterations does not straddle a
// rounding boundary; sim/real3d/tb_r3d_cr.cpp measures that against libm.
// Range: finite inputs, |tan| < 4 (frustum half-angles); flags otherwise.

#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace r3d {

struct cordic_tables {
  int64_t atan_t[64];
  int64_t kinv;          // 1/K, Q3.60
  cordic_tables() {
    long double k = 1.0L;
    for (int i = 0; i < 64; i++) {
      atan_t[i] = int64_t(std::llroundl(std::atanl(std::ldexpl(1.0L, -i)) * std::ldexpl(1.0L, 60)));
      k *= std::sqrtl(1.0L + std::ldexpl(1.0L, -2 * i));
    }
    kinv = int64_t(std::llroundl(std::ldexpl(1.0L, 60) / k));
  }
};

inline uint32_t cr_round_fixed(bool neg, uint64_t mag, int frac_bits, bool &bad) {
  // mag * 2^-frac_bits rounded to FP32 (nearest even)
  if (mag == 0) return neg ? 0x80000000u : 0;
  int msb = 63 - __builtin_clzll(mag);
  int e = msb - frac_bits;                 // unbiased exponent
  uint64_t m;
  bool rbit, sticky;
  if (msb >= 23) {
    int sh = msb - 23;
    m = mag >> sh;
    rbit = sh > 0 ? (mag >> (sh - 1)) & 1 : 0;
    sticky = sh > 1 ? (mag & ((uint64_t(1) << (sh - 1)) - 1)) != 0 : false;
  } else {
    m = mag << (23 - msb); rbit = false; sticky = false;
  }
  if (rbit && (sticky || (m & 1))) m++;
  if (m >> 24) { m >>= 1; e++; }
  if (e + 127 <= 0 || e + 127 >= 255) { bad = true; return 0; }
  return (neg ? 0x80000000u : 0) | (uint32_t(e + 127) << 23) | uint32_t(m & 0x7fffff);
}

// returns tanf((float)atan2(y, x) * 0.5f); bad set when outside the range
inline uint32_t cr_half_tan(uint32_t yb, uint32_t xb, bool &bad) {
  static cordic_tables const T;
  bad = false;
  auto unpack = [](uint32_t b, int &e, int64_t &m) {
    int const ex = int((b >> 23) & 0xff);
    if (ex == 0) { e = -200; m = 0; return; }            // zero / subnormal: zero
    e = ex - 127; m = int64_t((b & 0x7fffff) | 0x800000);
  };
  if (((xb >> 23) & 0xff) == 0xff || ((yb >> 23) & 0xff) == 0xff) { bad = true; return 0; }
  int ex, ey; int64_t mx, my;
  unpack(xb, ex, mx); unpack(yb, ey, my);
  bool const xneg = xb >> 31, yneg = yb >> 31;
  // ---- 1. align: the larger magnitude's leading one at bit 60
  int const emax = std::max(ex, ey);
  auto place = [&](int64_t m, int e) -> int64_t {
    if (m == 0) return 0;
    int const sh = 37 - (emax - e);                     // 23 + 37 = 60
    if (sh >= 0) return m << sh;
    if (sh <= -63) return 0;
    return m >> (-sh);
  };
  int64_t X = place(mx, ex), Y = place(my, ey);
  if (xneg) X = -X;
  if (yneg) Y = -Y;
  // ---- 2. vectoring
  int64_t const PI60 = int64_t(std::llroundl(3.14159265358979323846264338327950288L * std::ldexpl(1.0L, 60)));
  int64_t Z = 0;
  if (Y == 0 && !xneg) {
    return yneg ? 0x80000000u : 0;                      // atan2(+-0, +x) = +-0: tan = +-0
  }
  if (X == 0 && Y == 0) {
    Z = yneg ? -PI60 : PI60;                            // atan2(+-0, -0) = +-pi
  } else {
    if (xneg) { Z = yneg ? -PI60 : PI60; X = -X; Y = -Y; }
    for (int i = 0; i < 62; i++) {
      int64_t const xs = X >> i, ys = Y >> i;
      if (Y >= 0) { X = X + ys; Y = Y - xs; Z = Z + T.atan_t[i]; }
      else        { X = X - ys; Y = Y + xs; Z = Z - T.atan_t[i]; }
    }
  }
  // ---- 3. round to FP32, halve
  bool const zneg = Z < 0;
  uint64_t const zm = uint64_t(zneg ? -Z : Z);
  uint32_t const thf = cr_round_fixed(zneg, zm, 60, bad);
  if (bad) return 0;
  // h = thf / 2 in Q3.60
  int const he = int((thf >> 23) & 0xff) - 127 - 1;
  int64_t const hm = int64_t((thf & 0x7fffff) | 0x800000);
  int const hsh = 60 + he - 23;
  int64_t H = hsh >= 0 ? (hm << hsh) : (hsh <= -63 ? 0 : (hm >> (-hsh)));
  if (thf >> 31) H = -H;
  if ((thf & 0x7fffffff) == 0) return thf;               // tan(+-0)
  // ---- 4. rotation from (1/K, 0)
  int64_t C = T.kinv, S = 0, A = H;
  for (int i = 0; i < 62; i++) {
    int64_t const cs = C >> i, ss = S >> i;
    if (A >= 0) { C = C - ss; S = S + cs; A = A - T.atan_t[i]; }
    else        { C = C + ss; S = S - cs; A = A + T.atan_t[i]; }
  }
  // ---- 5. tan = |S| / C, 64 restoring steps -> Q2.62
  if (C <= 0) { bad = true; return 0; }
  bool const tneg = S < 0;
  uint64_t const num = uint64_t(tneg ? -S : S), den = uint64_t(C);
  if (num >= 4 * den) { bad = true; return 0; }
  // q = floor(num * 2^62 / den), the remainder as sticky (the RTL: 64 restoring steps)
  unsigned __int128 const n = (unsigned __int128)num << 62;
  uint64_t q = uint64_t(n / den);
  unsigned __int128 const r = n % den;
  // fold the remainder into the lowest bit as sticky
  if (r != 0) q |= 1;
  uint32_t const out = cr_round_fixed(tneg, q, 62, bad);
  return out;
}

} // namespace r3d
