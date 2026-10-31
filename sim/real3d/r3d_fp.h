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
// FP32 arithmetic for the Real3D geometry model, one function per hardware
// operation. Every operation is IEEE-754 binary32, round to nearest even, so
// each one is what the shared fp_mul / fp_add units of my Model 1 and Model 2
// cores produce (bit-exact against host float on their benches, SUBNORM=1).
//
// Host float is used for mul, add and the correctly rounded divide and square
// root (these match a correctly rounded RTL unit by definition). The
// reciprocal is modelled bit by bit, the way the RTL computes it, because it
// is the one unit that is not a plain IEEE operation: a 64-entry seed, two
// fixed-point Newton steps and a remainder check that makes it correctly
// rounded. r3d_fp_selftest() checks it against 1.0f / x on every mantissa.
//
// FTZ mode (flush subnormal inputs and results to signed zero) models the
// cheaper units with SUBNORM=0; the default keeps subnormals, as MAME does.

#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <initializer_list>

namespace r3d {

inline uint32_t fbits(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
inline float bitsf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

struct fp_counts {
  uint64_t mul = 0, add = 0, recip = 0, div = 0, sqrt = 0, half = 0, cmp = 0;
};

struct fp_unit {
  bool ftz = false;
  fp_counts n;

  float flush(float x) const {
    if (ftz && std::fpclassify(x) == FP_SUBNORMAL) return bitsf(fbits(x) & 0x80000000u);
    return x;
  }
  float mul(float a, float b) { n.mul++; return flush(flush(a) * flush(b)); }
  float add(float a, float b) { n.add++; return flush(flush(a) + flush(b)); }
  float sub(float a, float b) { n.add++; return flush(flush(a) - flush(b)); }
  float div(float a, float b) { n.div++; return flush(flush(a) / flush(b)); }
  float sqrt(float a) { n.sqrt++; return flush(std::sqrt(flush(a))); }
  // x * 0.5 exactly as IEEE does it: an exponent decrement, or a rounded right
  // shift once the result is subnormal. No multiplier.
  float half(float a) { n.half++; return flush(flush(a) * 0.5f); }
  float recip(float a);
};

// ---- reciprocal, bit-level ------------------------------------------------
//
// Mantissa M in [2^23, 2^24) (value m = M / 2^23). Seed from the top 6
// fraction bits, Newton r' = r (2 - m r) twice in 30-bit fixed point, then
// Q0 = an estimate of 2^47 / M within one unit, and a remainder test picks
// Q in {Q0 - 1, Q0, Q0 + 1} such that M (2Q - 1) <= 2^48 < M (2Q + 1), which
// is round to nearest (no ties exist for M != 2^23).

struct recip_table {
  uint32_t seed[64];  // 1/m at the centre of each interval, Q1.10
  recip_table() {
    for (int i = 0; i < 64; i++) {
      double const mid = 1.0 + (i + 0.5) / 64.0;
      seed[i] = uint32_t(std::lround(1024.0 / mid));
    }
  }
};

inline uint32_t recip_mantissa(uint32_t M, uint32_t *q0_out = nullptr) {
  static recip_table const t;
  // r in Q0.30 (value r / 2^30), m in Q1.23
  uint64_t r = uint64_t(t.seed[(M >> 17) & 63]) << 20;
  for (int it = 0; it < 2; it++) {
    uint64_t const mr = (uint64_t(M) * r) >> 23;          // m*r, Q.30
    uint64_t const two = uint64_t(2) << 30;
    uint64_t const e = two - mr;                           // 2 - m r, Q.30
    r = (r * e) >> 30;
  }
  // 1/m = r / 2^30 ; 2^47 / M = 2^24 / m = r / 2^6
  uint32_t q0 = uint32_t(r >> 6);
  if (q0_out) *q0_out = q0;
  uint64_t const two48 = uint64_t(1) << 48;
  // at most one step either way
  if (uint64_t(M) * (2 * uint64_t(q0) + 1) <= two48) q0++;
  else if (uint64_t(M) * (2 * uint64_t(q0) - 1) > two48) q0--;
  return q0;
}

inline float fp_unit::recip(float a) {
  n.recip++;
  a = flush(a);
  uint32_t const u = fbits(a);
  uint32_t const s = u & 0x80000000u;
  int e = int((u >> 23) & 0xff);
  uint32_t f = u & 0x7fffff;
  if (e == 0xff) return f ? bitsf(0x7fc00000u) : bitsf(s);   // NaN, 1/inf = 0
  if (e == 0 && f == 0) return bitsf(s | 0x7f800000u);       // 1/0 = inf
  if (e == 0) {                                              // subnormal input
    while (!(f & 0x800000)) { f <<= 1; e--; }
    f &= 0x7fffff; e++;
  }
  uint32_t const M = f | 0x800000;
  int re;
  uint32_t Q;
  if (f == 0) { Q = 0x800000; re = 254 - e; }                // exact power of two
  else { Q = recip_mantissa(M); re = 253 - e; }              // result in (0.5, 1)
  if (Q == 0x1000000) { Q >>= 1; re++; }
  if (re >= 255) return bitsf(s | 0x7f800000u);
  if (re <= 0) {
    // subnormal result (|w| > 2^125, never seen after the W clip): the host
    // divide stands in; the RTL flushes it (r3d_recip.sv)
    return flush(1.0f / a);
  }
  return flush(bitsf(s | (uint32_t(re) << 23) | (Q & 0x7fffff)));
}

// Exhaustive mantissa test, plus exponent sweep. Returns mismatches.
inline uint64_t r3d_fp_selftest(uint64_t *worst_q0_err = nullptr) {
  fp_unit u;
  uint64_t bad = 0, worst = 0;
  for (uint32_t f = 0; f < 0x800000; f++) {
    uint32_t q0;
    if (f) {
      uint32_t const q = recip_mantissa(f | 0x800000, &q0);
      int64_t const d = int64_t(q) - int64_t(q0);
      if (uint64_t(d < 0 ? -d : d) > worst) worst = uint64_t(d < 0 ? -d : d);
      (void)q;
    }
    for (int e : {1, 60, 126, 127, 128, 200, 253, 254}) {
      float const x = bitsf((uint32_t(e) << 23) | f);
      if (fbits(u.recip(x)) != fbits(1.0f / x)) bad++;
      if (fbits(u.recip(-x)) != fbits(1.0f / -x)) bad++;
    }
  }
  for (uint32_t f = 1; f < 0x800000; f += 977) {             // subnormal inputs
    float const x = bitsf(f);
    if (fbits(u.recip(x)) != fbits(1.0f / x)) bad++;
  }
  if (worst_q0_err) *worst_q0_err = worst;
  return bad;
}

} // namespace r3d
