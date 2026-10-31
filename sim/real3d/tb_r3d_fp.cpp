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
// r3d_fp_mul / r3d_fp_add (SUBNORM=1 and 0) and r3d_recip (divide and
// reciprocal) against host float: random operands biased to edges (zeros of
// both signs, subnormals, near overflow, cancellation, infinities, NaN). NaN
// results compare as NaN. The reciprocal is also run over every mantissa at
// several exponents. SUBNORM=0 and the divide are checked wherever no operand
// and no exact result is subnormal (the flush-to-zero range); the rest are
// counted as outside it.

#include "Vtb_r3d_fp_top.h"
#include "verilated.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <random>

static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long n = argc > 1 ? std::atol(argv[1]) : 20000000;
  bool full_recip = argc > 2 ? std::atoi(argv[2]) : 1;
  Vtb_r3d_fp_top *t = new Vtb_r3d_fp_top;
  std::mt19937_64 rng(12345);
  auto rnd = [&]() -> uint32_t {
    uint32_t const k = rng() % 16;
    uint32_t r = uint32_t(rng());
    switch (k) {
      case 0: return r & 0x80000000u;                       // +-0
      case 1: return (r & 0x80000000u) | (r & 0x7fffff);    // subnormal
      case 2: return (r & 0x80000000u) | 0x7f800000u;       // inf
      case 3: return 0x7fc00000u;                           // NaN
      case 4: return (r & 0x807fffffu) | (uint32_t(1 + rng() % 4) << 23);   // tiny normal
      case 5: return (r & 0x807fffffu) | (uint32_t(250 + rng() % 5) << 23); // huge
      default: return r & 0xbfffffffu ? r : r;              // anything
    }
  };
  struct q { uint32_t a, b, kind; };
  std::deque<q> pend_m, pend_a, pend_r;
  uint64_t bad_mz = 0, bad_az = 0, out_mz = 0, out_az = 0, nd = 0, bad_d = 0, out_d = 0, bad_tiny = 0;
  auto sub_ = [](uint32_t x) { return ((x >> 23) & 0xff) == 0 && (x & 0x7fffff); };
  auto tinyres = [&](float x) { return std::fpclassify(x) == FP_SUBNORMAL; };
  uint64_t bad_m = 0, bad_a = 0, bad_r = 0, nm = 0, na = 0, nr = 0, out_r = 0;
  auto same = [](uint32_t x, uint32_t y) { return x == y || (std::isnan(bf(x)) && std::isnan(bf(y))); };
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; tick(); tick(); t->rst = 0;
  uint32_t rf = 0, re = 0; int rexp[] = {1, 60, 126, 127, 128, 200, 252};  // every mantissa at each
  long const nsweep = full_recip ? 0x800000L * 7 : 0;
  long total = n + nsweep + (full_recip ? 0x800000L * 2 : 0);   // then every divisor mantissa twice
  for (long i = 0; i < total + 64; i++) {
    bool const feed = i < total;
    uint32_t a = rnd(), b = rnd();
    // cancellation and equal-magnitude cases
    if ((rng() & 7) == 0) b = a ^ 0x80000000u;
    if ((rng() & 15) == 0) b = (a & 0xff800000u) | (rng() & 0x7fffff);
    t->m_valid = feed; t->m_a = a; t->m_b = b;
    t->a_valid = feed; t->a_a = a ^ 0x5a5a; t->a_b = b;
    uint32_t ra;
    if (i < n) ra = rnd();
    else if (i < n + nsweep) { ra = (uint32_t(rexp[re]) << 23) | rf; if (++rf == 0x800000) { rf = 0; re++; } if (re >= 7) re = 0; }
    else ra = 0;
    // every fourth reciprocal slot is a divide ra2 / ra
    bool dv = i < n && (i & 3) == 1;
    uint32_t ra2 = rnd();
    if (i >= n + nsweep) {
      dv = true;
      long const k = i - n - nsweep;
      ra = (uint32_t(k & 0x7fffff)) | (uint32_t(k < 0x800000 ? 100 : 150) << 23);
      ra2 = (uint32_t(rng()) & 0x807fffffu) | (uint32_t(64 + rng() % 128) << 23);
    }
    t->r_valid = feed; t->r_x = ra; t->r_div = dv; t->r_a = ra2;
    t->eval();
    if (feed) { pend_m.push_back({a, b, 0}); pend_a.push_back({a ^ 0x5a5a, b, 0}); pend_r.push_back({ra, ra2, uint32_t(dv)}); }
    tick();
    if (t->m_ovalid) {
      q const p = pend_m.front(); float const h = bf(p.a) * bf(p.b);
      if (sub_(p.a) || sub_(p.b) || tinyres(h) || (h != 0 && std::fabs(h) < 1.2e-38f) || (h == 0 && bf(p.a) != 0 && bf(p.b) != 0)) out_mz++;
      else if (!same(t->mz_res, fb(h))) { if (bad_mz++ < 5) std::printf("mulz %08X*%08X rtl %08X host %08X\n", p.a, p.b, t->mz_res, fb(h)); }
    }
    if (t->a_ovalid) {
      q const p = pend_a.front(); float const h = bf(p.a) + bf(p.b);
      if (sub_(p.a) || sub_(p.b) || tinyres(h)) out_az++;
      else if (!same(t->az_res, fb(h))) { if (bad_az++ < 5) std::printf("addz %08X+%08X rtl %08X host %08X\n", p.a, p.b, t->az_res, fb(h)); }
    }
    if (t->m_ovalid) { q const p = pend_m.front(); pend_m.pop_front(); nm++; if (!same(t->m_res, fb(bf(p.a) * bf(p.b)))) { if (bad_m++ < 5) std::printf("mul %08X*%08X rtl %08X host %08X\n", p.a, p.b, t->m_res, fb(bf(p.a) * bf(p.b))); } }
    if (t->a_ovalid) { q const p = pend_a.front(); pend_a.pop_front(); na++; if (!same(t->a_res, fb(bf(p.a) + bf(p.b)))) { if (bad_a++ < 5) std::printf("add %08X+%08X rtl %08X host %08X\n", p.a, p.b, t->a_res, fb(bf(p.a) + bf(p.b))); } }
    if (t->r_ovalid) {
      q const p = pend_r.front(); pend_r.pop_front();
      uint32_t const e = (p.a >> 23) & 0xff;
      if (p.kind) {
        float const h = bf(p.b) / bf(p.a);
        bool const outside = sub_(p.a) || sub_(p.b) || tinyres(h) || (h == 0 && bf(p.b) != 0 && !std::isinf(bf(p.a)));
        if (outside) { out_d++; if (!t->r_tiny && (sub_(p.a) || sub_(p.b) || tinyres(h))) bad_tiny++; }
        else { nd++; if (t->r_tiny) bad_tiny++; if (!same(t->r_res, fb(h))) { if (bad_d++ < 5) std::printf("div %08X/%08X rtl %08X host %08X\n", p.b, p.a, t->r_res, fb(h)); } }
      } else
      // documented: subnormal input, or a result below 2^-126 (|x| >= 2^126)
      if ((e == 0 && (p.a & 0x7fffff)) || (e >= 253 && e != 255)) { out_r++; }
      else { nr++; if (!same(t->r_res, fb(1.0f / bf(p.a)))) { if (bad_r++ < 5) std::printf("recip %08X rtl %08X host %08X\n", p.a, t->r_res, fb(1.0f / bf(p.a))); } }
    }
  }
  std::printf("flush-to-zero: fp_mul %llu wrong (%llu outside), fp_add %llu wrong (%llu outside); divide %llu ops, %llu wrong (%llu outside), tiny flag wrong %llu\n",
              (unsigned long long)bad_mz, (unsigned long long)out_mz, (unsigned long long)bad_az, (unsigned long long)out_az,
              (unsigned long long)nd, (unsigned long long)bad_d, (unsigned long long)out_d, (unsigned long long)bad_tiny);
  std::printf("fp_mul %llu ops, %llu wrong; fp_add %llu ops, %llu wrong; recip %llu ops, %llu wrong (%llu outside its documented range)\n",
              (unsigned long long)nm, (unsigned long long)bad_m, (unsigned long long)na, (unsigned long long)bad_a,
              (unsigned long long)nr, (unsigned long long)bad_r, (unsigned long long)out_r);
  delete t;
  return (bad_m || bad_a || bad_r || bad_mz || bad_az || bad_d || bad_tiny) ? 1 : 0;
}
