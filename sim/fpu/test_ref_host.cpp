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
// The FPU reference model against the host's IEEE arithmetic.
//
// Only where the two must agree: results and FI (the host's inexact flag) in
// every rounding mode. The first half uses normal-range operands and results;
// the second any finite operand, denormals and overflows included -- the
// values and the inexact flag still agree, only UX may not (the host detects
// tininess after rounding, the PowerPC before), so UX is not compared. Single-precision
// operations use single operands and host float arithmetic (fmaf for the
// multiply-add), so the host also rounds once. fres is 1.0f / x in float;
// frsqrte is 1.0 / sqrt(x) in double, both in the current mode.
//
//   test_ref_host [count] [seed]

#include "ppc_fpu_ref.h"

#include <cfenv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#pragma STDC FENV_ACCESS ON

using namespace ppcfpu;

namespace {

std::mt19937_64 rng;

uint64_t d2u(double d) { uint64_t u; std::memcpy(&u, &d, 8); return u; }
double u2d(uint64_t u) { double d; std::memcpy(&d, &u, 8); return d; }
uint32_t f2u(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
float u2f(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

// normal-range double with a modest exponent
double rnd_d(int span) {
  uint64_t s = rng() & 1, e = 1023 - span + rng() % (2 * span + 1), f = rng() & 0xFFFFFFFFFFFFFull;
  if (rng() % 4 == 0) f &= ~0xFFFFFFFull;  // short fractions make ties and exact results
  return u2d((s << 63) | (e << 52) | f);
}
float rnd_f(int span) {
  uint32_t s = rng() & 1, e = 127 - span + rng() % (2 * span + 1), f = rng() & 0x7FFFFF;
  if (rng() % 4 == 0) f &= ~0xFFFu;
  return u2f((s << 31) | (e << 23) | f);
}

// any finite double, denormals and extremes weighted up
bool wide = false;
double pick_d(int span) {
  if (!wide) return rnd_d(span);
  uint64_t s = rng() & 1, f = rng() & 0xFFFFFFFFFFFFFull, e;
  switch (rng() % 6) {
  case 0: e = 0; break;
  case 1: e = 1 + rng() % 60; break;
  case 2: e = 2046 - rng() % 60; break;
  case 3: e = 1023 - 540 + rng() % 1080; break;
  default: e = rng() % 2047; break;
  }
  if (rng() % 4 == 0) f &= ~0xFFFFFFFull;
  return u2d((s << 63) | (e << 52) | f);
}
float pick_f(int span) {
  if (!wide) return rnd_f(span);
  uint32_t s = rng() & 1, f = rng() & 0x7FFFFF, e;
  switch (rng() % 5) {
  case 0: e = 0; break;
  case 1: e = 1 + rng() % 30; break;
  case 2: e = 254 - rng() % 30; break;
  default: e = rng() % 255; break;
  }
  return u2f((s << 31) | (e << 23) | f);
}

const int host_mode[4] = {FE_TONEAREST, FE_TOWARDZERO, FE_UPWARD, FE_DOWNWARD};

long fails = 0, checks = 0;

void check(const char *what, uint64_t got, uint64_t want, bool fi_got, bool fi_want, uint64_t a, uint64_t b, uint64_t c, int rn) {
  checks++;
  if (got == want && fi_got == fi_want) return;
  if (++fails <= 20)
    printf("FAIL %-8s rn=%d a=%016llx b=%016llx c=%016llx model=%016llx fi=%d host=%016llx fi=%d\n", what, rn,
           (unsigned long long)a, (unsigned long long)b, (unsigned long long)c, (unsigned long long)got, fi_got,
           (unsigned long long)want, fi_want);
}

bool normal_d(double x) { return std::isnormal(x); }
bool normal_f(float x) { return std::isnormal(x); }

}  // namespace

int main(int argc, char **argv) {
  long n = argc > 1 ? atol(argv[1]) : 1000000;
  rng.seed(argc > 2 ? atol(argv[2]) : 1);

  for (long i = 0; i < 2 * n; i++) {
    wide = i >= n;  // second half: any finite operand, the value and FI must still agree
    int rn = rng() & 3;
    uint32_t fs = rn;
    int k = rng() % 12;
    fesetround(host_mode[rn]);
    feclearexcept(FE_ALL_EXCEPT);
    In in{};
    in.fpscr = fs;
    double h = 0;
    float hf = 0;
    bool sgl = false;
    switch (k) {
    case 0: case 1: case 2: case 3: case 4: {  // double fadd fsub fmul fdiv fmadd
      double a = pick_d(60), b = pick_d(60), c = pick_d(60);
      if (k == 0) { in.op = FADD; h = a + b; }
      if (k == 1) { in.op = FSUB; h = a - b; }
      if (k == 2) { in.op = FMUL; h = a * c; }
      if (k == 3) { in.op = FDIV; h = a / b; }
      if (k == 4) {
        // product and addend of similar size, for cancellation
        if (rng() & 1) b = -(a * c) * (1 + (double(int(rng() % 9) - 4) * 0x1p-50));
        feclearexcept(FE_ALL_EXCEPT);
        in.op = FMADD; h = std::fma(a, c, b);
      }
      in.a = d2u(a); in.b = d2u(b); in.c = d2u(c);
      if (wide ? std::isnan(h) : !normal_d(h)) continue;
      break;
    }
    case 5: case 6: case 7: case 8: {  // single fadds fmuls fdivs fmadds
      float a = pick_f(30), b = pick_f(30), c = pick_f(30);
      sgl = true;
      if (k == 5) { in.op = FADD; hf = a + b; }
      if (k == 6) { in.op = FMUL; hf = a * c; }
      if (k == 7) { in.op = FDIV; hf = a / b; }
      if (k == 8) { in.op = FMSUB; hf = std::fmaf(a, c, -b); }
      in.a = d2u(a); in.b = d2u(b); in.c = d2u(c);
      if (wide ? std::isnan(hf) : !normal_f(hf)) continue;
      h = hf;
      break;
    }
    case 9: {  // frsp
      double b = pick_d(100);
      in.op = FRSP; in.b = d2u(b);
      hf = float(b); h = hf; sgl = true;
      if (wide ? std::isnan(hf) : !normal_f(hf)) continue;
      break;
    }
    case 10: {  // fres (single 1.0f / x) and frsqrte (1.0 / sqrt(x))
      if (rng() & 1) {
        float b = pick_f(30);
        in.op = FRES; in.single = true; in.b = d2u(b);
        hf = 1.0f / b; h = hf; sgl = true;
        if (wide ? std::isnan(hf) : !normal_f(hf)) continue;
      } else {
        double b = std::fabs(pick_d(300));
        in.op = FRSQRTE; in.b = d2u(b);
        h = 1.0 / std::sqrt(b);
        feclearexcept(FE_ALL_EXCEPT);
        volatile double s = std::sqrt(b);
        h = 1.0 / s;  // FI of the second step only
        if (wide ? std::isnan(h) : !normal_d(h)) continue;
      }
      break;
    }
    case 11: {  // fctiw
      double b = rnd_d(28) * ((rng() & 1) ? 1.0 : 0x1p-20);
      in.op = FCTIW; in.b = d2u(b);
      double r = std::nearbyint(b);
      feclearexcept(FE_ALL_EXCEPT);
      double r2 = std::rint(b);  // rint raises inexact
      (void)r;
      if (r2 > 2147483647.0 || r2 < -2147483648.0) continue;
      h = r2;
      break;
    }
    }
    bool fi_host = fetestexcept(FE_INEXACT);
    fesetround(FE_TONEAREST);
    in.single = sgl;
    Out o = exec(in);
    bool fi_model = o.fpscr & FI;
    if (in.op == FCTIW) {
      uint32_t want = uint32_t(int32_t(h));
      check("fctiw", o.frd & 0xFFFFFFFF, want, fi_model, fi_host, in.a, in.b, in.c, rn);
    } else {
      check(op_name[in.op], o.frd, d2u(h), fi_model, fi_host, in.a, in.b, in.c, rn);
    }
    // FX/XX bookkeeping: an inexact result sets XX and FX from a clear FPSCR
    if (fi_model && !((o.fpscr & XX) && (o.fpscr & FX)) && in.op != FRES && in.op != FRSQRTE) {
      if (++fails <= 20) printf("FAIL %s: FI without XX/FX, fpscr %08x\n", op_name[in.op], o.fpscr);
    }
  }

  // lfs / stfs: every normal single round-trips, and lfs agrees with (double)float
  for (long i = 0; i < n / 4; i++) {
    uint32_t w = uint32_t(rng());
    uint32_t e = (w >> 23) & 0xFF;
    if (e == 0xFF && (w & 0x7FFFFF)) continue;  // NaN: the host may quiet
    uint64_t d = lfs(w);
    checks++;
    if (d != d2u(double(u2f(w))) || stfs(d) != w) {
      if (++fails <= 20) printf("FAIL lfs/stfs %08x -> %016llx -> %08x\n", w, (unsigned long long)d, stfs(d));
    }
  }

  printf("%ld checks, %ld failures\n", checks, fails);
  return fails ? 1 : 0;
}
