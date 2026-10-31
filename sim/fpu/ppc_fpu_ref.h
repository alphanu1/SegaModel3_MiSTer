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
// Reference model of the 603e floating-point unit: results and FPSCR.
//
// Written from the 32-bit PowerPC Programming Environments Manual (PEM) and the
// MPC603e User's Manual, independently of the RTL. The RTL computes in a fixed
// window with guard, round and sticky bits; this model computes the exact
// result with MPFR and lets MPFR round it, so a shared misreading has to be
// made twice to pass.
//
// Rules followed, with the manual section they rest on:
//  - one rounding of the infinitely precise result, multiply-add included
//    [PEM 3.3.5, 4.2.2.2, D.2]; fnmadd/fnmsub round, then negate; a NaN result
//    is never negated [PEM 8-75..8-78].
//  - tininess before rounding [PEM 3.3.5, 3.3.6.2.2]; UE=0: UX = tiny and
//    inexact, the result denormalised and rounded; UE=1: UX = tiny, the
//    exponent adjusted by +1536 / +192 [PEM Table 3-16].
//  - overflow on the rounded result with an unbounded exponent; OE=0 gives the
//    Table 3-15 default with FI = XX = 1; OE=1 adjusts by -1536 / -192.
//  - NaN priority frA, frB, frC [PEM 3.3.1.7]; SNaN quieted by setting
//    fraction bit 12; default QNaN 0x7FF8000000000000.
//  - VE=1 invalid and ZE=1 zero divide suppress the write; FPRF unchanged,
//    FR and FI cleared [PEM 3.3.6.1, Table 3-12 read as in the notes, C1].
//
// Where the manuals leave a result undefined the choice is stated at the place
// it is made, marked UNDEFINED. The rule (study D-17): follow Supermodel.
//
// Self-contained: the core's lockstep model (sim/cpu/ppc_iss.h) can include
// this file and call exec() with the FPR images and the FPSCR.

#pragma once
#include <cstdint>
#include <cstring>
#include <mpfr.h>

namespace ppcfpu {

// ------------------------------------------------------------ operations
enum Op : int {
  FADD, FSUB, FMUL, FDIV, FMADD, FMSUB, FNMADD, FNMSUB,
  FRES, FRSQRTE, FSEL, FRSP, FCTIW, FCTIWZ, FCMPU, FCMPO,
  FMR, FNEG, FABS, FNABS,
  MFFS, MTFSF, MTFSFI, MTFSB0, MTFSB1, MCRFS,
  NUM_OPS
};

static const char *const op_name[NUM_OPS] = {
  "fadd", "fsub", "fmul", "fdiv", "fmadd", "fmsub", "fnmadd", "fnmsub",
  "fres", "frsqrte", "fsel", "frsp", "fctiw", "fctiwz", "fcmpu", "fcmpo",
  "fmr", "fneg", "fabs", "fnabs",
  "mffs", "mtfsf", "mtfsfi", "mtfsb0", "mtfsb1", "mcrfs"
};

// ------------------------------------------------------------ FPSCR bits
// LSB-0 masks; IBM bit n is mask 1 << (31 - n) [PEM Table 2-4].
constexpr uint32_t FX = 0x80000000, FEX = 0x40000000, VX = 0x20000000, OX = 0x10000000;
constexpr uint32_t UX = 0x08000000, ZX = 0x04000000, XX = 0x02000000, VXSNAN = 0x01000000;
constexpr uint32_t VXISI = 0x00800000, VXIDI = 0x00400000, VXZDZ = 0x00200000, VXIMZ = 0x00100000;
constexpr uint32_t VXVC = 0x00080000, FR = 0x00040000, FI = 0x00020000;
constexpr uint32_t FPRF_M = 0x0001F000, FPCC_M = 0x0000F000;
constexpr uint32_t VXSOFT = 0x00000400, VXSQRT = 0x00000200, VXCVI = 0x00000100;
constexpr uint32_t VE = 0x80, OE = 0x40, UE = 0x20, ZE = 0x10, XE = 0x08, NI = 0x04;
// exception bits whose 0 -> 1 transition sets FX, and the VX summary's inputs
constexpr uint32_t EXC_BITS = 0x1FF80700;
constexpr uint32_t VX_BITS  = 0x01F80700;

// FPRF codes, C<>=? [PEM Table 2-5]
constexpr uint32_t C_QNAN = 0x11, C_NINF = 0x09, C_NNORM = 0x08, C_NDEN = 0x18, C_NZERO = 0x12;
constexpr uint32_t C_PZERO = 0x02, C_PDEN = 0x14, C_PNORM = 0x04, C_PINF = 0x05;

constexpr uint64_t QNAN_DEFAULT = 0x7FF8000000000000ull;
constexpr uint64_t SIGN64 = 0x8000000000000000ull;

// FEX and VX are always derived [PEM p.2-7]
inline uint32_t summaries(uint32_t f) {
  f &= ~(FEX | VX);
  if (f & VX_BITS) f |= VX;
  uint32_t en = f & 0xF8;  // VE OE UE ZE XE
  if (((f & VX) && (en & VE)) || ((f & OX) && (en & OE)) || ((f & UX) && (en & UE)) ||
      ((f & ZX) && (en & ZE)) || ((f & XX) && (en & XE)))
    f |= FEX;
  return f;
}

// ---------------------------------------------------------------- inputs
struct In {
  Op       op;
  bool     single;   // opcode 59 form (fadds ... fres)
  uint64_t a, b, c;  // frA, frB, frC images
  uint32_t fpscr;
  uint32_t imm;      // mtfsf: FM (8 bits); mtfsfi: crfD << 4 | IMM; mtfsb0/1: crbD; mcrfs: crfS; fcmp: unused
};

struct Out {
  bool     wr;       // frD written
  uint64_t frd;
  uint32_t fpscr;
  bool     crw;      // a CR field is written (fcmpu, fcmpo, mcrfs)
  uint8_t  cr;       // its value (LT GT EQ UN / the FPSCR field)
};

// -------------------------------------------------------------- operands
inline uint32_t ex(uint64_t x) { return (x >> 52) & 0x7FF; }
inline uint64_t fr(uint64_t x) { return x & 0x000FFFFFFFFFFFFFull; }
inline bool is_nan(uint64_t x)  { return ex(x) == 0x7FF && fr(x) != 0; }
inline bool is_snan(uint64_t x) { return is_nan(x) && !((x >> 51) & 1); }
inline bool is_inf(uint64_t x)  { return ex(x) == 0x7FF && fr(x) == 0; }
inline bool is_zero(uint64_t x) { return (x & ~SIGN64) == 0; }
inline bool sgn(uint64_t x)     { return x >> 63; }
inline uint64_t quiet(uint64_t x) { return x | 0x0008000000000000ull; }

// FPRF of a value held in an FPR. For single-precision results the
// denormal/normal split is judged against the single format [PEM 3.3.6]: a
// biased double exponent below 897 is below 2^-126.
inline uint32_t fprf_of(uint64_t x, bool single) {
  bool s = sgn(x);
  uint32_t e = ex(x);
  if (is_nan(x)) return C_QNAN;
  if (is_inf(x)) return s ? C_NINF : C_PINF;
  if (is_zero(x)) return s ? C_NZERO : C_PZERO;
  bool den = (e == 0) || (single && e < 897);
  return den ? (s ? C_NDEN : C_PDEN) : (s ? C_NNORM : C_PNORM);
}

// -------------------------------------------------------- MPFR utilities
struct Mp {
  mpfr_t v;
  explicit Mp(mpfr_prec_t p) { mpfr_init2(v, p); }
  ~Mp() { mpfr_clear(v); }
  Mp(const Mp &) = delete;
  Mp &operator=(const Mp &) = delete;
};

// An FPR image as an exact MPFR value (finite only).
inline void set_fpr(mpfr_t r, uint64_t x) {
  double d;
  std::memcpy(&d, &x, 8);
  mpfr_set_d(r, d, MPFR_RNDN);  // exact: r has at least 53 bits
}

inline mpfr_rnd_t rnd_of(uint32_t rn) {
  static const mpfr_rnd_t m[4] = {MPFR_RNDN, MPFR_RNDZ, MPFR_RNDU, MPFR_RNDD};
  return m[rn & 3];
}

// Pack a finite value of at most 53 significant bits into an FPR image. With
// `den` a value below 2^-1022 is stored as a double denormal (it is then on
// the 2^-1074 grid). Without it the biased exponent is taken modulo 2^11: that
// only matters for an exponent-adjusted result (OE=1 / UE=1) of a
// single-precision instruction whose operands are not single values
// (UNDEFINED, PEM 3.3.4), and the RTL forms the field the same way.
inline uint64_t pack(mpfr_t v, bool den = true) {
  if (mpfr_zero_p(v)) return mpfr_signbit(v) ? SIGN64 : 0;
  bool s = mpfr_signbit(v);
  long E = mpfr_get_exp(v) - 1;  // IEEE exponent of the leading bit
  Mp t(64);
  mpfr_abs(t.v, v, MPFR_RNDN);
  mpfr_mul_2si(t.v, t.v, 52 - E, MPFR_RNDN);  // significand as an integer, 1.xxx * 2^52
  uint64_t sig = mpfr_get_uj(t.v, MPFR_RNDZ);
  uint64_t bits;
  if (den && E < -1022) {
    // double denormal: the value is on the 2^-1074 grid (subnormalised)
    Mp u(64);
    mpfr_abs(u.v, v, MPFR_RNDN);
    mpfr_mul_2si(u.v, u.v, 1074, MPFR_RNDN);
    bits = mpfr_get_uj(u.v, MPFR_RNDZ);
  } else {
    bits = (uint64_t(uint32_t(E + 1023) & 0x7FF) << 52) | (sig & 0x000FFFFFFFFFFFFFull);
  }
  return bits | (s ? SIGN64 : 0);
}

// ------------------------------------------------------------- rounding
struct Fmt { int p, emin, emax, adj; };
static constexpr Fmt FMT_D{53, -1022, 1023, 1536};
static constexpr Fmt FMT_S{24, -126, 127, 192};

struct Rounded {
  uint64_t bits;
  bool fr, fi, ox, ux;
};

// Round the result of `op` to the format. `op(r, rnd)` writes the correctly
// rounded result into r (at r's precision, in the current exponent range) and
// returns MPFR's ternary value. `tiny` is |infinitely precise result| < 2^emin.
// The result is nonzero and finite (zeros are handled by the callers, which
// know the sign rules).
template <class OpF>
Rounded round_fmt(const Fmt &f, OpF op, bool tiny, uint32_t fpscr) {
  Rounded o{0, false, false, false, false};
  mpfr_rnd_t rnd = rnd_of(fpscr);
  bool oe = fpscr & OE, ue = fpscr & UE, ni = fpscr & NI;

  // the rounded result with an unbounded exponent range
  Mp w(f.p);
  int tw = op(w.v, rnd);
  bool neg = mpfr_signbit(w.v);
  long E = mpfr_get_exp(w.v) - 1;
  auto fr_of = [&](int t) { return neg ? (t < 0) : (t > 0); };

  if (E > f.emax) {
    // overflow: the rounded magnitude exceeds the largest finite [PEM 3.3.6.2.1]
    o.ox = true;
    if (oe) {
      mpfr_mul_2si(w.v, w.v, -f.adj, MPFR_RNDN);
      o.bits = pack(w.v, false);
      o.fi = tw != 0;
      o.fr = fr_of(tw);
    } else {
      // Table 3-15: RN -> inf; RZ -> max; RP -> +inf / -max; RM -> +max / -inf
      uint32_t rn = fpscr & 3;
      bool to_inf = (rn == 0) || (rn == 2 && !neg) || (rn == 3 && neg);
      uint64_t mag = to_inf ? 0x7FF0000000000000ull
                            : (f.p == 53 ? 0x7FEFFFFFFFFFFFFFull : 0x47EFFFFFE0000000ull);
      o.bits = mag | (neg ? SIGN64 : 0);
      o.fi = true;
      // UNDEFINED: FR after a disabled overflow [PEM 3.3.6.2.1, D-8; notes C5]. 0.
      o.fr = false;
    }
    return o;
  }

  if (tiny) {
    if (ue) {
      // enabled underflow: normalised, exponent + 1536 / + 192, UX on tininess
      o.ux = true;
      mpfr_mul_2si(w.v, w.v, f.adj, MPFR_RNDN);
      o.bits = pack(w.v, false);
      o.fi = tw != 0;
      o.fr = fr_of(tw);
      return o;
    }
    // disabled: denormalise and round once, to the fixed 2^(emin-p+1) grid.
    // The value is formed at 600 bits toward zero in the default exponent
    // range (its ternary value is the sticky beyond them) and rounded here by
    // hand. MPFR's own subnormal emulation (a reduced emin / emax) needs the
    // operands inside the reduced range too, which a single-precision op on
    // double operands breaks: fdivs of -2^191 by -2^341 came back as garbage.
    Mp x(600), ip(600), fq(600);
    int tx = op(x.v, MPFR_RNDZ);
    mpfr_abs(x.v, x.v, MPFR_RNDN);
    mpfr_mul_2si(x.v, x.v, -(f.emin - f.p + 1), MPFR_RNDN);  // in grid units, exact
    mpfr_trunc(ip.v, x.v);
    mpfr_sub(fq.v, x.v, ip.v, MPFR_RNDN);                     // the fraction, exact
    int ch = mpfr_cmp_d(fq.v, 0.5);
    bool inexact = !mpfr_zero_p(fq.v) || tx != 0;
    bool above = ch > 0 || (ch == 0 && tx != 0), tie = (ch == 0 && tx == 0);
    uint64_t n = mpfr_get_uj(ip.v, MPFR_RNDZ);
    bool up = false;
    switch (fpscr & 3) {
    case 0: up = above || (tie && (n & 1)); break;
    case 1: up = false; break;
    case 2: up = !neg && inexact; break;
    default: up = neg && inexact; break;
    }
    if (up) n++;
    o.fi = inexact;
    o.ux = o.fi;  // tiny and inexact
    o.fr = up;
    // NI=1 (603e 2.3.4.2): a denormalised result is delivered as a zero of the
    // same sign. Flags as for NI=0. PROVISIONAL: the manuals say nothing about
    // FR, FI, UX, XX or FPRF in this mode (notes 3.4).
    if (n == 0 || (ni && n < (uint64_t(1) << (f.p - 1)))) {
      o.bits = neg ? SIGN64 : 0;
      return o;
    }
    Mp r(64);
    mpfr_set_uj_2exp(r.v, n, f.emin - f.p + 1, MPFR_RNDN);
    if (neg) mpfr_neg(r.v, r.v, MPFR_RNDN);
    o.bits = pack(r.v);
    return o;
  }

  o.bits = pack(w.v);
  o.fi = tw != 0;
  o.fr = fr_of(tw);
  return o;
}

// |x| < 2^e
inline bool below_pow2(mpfr_t x, int e) {
  Mp t(2);
  mpfr_set_ui_2exp(t.v, 1, e, MPFR_RNDN);
  return mpfr_cmpabs(x, t.v) < 0;
}

// ------------------------------------------------------- FPSCR updating
// What one instruction does to the FPSCR, before the summaries.
struct Upd {
  uint32_t causes = 0;    // exception bits to set (sticky OR)
  bool     frfi = false;  // FR and FI written
  bool     frv = false, fiv = false;
  int      fprf = -1;     // FPRF written (5 bits), -1 unchanged
  int      fpcc = -1;     // FPCC only (compares)
};

inline uint32_t apply(uint32_t f, const Upd &u) {
  uint32_t rise = u.causes & ~f & EXC_BITS;
  f |= u.causes;
  if (rise) f |= FX;  // the implicit FX rule [PEM Table 2-4]
  if (u.frfi) f = (f & ~(FR | FI)) | (u.frv ? FR : 0) | (u.fiv ? FI : 0);
  if (u.fprf >= 0) f = (f & ~FPRF_M) | (uint32_t(u.fprf) << 12);
  if (u.fpcc >= 0) f = (f & ~FPCC_M) | (uint32_t(u.fpcc & 0xF) << 12);
  return summaries(f);
}

// ------------------------------------------------- loads and stores [D.6, D.7]
// lfs: single -> double, no rounding, no quieting.
inline uint64_t lfs(uint32_t w) {
  uint64_t s = uint64_t(w >> 31) << 63;
  uint32_t e = (w >> 23) & 0xFF, f = w & 0x7FFFFF;
  if (e == 0 && f != 0) {
    int exp = -126;
    uint64_t frac = uint64_t(f) << 29;  // frac[1..52] below the leading bit position 52
    while (!(frac & (1ull << 52))) { frac <<= 1; exp--; }
    return s | (uint64_t(exp + 1023) << 52) | (frac & 0x000FFFFFFFFFFFFFull);
  }
  // normalised, or zero / infinity / NaN: bit replication of WORD[1]
  uint64_t w1 = (w >> 30) & 1;
  uint64_t r = s | (w1 << 62);
  if (e != 0 && e != 0xFF) r |= uint64_t(!w1 ? 7 : 0) << 59;
  else if (e == 0xFF) r |= uint64_t(7) << 59;
  r |= uint64_t(w & 0x3FFFFFFF) << 29;
  return r;
}

// stfs: double -> single by bit selection and right shift; never rounds.
inline uint32_t stfs(uint64_t x) {
  uint32_t e = ex(x);
  uint32_t s = uint32_t(x >> 63) << 31;
  if (e > 896 || (x & ~SIGN64) == 0)
    return s | (uint32_t((x >> 62) & 1) << 30) | uint32_t((x >> 29) & 0x3FFFFFFF);
  if (e >= 874) {
    int exp = int(e) - 1023;
    uint64_t frac = (1ull << 52) | fr(x);  // 1.f as bits 52..0
    while (exp < -126) { frac >>= 1; exp++; }
    return s | uint32_t((frac >> 29) & 0x7FFFFF);
  }
  // UNDEFINED below 2^-149 [PEM D-16, B-19]. Supermodel stores (float)x in the
  // host's round-to-nearest: a value between 2^-150 and 2^-149 becomes the
  // smallest denormal, anything smaller (2^-150 itself ties to even) zero.
  if (e == 873 && fr(x) != 0) return s | 1;
  return s;
}

// ------------------------------------------------- the frsqrte estimate
// {B, S} per entry, index {parity of e, top 6 fraction bits}: the same
// integers as rtl/cpu/ppc_fpu_rsqe.sv, both written by sim/fpu/gen_rsqe_tab.py
static const uint32_t rsqe_tab[128][2] = {
  // BEGIN rsqe table (sim/fpu/gen_rsqe_tab.py)
  {4194253u, 8097u},
  {4161866u, 7913u},
  {4130218u, 7735u},
  {4099281u, 7563u},
  {4069028u, 7398u},
  {4039436u, 7239u},
  {4010481u, 7086u},
  {3982139u, 6938u},
  {3954389u, 6795u},
  {3927212u, 6656u},
  {3900587u, 6523u},
  {3874497u, 6394u},
  {3848924u, 6269u},
  {3823850u, 6148u},
  {3799260u, 6031u},
  {3775138u, 5917u},
  {3751470u, 5807u},
  {3728241u, 5701u},
  {3705439u, 5597u},
  {3683050u, 5497u},
  {3661062u, 5400u},
  {3639463u, 5305u},
  {3618242u, 5214u},
  {3597388u, 5125u},
  {3576891u, 5038u},
  {3556740u, 4954u},
  {3536925u, 4872u},
  {3517438u, 4792u},
  {3498270u, 4715u},
  {3479412u, 4639u},
  {3460855u, 4566u},
  {3442592u, 4494u},
  {3424616u, 4425u},
  {3406918u, 4357u},
  {3389491u, 4291u},
  {3372329u, 4226u},
  {3355426u, 4163u},
  {3338774u, 4102u},
  {3322367u, 4042u},
  {3306200u, 3983u},
  {3290267u, 3926u},
  {3274562u, 3871u},
  {3259079u, 3816u},
  {3243814u, 3763u},
  {3228762u, 3711u},
  {3213917u, 3661u},
  {3199276u, 3611u},
  {3184832u, 3562u},
  {3170582u, 3515u},
  {3156522u, 3469u},
  {3142648u, 3423u},
  {3128954u, 3379u},
  {3115438u, 3336u},
  {3102096u, 3293u},
  {3088924u, 3252u},
  {3075918u, 3211u},
  {3063075u, 3171u},
  {3050392u, 3132u},
  {3037865u, 3094u},
  {3025490u, 3056u},
  {3013266u, 3019u},
  {3001189u, 2983u},
  {2989256u, 2948u},
  {2977464u, 2913u},
  {2965785u, 5726u},
  {2942884u, 5595u},
  {2920505u, 5469u},
  {2898629u, 5348u},
  {2877238u, 5231u},
  {2856313u, 5119u},
  {2835838u, 5010u},
  {2815797u, 4906u},
  {2796175u, 4805u},
  {2776958u, 4707u},
  {2758132u, 4612u},
  {2739683u, 4521u},
  {2721600u, 4433u},
  {2703870u, 4347u},
  {2686482u, 4264u},
  {2669425u, 4184u},
  {2652690u, 4106u},
  {2636265u, 4031u},
  {2620141u, 3958u},
  {2604310u, 3887u},
  {2588762u, 3818u},
  {2573489u, 3752u},
  {2558484u, 3687u},
  {2543738u, 3624u},
  {2529244u, 3562u},
  {2514995u, 3503u},
  {2500984u, 3445u},
  {2487204u, 3389u},
  {2473650u, 3334u},
  {2460316u, 3280u},
  {2447194u, 3229u},
  {2434280u, 3178u},
  {2421569u, 3129u},
  {2409054u, 3081u},
  {2396732u, 3034u},
  {2384597u, 2988u},
  {2372644u, 2944u},
  {2360869u, 2900u},
  {2349268u, 2858u},
  {2337836u, 2817u},
  {2326570u, 2776u},
  {2315465u, 2737u},
  {2304517u, 2699u},
  {2293723u, 2661u},
  {2283080u, 2624u},
  {2272583u, 2588u},
  {2262229u, 2553u},
  {2252016u, 2519u},
  {2241940u, 2486u},
  {2231998u, 2453u},
  {2222187u, 2421u},
  {2212505u, 2389u},
  {2202948u, 2359u},
  {2193513u, 2329u},
  {2184199u, 2299u},
  {2175003u, 2270u},
  {2165921u, 2242u},
  {2156953u, 2215u},
  {2148095u, 2187u},
  {2139345u, 2161u},
  {2130701u, 2135u},
  {2122161u, 2110u},
  {2113723u, 2085u},
  {2105390u, 2060u},  // END rsqe table
};

// -------------------------------------------------------- the operations
inline Out exec(const In &in) {
  Out o{false, 0, in.fpscr, false, 0};
  const uint32_t f0 = in.fpscr;
  const bool ve = f0 & VE, ze = f0 & ZE;
  const uint64_t a = in.a, b = in.b, c = in.c;
  Upd u;
  const Fmt &fm = in.single ? FMT_S : FMT_D;
  // single results hold only the single fraction: the low 29 bits are zero
  // [PEM Fig 3-18]; a NaN result of a single-precision instruction is cut the
  // same way (frsp's rule, PEM D-8; the case is UNDEFINED for a NaN operand
  // that is not a single value, PEM 3.3.4)
  const uint64_t smask = in.single ? ~uint64_t(0x1FFFFFFF) : ~uint64_t(0);

  auto finish_arith = [&](uint64_t res, bool invalid, bool zdiv, bool affects_xx, bool sglcls) {
    // invalid / zero divide enabled: no result, FPRF unchanged, FR FI cleared
    if ((invalid && ve) || (zdiv && ze)) {
      o.wr = false;
      u.frfi = true; u.frv = u.fiv = false;
    } else {
      o.wr = true;
      o.frd = res;
      u.fprf = int(fprf_of(res, sglcls));
    }
    if (affects_xx && u.frfi && u.fiv) u.causes |= XX;
  };

  switch (in.op) {
  // ------------------------------------------------ multiply-add family
  case FADD: case FSUB: case FMUL:
  case FMADD: case FMSUB: case FNMADD: case FNMSUB: {
    const bool has_c = (in.op != FADD && in.op != FSUB);
    const bool has_b = (in.op != FMUL);
    const bool subb  = (in.op == FSUB || in.op == FMSUB || in.op == FNMSUB);
    const bool negr  = (in.op == FNMADD || in.op == FNMSUB);
    const uint64_t cc = has_c ? c : 0x3FF0000000000000ull;  // fadd = frA * 1.0 + frB
    bool snan = is_snan(a) || (has_b && is_snan(b)) || (has_c && is_snan(c));
    if (snan) u.causes |= VXSNAN;
    u.frfi = true;  // rounding always writes FR and FI [PEM-R1 Table 1-1]

    bool pinf = (is_inf(a) || is_inf(cc)) && !is_nan(a) && !is_nan(cc);
    bool imz  = !is_nan(a) && !is_nan(cc) &&
                ((is_inf(a) && is_zero(cc)) || (is_zero(a) && is_inf(cc)));
    if (imz) u.causes |= VXIMZ;
    bool ps = sgn(a) ^ sgn(cc);
    bool bs = sgn(b) ^ subb;
    bool isi = has_b && pinf && !imz && is_inf(b) && (ps != bs);
    if (isi) u.causes |= VXISI;

    uint64_t res;
    bool invalid = snan || imz || isi;
    if (is_nan(a))                 res = quiet(a);
    else if (has_b && is_nan(b))   res = quiet(b);
    else if (has_c && is_nan(c))   res = quiet(c);
    else if (imz || isi)           res = QNAN_DEFAULT;
    else {
      if (pinf)                       res = 0x7FF0000000000000ull | (uint64_t(ps) << 63);
      else if (has_b && is_inf(b))    res = 0x7FF0000000000000ull | (uint64_t(bs) << 63);
      else {
        // finite: exact product and sum. 3400 bits hold any exact sum of a
        // product of two doubles and a double (span below 3200 bits).
        Mp ma(53), mb(53), mc(53), ex_(3400);
        set_fpr(ma.v, a); set_fpr(mb.v, b); set_fpr(mc.v, cc);
        if (subb) mpfr_neg(mb.v, mb.v, MPFR_RNDN);
        mpfr_rnd_t rnd = rnd_of(f0);
        if (has_b) mpfr_fma(ex_.v, ma.v, mc.v, mb.v, rnd);  // exact at 3400 bits
        else       mpfr_mul(ex_.v, ma.v, mc.v, rnd);
        if (mpfr_zero_p(ex_.v)) {
          // exact zero: MPFR follows IEEE for the sign (product sign for fmul;
          // x + -x is +0 except toward -inf) [PEM 3.3.2]
          res = mpfr_signbit(ex_.v) ? SIGN64 : 0;
        } else {
          bool tiny = below_pow2(ex_.v, fm.emin);
          Rounded r = round_fmt(fm, [&](mpfr_t d, mpfr_rnd_t rr) { return mpfr_set(d, ex_.v, rr); },
                                tiny, f0);
          res = r.bits;
          u.frv = r.fr; u.fiv = r.fi;
          if (r.ox) u.causes |= OX;
          if (r.ux) u.causes |= UX;
          if (r.ox && !(f0 & OE)) u.causes |= XX;
        }
      }
      if (negr) res ^= SIGN64;  // rounded, then negated
    }
    res &= smask;
    finish_arith(res, invalid, false, true, in.single);
    break;
  }

  // ------------------------------------------------------------ frsp
  case FRSP: {
    u.frfi = true;
    bool invalid = false;
    uint64_t res;
    if (is_nan(b)) {
      if (is_snan(b)) { u.causes |= VXSNAN; invalid = true; }
      res = quiet(b) & ~uint64_t(0x1FFFFFFF);  // frB[0-34] || 29 zeros, bit 12 set
    } else if (is_inf(b) || is_zero(b)) {
      res = b;
    } else {
      Mp mb(53);
      set_fpr(mb.v, b);
      Rounded r = round_fmt(FMT_S, [&](mpfr_t d, mpfr_rnd_t rr) { return mpfr_set(d, mb.v, rr); },
                            below_pow2(mb.v, -126), f0);
      res = r.bits;
      u.frv = r.fr; u.fiv = r.fi;
      if (r.ox) u.causes |= OX;
      if (r.ux) u.causes |= UX;
      if (r.ox && !(f0 & OE)) u.causes |= XX;
    }
    finish_arith(res, invalid, false, true, true);
    break;
  }

  // ------------------------------------------------------------ fdiv
  case FDIV: {
    u.frfi = true;
    bool snan = is_snan(a) || is_snan(b);
    if (snan) u.causes |= VXSNAN;
    bool idi = is_inf(a) && is_inf(b);
    bool zdz = is_zero(a) && is_zero(b);
    bool zd  = is_zero(b) && !is_zero(a) && !is_inf(a) && !is_nan(a);
    if (idi) u.causes |= VXIDI;
    if (zdz) u.causes |= VXZDZ;
    if (zd)  u.causes |= ZX;
    bool qs = sgn(a) ^ sgn(b);
    uint64_t res;
    if (is_nan(a))       res = quiet(a);
    else if (is_nan(b))  res = quiet(b);
    else if (idi || zdz) res = QNAN_DEFAULT;
    else if (zd || is_inf(a)) res = 0x7FF0000000000000ull | (uint64_t(qs) << 63);
    else if (is_inf(b) || is_zero(a)) res = uint64_t(qs) << 63;
    else {
      Mp ma(53), mb(53), t(53);
      set_fpr(ma.v, a); set_fpr(mb.v, b);
      // tiny: |a| < 2^emin |b|
      mpfr_mul_2si(t.v, mb.v, fm.emin, MPFR_RNDN);
      bool tiny = mpfr_cmpabs(ma.v, t.v) < 0;
      Rounded r = round_fmt(fm, [&](mpfr_t d, mpfr_rnd_t rr) { return mpfr_div(d, ma.v, mb.v, rr); },
                            tiny, f0);
      res = r.bits;
      u.frv = r.fr; u.fiv = r.fi;
      if (r.ox) u.causes |= OX;
      if (r.ux) u.causes |= UX;
      if (r.ox && !(f0 & OE)) u.causes |= XX;
    }
    res &= smask;
    finish_arith(res, snan || idi || zdz, zd, true, in.single);
    break;
  }

  // ------------------------------------------------------------ fres
  // The 603e runs fres in fdivs's 18-cycle slot, and Supermodel's note reads
  // "on the 603 fres behaves the same as fdivs RT, 1.0, RB". UNDEFINED bits
  // [PEM 8-79]: 1.0 / frB rounded once to single, in the current RN.
  // UNDEFINED FR, FI [PEM 8-79]: from that rounding. XX: fres's own list
  // [PEM 8-80] leaves it out, so it is not changed (notes C7).
  case FRES: {
    u.frfi = true;
    bool snan = is_snan(b);
    if (snan) u.causes |= VXSNAN;
    bool zd = is_zero(b);
    if (zd) u.causes |= ZX;
    uint64_t res;
    if (is_nan(b))      res = quiet(b);
    else if (zd)        res = 0x7FF0000000000000ull | (b & SIGN64);
    else if (is_inf(b)) res = b & SIGN64;
    else {
      Mp mb(53), t(53), one(2);
      set_fpr(mb.v, b);
      mpfr_set_ui(one.v, 1, MPFR_RNDN);
      mpfr_mul_2si(t.v, mb.v, -126, MPFR_RNDN);
      bool tiny = mpfr_cmpabs(one.v, t.v) < 0;
      Rounded r = round_fmt(FMT_S, [&](mpfr_t d, mpfr_rnd_t rr) { return mpfr_div(d, one.v, mb.v, rr); },
                            tiny, f0);
      res = r.bits;
      u.frv = r.fr; u.fiv = r.fi;
      if (r.ox) u.causes |= OX;
      if (r.ux) u.causes |= UX;
    }
    res &= ~uint64_t(0x1FFFFFFF);
    finish_arith(res, snan, zd, false, true);
    break;
  }

  // --------------------------------------------------------- frsqrte
#ifdef PPC_FPU_RSQRTE_EST
  // The pipelined estimate (study D-22, rtl/cpu/ppc_fpu_rsqe.sv), bit for bit:
  // frB = m * 2^e, x = m (e even) or 2m (e odd), 1/sqrt(frB) = y * 2^-floor(e/2);
  // y from rsqe_tab (128 lines B - S * dx, sim/fpu/gen_rsqe_tab.py), 2y
  // truncated to 23 fraction bits. Within one part in 32 [PEM 8-82]: worst
  // 2^-16.26 over every input. Specials as the PEM's frsqrte table.
  // UNDEFINED FR, FI [PEM 8-83]: Supermodel leaves them unchanged (no
  // Supermodel FP op writes them), so they are not written here either,
  // except that an enabled invalid or zero divide clears them [PEM 3.3.6.1].
  // XX: not in frsqrte's list, unchanged.
  case FRSQRTE: {
    bool snan = is_snan(b);
    if (snan) u.causes |= VXSNAN;
    bool neg = sgn(b) && !is_zero(b) && !is_nan(b);
    if (neg) u.causes |= VXSQRT;
    bool zd = is_zero(b);
    if (zd) u.causes |= ZX;
    uint64_t res;
    if (is_nan(b))      res = quiet(b);
    else if (neg)       res = QNAN_DEFAULT;
    else if (zd)        res = 0x7FF0000000000000ull | (b & SIGN64);
    else if (is_inf(b)) res = 0;
    else {
      uint32_t E = ex(b);
      uint64_t F = fr(b);
      uint64_t mf;
      uint32_t rx2;
      bool par;
      if (E == 0) {
        int lz = 0;
        while (!((F << lz) & (1ull << 51))) lz++;
        uint64_t fsh = (F << lz) & 0x000FFFFFFFFFFFFFull;
        mf  = (fsh >> 33) & 0x3FFFF;
        rx2 = 3068u + uint32_t(lz);
        par = !(lz & 1);
      } else {
        mf  = (F >> 34) & 0x3FFFF;
        rx2 = 3068u - E;
        par = !(E & 1);
      }
      uint32_t idx = (uint32_t(par) << 6) | uint32_t(mf >> 12);
      uint64_t dx  = mf & 0xFFF;
      uint64_t y   = (uint64_t(rsqe_tab[idx][0]) << 10) - uint64_t(rsqe_tab[idx][1]) * dx;
      res = (uint64_t(rx2 >> 1) << 52) | (((y >> 8) & 0x7FFFFF) << 29);
    }
    finish_arith(res, snan || neg, zd, false, false);
    break;
  }
#else
  // UNDEFINED bits [PEM 8-82]: Supermodel's 1.0 / sqrt(frB) in the host's
  // current rounding mode -- the square root rounded to double, then the
  // quotient rounded to double, both in RN. FR, FI from the second rounding;
  // XX not changed (frsqrte's list, PEM 8-83).
  case FRSQRTE: {
    u.frfi = true;
    bool snan = is_snan(b);
    if (snan) u.causes |= VXSNAN;
    bool neg = sgn(b) && !is_zero(b) && !is_nan(b);
    if (neg) u.causes |= VXSQRT;
    bool zd = is_zero(b);
    if (zd) u.causes |= ZX;
    uint64_t res;
    if (is_nan(b))      res = quiet(b);
    else if (neg)       res = QNAN_DEFAULT;
    else if (zd)        res = 0x7FF0000000000000ull | (b & SIGN64);
    else if (is_inf(b)) res = 0;
    else {
      Mp mb(53), s(53), one(2);
      set_fpr(mb.v, b);
      mpfr_set_ui(one.v, 1, MPFR_RNDN);
      Rounded r = round_fmt(FMT_D, [&](mpfr_t d, mpfr_rnd_t rr) {
                              mpfr_sqrt(s.v, mb.v, rr);
                              return mpfr_ui_div(d, 1, s.v, rr);
                            }, false, f0);
      res = r.bits;
      u.frv = r.fr; u.fiv = r.fi;
    }
    finish_arith(res, snan || neg, zd, false, false);
    break;
  }
#endif

  // ------------------------------------------------------- fctiw(z)
  case FCTIW: case FCTIWZ: {
    u.frfi = true;
    uint32_t rn = (in.op == FCTIWZ) ? 1 : (f0 & 3);
    bool invalid = false, big_pos = false;
    uint32_t lo = 0;
    if (is_nan(b)) {
      invalid = true;
      if (is_snan(b)) u.causes |= VXSNAN;
    } else if (is_inf(b)) {
      invalid = true; big_pos = !sgn(b);
    } else {
      Mp mb(53), r(64);
      set_fpr(mb.v, b);
      int t = mpfr_rint(r.v, mb.v, rnd_of(rn));  // the D.4.2 model: round, then range-check
      // 2^31 - 1 < r or r < -2^31
      if (mpfr_cmp_si(r.v, 2147483647L) > 0) { invalid = true; big_pos = true; }
      else if (mpfr_cmp_si(r.v, -2147483648L) < 0) { invalid = true; big_pos = false; }
      else {
        lo = uint32_t(int32_t(mpfr_get_si(r.v, MPFR_RNDN)));
        bool negv = sgn(b);
        u.fiv = t != 0;
        u.frv = negv ? (t < 0) : (t > 0);
        if (u.fiv) u.causes |= XX;
      }
    }
    if (invalid) {
      u.causes |= VXCVI;
      u.frv = u.fiv = false;
      lo = big_pos ? 0x7FFFFFFF : 0x80000000;
    }
    // UNDEFINED frD[0-31] [PEM 8-62]: Supermodel writes the in-range result
    // zero-extended and the clamped ones sign-extended.
    uint64_t hi = (invalid && (lo >> 31)) ? 0xFFFFFFFFull : 0;
    if (invalid && ve) { o.wr = false; }
    else { o.wr = true; o.frd = (hi << 32) | lo; }
    // UNDEFINED FPRF [PEM 8-62]: Supermodel leaves it unchanged.
    break;
  }

  // ---------------------------------------------------------- compares
  case FCMPU: case FCMPO: {
    uint8_t cr;
    if (is_nan(a) || is_nan(b)) cr = 1;
    else {
      Mp ma(53), mb(53);
      set_fpr(ma.v, a); set_fpr(mb.v, b);
      int k = mpfr_cmp(ma.v, mb.v);  // +0 = -0
      cr = k < 0 ? 8 : k > 0 ? 4 : 2;
    }
    bool sn = is_snan(a) || is_snan(b);
    if (sn) u.causes |= VXSNAN;
    if (in.op == FCMPO) {
      if (sn) { if (!ve) u.causes |= VXVC; }
      else if (is_nan(a) || is_nan(b)) u.causes |= VXVC;
    }
    u.fpcc = cr;
    o.crw = true; o.cr = cr;
    break;
  }

  // ------------------------------------------------- select and moves
  case FSEL: {
    o.wr = true;
    bool ge = !is_nan(a) && (is_zero(a) || !sgn(a));
    o.frd = ge ? c : b;
    break;
  }
  case FMR:   o.wr = true; o.frd = b; break;
  case FNEG:  o.wr = true; o.frd = b ^ SIGN64; break;
  case FABS:  o.wr = true; o.frd = b & ~SIGN64; break;
  case FNABS: o.wr = true; o.frd = b | SIGN64; break;

  // ------------------------------------------------------ FPSCR moves
  // UNDEFINED frD[0-31] of mffs [PEM 8-125]: Supermodel writes zero.
  case MFFS: o.wr = true; o.frd = f0; break;

  case MTFSF: {
    // FM[0] is field 0 (FPSCR bits 0-3). FEX and VX are not taken from frB.
    // UNDEFINED reserved bit 20 [PEM p.2-9]: written and read back, as
    // Supermodel's field mask does.
    uint32_t m = 0;
    for (int i = 0; i < 8; i++)
      if ((in.imm >> (7 - i)) & 1) m |= 0xFu << (28 - 4 * i);
    m &= ~(FEX | VX);
    uint32_t v = uint32_t(b);
    o.fpscr = summaries((f0 & ~m) | (v & m));
    return o;
  }
  case MTFSFI: {
    int fld = (in.imm >> 4) & 7;
    uint32_t m = (0xFu << (28 - 4 * fld)) & ~(FEX | VX);
    uint32_t v = (in.imm & 0xF) << (28 - 4 * fld);
    o.fpscr = summaries((f0 & ~m) | (v & m));
    return o;
  }
  case MTFSB0: {
    int bit = in.imm & 31;
    uint32_t m = 1u << (31 - bit);
    if (bit != 1 && bit != 2) o.fpscr = summaries(f0 & ~m);
    else o.fpscr = summaries(f0);
    return o;
  }
  case MTFSB1: {
    // mtfsb1 is not exempt from the FX rule [PEM Table 2-4, 3.3.6]: setting an
    // exception bit that was 0 sets FX. Setting one already 1 leaves FX.
    int bit = in.imm & 31;
    uint32_t m = 1u << (31 - bit);
    uint32_t f = f0;
    if (bit != 1 && bit != 2) {
      if ((m & EXC_BITS) && !(f & m)) f |= FX;
      f |= m;
    }
    o.fpscr = summaries(f);
    return o;
  }
  case MCRFS: {
    int fld = in.imm & 7;
    o.crw = true;
    o.cr = (f0 >> (28 - 4 * fld)) & 0xF;
    static const uint32_t clr[8] = {0x90000000, 0x0F000000, 0x00F00000, 0x00080000, 0, 0x00000700, 0, 0};
    o.fpscr = summaries(f0 & ~clr[fld]);
    return o;
  }
  default: break;
  }

  o.fpscr = apply(f0, u);
  return o;
}

}  // namespace ppcfpu
