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
// The FPU units against the reference model (ppc_fpu_ref.h), bit for bit.
//
// Pipeline phase: a stream of multiply-add operations, one issued most cycles,
// with divides, reciprocal estimates and square-root estimates started
// whenever the iterative unit is free (and a bubble left when it asks to hand
// a result over). Every result on the port is matched by tag and compared:
// the write enable, all 64 result bits when written, and the new FPSCR from
// ppc_fpu_fpscr fed with the result's flags and the FPSCR the operation saw.
//
// Combinational phase: the small unit (compares, fsel, moves, mffs) with its
// CR field, the FPSCR instructions (mtfsf, mtfsfi, mtfsb0, mtfsb1, mcrfs), and
// the lfs / stfs conversions.
//
// Directed cases run first: zeros, denormals, extremes, ties at the guard
// position, overflow and underflow boundaries, NaNs in every operand place,
// invalid operations, fctiw around +-2^31, single operations on double-only
// operands.
//
// FPU_FAST (built with fpu_tb_top FAST = 1): the arithmetic unit is
// ppc_fpu_fast. Every op is routed by the RTL's own fpu_fast_route (checked
// against the bench's rule), fast ops issue only on f_fast_ok, f-port results
// must arrive in issue order, and every single op that went fast is issued a
// second time to the double unit: the two results must be identical (and both
// equal to the model). frsqrte runs on the fast port as the estimate
// (PPC_FPU_RSQRTE_EST), each result also checked against 1/sqrt(frB) with MPFR.
//
//   tb_fpu [pipeline_ops] [comb_ops] [seed]

#include "Vfpu_tb_top.h"
#include "verilated.h"
#include "ppc_fpu_ref.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <random>
#include <vector>

using namespace ppcfpu;

namespace {

std::mt19937_64 rng;
uint64_t R() { return rng(); }
int Ri(int n) { return int(rng() % uint64_t(n)); }
bool P(int pct) { return Ri(100) < pct; }

uint64_t d2u(double d) { uint64_t u; std::memcpy(&u, &d, 8); return u; }
double u2d(uint64_t u) { double d; std::memcpy(&d, &u, 8); return d; }

uint64_t mk(uint64_t s, uint64_t e, uint64_t f) { return (s << 63) | ((e & 0x7FF) << 52) | (f & 0xFFFFFFFFFFFFFull); }

// ------------------------------------------------------------ operands
uint64_t frac_pattern() {
  switch (Ri(8)) {
  case 0: return 0;
  case 1: return 0xFFFFFFFFFFFFFull;
  case 2: return 1ull << Ri(52);
  case 3: return R() & ~0x1FFFFFFFull;            // single fraction
  case 4: return (R() & ~0x1FFFFFFFull) | (1ull << Ri(29));
  case 5: return R() & (0xFFFFFFFFFFFFFull << Ri(52));  // short
  case 6: return 0xFFFFFFFFFFFFFull >> Ri(52);
  default: return R();
  }
}

#ifdef FPU_FAST
// a value held exactly in single: zero, infinity, NaN with a single payload,
// or a 24-bit significand at an exponent of 2^-149 ... 2^127
uint64_t gen_sgl_val() {
  uint64_t s = R() & 1;
  int k = Ri(100);
  if (k < 4) return mk(s, 0, 0);
  if (k < 7) return mk(s, 0x7FF, 0);
  if (k < 10) return mk(s, 0x7FF, (P(50) ? (1ull << 51) : 0) | (R() & 0x7FFFFE0000000ull) | (1ull << (29 + Ri(22))));
  uint64_t e;
  if (k < 20) e = 874 + Ri(23);                       // single denormals
  else if (k < 30) e = P(50) ? 897 + Ri(4) : 1150 - Ri(4);
  else if (k < 50) e = 1023 - 30 + Ri(60);
  else e = 874 + Ri(277);
  return mk(s, e, frac_pattern() & ~0x1FFFFFFFull);
}

// the route rule, written again from the study's wording (D-21)
bool sgl_ok(uint64_t x) {
  uint32_t e = ex(x);
  if (e == 0) return fr(x) == 0;
  if (x & 0x1FFFFFFFull) return false;
  return e == 0x7FF || (e >= 874 && e <= 1150);
}
bool c_route(const In &in) {
  if (in.op == FRSQRTE) return true;
  if (!in.single) return false;
  switch (in.op) {
  case FADD: case FSUB: return sgl_ok(in.a) && sgl_ok(in.b);
  case FMUL: return sgl_ok(in.a) && sgl_ok(in.c);
  case FMADD: case FMSUB: case FNMADD: case FNMSUB: return sgl_ok(in.a) && sgl_ok(in.b) && sgl_ok(in.c);
  default: return false;
  }
}
#endif

uint64_t gen_val(bool sgl) {
  uint64_t s = R() & 1;
  int k = Ri(100);
  if (k < 3) return mk(s, 0, 0);
  if (k < 6) return mk(s, 0x7FF, 0);
  if (k < 8) return mk(s, 0x7FF, (1ull << 51) | (P(50) ? (R() & 0x7FFFFFFFFFFFFull) : (R() & 0x1FFFFFFFull)));
  if (k < 10) { uint64_t f = P(50) ? (R() & 0x7FFFFFFFFFFFFull) : (R() & 0x1FFFFFFFull); return mk(s, 0x7FF, f ? f : 1); }
  if (k < 17) return mk(s, 0, frac_pattern() | (P(10) ? 1 : 0));          // double denormal
  if (k < 22) return mk(s, P(50) ? 1 + Ri(4) : 2046 - Ri(4), frac_pattern());
  if (k < 32) return mk(s, P(50) ? 860 + Ri(45) : 1140 + Ri(16), frac_pattern());
  if (sgl || k < 62) {
    // single-representable: exponent within single range (874..1150), short fraction
    uint64_t e = P(15) ? 874 + Ri(23) : 897 + Ri(254);
    uint64_t f = R() & ~0x1FFFFFFFull;
    if (e < 897) f &= ~0ull << (52 - (int(e) - 873));  // a single denormal has fewer bits
    return mk(s, e, P(20) ? (frac_pattern() & ~0x1FFFFFFFull) : f);
  }
  if (k < 80) return mk(s, 1023 - 40 + Ri(80), frac_pattern());
  return mk(s, 1 + Ri(2046), frac_pattern());
}

uint32_t gen_fpscr() {
  uint32_t f = uint32_t(R());
  f &= ~0x00000800u | (P(10) ? 0x800u : 0);  // the reserved bit, now and then
  uint32_t ctl = f & 3;                        // RN
  for (uint32_t b : {VE, OE, UE, ZE, XE, NI})
    if (P(22)) ctl |= b;
  f = (f & ~0xFFu) | ctl;
  if (P(30)) f &= 0x000000FF;                  // clean status now and then
  return summaries(f);
}

// a value d ulps (of precision p) away from x
uint64_t nudge(uint64_t x, int d, int p) {
  if ((x & 0x7FF0000000000000ull) == 0x7FF0000000000000ull) return x;
  uint64_t step = 1ull << (53 - p);
  return x + uint64_t(int64_t(d)) * step;
}

// operands that make the interesting cases likely
void shape(Op op, bool sgl, uint64_t &a, uint64_t &b, uint64_t &c) {
  int p = sgl ? 24 : 53;
  int k = Ri(100);
  if (op == FADD || op == FSUB) {
    if (k < 20) b = nudge(a ^ (op == FADD ? SIGN64 : 0), Ri(9) - 4, p);       // cancellation
    else if (k < 45) {                                                       // guard / sticky ties
      int ea = int(ex(a));
      if (ea > 60 && ea < 2000)
        b = mk(R() & 1, uint64_t(ea - p + Ri(5) - 2), P(50) ? 0 : frac_pattern() & (R() | 0xF000000000000ull));
    } else if (k < 55) b = mk(R() & 1, P(50) ? 2046 : 1 + Ri(3), frac_pattern());
  } else if (op == FMUL || op == FMADD || op == FMSUB || op == FNMADD || op == FNMSUB || op == FDIV) {
    if (k < 25) {
      // exponents chosen to land the product near overflow or underflow
      int target = P(50) ? (sgl ? 127 : 1023) + Ri(5) - 2 : (sgl ? -126 - Ri(30) : -1022 - Ri(60));
      if (Ri(3) == 0) target = (sgl ? -149 : -1074) + Ri(5) - 2;
      int ea = 1 + Ri(2046);
      int eaU = ea - 1023;
      int ecU = (op == FDIV) ? eaU - target : target - eaU;
      int ec = ecU + 1023;
      if (ec >= 1 && ec <= 2046) {
        a = mk(R() & 1, uint64_t(ea), frac_pattern());
        uint64_t &o = (op == FDIV) ? b : c;
        o = mk(R() & 1, uint64_t(ec), frac_pattern());
      }
    }
    if (op != FMUL && op != FDIV && k >= 25 && k < 55) {
      // addend that cancels the product to within a few ulps
      double pr = u2d(a) * u2d(c);
      if (std::isfinite(pr) && pr != 0) {
        uint64_t q = d2u(pr);
        bool sb = (op == FMSUB || op == FNMSUB);
        q = nudge(q, Ri(9) - 4, P(50) ? 53 : 24);
        b = sb ? q : (q ^ SIGN64);
      }
    } else if (op != FMUL && op != FDIV && k >= 55 && k < 70) {
      // addend a guard / round / sticky position below the product
      double pr = u2d(a) * u2d(c);
      if (std::isfinite(pr) && pr != 0) {
        int e = int(ex(d2u(pr)));
        int eb = e - p + Ri(7) - 3;
        if (eb >= 1 && eb <= 2046) b = mk(R() & 1, uint64_t(eb), P(50) ? 0 : frac_pattern());
      }
    }
  } else if (op == FCTIW || op == FCTIWZ) {
    if (k < 40) {
      double base = P(50) ? 2147483648.0 : -2147483648.0;
      double v = base + double(Ri(17) - 8) * 0.25;
      if (P(30)) v = base * (P(50) ? 1.0 : 0.5) + 0.5;
      b = d2u(v);
    } else if (k < 70) {
      b = d2u(double(int64_t(R() % 2000001) - 1000000) * 0.5 + (P(30) ? 0x1p-30 : 0));
    }
  } else if (op == FRSP) {
    if (k < 40) {
      // around single's limits: 2^128, 2^-126, 2^-149, 2^-150
      static const int lims[] = {1150, 1151, 1149, 897, 896, 874, 873, 872, 875};
      b = mk(R() & 1, uint64_t(lims[Ri(9)]), frac_pattern());
    }
  }
}

// -------------------------------------------------------------- the DUT
Vfpu_tb_top *top;
uint64_t cycle = 0;

#ifdef FPU_FAST_EF
bool ef_mode = false;   // run_ef drives the single unit's sources itself
#endif
void tick() {
#ifdef FPU_FAST_EF
  if (!ef_mode) { top->f_sel = 0; top->f_aq = top->f_a; top->f_bq = top->f_b; top->f_cq = top->f_c; }
#endif
  top->clk = 0; top->eval();
  top->clk = 1; top->eval();
  cycle++;
}

int fma_code(Op op) {
  switch (op) {
  case FADD: return 0; case FSUB: return 1; case FMUL: return 2; case FMADD: return 3;
  case FMSUB: return 4; case FNMADD: return 5; case FNMSUB: return 6; case FRSP: return 7;
  case FCTIW: return 8; case FCTIWZ: return 9; case FRSQRTE: return 10; default: return -1;
  }
}

struct Pend { In in; Out want; uint64_t t0; bool fast = false; int pair = -1; long seq = -1; };
std::map<int, Pend> pend;
long fails = 0, checked = 0;
long per_op[NUM_OPS] = {};
struct DivStat { long n = 0, mn = 1 << 30, mx = 0, sum = 0; } dstat[4];  // fdiv, fdivs, fres, frsqrte

#ifdef FPU_FAST
// fast results and their double-unit twins
struct Twin { int n = 0; bool wr[2]; uint64_t res[2]; uint32_t fs[2]; };
std::map<int, Twin> twins;
int next_pair = 0;
long n_fast = 0, n_twin = 0, n_route_bad = 0, n_stall = 0, n_rsq = 0;
long last_fseq = -1, fseq = 0;
double rsq_worst = 0;
// a fast op's twin waiting for the double unit
struct Item { In in; int pair; bool dup; };
std::deque<Item> dup_q;
#endif

// what the checked operations exercised, from the model's answers
struct Cov {
  long ux = 0, ux_en = 0, ox = 0, ox_en = 0, fi = 0, fr = 0, exact = 0, den = 0, zero = 0, nan = 0;
  long vxsnan = 0, vxisi = 0, vximz = 0, vxidi = 0, vxzdz = 0, zx = 0, vxsqrt = 0, vxcvi = 0, vxvc = 0;
  long suppressed = 0, ni_flush = 0;
} cov;

void count(const In &in, const Out &w) {
  uint32_t f = w.fpscr, g = in.fpscr;
  auto rose = [&](uint32_t m) { return (f & m) && !(g & m); };
  if (rose(UX)) { cov.ux++; if (g & UE) cov.ux_en++; }
  if (rose(OX)) { cov.ox++; if (g & OE) cov.ox_en++; }
  if (f & FI) cov.fi++;
  if (f & FR) cov.fr++;
  if (!w.wr) cov.suppressed++;
  if (rose(VXSNAN)) cov.vxsnan++;
  if (rose(VXISI)) cov.vxisi++;
  if (rose(VXIMZ)) cov.vximz++;
  if (rose(VXIDI)) cov.vxidi++;
  if (rose(VXZDZ)) cov.vxzdz++;
  if (rose(ZX)) cov.zx++;
  if (rose(VXSQRT)) cov.vxsqrt++;
  if (rose(VXCVI)) cov.vxcvi++;
  if (rose(VXVC)) cov.vxvc++;
  bool arith = in.op <= FNMSUB || in.op == FRES || in.op == FRSQRTE || in.op == FRSP;
  if (w.wr && arith) {
    uint32_t c = fprf_of(w.frd, in.single || in.op == FRSP);
    if (c == C_PDEN || c == C_NDEN) cov.den++;
    if (c == C_PZERO || c == C_NZERO) { cov.zero++; if ((g & NI) && (f & UX)) cov.ni_flush++; }
    if (c == C_QNAN) cov.nan++;
    if (!(f & FI)) cov.exact++;
  }
}

void report(const char *what, const In &in, const Out &want, bool wr, uint64_t res, uint32_t fs) {
  if (++fails > 25) return;
  printf("FAIL %s %s%s a=%016llx b=%016llx c=%016llx fpscr=%08x\n", what, op_name[in.op], in.single ? "s" : "",
         (unsigned long long)in.a, (unsigned long long)in.b, (unsigned long long)in.c, in.fpscr);
  printf("     want wr=%d frd=%016llx fpscr=%08x\n", want.wr, (unsigned long long)want.frd, want.fpscr);
  printf("     got  wr=%d frd=%016llx fpscr=%08x\n", wr, (unsigned long long)res, fs);
}

void check_out() {
  if (!top->o_valid) return;
  auto it = pend.find(top->o_tag);
  if (it == pend.end()) { printf("FAIL unexpected tag %d at cycle %llu\n", top->o_tag, (unsigned long long)cycle); fails++; return; }
  Pend p = it->second;
  pend.erase(it);
  bool wr = (top->o_flags >> 22) & 1;
  top->fs_op = 1;  // FS_FLAGS
  top->fs_sel = 0;
  top->fs_in = p.in.fpscr;
  top->eval();
  uint32_t fs = top->fs_out;
  checked++;
  per_op[p.in.op]++;
  count(p.in, p.want);
  if (wr != p.want.wr || (wr && top->o_res != p.want.frd) || fs != p.want.fpscr)
    report(p.fast ? "fast" : "pipe", p.in, p.want, wr, top->o_res, fs);
#ifdef FPU_FAST
  if (!top->o_div && p.seq >= 0) {
    if (p.seq <= last_fseq) report("order", p.in, p.want, wr, top->o_res, fs);
    last_fseq = p.seq;
  }
  if (p.pair >= 0) {
    Twin &t = twins[p.pair];
    int k = p.fast ? 0 : 1;
    t.wr[k] = wr; t.res[k] = top->o_res; t.fs[k] = fs;
    if (++t.n == 2) {
      n_twin++;
      if (t.wr[0] != t.wr[1] || (t.wr[0] && t.res[0] != t.res[1]) || t.fs[0] != t.fs[1]) {
        Out w{}; w.wr = t.wr[1]; w.frd = t.res[1]; w.fpscr = t.fs[1];
        report("twin (want = double unit)", p.in, w, t.wr[0], t.res[0], t.fs[0]);
      }
      twins.erase(p.pair);
    }
  }
  // the estimate against 1/sqrt(frB), independently of the table
  if (p.in.op == FRSQRTE && wr) {
    uint64_t b = p.in.b;
    if (!sgn(b) && !is_zero(b) && !is_nan(b) && !is_inf(b)) {
      Mp mb(53), me(53), t(300);
      set_fpr(mb.v, b); set_fpr(me.v, top->o_res);
      mpfr_sqrt(t.v, mb.v, MPFR_RNDN);
      mpfr_mul(t.v, t.v, me.v, MPFR_RNDN);
      mpfr_sub_ui(t.v, t.v, 1, MPFR_RNDN);
      double e = std::fabs(mpfr_get_d(t.v, MPFR_RNDN));
      n_rsq++;
      if (e > rsq_worst) rsq_worst = e;
      if (e > 1.0 / 32 || e > 1.28e-5) report("rsqrte bound", p.in, p.want, wr, top->o_res, fs);
    }
  }
#endif
  if (top->o_div) {
    int k = p.in.op == FDIV ? (p.in.single ? 1 : 0) : p.in.op == FRES ? 2 : 3;
    long n = long(cycle - p.t0);
    DivStat &d = dstat[k];
    d.n++; d.sum += n; if (n < d.mn) d.mn = n; if (n > d.mx) d.mx = n;
  }
}

// ------------------------------------------------- the pipeline stream
std::deque<In> dir_fma, dir_div;  // directed cases for each unit

In gen_pipe_op(bool div) {
  In in{};
  if (div) {
    int k = Ri(4);
#ifdef FPU_FAST
    in.op = k < 3 ? FDIV : FRES;                // frsqrte is on the fast port
#else
    in.op = k < 2 ? FDIV : k == 2 ? FRES : FRSQRTE;
#endif
    in.single = (in.op == FRES) || (in.op == FDIV && P(50));
  } else {
#ifdef FPU_FAST
    static const Op ops[] = {FADD, FSUB, FMUL, FMADD, FMSUB, FNMADD, FNMSUB, FRSP, FCTIW, FCTIWZ, FRSQRTE};
    in.op = ops[Ri(11)];
    in.single = (in.op <= FNMSUB) && P(70);
#else
    static const Op ops[] = {FADD, FSUB, FMUL, FMADD, FMSUB, FNMADD, FNMSUB, FRSP, FCTIW, FCTIWZ};
    in.op = ops[Ri(10)];
    in.single = (in.op != FRSP && in.op != FCTIW && in.op != FCTIWZ) && P(50);
#endif
  }
  bool sgl_ops = in.single && P(80);  // mostly single operands for single operations
  in.a = gen_val(sgl_ops); in.b = gen_val(sgl_ops); in.c = gen_val(sgl_ops);
#ifdef FPU_FAST
  if (in.single && P(60)) { in.a = gen_sgl_val(); in.b = gen_sgl_val(); in.c = gen_sgl_val(); }
#endif
  if (in.op == FRSQRTE && P(70)) in.b &= ~SIGN64;
  shape(in.op, in.single, in.a, in.b, in.c);
  in.fpscr = gen_fpscr();
  return in;
}

// n_rand random operations, or (n_rand < 0) the directed queues until empty
void run_pipeline(long n_rand, int issue_pct, int div_pct) {
  bool dir = n_rand < 0;
  long issued = 0;
  int ftag = 0, dtag = 0;
  bool prev_req = false;
  auto more_div = [&] { return dir ? !dir_div.empty() : issued < n_rand; };
  auto more_fma = [&] { return dir ? !dir_fma.empty() : issued < n_rand; };
#ifdef FPU_FAST
  Item next{};
  bool have_next = false;
  while (more_div() || more_fma() || !pend.empty() || have_next || !dup_q.empty()) {
#else
  while (more_div() || more_fma() || !pend.empty()) {
#endif
    // divider
    top->d_start = 0;
    if (more_div() && !top->d_busy && P(div_pct)) {
      In in;
      if (dir) { in = dir_div.front(); dir_div.pop_front(); }
      else in = gen_pipe_op(true);
      int tag = 128 | (dtag++ & 127);
      top->d_start = 1;
      top->d_op = in.op == FDIV ? 0 : in.op == FRES ? 1 : 2;
      top->d_single = in.single;
      top->d_ctl = in.fpscr & 0xFF;
      top->d_a = in.a; top->d_b = in.b; top->d_tag = tag;
      pend[tag] = Pend{in, exec(in), cycle};
      issued++;
    }
    // multiply-add: one bubble when the iterative unit first asks to hand over
    bool req = top->d_inj_req;
    top->f_valid = 0;
#ifdef FPU_FAST
    top->f_fast = 0;
    if (!have_next) {
      if (!dup_q.empty() && (P(50) || !more_fma())) { next = dup_q.front(); dup_q.pop_front(); have_next = true; }
      else if (more_fma()) {
        In in;
        if (dir) { in = dir_fma.front(); dir_fma.pop_front(); }
        else in = gen_pipe_op(false);
        next = Item{in, -1, false};
        have_next = true;
      }
    }
    if (have_next && !(req && !prev_req) && P(issue_pct)) {
      const In &in = next.in;
      top->f_op = fma_code(in.op);
      top->f_single = in.single;
      top->f_ctl = in.fpscr & 0xFF;
      top->f_a = in.a; top->f_b = in.b; top->f_c = in.c;
      top->eval();
      bool route = top->f_route;
      if (!next.dup && route != c_route(in)) {
        n_route_bad++;
        report(route ? "route (RTL fast)" : "route (RTL double)", in, exec(in), false, 0, 0);
      }
      bool fast = route && !next.dup;
      if (fast && !top->f_fast_ok) {
        n_stall++;                                  // wait for the port
      } else {
        int tag = ftag++ & 127;
        top->f_valid = 1;
        top->f_fast = fast;
        top->f_tag = tag;
        Pend pd{in, exec(in), cycle};
        pd.fast = fast; pd.seq = fseq++; pd.pair = next.pair;
        if (fast && in.op != FRSQRTE) {
          pd.pair = next_pair++;
          dup_q.push_back(Item{in, pd.pair, true});
        }
        pend[tag] = pd;
        if (!next.dup) issued++;
        if (fast) n_fast++;
        have_next = false;
      }
    }
#else
    if (more_fma() && !(req && !prev_req) && P(issue_pct)) {
      In in;
      if (dir) { in = dir_fma.front(); dir_fma.pop_front(); }
      else in = gen_pipe_op(false);
      int tag = ftag++ & 127;
      top->f_valid = 1;
      top->f_op = fma_code(in.op);
      top->f_single = in.single;
      top->f_ctl = in.fpscr & 0xFF;
      top->f_a = in.a; top->f_b = in.b; top->f_c = in.c; top->f_tag = tag;
      pend[tag] = Pend{in, exec(in), cycle};
      issued++;
    }
#endif
    prev_req = req;
    tick();
    check_out();
    if (pend.size() > 200) { printf("FAIL results lost\n"); fails++; break; }
  }
}

#ifdef FPU_FAST_EF
// ------------------------------------------- the single unit's early forward (FPU_FAST_EF, EF = 1)
// A stream of fast-port ops (single arithmetic, some frsqrte) with VE = ZE = OE = UE = 0, so every result is
// written and is a single value. An operand the op reads is, now and then, the result of the op issued LS - 1
// cycles before (f_sel 3: from the single unit's R5, single-unit producers and consumers only), LS cycles
// before (2: the output register), or earlier (1: the held copy, 0: the operand register), the value the
// reference model gave it. The sources the RTL must not read for that operand get random bits, and so does
// the port's own operand (f_a ...) for an R5 one.
long n_efs[4] = {}, n_ef_ops = 0;
In gen_ef_op() {
  In in{};
  static const Op ops[] = {FADD, FSUB, FMUL, FMADD, FMSUB, FNMADD, FNMSUB, FMADD, FNMSUB, FMUL};
  in.op = P(8) ? FRSQRTE : ops[Ri(10)];
  in.single = in.op != FRSQRTE;
  in.a = gen_sgl_val(); in.b = gen_sgl_val(); in.c = gen_sgl_val();
  if (in.op == FRSQRTE && P(70)) in.b &= ~SIGN64;
  shape(in.op, true, in.a, in.b, in.c);
  if (!sgl_ok(in.a)) in.a = gen_sgl_val();
  if (!sgl_ok(in.b)) in.b = gen_sgl_val();
  if (!sgl_ok(in.c)) in.c = gen_sgl_val();
  uint32_t f = gen_fpscr();
  in.fpscr = f & ~(VE | ZE | OE | UE);
  return summaries(in.fpscr) == in.fpscr ? in : (in.fpscr = summaries(in.fpscr), in);
}
void run_ef(long n, int ls, int issue_pct) {
  struct H { uint64_t res; bool sfma; };
  std::map<uint64_t, H> at;       // issue cycle -> its result
  long issued = 0;
  int ftag = 0;
  ef_mode = true;
  while (issued < n || !pend.empty()) {
    top->f_valid = 0; top->f_fast = 0; top->d_start = 0; top->f_sel = 0;
    if (issued < n && P(issue_pct)) {
      In in = gen_ef_op();
      uint64_t *opv[3] = {&in.a, &in.b, &in.c};
      bool rd[3] = {in.op != FRSQRTE, in.op != FMUL, in.op != FADD && in.op != FSUB && in.op != FRSQRTE};
      uint64_t q[3], x[3], pa[3];
      int sel[3] = {0, 0, 0};
      for (int p = 0; p < 3; p++) {
        int k = Ri(4);
        if (rd[p] && P(50)) {
          uint64_t d = k == 0 ? uint64_t(ls - 1) : k == 1 ? uint64_t(ls) : uint64_t(ls + 1 + Ri(6));
          auto it = cycle >= d ? at.find(cycle - d) : at.end();
          if (it != at.end() && !(k == 0 && (!it->second.sfma || in.op == FRSQRTE))) {
            *opv[p] = it->second.res;
            sel[p] = 3 - k;
            n_efs[k]++;
          }
        }
        q[p] = sel[p] >= 1 ? R() : *opv[p];
        x[p] = sel[p] == 1 ? *opv[p] : R();
        pa[p] = sel[p] == 3 ? R() : *opv[p];
      }
      if (sel[0] == 3 || sel[1] == 3 || sel[2] == 3) n_ef_ops++;
      if (!top->f_fast_ok) { printf("FAIL EF stream: f_fast_ok low\n"); fails++; }
      int tag = ftag++ & 127;
      top->f_valid = 1; top->f_fast = 1; top->f_tag = tag;
      top->f_op = fma_code(in.op); top->f_single = in.single; top->f_ctl = in.fpscr & 0xFF;
      top->f_a = pa[0]; top->f_b = pa[1]; top->f_c = pa[2];
      top->f_aq = q[0]; top->f_bq = q[1]; top->f_cq = q[2];
      top->f_ax = x[0]; top->f_bx = x[1]; top->f_cx = x[2];
      top->f_sel = sel[0] | (sel[1] << 2) | (sel[2] << 4);
      Out w = exec(in);
      Pend pd{in, w, cycle};
      pd.fast = true;
      pend[tag] = pd;
      at[cycle] = H{w.frd, in.op != FRSQRTE};
      issued++;
    }
    tick();
    check_out();
    at.erase(at.begin(), at.lower_bound(cycle > 40 ? cycle - 40 : 0));
    if (pend.size() > 200) { printf("FAIL results lost\n"); fails++; break; }
  }
  top->f_valid = 0; top->f_sel = 0;
  ef_mode = false;
}
#endif

// --------------------------------------------- the combinational units
void run_comb(long n) {
  for (long i = 0; i < n; i++) {
    In in{};
    in.fpscr = gen_fpscr();
    int k = Ri(14);
    in.a = gen_val(false); in.b = gen_val(false); in.c = gen_val(false);
    if (k < 8) {
      static const Op ops[] = {FCMPU, FCMPO, FSEL, FMR, FNEG, FABS, FNABS, MFFS};
      in.op = ops[k];
      if ((in.op == FCMPU || in.op == FCMPO) && P(30)) in.b = P(50) ? in.a : (in.a ^ SIGN64);
      Out want = exec(in);
      top->m_op = k;
      top->m_ctl = in.fpscr & 0xFF;
      top->m_a = in.a; top->m_b = in.b; top->m_c = in.c;
      top->m_fpscr = in.fpscr;
      top->fs_op = (k < 2) ? 1 : 0;  // FS_FLAGS for compares, FS_NONE otherwise
      top->fs_sel = 1;
      top->fs_in = in.fpscr;
#ifdef MISC_PIPE
      tick();   // the small unit built with PIPE = 1: its result a cycle later
#endif
      top->eval();
      bool wr = (top->m_flags >> 22) & 1;
      uint32_t fs = top->fs_out;
      checked++; per_op[in.op]++;
      bool bad = wr != want.wr || (wr && top->m_res != want.frd) || fs != want.fpscr ||
                 (want.crw && top->m_cr != want.cr);
      if (bad) report("comb", in, want, wr, top->m_res, fs);
    } else if (k < 13) {
      static const Op ops[] = {MTFSF, MTFSFI, MTFSB0, MTFSB1, MCRFS};
      in.op = ops[k - 8];
      switch (in.op) {
      case MTFSF:  in.imm = uint32_t(R() & 0xFF); if (P(30)) in.b = R(); break;
      case MTFSFI: in.imm = uint32_t(R() & 0x7F); break;
      case MTFSB0: case MTFSB1: in.imm = uint32_t(R() & 31); break;
      default:     in.imm = uint32_t(R() & 7); break;
      }
      Out want = exec(in);
      top->fs_op = 2 + (k - 8);  // FS_MTFSF ...
      top->fs_in = in.fpscr;
      top->fs_fm = in.imm & 0xFF;
      top->fs_frb = uint32_t(in.b);
      top->fs_fld = (in.op == MTFSFI) ? ((in.imm >> 4) & 7) : (in.imm & 7);
      top->fs_imm = in.imm & 0xF;
      top->fs_bit = in.imm & 31;
      top->eval();
      uint32_t fs = top->fs_out;
      checked++; per_op[in.op]++;
      bool bad = fs != want.fpscr || (want.crw && top->fs_crf != want.cr) || top->fs_cr1 != (fs >> 28);
      if (bad) report("fpscr", in, want, false, top->fs_crf, fs);
    } else {
      // lfs: every single image; stfs: every double image, model D.6 / D.7
      uint32_t w = uint32_t(R());
      if (P(30)) w &= 0x807FFFFF;  // denormals and zeros
      top->l_word = w;
      uint64_t d = in.a;
      if (P(40)) d = mk(R() & 1, 860 + Ri(40), R());
      top->s_dbl = d;
#ifdef MISC_PIPE
      tick();   // the conversions built with PIPE = 1: a cycle later
#endif
      top->eval();
      checked += 2;
      if (top->l_dbl != lfs(w) || top->s_word != stfs(d)) {
        if (++fails <= 25)
          printf("FAIL ldst w=%08x lfs got %016llx want %016llx; d=%016llx stfs got %08x want %08x\n", w,
                 (unsigned long long)top->l_dbl, (unsigned long long)lfs(w), (unsigned long long)d,
                 top->s_word, stfs(d));
      }
    }
  }
}

// ------------------------------------------------------ directed cases
void add_directed() {
  // values: zeros, denormals (double and single held as double), extremes,
  // infinities, NaNs (quiet / signalling, payload high / low), 1.0, ties
  std::vector<uint64_t> v = {
    0, SIGN64, 1, SIGN64 | 1, 0x000FFFFFFFFFFFFFull, 0x0008000000000000ull,
    0x0010000000000000ull, 0x8010000000000000ull, 0x7FEFFFFFFFFFFFFFull, 0xFFEFFFFFFFFFFFFFull,
    0x7FF0000000000000ull, 0xFFF0000000000000ull, 0x7FF8000000000000ull, 0xFFF8000000000001ull,
    0x7FF4000000000000ull, 0x7FF0000000000001ull, 0x7FF00000E0000000ull,
    0x3FF0000000000000ull, 0xBFF0000000000000ull, 0x3FF0000000000001ull, 0x3FF0000010000000ull,
    0x3FF0000020000000ull, 0x3FF0000030000000ull, 0x3CA0000000000000ull, 0x3E70000000000000ull,
    0x47EFFFFFE0000000ull, 0x47F0000000000000ull, 0x47EFFFFFF0000000ull, 0x3810000000000000ull,
    0x380FFFFFE0000000ull, 0x36A0000000000000ull, 0x3690000000000000ull, 0x3698000000000000ull,
    0x36A0000000000001ull, 0x41DFFFFFFFC00000ull, 0x41E0000000000000ull, 0xC1E0000000000000ull,
    0xC1E0000000100000ull, 0x41DFFFFFFFE00000ull, 0xC1E0000000200000ull, 0x3FE0000000000000ull,
    0xBFE0000000000000ull, 0x3FF8000000000000ull, 0x4004000000000000ull, 0x0000000000000003ull,
    0x7FE0000000000000ull, 0x0020000000000000ull, 0x3CB0000000000000ull,
  };
  static const Op ops[] = {FADD, FSUB, FMUL, FMADD, FMSUB, FNMADD, FNMSUB, FRSP, FCTIW, FCTIWZ,
                           FDIV, FRES, FRSQRTE};
  for (Op op : ops)
    for (int sgl = 0; sgl < 2; sgl++) {
      if (sgl && (op == FRSP || op == FCTIW || op == FCTIWZ || op == FRSQRTE)) continue;
      if (!sgl && op == FRES) continue;
      for (size_t i = 0; i < v.size(); i++)
        for (size_t j = 0; j < v.size(); j++) {
          // two operands from the list, the third from a few; every control
          // setting that changes the result cycles through rn, OE/UE, VE/ZE, NI
          for (int t = 0; t < 2; t++) {
            In in{};
            in.op = op; in.single = sgl;
            in.a = v[i]; in.b = v[j];
            in.c = v[(i * 7 + j * 3 + t) % v.size()];
            uint32_t ctl = uint32_t((i + j + t) & 3);
            int m = int((i * 5 + j + t * 3) % 8);
            if (m & 1) ctl |= OE | UE;
            if (m & 2) ctl |= VE | ZE;
            if (m & 4) ctl |= NI;
            in.fpscr = ctl;
#ifdef FPU_FAST
            auto &q = (op == FDIV || op == FRES) ? dir_div : dir_fma;
#else
            auto &q = (op == FDIV || op == FRES || op == FRSQRTE) ? dir_div : dir_fma;
#endif
            q.push_back(in);
            // the same with the operands rotated, so each NaN is tried in each place
            if (op == FMADD || op == FNMSUB) {
              In s2 = in; s2.a = in.c; s2.c = in.a; q.push_back(s2);
              In s3 = in; s3.b = in.c; s3.c = in.b; q.push_back(s3);
            }
          }
        }
    }
#ifdef FPU_FAST
  // single values only, so every case takes the fast port (and its twin the
  // double unit): single's limits, denormals, ulps around 1, NaN payloads
  std::vector<uint64_t> w = {
    0, SIGN64, 0x36A0000000000000ull, 0xB6A0000000000000ull, 0x36B0000000000000ull,
    0x380FFFFFC0000000ull, 0x3810000000000000ull, 0x3800000000000000ull, 0x8810000000000000ull,
    0x47EFFFFFE0000000ull, 0xC7EFFFFFE0000000ull, 0x47E0000000000000ull, 0x3FF0000000000000ull,
    0xBFF0000000000000ull, 0x3FF0000020000000ull, 0x3FEFFFFFE0000000ull, 0x3FF8000000000000ull,
    0x4000000000000000ull, 0x3E70000000000000ull, 0x3E80000000000000ull, 0x7FF0000000000000ull,
    0xFFF0000000000000ull, 0x7FF8000000000000ull, 0x7FF4000000000000ull, 0xFFFC000020000000ull,
    0x3F80000000000000ull, 0x41DFFFFFE0000000ull, 0x3FFFFFFFE0000000ull, 0x3690000000000000ull + (1ull << 52),
  };
  static const Op sops[] = {FADD, FSUB, FMUL, FMADD, FMSUB, FNMADD, FNMSUB};
  for (Op op : sops)
    for (size_t i = 0; i < w.size(); i++)
      for (size_t j = 0; j < w.size(); j++)
        for (int t = 0; t < 3; t++) {
          In in{};
          in.op = op; in.single = true;
          in.a = w[i]; in.b = w[j]; in.c = w[(i * 5 + j * 3 + t * 7) % w.size()];
          uint32_t ctl = uint32_t((i + j * 3 + t) & 3);
          int m = int((i * 3 + j + t * 5) % 8);
          if (m & 1) ctl |= OE | UE;
          if (m & 2) ctl |= VE | ZE;
          if (m & 4) ctl |= NI;
          in.fpscr = ctl;
          dir_fma.push_back(in);
        }
  // frsqrte: every table segment's ends, both parities, and denormals
  for (int p = 0; p < 2; p++)
    for (int i = 0; i < 64; i++)
      for (int e = 0; e < 4; e++) {
        In in{};
        in.op = FRSQRTE;
        uint64_t f = (uint64_t(i) << 46) | (e & 1 ? 0xFFFFFFFFFFFull >> (e == 3 ? 0 : 12) : 0);
        uint64_t ee = e < 2 ? uint64_t(1023 + p) : (e == 2 ? uint64_t(1 + p) : uint64_t(2045 + p));
        in.b = mk(0, ee, f);
        in.fpscr = uint32_t((i + e) & 3) | (e == 3 ? (VE | ZE) : 0);
        dir_fma.push_back(in);
        In d = in; d.b = mk(0, 0, f >> (e * 9 + p)) | 1;    // denormal frB
        dir_fma.push_back(d);
      }
#endif
}

// start-to-result cycles with normal operands and an idle multiply-add pipe:
// start presented in cycle 0, the result on the port in cycle n
long measure(Op op, bool sgl) {
  In in{};
  in.op = op; in.single = sgl;
  in.a = 0x3FF5555555555555ull; in.b = 0x4008000000000001ull;
  top->f_valid = 0;
  top->d_start = 1;
  top->d_op = op == FDIV ? 0 : op == FRES ? 1 : 2;
  top->d_single = sgl; top->d_ctl = 0; top->d_a = in.a; top->d_b = in.b; top->d_tag = 200;
  pend[200] = Pend{in, exec(in), cycle};
  uint64_t t0 = cycle;
  tick();
  top->d_start = 0;
  while (!top->o_valid) tick();
  long n = long(cycle - t0);
  check_out();
  return n;
}

}  // namespace

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long n_pipe = argc > 1 ? atol(argv[1]) : 1000000;
  long n_comb = argc > 2 ? atol(argv[2]) : 1000000;
  rng.seed(argc > 3 ? atol(argv[3]) : 1);

  top = new Vfpu_tb_top;
  top->rst = 1;
  for (int i = 0; i < 4; i++) tick();
  top->rst = 0;

  add_directed();
  long nd = long(dir_fma.size() + dir_div.size());
  // directed: issue every cycle, the divider offered every chance
  run_pipeline(-1, 100, 100);
  printf("directed: %ld cases, %ld failures\n", nd, fails);
  // random: mostly back-to-back, some gaps; divides interleaved
  run_pipeline(n_pipe, 90, 30);
  run_comb(n_comb);
#ifdef FPU_FAST_EF
  {
    // the fast port's latency first (an idle unit)
    In in{}; in.op = FADD; in.single = true; in.a = 0x3FF0000000000000ull; in.b = in.a;
    top->f_valid = 1; top->f_op = 0; top->f_single = 1; top->f_fast = 1; top->f_ctl = 0;
    top->f_a = in.a; top->f_b = in.b; top->f_c = 0; top->f_tag = 203;
    pend[203] = Pend{in, exec(in), cycle};
    uint64_t t0 = cycle;
    tick();
    top->f_valid = 0; top->f_fast = 0;
    while (!top->o_valid) tick();
    int ls = int(cycle - t0);
    check_out();
    long f0 = fails, c0 = checked;
    run_ef(n_pipe, ls, 92);
    run_ef(n_pipe / 4, ls, 60);
    printf("early forward (LS %d): %ld ops checked, %ld failures; dependent operands by source: R5 %ld, output register %ld, held %ld, register %ld; ops with an R5 operand %ld\n",
           ls, checked - c0, fails - f0, n_efs[0], n_efs[1], n_efs[2], n_efs[3], n_ef_ops);
  }
#endif

  printf("checked %ld, failures %ld\n", checked, fails);
  printf("coverage (pipeline): UX %ld (UE=1 %ld), OX %ld (OE=1 %ld), FI %ld, FR %ld, exact %ld,\n"
         "  denormal results %ld, zero results %ld (NI flush %ld), NaN results %ld, no write %ld,\n"
         "  VXSNAN %ld VXISI %ld VXIMZ %ld VXIDI %ld VXZDZ %ld ZX %ld VXSQRT %ld VXCVI %ld\n",
         cov.ux, cov.ux_en, cov.ox, cov.ox_en, cov.fi, cov.fr, cov.exact, cov.den, cov.zero, cov.ni_flush, cov.nan,
         cov.suppressed, cov.vxsnan, cov.vxisi, cov.vximz, cov.vxidi, cov.vxzdz, cov.zx, cov.vxsqrt, cov.vxcvi);
  for (int i = 0; i < NUM_OPS; i++)
    if (per_op[i]) printf("  %-8s %ld\n", op_name[i], per_op[i]);
#ifdef FPU_FAST
  printf("iterative unit, normal operands, idle pipe: fdiv %ld, fdivs %ld, fres %ld cycles\n",
         measure(FDIV, false), measure(FDIV, true), measure(FRES, true));
  printf("fast port: %ld fast ops, %ld twins compared, %ld route mismatches, %ld issue waits;\n"
         "  frsqrte estimates checked %ld, worst relative error %.4g (2^%.2f)\n",
         n_fast, n_twin, n_route_bad, n_stall, n_rsq, rsq_worst, rsq_worst > 0 ? std::log2(rsq_worst) : 0.0);
  {
    In in{}; in.op = FADD; in.single = true; in.a = 0x3FF0000000000000ull; in.b = in.a;
    top->f_valid = 1; top->f_op = 0; top->f_single = 1; top->f_fast = 1; top->f_ctl = 0;
    top->f_a = in.a; top->f_b = in.b; top->f_c = 0; top->f_tag = 202;
    pend[202] = Pend{in, exec(in), cycle};
    uint64_t t0 = cycle;
    tick();
    top->f_valid = 0; top->f_fast = 0;
    while (!top->o_valid) tick();
    printf("fast port latency: %llu cycles\n", (unsigned long long)(cycle - t0));
    check_out();
  }
#else
  printf("iterative unit, normal operands, idle pipe: fdiv %ld, fdivs %ld, fres %ld, frsqrte %ld cycles\n",
         measure(FDIV, false), measure(FDIV, true), measure(FRES, true), measure(FRSQRTE, false));
#endif
  // the multiply-add latency, issue to result
  {
    In in{}; in.op = FADD; in.a = 0x3FF0000000000000ull; in.b = in.a;
    top->f_valid = 1; top->f_op = 0; top->f_single = 0; top->f_ctl = 0;
    top->f_a = in.a; top->f_b = in.b; top->f_c = 0; top->f_tag = 201;
    pend[201] = Pend{in, exec(in), cycle};
    uint64_t t0 = cycle;
    tick();
    top->f_valid = 0;
    while (!top->o_valid) tick();
    printf("multiply-add latency: %llu cycles\n", (unsigned long long)(cycle - t0));
    check_out();
  }
  static const char *dn[4] = {"fdiv", "fdivs", "fres", "frsqrte"};
  for (int k = 0; k < 4; k++)
    if (dstat[k].n)
      printf("  %-8s start to result: min %ld, mean %.1f, max %ld cycles (%ld ops)\n", dn[k], dstat[k].mn,
             double(dstat[k].sum) / double(dstat[k].n), dstat[k].mx, dstat[k].n);
  delete top;
  return fails ? 1 : 0;
}
