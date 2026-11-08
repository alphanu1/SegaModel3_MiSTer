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
// Reference instruction-set model for the 603e: integer, MMU and FP.
//
// Written from the instruction definitions in the 32-bit PowerPC Programming
// Environments Manual and the MPC603e User's Manual, independently of the RTL,
// so a shared misreading has to be made twice to pass. One architectural
// instruction per step; no pipeline, no timing.
//
// Where the manual leaves a result undefined, this follows Supermodel, as the
// RTL does: divw by zero is 0 for a non-negative dividend and -1 otherwise,
// 0x80000000 / -1 is -1, divwu by zero is 0. Traps, sc and illegal opcodes take
// the program (0x700) and system call (0xC00) exceptions.
//
// The 603e MMU (603e manual ch. 5) and the exceptions it needs (ch. 4):
// real mode, BAT and page translation with protection for fetch and data,
// segment registers, the 64-entry two-way ITLB and DTLB with their LRU, the
// software table-search resources (DMISS, DCMP, HASH1, HASH2, IMISS, ICMP,
// RPA, tlbld, tlbli, tlbie, MSR[TGPR] with shadow r0-r3), DSI, ISI, the three
// TLB miss exceptions, and the external and decrementer interrupts, which the
// caller raises at an instruction boundary (interrupt()). The decrementer and
// timebase values are the caller's (dec_read and friends), since their timing
// is the bench's.
//
// Choices where the manual is ambiguous or self-contradictory (each also in
// the RTL, ppc_core.sv):
//   SRR1[15] (S/L) is 1 for a store miss: tables 4-5 and 4-17 and the example
//     handler's DSISR[6] derivation; tables 5-4 and 5-10 say the reverse.
//   DMISS, IMISS, DCMP and ICMP are read/write (2.1.2.2, 2.1.2.3); 1.x and 5.x
//     call them read-only. HASH1/HASH2 are read-only and formed when read, from
//     SDR1 and the DMISS/DCMP or IMISS/ICMP pair of the most recent miss
//     (2.1.2.4: "constructed from the DMISS or IMISS contents").
//   tlbld/tlbli: V and VSID from DCMP/ICMP, EA[4-14] and the set from rB (the
//     instruction definition, 2.3.8: "EA <- (rB)"), the way from SRR1[WAY].
//   SRR1[WAY] ("next TLB set to be replaced, set per LRU") is the way of the
//     set not loaded most recently by tlbld/tlbli: replacement order, not use
//     order. Use order would depend on speculative fetches and on accesses the
//     core serves from its micro-TLBs without reading the TLB. A store that
//     hits an entry with C = 0 gives the way that hit, so the handler's tlbld
//     replaces that entry instead of making a second copy of it. tlbie leaves
//     the order alone.
//   A double TLB hit takes way 0 (a programming error; the core's software
//     reload never makes one). A double BAT hit is undefined (5.3) and never
//     generated: the RTL ORs the hits, this model takes the first.
//   BAT physical address is BRPN | (EA & BL), as the PEM writes it (7.4.3).
//   ISI SRR1[3] (N, T, guarded) and SRR1[4] (protection) are set
//     independently; a page with T = 1 or N = 1 faults before the TLB is
//     looked at (figure 5-6), so an N or T fault is never a TLB miss.
//   Privilege (program exception for supervisor instructions with MSR[PR] =
//     1) and MSR[LE] are not modelled, as in the RTL.
//   Misaligned accesses (603e 2.2.3, 4.5.6). An integer halfword or word that
//     is not naturally aligned is done in hardware: byte for byte here, one or
//     two word accesses in the RTL (the trace sees each word a load reads).
//     The alignment exception (0x600; SRR0 the instruction, DSISR from its
//     encoding by table 4-13, DAR the EA) is taken instead, before anything
//     is done, for: lmw, stmw, lwarx and stwcx. not word-aligned (lmw / stmw:
//     DAR = EA + 4, 4.5.6.2); an FP load or store not word-aligned; an FP
//     double crossing a page with MSR[DR] = 1 (4.5.6.1.1, EA ending
//     0xFF9-0xFFF) with no DBAT hit for its EA (a page-address translation
//     access; one that hits a DBAT is two words, each translated on its own,
//     even where the second leaves the block: "no special handling for
//     accesses that fall into BAT regions", 4.5.6.1.1; R129).
//     A halfword or word crossing a page is done in hardware with DR = 1 too
//     (4.5.3: "The 603e supports the crossing of page boundaries"; study R124,
//     xpage, amending D-39, after Scud Race's lwzx at FF292FFE): the first
//     page's bytes are translated and accessed, then the second word, at
//     (EA & ~3) + 4, the second page's first byte, is translated on its own. A
//     fault there is taken "in the middle of the instruction" (4.5.3): DSI,
//     DAR = that byte ("a byte address in the first word of the offending
//     page"), or a DTLB miss, DMISS = it; a store's bytes in the first page
//     have then been written (PEM 6.1.4), a load writes no register.
//
// Floating point: every 603e FP instruction, the arithmetic, the FPSCR and
// CR1 from sim/fpu/ppc_fpu_ref.h (its own MPFR model, written from the
// manuals), the loads and stores here. Choices, each also in the RTL:
//   fsqrt, fsqrts and every unassigned encoding of opcodes 59 and 63 are
//     illegal whatever MSR[FP] is (603e table B-1, 4.5.7.2); FP unavailable
//     (0x800) is for the instructions the 603e has (603e 4.5.8).
//   MSR[FE0, FE1] other than 00 is precise mode, as the 603e implements the
//     imprecise modes (603e 4.1, 4.5.7.1): after an instruction that may set
//     the FPSCR (arithmetic, compares, mtfsf, mtfsfi, mtfsb0/1, mcrfs), FEX = 1
//     takes the program exception with SRR0 that instruction and SRR1[11],
//     its results written (PEM 3.3.6, table 6-16). FEX = 1 found at an
//     instruction boundary with FE nonzero (only after an MSR write: mtmsr,
//     rfi) is taken before that instruction, SRR0 it, SRR1[11] and SRR1[15]
//     (PEM table 6-16: "SRR0 points to the instruction that would have
//     executed next").
//   lfd / stfd are two word accesses, high word first. A double crossing a
//     page with MSR[DR] = 1 takes the alignment exception (603e 4.5.6.1.1,
//     above) when its EA has no DBAT hit; with one, its second word may leave
//     the block and fault on its own page (a load then writes nothing, a
//     store's first word is in memory, PEM 6.1.4).

#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include "ppc_fpu_ref.h"

struct PpcIss {
  uint32_t gpr[32] = {};
  uint32_t pc = 0xFFF00100, cr = 0, xer = 0, lr = 0, ctr = 0, msr = 0x40, srr0 = 0, srr1 = 0;
  bool     res = false;
  std::map<int, uint32_t> spr;
  uint32_t pvr = 0x00060101;
  uint64_t fpr[32] = {};
  uint32_t fpscr = 0;                                    // zero at hard reset (603e table 4-9)
  std::function<void(int, uint64_t)>    on_fpw = [](int, uint64_t) {};   // each FPR write
  static constexpr uint32_t MSR_FP = 0x2000, MSR_FE = 0x0900;

  std::function<uint8_t(uint32_t)>       rd8;
  std::function<void(uint32_t, uint8_t)> wr8;            // data stores only
  std::function<void(uint32_t)>          on_load = [](uint32_t) {};   // each data load, its physical address

  // decrementer and timebase, the caller's: their values depend on when the
  // instruction executes. The defaults are a counter that never ticks.
  uint32_t dec = 0xFFFFFFFF;
  uint64_t tbr = 0;
  std::function<uint32_t()>              dec_read  = [this]() { return dec; };
  std::function<void(uint32_t)>          dec_write = [this](uint32_t v) { dec = v; };
  std::function<uint32_t(bool)>          tb_read   = [this](bool hi) { return uint32_t(hi ? tbr >> 32 : tbr); };
  std::function<void(bool, uint32_t)>    tb_write  = [this](bool hi, uint32_t v) {
    tbr = hi ? (tbr & 0xFFFFFFFFull) | (uint64_t(v) << 32) : (tbr & ~0xFFFFFFFFull) | v; };

  // ------------------------------------------------------------------ MMU
  static constexpr uint32_t MSR_TGPR = 0x00020000, MSR_EE = 0x8000, MSR_PR = 0x4000,
                            MSR_IR = 0x20, MSR_DR = 0x10;
  uint32_t sr[16] = {};
  struct Tlb { bool v = false; uint32_t vsid = 0, pi = 0, rpn = 0; bool c = false; uint32_t wimg = 0, pp = 0; };
  Tlb      itlb[32][2], dtlb[32][2];
  uint8_t  ilru[32] = {}, dlru[32] = {};       // the way to replace next
  uint32_t tgpr[4] = {};                       // r0-r3 while MSR[TGPR] = 0 is the other set
  // 603e table-search registers and DSISR/DAR (all zero at hard reset, table 4-9)
  uint32_t dsisr = 0, dar = 0, dmiss = 0, dcmp = 0, imiss = 0, icmp = 0;
  bool     last_i = false;                     // the most recent TLB miss was the ITLB's

  static constexpr uint32_t MSR_MASK  = 0x0007FF73;
  static constexpr uint32_t SRR1_KEEP = 0x0000FF73;
  static constexpr uint32_t XER_MASK  = 0xE000007F;

  uint32_t rd32(uint32_t a) { return (rd8(a) << 24) | (rd8(a + 1) << 16) | (rd8(a + 2) << 8) | rd8(a + 3); }
  uint16_t rd16(uint32_t a) { return (rd8(a) << 8) | rd8(a + 1); }
  void wr32(uint32_t a, uint32_t v) { wr8(a, v >> 24); wr8(a + 1, v >> 16); wr8(a + 2, v >> 8); wr8(a + 3, v); }
  void wr16(uint32_t a, uint16_t v) { wr8(a, v >> 8); wr8(a + 1, v); }

  static uint32_t bswap32(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xFF00) | ((v << 8) & 0xFF0000) | (v << 24); }
  static uint16_t bswap16(uint16_t v) { return (v >> 8) | (v << 8); }

  bool known_spr(int n) const {
    return (n >= 272 && n <= 275) || n == 25 || n == 1008 || n == 1009 ||
           n == 1010 || n == 282 || n == 982 || (n >= 528 && n <= 543);
  }

  // HASH1 / HASH2 (2.1.2.4, PEM 7.6.1.1): the PTEG addresses for the most
  // recent miss's EA and VSID under the current SDR1
  uint32_t hash(bool secondary) const {
    uint32_t ea = last_i ? imiss : dmiss, cmp = last_i ? icmp : dcmp, sdr1 = spr.count(25) ? spr.at(25) : 0;
    uint32_t h = ((cmp >> 7) & 0x7FFFF) ^ ((ea >> 12) & 0xFFFF);
    if (secondary) h = ~h & 0x7FFFF;
    return (sdr1 & 0xFE000000) | ((((sdr1 >> 16) & 0x1FF) | ((h >> 10) & sdr1 & 0x1FF)) << 16) | ((h & 0x3FF) << 6);
  }
  uint32_t sp(int n) const { auto i = spr.find(n); return i == spr.end() ? 0 : i->second; }

  void set_cr_field(int f, uint32_t v) { int sh = 28 - 4 * f; cr = (cr & ~(0xFu << sh)) | ((v & 0xF) << sh); }
  uint32_t cr_field(int f) const { return (cr >> (28 - 4 * f)) & 0xF; }
  bool cr_bit(int b) const { return (cr >> (31 - b)) & 1; }
  void set_cr_bit(int b, bool v) { cr = (cr & ~(1u << (31 - b))) | (uint32_t(v) << (31 - b)); }
  bool so() const { return xer >> 31; }
  bool ca() const { return (xer >> 29) & 1; }
  void set_ca(bool v) { xer = (xer & ~(1u << 29)) | (uint32_t(v) << 29); }
  void set_ov(bool v) { xer = (xer & ~(1u << 30)) | (uint32_t(v) << 30); if (v) xer |= 1u << 31; }
  void cr0(uint32_t r) {
    int32_t s = int32_t(r);
    set_cr_field(0, (s < 0 ? 8 : s > 0 ? 4 : 2) | (so() ? 1 : 0));
  }

  static uint32_t mask(int mb, int me) {
    uint32_t lo = 0xFFFFFFFFu >> mb, hi = 0xFFFFFFFFu << (31 - me);
    return mb <= me ? (lo & hi) : (lo | hi);
  }
  static uint32_t rotl(uint32_t v, int n) { n &= 31; return n ? (v << n) | (v >> (32 - n)) : v; }

  uint64_t exc_count[0x20] = {};              // exceptions taken, by vector / 0x100
  uint64_t fp_prec = 0, fp_delayed = 0, fp_ops[ppcfpu::NUM_OPS] = {}, fp_ls[2] = {}, fp_supp = 0;
  // misaligned integer accesses done: inside a word, then crossing a word, a
  // doubleword, a 32-byte line and a page; [5] stores of them all; page
  // crossings with DR = 1: [6] done, [7] done with both pages page-translated,
  // [8] done with one word in a DBAT block and the other not (or another
  // block), [9] second page's DTLB miss taken, [10] second page's DSI taken;
  // [11]-[14] crossings with DR = 1 met (done or faulted), by EA's page offset:
  // a word at 0xFFD, 0xFFE, 0xFFF, a halfword at 0xFFF
  // [15]-[17] FP doubles crossing a page with DR = 1: trapped (no DBAT hit), split inside the block, split
  // leaving it (R129)
  uint64_t mis_n[18] = {};
  void exception(uint32_t vec, uint32_t save_pc, uint32_t srr1_bits) {
    exc_count[(vec >> 8) & 0x1F]++;
    if (vec == 0x700 && (srr1_bits & 0x00100000)) (srr1_bits & 0x00010000 ? fp_delayed : fp_prec)++;
    srr0 = save_pc;
    srr1 = (msr & SRR1_KEEP) | srr1_bits;
    uint32_t ile = (msr >> 16) & 1;
    msr = (msr & ~0x0006EF33u) | ile;
    pc = ((msr >> 6) & 1 ? 0xFFF00000u : 0u) | vec;
  }

  // An asynchronous interrupt at this instruction boundary (4.5.5, 4.5.9):
  // SRR0 is the instruction that would have executed next.
  void interrupt(uint32_t vec) { exception(vec, pc, 0); }

  // ---------------------------------------------------------- translation
  // Effective to physical for one access (5.1.6, figures 5-5 and 5-6). On a
  // fault the exception is taken here (SRR0 = this instruction) and false is
  // returned; the instruction then has no further effect.
  // The access's WIMG is left in last_wimg (real mode 0011, 603e 3.5). With
  // probe set a fault takes no exception (dcbt / dcbtst, 3.7.2).
  enum Acc { FETCH, LOAD, STORE };
  uint32_t last_wimg = 3;
  bool xlate(uint32_t ea, Acc acc, uint32_t &pa, bool probe = false) {
    bool ifetch = acc == FETCH, store = acc == STORE;
    last_wimg = 3;
    if (!(msr & (ifetch ? MSR_IR : MSR_DR))) { pa = ea; return true; }      // 5.2 real mode
    bool pr = msr & MSR_PR;
    uint32_t st = store ? 0x02000000 : 0;                                   // DSISR[6]
    auto dsi = [&](uint32_t bits) { if (probe) return false; dsisr = bits | st; dar = ea; exception(0x300, pc, 0); return false; };
    auto isi = [&](uint32_t bits) { exception(0x400, pc, bits); return false; };
    // BATs (5.3, PEM 7.4): Vs/Vp by MSR[PR], BEPI under the BL mask
    int base = ifetch ? 528 : 536;
    for (int i = 0; i < 4; i++) {
      uint32_t u = sp(base + 2 * i), l = sp(base + 2 * i + 1);
      if (!(pr ? (u & 1) : (u & 2))) continue;
      uint32_t bl = (u >> 2) & 0x7FF, m = ~(bl << 17) & 0xFFFE0000;
      if ((ea & m) != (u & m)) continue;
      uint32_t pp = l & 3;                                                  // 00 none, x1 read, 10 read/write
      if (ifetch) {
        uint32_t bits = (pp == 0 ? 0x08000000 : 0) | ((l & 8) ? 0x10000000 : 0);   // SRR1[4], SRR1[3] (G)
        if (bits) return isi(bits);
      } else if (store ? pp != 2 : pp == 0) return dsi(0x08000000);        // DSISR[4]
      pa = (l & 0xFFFE0000) | (ea & (bl << 17)) | (ea & 0x1FFFF);
      last_wimg = (l >> 3) & 0xF;
      return true;
    }
    // segment (5.4, figure 5-6): T = 1 is not supported by the 603e
    uint32_t s = sr[ea >> 28];
    if (s & 0x80000000) return ifetch ? isi(0x10000000) : dsi(0x04000000); // SRR1[3] / DSISR[5]
    if (ifetch && (s & 0x10000000)) return isi(0x10000000);                 // N: no-execute
    uint32_t vsid = s & 0xFFFFFF, set = (ea >> 12) & 31, pi = (ea >> 17) & 0x7FF;
    uint32_t key = pr ? (s >> 29) & 1 : (s >> 30) & 1;                      // Kp : Ks
    Tlb *t = ifetch ? itlb[set] : dtlb[set];
    uint8_t *lru = ifetch ? &ilru[set] : &dlru[set];
    int w = -1;
    for (int k = 1; k >= 0; k--) if (t[k].v && t[k].vsid == vsid && t[k].pi == pi) w = k;
    auto miss = [&](uint32_t way) {
      if (probe) return false;
      // 4.5.12-4.5.14, table 4-17: SRR1 holds CR0, KEY, D/I, WAY, S/L
      uint32_t cmpv = 0x80000000 | (vsid << 7) | ((ea >> 22) & 0x3F);
      if (ifetch) { imiss = ea; icmp = cmpv; } else { dmiss = ea; dcmp = cmpv; }
      last_i = ifetch;
      uint32_t bits = (cr & 0xF0000000) | (key << 19) | (ifetch ? 0x40000 : 0) | (way << 17) | (store ? 0x10000 : 0);
      exception(ifetch ? 0x1000 : store ? 0x1200 : 0x1100, pc, bits);
      msr |= MSR_TGPR;                                                      // table 4-8
      return false;
    };
    if (w < 0) return miss(*lru);
    const Tlb &e = t[w];
    if (ifetch) {
      uint32_t bits = ((key && e.pp == 0) ? 0x08000000 : 0) | ((e.wimg & 1) ? 0x10000000 : 0);
      if (bits) return isi(bits);
    } else {
      bool ok = store ? (key ? e.pp == 2 : e.pp != 3) : (!key || e.pp != 0);   // PEM table 7-10
      if (!ok) return dsi(0x08000000);
      if (store && !e.c) return miss(uint32_t(w));                         // table 5-7: C = 0
    }
    pa = (e.rpn << 12) | (ea & 0xFFF);
    last_wimg = e.wimg;
    return true;
  }
  void tlb_load(bool inst, uint32_t ea) {                                    // tlbld / tlbli (2.3.8)
    uint32_t set = (ea >> 12) & 31, w = (srr1 >> 17) & 1, cmpv = inst ? icmp : dcmp, rpa = sp(982);
    Tlb &e = (inst ? itlb : dtlb)[set][w];
    e.v = cmpv >> 31; e.vsid = (cmpv >> 7) & 0xFFFFFF; e.pi = (ea >> 17) & 0x7FF;
    e.rpn = rpa >> 12; e.c = (rpa >> 7) & 1; e.wimg = (rpa >> 3) & 15; e.pp = rpa & 3;
    (inst ? ilru : dlru)[set] = uint8_t(w ^ 1);
  }

  // add with carry-in; returns sum, sets carry and overflow out-parameters
  static uint32_t addc(uint32_t a, uint32_t b, uint32_t c, bool &co, bool &ov) {
    uint64_t s = uint64_t(a) + b + c;
    uint32_t r = uint32_t(s);
    co = s >> 32;
    ov = ((a ^ r) & (b ^ r)) >> 31;
    return r;
  }

  // ------------------------------------------------------- floating point
  // The FP operation of an opcode 59 / 63 word, or -1: illegal. Opcode 59 is
  // the single-precision A form; 63's A form is told from its X form by
  // xo[5:1] (no X-form extended opcode has its low five bits in the A set).
  static int fp_op(uint32_t in) {
    using namespace ppcfpu;
    int op = in >> 26, a = (in >> 1) & 31, x = (in >> 1) & 0x3FF;
    switch (a) {
      case 18: return FDIV;  case 20: return FSUB;  case 21: return FADD;  case 25: return FMUL;
      case 28: return FMSUB; case 29: return FMADD; case 30: return FNMSUB; case 31: return FNMADD;
      case 24: return op == 59 ? FRES : -1;
      case 26: return op == 63 ? FRSQRTE : -1;
      case 23: return op == 63 ? FSEL : -1;
      case 22: return -1;                                       // fsqrt(s): not on the 603e
      default: break;
    }
    if (op != 63) return -1;
    switch (x) {
      case 0: return FCMPU; case 32: return FCMPO; case 12: return FRSP; case 14: return FCTIW; case 15: return FCTIWZ;
      case 40: return FNEG; case 72: return FMR; case 136: return FNABS; case 264: return FABS;
      case 583: return MFFS; case 711: return MTFSF; case 134: return MTFSFI; case 70: return MTFSB0;
      case 38: return MTFSB1; case 64: return MCRFS;
      default: return -1;
    }
  }
  // 0 not FP, 1 an FP load or store, 2 FP arithmetic / FPSCR
  static int fp_kind(uint32_t in) {
    int op = in >> 26, x = (in >> 1) & 0x3FF;
    if (op >= 48 && op <= 55) return 1;
    if (op == 31 && (x == 535 || x == 567 || x == 599 || x == 631 || x == 663 || x == 695 ||
                     x == 727 || x == 759 || x == 983)) return 1;
    if ((op == 59 || op == 63) && fp_op(in) >= 0) return 2;
    return 0;
  }
  // may change the FPSCR: precise mode checks FEX after it
  static bool fp_sets(uint32_t in) {
    using namespace ppcfpu;
    int o = fp_op(in);
    return !(o == FSEL || o == FMR || o == FNEG || o == FABS || o == FNABS || o == MFFS);
  }
  void fp_arith(uint32_t in) {
    using namespace ppcfpu;
    int o = fp_op(in);
    int fd = (in >> 21) & 31;
    ppcfpu::In fi{};
    fi.op = Op(o);
    fi.single = (in >> 26) == 59;
    fi.a = fpr[(in >> 16) & 31]; fi.b = fpr[(in >> 11) & 31]; fi.c = fpr[(in >> 6) & 31];
    fi.fpscr = fpscr;
    switch (o) {
      case MTFSF:  fi.imm = (in >> 17) & 0xFF; break;
      case MTFSFI: fi.imm = (((in >> 23) & 7) << 4) | ((in >> 12) & 15); break;
      case MTFSB0: case MTFSB1: fi.imm = (in >> 21) & 31; break;
      case MCRFS:  fi.imm = (in >> 18) & 7; break;
      default: break;
    }
    ppcfpu::Out r = exec(fi);
    fp_ops[o]++;
    if (!r.wr && o != FCMPU && o != FCMPO && o < MFFS) fp_supp++;    // an enabled exception: no frD
    if (r.wr) set_fpr(fd, r.frd);
    fpscr = r.fpscr;
    if (r.crw) set_cr_field((in >> 23) & 7, r.cr);             // fcmpu, fcmpo, mcrfs: crfD
    bool rc = (in & 1) && o != FCMPU && o != FCMPO && o != MCRFS;
    if (rc) set_cr_field(1, fpscr >> 28);                       // CR1: FX FEX VX OX (PEM table 2-2)
  }

  // One instruction. While MSR[TGPR] = 1, r0-r3 are the shadow set (5.5.2.1):
  // they are swapped in for the instruction and back after, so gpr[] always
  // holds the normal set when the caller looks.
  void swap_tgpr() { for (int i = 0; i < 4; i++) std::swap(gpr[i], tgpr[i]); }
  void step() {
    bool t = msr & MSR_TGPR;
    if (t) swap_tgpr();
    step_body();
    if (t) swap_tgpr();
  }

  void set_fpr(int r, uint64_t v) { fpr[r] = v; on_fpw(r, v); }

  void step_body() {
    // an enabled FP exception left pending by an MSR write (PEM table 6-16)
    if ((msr & MSR_FE) && (fpscr & ppcfpu::FEX)) { exception(0x700, pc, 0x00110000); return; }
    uint32_t ipa;
    if (!xlate(pc, FETCH, ipa)) return;
    uint32_t in = rd32(ipa);
    uint32_t npc = pc + 4;
    int op = in >> 26, rd = (in >> 21) & 31, ra = (in >> 16) & 31, rb = (in >> 11) & 31;
    int xo10 = (in >> 1) & 0x3FF, xo9 = (in >> 1) & 0x1FF;
    bool oe = (in >> 10) & 1, rc = in & 1;
    int32_t simm = int16_t(in & 0xFFFF);
    uint32_t uimm = in & 0xFFFF;
    auto A0 = [&]() { return ra ? gpr[ra] : 0u; };
    bool co, ov;

    auto illegal = [&]() { exception(0x700, pc, 0x00080000); };
    auto trap = [&](uint32_t a, uint32_t b) {
      int to = rd;
      bool t = ((to & 16) && int32_t(a) < int32_t(b)) || ((to & 8) && int32_t(a) > int32_t(b)) ||
               ((to & 4) && a == b) || ((to & 2) && a < b) || ((to & 1) && a > b);
      if (t) { exception(0x700, pc, 0x00020000); return true; }
      return false;
    };
    auto branch = [&](uint32_t target, bool lk) {
      if (lk) lr = pc + 4;
      npc = target;
    };
    auto bc_cond = [&](int bo, int bi, bool use_ctr) {
      bool ctr_ok = true;
      if (use_ctr && !(bo & 4)) { ctr--; ctr_ok = (ctr != 0) ^ ((bo >> 1) & 1); }
      bool cond_ok = (bo & 16) || (cr_bit(bi) == ((bo >> 3) & 1));
      return ctr_ok && cond_ok;
    };
    // One translation covers each access but an integer one crossing a page,
    // whose second word is translated on its own (ld, st).
    bool fault = false;
    // table 4-13: indexed (X form, opcode 31) or immediate-index (D form)
    auto align_exc = [&](uint32_t dar_v) {
      bool xf = (in >> 26) == 31;
      dsisr = (xf ? (((in >> 1) & 3) << 15) | (((in >> 6) & 1) << 14) | (((in >> 7) & 15) << 10)
                  : (((in >> 26) & 1) << 14) | (((in >> 27) & 15) << 10)) |
              (((in >> 21) & 31) << 5) | ((in >> 16) & 31);
      dar = dar_v;
      exception(0x600, pc, 0);
      fault = true;
    };
    // D-39 as amended (R124, R129): with MSR[DR] = 1 an FP double crossing a page takes the alignment
    // exception only with no BAT hit for its EA (a page-address translation access, 4.5.6.1.1)
    auto dbat_of = [&](uint32_t ea) -> int {
      bool pr = msr & MSR_PR;
      for (int i = 0; i < 4; i++) {
        uint32_t u = sp(536 + 2 * i);
        if (!(pr ? (u & 1) : (u & 2))) continue;
        uint32_t bl = (u >> 2) & 0x7FF, m = ~(bl << 17) & 0xFFFE0000;
        if ((ea & m) == (u & m)) return i;
      }
      return -1;
    };
    // (R129: an FP double whose EA hits a DBAT is not a page-address translation access, 4.5.6.1.1, so it is
    // two words, each translated on its own, whether or not its second word leaves the block)
    auto pg_cross = [&](uint32_t ea, int size) {
      if (!(msr & MSR_DR) || (ea & 0xFFF) + uint32_t(size) <= 0x1000) return false;
      return dbat_of(ea) < 0;
    };
    auto mis_count = [&](uint32_t a, int size, bool isst) {          // (by EA)
      if (size == 1 || !(a & uint32_t(size - 1))) return;
      uint32_t e = a + uint32_t(size) - 1;
      int k = (e >> 12) != (a >> 12) ? 4 : (e >> 5) != (a >> 5) ? 3 : (e >> 3) != (a >> 3) ? 2 : (e >> 2) != (a >> 2) ? 1 : 0;
      mis_n[k]++;
      if (isst) mis_n[5]++;
      if (k == 4 && (msr & MSR_DR)) {
        int b0 = dbat_of(a), b1 = dbat_of(e);
        mis_n[6]++;
        if (b0 < 0 && b1 < 0) mis_n[7]++; else if (b0 != b1) mis_n[8]++;
      }
    };
    // a page crossing's second word: (EA & ~3) + 4, the second page's first
    // byte, translated on its own; a fault is counted and taken (4.5.3)
    auto xpg_met = [&](uint32_t ea, int size) {
      if (!(msr & MSR_DR) || size < 2 || (ea & 0xFFF) + uint32_t(size) <= 0x1000) return;
      mis_n[size == 2 ? 14 : 11 + int((ea & 3) - 1)]++;
    };
    auto xpg2 = [&](uint32_t ea, Acc acc, uint32_t &a2) {
      if (xlate((ea & ~3u) + 4, acc, a2)) return true;
      uint32_t v = pc & 0xFFFF;
      if (v == 0x1100 || v == 0x1200) mis_n[9]++; else if (v == 0x300) mis_n[10]++;
      fault = true;
      return false;
    };
    auto ld = [&](uint32_t ea, int size, bool sgn, bool brev) -> uint32_t {
      uint32_t a, a2 = 0;
      xpg_met(ea, size);
      if (!xlate(ea, LOAD, a)) { fault = true; return 0; }
      bool xp = size > 1 && (ea & 0xFFF) + uint32_t(size) > 0x1000;
      uint32_t n1 = xp ? 0x1000 - (ea & 0xFFF) : uint32_t(size);      // bytes in the first page
      on_load(a & ~3u);
      if (xp) { if (!xpg2(ea, LOAD, a2)) return 0; on_load(a2); }      // the second page's word
      else if ((a & 3) + uint32_t(size) > 4) on_load((a & ~3u) + 4);  // the second word of a split
      mis_count(ea, size, false);
      uint32_t v;
      if (xp) { v = 0; for (uint32_t k = 0; k < uint32_t(size); k++) v = (v << 8) | rd8(k < n1 ? a + k : a2 + (k - n1)); }
      else if (size == 4 && !(a & 3)) v = rd32(a);
      else if (size == 2 && !(a & 1)) v = rd16(a);
      else if (size == 1) v = rd8(a);
      else { v = 0; for (int k = 0; k < size; k++) v = (v << 8) | rd8(a + k); }
      if (size == 4) return brev ? bswap32(v) : v;
      if (size == 2) { uint16_t h = uint16_t(v); if (brev) h = bswap16(h); return sgn ? uint32_t(int32_t(int16_t(h))) : h; }
      return v;
    };
    auto st = [&](uint32_t ea, int size, uint32_t v, bool brev) {
      uint32_t a, a2 = 0;
      xpg_met(ea, size);
      if (!xlate(ea, STORE, a)) { fault = true; return; }
      if (brev) v = size == 4 ? bswap32(v) : bswap16(uint16_t(v));
      if (size > 1 && (ea & 0xFFF) + uint32_t(size) > 0x1000) {       // across a page: the first page's
        uint32_t n1 = 0x1000 - (ea & 0xFFF);                           // bytes, then the second word's
        for (uint32_t k = 0; k < n1; k++) wr8(a + k, uint8_t(v >> (8 * (uint32_t(size) - 1 - k))));
        if (!xpg2(ea, STORE, a2)) return;                              // (the first page's bytes stay, PEM 6.1.4)
        for (uint32_t k = n1; k < uint32_t(size); k++) wr8(a2 + (k - n1), uint8_t(v >> (8 * (uint32_t(size) - 1 - k))));
        mis_count(ea, size, true);
        return;
      }
      mis_count(ea, size, true);
      if (size == 4 && !(a & 3)) wr32(a, v);
      else if (size == 2 && !(a & 1)) wr16(a, uint16_t(v));
      else if (size == 1) wr8(a, uint8_t(v));
      else for (int k = 0; k < size; k++) wr8(a + k, uint8_t(v >> (8 * (size - 1 - k))));
    };

    // ------------------------------------------------------- floating point
    int fpk = fp_kind(in);
    if (fpk) {
      if (!(msr & MSR_FP)) { exception(0x800, pc, 0); return; }        // 603e 4.5.8
      if (fpk == 1) {                                                   // loads and stores
        bool x = op == 31;
        int xo = x ? xo10 : 0;
        bool isst = x ? (xo & 128) || xo == 983 : op >= 52;
        bool dbl = x ? (xo & 64) && xo != 983 : (op & 2);
        bool upd = x ? (xo & 32) && xo != 983 : (op & 1);
        bool iw  = x && xo == 983;
        uint32_t ea = (upd ? gpr[ra] : A0()) + (x ? gpr[rb] : uint32_t(simm));
        fp_ls[isst]++;
        if (dbl && !(ea & 3) && (msr & MSR_DR) && (ea & 0xFFF) + 8u > 0x1000u) {
          int k0 = dbat_of(ea);
          mis_n[k0 < 0 ? 15 : dbat_of(ea + 7) == k0 ? 16 : 17]++;
        }
        if ((ea & 3) || (dbl && pg_cross(ea, 8))) { align_exc(ea); return; }   // 4.5.6, 4.5.6.1.1
        if (!isst) {
          uint32_t w0 = ld(ea, 4, false, false);
          if (fault) return;
          uint64_t v;
          if (dbl) { uint32_t w1 = ld(ea + 4, 4, false, false); if (fault) return; v = (uint64_t(w0) << 32) | w1; }
          else v = ppcfpu::lfs(w0);
          set_fpr(rd, v);
        } else {
          uint64_t v = fpr[rd];
          if (dbl) {
            st(ea, 4, uint32_t(v >> 32), false); if (fault) return;
            if (upd) gpr[ra] = ea;                                      // the first uop's (ppc_decode.sv)
            st(ea + 4, 4, uint32_t(v), false); if (fault) return;
          } else {
            st(ea, 4, iw ? uint32_t(v) : ppcfpu::stfs(v), false); if (fault) return;
          }
        }
        if (upd) gpr[ra] = ea;
        pc = npc;
        return;
      }
      fp_arith(in);
      if (fp_sets(in) && (msr & MSR_FE) && (fpscr & ppcfpu::FEX)) { exception(0x700, pc, 0x00100000); return; }
      pc = npc;
      return;
    }

    switch (op) {
      case 3: if (trap(gpr[ra], uint32_t(simm))) return; break;
      case 7: gpr[rd] = uint32_t(int64_t(int32_t(gpr[ra])) * simm); break;
      case 8: gpr[rd] = addc(~gpr[ra], uint32_t(simm), 1, co, ov); set_ca(co); break;
      case 10: set_cr_field(rd >> 2, (gpr[ra] < uimm ? 8 : gpr[ra] > uimm ? 4 : 2) | so()); break;
      case 11: { int32_t a = gpr[ra]; set_cr_field(rd >> 2, (a < simm ? 8 : a > simm ? 4 : 2) | so()); break; }
      case 12: case 13: gpr[rd] = addc(gpr[ra], uint32_t(simm), 0, co, ov); set_ca(co); if (op == 13) cr0(gpr[rd]); break;
      case 14: gpr[rd] = A0() + uint32_t(simm); break;
      case 15: gpr[rd] = A0() + (uint32_t(simm) << 16); break;
      case 16: {
        uint32_t tgt = (((in >> 1) & 1) ? 0 : pc) + uint32_t(int32_t(int16_t(in & 0xFFFC)));
        bool lk = in & 1;
        if (bc_cond(rd, ra, true)) branch(tgt, lk); else if (lk) lr = pc + 4;
        break;
      }
      case 17: exception(0xC00, pc + 4, 0); return;
      case 18: {
        int32_t li = int32_t(in << 6) >> 6;
        branch(((((in >> 1) & 1) ? 0 : pc) + uint32_t(li & ~3)), in & 1);
        break;
      }
      case 19:
        switch (xo10) {
          case 16: { uint32_t tgt = lr & ~3u; bool lk = in & 1; if (bc_cond(rd, ra, true)) branch(tgt, lk); else if (lk) lr = pc + 4; break; }
          case 528: { uint32_t tgt = ctr & ~3u; bool lk = in & 1; if (bc_cond(rd, ra, false)) branch(tgt, lk); else if (lk) lr = pc + 4; break; }
          case 50: msr = (srr1 & SRR1_KEEP) | (msr & 0x10000); pc = srr0 & ~3u; return;
          case 150: break;
          case 0: set_cr_field(rd >> 2, cr_field(ra >> 2)); break;
          case 33: case 129: case 193: case 225: case 257: case 289: case 417: case 449: {
            int tt = (in >> 6) & 15;
            bool a = cr_bit(ra), b = cr_bit(rb);
            set_cr_bit(rd, (tt >> ((a ? 2 : 0) + (b ? 1 : 0))) & 1);
            break;
          }
          default: illegal(); return;
        }
        break;
      case 20: case 21: case 23: {
        int sh = op == 23 ? (gpr[rb] & 31) : rb;
        uint32_t m = mask((in >> 6) & 31, (in >> 1) & 31), r = rotl(gpr[rd], sh);
        gpr[ra] = (r & m) | (op == 20 ? (gpr[ra] & ~m) : 0);
        if (rc) cr0(gpr[ra]);
        break;
      }
      case 24: gpr[ra] = gpr[rd] | uimm; break;
      case 25: gpr[ra] = gpr[rd] | (uimm << 16); break;
      case 26: gpr[ra] = gpr[rd] ^ uimm; break;
      case 27: gpr[ra] = gpr[rd] ^ (uimm << 16); break;
      case 28: gpr[ra] = gpr[rd] & uimm; cr0(gpr[ra]); break;
      case 29: gpr[ra] = gpr[rd] & (uimm << 16); cr0(gpr[ra]); break;
      case 32: case 33: case 34: case 35: case 40: case 41: case 42: case 43: {
        bool upd = op & 1;
        uint32_t ea = (upd ? gpr[ra] : A0()) + uint32_t(simm);
        int size = (op & 8) ? 2 : (op & 2) ? 1 : 4;
        uint32_t v = ld(ea, size, (op & 0x0E) == 0x0A, false);
        if (fault) return;
        gpr[rd] = v;
        if (upd) gpr[ra] = ea;
        break;
      }
      case 36: case 37: case 38: case 39: case 44: case 45: {
        bool upd = op & 1;
        uint32_t ea = (upd ? gpr[ra] : A0()) + uint32_t(simm);
        int size = (op & 8) ? 2 : (op & 2) ? 1 : 4;
        st(ea, size, gpr[rd], false);
        if (fault) return;
        if (upd) gpr[ra] = ea;
        break;
      }
      // lmw / stmw: a fault part way leaves the earlier words done (4.5.3)
      case 46: { uint32_t ea = A0() + uint32_t(simm); if (ea & 3) { align_exc(ea + 4); return; } for (int r = rd; r < 32; r++, ea += 4) { uint32_t v = ld(ea, 4, false, false); if (fault) return; gpr[r] = v; } break; }
      case 47: { uint32_t ea = A0() + uint32_t(simm); if (ea & 3) { align_exc(ea + 4); return; } for (int r = rd; r < 32; r++, ea += 4) { st(ea, 4, gpr[r], false); if (fault) return; } break; }
      case 31: {
        uint32_t a = gpr[ra], b = gpr[rb], s = gpr[rd];
        bool xo = true;
        uint32_t r = 0;
        bool wr = true, setca = false, cav = false, ovv = false;
        switch (xo9) {
          case 266: r = addc(a, b, 0, co, ovv); break;
          case 10:  r = addc(a, b, 0, cav, ovv); setca = true; break;
          case 138: r = addc(a, b, ca(), cav, ovv); setca = true; break;
          case 202: r = addc(a, 0, ca(), cav, ovv); setca = true; break;
          case 234: r = addc(a, 0xFFFFFFFF, ca(), cav, ovv); setca = true; break;
          case 40:  r = addc(~a, b, 1, co, ovv); break;
          case 8:   r = addc(~a, b, 1, cav, ovv); setca = true; break;
          case 136: r = addc(~a, b, ca(), cav, ovv); setca = true; break;
          case 200: r = addc(~a, 0, ca(), cav, ovv); setca = true; break;
          case 232: r = addc(~a, 0xFFFFFFFF, ca(), cav, ovv); setca = true; break;
          case 104: r = addc(~a, 0, 1, co, ovv); break;
          case 235: { int64_t p = int64_t(int32_t(a)) * int32_t(b); r = uint32_t(p); ovv = p != int64_t(int32_t(r)); break; }
          case 75:  { int64_t p = int64_t(int32_t(a)) * int32_t(b); r = uint32_t(uint64_t(p) >> 32); break; }
          case 11:  { uint64_t p = uint64_t(a) * b; r = uint32_t(p >> 32); break; }
          case 491:
            if (b == 0 && int32_t(a) >= 0) { r = 0; ovv = true; }
            else if (b == 0 || (b == 0xFFFFFFFF && a == 0x80000000)) { r = 0xFFFFFFFF; ovv = true; }
            else { r = uint32_t(int32_t(a) / int32_t(b)); ovv = false; }
            break;
          case 459:
            if (b == 0) { r = 0; ovv = true; } else { r = a / b; ovv = false; }
            break;
          default: xo = false;
        }
        if (xo) {
          (void)wr;
          if (setca) set_ca(cav);
          if (oe && xo9 != 75 && xo9 != 11) set_ov(ovv);
          gpr[rd] = r;
          if (rc) cr0(r);
          break;
        }
        switch (xo10) {
          case 0: { int32_t x = a, y = b; set_cr_field(rd >> 2, (x < y ? 8 : x > y ? 4 : 2) | so()); break; }
          case 32: set_cr_field(rd >> 2, (a < b ? 8 : a > b ? 4 : 2) | so()); break;
          case 4: if (trap(a, b)) return; break;
          case 28:  gpr[ra] = s & b;    if (rc) cr0(gpr[ra]); break;
          case 60:  gpr[ra] = s & ~b;   if (rc) cr0(gpr[ra]); break;
          case 124: gpr[ra] = ~(s | b); if (rc) cr0(gpr[ra]); break;
          case 284: gpr[ra] = ~(s ^ b); if (rc) cr0(gpr[ra]); break;
          case 316: gpr[ra] = s ^ b;    if (rc) cr0(gpr[ra]); break;
          case 412: gpr[ra] = s | ~b;   if (rc) cr0(gpr[ra]); break;
          case 444: gpr[ra] = s | b;    if (rc) cr0(gpr[ra]); break;
          case 476: gpr[ra] = ~(s & b); if (rc) cr0(gpr[ra]); break;
          case 24:  gpr[ra] = (b & 32) ? 0 : s << (b & 31); if (rc) cr0(gpr[ra]); break;
          case 536: gpr[ra] = (b & 32) ? 0 : s >> (b & 31); if (rc) cr0(gpr[ra]); break;
          case 792: case 824: {
            int n = xo10 == 824 ? rb : (b & 63);
            uint32_t res2; bool c;
            if (n >= 32) { res2 = int32_t(s) < 0 ? 0xFFFFFFFF : 0; c = int32_t(s) < 0; }
            else { res2 = uint32_t(int32_t(s) >> n); c = int32_t(s) < 0 && n && (s & ((1u << n) - 1)); }
            gpr[ra] = res2; set_ca(c); if (rc) cr0(res2);
            break;
          }
          case 26: { uint32_t n = 0; while (n < 32 && !((s >> (31 - n)) & 1)) n++; gpr[ra] = n; if (rc) cr0(n); break; }
          case 922: gpr[ra] = uint32_t(int32_t(int16_t(s))); if (rc) cr0(gpr[ra]); break;
          case 954: gpr[ra] = uint32_t(int32_t(int8_t(s)));  if (rc) cr0(gpr[ra]); break;
          case 23: case 87: case 279: case 343: case 55: case 119: case 311: case 375: {
            bool upd = xo10 & 32;
            uint32_t ea = (upd ? a : A0()) + b;
            int size = (xo10 & 256) ? 2 : (xo10 & 64) ? 1 : 4;
            uint32_t v = ld(ea, size, xo10 == 343 || xo10 == 375, false);
            if (fault) return;
            gpr[rd] = v;
            if (upd) gpr[ra] = ea;
            break;
          }
          case 20:  { if ((A0() + b) & 3) { align_exc(A0() + b); return; }
                      uint32_t v = ld(A0() + b, 4, false, false); if (fault) return; gpr[rd] = v; res = true; break; }
          case 534: { uint32_t v = ld(A0() + b, 4, false, true);  if (fault) return; gpr[rd] = v; break; }
          case 790: { uint32_t v = ld(A0() + b, 2, false, true);  if (fault) return; gpr[rd] = v; break; }
          case 151: case 215: case 407: case 183: case 247: case 439: {
            bool upd = xo10 & 32;
            uint32_t ea = (upd ? a : A0()) + b;
            int size = (xo10 & 256) ? 2 : (xo10 & 64) ? 1 : 4;
            st(ea, size, s, false);
            if (fault) return;
            if (upd) gpr[ra] = ea;
            break;
          }
          case 150: {
            // translated as a store whether or not it stores (4.5.3, table 5-8)
            uint32_t a;
            if ((A0() + b) & 3) { align_exc(A0() + b); return; }
            if (!xlate(A0() + b, STORE, a)) return;
            if (res) wr32(a, s);
            set_cr_field(0, (res ? 2 : 0) | so());
            res = false;
            break;
          }
          case 662: st(A0() + b, 4, s, true); if (fault) return; break;
          case 918: st(A0() + b, 2, s, true); if (fault) return; break;
          case 339: case 371: {
            int n = ra | (rb << 5);
            uint32_t v = 0;
            if (n == 1) v = xer; else if (n == 8) v = lr; else if (n == 9) v = ctr;
            else if (n == 26) v = srr0; else if (n == 27) v = srr1; else if (n == 287) v = pvr;
            else if (n == 22) v = dec_read();
            else if (n == 268 || n == 269) v = tb_read(n == 269);
            else if (n == 18) v = dsisr; else if (n == 19) v = dar;
            else if (n == 976) v = dmiss; else if (n == 977) v = dcmp;
            else if (n == 978 || n == 979) v = hash(n == 979);
            else if (n == 980) v = imiss; else if (n == 981) v = icmp;
            else if (known_spr(n)) v = spr[n];
            gpr[rd] = v;
            break;
          }
          case 467: {
            int n = ra | (rb << 5);
            if (n == 1) xer = s & XER_MASK; else if (n == 8) lr = s; else if (n == 9) ctr = s;
            else if (n == 26) srr0 = s; else if (n == 27) srr1 = s;
            else if (n == 22) dec_write(s);
            else if (n == 284 || n == 285) tb_write(n == 285, s);
            else if (n == 18) dsisr = s; else if (n == 19) dar = s;
            else if (n == 976) dmiss = s; else if (n == 977) dcmp = s;
            else if (n == 980) imiss = s; else if (n == 981) icmp = s;
            // HID0[ICFI] and [DCFI] clear as the invalidation begins (table 2-2): they read 0
            else if (n == 1008) spr[n] = s & ~0x00000C00u;
            else if (known_spr(n)) spr[n] = s;                 // HASH1/HASH2 are read-only
            break;
          }
          case 19:  gpr[rd] = cr; break;
          case 144: {
            int crm = (in >> 12) & 0xFF;
            for (int f = 0; f < 8; f++) if (crm & (0x80 >> f)) set_cr_field(f, s >> (28 - 4 * f));
            break;
          }
          case 83:  gpr[rd] = msr; break;
          case 146: msr = s & MSR_MASK; break;
          case 512: set_cr_field(rd >> 2, xer >> 28); xer &= 0x0FFFFFFF; break;
          // segment registers (PEM 8: mtsr, mtsrin, mfsr, mfsrin)
          case 210: sr[(in >> 16) & 15] = s; break;
          case 242: sr[b >> 28] = s; break;
          case 595: gpr[rd] = sr[(in >> 16) & 15]; break;
          case 659: gpr[rd] = sr[b >> 28]; break;
          // tlbie: both ways of the set, both TLBs (5.4.3.2)
          case 306: { uint32_t set = (b >> 12) & 31; for (int w = 0; w < 2; w++) itlb[set][w].v = dtlb[set][w].v = false; break; }
          case 978:  tlb_load(false, b); break;                 // tlbld
          case 1010: tlb_load(true, b);  break;                 // tlbli
          // Cache operations (3.7). Their effect on the caches is not modelled:
          // the generator uses them where the result does not depend on what
          // is cached. What is architectural is: dcbz zeroes the block; the
          // translation and protection of each, as a load (dcbst, dcbf) or a
          // store (dcbz, dcbi), table 7-31; dcbt and dcbtst never fault; dcbz
          // to a W or I page takes the alignment exception (3.7.4, 4.5.6); dcbi
          // is supervisor-only (3.7.1); icbi is not translated (3.7.8).
          case 1014: {                                                   // dcbz
            uint32_t ea = A0() + b, pa;
            if (!xlate(ea, STORE, pa)) return;
            if (last_wimg & 0xC) {
              dsisr = (((in >> 1) & 3) << 15) | (((in >> 6) & 1) << 14) | (((in >> 7) & 15) << 10) | (((in >> 21) & 31) << 5) | ((in >> 16) & 31);
              dar = ea;
              exception(0x600, pc, 0);
              return;
            }
            pa &= ~31u;
            for (int k = 0; k < 32; k++) wr8(pa + k, 0);
            break;
          }
          case 470:                                                      // dcbi
            if (msr & MSR_PR) { exception(0x700, pc, 0x00040000); return; }
            { uint32_t pa; if (!xlate(A0() + b, STORE, pa)) return; }
            break;
          case 54: case 86: { uint32_t pa; if (!xlate(A0() + b, LOAD, pa)) return; } break;   // dcbst dcbf
          case 246: case 278: { uint32_t pa; xlate(A0() + b, LOAD, pa, true); } break;        // dcbtst dcbt
          case 982: case 598: case 854: case 566:                        // icbi, sync, eieio, tlbsync
            break;
          default: illegal(); return;
        }
        break;
      }
      default: illegal(); return;
    }
    pc = npc;
  }
};
