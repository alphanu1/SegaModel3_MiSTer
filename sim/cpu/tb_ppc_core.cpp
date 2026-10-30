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
// ppc_core against the reference model (ppc_iss.h), instruction by instruction.
//
// Each seed builds a random program over the integer slice, loads it into two
// copies of memory, and runs the RTL with random wait states on both ports.
// Every time the RTL retires the last uop of an instruction, the model steps
// once and the two are compared: the retired PC, all 32 GPRs, CR, XER, LR, CTR,
// MSR, SRR0, SRR1. Data memory is compared when the program reaches its final
// `b .`. Any difference stops the run with both states printed.
//
//   tb_ppc_core [seeds] [first_seed] [length] [max_wait]

#include "Vppc_core.h"
#include "verilated.h"
#include "ppc_iss.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

namespace {

// ---------------------------------------------------------------- memory
struct Mem {
  std::vector<uint8_t> lo = std::vector<uint8_t>(0x200000);     // 0x00000000 - 0x001FFFFF
  std::vector<uint8_t> hi = std::vector<uint8_t>(0x10000);      // 0xFFF00000 - 0xFFF0FFFF
  uint8_t *at(uint32_t a) {
    if (a < lo.size()) return &lo[a];
    if (a >= 0xFFF00000u && a - 0xFFF00000u < hi.size()) return &hi[a - 0xFFF00000u];
    return nullptr;
  }
  uint8_t rd8(uint32_t a) { uint8_t *p = at(a); return p ? *p : 0xFF; }
  void wr8(uint32_t a, uint8_t v) { if (uint8_t *p = at(a)) *p = v; }
  uint32_t rd32(uint32_t a) { return (rd8(a) << 24) | (rd8(a + 1) << 16) | (rd8(a + 2) << 8) | rd8(a + 3); }
  void wr32(uint32_t a, uint32_t v) { wr8(a, v >> 24); wr8(a + 1, v >> 16); wr8(a + 2, v >> 8); wr8(a + 3, v); }
};

// -------------------------------------------------------------- encoders
uint32_t D(int op, int rt, int ra, uint32_t imm) { return (op << 26) | (rt << 21) | (ra << 16) | (imm & 0xFFFF); }
uint32_t X(int rt, int ra, int rb, int xo, int rc = 0) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (xo << 1) | rc; }
uint32_t XO(int rt, int ra, int rb, int xo9, int oe, int rc) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (oe << 10) | (xo9 << 1) | rc; }
uint32_t M(int op, int rs, int ra, int sh, int mb, int me, int rc) { return (op << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1) | rc; }
uint32_t XL(int bt, int ba, int bb, int xo, int lk = 0) { return (19u << 26) | (bt << 21) | (ba << 16) | (bb << 11) | (xo << 1) | lk; }
uint32_t SPR(int rt, int spr, int xo) { return X(rt, spr & 31, spr >> 5, xo); }
uint32_t B(int32_t off, int aa = 0, int lk = 0) { return (18u << 26) | (uint32_t(off) & 0x03FFFFFC) | (aa << 1) | lk; }
uint32_t BC(int bo, int bi, int32_t off, int lk = 0) { return (16u << 26) | (bo << 21) | (bi << 16) | (uint32_t(off) & 0xFFFC) | lk; }

constexpr uint32_t CODE = 0x1000, DATA = 0x100000, END_INSN = 0x48000000;

// --------------------------------------------------------------- program
struct Gen {
  std::mt19937 rng;
  std::vector<uint32_t> code;
  explicit Gen(uint32_t seed) : rng(seed) {}

  uint32_t r32() { return rng(); }
  int rnd(int n) { return int(rng() % n); }
  // destinations avoid the bases (r1 r2 r3), the update base (r6) and r9 (handlers)
  int dst() { static const int ok[] = {0,4,5,7,8,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}; return ok[rnd(27)]; }
  int src() { return rnd(32); }
  uint32_t edge() {
    static const uint32_t e[] = {0, 1, 0xFFFFFFFF, 0x80000000, 0x7FFFFFFF, 2, 0xFFFFFFFE, 0x10000};
    return rnd(3) ? r32() : e[rnd(8)];
  }
  void emit(uint32_t w) { code.push_back(w); }
  void li32(int r, uint32_t v) { emit(D(15, r, 0, v >> 16)); emit(D(24, r, r, v & 0xFFFF)); }

  void alu_imm() {
    static const int ops[] = {14, 15, 12, 13, 8, 7, 24, 25, 26, 27, 28, 29};
    int op = ops[rnd(12)];
    if (op >= 24) emit(D(op, src(), dst(), r32()));
    else          emit(D(op, dst(), src(), r32()));
  }
  void alu_xo() {
    static const int ops[] = {266, 10, 138, 202, 234, 40, 8, 136, 200, 232, 104, 235, 75, 11, 491, 459};
    int x = ops[rnd(16)];
    int oe = (x == 75 || x == 11) ? 0 : rnd(2);
    emit(XO(dst(), src(), src(), x, oe, rnd(2)));
  }
  void alu_x() {
    static const int ops[] = {28, 60, 124, 284, 316, 412, 444, 476, 24, 536, 792, 824, 26, 922, 954};
    emit(X(src(), dst(), src(), ops[rnd(15)], rnd(2)));
  }
  void rotate() {
    static const int ops[] = {20, 21, 23};
    emit(M(ops[rnd(3)], src(), dst(), rnd(32), rnd(32), rnd(32), rnd(2)));
  }
  void compare() {
    int bf = rnd(8) << 2;
    switch (rnd(4)) {
      case 0: emit(X(bf, src(), src(), 0)); break;
      case 1: emit(X(bf, src(), src(), 32)); break;
      case 2: emit(D(11, bf, src(), r32())); break;
      default: emit(D(10, bf, src(), r32())); break;
    }
  }
  void crops() {
    static const int ops[] = {33, 129, 193, 225, 257, 289, 417, 449};
    switch (rnd(5)) {
      case 0: emit(XL(rnd(8) << 2, rnd(8) << 2, 0, 0)); break;                   // mcrf
      case 1: emit((31u << 26) | (src() << 21) | (uint32_t(rnd(256)) << 12) | (144u << 1)); break; // mtcrf
      case 2: emit(X(dst(), 0, 0, 19)); break;                                   // mfcr
      case 3: emit(X(rnd(8) << 2, 0, 0, 512)); break;                            // mcrxr
      default: emit(XL(rnd(32), rnd(32), rnd(32), ops[rnd(8)])); break;
    }
  }
  void spr() {
    static const int sprs[] = {1, 8, 9, 272, 273, 274, 275, 18, 19, 25, 1008, 976, 982, 528, 535, 543, 26, 27, 287, 999};
    int n = sprs[rnd(20)];
    if (rnd(2)) emit(SPR(dst(), n, 339));
    else if (n != 287) emit(SPR(src(), n, 467));
  }
  void msr() {
    if (rnd(2)) emit(X(dst(), 0, 0, 83));
    else { int r = src(); emit(D(24, r, 7, 0x0040)); emit(X(7, 0, 0, 146)); }   // keep MSR[IP]
  }
  void mem() {
    int base = rnd(2) ? 1 : 2;
    int k = rnd(14);
    if (k < 6) {                                         // D-form loads and stores
      static const int ld[] = {32, 34, 40, 42}, st[] = {36, 38, 44};
      bool isld = rnd(2);
      int op = isld ? ld[rnd(4)] : st[rnd(3)];
      int sz = (op & 8) ? 2 : (op & 2) ? 1 : 4;
      uint32_t off = (r32() % 0x7F00) & ~uint32_t(sz - 1);
      emit(D(op, isld ? dst() : src(), base, off));
    } else if (k < 8) {                                  // update forms through r6
      static const int ops[] = {33, 35, 41, 43, 37, 39, 45};
      int op = ops[rnd(7)];
      int sz = (op & 8) ? 2 : (op & 2) ? 1 : 4;
      li32(6, DATA + ((r32() % 0x7000) & ~3u));
      int rt = dst(); if (rt == 6) rt = 7;
      emit(D(op, (op & 4) ? src() : rt, 6, (r32() % 0x400) & ~uint32_t(sz - 1)));
    } else if (k < 11) {                                 // indexed through r3
      static const int ops[] = {23, 87, 279, 343, 151, 215, 407, 534, 790, 662, 918};
      int x = ops[rnd(11)];
      bool isst = (x == 151 || x == 215 || x == 407 || x == 662 || x == 918);
      emit(D(14, 3, 0, (r32() % 0x3F00) & ~3u));        // li r3, aligned offset
      emit(X(isst ? src() : dst(), base, 3, x));
    } else if (k < 12) {                                 // lwarx / stwcx.
      emit(D(14, 3, 0, (r32() % 0x3F00) & ~3u));
      if (rnd(2)) emit(X(dst(), base, 3, 20));
      emit(X(src(), base, 3, 150, 1));
    } else {                                             // lmw / stmw
      int r = 24 + rnd(8);
      emit(D(rnd(2) ? 46 : 47, r, 1, (r32() % 0x7000) & ~3u));
    }
  }
  void muldiv() {
    li32(10, edge()); li32(11, edge());
    static const int ops[] = {235, 75, 11, 491, 459};
    int x = ops[rnd(5)];
    emit(XO(dst(), 10, 11, x, (x == 75 || x == 11) ? 0 : rnd(2), rnd(2)));
  }
  void filler(int n) { for (int i = 0; i < n; i++) alu_imm(); }
  void branch() {
    int k = rnd(6);
    int skip = 1 + rnd(3);
    switch (k) {
      case 0: emit(B(4 * (skip + 1))); filler(skip); break;
      case 1: case 2: emit(BC(rnd(32), rnd(32), 4 * (skip + 1), rnd(2))); filler(skip); break;
      case 3: {                                          // bclr through a computed LR
        emit(BC(20, 31, 4, 1));                          // bcl 20,31,$+4: LR = next
        emit(SPR(12, 8, 339));                           // mflr r12
        emit(D(14, 12, 12, 20));                         // addi r12,r12,20
        emit(SPR(12, 8, 467));                           // mtlr r12
        emit(XL(rnd(32), rnd(32), 0, 16, rnd(2)));       // bclr / bclrl
        filler(1);
        break;
      }
      case 4: {                                          // bcctr through a computed CTR
        emit(BC(20, 31, 4, 1));
        emit(SPR(12, 8, 339));
        emit(D(14, 12, 12, 20));
        emit(SPR(12, 9, 467));                           // mtctr r12
        emit(XL(rnd(32) | 4, rnd(32), 0, 528, rnd(2)));  // bcctr (BO_2 set)
        filler(1);
        break;
      }
      default: {                                         // bdnz-style with a small CTR
        emit(D(14, 12, 0, rnd(3)));
        emit(SPR(12, 9, 467));
        emit(BC(rnd(2) ? 16 : 18, rnd(32), 4 * (skip + 1)));
        filler(skip);
        break;
      }
    }
  }
  void exc() {
    switch (rnd(4)) {
      case 0: emit(17u << 26 | 2); break;                               // sc
      case 1: emit(D(3, rnd(32), src(), r32())); break;                 // twi
      case 2: emit(X(rnd(32), src(), src(), 4)); break;                 // tw
      default: emit(rnd(2) ? 0xFC00002Au : (1u << 26)); break;          // FP / reserved: illegal
    }
  }

  void build(int len) {
    for (int r = 0; r < 32; r++) if (r != 1 && r != 2 && r != 3) li32(r, edge());
    li32(1, DATA); li32(2, DATA + 0x8000); emit(D(14, 3, 0, 0x100));
    li32(12, r32()); emit(SPR(12, 9, 467)); emit(SPR(12, 8, 467));
    li32(12, r32() & 0xE000007F); emit(SPR(12, 1, 467));
    while (int(code.size()) < len) {
      int k = rnd(100);
      if      (k < 20) alu_imm();
      else if (k < 34) alu_xo();
      else if (k < 46) alu_x();
      else if (k < 54) rotate();
      else if (k < 60) compare();
      else if (k < 66) crops();
      else if (k < 70) spr();
      else if (k < 72) msr();
      else if (k < 86) mem();
      else if (k < 89) muldiv();
      else if (k < 97) branch();
      else exc();
    }
    emit(END_INSN);
  }
};

void install(Mem &m, const std::vector<uint32_t> &code, std::mt19937 &rng) {
  for (uint32_t a = DATA; a < DATA + 0x10000; a++) m.wr8(a, uint8_t(rng()));
  m.wr32(0xFFF00100, B(CODE, 1));                                 // ba CODE
  for (uint32_t base : {0x00000000u, 0xFFF00000u}) {
    // program exception: skip the faulting instruction
    m.wr32(base + 0x700, SPR(9, 26, 339));                        // mfsrr0 r9
    m.wr32(base + 0x704, D(14, 9, 9, 4));                         // addi r9,r9,4
    m.wr32(base + 0x708, SPR(9, 26, 467));                        // mtsrr0 r9
    m.wr32(base + 0x70C, XL(0, 0, 0, 50));                        // rfi
    m.wr32(base + 0xC00, XL(0, 0, 0, 50));                        // sc: rfi (SRR0 is pc+4)
  }
  for (size_t i = 0; i < code.size(); i++) m.wr32(CODE + 4 * i, code[i]);
}

// ------------------------------------------------------------- one seed
bool run(uint32_t seed, int len, int max_wait, uint64_t &insns, uint64_t &cycles) {
  Gen g(seed);
  g.build(len);
  std::mt19937 drng(seed ^ 0x9E3779B9u);
  Mem rtl_mem, iss_mem;
  {
    std::mt19937 a(seed * 7 + 1), b(seed * 7 + 1);
    install(rtl_mem, g.code, a);
    install(iss_mem, g.code, b);
  }

  PpcIss iss;
  iss.rd8 = [&](uint32_t a) { return iss_mem.rd8(a); };
  iss.wr8 = [&](uint32_t a, uint8_t v) { iss_mem.wr8(a, v); };

  Vppc_core top;
  uint32_t gpr[32] = {};
  std::mt19937 lat(seed ^ 0x51ED270Bu);
  auto wait_for = [&]() { return max_wait ? 1 + int(lat() % max_wait) : 1; };

  bool i_pend = false, d_pend = false;
  int i_cnt = 0, d_cnt = 0;
  uint32_t i_word = 0, d_word = 0;

  top.clk = 0; top.rst = 1; top.i_valid = 0; top.d_valid = 0; top.tb_tick = 0;
  for (int i = 0; i < 4; i++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;

  uint64_t idle = 0;
  for (cycles = 0; cycles < 50ull * len + 100000; cycles++) {
    // responses for this cycle
    top.i_valid = i_pend && i_cnt == 0;
    top.i_data  = i_word;
    top.d_valid = d_pend && d_cnt == 0;
    top.d_rdata = d_word;
    top.clk = 0; top.eval();

    bool ireq = top.i_req, dreq = top.d_req, dwe = top.d_we;
    uint32_t ia = top.i_addr, da = top.d_addr, dwd = top.d_wdata;
    int dbe = top.d_be;
    bool i_took = top.i_valid, d_took = top.d_valid;

    top.clk = 1; top.eval();

    // memory side of the edge
    if (i_took) i_pend = false;
    if (d_took) d_pend = false;
    if (i_pend) i_cnt--;
    if (d_pend) d_cnt--;
    if (ireq) { i_pend = true; i_cnt = wait_for() - 1; i_word = rtl_mem.rd32(ia); }
    if (dreq) {
      d_pend = true; d_cnt = wait_for() - 1;
      if (dwe) {
        for (int b = 0; b < 4; b++) if (dbe & (8 >> b)) rtl_mem.wr8(da + b, uint8_t(dwd >> (24 - 8 * b)));
      } else d_word = rtl_mem.rd32(da);
    }

    // retire
    if (!top.rt_valid) { if (++idle > 2000) { printf("seed %u: no retire for 2000 cycles at cycle %llu\n", seed, (unsigned long long)cycles); return false; } continue; }
    idle = 0;
    if (top.rt_gpr_we) gpr[top.rt_gpr] = top.rt_gpr_val;
    if (!top.rt_last) continue;

    uint32_t rpc = top.rt_pc;
    if (rpc != iss.pc) {
      printf("seed %u: retired pc %08X, model pc %08X (insn %08X)\n", seed, rpc, iss.pc, top.rt_insn);
      return false;
    }
    if (top.rt_insn == END_INSN) {
      for (uint32_t a = DATA; a < DATA + 0x10000; a++)
        if (rtl_mem.rd8(a) != iss_mem.rd8(a)) { printf("seed %u: memory %08X rtl %02X model %02X\n", seed, a, rtl_mem.rd8(a), iss_mem.rd8(a)); return false; }
      return true;
    }
    iss.step();
    insns++;

    bool bad = false;
    for (int r = 0; r < 32; r++) if (gpr[r] != iss.gpr[r]) { printf("  r%d rtl %08X model %08X\n", r, gpr[r], iss.gpr[r]); bad = true; }
    auto chk = [&](const char *n, uint32_t a, uint32_t b) { if (a != b) { printf("  %s rtl %08X model %08X\n", n, a, b); bad = true; } };
    chk("cr", top.rt_cr, iss.cr);
    chk("xer", top.rt_xer, iss.xer);
    chk("lr", top.rt_lr, iss.lr);
    chk("ctr", top.rt_ctr, iss.ctr);
    chk("msr", top.rt_msr, iss.msr);
    chk("srr0", top.rt_srr0, iss.srr0);
    chk("srr1", top.rt_srr1, iss.srr1);
    if (bad) { printf("seed %u: state differs after pc %08X insn %08X (instruction %llu)\n", seed, rpc, top.rt_insn, (unsigned long long)insns); return false; }
  }
  printf("seed %u: did not reach the end in %llu cycles\n", seed, (unsigned long long)cycles);
  return false;
}

} // namespace

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int seeds = argc > 1 ? atoi(argv[1]) : 200;
  uint32_t first = argc > 2 ? strtoul(argv[2], nullptr, 0) : 1;
  int len = argc > 3 ? atoi(argv[3]) : 1500;
  int max_wait = argc > 4 ? atoi(argv[4]) : 4;
  uint64_t insns = 0, cycles_all = 0;
  int pass = 0;
  for (int s = 0; s < seeds; s++) {
    uint64_t cyc = 0;
    if (run(first + s, len, max_wait, insns, cyc)) pass++;
    else { printf("FAIL\n"); return 1; }
    cycles_all += cyc;
  }
  printf("PASS: %d seeds, %llu instructions compared against the model, %llu cycles (max wait %d)\n",
         pass, (unsigned long long)insns, (unsigned long long)cycles_all, max_wait);
  return 0;
}
