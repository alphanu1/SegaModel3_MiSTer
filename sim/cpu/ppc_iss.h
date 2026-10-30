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
// Reference instruction-set model for the 603e integer slice (M0-A2).
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
// Not modelled yet (as in the RTL): translation, floating point, interrupts,
// timebase and decrementer reads, misaligned accesses.

#pragma once
#include <cstdint>
#include <functional>
#include <map>

struct PpcIss {
  uint32_t gpr[32] = {};
  uint32_t pc = 0xFFF00100, cr = 0, xer = 0, lr = 0, ctr = 0, msr = 0x40, srr0 = 0, srr1 = 0;
  bool     res = false;
  std::map<int, uint32_t> spr;
  uint32_t pvr = 0x00060101;

  std::function<uint8_t(uint32_t)>       rd8;
  std::function<void(uint32_t, uint8_t)> wr8;

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
    return (n >= 272 && n <= 275) || n == 18 || n == 19 || n == 25 || n == 1008 || n == 1009 ||
           n == 1010 || n == 282 || (n >= 976 && n <= 982) || (n >= 528 && n <= 543);
  }

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

  void exception(uint32_t vec, uint32_t save_pc, uint32_t srr1_bits) {
    srr0 = save_pc;
    srr1 = (msr & SRR1_KEEP) | srr1_bits;
    uint32_t ile = (msr >> 16) & 1;
    msr = (msr & ~0x0006EF33u) | ile;
    pc = ((msr >> 6) & 1 ? 0xFFF00000u : 0u) | vec;
  }

  // add with carry-in; returns sum, sets carry and overflow out-parameters
  static uint32_t addc(uint32_t a, uint32_t b, uint32_t c, bool &co, bool &ov) {
    uint64_t s = uint64_t(a) + b + c;
    uint32_t r = uint32_t(s);
    co = s >> 32;
    ov = ((a ^ r) & (b ^ r)) >> 31;
    return r;
  }

  void step() {
    uint32_t in = rd32(pc);
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
    auto ld = [&](uint32_t ea, int size, bool sgn, bool brev) -> uint32_t {
      if (size == 4) { uint32_t v = rd32(ea & ~3u); return brev ? bswap32(v) : v; }
      if (size == 2) { uint16_t h = rd16(ea & ~1u); if (brev) h = bswap16(h); return sgn ? uint32_t(int32_t(int16_t(h))) : h; }
      return rd8(ea);
    };
    auto st = [&](uint32_t ea, int size, uint32_t v, bool brev) {
      if (size == 4) wr32(ea & ~3u, brev ? bswap32(v) : v);
      else if (size == 2) wr16(ea & ~1u, brev ? bswap16(uint16_t(v)) : uint16_t(v));
      else wr8(ea, uint8_t(v));
    };

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
        gpr[rd] = ld(ea, size, (op & 0x0E) == 0x0A, false);
        if (upd) gpr[ra] = ea;
        break;
      }
      case 36: case 37: case 38: case 39: case 44: case 45: {
        bool upd = op & 1;
        uint32_t ea = (upd ? gpr[ra] : A0()) + uint32_t(simm);
        int size = (op & 8) ? 2 : (op & 2) ? 1 : 4;
        st(ea, size, gpr[rd], false);
        if (upd) gpr[ra] = ea;
        break;
      }
      case 46: { uint32_t ea = A0() + uint32_t(simm); for (int r = rd; r < 32; r++, ea += 4) gpr[r] = rd32(ea & ~3u); break; }
      case 47: { uint32_t ea = A0() + uint32_t(simm); for (int r = rd; r < 32; r++, ea += 4) wr32(ea & ~3u, gpr[r]); break; }
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
            gpr[rd] = ld(ea, size, xo10 == 343 || xo10 == 375, false);
            if (upd) gpr[ra] = ea;
            break;
          }
          case 20:  gpr[rd] = ld(A0() + b, 4, false, false); res = true; break;
          case 534: gpr[rd] = ld(A0() + b, 4, false, true); break;
          case 790: gpr[rd] = ld(A0() + b, 2, false, true); break;
          case 151: case 215: case 407: case 183: case 247: case 439: {
            bool upd = xo10 & 32;
            uint32_t ea = (upd ? a : A0()) + b;
            int size = (xo10 & 256) ? 2 : (xo10 & 64) ? 1 : 4;
            st(ea, size, s, false);
            if (upd) gpr[ra] = ea;
            break;
          }
          case 150: {
            if (res) st(A0() + b, 4, s, false);
            set_cr_field(0, (res ? 2 : 0) | so());
            res = false;
            break;
          }
          case 662: st(A0() + b, 4, s, true); break;
          case 918: st(A0() + b, 2, s, true); break;
          case 339: case 371: {
            int n = ra | (rb << 5);
            uint32_t v = 0;
            if (n == 1) v = xer; else if (n == 8) v = lr; else if (n == 9) v = ctr;
            else if (n == 26) v = srr0; else if (n == 27) v = srr1; else if (n == 287) v = pvr;
            else if (known_spr(n)) v = spr[n];
            gpr[rd] = v;
            break;
          }
          case 467: {
            int n = ra | (rb << 5);
            if (n == 1) xer = s & XER_MASK; else if (n == 8) lr = s; else if (n == 9) ctr = s;
            else if (n == 26) srr0 = s; else if (n == 27) srr1 = s;
            else if (known_spr(n)) spr[n] = s;
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
          case 54: case 86: case 246: case 278: case 470: case 982: case 598: case 854:
          case 566: case 306: case 210: case 242: case 978: case 1010:
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
