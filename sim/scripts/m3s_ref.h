// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Reference model of the NCR 53C810 as Model 3 uses it (M0-F): the register
// window, PCI configuration as far as the games touch it, and the SCRIPTS
// processor for the captured subset (memory move; INT, behind int_op).
// Everything runs at once when it is started, as both emulators do; the RTL
// bench waits for the RTL to finish before the next CPU access.
//
// Three forms:
//   HW    the LSI53C810A manual (sim/scripts/notes.md), which the RTL is built to
//   SM    Supermodel 036af1c's 53C810.cpp, where it differs from HW in what the
//         games can see: DCNTL.STD stays set after an interrupt, a single step
//         runs even when halted, a DSTAT read clears SSI only
//   MAME  MAME 0.289's 53c810.cpp: a single step sets ISTAT.SIP as well as DIP,
//         DSTAT has no DFE and is never cleared, an ISTAT read releases the
//         interrupt and a DSTAT read does not, the interrupt ignores DIEN, and a
//         DCNTL write with SSM or STD steps
// The SM and MAME forms exist to replay each emulator's capture exactly, which
// proves each difference is understood; the HW form is what the RTL matches.
//
// Bus words are big-endian: lane 3 (be bit 3) is bits 31:24, the byte at
// offset 0. The chip's registers are little-endian, so register byte N sits
// in lane 3 - (N & 3) of word N & ~3. A SCRIPTS word as the chip sees it is
// the bus word byte-reversed; a memory move copies bus words unchanged
// (byte addresses are preserved across the MPC105, as both emulators do).

#pragma once
#include <cstdint>
#include <functional>
#include <vector>

struct M3sRef {
  enum Form { HW, SM, MAME };
  Form form = HW;
  bool int_op = true;     // decode INT (RTL parameter INT_OP)
  bool rd_all = false;    // TEMP, DBC/DCMD and DSPS readable (RTL parameter RD_ALL)

  std::function<uint32_t(uint32_t)> rd;          // memory, word-aligned bus words
  std::function<void(uint32_t, uint32_t)> wr;

  // operating registers
  uint32_t dsp = 0, dsps = 0, temp = 0, dbc = 0;
  uint8_t dcmd = 0, dmode = 0, dien = 0, dcntl = 0;
  uint8_t dstat = 0;       // SSI 0x08, SIR 0x04, IID 0x01 (DFE is live, 0x80)
  uint8_t istat = 0;       // the CPU's bits: SRST 0x40, SIGP 0x20, SEM 0x10 (DIP is live). MAME: the whole register
  bool halted = false, irq_l = false;
  // PCI configuration
  uint8_t cmd = 0, lat = 0;
  uint32_t bar1 = 0;       // bits 31:8

  // what the last start did
  struct Insn { uint32_t dsp, w0, w1, w2; int n; };
  struct Move { uint32_t src, dst, len; };
  std::vector<Insn> insns;
  std::vector<Move> moves;
  uint64_t words = 0;

  static uint32_t swap(uint32_t x) {
    return (x << 24) | ((x & 0xFF00) << 8) | ((x >> 8) & 0xFF00) | (x >> 24);
  }

  void reset_ops() {
    dsp = dsps = temp = dbc = 0;
    dcmd = dmode = dien = dcntl = 0;
    dstat = 0;
    halted = false;
    irq_l = false;
  }
  void reset() { reset_ops(); istat = 0; cmd = 0; lat = 0; bar1 = 0; }

  bool irq() const { return irq_l && (form != HW || !(dcntl & 0x02)); }   // DCNTL.IRQD masks the pin

  // ---------------------------------------------------------------- PCI configuration
  uint32_t cfg_read(unsigned idx) const {
    switch (idx) {
      case 0: return 0x00011000;                    // device 0001 (53C810), vendor 1000 (LSI)
      case 1: return cmd;
      case 3: return uint32_t(lat) << 8;
      case 5: return bar1 << 8;
      default: return 0;
    }
  }
  void cfg_write(unsigned idx, uint32_t v, unsigned be = 0xF) {   // be: PCI lanes, bit 0 = bits 7:0
    if (idx == 1 && (be & 1)) cmd = v & 0x07;
    if (idx == 3 && (be & 2)) lat = (v >> 8) & 0xFF;
    if (idx == 5) {
      for (int i = 1; i < 4; i++)
        if (be & (1 << i)) bar1 = (bar1 & ~(0xFFu << (8 * (i - 1)))) | (((v >> (8 * i)) & 0xFF) << (8 * (i - 1)));
    }
  }

  // ---------------------------------------------------------------- register window
  static unsigned lane(unsigned reg) { return 3 - (reg & 3); }
  uint8_t dip() const { return dstat ? 1 : 0; }

  uint32_t read(unsigned off, unsigned be) {
    uint32_t v = 0;
    auto put = [&](unsigned reg, uint8_t b) { if (be & (1u << lane(reg))) v |= uint32_t(b) << (8 * lane(reg)); };
    auto put32 = [&](unsigned base, uint32_t le) { for (int i = 0; i < 4; i++) put(base + i, uint8_t(le >> (8 * i))); };
    switch (off & 0x7C) {
      case 0x0C:
        if (be & 8) {
          if (form == MAME) { put(0x0C, dstat); istat &= ~1; }
          else {
            put(0x0C, 0x80 | dstat);
            dstat = (form == SM) ? (dstat & ~0x08) : 0;
            irq_l = false;
          }
        }
        break;
      case 0x14:
        if (be & 8) {
          if (form == MAME) { put(0x14, istat); irq_l = false; }
          else put(0x14, (istat & 0x70) | dip());
        }
        break;
      case 0x1C: if (rd_all) put32(0x1C, temp); break;
      case 0x24: if (rd_all) put32(0x24, (dbc & 0xFFFFFF) | (uint32_t(dcmd) << 24)); break;
      case 0x2C: put32(0x2C, dsp); break;
      case 0x30: if (rd_all) put32(0x30, dsps); break;
      case 0x38: put(0x38, dmode); put(0x39, dien); put(0x3B, dcntl); break;
      default: break;
    }
    return v;
  }

  void write(unsigned off, unsigned be, uint32_t v) {
    auto byte = [&](unsigned reg) { return uint8_t(v >> (8 * lane(reg))); };
    auto has = [&](unsigned reg) { return (be >> lane(reg)) & 1; };
    switch (off & 0x7C) {
      case 0x14:
        if (has(0x14)) {
          uint8_t d = byte(0x14);
          if (form == MAME) { istat = d; break; }
          if (d & 0x40) { reset_ops(); istat = 0x40; break; }   // SRST: operating registers to defaults while set
          istat = d & 0x70;
        }
        break;
      case 0x2C: {
        for (unsigned i = 0; i < 4; i++)
          if (has(0x2C + i)) dsp = (dsp & ~(0xFFu << (8 * i))) | (uint32_t(byte(0x2C + i)) << (8 * i));
        dsp &= ~3u;
        if (has(0x2F)) {                     // writing the top byte starts SCRIPTS unless DMODE.MAN
          halted = false;
          // HW: the run honours SSM. Both emulators run on regardless (Supermodel's
          // "VF3 writes DSP and then sets single step" note comes from this)
          if (!(dmode & 1)) run(form == HW ? (dcntl & 0x10) != 0 : false);
        }
        break;
      }
      case 0x38:
        if (has(0x38)) dmode = byte(0x38);
        if (has(0x39)) dien = byte(0x39);
        if (has(0x3B)) {
          dcntl = byte(0x3B);
          if (form == MAME) {
            if ((dcntl & 0x14) && !halted) run(true);
            else if ((dcntl & 0x04) && !halted) run(false);
          } else if (dcntl & 0x04) {
            if (!halted || (form == SM && (dcntl & 0x10))) run((dcntl & 0x10) != 0);
          }
        }
        break;
      default: break;
    }
  }

  // ---------------------------------------------------------------- SCRIPTS
  uint32_t fetch() { uint32_t w = swap(rd(dsp)); dsp += 4; return w; }

  void interrupt(uint8_t bits) {
    if (form == MAME) {
      // MAME: single step ORs 3 into ISTAT and 8 into DSTAT; INT 1 and 4
      istat |= (bits & 0x08) ? 0x03 : 0x01;
      dstat |= bits;
      irq_l = true;
      return;
    }
    dstat |= bits;
    if (form != SM) dcntl &= ~0x04;                  // STD stays set until an interrupt
    if (dstat & dien & 0x0D) irq_l = true;
  }

  // Runs from DSP until an interrupt: one instruction when single is set.
  void run(bool single) {
    insns.clear(); moves.clear();
    for (int guard = 0; guard < 1 << 20; guard++) {
      Insn in{dsp, 0, 0, 0, 2};
      uint32_t w0 = fetch(), w1 = fetch();
      in.w0 = w0; in.w1 = w1;
      dcmd = w0 >> 24; dbc = w0 & 0xFFFFFF; dsps = w1;
      bool mmov = (dcmd & 0xE0) == 0xC0 && (form != HW || (dcmd & 0x1E) == 0);
      bool intr = int_op && (dcmd & 0xF8) == 0x98;
      if (mmov) {
        temp = fetch(); in.w2 = temp; in.n = 3;
        insns.push_back(in);
        if (form == HW && (dbc == 0 || ((dsps | temp | dbc) & 3))) {   // zero count: manual; sub-word: not built
          halted = true; interrupt(0x01); return;
        }
        moves.push_back({dsps, temp, dbc});
        uint32_t n = (dbc + 3) >> 2;
        for (uint32_t i = 0; i < n; i++) { wr(temp, rd(dsps)); dsps += 4; temp += 4; }
        words += n;
        dbc = 0;
      } else if (intr) {
        insns.push_back(in);
        if (dbc & 0x00360000) { halted = true; interrupt(0x01); return; }   // carry, IF, data or phase compare: not built
        if (dbc & 0x00080000) {
          if (form == SM) dsp -= 8;           // Supermodel leaves DSP at the INT
          halted = true;
          interrupt(single ? 0x0C : 0x04);
          return;
        }
      } else {
        insns.push_back(in);
        halted = true; interrupt(0x01); return;          // illegal instruction
      }
      if (single) { interrupt(0x08); return; }
    }
  }
};
