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
// tb_m3_bus_tg (make -f sim/system/system.mk test_bus_tg): m3_bus's tile generator bus timing (TGT, study R152) against Supermodel's model of it
// (Supermodel 0d1dd3d, CTileGen::BusAccessCycles / BusReadCycles, gm_matthew's measurements), restated
// here in its own units (1/16 of a clock; one VRAM clock is 16 * MHz / 32 of them).
//
// The CPU (master 0) makes random single-word accesses: tile RAM writes, palette writes, tile generator
// register writes, CROM reads, I/O reads, work RAM reads and writes, with random gaps; the SDRAM port
// answers after a random delay; the line events come every 656 pixels (16 MHz), lines 0-423, 0-383 shown.
// For each access the cycle it leaves the decode (B_DEC) is compared with the cycle Supermodel's model takes
// it: for a write, when the chip takes it; for a read, when every write before it has landed. Work RAM
// accesses never wait. The two differ by the RTL's VRAM-clock steps (a 32 MHz enable on the clock) and its
// 13-clock landing (Supermodel 12.6); the bench reports the spread and fails past TOL clocks.
//
// The RTL carries the busy and landing times in sixteenths of a clock, as Supermodel, so every access is
// taken in Supermodel's clock; an access in a fetch slot one clock before it (the slot's start is a fraction
// into a clock: the RTL takes it in that clock, Supermodel's rounding up the next). A slot missed (the access
// a tile pair late) fails, as does any other difference past TOL (default 1). --notgt: TGT is off, and no
// access may wait.
//
//   Vm3_bus --cycles N [--seed S] [--tol T] [--gap MAX] [--mhz M]
#include "Vm3_bus.h"
#include "Vm3_bus___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <algorithm>

// ---- Supermodel's model, restated (TileGen.cpp, 0d1dd3d) ----
struct SmTiming {
  uint64_t clk, issue, complete, lineStart = 0, busFree = 0, writesDone = 0; bool fetching = false;
  explicit SmTiming(unsigned hz) {
    clk = (hz + 1000000) / 2000000; if (!clk) clk = 1;
    issue = 5 * clk; complete = (63 * clk + 2) / 5;
  }
  void line(uint64_t cyc) { lineStart = cyc * 16; fetching = true; }
  void blank() { fetching = false; }
  // the cycle (in clocks, fractional rounded up) at which the access is taken (write) or its read may go
  uint64_t access(uint64_t cyc, bool write, bool tileRAM, uint64_t *takenAt) {
    uint64_t now = cyc * 16, start = std::max(now, busFree); bool fetchSlot = false;
    if (tileRAM && fetching) {
      uint64_t pair = 32 * clk, fetchEnd = lineStart + 32 * pair;
      if (start < fetchEnd) {
        uint64_t firstSlot = lineStart + 28 * clk;
        uint64_t slot = start <= firstSlot ? firstSlot : firstSlot + ((start - firstSlot + pair - 1) / pair) * pair;
        start = std::min(slot, fetchEnd); fetchSlot = start < fetchEnd;
      }
    }
    busFree = start + (fetchSlot ? 4 * clk : issue);
    uint64_t c = start + complete; writesDone = std::max(writesDone, c);
    *takenAt = (start + 15) / 16;
    uint64_t w = write ? start : std::max(c, writesDone);
    return (w + 15) / 16;
  }
  uint64_t read(uint64_t cyc) { uint64_t now = cyc * 16; return (std::max(now, writesDone) + 15) / 16; }
};

static Vm3_bus *t;
static uint64_t cyc = 0;
static void tick() { t->clk = 1; t->eval(); t->clk = 0; t->eval(); cyc++; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  uint64_t ncyc = 2000000; unsigned seed = 1, mhz = 110; bool notgt = false; int tol = 1, gapmax = 40;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--cycles") ncyc = std::strtoull(argv[++i], nullptr, 0);
    else if (a == "--seed") seed = unsigned(std::atoi(argv[++i]));
    else if (a == "--tol") tol = std::atoi(argv[++i]);
    else if (a == "--gap") gapmax = std::atoi(argv[++i]);
    else if (a == "--mhz") mhz = unsigned(std::atoi(argv[++i]));
    else if (a == "--notgt") notgt = true;
  }
  std::mt19937 rng(seed);
  auto rnd = [&](int n) { return int(rng() % unsigned(n)); };
  t = new Vm3_bus;
  t->rst = 1; t->m_req = 0; t->sc_ack = 0; t->cf_ack = 0; t->jt_busy = 0; t->ln_start = 0; t->ln = 0; t->s_ack = 0;
  t->s_wnext = 0; t->s_rvalid = 0; t->h_rdy = 1; t->h_rv = 0; t->sn_ack = 0; t->sec_ack = 0; t->sc_win_en = 0;
  t->step2 = 0; t->fixed8 = 0; t->crom_slot = 0; t->crom_zero = 0; t->jt_cfg_we = 0; t->scsi_irq = 0; t->pingpong_i = 0;
  for (int i = 0; i < 8; i++) tick();
  t->rst = 0;
  for (int i = 0; i < 8; i++) tick();

  SmTiming sm(mhz * 1000000u);
  // line events: a pixel at 16 MHz from the clock, as m3_system's ce_pix; 656 pixels a line
  unsigned pacc = 0; int hcnt = 0, vline = 0;
  // the SDRAM port
  bool s_busy = false; int s_left = 0; bool s_rd = false; int s_phase = 0;
  // master 0
  enum { M_IDLE, M_REQ, M_WAIT_RV } ms = M_IDLE; int gap = 0;
  bool m_wr = false; int kind = 0; uint32_t addr = 0;
  // measurement
  bool in_dec = false; uint64_t dec_in = 0; int dec_kind = 0; bool dec_wr = false;
  long n = 0, bad = 0, waited = 0, nslot = 0; std::map<long, long> hist, hk[8]; long nk[8] = {0}, wk[8] = {0};
  auto *r = t->rootp;
  const char *kname[8] = {"tile wr", "pal wr", "tg reg wr", "CROM rd", "I/O rd", "RAM rd", "RAM wr", "?"};

  while (cyc < ncyc) {
    // ---- inputs for this cycle ----
    t->ln_start = 0;
    pacc += 16; bool ce = false; if (pacc >= mhz) { pacc -= mhz; ce = true; }
    if (ce) {
      if (hcnt == 655) {
        hcnt = 0; vline = (vline == 423) ? 0 : vline + 1;
        t->ln_start = 1; t->ln = vline;
        // Supermodel: a shown line starts the fetch; the frame's blanking stops it
        if (vline < 384) sm.line(cyc); else if (vline == 384) sm.blank();
      } else hcnt++;
    }
    // SDRAM port
    t->s_ack = 0; t->s_rvalid = 0;
    if (!s_busy && t->s_req) { s_busy = true; s_left = 1 + rnd(12); s_rd = !(t->s_cmd & 1); s_phase = 0; }
    if (s_busy) {
      if (s_left > 0) s_left--;
      else if (s_phase == 0) { t->s_ack = 1; s_phase = 1; if (!s_rd) s_busy = false; else s_left = 1 + rnd(4); }
      else { t->s_rvalid = 1; t->s_rdata = rng(); s_busy = false; }
    }
    // master 0
    if (ms == M_IDLE) {
      if (gap > 0) gap--;
      else {
        int p = rnd(100);
        kind = p < 30 ? 0 : p < 45 ? 1 : p < 50 ? 2 : p < 65 ? 3 : p < 70 ? 4 : p < 85 ? 5 : 6;
        switch (kind) {
          case 0: addr = 0xF1000000u | (uint32_t(rnd(0x40000)) << 2); m_wr = true; break;
          case 1: addr = 0xF1100000u | (uint32_t(rnd(0x8000)) << 2); m_wr = true; break;
          case 2: addr = 0xF1180000u | (uint32_t(0x10 + rnd(8)) << 2); m_wr = true; break;
          case 3: addr = 0xFF800000u | (uint32_t(rnd(0x100000)) << 2); m_wr = false; break;
          case 4: addr = 0xF0040000u | (uint32_t(rnd(16)) << 2); m_wr = false; break;
          case 5: addr = uint32_t(rnd(0x200000)) << 2; m_wr = false; break;
          default: addr = uint32_t(rnd(0x200000)) << 2; m_wr = true; break;
        }
        ms = M_REQ;
      }
    }
    if (ms == M_REQ) {
      t->m_req = 1; t->m_cmd = m_wr ? 3 : 2; t->m_addr = addr; t->m_be = 0xF; t->m_wdata = rng();
    } else t->m_req = 0;
    // ---- the edge ----
    uint8_t const bs_before = r->m3_bus__DOT__bs;
    tick();
    uint8_t const bs_after = r->m3_bus__DOT__bs;
    uint32_t const ack = t->m_ack, rv = t->m_rvalid;
    // B_DEC is state 1
    if (bs_after == 1 && bs_before != 1) { in_dec = true; dec_in = cyc - 1; dec_kind = kind; dec_wr = m_wr; }
    if (in_dec && bs_before == 1 && bs_after != 1) {
      in_dec = false;
      uint64_t const left = cyc - 2;              // B_DEC's last clock less its one clock: the clocks it waited, from dec_in
      // Supermodel: the access reaches the chip in the first clock of B_DEC
      uint64_t expect = dec_in, taken = 0;
      // B_DEC's first clock is dec_in + 1: the access reaches the chip then; left + 1 is the clock it is taken in
      if (dec_kind == 0 || dec_kind == 1) expect = sm.access(dec_in + 1, true, dec_kind == 0, &taken) - 1;
      else if (dec_kind == 3 || dec_kind == 4) expect = sm.read(dec_in + 1) - 1;
      long d = long(left) - long(expect);
      static uint64_t lt_in = 0, lt_left = 0, lt_sm = 0; static int lt_k = -1;
      hist[std::max(-9L, std::min(9L, d))]++; hk[dec_kind][std::max(-9L, std::min(9L, d))]++;
      struct Upd { uint64_t &a, &b, &c; int &k; } u{lt_in, lt_left, lt_sm, lt_k}; n++; nk[dec_kind]++;
      if (left > dec_in) { waited++; wk[dec_kind]++; }
      bool const slotr = (dec_kind == 0 || dec_kind == 3 || dec_kind == 4 || dec_kind == 1) && d >= long(mhz) - tol && d <= long(mhz) + tol;
      if (slotr && !notgt) nslot++;
      if (notgt) { if (left != dec_in) { if (bad < 10) std::printf("WAITED with TGT off: %s\n", kname[dec_kind]); bad++; } }
      else if ((!slotr && (d > tol || d < -tol)) || ((dec_kind == 5 || dec_kind == 6 || dec_kind == 2) && left != dec_in)) {
        if (bad < 10) std::printf("  previous: %s in %llu left %llu Supermodel %llu\n", lt_k >= 0 ? kname[lt_k] : "-", (unsigned long long)lt_in, (unsigned long long)lt_left, (unsigned long long)lt_sm);
        if (bad < 10) std::printf("MISMATCH cycle %llu %s at %08X: in B_DEC %llu, left %llu, Supermodel %llu (line %d, pixel %d)\n",
                                  (unsigned long long)cyc, kname[dec_kind], addr, (unsigned long long)dec_in,
                                  (unsigned long long)left, (unsigned long long)expect, vline, hcnt);
        bad++;
      }
      if (dec_kind <= 1 || dec_kind == 3 || dec_kind == 4) { u.a = dec_in; u.b = left; u.c = expect; u.k = dec_kind; }
    }
    if (!in_dec && bs_before == 1 && bs_after != 1) {}
    if (ms == M_REQ && (ack & 1)) { if (m_wr) { ms = M_IDLE; gap = rnd(gapmax + 1); } else ms = M_WAIT_RV; }
    else if (ms == M_WAIT_RV && (rv & 1)) { ms = M_IDLE; gap = rnd(gapmax + 1); }
  }
  std::printf("accesses %ld, waited %ld, outside +-%d clocks %ld\n", n, waited, tol, bad);
  for (int k = 0; k < 7; k++) { std::printf("  %-10s %8ld  waited %8ld  diff", kname[k], nk[k], wk[k]); for (auto &kv : hk[k]) std::printf(" %ld:%ld", kv.first, kv.second); std::printf("\n"); }
  std::printf("RTL - Supermodel (clocks, clamped to +-9):");
  for (auto &kv : hist) std::printf(" %ld:%ld", kv.first, kv.second);
  // the RTL's clock-rounded busy times can miss a slot Supermodel's fractional ones just catch: a tile pair late
  bool const fail = bad || nslot > 0;
  std::printf("\none tile pair late (slot rounding) %ld (%.4f%%, limit 0)\n%s\n", nslot, 100.0 * double(nslot) / double(n ? n : 1), fail ? "FAIL" : "PASS");
  delete t;
  return fail ? 1 : 0;
}
