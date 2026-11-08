// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_io (315-5649 and 93C46) against a Supermodel bus trace: every access to
// F0040000-F004003F is replayed in order (a 32-bit access is the byte in bits
// 31:24, as the bus hands it over) and every read must give the trace's byte.
//
//   Vm3_io TRACE
#include "Vm3_io.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <vector>

#pragma pack(push, 1)
struct Rec { uint32_t frame, pc, addr; uint8_t kind, size, master, pad; uint64_t data; };
#pragma pack(pop)

static Vm3_io *t;
static void tick() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }

int main(int argc, char **argv) {
  if (argc < 2) return 2;
  std::FILE *f = std::fopen(argv[1], "rb");
  if (!f) { std::perror(argv[1]); return 2; }
  std::vector<Rec> tr;
  Rec r;
  while (std::fread(&r, sizeof r, 1, f) == 1) tr.push_back(r);
  std::fclose(f);
  t = new Vm3_io;
  t->in0 = 0xff; t->test_b_n = 1; t->service_b_n = 1; t->in2 = 0xff; t->in3 = 0xff; t->dsw = 0x7f;
  t->por_n = 0; t->rst_n = 0; t->sel = 0; t->clk = 0; t->eval();
  tick(); t->por_n = 1; t->rst_n = 1; tick(); tick();
  long rd = 0, wr = 0, bad = 0, ee_rd = 0, busy_runs = 0, polls = 0;
  bool trace_busy = false;
  uint8_t pa = 0xff;
  long masked = 0;     // the trace showed busy (DO 0) since the last ready read
  auto access = [&](uint32_t addr, bool w, uint8_t byte) {
    t->off = (addr >> 2) & 15; t->we = w; t->wbyte = byte; t->sel = 1;
    t->eval();
    uint8_t got = t->rbyte;
    tick();
    t->sel = 0;
    for (int i = 0; i < 6; i++) tick();     // the bus's pace
    return got;
  };
  for (auto &x : tr) {
    if (x.kind > 1 || x.addr < 0xF0040000 || x.addr >= 0xF0040040) continue;
    if ((x.addr & 3) != 0) { std::printf("unaligned access %08X\n", x.addr); bad++; continue; }
    uint8_t byte = x.size == 4 ? uint8_t(x.data >> 24) : x.size == 1 ? uint8_t(x.data) : uint8_t(x.data >> 8);
    // the EEPROM's ready/busy status: how long busy lasts is timing, so a
    // run of busy reads in the trace is one busy read here, and the read
    // after it polls until the RTL is ready, as the game's loop does
    // with CS low the 93C46's DO is not driven: Supermodel reads 0 there (its
    // busy count re-arms on every CS-low write), MAME (and this RTL) 1, a
    // pull-up. The game's per-frame input poll reads bank 1 so; bit 5 is
    // not compared there.
    bool cs_low = !((pa >> 6) & 1) && (pa & 1);
    if (x.kind == 1 && x.addr == 0xF0040000) pa = byte;
    if (x.kind == 0 && x.addr == 0xF0040004 && cs_low) {
      uint8_t got = access(x.addr, 0, 0);
      rd++; ee_rd++; masked++;
      if ((got | 0x20) != (byte | 0x20)) { if (bad < 10) std::printf("frame %u pc %08X: read F0040004 (CS low) = %02X, trace %02X\n", x.frame, x.pc, got, byte); bad++; }
      continue;
    }
    if (x.kind == 0 && x.addr == 0xF0040004 && byte == 0xDF) { trace_busy = true; rd++; ee_rd++; continue; }
    uint8_t got = access(x.addr, x.kind, byte);
    if (x.kind == 0 && x.addr == 0xF0040004 && trace_busy) {
      bool seen_busy = (got == 0xDF);
      long n = 0;
      while (got == 0xDF && n < 1000000) { got = access(x.addr, 0, 0); n++; }
      polls += n;
      if (!seen_busy) { if (bad < 10) std::printf("frame %u: the trace was busy, the RTL never\n", x.frame); bad++; }
      busy_runs++;
      trace_busy = false;
    }
    if (x.kind == 0) {
      rd++;
      if (x.addr == 0xF0040004) ee_rd++;
      if (got != byte) { if (bad < 10) std::printf("frame %u pc %08X: read %08X = %02X, trace %02X\n", x.frame, x.pc, x.addr, got, byte); bad++; }
    } else wr++;
  }
  std::printf("m3_io: %ld writes, %ld reads (%ld of port B, %ld with CS low), %ld busy runs (%ld polls here), %ld mismatches\n", wr, rd, ee_rd, masked, busy_runs, polls, bad);
  std::printf("%s\n", bad == 0 && rd > 0 ? "PASS" : "FAIL");
  return bad == 0 && rd > 0 ? 0 : 1;
}
