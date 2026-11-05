// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_jtag against a Supermodel bus trace (bustrace.py's format): every write
// to F010000C is applied, and every read of F0100010 (TDO in bit 5) must give
// the byte the trace has. The Real3D configuration registers come from the
// trace's writes to 9C000000/4/8 (byte-reversed, as Supermodel stores them).
//
//   Vm3_jtag TRACE [--fault]
//
// --fault flips one TDO read's expectation in the middle, so the check is
// seen to fail.
#include "Vm3_jtag.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <vector>

#pragma pack(push, 1)
struct Rec { uint32_t frame, pc, addr; uint8_t kind, size, master, pad; uint64_t data; };
#pragma pack(pop)

static Vm3_jtag *t;
static uint64_t cyc;
static void tick() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++; }

static uint32_t swap32(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24); }

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: Vm3_jtag TRACE [--fault] [--step N]\n"); return 2; }
  bool fault = false;
  int jstep = 0;                       // 0 Step 1.0 ... 3 Step 2.1 (m3_jtag's jstep)
  for (int i = 2; i < argc; i++) {
    if (!std::strcmp(argv[i], "--fault")) fault = true;
    else if (!std::strcmp(argv[i], "--step") && i + 1 < argc) jstep = std::atoi(argv[++i]);
  }
  // the system registers at F0 or any of its mirrors up to FE (Daytona 2 uses FE)
  auto sys = [](uint32_t a, uint32_t off) { return (a & 0xF1FFFFFFu) == (0xF0100000u | off) && (a >> 28) == 0xF; };
  std::FILE *f = std::fopen(argv[1], "rb");
  if (!f) { std::perror(argv[1]); return 2; }
  std::vector<Rec> tr;
  Rec r;
  while (std::fread(&r, sizeof r, 1, f) == 1) tr.push_back(r);
  std::fclose(f);

  t = new Vm3_jtag;
  t->rst = 1; t->wr = 0; t->cfg0 = t->cfg1 = t->cfg2 = 0; t->jstep = jstep;
  for (int i = 0; i < 4; i++) tick();
  t->rst = 0;
  while (t->busy) tick();

  long writes = 0, reads = 0, bad = 0, cfgw = 0, maxbusy = 0;
  long nreads_total = 0;
  for (auto &x : tr) if (x.kind == 0 && sys(x.addr, 0x10)) nreads_total++;
  for (auto &x : tr) {
    if (x.kind == 1 && x.size == 4 && x.addr >= 0x9C000000 && x.addr < 0x9C00000C) {
      uint32_t v = swap32(uint32_t(x.data)) & 0x7FFFF;
      if (x.addr == 0x9C000000) t->cfg0 = v; else if (x.addr == 0x9C000004) t->cfg1 = v; else t->cfg2 = v;
      cfgw++;
    } else if (x.kind == 1 && sys(x.addr, 0x0C) && x.size == 1) {
      t->wr = 1; t->wdata = uint8_t(x.data);
      tick();
      t->wr = 0;
      long b = 0;
      while (t->busy) { tick(); b++; }
      if (b > maxbusy) maxbusy = b;
      tick();
      writes++;
    } else if (x.kind == 0 && sys(x.addr, 0x10) && x.size == 1) {
      uint8_t exp = uint8_t(x.data) & 0x20;
      if (fault && reads == nreads_total / 2) exp ^= 0x20;
      uint8_t got = t->tdo ? 0x20 : 0x00;
      if (got != exp) {
        if (bad < 10) std::printf("frame %u pc %08X: TDO read %02X, trace %02X (read %ld)\n", x.frame, x.pc, got, exp, reads);
        bad++;
      }
      reads++;
    }
  }
  std::printf("m3_jtag: %ld TCK/TMS/TDI writes, %ld TDO reads, %ld configuration writes, %ld mismatches, longest pass %ld clocks\n",
              writes, reads, cfgw, bad, maxbusy);
  std::printf("%s\n", (bad == 0 && reads > 0) ? "PASS" : "FAIL");
  delete t;
  return (bad == 0 && reads > 0) ? 0 : 1;
}
