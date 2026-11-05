// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_sec (the 315-5881 and its RAM, study R100) against a Supermodel bus
// trace: every security-board access of the trace in order (RAM writes, the
// stream resets, the subkeys), and every data and status read compared.
//   Vm3_sec TRACE KEY [--fault]
#include "Vm3_sec.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#pragma pack(push, 1)
struct Rec { uint32_t frame, pc, addr; uint8_t kind, size, master, pad; uint64_t data; };
#pragma pack(pop)

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  if (argc < 3) { std::fprintf(stderr, "usage: Vm3_sec TRACE KEY [--fault]\n"); return 2; }
  bool fault = argc > 3 && !std::strcmp(argv[3], "--fault");
  std::FILE *f = std::fopen(argv[1], "rb");
  if (!f) { std::perror(argv[1]); return 2; }
  std::vector<Rec> tr;
  Rec r;
  while (std::fread(&r, sizeof r, 1, f) == 1) if (r.kind != 2 && (r.addr >> 28) == 0xF && (r.addr & 0xFFFFFF) >= 0x180000 && (r.addr & 0xFFFFFF) < 0x1A0040) tr.push_back(r);
  std::fclose(f);
  Vm3_sec *t = new Vm3_sec;
  t->game_key = uint32_t(std::strtoul(argv[2], nullptr, 16)) & 0x3FFFFFFF;
  if (fault) t->game_key ^= 1u << 4;
  long cyc = 0, bad = 0, reads = 0, maxw = 0;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++; };
  t->rst = 1; t->req = 0;
  for (int i = 0; i < 4; i++) tick();
  t->rst = 0;
  for (auto &x : tr) {
    uint32_t off = x.addr & 0x3FFFF;                       // xx180000-xx1BFFFF
    t->req = 1; t->we = x.kind == 1; t->addr = off >> 2;
    t->be = x.size == 4 ? 0xF : x.size == 2 ? (0xC >> (off & 2)) : (0x8 >> (off & 3));
    t->wdata = x.size == 4 ? uint32_t(x.data) : x.size == 2 ? uint32_t(x.data) << (16 - 8 * (off & 2)) : uint32_t(x.data) << (24 - 8 * (off & 3));
    long w = 0;
    do { tick(); w++; } while (!t->ack && w < 1000);
    t->req = 0;
    if (w > maxw) maxw = w;
    if (x.kind == 0) {
      uint32_t exp = uint32_t(x.data);
      if (!t->ack || t->rdata != exp) { if (bad++ < 10) std::printf("frame %u pc %08X read %08X: %08X, trace %08X\n", x.frame, x.pc, x.addr, t->rdata, exp); }
      reads++;
    }
    tick();
  }
  std::printf("m3_sec: %zu accesses, %ld reads, %ld differ, longest access %ld clocks, compressed %d -> %s\n", tr.size(), reads, bad,
              maxw, int(t->cmp_seen), (bad || !reads) ? "FAIL" : "PASS");
  delete t;
  return (bad || !reads) ? 1 : 0;
}
