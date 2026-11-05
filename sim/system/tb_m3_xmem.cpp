// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_xmem against a reference (study R100): random geometry VROM requests
// (1-8 words, any alignment, both VROM sizes, VF3's zero half) and r3d_texup
// reads, on a pipelined read port with random acknowledge and latency. The
// region's beats hold a hash of their address; a word is checked against the
// MRA layout's rule, half h of beat b * BLK + g for word {g, b, h}.
//   Vm3_xmem [requests] [seed]
#include "Vm3_xmem.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

static uint64_t beat_val(uint32_t b) {
  uint64_t x = 0x9E3779B97F4A7C15ull * (uint64_t(b) + 1);
  x ^= x >> 29; x *= 0xBF58476D1CE4E5B9ull; x ^= x >> 32;
  return x;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long nreq = argc > 1 ? std::atol(argv[1]) : 100000;
  unsigned seed = argc > 2 ? unsigned(std::atoi(argv[2])) : 1;
  std::mt19937 rng(seed);
  Vm3_xmem *t = new Vm3_xmem;
  uint64_t cyc = 0;
  struct rd { uint64_t at; uint32_t a; };
  std::deque<rd> q;
  long bad = 0, words = 0, texs = 0;
  auto tick = [&]() {
    // the port: this cycle's ack and read beat
    t->b_ack = t->b_req && (rng() % 4 != 0);
    t->b_rvalid = 0;
    if (!q.empty() && q.front().at <= cyc) { t->b_rvalid = 1; t->b_dout = beat_val(q.front().a); q.pop_front(); }
    t->clk = 0; t->eval();
    if (t->b_req && t->b_ack) {
      uint64_t at = cyc + 12 + rng() % 50;
      if (!q.empty() && q.back().at >= at) at = q.back().at + 1;
      q.push_back({at, t->b_addr});
    }
    t->clk = 1; t->eval(); cyc++;
  };
  t->rst = 1; t->g_req = 0; t->t_req = 0;
  for (int i = 0; i < 4; i++) tick();
  t->rst = 0;
  for (long n = 0; n < nreq; n++) {
    bool v64 = (n / 1000) & 1;
    t->vrom64 = v64;
    if (rng() % 5 == 0) {
      // a texup read: an SDRAM word address in the VROM image (VF3's layout)
      uint32_t off = (rng() % (1u << 22)) << 2;                  // 16-bit units, beat aligned
      t->t_req = 1; t->t_addr = 0x2000000 + off;                   // SDR_VROM / 2
      long g = 0;
      while (!t->t_ack && g++ < 10000) tick();
      if (!t->t_ack || t->t_dout != beat_val(off >> 2)) { if (bad++ < 10) std::printf("tex %ld: wrong\n", n); }
      t->t_req = 0; tick();
      texs++;
      continue;
    }
    uint32_t a = rng() & 0xFFFFFF;
    if (rng() % 8 == 0) a = (a & ~7u) | 6;                          // run across a group
    unsigned len = rng() % 8;
    if (a + len > 0xFFFFFF) a -= 8;
    t->g_req = 1; t->g_addr = a; t->g_len = len;
    tick();
    long g = 0;
    while (!t->g_ack && g++ < 100) tick();
    t->g_req = 0;
    unsigned got = 0;
    g = 0;
    while (got <= len && g++ < 2000) {
      tick();
      if (t->g_rvalid) {
        uint32_t i = a + got, exp;
        if (!v64 && !(i & 0x800000)) exp = 0;
        else {
          uint32_t p = i >> 1, b = p & 3, gg = v64 ? (p >> 2) & 0x1FFFFF : (p >> 2) & 0xFFFFF;
          uint64_t v = beat_val(b * (v64 ? 0x200000u : 0x100000u) + gg);
          exp = (i & 1) ? uint32_t(v >> 32) : uint32_t(v);
        }
        if (t->g_rdata != exp && bad++ < 10) std::printf("req %ld word %u (addr %06X len %u v64 %d): %08X want %08X\n", n, got, a, len, v64, t->g_rdata, exp);
        got++; words++;
      }
    }
    if (got != len + 1 && bad++ < 10) std::printf("req %ld: %u words of %u\n", n, got, len + 1);
    for (int k = 0; k < int(rng() % 4); k++) tick();
  }
  for (int k = 0; k < 200; k++) tick();
  if (!q.empty() && bad++ < 10) std::printf("reads left in flight: %zu\n", q.size());
  std::printf("%ld requests, %ld words, %ld texture reads, %llu cycles: %ld wrong -> %s\n", nreq, words, texs,
              (unsigned long long)cyc, bad, bad ? "FAIL" : "PASS");
  delete t;
  return bad ? 1 : 0;
}
