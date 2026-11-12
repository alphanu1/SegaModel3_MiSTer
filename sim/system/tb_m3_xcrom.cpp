// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_xcrom (banked CROM in the extra memory, study R170) against a model: random single reads and line
// fills from m3_bus's side (s_req held to s_ack, the request's fields steady), a DDR3 arbiter port
// with random grant latency and gaps between beats (m2_ddr3's contract as m3_ddr_arb passes it: the
// beats in order, m_ack with the last or later). Every word is checked against the region's contents
// and its order (critical word first, wrapping in the line); the s_* handshake (no word in s_ack's
// clock, s_ack one clock) and the burst asked for (the line's beat 0 and four beats, or one).
//   Vm3_xcrom [--seed N] [--n TRANSACTIONS] [--fault]   (--fault: the model's word order is the line's
//   from word 0, which the RTL must fail)
#include "Vm3_xcrom.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>

static uint64_t beatval(uint32_t beat) {      // the region's contents: a hash of the beat
  uint64_t x = beat * 0x9E3779B97F4A7C15ull; x ^= x >> 29; x *= 0xBF58476D1CE4E5B9ull; return x ^ (x >> 32);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  unsigned seed = 1; long ntr = 200000; bool fault = false;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--seed")) seed = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--n")) ntr = std::atol(argv[++i]);
    else if (!std::strcmp(argv[i], "--fault")) fault = true;
  }
  std::mt19937 rng(seed);
  const uint32_t BASE = 0x1C00000;
  Vm3_xcrom *t = new Vm3_xcrom;
  long cyc = 0, bad = 0, words = 0, lines = 0, singles = 0, maxlat = 0;
  // the port model
  bool p_busy = false; uint32_t p_addr = 0, p_left = 0, p_n = 0; int p_wait = 0; bool p_ackpend = false; int p_ackdly = 0;
  auto fail = [&](const char *m) { if (bad++ < 10) std::printf("cycle %ld: %s\n", cyc, m); };
  t->rst = 1; t->s_req = 0; t->m_rvalid = 0; t->m_ack = 0;
  for (int i = 0; i < 4; i++) { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }
  t->rst = 0;
  // the transaction in flight from the bus's side
  bool tr_on = false, acked = false; int ackage = 0, cmd = 0; uint32_t sa = 0; int got = 0, want = 0; long t0 = 0;
  long done = 0;
  while (done < ntr && cyc < ntr * 400) {
    // inputs for this clock (set before the edge, from the state after the last)
    if (!tr_on && (rng() % 3 == 0)) {
      tr_on = true; acked = false; ackage = 0; got = 0;
      cmd = (rng() % 3 == 0) ? 2 : 0;                      // MC_RD or MC_LINE_RD
      sa = (rng() % 0x400000) << 2;                         // a word in the 16 MiB region
      want = cmd == 0 ? 8 : 1; t0 = cyc;
    }
    // m3_bus holds s_req through s_ack's clock and drops it the clock after
    t->s_req = tr_on && ackage < 2; t->s_cmd = cmd; t->s_sa = sa >> 2;
    if (ackage == 1) ackage = 2;
    // the DDR3 port: take a request, then beats with gaps
    t->m_rvalid = 0; t->m_ack = 0;
    if (!p_busy && t->m_req) {
      p_busy = true; p_addr = t->m_addr; p_n = t->m_blen; p_left = p_n; p_wait = 2 + rng() % 60; p_ackpend = false;
      uint32_t const exp_addr = BASE + (cmd == 0 ? ((sa & 0xFFFFFF) >> 5) << 2 : (sa & 0xFFFFFF) >> 3);
      if (p_addr != exp_addr || p_n != (cmd == 0 ? 4u : 1u)) fail("burst address or length");
    } else if (p_busy) {
      if (p_wait > 0) p_wait--;
      else if (p_left > 0) {
        if (rng() % 4) {
          t->m_rvalid = 1; t->m_dout = beatval(p_addr + (p_n - p_left)); p_left--;
          if (p_left == 0) { if (rng() % 2) { t->m_ack = 1; p_busy = false; } else { p_ackpend = true; p_ackdly = rng() % 4; } }
        }
      } else if (p_ackpend) {
        if (p_ackdly-- <= 0) { t->m_ack = 1; p_busy = false; p_ackpend = false; }
      }
    }
    t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++;
    // outputs after the edge
    if (t->s_ack) {
      if (!tr_on || acked) fail("s_ack with no request");
      acked = true; ackage = 1;
      if (t->s_rvalid) fail("a word in s_ack's clock");
    }
    if (t->s_rvalid) {
      if (!tr_on || !acked || got >= want) fail("a word with none due");
      else {
        uint32_t w;                                          // the word due: crit + got, wrapping in the line
        if (cmd == 0) w = ((sa >> 2) & ~7u) | ((((fault ? 0 : sa >> 2) & 7) + got) & 7);
        else w = sa >> 2;
        uint32_t const b = BASE + ((w & 0x3FFFFF) >> 1);
        uint32_t const e = (w & 1) ? uint32_t(beatval(b) >> 32) : uint32_t(beatval(b));
        if (t->s_rdata != e) fail("word value or order");
        got++; words++;
        if (got == want) { tr_on = false; done++; if (cmd == 0) lines++; else singles++; maxlat = std::max(maxlat, cyc - t0); }
      }
    } else if (t->s_rdata != 0) fail("s_rdata not zero outside s_rvalid");
  }
  if (done < ntr) fail("timed out");
  std::printf("m3_xcrom: %ld transactions (%ld lines, %ld single reads), %ld words, longest %ld clocks, %ld cycles -> %s\n",
              done, lines, singles, words, maxlat, cyc, bad ? "FAIL" : "PASS");
  delete t;
  return bad ? 1 : 0;
}
