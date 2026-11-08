// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// tb_m3_xmem2 (study R127): m3_xmem against a model of its two clients and of m3_ddr3p's pipelined port.
//   tb_xmem SEED CYCLES [--v64 0|1|r] [--gout N] [--ack PCT] [--lat LO HI] [--tex PCT] [--seq PCT] [--xlin]
// --xlin: the build's XLIN (VROM in pair order: pair p at beat p; a request is one burst of b_blen beats)
// The VROM image is a hash of the beat address. The geometry client asks 1-8 word requests (mostly runs of
// 8-word bursts, as r3d_mstream, sometimes a new start anywhere, any length), keeps up to --gout requests taken
// (0: as many as the port takes), and checks every word in order; the texture client asks one beat at a time
// at an SDRAM word address in the VROM image's layout and checks it. The port takes a request with --ack %
// probability a cycle and gives the beats back in order after LO..HI cycles. Ends with every request answered.
#include "Vm3_xmem.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
static uint64_t hb(uint64_t a) { uint64_t x = a * 0x9E3779B97F4A7C15ull + 0x1234567; x ^= x >> 29; x *= 0xBF58476D1CE4E5B9ull; x ^= x >> 32; return x; }
static const uint32_t SDR_VROM_W = 0x4000000 >> 1;   // m3_xmem's SDR_VROM in SDRAM words
int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  uint32_t seed = argc > 1 ? std::atoi(argv[1]) : 1; long ncyc = argc > 2 ? std::atol(argv[2]) : 1000000;
  int v64m = 2, gout = 4, ackp = 70, llo = 12, lhi = 40, texp = 2, seqp = 90; bool xlin = false;
  for (int i = 3; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--v64") { std::string v = argv[++i]; v64m = v == "r" ? 2 : std::atoi(v.c_str()); }
    else if (a == "--gout") gout = std::atoi(argv[++i]);
    else if (a == "--ack") ackp = std::atoi(argv[++i]);
    else if (a == "--lat") { llo = std::atoi(argv[++i]); lhi = std::atoi(argv[++i]); }
    else if (a == "--tex") texp = std::atoi(argv[++i]);
    else if (a == "--seq") seqp = std::atoi(argv[++i]);
    else if (a == "--xlin") xlin = true;
  }
  std::mt19937 rng(seed);
  auto R = [&](int n) { return int(rng() % unsigned(n)); };
  Vm3_xmem *t = new Vm3_xmem;
  bool v64 = v64m == 2 ? (seed & 1) : v64m;
  t->vrom64 = v64; t->rst = 1; t->clk = 0; t->g_req = 0; t->t_req = 0; t->b_ack = 0; t->b_rvalid = 0;
  for (int i = 0; i < 4; i++) { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }
  t->rst = 0;
  auto vword = [&](uint32_t w) -> uint32_t {
    if (!v64 && !((w >> 23) & 1)) return 0;
    uint32_t p = (w >> 1) & 0x7FFFFF;
    uint64_t b = xlin ? (v64 ? p : (p & 0x3FFFFF)) : v64 ? ((uint64_t(p & 3) << 21) | (p >> 2)) : ((uint64_t(p & 3) << 20) | ((p >> 2) & 0xFFFFF));
    uint64_t q = hb(b);
    return (w & 1) ? uint32_t(q >> 32) : uint32_t(q);
  };
  struct Fl { long at; uint64_t addr; };
  std::deque<Fl> fl;                       // beats taken by the port, in order
  std::deque<uint32_t> exp;                // geometry words expected, in order
  std::deque<int> reqw;                    // words of each request taken and not finished
  long nreq = 0, nwords = 0, ntex = 0, bad = 0, ghost = 0;
  uint32_t nxt = 0x800000 | R(0x7FFFF8); int gl = 7; bool gpend = false;
  bool tpend = false; uint32_t ta = 0; long tsince = 0;
  long cyc = 0, last_prog = 0; bool stop_new = false;
  long maxfl = 0;
  for (;;) {
    stop_new = cyc >= ncyc;
    // ---- this cycle's inputs: the port's beat, its ack
    t->clk = 0; t->eval();
    t->b_rvalid = 0;
    if (!fl.empty() && fl.front().at <= cyc) { t->b_rvalid = 1; t->b_dout = hb(fl.front().addr); fl.pop_front(); }
    t->b_ack = t->b_req && R(100) < ackp;
    if (t->b_ack) {
      long at = cyc + llo + R(lhi - llo + 1);
      int const n = xlin ? (t->b_blen ? t->b_blen : 256) : 1;
      if (!xlin && t->b_blen != 1) { std::printf("FAIL: a burst of %d without XLIN\n", t->b_blen); return 1; }
      for (int k = 0; k < n; k++) {
        if (!fl.empty() && fl.back().at >= at) at = fl.back().at + (k ? 1 : 0);
        fl.push_back({at, uint64_t(t->b_addr) + k});
      }
      if (long(fl.size()) > maxfl) maxfl = fl.size();
    }
    // ---- the clients' requests (held until ack)
    if (!gpend && !stop_new && (gout == 0 || int(reqw.size()) < gout) && R(100) < 60) {
      if (R(100) >= seqp) {                // a new start anywhere: the zero region too (VF3), any length
        nxt = R(4) == 0 ? R(0x800000) : (0x800000 | R(0x7FFFF8));
        if (v64) nxt = R(0x1000000);
        if (nxt > 0xFFFFF8) nxt = 0xFFFFF8;
        gl = R(8);
      } else gl = 7;
      gpend = true;
    }
    t->g_req = gpend; t->g_addr = nxt; t->g_len = gl;
    if (!tpend && !stop_new && R(1000) < texp * 10) {
      uint32_t b = v64 ? R(0x800000) : R(0x400000);
      ta = SDR_VROM_W + (b << 2); tpend = true; tsince = cyc;
    }
    t->t_req = tpend; t->t_addr = ta;
    t->eval();
    t->clk = 1; t->eval();
    cyc++;
    // ---- outputs, registered at this edge
    if (t->g_ack) {
      if (!gpend) { std::printf("FAIL: g_ack with no request, cycle %ld\n", cyc); return 1; }
      for (int k = 0; k <= gl; k++) exp.push_back(vword(nxt + k));
      reqw.push_back(gl + 1); nreq++;
      gpend = false; nxt += gl + 1; if (nxt > 0xFFFFF8) nxt = 0x800000;
      last_prog = cyc;
    }
    if (t->g_rvalid) {
      if (exp.empty()) { ghost++; if (ghost < 5) std::printf("FAIL: a word with none expected, cycle %ld\n", cyc); }
      else {
        if (t->g_rdata != exp.front()) { bad++; if (bad < 5) std::printf("wrong word: cycle %ld got %08X want %08X\n", cyc, t->g_rdata, exp.front()); }
        exp.pop_front(); nwords++;
        if (--reqw.front() == 0) reqw.pop_front();
      }
      last_prog = cyc;
    }
    if (t->t_ack) {
      if (!tpend) { std::printf("FAIL: t_ack with no request, cycle %ld\n", cyc); return 1; }
      uint64_t tb = (ta - SDR_VROM_W) >> 2;     // {b, g}
      if (xlin) tb = v64 ? (((tb & 0x1FFFFF) << 2) | (tb >> 21)) : (((tb & 0xFFFFF) << 2) | (tb >> 20));
      uint64_t want = hb(tb);
      if (t->t_dout != want) { bad++; if (bad < 5) std::printf("wrong texture beat: cycle %ld got %016llX want %016llX\n", cyc, (unsigned long long)t->t_dout, (unsigned long long)want); }
      tpend = false; ntex++; last_prog = cyc;
    }
    if (cyc - last_prog > 2000) { std::printf("FAIL: no progress for 2000 cycles at %ld (%zu words, %d texture pending)\n", cyc, exp.size(), tpend); return 1; }
    if (stop_new && exp.empty() && !tpend && !gpend && fl.empty()) break;
  }
  bool ok = bad == 0 && ghost == 0;
  std::printf("%s seed %u v64 %d: %ld cycles, %ld requests, %ld words, %ld texture beats, %ld wrong, %ld unexpected, port beats in flight max %ld\n",
              ok ? "PASS" : "FAIL", seed, v64, cyc, nreq, nwords, ntex, bad, ghost, maxfl);
  delete t;
  return ok ? 0 : 1;
}
