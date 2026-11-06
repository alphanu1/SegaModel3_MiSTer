// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_sdr_port on m2_sdram, the SDRAM at its pins (sdram_dev.h). Random
// transactions of the caches' protocol, in both byte forms, against a byte
// image of what the bus should see: single reads and writes with random
// lanes, line fills from every critical word, cast-outs, zero reads. A second
// m2_sdram port reads 4-word bursts at random for contention. The master side
// checks the protocol: fields held to the acknowledge, beats only after it,
// eight beats a line in critical-word-first order.
//
//   Vm3_sdr_tb TRANSACTIONS SEED [--fault]
#include "Vm3_sdr_tb.h"
#include "verilated.h"
#include "sdram_dev.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

static Vm3_sdr_tb *t;
static sdram_dev sd;
static uint64_t cyc;
static std::mt19937 rng;
static bool q_on = true;
static uint32_t q_n = 0;

static void tick() {
  t->sd_dq_i = sd.dq(cyc);
  t->clk = 0; t->eval();
  sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
  t->clk = 1; t->eval();
  cyc++;
  // the contention port: a request every so often, dropped at its acknowledge
  if (q_on) {
    if (t->q_ack) { t->q_req = 0; q_n++; }
    else if (!t->q_req && (rng() % 100) < 30) { t->q_req = 1; t->q_addr = (rng() & 0x3FFFFFC) | 0; }
  }
}

// the bus's view: CPU byte A in word form is SDRAM byte A ^ 3 within its word
static uint8_t img(uint32_t a) { return uint8_t(sd.m[a >> 1] >> ((a & 1) * 8)); }
static uint32_t cpu_word(uint32_t a, bool swap) {
  uint32_t const b = a & ~3u;
  uint8_t const b0 = img(b), b1 = img(b + 1), b2 = img(b + 2), b3 = img(b + 3);
  return swap ? (uint32_t(b0) << 24) | (uint32_t(b1) << 16) | (uint32_t(b2) << 8) | b3
              : (uint32_t(b3) << 24) | (uint32_t(b2) << 16) | (uint32_t(b1) << 8) | b0;
}
// the expected image, kept separately and written as the RTL should
static std::vector<uint8_t> ref;
static void ref_write(uint32_t a, bool swap, uint32_t v, uint32_t be) {
  for (int k = 0; k < 4; k++) {          // CPU byte k: bits 31 - 8k, lane be[3 - k]
    if (!((be >> (3 - k)) & 1)) continue;
    uint8_t const byte = uint8_t(v >> (24 - 8 * k));
    uint32_t const sa = (a & ~3u) + (swap ? uint32_t(k) : uint32_t(3 - k));
    ref[sa] = byte;
  }
}
static uint32_t ref_word(uint32_t a, bool swap) {
  uint32_t const b = a & ~3u;
  return swap ? (uint32_t(ref[b]) << 24) | (uint32_t(ref[b + 1]) << 16) | (uint32_t(ref[b + 2]) << 8) | ref[b + 3]
              : (uint32_t(ref[b + 3]) << 24) | (uint32_t(ref[b + 2]) << 16) | (uint32_t(ref[b + 1]) << 8) | ref[b];
}

int main(int argc, char **argv) {
  long n = argc > 1 ? std::atol(argv[1]) : 20000;
  rng.seed(argc > 2 ? unsigned(std::atoi(argv[2])) : 1);
  bool fault = argc > 3 && !std::strcmp(argv[3], "--fault");
  bool lw1 = false, seq = false, nowait = false; int idle_max = 3;   // --seq: half the line reads are of the line after the last (the read-ahead, R107)
  for (int k = 3; k < argc; k++) {
    if (!std::strcmp(argv[k], "--bxor")) sd.bxor = true;   // the build's BXOR (R107)
    if (!std::strcmp(argv[k], "--lw1")) lw1 = true;        // the build's LW1 (R107)
    if (!std::strcmp(argv[k], "--seq")) seq = true;
    if (!std::strcmp(argv[k], "--nowait")) nowait = true;
    if (!std::strcmp(argv[k], "--idle") && k + 1 < argc) idle_max = std::atoi(argv[k + 1]);   // longer gaps: a read-ahead completes
  }
  t = new Vm3_sdr_tb;
  // a small working set (so reads hit earlier writes) spread over both chips
  // and several rows: 4 windows of 16 KiB
  static const uint32_t win[4] = {0x0000000, 0x00F8000, 0x4123000, 0x7FFC000};
  ref.assign(size_t(1) << 27, 0);
  for (int w = 0; w < 4; w++)
    for (uint32_t a = 0; a < 0x4000; a += 2) {
      uint16_t v = uint16_t(rng());
      sd.m[(win[w] + a) >> 1] = v;
      ref[win[w] + a] = uint8_t(v); ref[win[w] + a + 1] = uint8_t(v >> 8);
    }
  t->rst_n = 0; t->s_req = 0; t->q_req = 0;
  for (int i = 0; i < 4; i++) tick();
  t->rst_n = 1;
  while (!t->ready) tick();

  long bad = 0, proto = 0, nrd = 0, nwr = 0, nlr = 0, nlw = 0, nz = 0;
  uint64_t lat_sum = 0, lat_lr_first = 0, lat_lr_last = 0, n_fast = 0;   // n_fast: fills whose first beat came in 8 clocks (a read-ahead's)
  for (long i = 0; i < n && bad < 10 && proto < 10; i++) {
    int const kind = int(rng() % 4);
    bool const swap = rng() & 1;
    bool const zero = (kind == 0 || kind == 2) && (rng() % 8) == 0;
    uint32_t a = win[rng() % 4] + (rng() & 0x3FFC);
    uint32_t const be = (kind == 1) ? (rng() & 15) : 15;
    uint32_t const wv = rng();
    t->s_cmd = kind == 0 ? 2 : kind == 1 ? 3 : kind == 2 ? 0 : 1;     // MC_RD, MC_WR, MC_LINE_RD, MC_LINE_WR
    if (kind == 3) a &= ~31u;
    static uint32_t last_lr = 0;
    if (seq && kind == 2) {
      uint32_t const nx = last_lr + 32;
      bool in = false; for (int w = 0; w < 4; w++) if (nx >= win[w] && nx + 32 <= win[w] + 0x4000) in = true;
      if (in && (rng() & 1)) a = nx | (a & 31u);
      last_lr = a & ~31u;
    }
    t->s_sa = a; t->s_swap = swap; t->s_zero = zero; t->s_be = be; t->s_wdata = wv;
    // idle gap, sometimes none
    if (rng() & 1) for (int g = int(rng() % unsigned(idle_max + 1)); g > 0; g--) tick();
    if (!nowait) while (!t->s_idle) tick();   // (--nowait: as the bus does, the request goes up at once and waits for s_ack)
    while (t->s_ack) tick();                   // (the last acknowledge is not this one's: the bus asks two clocks after)
    t->s_req = 1;
    uint64_t const t0 = cyc;
    long guard = 0;
    while (!t->s_ack) { tick(); if (t->s_rvalid || t->s_wnext) { proto++; std::printf("beat before acknowledge\n"); } if (++guard > 100000) { std::printf("no acknowledge\n"); return 1; } }
    t->s_req = 0;
    if (kind == 1) {                      // posted write
      ref_write(a, swap, wv, be);
      nwr++;
    } else if (kind == 3) {               // cast-out: beat k is word k
      uint32_t beats[8];
      for (int k = 0; k < 8; k++) beats[k] = rng();
      // the bus as m3_bus has it (study R107): s_wnext goes on to the master a clock later as
      // m_wnext, the master shows its next beat in the clock after that, and the bus registers it
      // into s_wdata, so the beat shown follows each s_wnext by two clocks. A beat is taken when
      // s_wnext is high: it must be the next beat in order by then.
      // With --lw1 the port takes each beat two clocks after its s_wnext (m3_sdr_port LW1): the bus
      // keeps s_wdata following the master for one clock after it has seen the eighth s_wnext,
      // and then it is the next transaction's (random here).
      int k = 0, sh = 0, n1 = 0, n2 = 0;   // sh: the beat shown; n1, n2: s_wnext counts one and two clocks back
      int post = -1;                       // clocks since the eighth s_wnext (lw1)
      t->s_wdata = beats[0];
      guard = 0;
      while (k < 8 || (lw1 && post < 3)) {
        bool const wn = t->s_wnext;      // taken at this edge: the beat shown now
        if (wn) {
          if (k >= 8) { proto++; std::printf("a ninth s_wnext\n"); }
          if (!lw1 && sh != k) { proto++; if (proto < 10) std::printf("cast-out beat %d taken while beat %d is shown\n", k, sh); }
          ref_write(a + 4 * (k & 7), swap, beats[k & 7], 15); k++;
        }
        tick();
        sh = n2 < 8 ? n2 : 7; n2 = n1; n1 += wn;
        if (k == 8) post++;
        t->s_wdata = (lw1 && post >= 2) ? uint32_t(rng()) : beats[sh];
        if (++guard > 100000) { std::printf("cast-out stalls\n"); return 1; }
      }
      nlw++;
    } else {
      int const nb = kind == 0 ? 1 : 8;
      uint32_t const crit = (a >> 2) & 7;
      for (int k = 0; k < nb; k++) {
        guard = 0;
        do { tick(); if (++guard > 100000) { std::printf("read stalls\n"); return 1; } } while (!t->s_rvalid);
        if (k == 0) { lat_sum += cyc - t0; if (nb == 8) { lat_lr_first += cyc - t0; if (cyc - t0 <= 8) n_fast++; } }
        if (k == 7) lat_lr_last += cyc - t0;
        uint32_t const wa = kind == 0 ? a : (a & ~31u) + 4 * ((crit + k) & 7);
        uint32_t exp = zero ? 0 : ref_word(wa, swap);
        if (fault && i == n / 2) exp ^= 1;
        if (t->s_rdata != exp) {
          if (bad < 10) std::printf("txn %ld %s %s at %07X beat %d: %08X, expected %08X\n", i, kind == 0 ? "RD" : "LINE_RD",
                                    swap ? "byte" : "word", wa, k, t->s_rdata, exp);
          bad++;
        }
      }
      if (zero) nz++; else if (kind == 0) nrd++; else nlr++;
    }
  }
  while (!t->s_idle) tick();
  for (int i = 0; i < 50; i++) tick();
  // the whole image against the reference
  long img_bad = 0;
  for (int w = 0; w < 4; w++)
    for (uint32_t a = 0; a < 0x4000; a++)
      if (img(win[w] + a) != ref[win[w] + a]) { if (img_bad < 5) std::printf("image byte %07X: %02X, expected %02X\n", win[w] + a, img(win[w] + a), ref[win[w] + a]); img_bad++; }
  std::printf("m3_sdr_port: %ld reads, %ld writes, %ld line fills, %ld cast-outs, %ld zero reads; %u contending reads\n",
              nrd, nwr, nlr, nlw, nz, q_n);
  std::printf("  mean first-beat latency %.1f clocks over reads and fills; %ld data mismatches, %ld image bytes wrong, %ld protocol errors\n",
              double(lat_sum) / double(nrd + nlr + nz ? nrd + nlr + nz : 1), bad, img_bad, proto);
  std::printf("  line fills: %llu with the first beat within 8 clocks\n", (unsigned long long)n_fast);
  std::printf("  line fills: mean first beat %.1f, last beat %.1f clocks\n", double(lat_lr_first) / double(nlr + (nz ? 0 : 0) ? nlr : 1),
              double(lat_lr_last) / double(nlr ? nlr : 1));
  bool ok = bad == 0 && img_bad == 0 && proto == 0;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
