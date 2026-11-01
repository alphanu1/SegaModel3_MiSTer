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
// m3_ddr3p (NP = 2) against a reference memory: port L does random reads and
// write bursts one at a time on m2_ddr3's contract, each P port random reads
// with up to `--inf` of its own in flight; every beat is checked, in order, by
// owner. The DDR3 model: BUSY a random PCT of cycles and always while K reads
// are held (--maxrd), beats LO..HI cycles after the take, 90% of cycles.
//
//   Vm3_ddr3p [--busy PCT] [--lat LO HI] [--maxrd K] [--inf N] [--cycles C] [--seed S]

#include "Vm3_ddr3p.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int busy = 20, lat_lo = 8, lat_hi = 40, inf = 8;
  bool nowrite = false, nop = false;
  unsigned maxrd = 0, seed = 1;
  uint64_t cycles = 2000000;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--maxrd") maxrd = unsigned(std::atoi(nx().c_str()));
    else if (a == "--inf") inf = std::atoi(nx().c_str());
    else if (a == "--cycles") cycles = uint64_t(std::atoll(nx().c_str()));
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--nowrite") nowrite = true;
    else if (a == "--nop") nop = true;
  }
  std::mt19937 rng(seed);
  uint64_t const BASE = 0x06000000, SIZE = 1 << 16;
  std::vector<uint64_t> mem(SIZE), ref(SIZE);
  for (size_t i = 0; i < SIZE; i++) mem[i] = ref[i] = (uint64_t(rng()) << 32) | rng();
  struct burst { uint64_t due, addr; uint32_t n; };
  std::deque<burst> rq;
  uint32_t wr_left = 0; uint64_t wr_addr = 0, cyc = 0, bad_addr = 0;
  auto at = [&](uint64_t a) -> uint64_t & { static uint64_t s; if (a < BASE || a >= BASE + SIZE) { bad_addr++; return s; } return mem[a - BASE]; };

  auto *t = new Vm3_ddr3p;
  // L: state
  bool l_on = false, l_we = false; uint32_t l_addr = 0, l_n = 0, l_k = 0; std::vector<uint64_t> l_data;
  // P: expected beats, in order, per port
  std::deque<uint64_t> pexp[2]; int pinf[2] = {0, 0}; std::deque<int> plen[2];
  uint64_t errs = 0, l_reads = 0, l_writes = 0, p_reads[2] = {0, 0}, beats = 0;
  uint32_t preq_a[2] = {0, 0}, preq_n[2] = {0, 0}; bool preq[2] = {false, false}, pwe[2] = {false, false};
  std::vector<uint64_t> pwd; uint32_t pw_k = 0, pw_n = 0, pw_a = 0; bool pw_on = false; uint64_t p_writes = 0;

  t->rst_n = 0; t->base = BASE; t->clk = 0; t->eval();
  for (int i = 0; i < 4; i++) { t->clk = 1; t->eval(); t->clk = 0; t->eval(); }
  t->rst_n = 1;
  for (; cyc < cycles; cyc++) {
    // the memory's answer this cycle
    bool rdy = false; uint64_t d = 0;
    if (!rq.empty() && rq.front().due <= cyc && (rng() % 100) >= 10) {
      auto &b = rq.front(); rdy = true; d = at(b.addr); b.addr++;
      if (--b.n == 0) rq.pop_front();
    }
    t->DDRAM_BUSY = ((int(rng() % 100) < busy) || (maxrd && rq.size() >= maxrd)) ? 1 : 0;
    t->DDRAM_DOUT_READY = rdy; t->DDRAM_DOUT = d;
    // the clients' requests, set before the edge (held until taken)
    if (!l_on && !t->ack && (rng() % 8) == 0) {
      l_on = true; l_we = !nowrite && (rng() % 3) == 0; l_n = 1 + rng() % 16; l_addr = uint32_t(SIZE / 2) + rng() % (SIZE / 2 - 16); l_k = 0;   // L: the upper half, P: the lower (no write races a P read)
      l_data.clear();
      if (l_we) { for (uint32_t i = 0; i < l_n; i++) l_data.push_back((uint64_t(rng()) << 32) | rng()); }
    }
    t->req = l_on; t->we = l_we; t->addr = l_addr; t->blen = l_n; t->be = 0xff;
    t->din = (l_we && l_k < l_n) ? l_data[l_k] : 0;
    for (int p = 0; p < 2; p++) {
      if (!nop && !preq[p] && pinf[p] < inf && (rng() % 3) == 0 && !(p == 1 && pw_on)) {
        preq[p] = true; preq_n[p] = 1 + rng() % 8;
        // port 1 writes a third of the time, in the second quarter; reads keep to the first
        pwe[p] = (p == 1) && !nowrite && (rng() % 3) == 0;
        preq_a[p] = pwe[p] ? uint32_t(SIZE / 4) + rng() % (SIZE / 4 - 16) : rng() % (SIZE / 4 - 16);
      }
    }
    t->p_req = (preq[0] ? 1 : 0) | (preq[1] ? 2 : 0);
    {   // p_addr is [1:0][24:0], 50 bits
      uint64_t pa = (uint64_t(preq_a[1]) << 25) | preq_a[0];
      t->p_addr = pa;
    }
    t->p_blen = (preq_n[1] << 8) | preq_n[0];
    t->p_we = (pwe[0] ? 1 : 0) | (pwe[1] ? 2 : 0);
    {   // port 1's write data: the burst being taken (its words chosen when the burst was)
      uint64_t const w = (pw_on && pw_k < pw_n) ? pwd[pw_k] : 0;
      t->p_din[0] = 0; t->p_din[1] = 0; t->p_din[2] = uint32_t(w); t->p_din[3] = uint32_t(w >> 32);
    }
    t->clk = 0; t->eval();
    // what is taken this cycle (combinational outputs)
    bool const wacc = t->wacc;
    uint32_t const pack = t->p_ack;
    if (!t->DDRAM_BUSY) {
      if (t->DDRAM_WE) {
        if (wr_left == 0) { wr_addr = t->DDRAM_ADDR; wr_left = t->DDRAM_BURSTCNT; }
        at(wr_addr) = t->DDRAM_DIN; wr_addr++; wr_left--;
      } else if (t->DDRAM_RD) {
        uint64_t due = cyc + uint64_t(lat_lo + int(rng() % unsigned(lat_hi - lat_lo + 1)));
        if (!rq.empty()) due = std::max(due, rq.back().due);
        rq.push_back({due, t->DDRAM_ADDR, t->DDRAM_BURSTCNT});
      }
    }
    uint32_t const pwacc = t->p_wacc;
    if ((pwacc >> 1) & 1) {
      if (!pw_on) errs++;
      else { ref[pw_a + pw_k] = pwd[pw_k]; if (++pw_k == pw_n) pw_on = false; }
    }
    for (int p = 0; p < 2; p++) if ((pack >> p) & 1 && pwe[p]) {
      pw_on = true; pw_k = 0; pw_n = preq_n[p]; pw_a = preq_a[p]; pwd.clear();
      for (uint32_t i = 0; i < pw_n; i++) pwd.push_back((uint64_t(cyc) << 20) ^ (uint64_t(i) << 50) ^ 0x5A5A0000ull ^ i);
      preq[p] = false; p_writes++;
    }
    for (int p = 0; p < 2; p++) if ((pack >> p) & 1 && !pwe[p]) {
      for (uint32_t i = 0; i < preq_n[p]; i++) pexp[p].push_back(ref[preq_a[p] + i]);
      pinf[p]++; plen[p].push_back(int(preq_n[p])); preq[p] = false; p_reads[p]++;
    }
    if (std::getenv("TR") && cyc < 140)
      std::printf("%3llu busy %d rd %d we %d a %llx n %u | rdy %d | req %d a %u n %u lk %u | l_busy %d\n", (unsigned long long)cyc, t->DDRAM_BUSY, t->DDRAM_RD, t->DDRAM_WE,
                  (unsigned long long)t->DDRAM_ADDR, t->DDRAM_BURSTCNT, rdy, t->req, l_addr, l_n, l_k, 0);
    t->clk = 1; t->eval();
    if (std::getenv("TR") && cyc < 140) std::printf("      after: rvalid %d ack %d\n", t->rvalid, t->ack);
    // registered outputs after the edge: this cycle's beats
    if (t->rvalid) {
      if (!l_on || l_we || l_k >= l_n) { errs++; if (errs < 10) std::printf("%llu: L beat with no read\n", (unsigned long long)cyc); }
      else { if (t->dout != ref[l_addr + l_k]) { errs++; if (errs < 10) std::printf("%llu: L beat %u of %u wrong (addr %u): %016llx want %016llx\n", (unsigned long long)cyc, l_k, l_n, l_addr, (unsigned long long)t->dout, (unsigned long long)ref[l_addr + l_k]); } l_k++; }
      beats++;
    }
    for (int p = 0; p < 2; p++) if ((t->p_rvalid >> p) & 1) {
      if (pexp[p].empty()) { errs++; if (errs < 10) std::printf("%llu: P%d beat with none due\n", (unsigned long long)cyc, p); }
      else {
        if (t->dout != pexp[p].front()) { errs++; if (errs < 10) std::printf("%llu: P%d beat wrong\n", (unsigned long long)cyc, p); }
        pexp[p].pop_front();
        if (--plen[p].front() == 0) { plen[p].pop_front(); pinf[p]--; }
      }
      beats++;
    }
    if (l_on && l_we && wacc) l_k++;
    if (t->ack) {
      if (!l_on || l_k != l_n) { errs++; if (errs < 10) std::printf("%llu: L ack after %u of %u\n", (unsigned long long)cyc, l_k, l_n); }
      if (l_on && l_we) { for (uint32_t i = 0; i < l_n; i++) ref[l_addr + i] = l_data[i]; l_writes++; } else l_reads++;
      l_on = false;
    }
  }
  // port 1's writes, landed
  uint64_t wbad = 0;
  for (uint64_t a = SIZE / 4; a < SIZE / 2; a++) if (mem[a] != ref[a]) wbad++;
  if (wbad) { std::printf("P writes: %llu words differ\n", (unsigned long long)wbad); errs += wbad; }
  std::printf("P1 writes %llu\n", (unsigned long long)p_writes);
  std::printf("%llu cycles: L %llu reads %llu writes, P %llu / %llu reads, %llu beats, %llu errors, %llu outside; acc max %u\n",
              (unsigned long long)cyc, (unsigned long long)l_reads, (unsigned long long)l_writes, (unsigned long long)p_reads[0],
              (unsigned long long)p_reads[1], (unsigned long long)beats, (unsigned long long)errs, (unsigned long long)bad_addr, unsigned(t->dbg_acc_max));
  std::printf("%s\n", (errs || bad_addr) ? "FAIL" : "PASS");
  delete t;
  return (errs || bad_addr) ? 1 : 0;
}
