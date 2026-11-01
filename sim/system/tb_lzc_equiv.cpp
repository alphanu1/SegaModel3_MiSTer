// SPDX-License-Identifier: GPL-3.0-or-later
// Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
// rtl/cpu/ppc_fpu_lzc.sv against sim/system/ppc_fpu_lzc_sim.sv (built as a
// pair, lzc_pair): random inputs with random leading-zero runs, and zero.
#include "Vlzc_pair.h"
#include "verilated.h"
#include <cstdio>
#include <random>
int main() {
  Vlzc_pair t; std::mt19937_64 r(1); long bad = 0, n = 0;
  int const W = int(sizeof(t.x) * 8) >= 162 ? 162 : 0;
  for (int i = 0; i < 200000; i++) {
    // a random word with a random number of leading zeros
    int lz = int(r() % 170);
    for (size_t k = 0; k < sizeof(t.x) / 4; k++) t.x[k] = uint32_t(r());
    for (int b = 161; b >= 0 && b > 161 - lz; b--) t.x[b / 32] &= ~(1u << (b % 32));
    if (i % 997 == 0) for (size_t k = 0; k < sizeof(t.x) / 4; k++) t.x[k] = 0;
    t.clk = 0; t.eval(); t.clk = 1; t.eval();
    n++;
    if (t.n_rtl != t.n_sim) { if (bad++ < 5) std::printf("%d: rtl %d sim %d\n", i, t.n_rtl, t.n_sim); }
  }
  (void)W;
  std::printf("lzc W=162: %ld inputs, %ld differ\n%s\n", n, bad, bad ? "FAIL" : "PASS");
  return bad ? 1 : 0;
}
