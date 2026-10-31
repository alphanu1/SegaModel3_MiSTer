// SPDX-License-Identifier: GPL-3.0-or-later
// r3d_fsqrt against sqrtf: every mantissa at an odd and an even exponent, plus specials.
#include "Vr3d_fsqrt.h"
#include "verilated.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Vr3d_fsqrt *t = new Vr3d_fsqrt;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; tick(); t->rst = 0;
  uint64_t n = 0, bad = 0;
  auto one = [&](uint32_t u) {
    t->in_valid = 1; t->x = u; tick(); t->in_valid = 0;
    int guard = 0; while (!t->out_valid && guard++ < 100) tick();
    float f; std::memcpy(&f, &u, 4); float r = std::sqrt(f); uint32_t ru; std::memcpy(&ru, &r, 4);
    n++;
    if (t->result != ru && !(std::isnan(r) && ((t->result >> 23 & 0xff) == 0xff) && (t->result & 0x7fffff))) { if (bad++ < 5) std::printf("sqrt %08X rtl %08X want %08X\n", u, t->result, ru); }
  };
  for (uint32_t e : {127u, 128u, 1u, 254u}) for (uint32_t f = 0; f < 0x800000; f += (e < 3 || e > 250) ? 4093 : 1) one((e << 23) | f);
  for (uint32_t u : {0u, 0x80000000u, 0x7f800000u, 0xff800000u, 0x7fc00000u, 0xbf800000u}) one(u);
  std::printf("r3d_fsqrt: %llu checked, %llu wrong\n", (unsigned long long)n, (unsigned long long)bad);
  return bad ? 1 : 0;
}
