// SPDX-License-Identifier: GPL-3.0-or-later
// r3d_setup_cr against its integer model (exact) and libm (MAME's expression).
#include "Vr3d_setup_cr.h"
#include "verilated.h"
#include "r3d_cordic.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
using namespace r3d;
static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long n = argc > 1 ? std::atol(argv[1]) : 200000;
  Vr3d_setup_cr *t = new Vr3d_setup_cr;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; tick(); t->rst = 0;
  std::mt19937_64 rng(9);
  std::uniform_real_distribution<double> ang(-1.5, 1.5), mag(0.01, 4.0);
  long bad_model = 0, bad_libm = 0, oor = 0, tot = 0;
  for (long i = 0; i < n; i++) {
    double const a = ang(rng), m = (i & 1) ? 1.0 : mag(rng);
    float y = float(std::sin(a) * m), x = float(std::cos(a) * m);
    if ((i & 7) == 3) x = -x;
    if (i % 1000 == 7) { y = (i & 8) ? -0.0f : 0.0f; x = (i & 16) ? -1.0f : 1.0f; }
    bool b;
    uint32_t const ref = cr_half_tan(fb(y), fb(x), b);
    t->in_valid = 1; t->y = fb(y); t->x = fb(x); tick(); t->in_valid = 0;
    int g = 0; while (!t->out_valid && g++ < 1000) tick();
    if (b != bool(t->bad)) { if (bad_model++ < 5) std::printf("bad flag differs y %08X x %08X\n", fb(y), fb(x)); continue; }
    if (b) { oor++; continue; }
    tot++;
    if (t->result != ref) { if (bad_model++ < 5) std::printf("y %08X x %08X rtl %08X model %08X\n", fb(y), fb(x), t->result, ref); }
    if (ref != fb(tanf(float(std::atan2(double(y), double(x))) * 0.5f))) bad_libm++;
  }
  std::printf("r3d_setup_cr: %ld checked, %ld differ from the integer model, %ld from libm, %ld outside its range\n", tot, bad_model, bad_libm, oor);
  return bad_model ? 1 : 0;
}
