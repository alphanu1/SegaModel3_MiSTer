// SPDX-License-Identifier: GPL-3.0-or-later
// The CORDIC set-up model against MAME's tanf((float)atan2((double)y, (double)x) * 0.5f).
#include "r3d_cordic.h"
#include <cstdio>
#include <cstring>
#include <random>
using namespace r3d;
static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
int main(int argc, char **argv) {
  long n = argc > 1 ? std::atol(argv[1]) : 10000000;
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> ang(-1.5, 1.5), mag(0.01, 4.0);
  long bad = 0, oor = 0, tot = 0;
  for (long i = 0; i < n; i++) {
    double const a = ang(rng), m = (i & 1) ? 1.0 : mag(rng);
    float y = float(std::sin(a) * m), x = float(std::cos(a) * m);
    if ((i & 7) == 3) x = -x;                               // the other half plane
    if ((i & 15) == 5) y = float(std::ldexp(std::sin(a), -int(rng() % 20)));
    bool b;
    uint32_t const got = cr_half_tan(fb(y), fb(x), b);
    float const fl = float(std::atan2(double(y), double(x)));
    uint32_t const want = fb(tanf(fl * 0.5f));
    if (b) { oor++; continue; }
    tot++;
    if (got != want) { if (bad++ < 8) std::printf("y %08X x %08X: cordic %08X libm %08X\n", fb(y), fb(x), got, want); }
  }
  std::printf("cordic set-up model: %ld checked, %ld differ from libm, %ld outside its range\n", tot, bad, oor);
  (void)bf;
  return bad ? 1 : 0;
}
