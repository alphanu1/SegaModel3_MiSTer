// SPDX-License-Identifier: GPL-3.0-or-later
#include "Vdbat_tb.h"
#include <cstdio>
#include <random>
int main(){ Vdbat_tb t; std::mt19937 r(3); long n=0, bad=0, hits=0;
  for (long i=0;i<20000000;i++){
    uint32_t x=r(), y=r(); uint32_t bl = (i&1) ? ((1u<<(r()%12))-1) : (r()&0x7FF);   // contiguous, or any
    uint32_t s=x+y; uint32_t bepi=(s>>17)&0x7FFF;
    if (r()%4) { bepi ^= (r()%3==0) ? (1u<<(r()%15)) : 0; bepi ^= (r()&0x7FF) & bl; }   // near the sum
    else bepi = r()&0x7FFF;
    t.x=x; t.y=y; t.bepi=bepi; t.bl=bl; t.valid=1; t.eval(); n++; hits += t.hit;
    if (t.bad) { if (bad<5) printf("bad x %08X y %08X bepi %04X bl %03X\n", x, y, bepi, bl); bad++; } }
  printf("%ld cases (%ld hits), %ld differ\n", n, hits, bad); return bad != 0; }
