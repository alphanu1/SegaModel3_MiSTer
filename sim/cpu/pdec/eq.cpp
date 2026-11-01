// SPDX-License-Identifier: GPL-3.0-or-later
// ppc_pdec.sv's table generator and check (run.sh)
#include "Veq.h"
#include <cstdio>
#include <random>
int main(){ Veq t; std::mt19937 r(7); long n=0, bad=0;
  for (uint32_t o=0;o<64;o++) for (uint32_t x=0;x<1024;x++) for (int k=0;k<64;k++){
    uint32_t w=(o<<26)|(x<<1)|(r()&0x03FFF801u); if(k<4) w=(w&~0x001F0001u)|((k&1)?0:(9u<<16))|((k&2)?1:0);
    t.i_data=w; t.eval(); n++; if (t.bad) { if(bad<5) printf("bad %08X\n", w); bad++; } }
  printf("%ld words, %ld differ\n", n, bad); }
