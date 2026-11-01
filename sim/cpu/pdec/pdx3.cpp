// SPDX-License-Identifier: GPL-3.0-or-later
// ppc_pdec.sv's table generator and check (run.sh)
#include "Vpdx3.h"
#include <cstdio>
#include <cstdint>
// dump: opcd xo ra0 rcb -> fields as one integer
int main(){ Vpdx3 t;
  for (uint32_t o=0;o<64;o++) for (uint32_t x=0;x<1024;x++){
    if (o!=19 && o!=31 && x) break;
    for (int c=0;c<4;c++){ uint32_t w=(o<<26)|(x<<1)|((c&1)?0:(5u<<16))|((c&2)?1u:0u);   // c bit0: ra0, bit1: rcb
      t.i_data=w; t.eval();
      uint64_t v = (uint64_t)t.ua | (uint64_t)t.ub<<1 | (uint64_t)t.us<<2 | (uint64_t)t.xs<<3 | (uint64_t)t.az<<4 | (uint64_t)t.yreg<<5 |
        (uint64_t)t.rc<<6 | (uint64_t)t.xer<<7 | (uint64_t)t.fpw<<8 | (uint64_t)t.fhi<<9 | (uint64_t)t.fuc<<10 | (uint64_t)t.fdbl<<11 |
        (uint64_t)t.ill<<12 | (uint64_t)t.immk<<13 | (uint64_t)t.rk<<16 | (uint64_t)t.op<<21;
      printf("%u %u %d %llu\n", o, x, c, (unsigned long long)v); } }
}
