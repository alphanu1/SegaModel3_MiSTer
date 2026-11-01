#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# ppc_core's dbat_hit against bat_match, 20M cases (from the repository root)
set -e
python3 - <<'PY'
c=open('rtl/cpu/ppc_core.sv').read()
def fn(n):
    i=c.index("  function automatic logic "+n+"("); return c[i:c.index("  endfunction", i)+len("  endfunction")]
t=open('sim/cpu/dbat/dbat_tb.sv').read()
i=t.index("  function automatic logic bat_match("); j=t.index("  wire [31:0] s = x + y;")
open('sm3-builds/dbat_tb.sv','w').write(t[:i]+fn('bat_match')+"\n"+fn('dbat_hit')+"\n"+t[j:])
PY
rm -rf sm3-builds/dbat_obj
verilator --cc --exe --build -j 8 -O2 -Wno-fatal --Mdir sm3-builds/dbat_obj --top-module dbat_tb sm3-builds/dbat_tb.sv sim/cpu/dbat/dbat.cpp > /dev/null
./sm3-builds/dbat_obj/Vdbat_tb
