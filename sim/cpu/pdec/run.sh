#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Regenerates rtl/cpu/ppc_pdec.sv's tables from ppc_decode (FPA = 0) and the
# core's pd_immk / pd_rk, then checks the module against the decoder on 4.2M
# words. Run from the repository root; work files go to sm3-builds/pdec.
#   sim/cpu/pdec/run.sh          dump, print the tables (tables.txt), check
set -e
R=$PWD; W=$R/sm3-builds/pdec; mkdir -p $W; cd $W
python3 - $R/rtl/cpu/ppc_core.sv <<'PY'
import sys
c=open(sys.argv[1]).read()
immk=c[c.index("  logic [2:0] pd_immk;"):c.index("  // Result kind, five bits in the queue")]
rk=c[c.index("  logic [4:0] pd_rk;"):c.index("  // The op compares behind the special-register bits")]
open('pdx3.sv','w').write("""module pdx3 import ppc_pkg::*; (input logic [31:0] i_data,
  output logic ua, ub, us, xs, az, yreg, rc, xer, fpw, fhi, fuc, fdbl, ill, output logic [2:0] immk, output logic [4:0] rk, output logic [5:0] op);
  uop_t pd_u;
  ppc_decode #(.FPA(1'b0)) d (.insn(i_data), .pc(32'd0), .step(5'd0), .pr(1'b0), .u(pd_u));
  wire  [5:0]  pd_op = i_data[31:26];
  wire  [9:0]  pd_x  = i_data[10:1];
"""+immk+rk+"""
  assign ua=pd_u.a_used; assign ub=pd_u.b_used; assign us=pd_u.s_used; assign xs=pd_u.x_from_s; assign az=pd_u.a_zero;
  assign yreg = pd_u.bsel==BSEL_RB; assign rc=pd_u.rc; assign xer=pd_u.set_ca || pd_u.oe;
  assign fpw=pd_u.fpw; assign fhi=pd_u.fhi; assign fuc=pd_u.fuc; assign fdbl=pd_u.fdbl;
  assign ill = pd_u.op==OP_ILLEGAL; assign op = pd_u.op; assign immk = pd_immk; assign rk = pd_rk;
endmodule
""")
PY
C=$R/rtl/cpu
rm -rf od; verilator --cc --exe --build -j 8 -O2 -Wno-fatal --Mdir od --top-module pdx3 $C/ppc_pkg.sv $C/ppc_fpu_pkg.sv $C/ppc_decode.sv pdx3.sv $R/sim/cpu/pdec/pdx3.cpp > /dev/null
./od/Vpdx3 > dump.txt
python3 $R/sim/cpu/pdec/gen.py > tables.txt
echo "tables: $W/tables.txt (the three case statements of ppc_pdec.sv)"
rm -rf oe; verilator --cc --exe --build -j 8 -O2 -Wno-fatal --Mdir oe --top-module eq $C/ppc_pkg.sv $C/ppc_fpu_pkg.sv $C/ppc_decode.sv \
  $C/ppc_pdec.sv pdx3.sv $R/sim/cpu/pdec/eq.sv $R/sim/cpu/pdec/eq.cpp > /dev/null
./oe/Veq
