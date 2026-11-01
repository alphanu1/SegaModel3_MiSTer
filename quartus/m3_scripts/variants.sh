#!/bin/bash
# variants.sh: seed-1 fits of the build variants measured for the study's 8.4 table:
# parameters (LINE 0, INT_OP 0, RD_ALL 1, ENABLE 0) and RTL variants written by
# sim/scripts/notes.md's recipe into sm3-builds/scripts_fit/varsrc (FIFO in flops,
# three adders, separate move pointers). At most 3 Quartus flows machine-wide.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
PKG=rtl/cpu/ppc_cache_pkg.sv; TOP=quartus/m3_scripts/sc_fit_top.sv
slot() {
  while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 20; done
}
P=""
run() {   # name rtl-file [parameter lines]
  N=$1; F=$2; shift 2
  EXTRA=""; for p in "$@"; do EXTRA="$EXTRA
set_parameter -name ${p%=*} ${p#*=}"; done
  slot
  QSF_EXTRA="$EXTRA" $R/quartus/m3_scripts/fit.sh $N sc_fit_top 1 $PKG $F $TOP &
  P="$P $!"; sleep 45
}
run v_line0   rtl/io/m3_scripts.sv LINE=0
run v_noint   rtl/io/m3_scripts.sv INT_OP=0
run v_rdall   rtl/io/m3_scripts.sv RD_ALL=1
run v_off     rtl/io/m3_scripts.sv ENABLE=0
run v_flops   sm3-builds/scripts_fit/varsrc/m3_scripts_flops.sv
run v_adders  sm3-builds/scripts_fit/varsrc/m3_scripts_adders.sv
run v_noreuse sm3-builds/scripts_fit/varsrc/m3_scripts_noreuse.sv
for p in $P; do while kill -0 $p 2>/dev/null; do sleep 20; done; done
$R/quartus/m3_scripts/summary.sh v_
