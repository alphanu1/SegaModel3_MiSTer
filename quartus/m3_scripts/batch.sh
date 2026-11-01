#!/bin/bash
# batch.sh <name> [set_parameter lines...]: analyze every m3_scripts file, then fit seeds 1-3
# (SEEDS overrides) of sc_fit_top, each started only when fewer than 3 Quartus flows run
# machine-wide and sm3-builds/QUARTUS_PRIORITY is absent; waits with kill -0; prints the summary.
N=$1; shift
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
FILES="rtl/cpu/ppc_cache_pkg.sv rtl/io/m3_scripts.sv quartus/m3_scripts/sc_fit_top.sv"
slot() {
  while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 30; done
}
$R/quartus/m3_scripts/analyze.sh || exit 1
EXTRA=""
for p in "$@"; do EXTRA="$EXTRA
set_parameter -name ${p%=*} ${p#*=}"; done
P=""
for s in ${SEEDS:-1 2 3}; do
  slot
  QSF_EXTRA="$EXTRA" $R/quartus/m3_scripts/fit.sh $N sc_fit_top $s $FILES &
  P="$P $!"
  sleep 60
done
for p in $P; do while kill -0 $p 2>/dev/null; do sleep 20; done; done
$R/quartus/m3_scripts/summary.sh $N
