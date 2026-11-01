#!/bin/bash
# batch.sh <name> <SM 1|0>: analyze every tile generator file, then fit
# seeds 1-3 of tg_fit_top, each started only when fewer than 3 Quartus flows run machine-wide
# and sm3-builds/QUARTUS_PRIORITY is absent; waits with kill -0; prints the summary.
N=$1; SMV=$2
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
FILES="rtl/real3d/r3d_ram.sv rtl/video/m3_tilegen_pal.sv rtl/video/m3_tilegen_fetch.sv rtl/video/m3_tilegen.sv quartus/tilegen/tg_fit_top.sv"
slot() {   # wait for a free Quartus slot
  while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 30; done
}
A=$R/sm3-builds/tilegen/fit/${N}_analyze
rm -rf $A; mkdir -p $A; cd $A
slot
for f in $FILES; do
  $Q/quartus_map --analyze_file=$R/$f tg_an > an_$(basename $f).log 2>&1 || { echo "analyze failed: $f"; tail -20 an_$(basename $f).log; exit 1; }
done
echo "analyze: all files pass"
cd $R; rm -rf $A
P=""
for s in ${SEEDS:-1 2 3}; do
  slot
  QSF_EXTRA="set_parameter -name SM $SMV" $R/quartus/tilegen/fit.sh $N tg_fit_top $s $FILES &
  P="$P $!"
  sleep 90
done
for p in $P; do while kill -0 $p 2>/dev/null; do sleep 30; done; done
$R/quartus/tilegen/summary.sh $N
