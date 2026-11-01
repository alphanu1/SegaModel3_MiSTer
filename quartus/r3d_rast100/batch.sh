#!/bin/bash
# batch.sh <name> <top> <file>...: wait until no quartus_fit or quartus_map
# runs, check every file with quartus_map --analyze_file, then fit seeds 1-3
# side by side and print the summary.
N=$1; T=$2; shift 2
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
while pgrep -x quartus_fit >/dev/null || pgrep -x quartus_map >/dev/null; do sleep 30; done
A=$R/sm3-builds/rast/fit100/${N}_analyze
rm -rf $A; mkdir -p $A; cd $A
for f in "$@"; do
  $Q/quartus_map --analyze_file=$R/$f r3d_an > an_$(basename $f).log 2>&1 || { echo "analyze failed: $f"; tail -20 an_$(basename $f).log; exit 1; }
done
cd $R; rm -rf $A
for s in ${SEEDS:-1 2 3}; do $R/quartus/r3d_rast100/fit.sh $N $T $s "$@" & done
wait
$R/quartus/r3d_rast100/summary.sh $N
