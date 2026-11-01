#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# run_rast.sh BIN GAME CHUNK JOBS LOG [bench args]: the whole-rasteriser bench on
# all of GAME's frames in chunks of CHUNK frames, JOBS at a time; FRAME lines and
# each chunk's summary into LOG
B=$1; G=$2; C=$3; J=$4; L=$5; shift 5
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
N=$(ls $R/sm3-builds/rast/tris_$G/*.tri | wc -l)
: > $L
for ((f=0; f<N; f+=C)); do
  while [ $(jobs -rp | wc -l) -ge $J ]; do sleep 2; done
  ( $B --tris $R/sm3-builds/rast/tris_$G --dumps $R/sm3-builds/census/dumps_$G --first $f --frames $C "$@" +verilator+rand+reset+2 > $L.$f 2>&1; echo "EXIT $f $?" >> $L.$f ) &
done
wait
for ((f=0; f<N; f+=C)); do cat $L.$f >> $L; rm -f $L.$f; done
grep -c "^FRAME" $L; grep "^r3d_rast:" $L | awk '{fr+=$2; w+=$7; v+=$13} END {print "frames", fr, "pixels wrong", w, "violations", v}'; grep "EXIT" $L | grep -v " 0$"
