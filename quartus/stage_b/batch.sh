#!/bin/bash
# batch.sh <name> <top> <file>...: one standalone fit at seed 1 (2026-11-01:
# one seed for test and block fits; sweeps only to close release builds). The
# folder must be free. Waits while QUARTUS_PRIORITY exists or three quartus_sh
# run machine-wide.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
N=$1; T=$2; shift 2
[ -e $R/sm3-builds/sb/fit/${N}_s1 ] && { echo "${N}_s1 exists"; exit 1; }
while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 30; done
$R/quartus/stage_b/fit.sh $N $T 1 "$@"
$R/quartus/stage_b/summary.sh $N
