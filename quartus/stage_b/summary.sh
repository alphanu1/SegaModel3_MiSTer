#!/bin/bash
# summary.sh <name>: ALM, registers, M10K, DSP and slow-model Fmax of each seed
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer/sm3-builds/sb/fit
for d in $R/$1_s*; do
  f=$d/output_files/*.fit.summary; s=$d/output_files/*.sta.rpt
  alm=$(grep -m1 "Logic utilization (in ALMs)" $f | sed 's/.*: //; s/ \/.*//')
  reg=$(grep -m1 "Total registers" $f | sed 's/.*: //')
  m10=$(grep -m1 "Total RAM Blocks" $f | sed 's/.*: //; s/ \/.*//')
  dsp=$(grep -m1 "Total DSP Blocks" $f | sed 's/.*: //; s/ \/.*//')
  fmx=$(grep -A6 "Slow 1100mV 100C Model Fmax Summary" $s | grep -m1 "MHz" | awk '{print $2, $3}')
  fmc=$(grep -A6 "^; Slow 1100mV -40C Model Fmax Summary" $s | grep -m1 "MHz" | awk '{print $2, $3}')
  echo "$(basename $d): ALM $alm  reg $reg  M10K $m10  DSP $dsp  Fmax(100C) $fmx  Fmax(-40C) $fmc"
done
