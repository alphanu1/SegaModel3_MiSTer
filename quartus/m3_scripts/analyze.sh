#!/bin/bash
# analyze.sh: quartus_map --analyze_file on every m3_scripts file (Quartus 17 rejects some
# things Verilator accepts). Waits for a free slot.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
FILES="rtl/cpu/ppc_cache_pkg.sv rtl/io/m3_scripts.sv quartus/m3_scripts/sc_fit_top.sv"
while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 20; done
A=$R/sm3-builds/scripts_fit/analyze; rm -rf $A; mkdir -p $A; cd $A
for f in $FILES; do
  $Q/quartus_map --analyze_file=$R/$f sc_an > an_$(basename $f).log 2>&1 || { echo "analyze failed: $f"; grep -i "error" an_$(basename $f).log | head; exit 1; }
  grep -i "warning (10" an_$(basename $f).log | head -5
done
echo "analyze: all files pass"
cd $R; rm -rf $A
