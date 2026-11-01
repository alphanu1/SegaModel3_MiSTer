#!/bin/bash
# analyze.sh: quartus_map --analyze_file on every stage-A system file (Quartus 17 rejects some
# things Verilator accepts). Waits for a free slot.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
FILES="rtl/cpu/ppc_cache_pkg.sv rtl/dbg/m3_uart_tx.sv rtl/dbg/m3_cpu_mon.sv rtl/sys/m3_jtag.sv rtl/io/m3_93c46.sv rtl/io/m3_io.sv rtl/sys/m3_sdr_port.sv
       rtl/sys/m3_bus.sv rtl/sys/m3_loader.sv rtl/sys/m3_system.sv rtl/pll.v rtl/pll/pll_0002.v"
while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 20; done
A=$R/sm3-builds/sys/analyze; rm -rf $A; mkdir -p $A; cd $A
for f in $FILES; do
  $Q/quartus_map --analyze_file=$R/$f sys_an > an_$(basename $f).log 2>&1 || { echo "analyze failed: $f"; grep -i "error" an_$(basename $f).log | head; exit 1; }
  grep -i "warning (10" an_$(basename $f).log | head -5
done
echo "analyze: all files pass"
cd $R; rm -rf $A
