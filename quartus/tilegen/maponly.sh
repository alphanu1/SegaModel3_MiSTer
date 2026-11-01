#!/bin/bash
# maponly.sh <name>: quartus_map of tg_fit_top (Supermodel switches) and its RAM summary, when a slot is free
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 20; done
D=$R/sm3-builds/tilegen/fit/map_$1; rm -rf $D; mkdir -p $D; cd $D
echo 'PROJECT_REVISION = "tg_fit_top"' > tg_fit_top.qpf
{ echo 'set_global_assignment -name FAMILY "Cyclone V"'; echo 'set_global_assignment -name DEVICE 5CSEBA6U23I7'
  echo 'set_global_assignment -name TOP_LEVEL_ENTITY tg_fit_top'
  for f in rtl/real3d/r3d_ram.sv rtl/video/m3_tilegen_pal.sv rtl/video/m3_tilegen_fetch.sv rtl/video/m3_tilegen.sv quartus/tilegen/tg_fit_top.sv; do echo "set_global_assignment -name SYSTEMVERILOG_FILE $R/$f"; done
  echo 'set_instance_assignment -name VIRTUAL_PIN ON -to *'; echo 'set_instance_assignment -name VIRTUAL_PIN OFF -to clk'; } > tg_fit_top.qsf
$Q/quartus_map tg_fit_top > map.log 2>&1 || { echo "map failed"; grep -a "Error" map.log | head; exit 1; }
grep -a "Total block memory bits\|Total RAM Blocks\|Logic utilization\|Total registers" output_files/tg_fit_top.map.summary
grep -a -A12 "^; RAM Summary" output_files/tg_fit_top.map.rpt | cut -c1-200 | sed 's/  */ /g'
rm -rf db incremental_db
