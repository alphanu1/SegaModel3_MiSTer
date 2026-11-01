#!/bin/bash
# fit.sh <name> <top> <seed> <file>...: standalone fit of the tile generator at
# 10 ns, Quartus 17.0, every port but clk a virtual pin, in sm3-builds/tilegen/fit/<name>_s<seed>;
# QSF_EXTRA adds lines to the qsf
N=$1; T=$2; S=$3; shift 3
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
D=$R/sm3-builds/tilegen/fit/${N}_s$S
rm -rf $D; mkdir -p $D
cp $R/quartus/tilegen/tilegen.sdc $D/$T.sdc
echo "PROJECT_REVISION = \"$T\"" > $D/$T.qpf
{
  echo "set_global_assignment -name FAMILY \"Cyclone V\""
  echo "set_global_assignment -name DEVICE 5CSEBA6U23I7"
  echo "set_global_assignment -name TOP_LEVEL_ENTITY $T"
  echo "set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files"
  echo "set_global_assignment -name SDC_FILE $T.sdc"
  for f in "$@"; do echo "set_global_assignment -name SYSTEMVERILOG_FILE $R/$f"; done
  echo "set_global_assignment -name OPTIMIZATION_MODE \"HIGH PERFORMANCE EFFORT\""
  echo "set_global_assignment -name AUTO_SHIFT_REGISTER_RECOGNITION OFF"
  echo "set_global_assignment -name NUM_PARALLEL_PROCESSORS 2"
  echo "set_global_assignment -name SEED $S"
  echo "set_instance_assignment -name VIRTUAL_PIN ON -to *"
  echo "set_instance_assignment -name VIRTUAL_PIN OFF -to clk"
  [ -n "$QSF_EXTRA" ] && echo "$QSF_EXTRA"   # e.g. a top-level parameter: set_parameter -name SB 9
} > $D/$T.qsf
cd $D
/home/ben/intelFPGA_lite/17.0/quartus/bin/quartus_sh --flow compile $T > flow.log 2>&1
E=$?
# the worst setup paths, before the databases go
cat > paths.tcl <<TCL
project_open $T
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 10 -detail summary -file paths.txt
report_timing -setup -npaths 1 -detail full_path -file path1.txt
TCL
/home/ben/intelFPGA_lite/17.0/quartus/bin/quartus_sta -t paths.tcl > sta_paths.log 2>&1
# keep the reports, drop the databases
rm -rf db incremental_db
echo "$N seed $S exit $E"
