#!/bin/bash
# fit.sh <name> <top> <seed> <file>...: standalone fit of a rasteriser block at
# 10 ns, Quartus 17.0, every port but clk a virtual pin, in sm3-builds/rast/fit100/<name>_s<seed>
N=$1; T=$2; S=$3; shift 3
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
D=$R/sm3-builds/rast/fit100/${N}_s$S
rm -rf $D; mkdir -p $D
cp $R/quartus/r3d_rast100/r3d_rast100.sdc $D/$T.sdc
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
} > $D/$T.qsf
cd $D
/home/ben/intelFPGA_lite/17.0/quartus/bin/quartus_sh --flow compile $T > flow.log 2>&1
E=$?
# keep the reports, drop the databases
rm -rf db incremental_db
echo "$N seed $S exit $E"
