#!/bin/bash
# fit.sh <name> <seed>: copy the tree (release form: Model3.qsf / Model3.sv, the stage-A system), set
# SEED, compile Model3, report per-corner timing and area. Waits for a free Quartus slot (at most 3
# flows machine-wide, none while sm3-builds/QUARTUS_PRIORITY exists).
B=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
D=$B/sm3-builds/$1
[ -e $D ] && { echo "$D exists"; exit 1; }
mkdir -p $D
rsync -a --exclude sm3-builds --exclude tools --exclude .git --exclude 'obj_*' --exclude docs --exclude mra \
      --exclude db --exclude incremental_db --exclude output_files $B/ $D/
grep -q 'sys_dual_sdram.tcl' $D/Model3.qsf && grep -q "m3_system #(.CLK_HZ(SYS_HZ)) sys3" $D/Model3.sv || { echo "not the stage-A release form"; exit 1; }
# a diagnostic build: PATCH is a shell command run in the copy (e.g. a clock or an RTL swap)
if [ -n "$PATCH" ]; then (cd $D && eval "$PATCH") || { echo "patch failed"; exit 1; }; echo "$PATCH" > $D/patch.txt; fi
sed -i "s/^set_global_assignment -name SEED .*/set_global_assignment -name SEED $2/" $D/Model3.qsf
grep -q "^set_global_assignment -name SEED $2" $D/Model3.qsf || echo "set_global_assignment -name SEED $2" >> $D/Model3.qsf
while [ -e $B/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 30; done
cd $D
$Q/quartus_sh --flow compile Model3 > flow.log 2>&1
cat > wp.tcl <<'TCL'
project_open Model3
create_timing_netlist
read_sdc
update_timing_netlist
foreach corner {slow_100c slow_n40c fast_100c fast_n40c} { }
report_timing -setup -npaths 200 -nworst 1 -detail summary -to_clock [get_clocks {*general[1]*}] -file worst.txt -panel_name x
report_timing -setup -npaths 4 -nworst 1 -detail full_path -to_clock [get_clocks {*general[1]*}] -file detail.txt -panel_name y
TCL
$Q/quartus_sta -t wp.tcl > sta_wp.log 2>&1
grep -B2 -A8 "Fmax Summary" output_files/Model3.sta.rpt > result.txt
grep -A14 "Setup Summary  " output_files/Model3.sta.rpt | head -60 >> result.txt
grep -A14 "Hold Summary  " output_files/Model3.sta.rpt | head -40 >> result.txt
grep -m4 "Logic utilization\|block memory\|DSP\|RAM Blocks" output_files/Model3.fit.summary >> result.txt
rm -rf db incremental_db
echo done > done.txt
