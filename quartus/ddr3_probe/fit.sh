#!/bin/bash
# fit.sh <folder>:<seed>[:MACRO=value]...: full builds of the DDR3 probe core (release form,
# 100 MHz), each in sm3-builds/probe/fit_s<folder> with SEED <seed>. A folder
# that exists is refused. Each seed starts as soon as the machine has a free
# Quartus slot (at most three quartus_map / quartus_fit at once, other
# projects' included), in the order given. Keeps the reports, the worst paths
# and the RBF; drops the databases.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
for A in "$@"; do
  F=${A%%:*}
  [ "$F" != "$A" ] || { echo "give <folder>:<seed>"; exit 1; }
  [ ! -e $R/sm3-builds/probe/fit_s$F ] || { echo "fit_s$F exists"; exit 1; }
done
for A in "$@"; do
  F=$(echo $A | cut -d: -f1); S=$(echo $A | cut -d: -f2); M=$(echo $A | cut -d: -f3)
  D=$R/sm3-builds/probe/fit_s$F
  mkdir -p $D
  cp $R/quartus/ddr3_probe/ddr3_probe.qpf $R/quartus/ddr3_probe/ddr3_probe.sdc $D/
  sed -e "s|\.\./\.\./|$R/|g" -e "s|^set_global_assignment -name SEARCH_PATH \.\./\.\.$|set_global_assignment -name SEARCH_PATH $R|" \
      -e "s|^set_global_assignment -name SEED .*|set_global_assignment -name SEED $S|" \
      -e "s|^set_global_assignment -name NUM_PARALLEL_PROCESSORS .*|set_global_assignment -name NUM_PARALLEL_PROCESSORS 8|" \
    $R/quartus/ddr3_probe/DDR3_probe.qsf > $D/DDR3_probe.qsf
  # (the qsf ends without a newline)
  for MM in $(echo "$M" | tr '+' ' '); do printf '\nset_global_assignment -name VERILOG_MACRO "%s"\n' "$MM" >> $D/DDR3_probe.qsf; done
done
for A in "$@"; do
  F=$(echo $A | cut -d: -f1); S=$(echo $A | cut -d: -f2)
  while [ $(( $(pgrep -x quartus_fit | wc -l) + $(pgrep -x quartus_map | wc -l) )) -ge 3 ]; do sleep 15; done
  echo "fit_s$F seed $S started $(date +%H:%M:%S)"
  (
    D=$R/sm3-builds/probe/fit_s$F
    cd $D
    $Q/quartus_sh --flow compile ddr3_probe -c DDR3_probe > flow.log 2>&1
    E=$?
    cat > paths.tcl <<'TCL'
project_open ddr3_probe -revision DDR3_probe
create_timing_netlist -model slow
read_sdc
update_timing_netlist
report_timing -setup -npaths 20 -detail summary -file paths.txt
report_timing -setup -npaths 1 -detail full_path -file path1.txt
report_clock_fmax_summary -file fmax.txt
report_timing -setup -npaths 30 -detail summary -to_clock [get_clocks {*emu|pll*divclk}] -file core_paths.txt
report_timing -setup -npaths 10 -detail full_path -through [get_nets -nowarn {*ram_waitrequest*}] -file busy_paths.txt
report_timing -setup -npaths 10 -detail full_path -through [get_nets -nowarn {*ram_readdatavalid*}] -file rdv_paths.txt
report_timing -hold -npaths 10 -detail summary -through [get_nets -nowarn {*ram_waitrequest*}] -file busy_hold.txt
TCL
    $Q/quartus_sta -t paths.tcl > sta_paths.log 2>&1
    [ -n "$KEEPDB" ] || rm -rf db incremental_db   # KEEPDB=1 keeps the database for TimeQuest queries
    echo "fit_s$F seed $S exit $E $(date +%H:%M:%S)"
  ) &
  sleep 30    # let its quartus_map show before counting again
done
wait
echo "batch done $(date +%H:%M:%S)"
