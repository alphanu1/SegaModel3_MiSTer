#!/bin/bash
# fit.sh <folder>:<seed>...: full builds of the M2.5 viewer (release form,
# 100 MHz), as one batch (at most three), each in sm3-builds/viewer/fit_s<folder>
# with SEED <seed>. A folder that exists is refused (fit_s1 holds the board-tested
# RBF, fit_s7 the textured one queued for the board). At most three Quartus
# jobs at once, other projects' included. Keeps the reports, the worst paths
# and the RBF; drops the databases.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
[ $# -le 3 ] || { echo "three seeds at most"; exit 1; }
for A in "$@"; do
  F=${A%%:*}; S=${A##*:}
  [ "$F" != "$A" ] || { echo "give <folder>:<seed>"; exit 1; }
  D=$R/sm3-builds/viewer/fit_s$F
  [ ! -e $D ] || { echo "$D exists"; exit 1; }
done
for A in "$@"; do
  F=${A%%:*}; S=${A##*:}
  D=$R/sm3-builds/viewer/fit_s$F
  mkdir -p $D
  cp $R/quartus/m3_viewer/m3_viewer.qpf $R/quartus/m3_viewer/m3_viewer.sdc $D/
  sed -e "s|\.\./\.\./|$R/|g" -e "s|^set_global_assignment -name SEARCH_PATH \.\./\.\.$|set_global_assignment -name SEARCH_PATH $R|" \
      -e "s|^set_global_assignment -name SEED .*|set_global_assignment -name SEED $S|" \
      -e "s|^set_global_assignment -name NUM_PARALLEL_PROCESSORS .*|set_global_assignment -name NUM_PARALLEL_PROCESSORS 8|" \
    $R/quartus/m3_viewer/Model3_viewer.qsf > $D/Model3_viewer.qsf
done
# Each seed starts on its own as soon as a Quartus slot is free (a batch that
# waited for three at once starved behind other projects' jobs), never while
# sm3-builds/QUARTUS_PRIORITY exists. A slot is any quartus_map, quartus_fit,
# quartus_asm, quartus_sta or quartus_cdb; a started flow is given 90 s to show.
# After each flow: the worst paths and Fmax at both slow corners (100C and
# -40C; the qsf has multicorner analysis off).
busy() { echo $(( $(pgrep -x quartus_fit | wc -l) + $(pgrep -x quartus_map | wc -l) + $(pgrep -x quartus_asm | wc -l) + $(pgrep -x quartus_sta | wc -l) + $(pgrep -x quartus_cdb | wc -l) )); }
for A in "$@"; do
  while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(busy) -ge 3 ]; do sleep 30; done
  F=${A%%:*}; S=${A##*:}
  echo "fit_s$F seed $S started $(date +%H:%M)"
  (
    D=$R/sm3-builds/viewer/fit_s$F
    cd $D
    $Q/quartus_sh --flow compile m3_viewer -c Model3_viewer > flow.log 2>&1
    E=$?
    cat > paths.tcl <<'TCL'
project_open m3_viewer -revision Model3_viewer
create_timing_netlist -model slow
read_sdc
update_timing_netlist
foreach {t n} {100 100c -40 n40c} {
  set_operating_conditions -model slow -temperature $t -voltage 1100
  update_timing_netlist
  report_timing -setup -npaths 20 -detail summary -file paths_$n.txt
  report_timing -setup -npaths 1 -detail full_path -file path1_$n.txt
  report_timing -hold -npaths 5 -detail summary -file hold_$n.txt
  report_clock_fmax_summary -file fmax_$n.txt
}
TCL
    while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(busy) -ge 3 ]; do sleep 20; done
    $Q/quartus_sta -t paths.tcl > sta_paths.log 2>&1
    # the database stays if either corner's report is missing, for a rerun
    if [ -s fmax_100c.txt ] && [ -s fmax_n40c.txt ]; then rm -rf db incremental_db; fi
    echo "fit_s$F seed $S exit $E $(date +%H:%M)"
  ) &
  sleep 90
done
wait
echo "batch done $(date +%H:%M)"
