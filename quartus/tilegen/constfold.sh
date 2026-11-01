#!/bin/bash
# constfold.sh: quartus_map cf_top and print what Quartus computed for each output, from its
# "stuck at VCC/GND" messages (every output of cf_top is a constant). Waits for a free slot.
R=/home/ben/source/Mister_Cores/SegaModel3_MiSTer
Q=/home/ben/intelFPGA_lite/17.0/quartus/bin
while [ -e $R/sm3-builds/QUARTUS_PRIORITY ] || [ $(pgrep -x quartus_sh | wc -l) -ge 3 ]; do sleep 20; done
D=$R/sm3-builds/tilegen/fit/cf; rm -rf $D; mkdir -p $D; cd $D
echo 'PROJECT_REVISION = "cf_top"' > cf_top.qpf
printf 'set_global_assignment -name FAMILY "Cyclone V"\nset_global_assignment -name DEVICE 5CSEBA6U23I7\nset_global_assignment -name TOP_LEVEL_ENTITY cf_top\nset_global_assignment -name SYSTEMVERILOG_FILE %s\n' $R/quartus/tilegen/cf_top.sv > cf_top.qsf
$Q/quartus_map cf_top > map.log 2>&1 || { echo "map failed"; tail -20 map.log; exit 1; }
python3 - map.log <<'PY'
import re, sys, collections
v = collections.defaultdict(int); w = collections.defaultdict(int)
for l in open(sys.argv[1]):
    m = re.search(r'Pin "(\w+)\[(\d+)\]" is stuck at (VCC|GND)', l)
    if m:
        n, b, s = m.group(1), int(m.group(2)), m.group(3)
        w[n] = max(w[n], b + 1)
        if s == "VCC": v[n] |= 1 << b
for n in sorted(w): print(f"{n} = {v[n]}")
PY
cd $R; rm -rf $D
