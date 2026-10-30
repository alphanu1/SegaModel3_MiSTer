#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
#
# Put the CPU bring-up build on the board, load it, capture its debug UART
# (rtl/dbg/m3_cpu_mon.sv on ttyS1, 115200) for a while and bring the capture
# back for tools/cputest-compare.py. Adapted from the Model 2 core's
# tools/board-capture.sh.
#
#   M3_MISTER_IP=192.168.1.62 tools/cputest-capture.sh output_files/Model3.rbf out.txt 20
#
# This LOADS A CORE on the board: run it only when the board is free for it.
# No credentials live here: give ssh a key, or point SSH_ASKPASS at a script
# that prints the password and set SSH_ASKPASS_REQUIRE=force.
#
# The RBF it replaces is kept as Model3.rbf.prev; the copy is checked by MD5
# before the load is issued.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RBF=${1:?rbf}; OUT=${2:?out.txt}; SECS=${3:-20}
IP="${M3_MISTER_IP:?set M3_MISTER_IP to the address of the board}"
DEST=/media/fat/_Arcade/cores/Model3.rbf
SSH="ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 root@$IP"
$SSH "[ -f $DEST ] && cp -f $DEST $DEST.prev" 2>/dev/null
scp -o StrictHostKeyChecking=no "$RBF" root@$IP:$DEST 2>/dev/null
want=$(md5sum "$RBF" | cut -d' ' -f1); got=$($SSH "md5sum $DEST" 2>/dev/null | cut -d' ' -f1)
[ "$want" = "$got" ] || { echo "MD5 MISMATCH after copy"; exit 1; }
echo "deployed $RBF md5 $want"
scp -o StrictHostKeyChecking=no "$ROOT/tools/uart_capture.py" root@$IP:/tmp/uartcap.py 2>/dev/null
$SSH "echo 'load_core $DEST' > /dev/MiSTer_cmd" 2>/dev/null
echo "core load issued $(date +%T); capturing ${SECS}s"
sleep 3
$SSH "python3 /tmp/uartcap.py $SECS /tmp/uart.txt" 2>/dev/null
scp -o StrictHostKeyChecking=no root@$IP:/tmp/uart.txt "$OUT" 2>/dev/null
echo "capture $(wc -l < "$OUT") lines -> $OUT"
