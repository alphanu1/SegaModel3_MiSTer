#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the BTB bench. Each fault is one textual change to a COPY
# of rtl/cpu/ppc_btb.sv under sm3-builds/btb_fault/<name>/ (the working tree
# is never touched); the bench is built there and run on random seeds, then on
# a census trace if one is given. Reports caught (and by what) or missed.
#
# usage: sim/btb/faults.py [--trace <file>] [name ...]   (from the repository root)

import os, shutil, subprocess, sys

ROOT = os.getcwd()
OUT = os.path.join(ROOT, 'sm3-builds', 'btb_fault')
SEEDS = [1, 2, 3, 4]
CYCLES = 200000

FAULTS = [
  ('idx_bit', 'read index taken one bit up',
   "wire  [IW-1:0] ra = rd_pc[IW+1:2];", "wire  [IW-1:0] ra = rd_pc[IW+2:3];"),
  ('tag_short', 'tag compare drops PC[31]',
   "e_tag == rpc_q[31:IW+2]", "e_tag[TW-2:0] == rpc_q[30:IW+2]"),
  ('ctx_ignored', 'MSR[IR, PR] not compared',
   " && e_ctx == ctx", ""),
  ('rdw_none', 'a read on a write edge to the same entry is trusted',
   "rdw_q <= we && wa == ra;", "rdw_q <= 1'b0;"),
  ('sweep_short', 'the sweep stops one entry early',
   "if (sw_cnt == LAST) sw_act <= 1'b0;", "if (sw_cnt == LAST - 1'b1) sw_act <= 1'b0;"),
  ('kill_ignored', 'a redirect does not drop the held prediction',
   "if (rst || flush || sw_act || kill)", "if (rst || flush || sw_act)"),
  ('hold_overwrite', 'a held prediction is overwritten by a new capture',
   "else if (cap && hit && (!p_v || p_take)) begin", "else if (cap && hit) begin"),
  ('sweep_write', 'a core write takes the port during a sweep',
   "assign wa = sw_act ? sw_cnt : w_pc[IW+1:2];", "assign wa = (w_en && !sw_act) ? w_pc[IW+1:2] : (w_en ? w_pc[IW+1:2] : sw_cnt);"),
  ('inval_valid', 'an invalidating write leaves the entry valid',
   "wire  [BW-1:0] wb = {w_v && !sw_act,", "wire  [BW-1:0] wb = {!sw_act,"),
  ('flush_capture', 'a capture on the flush cycle is kept',
   "if (rst || flush || sw_act || kill)", "if (rst || sw_act || kill)"),
  ('reg_bit', 'the bclr / bcctr flag comes from the target\'s low bit',
   "wire          e_reg = q[EW-4-TW];", "wire          e_reg = q[XW];"),
]


def run(name, old, new, trace, r52, cfg):
  d = os.path.join(OUT, name + '_' + '_'.join(cfg))
  if os.path.isdir(d):
    shutil.rmtree(d)
  os.makedirs(os.path.join(d, 'rtl', 'cpu'))
  src = open(os.path.join(ROOT, 'rtl', 'cpu', 'ppc_btb.sv')).read()
  if src.count(old) != 1:
    return 'BAD FAULT (pattern found %d times)' % src.count(old)
  open(os.path.join(d, 'rtl', 'cpu', 'ppc_btb.sv'), 'w').write(src.replace(old, new))
  n, xw, st = cfg
  obj = os.path.join(d, 'obj')
  r = subprocess.run(['verilator', '--cc', '--exe', '--build', '-j', '8', '-O3', '-Wno-fatal', '--top-module', 'ppc_btb',
                      '-GN=' + n, '-GXW=' + xw, '-GSTYLE=' + st, '-CFLAGS',
                      '-O2 -std=c++17 -I%s/sim/btb -DBTB_N=%s -DBTB_XW=%s' % (ROOT, n, xw),
                      '--Mdir', obj, os.path.join(d, 'rtl', 'cpu', 'ppc_btb.sv'), os.path.join(ROOT, 'sim', 'btb', 'tb_btb.cpp')],
                     capture_output=True, text=True)
  if r.returncode:
    return 'BUILD FAILED'
  for s in SEEDS:
    r = subprocess.run([os.path.join(obj, 'Vppc_btb'), 'random', str(s), str(CYCLES)], capture_output=True, text=True)
    if r.returncode:
      first = [l for l in r.stdout.splitlines() if 'MISMATCH' in l][:1]
      shutil.rmtree(obj)
      return 'caught, random seed %d: %s' % (s, first[0][:110] if first else '')
  if trace:
    args = [os.path.join(obj, 'Vppc_btb'), 'trace', trace] + ([r52] if r52 else [])
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode:
      first = [l for l in r.stdout.splitlines() if 'MISMATCH' in l or 'DIFFERENT' in l][:1]
      shutil.rmtree(obj)
      return 'caught, trace: %s' % (first[0][:110] if first else '')
  shutil.rmtree(obj)
  return 'MISSED'


def main():
  args = sys.argv[1:]
  trace = r52 = None
  cfg = ('256', '0', '0')
  while args and args[0].startswith('--'):
    if args[0] == '--trace':
      trace, args = args[1], args[2:]
    elif args[0] == '--r52':
      r52, args = args[1], args[2:]
    elif args[0] == '--cfg':
      cfg, args = tuple(args[1].split(',')), args[2:]
  names = set(args)
  caught = total = 0
  for name, what, old, new in FAULTS:
    if names and name not in names:
      continue
    res = run(name, old, new, trace, r52, cfg)
    total += 1
    caught += res.startswith('caught')
    print('%-15s %-55s %s' % (name, what, res), flush=True)
  print('%d of %d caught' % (caught, total))


if __name__ == '__main__':
  main()
