#!/usr/bin/env python3
# area.py <fit.rpt>: the stage-A system's blocks from the fitter's "Resource
# Utilization by Entity" table: ALMs needed, registers, M10K, DSP.
import re, sys

rpt = open(sys.argv[1], errors='replace').read()
sec = rpt[rpt.index('; Fitter Resource Utilization by Entity'):]
sec = sec[:sec.index('\n\n')]
lines = [l for l in sec.splitlines() if l.startswith(';')]
hdr = [h.strip() for h in lines[1].strip(';').split(';')]
col = {h: i for i, h in enumerate(hdr)}

def num(s):
    s = s.strip().split(' ')[0].replace(',', '')
    try:
        return float(s)
    except ValueError:
        return 0.0

rows = []
for l in lines[2:]:
    f = l.strip(';').split(';')
    if len(f) < len(hdr):
        continue
    name = f[col['Compilation Hierarchy Node']]
    depth = (len(name) - len(name.lstrip(' '))) // 3
    rows.append((depth, name.strip().rstrip('|'), f))

ka = [h for h in hdr if h.startswith('ALMs needed')][0]
kr = [h for h in hdr if h.startswith('Dedicated Logic Registers')][0]
km = [h for h in hdr if h.startswith('M10K')][0]
kd = [h for h in hdr if h.startswith('DSP')][0]
want = ('|sys_top', '|emu:emu', '|m3_system:sys3', '|ppc_cpu:u_cpu', '|ppc_core:u_core', '|ppc_icache:u_ic',
        '|ppc_dcache:u_dc', '|m3_bus:u_bus', '|m3_sdr_port:u_sport', '|m2_sdram:u_sdram', '|m3_scripts:u_scsi',
        '|m3_io:u_io', '|m3_93c46:u_ee', '|m3_jtag:u_jtag', '|m3_tilegen:u_tg', '|m3_loader:u_ld',
        '|m2_video_timing:u_vt', '|m3v_overlay:g_ov.u_ov', '|m3_cpu_mon:g_mon.u_mon', '|m3_uart_tx:u_tx', '|hps_io:hps_io', '|pll:pll')
print(f"{'block':34s} {'ALM':>9s} {'regs':>7s} {'M10K':>5s} {'DSP':>4s}")
for d, n, f in rows:
    if n in want:
        print(f"{'  ' * min(d, 6) + n:34s} {num(f[col[ka]]):9.1f} {num(f[col[kr]]):7.0f} {num(f[col[km]]):5.0f} {num(f[col[kd]]):4.0f}")
