#!/usr/bin/env python3
# area.py <fit.rpt>: the viewer's blocks from the fitter's "Resource
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
want = ('|sys_top', '|emu:emu', '|m3v_core:core', '|r3d_geo:u_geo', '|m3v_store:u_store',
        '|m3v_raster:u_rast', '|r3d_rast_intake:u_in', '|r3d_rast_front:u_front', '|r3d_rast_zspan:u_za',
        '|r3d_rast_zband:u_zband', '|r3d_rast_interp:u_interp', '|r3d_rast_recip:u_recip', '|r3d_rast_texel:u_tf',
        '|r3d_rast_texel:u_tb', '|r3d_rast_tcache:u_tc', '|r3d_rast_colour:u_col', '|r3d_rast_band:u_band',
        '|m2_sdram:u_sdram', '|m3_ddr3p:u_ddr3',
        '|m3v_ddr_arb:u_darb', '|m3v_loader:u_ld', '|m3v_sdr_rd:u_crd', '|m3v_sdr_rd:u_vrd', '|m3v_ddr_rd:u_prd',
        '|m2_fb_read:u_fbr', '|m2_video_timing:u_vt', '|m3v_overlay:u_ov', '|m3v_probe:u_probe', '|hps_io:hps_io', '|pll:pll')
print(f"{'block':34s} {'ALM':>9s} {'regs':>7s} {'M10K':>5s} {'DSP':>4s}")
for d, n, f in rows:
    if n in want:
        print(f"{'  ' * min(d, 6) + n:34s} {num(f[col[ka]]):9.1f} {num(f[col[kr]]):7.0f} {num(f[col[km]]):5.0f} {num(f[col[kd]]):4.0f}")
