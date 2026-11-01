#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Planted faults in the M2.5 viewer's own blocks (rtl/viewer), each built into a
# copy of the tree under sm3-builds/viewer/faults and run through the viewer
# bench (sim/viewer/tb_m3v.cpp) on one VF3 frame. A fault is caught when the
# bench fails.
#
#   python3 sim/viewer/faults_viewer.py [NAME ...]

import os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, 'sm3-builds', 'viewer', 'faults')
DUMP = os.environ.get('M3V_FAULT_DUMP', 'sm3-builds/census/dumps_vf3/f007860.bin.gz')

# name: (file, text, replacement)
FAULTS = {
    # R76: the set-up is the geometry's (SETUP_OUT); m3v_setup's S1-S3 went with it.
    # The store's own word 0, its list and its viewport drop, and two geometry
    # faults that only the bench's check of the geometry's words sees first
    'G1_ymin_word':  ('rtl/viewer/m3v_store.sv', "W_Y1 = HW + 5'd1,", "W_Y1 = HW + 5'd0,"),
    'G2_ymax_word':  ('rtl/viewer/m3v_store.sv', "W_Y3 = HW + 5'd5;", "W_Y3 = HW + 5'd3;"),
    'G3_list_bit':   ('rtl/viewer/m3v_store.sv', "wl <= {w_data[1:0], w_data[2]};", "wl <= {w_data[1:0], w_data[3]};"),
    'G4_vp_kept':    ('rtl/viewer/m3v_store.sv', "if (!(rw == 5'd0 ? w_data[31:27] == 5'd0 : wvp)) begin", "if (1'b1) begin"),
    'G5_hdr_len':    ('rtl/viewer/m3v_core.sv', "m3v_store #(.LIST_BASE(LISTS), .LOD(1'b1)) u_store", "m3v_store #(.LIST_BASE(LISTS), .LOD(1'b0)) u_store"),
    'G6_nw':         ('rtl/viewer/m3v_store.sv', "rb[0]   <= {9'd0, 5'(wn - 5'd1), ymax, ymin};", "rb[0]   <= {9'd0, 5'(wn - 5'd2), ymax, ymin};"),
    'G7_lod_copy':   ('rtl/real3d/r3d_geo.sv', "default: e_hw = e_inv ? negf(fb_q) : fb_q;", "default: e_hw = fb_q;"),
    # the nearest mip level (R75): the LOD terms, the level into both texel instances
    'P1_lod_beat':   ('rtl/viewer/m3v_raster.sv', "if (hk == 3'd4) th4 <= i_data[20:0];", "if (hk == 3'd3) th4 <= i_data[20:0];"),
    'P2_lv_addr':    ('rtl/viewer/m3v_raster.sv', ".i_fmt(a_fmt), .i_lv(r_lv),", ".i_fmt(a_fmt), .i_lv(4'd0),"),
    'P3_lv_filter':  ('rtl/viewer/m3v_raster.sv', ".i_fmt(rq_a[2:0]), .i_lv(b_lv),", ".i_fmt(rq_a[2:0]), .i_lv(4'd0),"),
    'P4_u_bits':     ('rtl/viewer/m3v_raster.sv', "ring_a[pn] <= {r_lv, r_u[27:0],", "ring_a[pn] <= {r_lv, 7'd0, r_u[20:0],"),
    'P5_lvm':        ('rtl/viewer/m3v_raster.sv', "wire [3:0]   t1_lvm = t1_q[20:17];", "wire [3:0]   t1_lvm = t1_q[20:17] - 4'd1;"),
    'T1_alpha_fwd':  ('rtl/viewer/m3v_store.sv', "ri <= alist ? ri - 16'd1 : ri + 16'd1;", "ri <= ri + 16'd1;"),
    'T2_skip':       ('rtl/viewer/m3v_store.sv', "rb[0][8:0] > b1", "rb[0][8:0] >= b1"),
    # the textured raster (R64's white-shader faults went with the shader)
    'R1_shade_plane':('rtl/viewer/m3v_raster.sv', "wire [1:0] sp = th0[4] ? 2'd3 : 2'd1;", "wire [1:0] sp = th0[4] ? 2'd2 : 2'd1;"),
    'R2_eo':         ('rtl/viewer/m3v_raster.sv', "pe[0], th0[4], th4};", "pe[1], th0[4], th4};"),
    'R3_word_lag':   ('rtl/viewer/m3v_raster.sv', "wq <= wdl[3'(wdp - 3'd3)];", "wq <= wdl[3'(wdp - 3'd2)];"),
    'R4_tx_ty':      ('rtl/viewer/m3v_raster.sv', "tab2[tag_n] <= {th0[5], th1[25:20], th1[30:26],", "tab2[tag_n] <= {th0[5], th1[26:21], th1[30:26],"),
    'R5_contour_zk': ('rtl/viewer/m3v_raster.sv', "wire [1:0] kind_zk = (kind == K_CONTOUR) ? 2'd2 :", "wire [1:0] kind_zk = (kind == K_CONTOUR) ? 2'd1 :"),
    'R6_mirror':     ('rtl/viewer/m3v_raster.sv', "th0[8:6], th0[11:9], th0[16], th0[17], th0[14:12],", "th0[8:6], th0[11:9], th0[17], th0[16], th0[14:12],"),
    'R7_ring_c':     ('rtl/viewer/m3v_raster.sv', "  assign ra_c = co_n;", "  assign ra_c = co_n + 7'd1;"),
    'R8_tex_flag':   ('rtl/viewer/m3v_raster.sv', ".i_use(f2_t ? 4'b1111 : 4'b0000)", ".i_use(4'b0000)"),
    'L2_tex_tile':   ('rtl/viewer/m3v_loader.sv', "{5'd0, ta[21:13], ta[10:2], ta[12:11]}", "{5'd0, ta[21:13], ta[10:2], ta[11], ta[12]}"),
    'M1_ddr3p_rbeat':('rtl/mem/m3_ddr3p.sv', "wire           r_last = DDRAM_DOUT_READY && (rbeat + 8'd1 == rh_n);", "wire           r_last = DDRAM_DOUT_READY && (rbeat + 8'd2 >= rh_n);"),
    'A1_sdr_words':  ('rtl/viewer/m3v_sdr_rd.sv', "push_a = 1'b0; push_b = 1'b0; pd_a = p_dout[31:0]; pd_b = p_dout[63:32];", "push_a = 1'b0; push_b = 1'b0; pd_a = p_dout[63:32]; pd_b = p_dout[31:0];"),
    'A2_ddr_half':   ('rtl/viewer/m3v_ddr_rd.sv', "wire in1 = (w + 24'd1 >= a0) && (w + 24'd1 < a1);", "wire in1 = (w + 24'd1 >= a0) && (w + 24'd1 <= a1);"),
    'C1_8e_base':    ('rtl/viewer/m3v_core.sv', "cb_sdr = slot_w + AW'(26'h20_0000) +", "cb_sdr = slot_w + AW'(26'h10_0000) +"),
    'L1_poly_lane':  ('rtl/viewer/m3v_loader.sv', "e_adr = {dw, (fo[2:1])};", "e_adr = {dw, ~(fo[2:1])};"),
}


def run(name):
    f, a, b = FAULTS[name]
    d = os.path.join(OUT, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    for sub in ('rtl', 'sim'):
        shutil.copytree(os.path.join(ROOT, sub), os.path.join(d, sub))
    shutil.copy(os.path.join(ROOT, 'Makefile'), d)
    p = os.path.join(d, f)
    s = open(p).read()
    if a not in s:
        return name, 'NOT APPLIED'
    open(p, 'w').write(s.replace(a, b, 1))
    obj = os.path.join(d, 'obj')
    r = subprocess.run(['make', '-C', d, os.path.join(obj, 'Vm3v_core'), 'VIEW_OBJ=' + obj], capture_output=True, text=True)
    if r.returncode != 0:
        return name, 'BUILD FAILED'
    os.symlink(os.path.join(ROOT, 'sm3-builds'), os.path.join(d, 'sm3-builds-root'))
    try:
        r = subprocess.run([os.path.join(obj, 'Vm3v_core'), '--vrom', os.path.join(ROOT, 'sm3-builds/viewer/vf3.v3v'),
                            '--dump', os.path.join(ROOT, DUMP), '--out', os.path.join(d, 'png')],
                           capture_output=True, text=True, timeout=int(os.environ.get('M3V_FAULT_TIMEOUT', '3600')))
    except subprocess.TimeoutExpired:
        # a fault that hangs the viewer is caught (the bench would time out)
        shutil.rmtree(obj, ignore_errors=True)
        return name, 'caught | hung: no frame within the time limit'
    out = r.stdout.strip().splitlines()
    caught = r.returncode != 0
    shutil.rmtree(obj, ignore_errors=True)
    for sub in ('rtl', 'sim'):
        shutil.rmtree(os.path.join(d, sub), ignore_errors=True)
    summary = [l for l in out if 'differ' in l or 'wrong' in l or 'TIMEOUT' in l or 'loader' in l]
    return name, ('caught' if caught else 'NOT CAUGHT') + ' | ' + ' / '.join(x.strip() for x in summary[:3])


if __name__ == '__main__':
    names = sys.argv[1:] or list(FAULTS)
    res = [run(n) for n in names]
    for n, r in res:
        print(f'{n:16s} {r}')
    print(f"{sum(1 for _, r in res if r.startswith('caught'))} of {len(res)} caught")
