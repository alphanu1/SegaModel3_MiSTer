#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the 100 MHz rasteriser blocks: each fault is one edit to
# a copy of the RTL in sm3-builds/rast/faults100/<name>/, built against its
# block's bench and run on 20 VF3 frames and a synthetic or random set. A
# fault is caught when the bench reports a mismatch.
#
#   python3 sim/real3d/faults_rast.py [name ...]

import os, re, shutil, subprocess, sys

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(R, "sm3-builds/rast/faults100")
TRIS = os.path.join(R, "sm3-builds/rast/tris_vf3")
VFLAGS = ["--cc", "--exe", "--build", "-j", "16", "-O3", "-Wno-fatal", "-Wno-UNUSEDSIGNAL",
          "-Wno-DECLFILENAME", "-Wno-WIDTHTRUNC"]
SIM = ["sim/real3d/r3d_rast.cpp", "sim/real3d/r3d_rast_rtl.cpp"]
DUMPS = os.path.join(R, "sm3-builds/census/dumps_vf3")
RAND = ["+verilator+rand+reset+2"]

FRONT = ["rtl/real3d/r3d_div_lane.sv", "rtl/real3d/r3d_rast_div.sv", "rtl/real3d/r3d_rast_edge.sv",
         "rtl/real3d/r3d_rast_front.sv"]
BLOCKS = {
    "front": dict(top="r3d_rast_front", files=FRONT, tb="sim/real3d/tb_r3d_rast_edge.cpp", defs="-DFRONT",
                  runs=[["--tris", TRIS, "--frames", "20", "--band", "16"],
                        ["--synthetic", "20000", "--band", "16"]]),
    "div": dict(top="r3d_rast_div", files=FRONT[:2], tb="sim/real3d/tb_r3d_rast_div.cpp", defs="",
                runs=[["--tris", TRIS, "--frames", "20"], ["--random", "200000"]]),
    "zspan": dict(top="r3d_rast_zspan", files=["rtl/real3d/r3d_rast_zspan.sv"], tb="sim/real3d/tb_r3d_rast_zspan.cpp", defs="",
                  runs=[["--tris", TRIS, "--frames", "20", "--oow"], ["--random", "200000"]]),
    "intake": dict(top="r3d_rast_intake", files=["rtl/real3d/r3d_rast_intake.sv"],
                   tb="sim/real3d/tb_r3d_rast_intake.cpp", defs="",
                   runs=[["--tris", TRIS, "--frames", "20", "--oow"], ["--random", "200000"]]),
    "zband": dict(top="r3d_rast_zband", files=["rtl/real3d/r3d_rast_zband.sv"], tb="sim/real3d/tb_r3d_rast_zband.cpp", defs="",
                  vflags=["--x-initial", "unique", "--x-assign", "unique"], ldflags="-lz -pthread",
                  runs=[["--tris", TRIS, "--dumps", DUMPS, "--frames", "20"] + RAND, ["--synthetic", "20000"] + RAND]),
    "interp": dict(top="r3d_rast_interp", files=["rtl/real3d/r3d_rast_recip.sv", "rtl/real3d/r3d_rast_interp.sv"],
                   tb="sim/real3d/tb_r3d_rast_interp.cpp", defs="", ldflags="-lz -pthread",
                   runs=[["--tris", TRIS, "--dumps", DUMPS, "--frames", "20"], ["--random", "200000"]]),
    "fogp": dict(top="r3d_rast_fogp", files=["rtl/real3d/r3d_rast_fogp.sv"], tb="sim/real3d/tb_r3d_rast_fogp.cpp", defs="",
                 runs=[["--tris", os.path.join(R, "sm3-builds/rast/tris_bassdx"), "--dumps", os.path.join(R, "sm3-builds/census/dumps_bassdx"),
                        "--frames", "20"], ["--random", "200000"]]),
    "colour": dict(top="r3d_rast_colour", files=["rtl/real3d/r3d_rast_colour.sv"], tb="sim/real3d/tb_r3d_rast_colour.cpp", defs="",
                   runs=[["--tris", os.path.join(R, "sm3-builds/rast/tris_bassdx"), "--dumps", os.path.join(R, "sm3-builds/census/dumps_bassdx"),
                          "--frames", "20"], ["--random", "200000"]]),
    "texel": dict(top="r3d_rast_texel", files=["rtl/real3d/r3d_rast_texel.sv"], tb="sim/real3d/tb_r3d_rast_texel.cpp", defs="",
                  runs=[["--tris", TRIS, "--dumps", DUMPS, "--frames", "20"], ["--random", "200000"]]),
    "tcache": dict(top="r3d_rast_tcache", files=["rtl/real3d/r3d_rast_tcache.sv"], tb="sim/real3d/tb_r3d_rast_tcache.cpp", defs="",
                   vflags=["--x-initial", "unique", "--x-assign", "unique"],
                   runs=[["--tris", TRIS, "--dumps", DUMPS, "--frames", "10"] + RAND, ["--random", "200000"] + RAND]),
    "band": dict(top="r3d_rast_band", files=["rtl/real3d/r3d_ram.sv", "rtl/real3d/r3d_rast_band.sv"], tb="sim/real3d/tb_r3d_rast_band.cpp", defs="",
                 vflags=["--x-initial", "unique", "--x-assign", "unique"],
                 runs=[["--tris", TRIS, "--dumps", DUMPS, "--frames", "10"] + RAND, ["--random", "200"] + RAND]),
    # mip levels (MIP = 1, study D-33): render_d25 with mip = 1, lod = 1
    "texelm": dict(top="r3d_rast_texel", files=["rtl/real3d/r3d_rast_texel.sv"], tb="sim/real3d/tb_r3d_rast_texel.cpp", defs="",
                   vflags=["-GMIP=1"], runs=[["--mip", "--tris", TRIS, "--dumps", DUMPS, "--frames", "20"], ["--mip", "--random", "200000"]]),
    "interpm": dict(top="r3d_rast_interp", files=["rtl/real3d/r3d_rast_recip.sv", "rtl/real3d/r3d_rast_interp.sv"],
                    tb="sim/real3d/tb_r3d_rast_interp.cpp", defs="", ldflags="-lz -pthread", vflags=["-GMIP=1"],
                    runs=[["--mip", "--tris", TRIS, "--dumps", DUMPS, "--frames", "20"], ["--mip", "--random", "200000"]]),
    "intakem": dict(top="r3d_rast_intake", files=["rtl/real3d/r3d_rast_intake.sv"], tb="sim/real3d/tb_r3d_rast_intake.cpp",
                    defs="-DINTAKE_MIP", vflags=["-GMIP=1"], runs=[["--tris", TRIS, "--frames", "20", "--oow"], ["--random", "200000"]]),
}

# name: (block, file, old, new)
FAULTS = {
    "D1_neg_bit":     ("div", "rtl/real3d/r3d_rast_div.sv", "neg[j]  <= dxs[j][32];", "neg[j]  <= dxs[j][31];"),
    "D2_skip_r0":     ("front", "rtl/real3d/r3d_rast_div.sv", "g_r[j] = {16'd0, nn[53:38]};", "g_r[j] = {16'd0, nn[52:37]};"),
    "D3_exact_tie":   ("front", "rtl/real3d/r3d_div_lane.sv", "if (!s[32]) step", "if (!s[32] && s[31:0] != 32'd0) step"),
    "E1_prod_short":  ("front", "rtl/real3d/r3d_rast_edge.sv", "if (m_d1[31:2] == 30'd0 && m_d2[31:2] == 30'd0)",
                       "if (m_d1[31:4] == 28'd0 && m_d2[31:4] == 28'd0)"),
    "E2_band_start":  ("front", "rtl/real3d/r3d_rast_edge.sv", "wire signed [16:0] ys  = (z_r1 > b0s) ? z_r1 : b0s;",
                       "wire signed [16:0] ys  = (z_r1 > b0s + 17'sd1) ? z_r1 : b0s;"),
    "E3_upper_late":  ("front", "rtl/real3d/r3d_rast_edge.sv", "w_up <= (w_y + 17'sd1 < w_r2);", "w_up <= (w_y + 17'sd1 <= w_r2);"),
    "E4_clamp_hi":    ("front", "rtl/real3d/r3d_rast_edge.sv", "wire signed [19:0] hi  = (hi0 > W20) ? W20 : hi0;",
                       "wire signed [19:0] hi  = (hi0 > W20 - 20'sd1) ? W20 - 20'sd1 : hi0;"),
    "I1_no_round":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "{38'd0, s_q[0]}", "{38'd0, 1'b0}"),
    "I2_grad_exp":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "(grad ? 9'd9 : 9'd0)", "(grad ? 9'd8 : 9'd0)"),
    "I3_vtx_sat":     ("intake", "rtl/real3d/r3d_rast_intake.sv", "f_w[30:23] >= 8'd142", "f_w[30:23] >= 8'd143"),
    "I4_zero_e":      ("intake", "rtl/real3d/r3d_rast_intake.sv", "(mx == 9'd0) ? 9'sd2 :", "(mx == 9'd0) ? 9'sd1 :"),
    "I5_nan_sign":    ("intake", "rtl/real3d/r3d_rast_intake.sv", "else if (r_kind == K_VTX && r_nan) o_data <= 40'h00_8000_0000;",
                       "else if (r_kind == K_VTX && r_nan) o_data <= 40'h00_7fff_ffff;"),
    "Z1_man_window":  ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "zw2 = {ex[7:0], sh[40:25]};", "zw2 = {ex[7:0], sh[39:24]};"),
    "Z2_exp_bias":    ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "11'sd86;", "11'sd87;"),
    "Z3_lane1_init":  ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "w_l1 <= n_acc + {n_dx[40:0], 1'b0};", "w_l1 <= n_acc + n_dx;"),
    "Z4_zero_test":   ("zspan", "rtl/real3d/r3d_rast_zspan.sv", "    if (v[41]) n = 7'd64;\n", "    if (v[41]) n = 7'd64;\n    if (v == 42'sd0) n = 7'd0;\n"),
    "B1_no_fwd2":     ("zband", "rtl/real3d/r3d_rast_zband.sv", "(w2v0 && c_m20) ? w2d0 : c_qv0", "c_qv0"),
    "B2_fwd_order":   ("zband", "rtl/real3d/r3d_rast_zband.sv",
                       "(w1v1 && c_m11) ? w1d1 : (w2v1 && c_m21) ? w2d1",
                       "(w2v1 && c_m21) ? w2d1 : (w1v1 && c_m11) ? w1d1"),
    "B3_no_stamp":    ("zband", "rtl/real3d/r3d_rast_zband.sv", "c_qv0 <= q0[29:24] == gen;", "c_qv0 <= q0[28:24] == gen[4:0];"),
    "B4_sweep_31":    ("zband", "rtl/real3d/r3d_rast_zband.sv", "localparam int NSW = 32;", "localparam int NSW = 31;"),
    "B5_no_board":    ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire       sb_hit = (sb_n[a_row] != 7'd0) &&", "wire       sb_hit = 1'b0 &&"),
    "B6_no_bc_hold":  ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire       ctr_bc = (b_v &&", "wire       ctr_bc = 1'b0 && (b_v &&"),
    "B7_strict":      ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire        n1 = OOW ? (cz_b1 >= r1)", "wire        n1 = OOW ? (cz_b1 > r1)"),
    "B8_lane_order":  ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire           h_l    = pf_lane || !h_m[0];", "wire           h_l    = !pf_lane && h_m[1];"),
    "B10_old_gen_wr": ("zband", "rtl/real3d/r3d_rast_zband.sv", "v_wr   <= c_zw && (h_gen == gen);", "v_wr   <= c_zw;"),
    "B11_wr_in_sweep": ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire vport    = v_full && v_wr && !sw_on;", "wire vport    = v_full && v_wr;"),
    "B9_range_hi":    ("zband", "rtl/real3d/r3d_rast_zband.sv", "wire [8:0]  p_hi = c_pass[1] ? c_x1 : c_x;", "wire [8:0]  p_hi = c_x;"),
    "N1_no_round_up": ("interp", "rtl/real3d/r3d_rast_recip.sv", "up9 <= !dlo[29] && !p8;", "up9 <= 1'b0;"),
    "N2_lag_10":      ("interp", "rtl/real3d/r3d_rast_interp.sv", "localparam int LAG = 11;", "localparam int LAG = 10;"),
    "N3_shade_floor": ("interp", "rtl/real3d/r3d_rast_interp.sv", "else o_shade <= ss15 ? 32'd0 - qs15 : qs15;", "else o_shade <= ss15 ? ~qs15 : qs15;"),
    "N4_u_shift":     ("interp", "rtl/real3d/r3d_rast_interp.sv", "shu2 <= 10'(n1) + 10'sd24", "shu2 <= 10'(n1) + 10'sd23"),
    "N5_sat_at_32":   ("interp", "rtl/real3d/r3d_rast_interp.sv", "(top >= 11'sd31)", "(top >= 11'sd32)"),
    "N6_dx_even":     ("interp", "rtl/real3d/r3d_rast_interp.sv", "a = dx[17:0]; b = dy[17:0]; kx = {x, 1'b1};", "a = dx[17:0]; b = dy[17:0]; kx = {x, 1'b0};"),
    "F1_sh_bias":     ("fogp", "rtl/real3d/r3d_rast_fogp.sv", "- 11'sd123;", "- 11'sd122;"),
    "F2_sat_early":   ("fogp", "rtl/real3d/r3d_rast_fogp.sv", "else if (sh >= -11'sd34)", "else if (sh >= -11'sd35)"),
    "F3_no_neg_k":    ("fogp", "rtl/real3d/r3d_rast_fogp.sv", "kz = i_k[31] ||", "kz = 1'b0 &&"),
    "C1_no_wrap":     ("colour", "rtl/real3d/r3d_rast_colour.sv", "q = lo2[c][31:0] + {hi2[c], 27'd0};",
                       "q = lo2[c][31:0];"),
    "C2_contour_f7":  ("colour", "rtl/real3d/r3d_rast_colour.sv", "(i_kind == K_CONT && ta < 8'hf8)", "(i_kind == K_CONT && ta < 8'hf7)"),
    "C3_trans_shift": ("colour", "rtl/real3d/r3d_rast_colour.sv", "{5'd0, i_trans[5:2]}", "{5'd0, i_trans[5:3], 1'b0}"),
    "C4_fog_floor":   ("colour", "rtl/real3d/r3d_rast_colour.sv", "11'(d4[c] >>> 10)", "11'(d4[c] / 1024)"),
    "C5_fog_g":       ("colour", "rtl/real3d/r3d_rast_colour.sv", "g2 <= gg[14:4];", "g2 <= gg[15:5];"),
    "T1_mirror_seam": ("texel", "rtl/real3d/r3d_rast_texel.sv", "mir = (c < p) ? c :", "mir = (c <= p) ? c :"),
    "T2_fmt4_nibble": ("texel", "rtl/real3d/r3d_rast_texel.sv", "3'd4: dec = {p4(p[11:8]), {3{p4(p[15:12])}}};", "3'd4: dec = {p4(p[15:12]), {3{p4(p[11:8])}}};"),
    "T3_bank_pick":   ("texel", "rtl/real3d/r3d_rast_texel.sv", "else if (k2 == b) a = a2; else a = a3;", "else a = a3;"),
    "T4_v_lerp_half": ("texel", "rtl/real3d/r3d_rast_texel.sv", "o = 24'({1'b0, t6[c], 8'd0} + 24'(p6[c]));", "o = 24'({1'b0, t6[c], 8'd0} + 24'(p6[c]) + 24'd128);"),
    "T5_u2_no_wrap":  ("texel", "rtl/real3d/r3d_rast_texel.sv", "u1a <= a; u1b <= (a + 13'd1) & um;", "u1a <= a; u1b <= (a + 13'd1);"),
    "K1_beat_wrap":   ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "  logic [FB+2:0] bb_wp, bb_rp; ", "  logic [FB+1:0] bb_wp, bb_rp; "),
    "K2_no_gate":     ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "wire         gate_ok = rd_n == fq_r[DB:0];", "wire         gate_ok = 1'b1;"),
    "K3_no_busy":     ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "busy[w] = 1'b1;", "busy[w] = 1'b0;"),
    "K4_need_early":  ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "($signed(done_n - ph_need) >= 0)", "($signed(done_n - ph_need) >= -1)"),
    "K5_lane_swap":   ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "dwd0 = {bq[47:32], bq[15:0]};", "dwd0 = {bq[15:0], bq[47:32]};"),
    "K6_dedup_line":  ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "if (a_use[b] && !a_ok[b] && dup[b])", "if (a_use[b] && !a_ok[b] && mbh[b])"),
    "K7_j_swap":      ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "c_j[b] <= {l0_a[b][10], l0_a[b][0]};", "c_j[b] <= {l0_a[b][0], l0_a[b][10]};"),
    "K8_no_valid":    ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "if (e[EB-1] && e[TB-1:0] == tag_of(c_ln[b]))", "if (e[TB-1:0] == tag_of(c_ln[b]))"),
    "K9_no_wq_fwd":   ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "if (wq_v && set_of(c_ln[b]) == wq_set)", "if (1'b0 && wq_v && set_of(c_ln[b]) == wq_set)"),
    "K10_no_w_fwd":   ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "if (w_v  && set_of(c_ln[b]) == w_set)", "if (1'b0 && w_v && set_of(c_ln[b]) == w_set)"),
    "K12_row_stale":  ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "for (int b = 0; b < 4; b++) a_row[b] <= a_rowe[b];", "for (int b = 0; b < 4; b++) a_row[b] <= a_row[b];"),
    "K13_no_reread":  ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "tra[b] = c_go ? set_of(l0_ln[b]) : set_of(c_ln[b]);", "tra[b] = set_of(l0_ln[b]);"),
    "K14_rr_stuck":   ("tcache", "rtl/real3d/r3d_rast_tcache.sv", "rr <= vic + WB'(1);", "rr <= vic;"),
    "G1_no_w_haz": ("band", "rtl/real3d/r3d_rast_band.sv", "|| (w_v && w_a == p0_a && w_k == p0_x[0]));", ");"),
    "G2_no_p3_haz": ("band", "rtl/real3d/r3d_rast_band.sv", "(p3_v && p3_a == p0_a && p3_k == p0_x[0]) || ", ""),
    "G3_red_f": ("band", "rtl/real3d/r3d_rast_band.sv", "w_d[23:16] <= 8'({1'b0, p3_d[23:16]} + 9'(p3_rg[34:26]) - {1'b0, p3_f});", "w_d[23:16] <= 8'({1'b0, p3_d[23:16]} + 9'(p3_rg[34:26]));"),
    "G4_pack_shift": ("band", "rtl/real3d/r3d_rast_band.sv", "9'(p3_rg[16:8])", "9'(p3_rg[17:9])"),
    "G5_shown_raw": ("band", "rtl/real3d/r3d_rast_band.sv", "else if (w[23:0] != 24'd0) shown = {7'd0, 1'b1, sf_out};", "else if (w[23:0] != 24'd0) shown = {7'd0, 1'b1, w[23:0]};"),
    "G6_clear_next": ("band", "rtl/real3d/r3d_rast_band.sv", "+ 10'd16;   // the row of the band two ahead", "+ 10'd8;   // the row of the band two ahead"),
    "G7_rect_edge": ("band", "rtl/real3d/r3d_rast_band.sv", "x >= sf_x0 && x < sf_x1", "x >= sf_x0 && x <= sf_x1"),
    "G8_wr_all_bufs": ("band", "rtl/real3d/r3d_rast_band.sv", "&& 1'(i) == cur; bwa[i][k]", "; bwa[i][k]"),
    "G9_eob_no_drain": ("band", "rtl/real3d/r3d_rast_band.sv", "wire eob_go = p0_v && p0_eob && !inflight && cst == C_IDLE;", "wire eob_go = p0_v && p0_eob && cst == C_IDLE;"),
    "G10_lane_swap": ("band", "rtl/real3d/r3d_rast_band.sv", "assign m_din = {7'd0, fq[fq_rp][49:25], 7'd0, fq[fq_rp][24:0]};", "assign m_din = {7'd0, fq[fq_rp][24:0], 7'd0, fq[fq_rp][49:25]};"),
    "G11_bank_swap": ("band", "rtl/real3d/r3d_rast_band.sv", "always_comb d1 = p1_k ? brq[cur][1][23:0] : brq[cur][0][23:0];", "always_comb d1 = p1_k ? brq[cur][0][23:0] : brq[cur][1][23:0];"),
    # mip levels: the LOD, the sheet bases, the clamp, the triangle's term
    "M1_lod_plus1":   ("interpm", "rtl/real3d/r3d_rast_interp.sv", "else lv2 = d[11:8];", "else lv2 = d[11:8] + 4'd1;"),
    "M2_round_fold":  ("intakem", "rtl/real3d/r3d_rast_intake.sv", "17'(17'd97664 -", "17'(17'd97536 -"),
    "M3_base_x":      ("texelm", "rtl/real3d/r3d_rast_texel.sv", "xt = {th, 11'd0} | {15'd0, i_tx, 5'd0};", "xt = {1'b0, th, 10'd0} | {15'd0, i_tx, 5'd0};"),
    "M4_base_y":      ("texelm", "rtl/real3d/r3d_rast_texel.sv", "yt = {th, 10'd0} | {15'd0, i_ty, 5'd0};", "yt = {1'b0, th, 9'd0} | {15'd0, i_ty, 5'd0};"),
    "M5_no_clamp_hi": ("interpm", "rtl/real3d/r3d_rast_interp.sv", "else if (d[16:8] >= {5'd0, lvm2}) lv2 = lvm2;", "else if (1'b0) lv2 = lvm2;"),
    "M6_lvmax":       ("intakem", "rtl/real3d/r3d_rast_intake.sv", "lvm_r <= 4'd4 +", "lvm_r <= 4'd5 +"),
    "M7_no_clamp_lo": ("interpm", "rtl/real3d/r3d_rast_interp.sv", "if (!MIP || d[17]) lv2 = 4'd0;", "if (!MIP) lv2 = 4'd0;"),
    "M8_frac_level0": ("texelm", "rtl/real3d/r3d_rast_texel.sv", "fu1 <= ul[7:0]; fv1 <= vl[7:0];", "fu1 <= i_u[7:0]; fv1 <= vl[7:0];"),
    "M9_mask_level0": ("texelm", "rtl/real3d/r3d_rast_texel.sv", "pw = 13'd1 << su; ph = 13'd1 << sv;", "pw = 13'd32 << i_tw; ph = 13'd1 << sv;"),
    "M10_z_exp":      ("interpm", "rtl/real3d/r3d_rast_interp.sv", "ex2 <= 10'(n1) + {eo1[8], eo1} + 10'sd86;", "ex2 <= 10'(n1) + {eo1[8], eo1} + 10'sd87;"),
    "M11_b_sign":     ("intakem", "rtl/real3d/r3d_rast_intake.sv", "ok = !i_data[31] && i_data[30:23] != 8'hff", "ok = i_data[30:23] != 8'hff"),
    "M12_table_b":    ("intakem", "rtl/real3d/r3d_rast_intake.sv", "log2t(i_data[22:17])", "log2t(i_data[21:16])"),
}


def run(name):
    blk, fn, old, new = FAULTS[name]
    b = BLOCKS[blk]
    d = os.path.join(OUT, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    files = []
    for f in b["files"]:
        src = open(os.path.join(R, f)).read()
        if f == fn:
            assert src.count(old) == 1, (name, old)
            src = src.replace(old, new)
        dst = os.path.join(d, os.path.basename(f))
        open(dst, "w").write(src)
        files.append(dst)
    cmd = ["verilator"] + VFLAGS + b.get("vflags", []) + ["--top-module", b["top"], "-CFLAGS",
           "-O2 -std=c++17 -ffp-contract=off -I%s/sim/real3d %s" % (R, b["defs"]), "-LDFLAGS", b.get("ldflags", "-lz"),
           "--Mdir", os.path.join(d, "obj")] + files + [os.path.join(R, b["tb"])] + [os.path.join(R, s) for s in SIM]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    exe = os.path.join(d, "obj", "V" + b["top"])
    res = []
    for a in b["runs"]:
        try:
            p = subprocess.run([exe] + a, capture_output=True, text=True, timeout=b.get("timeout"))
        except subprocess.TimeoutExpired:
            res.append(-2)
            continue
        if b.get("exit"):
            m = re.search(r"(\d+) pixels wrong in", p.stdout)
            res.append((int(m.group(1)) if m else 0) or (1 if p.returncode else 0))
        else:
            m = re.search(r"(\d+) wrong", p.stdout)
            res.append(int(m.group(1)) if m else -1)
    shutil.rmtree(os.path.join(d, "obj"), ignore_errors=True)
    caught = any(x != 0 for x in res)
    print("%-16s %-7s %-40s %s" % (name, blk, " / ".join("%d wrong" % x for x in res), "caught" if caught else "NOT caught"), flush=True)
    return caught


if __name__ == "__main__":
    names = sys.argv[1:] or list(FAULTS)
    n = sum(run(x) for x in names)
    print("%d of %d caught" % (n, len(names)))
