#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Planted faults for the Real3D geometry (make test_real3d_faults). Each one
# goes into a copy of rtl/real3d under sm3-builds/r3d/faults2/ (R3D_FAULTDIR), builds the bench, runs
# 20 VF3 and 20 Bass Fishing frames (F5: all 400 Bass frames) of the default build
# (Supermodel frustum and culling, every D-19 row, the facing test; F13 MAME's
# correctly rounded frustum; D12 the build without culling). Prints the result.
# The D faults are the D-19 rows' (m2-notes section 11), the FA faults the
# facing test's and the R62 record's (section 15): FA3b and D2 run the
# FACE_DROP = 0 build, FA11 and FA12 add 20 Daytona 2 frames (Step 2.1), D11
# and N2 a 90% output stall. The FO and FS faults are the read timing of the
# output buffer and the side memories in M10K (study D-32). The T faults are
# read timing at the memory ports (m2-notes section 15): they run the bench's
# slow memory, every port acknowledged 50-400 cycles late with one read held.
# The SU faults are the set-up port's (SETUP_OUT, m2-notes section 15): they run
# the SETUP_OUT build and check its set-up records (--setup-out).
import os, shutil, subprocess, sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FAULTS = {
 'F1_stack_pop':    ('r3d_geo.sv', "  assign lv_ra = sp - 5'd1;", "  assign lv_ra = sp - 5'd2;"),
 'F2_vertex_reuse': ('r3d_geo.sv', "nref[kk] <= vref[s_];", "nref[kk] <= vref[3 - s_];"),
 'F3_clip_order':   ('r3d_geo.sv', "    paxis = p[0] ? {~a[31], a[30:0]} : a;", "    paxis = (p[0] ^ (p == 3'd1 || p == 3'd2)) ? {~a[31], a[30:0]} : a;"),
 'F4_recip_round':  ('r3d_recip.sv', "if (up10) q = q10 + 25'd1;", "if (1'b0) q = q10 + 25'd1;"),
 'F5_viewport_xy':  ('r3d_geo.sv', "5'd7: begin xb_addr <= 5'(R_BC_VX);", "5'd7: begin xb_addr <= 5'(R_BC_VY);"),
 'F6_pass_label':   ('r3d_walker.sv', "ev_c <= {30'd0, pass};", "ev_c <= {30'd0, pass[0], pass[1]};"),
 'F7_colour_table': ('r3d_geo.sv', "ms_caddr <= SM_CTAB ? d_ctab + {8'd0, h4[19:8]}", "ms_caddr <= SM_CTAB ? d_ctab + 20'd1 + {8'd0, h4[19:8]}"),
 'F8_shading_lit':  ('r3d_geo.sv', "ra_q[28] ? F_255 : ash, cv_q", "ra_q[28] ? ash : F_255, cv_q"),
 'F9_intensity_limit': ('r3d_vpu.sv', "else if ((wq[11].post == 2'd1 || wq[11].post == 2'd2) && !sum[31]", "else if ((wq[11].post == 2'd1) && !sum[31]"),
 'F10_tex_x':       ('r3d_geo.sv', "h5[4:0], h4[4:0], h5[7], h4[6]", "h5[4:0], h4[4:0], h5[6], h4[6]"),
 'F11_viewport_wh': ('r3d_geo.sv', "5'd6: begin xb_addr <= 5'(R_BC_VW);", "5'd6: begin xb_addr <= 5'(R_BC_VH);"),
 'F12_tls_bank':    ('r3d_geo.sv', "k_tbank <= c_bank;", "k_tbank <= !c_bank;"),
 'F13_cordic_round':('r3d_setup_cr.sv', "rm <= {1'b0, T[63:40]} + {24'd0, T[39] && ((|T[38:0]) || T[40])};", "rm <= {1'b0, T[63:40]};"),
 # overlap / interleave logic
 'N1_chunk_order':  ('r3d_geo.sv', "cp <= cq_start[cq_rp]; pc_end <= cq_end[cq_rp]; pc_ring <= cq_ring[cq_rp];",
                                   "cp <= cq_start[cq_rp]; pc_end <= cq_end[2'(cq_rp + 2'd1)]; pc_ring <= cq_ring[cq_rp];"),
 'N2_buffer_hazard':('r3d_geo.sv', "P_PJ0: if (!po_busy[pbuf] && eb_cnt != 2'd2)", "P_PJ0: if (1'b1)"),
 'N3_pbuf_tag':     ('r3d_geo.sv', "k_pbuf <= pbuf; vk_cls <= 2'd2;", "k_pbuf <= !pbuf; vk_cls <= 2'd2;"),
 'N4_ftz_sign':     ('r3d_fp_mul.sv', "        result <= {s3_sign, 8'h00, 23'd0};\n      end else if (s3_ovf)", "        result <= 32'h0000_0000;\n      end else if (s3_ovf)"),
 'N5_vs_window':    ('r3d_geo.sv', "M_CHW: if (dq_cnt != 2'd0 && cq_cnt < 3'd3)", "M_CHW: if (dq_cnt != 2'd0 && cq_cnt < 3'd4)"),
 'N6_reuse_window': ('r3d_geo.sv', "vin[v] <= vref_ok[v] && (chunk - vref_chunk[v]) < 8'd2;", "vin[v] <= vref_ok[v] && (chunk - vref_chunk[v]) < 8'd4;"),
 'N8_sm_frustum':   ('r3d_geo.sv', "xw_data[2] <= F_M1; xw_data[3] <= F_M1;", "xw_data[2] <= F_M1; xw_data[3] <= F_ONE;"),
 'N9_cull_verdict': ('r3d_geo.sv', "cu_out <= rlt(rr, x_rdata[0]) || rlt(rr, x_rdata[1]) || rlt(rr, x_rdata[2]) || rlt(rr, x_rdata[3]);",
                                   "cu_out <= rlt(rr, x_rdata[0]) || rlt(rr, x_rdata[1]) || rlt(rr, x_rdata[2]);"),
 # D-19 rows and the stream
 # row 18's face normals are a model switch now (study D-25): D1 aims at the
 # (2b + 1) / 255 normal conversion instead
 'D1_normal_conv':  ('r3d_geo.sv', "else tc_k <= k255a(kb[7], {kb[6:0] ^ {7{kb[7]}}, 1'b1});", "else tc_k <= k255a(1'b0, kb);"),
 'D2_double_second':('r3d_geo.sv', "if (e_two) begin e_inv <= 1'b0; e_two <= 1'b0; est <= E_P2; end", "if (1'b0) begin e_inv <= 1'b0; e_two <= 1'b0; est <= E_P2; end"),
 'D3_sun_clamp':    ('r3d_vpu.sv', "if (wq[11].post == 2'd1 && neg) pz = 1'b1;", "if (1'b0) pz = 1'b1;"),
 'D4_colour_base':  ('r3d_walker.sv', "ctab <= {rbuf[6][27:25], rbuf[5][31:28], rbuf[1][31:19]};", "ctab <= {rbuf[6][27:25], rbuf[5][31:28], rbuf[1][30:18]};"),
 'D5_fixed_sign':   ('r3d_geo.sv', "if (amb_ld || (tq_fw && tq_spec)) tc_k", "if (amb_ld || (tq_fw && !tq_spec)) tc_k"),
 'D6_trans_drop':   ('r3d_geo.sv', "drop = SM_TRANS && !h6[23] && h6[22:18] == 5'd0;", "drop = SM_TRANS && !h6[23] && h6[22:18] == 5'd1;"),
 'D7_light_z':      ('r3d_geo.sv', "xb_data <= SM_LIGHT ? negf(vq_r) : vq_r; end", "xb_data <= vq_r; end"),
 'D8_normal_matrix':('r3d_vpu.sv', "assign u_bc0   = (SM_LIGHT && u_tr)", "assign u_bc0   = (1'b0 && u_tr)"),
 'D9_reuse_key':    ('r3d_geo.sv', "reuse_ok = vin[v] && (p_fixed ? vkfix[v] : !vkfix[v]);", "reuse_ok = vin[v];"),
 'D10_copy_shade':  ('r3d_geo.sv', "(c == 3'd6 && e_inv && !re_q[90]) ? 3'd7 : c", "c"),
 'D11_list_hazard': ('r3d_geo.sv', "end else if (po_busy[pbuf]) begin", "end else if (1'b0) begin"),
 'D12_trans_latch': ('r3d_geo.sv', "vw_wd <= (cnt == 5'd1) ? vq_r : cr_addr32;", "vw_wd <= (cnt == 5'd1) ? ev_b : ev_c;"),
 'D13_const_load':  ('r3d_geo.sv', "3'd2: xb_data <= 32'h3f00_0000;   // 0.5", "3'd2: xb_data <= 32'h3f00_0001;   // 0.5"),
 'D14_div_ring':    ('r3d_vpu.sv', "if (dv[11] && dl[11] == 2'd3) drp <= drp + 2'd1;", "if (dv[11] && dl[11] == 2'd2) drp <= drp + 2'd1;"),
 # the area pass (m2-notes section 14)
 'G1_slot_alloc':   ('r3d_geo.sv', "for (int j = 0; j < 4; j++) if (2'(j) != iv[1:0]) used[cmap[j]] = 1'b1;", "for (int j = 0; j < 4; j++) if (2'(j) > iv[1:0]) used[cmap[j]] = 1'b1;"),
 'G2_slot_reuse':   ('r3d_geo.sv', "cmap[kk] <= pmap[s_];", "cmap[kk] <= pmap[kk];"),
 'G3_smfr_const':   ('r3d_geo.sv', "5'd6: begin xw_addr <= 8'(R_SF0); xw_data[1] <= F_ONE; xw_data[2] <= F_ONE; end", "5'd6: begin xw_addr <= 8'(R_SF0); xw_data[1] <= F_ONE; end"),
 'G4_conv_ebase':   ('r3d_geo.sv', "(n == 5'd6 || n == 5'd8) ? 8'd125 : 8'd123", "(n == 5'd6 || n == 5'd7) ? 8'd125 : 8'd123"),
 'G5_outcode_eq':   ('r3d_geo.sv', "opair[1] = ok && (bz || (w[31] ? (a[31] && !lt) : (a[31] || lt || eq)));", "opair[1] = ok && (bz || (w[31] ? (a[31] && !lt) : (a[31] || lt)));"),
 'G6_amb_byte':     ('r3d_geo.sv', "amb_b  <= vw_q[15:8];", "amb_b  <= vw_q[7:0];"),
 'G7_div_rem':      ('r3d_recip.sv', "up10 <= !dlo[29];", "up10 <= !d[29];"),
 'G8_region_end':   ('r3d_walker.sv', "({1'b0, rd_addr[2:0]} + {1'b0, rd_beat} >= 4'd8)", "({1'b0, rd_addr[2:0]} + {1'b0, rd_beat} >= 4'd7)"),
 'G9_record_bits':  ('r3d_geo.sv', "assign re_q = {re_sq[94:23], re_sq[43], re_sq[46], re_sq[22:0]};", "assign re_q = {re_sq[94:23], re_sq[46], re_sq[43], re_sq[22:0]};"),
 'G10_matrix_bank': ('r3d_geo.sv', "xb_we <= 1'b1; xb_bank <= cnt[1:0]; xb_addr <= 5'(R_BC_MB) + 5'(cnt[3:2]);", "xb_we <= 1'b1; xb_bank <= cnt[3:2]; xb_addr <= 5'(R_BC_MB) + 5'(cnt[1:0]);"),
 'G11_addr_adder':  ('r3d_geo.sv', "      M_VP3: o = 6'd26;", "      M_VP3: o = 6'd25;"),
 'G5b_outcode_sign':('r3d_geo.sv', "opair[0] = ok && (bz || (w[31] ? (!a[31] && !lt) : (!a[31] || lt || eq)));", "opair[0] = ok && (bz || (w[31] ? (!a[31] && lt) : (!a[31] || lt || eq)));"),
 'G11b_addr_adder': ('r3d_geo.sv', "      M_VP1: o = 6'd12;", "      M_VP1: o = 6'd13;"),
 'G12_result_mux':  ('r3d_vpu.sv', "assign wd = wpz ? 32'd0 : wpk ? PK : wd_r;", "assign wd = wd_r;"),
 # the facing test beside the pool and the R62 record (m2-notes section 15)
 'FA1_facing_sign': ('r3d_face.sv', "v_gt <= !sum[31] && !s_zero && !s_nan;", "v_gt <= sum[31] && !s_zero && !s_nan;"),
 'FA2_normal_matrix':('r3d_face.sv', "op_ak = A_NRM;\n      op_ba = {mbs[cur_slot], r, j};", "op_ak = A_NRM;\n      op_ba = {mbs[cur_slot], j, r};"),
 'FA3_copy_choice': ('r3d_geo.sv', "sc = dbl && !(FACE_DROP && e_lt);", "sc = dbl && !(FACE_DROP && e_gt);"),
 'FA3b_cull_flag':  ('r3d_geo.sv', "3'd3: e_hw = {25'd0, SM_CULL && e_cf, re_q[96], re_q[95:91]};", "3'd3: e_hw = {25'd0, SM_CULL && e_pf, re_q[96], re_q[95:91]};"),
 'FA4_fog_bits':    ('r3d_geo.sv', "re_wd <= {h6[17], h6[15:11], p_fixed,", "re_wd <= {h6[17], h6[14:10], p_fixed,"),
 'FA5_layer_bit':   ('r3d_geo.sv', "re_wd <= {h6[17], h6[15:11], p_fixed,", "re_wd <= {h6[16], h6[15:11], p_fixed,"),
 'FA6_vp_words':    ('r3d_geo.sv', "(vr_k == 3'd2) ? 5'd2 : (vr_k == 3'd3) ? 5'd3", "(vr_k == 3'd2) ? 5'd3 : (vr_k == 3'd3) ? 5'd2"),
 'FA7_vp_kz':       ('r3d_geo.sv', "3'd5: e_vw = FRUSTUM_SM ? F_ONE : negf(F_P23);", "3'd5: e_vw = FRUSTUM_SM ? F_TWO : negf(F_P23);"),
 'FA8_oow_word':    ('r3d_geo.sv', "vcomp = (k < 3'd3) ? k : k + 3'd1;", "vcomp = (k < 3'd2) ? k : k + 3'd1;"),
 'FA9_matrix_wait': ('r3d_face.sv', "nv = pend && tv[mbs[cp]] && !(CMUL && c_pend);", "nv = pend && !(CMUL && c_pend);"),
 'FA10_slot_gate':  ('r3d_geo.sv', "f_we = (f_nw || f_vw || f_rw) && f_ready;", "f_we = (f_nw || f_vw || f_rw);"),
 'FA11_step15_scale':('r3d_face.sv', "(LOD && ak_q == A_NP) ? 8'd115 : step15 ? 8'd116 : 8'd120;", "(LOD && ak_q == A_NP) ? 8'd115 : 8'd120;"),
 'FA12_step15_node':('r3d_walker.sv', "rd_addr <= {8'd0, baddr} + 32'd3;", "rd_addr <= {8'd0, baddr} + 32'd2;"),
 'FA13_back_skip':  ('r3d_geo.sv', "else if (SM_CULL && FACE_DROP && !ra_q[30] && fv_q[1]) cp <= cp + 6'd1;", "else if (SM_CULL && FACE_DROP && fv_q[1]) cp <= cp + 6'd1;"),
 'FA14_reused_v0':  ('r3d_geo.sv', "(SM_CULL && (dst == D_HDR || dst == D_CHK)) ? {fk, pmap[v0src]}", "(SM_CULL && (dst == D_HDR || dst == D_CHK)) ? {fk, pmap[2'(v0src + 2'd1)]}"),
 'N7_div_lane':     ('r3d_vpu.sv', "for (int k = 1; k < 4; k++) begin dq_x[k] <= opA[k]; dq_a[k] <= opB[k]; end",
                                   "for (int k = 1; k < 4; k++) begin dq_x[k] <= opA[k]; dq_a[k] <= opB[4 - k]; end"),
 # study R104: the divide's lane shift register, the reciprocal's exponent correction a stage on
 'N10_div_shift':   ('r3d_vpu.sv', "dq_x[2] <= dq_x[3]; dq_a[2] <= dq_a[3];", "dq_a[2] <= dq_a[3];"),
 'N11_recip_ge':    ('r3d_recip.sv', "c1.re <= c0.re - {9'd0, !c0.ge};", "c1.re <= c0.re;"),
 # study D-32: the output buffer and the side memories in M10K by default, their read timing
 'FO1_out_noahead': ('r3d_geo.sv', "    if (OUT_M10K) begin\n      if (est != E_OUT || !e_vtx)", "    if (1'b0) begin\n      if (est != E_OUT || !e_vtx)"),
 'FO2_out_stall':   ('r3d_geo.sv', "      else if (out_go) begin\n        if (e_c == 3'd5) begin q = e_q + 4'd1; c = 3'd0; end",
                                   "      else begin\n        if (e_c == 3'd5) begin q = e_q + 4'd1; c = 3'd0; end"),
 'FO3_out_vertex':  ('r3d_geo.sv', "if (e_c == 3'd5) begin q = e_q + 4'd1; c = 3'd0; end", "if (e_c == 3'd5) begin q = e_q; c = 3'd0; end"),
 'FS1_side_early':  ('r3d_geo.sv', "assign sa_data[gl] = (e_cls == 2'd1) ? (sa_hi ? tn_q : tp_q) : (sa_hi ? qb_q : qa_q);",
                                   "assign sa_data[gl] = (e_cls == 2'd1) ? (sa_addr[2] ? tn_q : tp_q) : (sa_addr[2] ? qb_q : qa_q);"),
 # the LOD word (LOD_WORD, study D-33): the L faults run the LOD build (L4 MAME's correctly rounded frustum)
 'L1_copy_sign':    ('r3d_geo.sv', "default: e_hw = e_inv ? negf(fb_q) : fb_q;", "default: e_hw = fb_q;"),
 'L2_np_scale':     ('r3d_face.sv', "(LOD && ak_q == A_NP) ? 8'd115", "(LOD && ak_q == A_NP) ? 8'd116"),
 'L3_c_sign':       ('r3d_geo.sv', "f_cd <= negf(vq_r);", "f_cd <= vq_r;"),
 'L4_kz2':          ('r3d_face.sv', "A_KZ: opa <= CMUL ? KZ2 : ar_q;", "A_KZ: opa <= CMUL ? F_ONE : ar_q;"),
 'L5_np_signed':    ('r3d_face.sv', "sg = !(LOD && op_ak == A_NP) && a_q[23];", "sg = a_q[23];"),
 'L6_q_par':        ('r3d_face.sv', "wq ? {wb[11].par, 2'd3, 2'd3}", "wq ? {!wb[11].par, 2'd3, 2'd3}"),
 'L7_c0_align':     ('r3d_face.sv', "op_c0 = (r == 2'd0) || (r == 2'd1 && s == 3'd0) || (LOD && r == 2'd3 && (s == 3'd4 || s == 3'd5));",
                                    "op_c0 = (r == 2'd0) || (LOD && r == 2'd3 && (s == 3'd3 || s == 3'd4));"),
 'L8_b_nowait':     ('r3d_geo.sv', "if (sc || so) begin if (!LOD_WORD || fb_rdy) est <= E_OUT; end", "if (sc || so) begin est <= E_OUT; end"),
 'L9_cota_word':    ('r3d_geo.sv', "      M_VP8: o = 6'd3;", "      M_VP8: o = 6'd2;"),
 'L10_np_word':     ('r3d_geo.sv', "(LOD_WORD && hk == 3'd5)", "(LOD_WORD && hk == 3'd4)"),
 # read timing at the ports (m2-notes section 15): the model stream started while a
 # burst of the model before was asked for and not yet acknowledged
 'T1_stream_late_ack': ('r3d_mstream.sv', "assign ready_start = !active && !draining && outst == 3'd0 && !pram_req && !vrom_req;",
                                          "assign ready_start = !active && !draining && outst == 3'd0;"),
 # the frame ended (busy low) with the stream's last prefetch still open at its port
 'T2_end_open':        ('r3d_geo.sv', "!out_valid && !w_busy && f_idle && ms_ready) begin", "!out_valid && !w_busy && f_idle) begin"),
 # the set-up on the pool (SETUP_OUT): the port, the kernel's group, the emitter's records
 'SU1_sort_c3':     ('r3d_geo.sv', "c3 = s_c1 ? s_ccb : s_cca;                    // c < lo", "c3 = s_c1 ? s_cca : s_ccb;                    // c < lo"),
 'SU2_sort_c1':     ('r3d_geo.sv', "lo = s_c1 ? s_qb : s_qa; hi = s_c1 ? s_qa : s_qb;", "lo = s_qa; hi = s_qb;"),
 'SU3_copy_shade':  ('r3d_geo.sv', "default: s_comp = (s_inv && !s_fix) ? 3'd7 : 3'd6;", "default: s_comp = 3'd6;"),
 'SU4_flat_ps':     ('r3d_geo.sv', "else nxt = {3'(e_sp) + 3'd2, (e_flat && e_sc == 2'd1) ? 2'd3 : e_sc + 2'd1};", "else nxt = {3'(e_sp) + 3'd2, e_sc + 2'd1};"),
 'SU5_flat_never':  ('r3d_geo.sv', "e_flat <= so_lq[30:0] < F_EPS;", "e_flat <= 1'b0;"),
 'SU6_nonfin_drop': ('r3d_geo.sv', "if (s_dchk && po_d[30:23] == 8'hff) s_nonfin <= 1'b1;", "if (1'b0) s_nonfin <= 1'b1;"),
 'SU7_queue_guard': ('r3d_geo.sv', "assign s_blk = (pst == P_Q1) || (pst == P_Q0 && !po_busy[pbuf]);", "assign s_blk = (pst == P_Q1);"),
 'SU8_stage_half':  ('r3d_geo.sv', "pq_wa <= PQA'({1'b1, sg, s_dk[4:3], s_dk[1:0]});", "pq_wa <= PQA'({1'b1, 1'b0, s_dk[4:3], s_dk[1:0]});"),
 'SU9_out_wait':    ('r3d_geo.sv', "end else if (tq_cnt != 5'd0 && so_full[ebs]) begin", "end else if (tq_cnt != 5'd0) begin"),
 'SU10_kernel_kind':('r3d_geo.sv', "vk_entry <= g_tex[skb] ? K_SETUP4 : K_SETUP2;", "vk_entry <= K_SETUP2;"),
 'SU11_half_free':  ('r3d_geo.sv', "K_IDLE: if (stg_busy[skb] && !so_full[skb]) begin", "K_IDLE: if (stg_busy[skb]) begin"),
 'SU12_word_lane':  ('r3d_geo.sv', "s_dv <= 1'b1; s_dk <= s_k; s_dql <= s_qg[1:0];", "s_dv <= 1'b1; s_dk <= s_k; s_dql <= s_qy[1:0];"),
 'SU13_group_size': ('r3d_geo.sv', "stg_busy[sg] <= 1'b1; g_n[sg] <= {1'b0, sl}; g_tex[sg] <= s_gtex;", "stg_busy[sg] <= 1'b1; g_n[sg] <= 3'd4; g_tex[sg] <= s_gtex;"),
 'SU14_two_records':('r3d_geo.sv', "s_inv <= 1'b0; s_two <= 1'b0; s_cf <= s_pf; s_ti <= 5'd2; s_st <= S_T0;", "s_inv <= 1'b0; s_two <= 1'b0; s_cf <= s_pf; s_ti <= 5'd3; s_st <= S_T0;"),
 'SU15_stage_wait': ('r3d_geo.sv', "S_T0: if (!stg_busy[sg] && tq_cnt <= 5'd14) begin", "S_T0: if (tq_cnt <= 5'd14) begin"),
 'FS2_side_late':   ('r3d_geo.sv', "  always_ff @(posedge clk) sa_hi <= sa_addr[2];",
                                   "  logic sa_hi0;\n  always_ff @(posedge clk) begin sa_hi0 <= sa_addr[2]; sa_hi <= sa_hi0; end"),
}
def run(name):
    f, old, new = FAULTS[name]
    d = os.path.join(ROOT, os.environ.get('R3D_FAULTDIR', 'sm3-builds/r3d/faults2'), name)
    shutil.rmtree(d, ignore_errors=True)
    shutil.copytree(os.path.join(ROOT, 'rtl/real3d'), os.path.join(d, 'rtl'))
    p = os.path.join(d, 'rtl', f)
    s = open(p).read()
    assert s.count(old) == 1, (name, s.count(old))
    open(p, 'w').write(s.replace(old, new))
    # the default build (Supermodel frustum, culling); the CORDIC only exists in MAME's
    lod = name.startswith('L')
    mame = name.startswith(('F13', 'L4'))
    nocull = name.startswith('D12')
    gen = '-GFRUSTUM_SM=0 -GSETUP_CR=1 -GCULL=0' if mame else '-GFRUSTUM_SM=1 -GCULL=0' if nocull else '-GFRUSTUM_SM=1 -GCULL=1'
    keep = name.startswith(('FA3b', 'D2'))   # D2: one record per polygon unless d = 0, both copies only here
    if keep: gen += ' -GFACE_DROP=0'
    if lod: gen += ' -GLOD_WORD=1'
    su = name.startswith('SU')
    if su: gen += ' -GSETUP_OUT=1'
    V = ' '.join(os.path.join(d, 'rtl', x) for x in ['r3d_ucode_pkg.sv','r3d_ram.sv','r3d_fp_mul.sv','r3d_fp_add.sv','r3d_recip.sv','r3d_fsqrt.sv','r3d_ucode.sv','r3d_ucode_su.sv','r3d_vpu.sv','r3d_walker.sv','r3d_mstream.sv','r3d_setup_cr.sv','r3d_face.sv','r3d_geo.sv'])
    # the SU faults' bench checks the set-up records against rtl_tri::setup (r3d_rast_rtl.cpp)
    cx = (' -ffp-contract=off -DGEO_SETUP_OUT', ' %s/sim/real3d/r3d_rast.cpp %s/sim/real3d/r3d_rast_rtl.cpp' % (ROOT, ROOT)) if su else ('', '')
    cmd = ('verilator --cc --exe --build -j 4 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-WIDTHTRUNC --top-module r3d_geo %s '
           '-CFLAGS "-O2 -std=c++17 -I%s/sim/real3d%s" -LDFLAGS "-lz" --Mdir %s/obj %s %s/sim/real3d/tb_r3d_geo.cpp %s/sim/real3d/r3d_geo.cpp%s > %s/build.log 2>&1'
           % (gen, ROOT, cx[0], d, V, ROOT, ROOT, cx[1], d))
    subprocess.run(cmd, shell=True, check=True)
    res = []
    setup = 'cr' if mame else 'sm' if nocull else 'sm --cull'
    if keep: setup += ' --face-keep'
    if lod: setup += ' --lod'
    if su: setup += ' --setup-out'
    # F7, D4: indexed colour appears in the early VF3 frames; N2 needs back-pressure
    # on the output; D9 (a flat polygon reusing a stored vertex) and D5 (fixed
    # shading, 0.6 polygons a frame) need every frame
    fr = '--frames 20' if name.startswith(('F7', 'D4')) else '--frames 400' if name.startswith(('D9', 'D5')) else '--frames 20 --first 190'
    if name.startswith(('N2', 'D11')): fr += ' --stall 90'
    if name.startswith('T'): fr += ' --hostile 7 --ack 50 400 --lat 2 20 --maxrd 1 --stall 0'
    # SU11 overwrites results the emitter has not sent: it shows when the output stalls
    if name.startswith('SU11'): fr += ' --stall 90'
    games = [('vf3', fr), ('bassdx', '--frames 400' if name.startswith(('F5', 'D9', 'D5')) else fr)]
    if name.startswith(('FA11', 'FA12')): games.append(('daytona2', '--frames 20 --first 190 --step 21'))
    for g, extra in games:
        vrom = 'daytona2' if g.startswith('daytona2') else g
        # a fault that hangs the RTL stops at 20 M cycles a frame (a frame takes under 2.5 M)
        out = subprocess.run('R3D_MAXCYC=20000000 %s/obj/Vr3d_geo --dumps %s/sm3-builds/census/dumps_%s --vrom %s/sm3-builds/r3d/vrom_%s.bin --setup %s %s' % (d, ROOT, os.environ.get('R3D_D2DUMPS', g) if g == 'daytona2' else g, ROOT, vrom, setup, extra),
                             shell=True, capture_output=True, text=True, timeout=7200).stdout
        open(os.path.join(d, 'run_%s.txt' % g), 'w').write(out)
        line = [l for l in out.split('\n') if l.startswith('r3d_geo:')]
        res.append('%s: %s' % (g, line[0].split(':', 1)[1].strip() if line else 'no result (hang or crash)'))
    shutil.rmtree(os.path.join(d, 'obj'), ignore_errors=True)
    print(name, '|', ' | '.join(res), flush=True)
if __name__ == '__main__':
    for n in sys.argv[1:]: run(n)
