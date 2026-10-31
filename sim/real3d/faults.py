#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Planted faults for the Real3D geometry (make test_real3d_faults). Each one
# goes into a copy of rtl/real3d under sm3-builds/r3d/faults2/, builds the bench, runs
# 20 VF3 and 20 Bass Fishing frames (F5: all 400 Bass frames) of the default build
# (Supermodel frustum and culling, every D-19 row; F13 MAME's correctly rounded
# frustum; D12 the build without culling). Prints the result. The D faults are
# the D-19 rows' (m2-notes section 11).
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
 'D1_normal_conv':  ('r3d_geo.sv', "else tc_k <= k255a(kb[7], kb[7] ? 8'(9'd0 - ko) : ko[7:0]);", "else tc_k <= k255a(1'b0, kb);"),
 'D2_double_second':('r3d_geo.sv', "if (e_inv) begin e_inv <= 1'b0; est <= E_P2; end", "if (1'b0) begin e_inv <= 1'b0; est <= E_P2; end"),
 'D3_sun_clamp':    ('r3d_vpu.sv', "if (wq[11].post == 2'd1 && neg) res = 32'h0000_0000;", "if (1'b0) res = 32'h0000_0000;"),
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
 'G6_amb_byte':     ('r3d_geo.sv', "kb = amb_ld ? vq_r[15:8] : tq_w[7:0];", "kb = amb_ld ? vq_r[7:0] : tq_w[7:0];"),
 'G7_div_rem':      ('r3d_recip.sv', "up10 <= !dlo[29];", "up10 <= !d[29];"),
 'G8_region_end':   ('r3d_walker.sv', "({1'b0, rd_addr[2:0]} + {1'b0, rd_beat} >= 4'd8)", "({1'b0, rd_addr[2:0]} + {1'b0, rd_beat} >= 4'd7)"),
 'G9_record_bits':  ('r3d_geo.sv', "assign re_q = {re_sq[88:23], re_sq[43], re_sq[46], re_sq[22:0]};", "assign re_q = {re_sq[88:23], re_sq[46], re_sq[43], re_sq[22:0]};"),
 'G10_matrix_bank': ('r3d_geo.sv', "xb_we <= 1'b1; xb_bank <= cnt[1:0]; xb_addr <= 5'(R_BC_MB) + 5'(cnt[3:2]);", "xb_we <= 1'b1; xb_bank <= cnt[3:2]; xb_addr <= 5'(R_BC_MB) + 5'(cnt[1:0]);"),
 'G11_addr_adder':  ('r3d_geo.sv', "      M_VP3: o = 6'd26;", "      M_VP3: o = 6'd25;"),
 'G5b_outcode_sign':('r3d_geo.sv', "opair[0] = ok && (bz || (w[31] ? (!a[31] && !lt) : (!a[31] || lt || eq)));", "opair[0] = ok && (bz || (w[31] ? (!a[31] && lt) : (!a[31] || lt || eq)));"),
 'G11b_addr_adder': ('r3d_geo.sv', "      M_VP1: o = 6'd12;", "      M_VP1: o = 6'd13;"),
 'G12_result_mux':  ('r3d_vpu.sv', "assign rr = mac_l ? res : d_res;", "assign rr = mac_l ? sum : d_res;"),
 'N7_div_lane':     ('r3d_vpu.sv', "for (int k = 1; k < 4; k++) begin dq_x[k] <= opA[k]; dq_a[k] <= opB[k]; end",
                                   "for (int k = 1; k < 4; k++) begin dq_x[k] <= opA[k]; dq_a[k] <= opB[4 - k]; end"),
}
def run(name):
    f, old, new = FAULTS[name]
    d = os.path.join(ROOT, 'sm3-builds/r3d/faults2', name)
    shutil.rmtree(d, ignore_errors=True)
    shutil.copytree(os.path.join(ROOT, 'rtl/real3d'), os.path.join(d, 'rtl'))
    p = os.path.join(d, 'rtl', f)
    s = open(p).read()
    assert s.count(old) == 1, (name, s.count(old))
    open(p, 'w').write(s.replace(old, new))
    # the default build (Supermodel frustum, culling); the CORDIC only exists in MAME's
    mame = name.startswith('F13')
    nocull = name.startswith('D12')
    gen = '-GFRUSTUM_SM=0 -GSETUP_CR=1 -GCULL=0' if mame else '-GFRUSTUM_SM=1 -GCULL=0' if nocull else '-GFRUSTUM_SM=1 -GCULL=1'
    V = ' '.join(os.path.join(d, 'rtl', x) for x in ['r3d_ucode_pkg.sv','r3d_ram.sv','r3d_fp_mul.sv','r3d_fp_add.sv','r3d_recip.sv','r3d_fsqrt.sv','r3d_ucode.sv','r3d_vpu.sv','r3d_walker.sv','r3d_mstream.sv','r3d_setup_cr.sv','r3d_geo.sv'])
    cmd = ('verilator --cc --exe --build -j 4 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-WIDTHTRUNC --top-module r3d_geo %s '
           '-CFLAGS "-O2 -std=c++17 -I%s/sim/real3d" -LDFLAGS "-lz" --Mdir %s/obj %s %s/sim/real3d/tb_r3d_geo.cpp %s/sim/real3d/r3d_geo.cpp > %s/build.log 2>&1'
           % (gen, ROOT, d, V, ROOT, ROOT, d))
    subprocess.run(cmd, shell=True, check=True)
    res = []
    setup = 'cr' if mame else 'sm' if nocull else 'sm --cull'
    # F7, D4: indexed colour appears in the early VF3 frames; N2 needs back-pressure
    # on the output; D9 (a flat polygon reusing a stored vertex) and D5 (fixed
    # shading, 0.6 polygons a frame) need every frame
    fr = '--frames 20' if name.startswith(('F7', 'D4')) else '--frames 400' if name.startswith(('D9', 'D5')) else '--frames 20 --first 190'
    if name.startswith('N2'): fr += ' --stall 90'
    for g, extra in (('vf3', fr), ('bassdx', '--frames 400' if name.startswith(('F5', 'D9', 'D5')) else fr)):
        out = subprocess.run('%s/obj/Vr3d_geo --dumps %s/sm3-builds/census/dumps_%s --vrom %s/sm3-builds/r3d/vrom_%s.bin --setup %s %s' % (d, ROOT, g, ROOT, g, setup, extra),
                             shell=True, capture_output=True, text=True, timeout=7200).stdout
        open(os.path.join(d, 'run_%s.txt' % g), 'w').write(out)
        line = [l for l in out.split('\n') if l.startswith('r3d_geo:')]
        res.append('%s: %s' % (g, line[0].split(':', 1)[1].strip() if line else 'no result (hang or crash)'))
    shutil.rmtree(os.path.join(d, 'obj'), ignore_errors=True)
    print(name, '|', ' | '.join(res), flush=True)
if __name__ == '__main__':
    for n in sys.argv[1:]: run(n)
