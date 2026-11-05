#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the integer loads with update (corea, suba),
# the pipelined data port, the pipelined D-cache, the
# core's cache interface and the fast FPU port. Each fault is made in a copy
# of rtl/ under
# sm3-builds/ldp_faults/<name>, the bench that should see it is built from
# the copy and run, and the fault is reported caught (the bench failed) or
# missed. The tree is never edited. A fault's old / new may be lists of
# replacements in one file; file 'cflags' instead drops old from the bench's
# compiler flags.
#
#   python3 sim/cpu/faults.py [name ...]

import os, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORK = os.path.join(ROOT, 'sm3-builds', 'ldp_faults')

CPU_RTL = ('ppc_pkg ppc_fpu_pkg ppc_decode ppc_alu ppc_muldiv ppc_regfile ppc_fpregfile ppc_fetchq '
           'ppc_fpu_lzc ppc_fpu_fma ppc_fpu_div ppc_fpu_misc ppc_fpu_ldst ppc_fpu_fpscr ppc_fpu ppc_fpu_sfma '
           'ppc_fpu_rsqe ppc_fpu_fast ppc_pdec ppc_pdw ppc_btb ppc_core '
           'ppc_cache_pkg ppc_cache_ram ppc_icache ppc_dcache ppc_cpu').split()

# the core's simulation checks of the fast port's wiring
NOCHK_ROUTE = ('      if (f_iss && ex_ffast != fpu_fast_route(fma_op_t\'(ex_u.fop), ex_u.fsgl, fa_q, fb_q, fc_q))\n'
               '        $fatal(1, "fast route %0d is not for EX\'s operands", ex_ffast);\n')
RT_ID       = "  wire         id_frt = q_pd.frq || (q_pd.frs && sv_a && (!q_pd.fub || sv_b) && (!q_pd.fuc || sv_c));"
NOCHK_FFW   = '      if (ex_ffw != ex_ffast || (ex_ffast && !ex_fma)) $fatal(1, "ex_ffw");\n'
NOCHK_RTE   = ("      if (ex_valid && ex_fma && !ex_ifault && ex_ffast != fpu_fast_route(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_q, fb_q, fc_q))\n"
               '        $fatal(1, "fast route %0d for EX\'s operands", ex_ffast);\n')
NOCHK_SV    = ('      if (rf_na != !fpu_sgl_val(rf_fa) || rf_nb != !fpu_sgl_val(rf_fb) || rf_nc != !fpu_sgl_val(rf_fc))\n'
               '        $fatal(1, "FPR single-value bit against the value read");\n')
NOCHK_FWQ   = '      $fatal(1, "fwq against EX\'s FP wait");\n'
NOCHK_DBAT  = '        $fatal(1, "DBAT %0d compare against bat_match", k);\n'
NOCHK_FPD   = '        $fatal(1, "FP predecode of %08X against the decoder", q_hi);\n'
NOCHK_ERC   = ('          $fatal(1, "early register compare, stage %0d: %b against %b", p, qe,\n'
               '                 {rd == id_rs, rd == q_hi[25:21], rd == q_hi[15:11], rd == q_hi[20:16]});\n')
NOCHK_EFS   = '      $fatal(1, "early forward selects: MEM %b / %b, WB %b / %b", qf_m, qe_m, qf_w, qe_w);\n'
NOCHK_PD    = '      $fatal(1, "predecode raw compares against the decoder, word %h", w);\n'
NOCHK_OK    = '      if (f_iss && ex_ffast && !f_fast_ok) $fatal(1, "fast op issued against the unit\'s own check");\n'
NOCHK_FFBL  = '      if (ffbl_q != ((fr_at >> LS) != \'0)) $fatal(1, "ffbl_q");\n'

# (name, file, old, new, benches)
FAULTS = [
  # misaligned accesses (603e 2.2.3, 4.5.6): the split, its merge and the
  # alignment exception (benches corem, subm: --mis)
  ('mis_s1_be', 'ppc_core.sv',
   "        st_be   = ex_sp1 ? 4'(8'(smask) << (3'd4 - {1'b0, boff})) : smask >> boff;",
   "        st_be   = smask >> boff;", ['corem']),
  ('mis_rot', 'ppc_core.sv',
   "(boff == 2'd1) ? {vl[7:0], vl[31:8]} :", "(boff == 2'd1) ? {vl[15:0], vl[31:16]} :", ['corem']),
  ('mis_ea4', 'ppc_core.sv',
   "op_x + op_y + {29'd0, ex_sp1, 2'b00};", "op_x + op_y;", ['corem']),
  ('mis_merge', 'ppc_core.sv',
   "    d = {w0, w1} << {bo, 3'b000};", "    d = {w1, w0} << {bo, 3'b000};", ['corem']),
  ('mis_w0_fwd', 'ppc_core.sv',
   "  wire  [31:0] mis_w0_m = (wb_adv && wb_sp0) ? ldw : mis_w0;", "  wire  [31:0] mis_w0_m = mis_w0;", ['corem', 'subm']),
  ('mis_wb_hold', 'ppc_core.sv',
   "  assign wb_hold  = wb_valid && ((wb_out && (!d_valid || wb_sp1))", "  assign wb_hold  = wb_valid && ((wb_out && !d_valid)", ['corem']),
  ('mis_irq', 'ppc_core.sv',
   "  wire  xc_irq   = irq_req && ex_first && !ex_s0ok;", "  wire  xc_irq   = irq_req && ex_first;", ['corem']),
  ('mis_s0_rdwe', 'ppc_core.sv',
   "      mx.rd_we = 1'b0;                                   // (an update store writes rA from the second)", "", ['corem']),
  ('mis_qfmx', 'ppc_core.sv',
   "!is_ld && !mis_hold && !algn;", "!is_ld && !algn;", ['corem']),
  ('mis_ldw', 'ppc_core.sv',                           # (load_use by d_valid, study R104: WB's load
   # forwards on its answer alone, wb_sp1 ignored)
   "(* keep = 1 *) wire load_use_d1 = !id_xuea && (lu_rest || (lu_wb && wb_sp1));",
   "(* keep = 1 *) wire load_use_d1 = !id_xuea && lu_rest;", ['corem']),
  ('mis_half3', 'ppc_core.sv',
   "ex_msh && boff == 2'd3;", "1'b0;", ['corem']),
  ('mis_half1', 'ppc_core.sv',
   "(bo == 2'd1) ? w[23:8] : w[15:0];", "(bo == 2'd1) ? w[31:16] : w[15:0];", ['corem']),
  ('algn_page', 'ppc_core.sv',
   "(msr_q[MSR_DR] && ((mis_x && ea[11:2] == 10'h3FF) ||", "(msr_q[MSR_DR] && ((1'b0 && ea[11:2] == 10'h3FF) ||", ['corem']),
  ('algn_fpdbl', 'ppc_core.sv',
   "(ex_u.fp && ex_fd0 && ea[11:0] == 12'hFFC)", "1'b0", ['corem']),
  ('algn_word', 'ppc_core.sv',
   "(ex_u.fp || ex_u.reserve || ex_u.cond || (ex_lsm && ex_first)) && boff != 2'd0)",
   "(ex_u.fp || ex_u.cond || (ex_lsm && ex_first)) && boff != 2'd0)", ['corem']),
  ('algn_dar4', 'ppc_core.sv',
   "m.result + {29'd0, m.algn && m.algd4, 2'b00};", "m.result;", ['corem']),
  ('algn_dsisr_d', 'ppc_core.sv',
   "        dsisr_q <= (m.insn[31:26] == 6'd31)", "        dsisr_q <= 1'b1", ['corem']),
  ('d39_leave', 'ppc_core.sv',
   "    db_leave = db_leave && &ea_x[16:12];", "    db_leave = 1'b0;", ['corem']),
  ('d39_stay', 'ppc_core.sv',
   "  wire         db_stay = db_any && !db_leave;", "  wire         db_stay = 1'b0;", ['corem']),
  ('algn_fpw', 'ppc_core.sv',
   "        wb_fpw    <= mem_adv && m.fpw && !m.rp && !m.algn;", "        wb_fpw    <= mem_adv && m.fpw && !m.rp;", ['corem']),
  ('load_use_mem', 'ppc_core.sv',
   "(m_ldw && lu_q(qe_m,", "(1'b0 && lu_q(qe_m,", ['core']),
  ('wb_fwd_early', 'ppc_core.sv',
   "wire lu_wb      = wb_valid && wb_live_q && wb_rd_we && lu_q(qe_w,", "wire lu_wb      = 1'b0 && lu_q(qe_w,", ['core']),
  ('answer_order', 'ppc_core.sv',
   "wire ans_wb = d_valid && wb_out;\n  wire ans_m  = d_valid && !wb_out && m_out;",
   "wire ans_wb = d_valid && wb_out && !m_out;\n  wire ans_m  = d_valid && m_out;", ['core']),
  ('dc_post_busy', 'ppc_dcache.sv',
   "base_ok && !x_v && (!r_v || r_to_sb);", "base_ok && !x_v;", ['dcache']),
  ('dc_rdy_parked', 'ppc_dcache.sv',
   "d_rdy_q <= !r_v_n || (r_fresh_n && !x_v_n);", "d_rdy_q <= !r_v_n || r_fresh_n;", ['dcache']),
  ('dc_wimg_i', 'ppc_dcache.sv',
   "wire         r_byp   = r_wimg[2] || !r_dce || r_rsv;", "wire         r_byp   = !r_dce || r_rsv;", ['dcache']),
  ('dc_sb_dcbz', 'ppc_dcache.sv',
   "wire wp_free = !f_act && !z_act && !(dq_v && dq_k == K_DCBZ);", "wire wp_free = !f_act && !z_act;", ['dcache']),
  ('dcbz_align', 'ppc_core.sv',
   "(bat_wimg[4 + k][3] || bat_wimg[4 + k][2]));\n      du_wi = du_wi | (du_use[k] && (du_wimg[k][3] || du_wimg[k][2]));",
   "bat_wimg[4 + k][3]);\n      du_wi = du_wi | (du_use[k] && du_wimg[k][3]);", ['core']),
  ('dcbi_priv', 'ppc_decode.sv',
   "if (xo10 == 10'd470 && pr) begin", "if (1'b0) begin", ['core']),
  ('icbi_set', 'ppc_core.sv',
   "assign icbi_set = m.result[11:5];", "assign icbi_set = m.result[10:4];", ['sub']),
  ('icbi_noinv', 'ppc_icache.sv',
   "tw_en_q <= 2'b11; tw_set_q <= icbi_set;", "tw_en_q <= 2'b01; tw_set_q <= icbi_set;", ['icache']),
  ('hid0_dce_ice', 'ppc_core.sv',
   "assign hid_ice    = hid0_q[15];\n  assign hid_dce    = hid0_q[14];",
   "assign hid_ice    = hid0_q[14];\n  assign hid_dce    = hid0_q[15];", ['sub']),
  ('sync_nodrain', 'ppc_core.sv',
   "wire sync_wait = ex_valid && ex_u.sync && (wb_mayh || (m.valid && m_iss) || !d_idle);",
   "wire sync_wait = 1'b0;", ['core', 'sub']),
  ('i_seq_always', 'ppc_core.sv',
   "assign i_seq  = !redir_q && !pt_q && f_sq && !f_xm;", "assign i_seq  = !redir_q;", ['sub']),
  ('replay_kill_wb', 'ppc_core.sv',
   "wire         m_kill   = m_kp && !wb_hold;", "wire         m_kill   = m_kp;", ['core']),
  ('replay_iss', 'ppc_core.sv',
   "assign m_iss = m.req && !m.rp;", "assign m_iss = m.req;", ['core']),
  ('dsi_second_ld', 'ppc_core.sv',
   "&& !xl_rp && !ex_xcm && !m_kp;", "&& !ex_xcm && !m_kp;", ['core']),
  ('ser_drain', 'ppc_core.sv',
   "assign ser_wait = ex_valid && ex_serz &&", "assign ser_wait = 1'b0 && ex_serz &&", ['core']),
  ('touch_fault', 'ppc_core.sv',
   "wire        sl_flt  = m.rp && sl_done && (sl_dsi_q || sl_tm_q) && !m.tch;",
   "wire        sl_flt  = m.rp && sl_done && (sl_dsi_q || sl_tm_q);", ['core']),
  # the timing pass's restructured logic (early compares, holds, kill target, CR kinds, decode)
  ('qe_e_hold', 'ppc_core.sv',
   "qe_e <= ex_hold  ? qe_hk[1] : qe_hf[0];", "qe_e <= qe_hf[0];", ['core']),
  ('qf_w_write', 'ppc_core.sv',
   "(ex_hold ? qe_hk[2] : qe_hf[2]) & {4{qf_wm}};", "(ex_hold ? qe_hk[2] : qe_hf[2]);", ['core']),
  ('hn_pop_head', 'ppc_core.sv',
   "!h_tk ? qe_hk[r][f] : q_cnt >= 3'd2 ?", "!h_tk ? qe_hk[r][f] : q_cnt >= 3'd3 ?", ['core']),
  # this pass: the early compares' two cases (_nochk drops the compare checks)
  ('qe_w_wbhold_nochk', 'ppc_core.sv',
   ["qe_w <= ex_hold ? (wb_hold ? qe_hk[3] : qe_hk[2]) : (wb_hold ? qe_hf[3] : qe_hf[2]);", NOCHK_ERC],
   ["qe_w <= wb_hold ? qe_hk[3] : ex_hold ? qe_hk[2] : qe_hf[2];", "          ;\n"], ['core']),
  ('qe_hf_rs_nochk', 'ppc_core.sv',
   ["  wire  [4:0] id_rs1 = id_rs + 5'd1;", NOCHK_ERC],
   ["  wire  [4:0] id_rs1 = id_rs_q;", "          ;\n"], ['core']),
  ('qf_m_hold_nochk', 'ppc_core.sv',
   ["(ex_hold ? qe_hk[1] : qe_hf[1]) & {4{qf_mx}};", NOCHK_EFS, NOCHK_ERC],
   ["qe_hf[1] & {4{qf_mx}};", "      ;\n", "          ;\n"], ['core']),
  ('kill_target', 'ppc_core.sv',
   "m_kp    ? (m.resync ? m.pc + 32'd4", "1'b0    ? (m.resync ? m.pc + 32'd4", ['core']),
  ('crf4_bit', 'ppc_core.sv',
   "ex_cra_s[3] ? m.crf4[2'd3 - ex_u.crba[1:0]]", "ex_cra_s[3] ? m.crf4[ex_u.crba[1:0]]", ['core']),
  ('imm4_count', 'ppc_core.sv',
   "imm4_q <= (id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_q) + 32'd4;",
   "imm4_q <= (id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_q) + 32'd8;", ['core']),
  ('fp_ok_fres', 'ppc_core.sv',
   "5'd24:   fp_ok = !dbl;", "5'd24:   fp_ok = 1'b1;", ['core']),
  ('mh_fsm', 'ppc_core.sv',
   "wire mh_3 = m.valid && ((m.rp && !sl_done) || (m.fsm && (fr_at[1] || wb_blk)));",
   "wire mh_3 = m.valid && ((m.rp && !sl_done) || (m.fsm && wb_blk));", ['core']),
  ('algc_sel', 'ppc_core.sv',
   "(m.dbany ? m.dbwi : m.duwi);", "(m.dbany ? m.duwi : m.dbwi);", ['core']),
  ('froh_rd', 'ppc_core.sv',
   "ex_froh    <= 32'd1 << q_hi[25:21];", "ex_froh    <= 32'd1 << q_hi[20:16];", ['core']),
  ('opf_fdx_step', 'ppc_core.sv',
   "id_s0 || q_pd.crack == 2'd2 || (q_pd.fdx && id_s1)) id_opf", "id_s0 || q_pd.crack == 2'd2) id_opf", ['core']),
  ('raw_rfi', 'ppc_core.sv',
   "wire       h_rfi   = h_op == 6'd19 && h_x == 10'd50;", "wire       h_rfi   = h_op == 6'd19 && h_x == 10'd51;", ['core']),
  ('dc_r_always', 'ppc_dcache.sv',
   "    if (d_rdy_q) begin\n      r_k", "    if (1'b1) begin\n      r_k", ['dcache', 'sub']),
  ('dc_rsv_page', 'ppc_dcache.sv',
   "wire         r_rsv   = r_a[31:12] == RSV_TAG;", "wire         r_rsv   = 1'b0;", ['dcache']),
  # the fast FPU port (D-21, D-22); the _nochk forms also drop the core's
  # simulation checks of the new wiring, so only the model comparison and
  # the schedule checks that were there before can see them
  ('ff_route_stale', 'ppc_core.sv',
   "ex_ffast <= id_frt;", "ex_ffast <= fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_q, fb_q, fc_q);", ['core']),
  ('ff_route_stale_nochk', 'ppc_core.sv',
   ["      ex_ffast <= id_frt;\n      ex_ffw   <= id_frt;", NOCHK_ROUTE, NOCHK_FFW, NOCHK_RTE],
   ["      ex_ffast <= q_pd.frq || (q_pd.frs && fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_q, fb_q, fc_q));\n"
    "      ex_ffw   <= q_pd.frq || (q_pd.frs && fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_q, fb_q, fc_q));", "", "", ""], ['core']),
  ('ff_route_nofwd_nochk', 'ppc_core.sv',
   ["  wire         sv_a   = f_hit_id[0] ? sv_fw : !rf_na;", NOCHK_ROUTE, NOCHK_RTE],
   ["  wire         sv_a   = !rf_na;", "", ""], ['core']),
  # this pass: the route from predecode and the FPRs' single-value bits
  ('rt_fwd_src_nochk', 'ppc_core.sv',
   ["  wire         sv_fw  = fo_valid ? sv_fo : sv_wbf;", NOCHK_ROUTE, NOCHK_RTE],
   ["  wire         sv_fw  = fo_valid ? sv_wbf : sv_fo;", "", ""], ['core']),
  ('rt_tag_write_nochk', 'ppc_core.sv',
   ["  wire         fw_nsv = !(fo_valid ? sv_fo : !wb_isld ? sv_wbf :", NOCHK_ROUTE, NOCHK_RTE, NOCHK_SV],
   ["  wire         fw_nsv = !(fo_valid ? sv_fo : !wb_isld ? 1'b1 :", "", "", ""], ['core']),
  ('rt_kind_fub_nochk', 'ppc_core.sv',
   [RT_ID, NOCHK_ROUTE, NOCHK_RTE],
   [RT_ID.replace("(!q_pd.fub || sv_b)", "sv_b"), "", ""], ['core']),
  ('pd_fctiwz_nochk', ['ppc_pdw.sv', 'ppc_core.sv'],
   ["pd_x == 10'd14 ||\n                                                     pd_x == 10'd15)));", NOCHK_FPD],
   ["pd_x == 10'd14 ||\n                                                     1'b0)));", "        ;\n"], ['core']),
  # this pass: EX's registered FP wait (fwq)
  ('fwq_keep_c2_nochk', 'ppc_core.sv',
   ["(ex_f2c && !ex_valid) ||", NOCHK_FWQ],
   ["ex_f2c ||", "      ;\n"], ['core']),
  ('fwq_new_fsx_nochk', 'ppc_core.sv',
   ["(((h_fser && !q_pd.ifault) || h_rfi || h_mtmsr) && !fw_idl) || q_pd.ffsx || q_pd.ff2c ||", NOCHK_FWQ],
   ["(((h_fser && !q_pd.ifault) || h_rfi || h_mtmsr) && !fw_idl) || q_pd.ff2c ||", "      ;\n"], ['core']),
  ('fwq_new_div', 'ppc_core.sv',
   "(q_pd.fdiv && (fw_ba || (fw_ra & FPRE) != '0))", "(q_pd.fdiv && fw_ba)", ['core']),
  # this pass: the carry-free DBAT compare, the fetch micro-TLB's hit
  ('dbat_run_nochk', 'ppc_core.sv',
   ["      rs[j] = um[j] && (j == 0 || !um[(j + 14) % 15]);", NOCHK_DBAT],
   ["      rs[j] = um[j] && j == 0;", "        ;\n"], ['core']),
  ('dbat_carry_nochk', 'ppc_core.sv',
   ["((x[17 + j] ^ y[17 + j]) & ~bepi[j]);", NOCHK_DBAT],
   ["((x[17 + j] ^ y[17 + j]) & bepi[j]);", "        ;\n"], ['core']),
  ('iu_hit_sel', 'ppc_core.sv',
   "wire  [1:0]  iu_hit = fc_iu[f_sel];", "wire  [1:0]  iu_hit = fc_iu[0];", ['core']),
  # this pass: the fetch queue's ring, the count's one-hot copy, the redirect copy
  ('fq_ring_slot', 'ppc_fetchq.sv',
   "    if (o[3]) rg[!rp] <= w_e;", "    if (o[3]) rg[rp] <= w_e;", ['core']),
  ('fq_ring_ptr', 'ppc_fetchq.sv',
   "if (pop[0] && ((!o[0] && !o[1] && !o[2]) || (o[2] && p2))) rp <= !rp;", "if (pop[0] && ((!o[0] && !o[1]) || (o[2] && p2))) rp <= !rp;", ['core']),
  ('fq_onehot_nochk', 'ppc_fetchq.sv',
   ["    if (pop[2])     h1[EW-1:HW] <= o[2] ? w_e[EW-1:HW] : (F2 && o[1]) ? w2e[EW-1:HW] : r2[EW-1:HW];",
    '$fatal(1, "fetch queue count copy");'],
   ["    if (pop[2])     h1[EW-1:HW] <= o[1] ? w_e[EW-1:HW] : (F2 && o[1]) ? w2e[EW-1:HW] : r2[EW-1:HW];", ';'], ['core']),
  ('redir_copy', 'ppc_core.sv',
   "  (* keep = 1 *) wire redirect_f = redirect_c || (br_go && br_mis);",
   "  (* keep = 1 *) wire redirect_f = redirect_c;", ['core']),
  ('br_z_case_nochk', 'ppc_core.sv',
   ["  wire        br_taken  = m_z ? brt_z1 : brt_z0;", '        $fatal(1, "branch decision against BI");\n'],
   ["  wire        br_taken  = brt_z0;", "        ;\n"], ['core']),
  ('mf_c0_late', 'ppc_core.sv',
   "  wire [31:0]                r_late  = r_sum | r_rl | r_rr | r_clz | mf_c0v;",
   "  wire [31:0]                r_late  = r_sum | r_rl | r_rr | r_clz;", ['core']),
  ('mrp_copy_nochk', 'ppc_core.sv',
   ["    if (!mem_hold) m_rpq <= mx.rp;", '$fatal(1, "m_rpq");', '$fatal(1, "q_hold");'],
   ["    if (!mem_hold) m_rpq <= mx.rp && mx.req;", ';', ';'], ['core']),
  ('fma_nc_raw', 'ppc_fpu_fma.sv',
   "  assign lzfc = has_c ? lzfc_r : 6'd0;", "  assign lzfc = lzfc_r;", ['core']),
  # this pass: the predecode tables (ppc_pdec), checked against the decoder on every word
  ('pdec_ua_ra0', 'ppc_pdec.sv',
   "  assign ua   = v[0] || (v[27] && w[20:16] != 5'd0);", "  assign ua   = v[0] || v[27];", ['core']),
  ('pdec_rc', 'ppc_pdec.sv',
   "  assign rc   = v[6] || (v[28] && w[0]);", "  assign rc   = v[6] || v[28];", ['core']),
  ('pdec_sel19', 'ppc_pdec.sv',
   "  assign v = opc == 6'd31 ? v31 : opc == 6'd19 ? v19 : vo;", "  assign v = opc == 6'd31 ? v31 : vo;", ['core']),
  ('pdec_xs', 'ppc_pdec.sv',
   "  assign xs   = v[3];", "  assign xs   = v[3] ^ w[5];", ['core']),
  ('sfma_nan_order', 'ppc_fpu_sfma.sv',
   "  wire [22:0] nf2  = r1_q.a_nan ? r1_q.ma[22:0] : r1_q.pb_nan ? r1_q.b[22:0] : r1_q.mc[22:0];",
   "  wire [22:0] nf2  = r1_q.pb_nan ? r1_q.b[22:0] : r1_q.a_nan ? r1_q.ma[22:0] : r1_q.mc[22:0];", ['core']),
  ('sfma_effsub', 'ppc_fpu_sfma.sv',
   "  wire r_sb2  = r1_q.sd.special ? sp_sign2 : (r1_q.has_b ? r1_q.sbe : r1_q.sp_);",
   "  wire r_sb2  = r1_q.sd.special ? sp_sign2 : r1_q.sbe;", ['core']),
  ('hd_fz_sel', 'ppc_core.sv',
   "    hd_fz  <= id_pop ? nx_haz[1] : nx_haz[0];", "    hd_fz  <= nx_haz[0];", ['core']),
  # this pass: the operand capture's two LUTs; the DBAT / micro-TLB OR
  ('cap_fw_order', 'ppc_core.sv',
   "    fw_fwd = !k[1] ? vm : !k[0] ? vl : vw;", "    fw_fwd = !k[1] ? vm : !k[0] ? vw : vl;", ['core']),
  ('cap_base_nochk', 'ppc_core.sv',
   ["  wire [31:0] y_cap = y_k == 2'd0 ? y_b : y_fw;", '      $fatal(1, "operand capture against fw_mux");\n'],
   ["  wire [31:0] y_cap = y_k[1] == 1'b0 ? y_b : y_fw;", "      ;\n"], ['core']),
  ('dor_low_nochk', 'ppc_core.sv',
   ["{db_pa | du_pahi[19:5], du_any ? du_pahi[4:0] : ea_x[16:12]};",
    '      $fatal(1, "a DBAT and a micro-TLB hit together, or the ORed translation differs");\n'],
   ["{db_pa | du_pahi[19:5], ea_x[16:12]};", "      ;\n"], ['core']),
  ('dor_wimg_nochk', 'ppc_core.sv',
   ["  wire  [3:0]  d_wimg_x = !msr_q[MSR_DR] ? 4'b0011 : db_wimg | du_wimg_s;",
    '      $fatal(1, "a DBAT and a micro-TLB hit together, or the ORed translation differs");\n'],
   ["  wire  [3:0]  d_wimg_x = !msr_q[MSR_DR] ? 4'b0011 : db_wimg;", "      ;\n"], ['core', 'sub']),
  ('du_hit_pr_nochk', 'ppc_core.sv',
   ["      du_hit[k] = du_v[k] && du_pr[k] == msr_q[MSR_PR] && sum_is(", '        $fatal(1, "a DBAT and a micro-TLB hit together, or the ORed translation differs");\n'],
   ["      du_hit[k] = du_v[k] && sum_is(", "        ;\n"], ['core']),
  # this pass: the single unit's S1 / S2 split
  ('sfma_e0_sel', 'ppc_fpu_sfma.sv',
   "s2_d.e0 = r1_q.e0s ? r1_q.e0b : r1_q.e0p;", "s2_d.e0 = r1_q.e0s ? r1_q.e0p : r1_q.e0b;", ['core']),
  ('sfma_sh_big', 'ppc_fpu_sfma.sv',
   "wire   sh_big = has_c ? shc > exp_t'(127) : shn > exp_t'(127);", "wire   sh_big = shc > exp_t'(127);", ['core']),
  ('ff_route_dbl', 'ppc_core.sv',
   ".f_fast (ex_ffast),", ".f_fast (1'b0),", ['core']),
  ('ff_slot_ix', 'ppc_core.sv',
   "    if (f_iss && ex_ffast) fr_ix[LS - 1] <= ex_frd;\n", "", ['core']),
  ('ff_slot_at', 'ppc_core.sv',
   "({FL{f_iss && ex_ffast}} & (FL'(1) << (LS - 1)))", "({FL{f_iss && ex_ffast}} & (FL'(1) << (LS - 2)))", ['core']),
  ('ff_wait_off', 'ppc_core.sv',
   "fp_wait = ex_valid && (fwq || (ex_ffw && ffbl_q) ||", "fp_wait = ex_valid && (fwq ||", ['core']),
  ('ff_wait_off_nochk', 'ppc_core.sv',
   ["fp_wait = ex_valid && (fwq || (ex_ffw && ffbl_q) ||", NOCHK_OK],
   ["fp_wait = ex_valid && (fwq ||", ""], ['core']),
  ('ff_order_nochk', 'ppc_core.sv',
   ["ffbl_q <= (fr_at_n >> LS) != '0;", NOCHK_OK, NOCHK_FFBL],
   ["ffbl_q <= fr_at_n[LS];", "", ""], ['core']),
  ('ff_rsqrte_div', 'ppc_decode.sv',
   "u.funit = 2'd1; u.fop = 4'(FMA_RSQRTE);", "u.funit = 2'd2; u.fop = 4'(DIV_RSQRTE);", ['core']),
  ('ff_rsqrte_est', 'cflags', '-DPPC_FPU_RSQRTE_EST', '', ['core']),
  # the predecode's raw-word compares (the _nochk forms drop their check)
  ('pd_ctr_bclr_nochk', 'ppc_pdw.sv',
   ["wire         r_bc    = pd_op == 6'd16 || (r_19 && pd_x == 10'd16);", NOCHK_PD],
   ["wire         r_bc    = pd_op == 6'd16;", "      ;\n"], ['core']),
  ('pd_cr_crlog_nochk', 'ppc_pdw.sv',
   ["                                  pd_x == 10'd449);", NOCHK_PD],
   ["                                  1'b0);", "      ;\n"], ['core']),
  ('pd_chk', 'ppc_pdw.sv',
   "wire         r_mcrxr = r_31 && pd_x == 10'd512;", "wire         r_mcrxr = r_31 && pd_x == 10'd513;", ['core']),
  # the timing pass's registered copies (wb_live_q, ex_ffw)
  ('wb_live_clr_nochk', 'ppc_core.sv',
   ["          wb_live_q <= 1'b0;\n", '      if (wb_live_q != (wb_isld && !wb_got)) $fatal(1, "wb_live_q");\n'], ["", ""], ['core']),
  ('ffw_unit_nochk', ['ppc_pdw.sv', 'ppc_core.sv', 'ppc_core.sv'],
   ["    pd.frq     = pa_op && pa_d && pa5 == 5'd26 && !mark;", NOCHK_FFW,
    '        $fatal(1, "fast route kind of %08X off the multiply-add port", q_hi);\n'],
   ["    pd.frq     = pa_op && pa5 == 5'd26 && !mark;", "", "        ;\n"], ['core']),
] + [
  # ---- D-30: the BTB (design C) and the I-cache's pre-read port
  ('btb_nochk', 'ppc_core.sv',
   "  wire         bq_ok  = bt_slot ? bq_ok2 : bq_ok1;", "  wire         bq_ok  = 1'b1;", ['corel', 'subl']),
  ('btb_chk_dir', 'ppc_core.sv',
   "(pw_pred && pw_tgt[31:2] == bt_tgt);", "(pw_tgt[31:2] == bt_tgt);", ['corel']),
  ('btb_chk_reg', 'ppc_core.sv',
   "  wire         bq_ok1 = bt_preg ? (pw_reg && pw_rstk)", "  wire         bq_ok1 = bt_preg ? 1'b1", ['corel']),
  ('btb_rtm_off', 'ppc_core.sv',
   "      mx.rtm  = BTB && ex_pred", "      mx.rtm  = 1'b0 && ex_pred", ['corel', 'subl']),
  ('btb_rtm_untaken', 'ppc_core.sv',
   "BR_BCCTR) && br_taken && !rt_eq;", "BR_BCCTR) && !rt_eq;", ['corel']),
  ('btb_rteq_sel', 'ppc_core.sv',
   "  wire        rt_eq  = (ex_u.bkind == BR_BCLR) ?", "  wire        rt_eq  = 1'b1 ?", ['corel']),
  ('btb_rp_gate', 'ppc_core.sv',
   "  wire         bt_use = bt_pv && !(bt_preg && rp_v);", "  wire         bt_use = bt_pv;", ['corel']),
  ('btb_soft_dup', 'ppc_core.sv',
   "  wire         f_soft1 = rsp_use && pw_pred && !bq_out;", "  wire         f_soft1 = rsp_use && pw_pred;", ['corel']),
  ('btb_pred_bit', 'ppc_core.sv',
   "  wire         w_prd   = pw_pred || (bq_out && !bt_slot && bt_preg && pw_reg && pw_rstk);", "  wire         w_prd   = pw_pred;", ['corel']),
  ('btb_bmis_keep', 'ppc_core.sv',
   "  wire  rsp_ok  = rsp && !f_discard && !redir_q && !bmis_q;", "  wire  rsp_ok  = rsp && !f_discard && !redir_q;", ['corel']),
  ('btb_bmis_tgt', 'ppc_core.sv',
   "        cr_tgt_q  <= pw_pred ? pw_tgt[31:2] :", "        cr_tgt_q  <= 1'b0 ? pw_tgt[31:2] :", ['corel']),
  ('btb_key', 'ppc_core.sv',
   "  wire  [31:2] f_rkey = f_rsp_pc[31:2] - (rq_pp_q ? 30'd2 : 30'd1);", "  wire  [31:2] f_rkey = f_rsp_pc[31:2];", ['corel']),
  ('btb_no_flush', 'ppc_core.sv',
   "        .flush (bt_flush), .busy ()", "        .flush (1'b0), .busy ()", ['corel', 'subl']),
  ('btb_iseq', 'ppc_core.sv',
   "  assign i_seq  = !redir_q && !pt_q && f_sq && !f_xm;", "  assign i_seq  = !redir_q && f_sq && !f_xm;", ['subl']),
  ('btb_pa_ea', 'ppc_core.sv',
   "i_arr ? i_pa : 20'hF00FF, i_way})", "i_arr ? f_rsp_pc[31:12] : 20'hF00FF, i_way})", ['subl']),
  ('btb_ctx', 'ppc_core.sv',
   "        .ctx ({f_ir, f_pr}),", "        .ctx (2'b00),", ['corel', 'subl']),
  ('ic_pre_way', 'ppc_icache.sv',
   "    if (PRE && pre_q) i_data <= dq[pw_q];", "    if (PRE && pre_q) i_data <= dq[st[4:3]];", ['icpre', 'subl']),
  ('ic_pre_haz', 'ppc_icache.sv',
   "pre_q && !pend && ta_v && !ta_look && !t_haz && !wk_act && !f_act;", "pre_q && !pend && ta_v && !ta_look && !wk_act && !f_act;", ['icpre', 'subl']),
  ('ic_pre_fill', 'ppc_icache.sv',
   "  wire         pre_take = PRE && i_pv && fast && !wk_act && !f_act && hid_ice;", "  wire         pre_take = PRE && i_pv && fast && !wk_act && hid_ice;", ['icpre']),
  ('ic_pre_fsq', 'ppc_icache.sv',
   "        if (pre_take) begin fs_q <= 1'b0; f2_q <= 1'b0; end", "", ['icpre']),
  ('ic_pre_tag', 'ppc_icache.sv',
   "  wire         phit     = pfast && tg[pw_q] == ppa_q && ppa_q != RSV_TAG;", "  wire         phit     = pfast && ppa_q != RSV_TAG;", ['icpre']),
  ('ic_pre_rsv', 'ppc_icache.sv',
   "  wire         phit     = pfast && tg[pw_q] == ppa_q && ppa_q != RSV_TAG;", "  wire         phit     = pfast && tg[pw_q] == ppa_q;", ['icpre', 'subl']),
  ('ic_pre_off', 'ppc_icache.sv',
   "  wire         pre_take = PRE && i_pv", "  wire         pre_take = 1'b0 && i_pv", ['icpre']),
  ('ic_pre_nx', 'ppc_icache.sv',
   "          nx      <= {ppa_q, i_addr[11:2]} + 30'd1;", "          nx      <= {ppa_q, i_addr[11:2]};", ['icpre']),
  # ---- D-30: two-word fetch
  ('f2_soft2', 'ppc_core.sv',
   "  wire         f_soft  = f_soft1 || f_soft2;", "  wire         f_soft  = f_soft1;", ['core', 'sub']),
  ('f2_drop', 'ppc_core.sv',
   "    .we2 (rsp_use && w2_in && !pw_pred && !redir_hard_q),", "    .we2 (rsp_use && w2_in && !redir_hard_q),", ['core']),
  ('f2_pc2', 'ppc_core.sv',
   ".w2_pc ({f_rsp4, 2'b00})", ".w2_pc (f_rsp_pc)", ['core']),
  ('f2_fpc', 'ppc_core.sv',
   "      if (f_issue)      f_pc <= fetch_pc + (i_w2 ? 32'd8 : 32'd4);", "      if (f_issue)      f_pc <= fetch_pc + 32'd4;", ['core']),
  ('f2_room', 'ppc_core.sv',
   "  wire  f_room2 = {1'b0, q_cnt} + {1'b0, f_nin} < 4'd3;", "  wire  f_room2 = {1'b0, q_cnt} + {1'b0, f_nin} < 4'd4;", ['core']),
  ('f2_line', 'ppc_core.sv',
   "f_room2 && fetch_pc[4:2] != 3'd7;", "f_room2;", ['core']),
  ('f2_nin', 'ppc_core.sv',
   "  wire  [2:0] f_nin  = {1'b0, rsp_use && w2_in, rsp_use && !w2_in};", "  wire  [2:0] f_nin  = {2'b0, rsp_use};", ['core']),
  ('f2_mark', 'ppc_core.sv',
   "    if (rst || s_flt) f_w2_q <= 1'b0;", "    if (rst) f_w2_q <= 1'b0;", ['core']),
  ('f2_slot_chk', 'ppc_core.sv',
   "  wire         bq_ok  = bt_slot ? bq_ok2 : bq_ok1;", "  wire         bq_ok  = bq_ok1;", ['corel', 'subl']),
  ('f2_pred2', 'ppc_core.sv',
   "  wire         w2_prd   = pw2_pred || (bq_out && bt_slot && bt_preg && pw2_reg && pw2_rstk);",
   "  wire         w2_prd   = pw2_pred;", ['corel']),
  ('fq_rp', 'ppc_fetchq.sv',
   "if (pop[0] && ((!o[0] && !o[1] && !o[2]) || (o[2] && p2))) rp <= !rp;", "if (pop[0] && !o[0] && !o[1] && !o[2]) rp <= !rp;", ['core']),
  ('fq_h1', 'ppc_fetchq.sv',
   "    else if (F2 && o[0]) h1[EW-1:HW] <= w2e[EW-1:HW];\n", "", ['core']),
  ('fq_ring1', 'ppc_fetchq.sv',
   "    if (F2 && o[1]) rg[rp] <= w2e;", "    if (F2 && o[1]) rg[!rp] <= w2e;", ['core']),
  ('ic2_bank', 'ppc_icache.sv',
   "  wire [8:0]  nx1_e = {nx1[11:5], nx1[4:3] + {1'b0, nx1[2]}};", "  wire [8:0]  nx1_e = {nx1[11:5], nx1[4:3]};", ['icache', 'sub']),
  ('ic2_av2', 'ppc_icache.sv',
   "            f2_q <= nxn[4:2] != 3'd7 && av(kl, nxn[4:2], f_act, f_line, avail) &&\n                    av(kl, nxn[4:2] + 3'd1, f_act, f_line, avail);",
   "            f2_q <= nxn[4:2] != 3'd7 && av(kl, nxn[4:2], f_act, f_line, avail);", ['icache']),
  ('ic2_xing', 'ppc_icache.sv',
   "  wire        xing = nx[4:2] == 3'd7 || (pair && nx[4:2] == 3'd6);", "  wire        xing = nx[4:2] == 3'd7;", ['icache']),
  ('ic2_srv', 'ppc_icache.sv',
   "            f2_q    <= !p_ui && pa[4:2] < 3'd6 && hid_ice &&", "            f2_q    <= !p_ui && pa[4:2] != 3'd7 && hid_ice &&", ['icache']),
  ('ic2_phit', 'ppc_icache.sv',
   "          f2_q    <= i_addr[4:2] < 3'd6;", "          f2_q    <= i_addr[4:2] != 3'd7;", ['icache', 'sub']),
  ('ic2_data2', 'ppc_icache.sv',
   "    i_data2 <= dq2[kl_way];", "    i_data2 <= dq2[st[4:3]];", ['icache']),
  # ---- D-30: the 603e CR look-ahead (EX checks every branch resolved at fetch)
  ('la_q', 'ppc_core.sv',
   "  wire         la_crb = (q_cnt > 3'd0 && (q_pd.w_cr  || q_pd.fwcr))  ||", "  wire         la_crb = 1'b0 ||", ['core']),
  ('la_q3', 'ppc_core.sv',
   "(q_cnt > 3'd3 && (q3_pd.w_cr || q3_pd.fwcr)) ||", "", ['core', 'corel']),
  ('la_ex', 'ppc_core.sv',
   "                        (ex_valid && ex_w_cr) || (m.valid && m.cr_we);", "                        (m.valid && m.cr_we);", ['core']),
  ('la_mem', 'ppc_core.sv',
   "                        (ex_valid && ex_w_cr) || (m.valid && m.cr_we);", "                        (ex_valid && ex_w_cr);", ['core']),
  ('la_ctr', 'ppc_core.sv',
   "                        (ex_valid && ex_w_ctr) || (m.valid && m.ctr_we);", "                        1'b0;", ['corel']),
  ('la_w1', 'ppc_core.sv',
   "la_f(i_data2, la_crb || w1_may[1], la_ctb || w1_may[0], cr_q, ctr1_q)", "la_f(i_data2, la_crb, la_ctb || w1_may[0], cr_q, ctr1_q)", ['core']),
  ('la_may', 'ppc_core.sv',
   "(op == 6'd31 && (w[0] || x == 10'd0 || x == 10'd32 || x == 10'd144 || x == 10'd512));",
   "(op == 6'd31 && (w[0] || x == 10'd0 || x == 10'd32 || x == 10'd144));", ['core']),
  ('la_bit', 'ppc_core.sv',
   "cr[5'd31 - w[20:16]] == bo[3]", "cr[w[20:16]] == bo[3]", ['core']),
  ('la_ctrz', 'ppc_core.sv',
   "    d    = (bo[2] || (!c1 ^ bo[1])) &&", "    d    = (bo[2] || !c1) &&", ['corel']),
  ('la_fp', 'ppc_core.sv',
   "  wire         la_crb = (q_cnt > 3'd0 && (q_pd.w_cr  || q_pd.fwcr))  || (q_cnt > 3'd1 && (q1_pd.w_cr || q1_pd.fwcr)) ||",
   "  wire         la_crb = (q_cnt > 3'd0 && (q_pd.w_cr))  || (q_cnt > 3'd1 && (q1_pd.w_cr)) ||", ['core']),
] + [
  # D-41: the data cache's snoop port, through the subsystem with a DMA master (--snoop 2)
  ('sn_core_kill', 'ppc_core.sv',
   "  wire        res_in = !res_kill && ((m.valid && m.res_set) ? 1'b1 : (m.valid && m.res_clr) ? 1'b0 : res_q);",
   "  wire        res_in = ((m.valid && m.res_set) ? 1'b1 : (m.valid && m.res_clr) ? 1'b0 : res_q);", ['subs']),
  ('sn_rsv_keep', 'ppc_dcache.sv',
   "rv_kill <= rsv_q ? rv_new : (rv_kill || (sn_kw && rv_pa == sn_addr));", "rv_kill <= rsv_q ? rv_new : rv_kill;", ['subs']),
  ('sn_rsv_late', 'ppc_dcache.sv',
   "assign res_kill = SNOOP && (rv_kill || (rsv_q && rv_new));", "assign res_kill = SNOOP && rv_kill;", ['subs']),
  ('sn_no_push', 'ppc_dcache.sv', "wire         sn_dty = st_sn[5 + sn_hw];", "wire         sn_dty = 1'b0;", ['subs']),
  ('sn_ack_early', 'ppc_dcache.sv', "SN_C:    if (sn_hit && sn_dty) sn_st <= SN_WB;", "SN_C:    if (1'b0) sn_st <= SN_WB;", ['subs']),
  ('sn_sb_miss', 'ppc_dcache.sv', "wire sn_ok_b = quiet && !tw_last && !wk_act;",
   "wire sn_ok_b = (quiet || sb_v) && !tw_last && !wk_act;", ['subs']),
  ('sn_no_inv', 'ppc_dcache.sv', "if (sn_iq) do_tw = 1'b1;                  // tw_tag: RSV_TAG", "", ['subs']),
  ('sn_post', 'ppc_dcache.sv', "tw_en_q == 2'd0 && !sn_np && !rst;", "tw_en_q == 2'd0 && !rst;", ['subs']),
  ('sn_one_steal', 'ppc_dcache.sv', "wire         sn_rd  = SNOOP && (sn_st == SN_A || sn_st == SN_B);",
   "wire         sn_rd  = SNOOP && (sn_st == SN_A);", ['subs']),
  # integer loads with update: step 1 writes rA from step 0's EA (VF3's lbzux
  # r4,r6,r4). Benches corea / suba: --alias (rD = rB and the reproducer)
  ('upd_rb_reread', 'ppc_core.sv',                     # the bug put back: rA + d / rB read again, after the load
   # (and its raw-word check, so that the data compare must see it)
   ["  wire id_xuea    = id_s1 && q_pd.crack == 2'd1 && !q_pd.fdx && !q_pd.fpw && !q_pd.ifault;",
    '      $fatal(1, "update load kind of %08X (step 1 %b) against the raw word", q_hi, id_s1);\n'],
   ["  wire id_xuea    = 1'b0;", "      ;\n"], ['corea', 'suba']),
  # study R104's restructured paths: the interrupt's vector chosen by ex_s0ok, the FMA's shift
  # with the special-operand decision last
  # study R104: the whole interrupt request registered (the Daytona 2 hang): caught only with --intdrop
  ('r104_irq_reg', 'ppc_core.sv', "  wire         irq_req  = ee_q && (int_q || dec_pend);",
   "  logic irq_req; always_ff @(posedge clk) irq_req <= !rst && ee_q && (int_q || dec_pend);", ['corei']),
  ('r103_irq_vec', 'ppc_core.sv', "  assign redirect_pc = ex_s0ok ? rp_s1 : rp_s0;", "  assign redirect_pc = rp_s1;", ['core']),
  ('r103_fma_spc', 'ppc_fpu_fma.sv', "assign sh = kq.cvt ? sh_cv : (sh_ns & {8{~special}});", "assign sh = kq.cvt ? sh_cv : sh_ns;", ['core']),
  ('upd_luse', 'ppc_core.sv',                          # step 1 waits on its own load: its EA has left EX
   ["wire load_use_d0 = !id_xuea && (", "wire load_use_d1 = !id_xuea && ("],
   ["wire load_use_d0 = (", "wire load_use_d1 = ("], ['corea']),
  ('upd_imm', 'ppc_core.sv',                           # y left at the form's immediate (d, or fimm_q's 4)
   "    if (id_xuea) id_imm = 32'd0;", "", ['corea']),
  ('upd_yreg', 'ppc_core.sv',                          # y from rB, as before
   "  wire        id_yreg  = q_pd.yreg && !id_xfea && !id_xuea;", "  wire        id_yreg  = q_pd.yreg && !id_xfea;", ['corea']),
  ('upd_xex', 'ppc_core.sv',                           # x not from EX's sum
   " || (id_xuea && ex_valid);", ";", ['corea']),
]

VFLAGS = ['--cc', '--exe', '--build', '-j', '8', '-O3', '-Wno-fatal', '-Wno-UNUSEDSIGNAL', '-Wno-DECLFILENAME']

def build_run(d, bench, drop=''):
    rtl = [os.path.join(d, 'rtl', 'cpu', f + '.sv') for f in CPU_RTL]
    sim = os.path.join(ROOT, 'sim')
    if bench in ('core', 'sub', 'corel', 'subl', 'corem', 'subm', 'subs', 'corea', 'suba', 'corei'):
        top = 'ppc_core' if bench in ('core', 'corel', 'corem', 'corea', 'corei') else 'ppc_cpu'
        defs = '' if top == 'ppc_core' else ' -DPPC_CPU'
        obj = os.path.join(d, 'obj_' + bench)
        cmd = ['verilator'] + VFLAGS + ['--top-module', top]
        if top == 'ppc_cpu': cmd += ['+define+PPC_CACHE_SIM', '--assert']
        cf = f'-O2 -std=c++17{defs} -DPPC_FPU_RSQRTE_EST -I{sim}/cpu -I{sim}/fpu -I{sim}/cache'
        if drop: cf = cf.replace(' ' + drop, '')
        cmd += ['-CFLAGS', cf,
                '-LDFLAGS', '-lmpfr -lgmp', '--Mdir', obj] + rtl + [os.path.join(sim, 'cpu', 'tb_ppc_core.cpp')]
        exe = os.path.join(obj, 'V' + top)
        runs = {'core':  [[exe, '400', '1', '1500', '9'], [exe, '200', '7001', '1500', '9', '--rdy', '25']],
                'sub':   [[exe, '80', '1', '1500', '80'], [exe, '80', '9001', '1500', '6']],
                # D-30: programs with loops, calls and rewritten code (the BTB's paths)
                'corel': [[exe, '300', '30001', '1500', '4', '--loops'], [exe, '150', '31001', '1500', '9', '--loops', '--rdy', '25']],
                'subl':  [[exe, '100', '33001', '1500', '6', '--loops'], [exe, '100', '34001', '1500', '80', '--loops']],
                # misaligned accesses
                'corei': [[exe, '200', '1', '1500', '4', '--intdrop', '300'], [exe, '150', '40001', '1500', '4', '--mis', '--intdrop', '100']],
                'corem': [[exe, '200', '40001', '1500', '4', '--mis'], [exe, '150', '41001', '1500', '9', '--mis', '--rdy', '25'],
                          [exe, '150', '42001', '1500', '4', '--mis', '--dlat', '1']],
                'subm':  [[exe, '100', '53001', '1500', '80', '--mis'], [exe, '60', '54001', '1500', '6', '--mis']],
                # update forms with overlapping registers, the reproducer
                'corea': [[exe, '200', '70001', '1500', '4', '--alias'], [exe, '150', '71001', '1500', '9', '--alias', '--mis'],
                          [exe, '150', '72001', '1500', '4', '--alias', '--dlat', '1']],
                'suba':  [[exe, '100', '73001', '1500', '80', '--alias'], [exe, '60', '74001', '1500', '6', '--alias']],
                # the snoop port (D-41): reads and writes from a DMA master
                'subs':  [[exe, '100', '60001', '1500', '80', '--snoop', '2'], [exe, '100', '61001', '1500', '6', '--snoop', '2'],
                          [exe, '60', '62001', '1500', '20', '--snoop', '2', '--loops']]}[bench]
    elif bench == 'icpre':
        top = 'ppc_icache'
        obj = os.path.join(d, 'obj_' + bench)
        cmd = ['verilator'] + VFLAGS + ['--assert', '+define+PPC_CACHE_SIM', '--top-module', top,
               '-CFLAGS', f'-O2 -std=c++17 -I{sim}/cache', '--Mdir', obj,
               os.path.join(d, 'rtl', 'cpu', 'ppc_cache_pkg.sv'), os.path.join(d, 'rtl', 'cpu', 'ppc_cache_ram.sv'),
               os.path.join(d, 'rtl', 'cpu', 'ppc_icache.sv'), os.path.join(sim, 'btb', 'tb_icache_pre.cpp')]
        exe = os.path.join(obj, 'V' + top)
        runs = [[exe, str(s), '200000'] for s in (1, 2, 3, 4)]
    else:
        top = 'ppc_dcache' if bench == 'dcache' else 'ppc_icache'
        obj = os.path.join(d, 'obj_' + bench)
        cmd = ['verilator'] + VFLAGS + ['--assert', '+define+PPC_CACHE_SIM', '--top-module', top,
               '-CFLAGS', f'-O2 -std=c++17 -I{sim}/cache', '--Mdir', obj,
               os.path.join(d, 'rtl', 'cpu', 'ppc_cache_pkg.sv'), os.path.join(d, 'rtl', 'cpu', 'ppc_cache_ram.sv'),
               os.path.join(d, 'rtl', 'cpu', top + '.sv'), os.path.join(sim, 'cache', 'tb_' + bench[0:1] + 'cache.cpp')]
        exe = os.path.join(obj, 'V' + top)
        runs = [[exe, '100000', str(s)] for s in (1, 2, 3, 4)] + [[exe, '50000', '5', '1', '80']]
    r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        return 'build failed: ' + r.stderr[-300:]
    for c in runs:
        try:
            p = subprocess.run(c, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=3600)
        except subprocess.TimeoutExpired:
            return 'caught (timeout) by ' + ' '.join(c[1:])
        if p.returncode != 0 or 'FAIL' in p.stdout:
            line = next((l for l in p.stdout.splitlines() if 'seed' in l or 'MISMATCH' in l or 'Error' in l or 'FAIL' in l), '')
            return 'caught by ' + ' '.join(c[1:]) + ': ' + line.strip()[:140]
    shutil.rmtree(obj, ignore_errors=True)
    return 'missed'

def main():
    want = set(sys.argv[1:])
    os.makedirs(WORK, exist_ok=True)
    for name, f, old, new, benches in FAULTS:
        if want and name not in want: continue
        d = os.path.join(WORK, name)
        shutil.rmtree(d, ignore_errors=True)
        shutil.copytree(os.environ.get('FAULT_RTL', os.path.join(ROOT, 'rtl')), os.path.join(d, 'rtl'))
        drop = ''
        if f == 'cflags':
            drop = old
        else:
            # a fault may span files: f is then a list, one file per replacement
            olds, news = (old, new) if isinstance(old, list) else ([old], [new])
            files = f if isinstance(f, list) else [f] * len(olds)
            texts = {}
            for fn in set(files): texts[fn] = open(os.path.join(d, 'rtl', 'cpu', fn)).read()
            bad = [(fn, o) for fn, o in zip(files, olds) if texts[fn].count(o) != 1]
            if bad:
                print(f'{name}: pattern not found once in {bad[0][0]}: {bad[0][1][:60]!r}'); continue
            for fn, o, n in zip(files, olds, news): texts[fn] = texts[fn].replace(o, n)
            for fn, t in texts.items(): open(os.path.join(d, 'rtl', 'cpu', fn), 'w').write(t)
        res = []
        for b in benches:
            res.append(f'{b}: ' + build_run(d, b, drop))
        print(f'{name}: ' + ' | '.join(res), flush=True)
        for b in benches: shutil.rmtree(os.path.join(d, 'obj_' + b), ignore_errors=True)

if __name__ == '__main__':
    main()
