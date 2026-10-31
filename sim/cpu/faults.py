#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the pipelined data port, the pipelined D-cache, the
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
           'ppc_fpu_rsqe ppc_fpu_fast ppc_core '
           'ppc_cache_pkg ppc_cache_ram ppc_icache ppc_dcache ppc_cpu').split()

# the core's simulation checks of the fast port's wiring
NOCHK_ROUTE = ('      if (f_iss && ex_ffast != fpu_fast_route(fma_op_t\'(ex_u.fop), ex_u.fsgl, fa_q, fb_q, fc_q))\n'
               '        $fatal(1, "fast route %0d is not for EX\'s operands", ex_ffast);\n')
RT_FAST     = "ex_ffast <= fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_cap, fb_cap, fc_cap);"
RT_FFW      = "                  fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_cap, fb_cap, fc_cap);"
NOCHK_FFW   = '      if (ex_ffw != (ex_fma && ex_ffast)) $fatal(1, "ex_ffw");\n'
NOCHK_PD    = '      $fatal(1, "predecode raw compares against the decoder, word %h", i_data);\n'
NOCHK_OK    = '      if (f_iss && ex_ffast && !f_fast_ok) $fatal(1, "fast op issued against the unit\'s own check");\n'
NOCHK_FFBL  = '      if (ffbl_q != ((fr_at >> LS) != \'0)) $fatal(1, "ffbl_q");\n'

# (name, file, old, new, benches)
FAULTS = [
  ('load_use_mem', 'ppc_core.sv',
   "(m_ldw && lu_q(qe_m,", "(1'b0 && lu_q(qe_m,", ['core']),
  ('wb_fwd_early', 'ppc_core.sv',
   "(wb_ldw && lu_q(qe_w,", "(1'b0 && lu_q(qe_w,", ['core']),
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
   "assign i_seq  = !redir_q && f_sq && !f_xm;", "assign i_seq  = !redir_q;", ['sub']),
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
   "qe_e <= ex_hold  ? qe_hc[1] : qe_hc[0];", "qe_e <= qe_hc[0];", ['core']),
  ('qf_w_write', 'ppc_core.sv',
   "qe_hc[2] & {4{qf_wm}};", "qe_hc[2];", ['core']),
  ('hn_pop_head', 'ppc_core.sv',
   "wire        hn_q1 = id_take && q_cnt >= 3'd2;", "wire        hn_q1 = id_take && q_cnt >= 3'd3;", ['core']),
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
   "ex_ffast <= fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_cap, fb_cap, fc_cap);",
   "ex_ffast <= fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_q, fb_q, fc_q);", ['core']),
  ('ff_route_stale_nochk', 'ppc_core.sv',
   [RT_FAST, RT_FFW, NOCHK_ROUTE, NOCHK_FFW],
   [RT_FAST.replace('fa_cap, fb_cap, fc_cap', 'fa_q, fb_q, fc_q'), RT_FFW.replace('fa_cap, fb_cap, fc_cap', 'fa_q, fb_q, fc_q'), "", ""], ['core']),
  ('ff_route_nofwd_nochk', 'ppc_core.sv',
   [RT_FAST, RT_FFW, NOCHK_ROUTE, NOCHK_FFW],
   [RT_FAST.replace('fa_cap, fb_cap, fc_cap', 'rf_fa, rf_fb, rf_fc'), RT_FFW.replace('fa_cap, fb_cap, fc_cap', 'rf_fa, rf_fb, rf_fc'), "", ""], ['core']),
  ('ff_route_dbl', 'ppc_core.sv',
   ".f_fast (ex_ffast),", ".f_fast (1'b0),", ['core']),
  ('ff_slot_ix', 'ppc_core.sv',
   "    if (f_iss && ex_ffast) fr_ix[LS - 1] <= ex_frd;\n", "", ['core']),
  ('ff_slot_at', 'ppc_core.sv',
   "({FL{f_iss && ex_ffast}} & (FL'(1) << (LS - 1)))", "({FL{f_iss && ex_ffast}} & (FL'(1) << (LS - 2)))", ['core']),
  ('ff_wait_off', 'ppc_core.sv',
   "              (ex_ffw && ffbl_q) ||\n", "", ['core']),
  ('ff_wait_off_nochk', 'ppc_core.sv',
   ["              (ex_ffw && ffbl_q) ||\n", NOCHK_OK],
   ["", ""], ['core']),
  ('ff_order_nochk', 'ppc_core.sv',
   ["ffbl_q <= (fr_at_n >> LS) != '0;", NOCHK_OK, NOCHK_FFBL],
   ["ffbl_q <= fr_at_n[LS];", "", ""], ['core']),
  ('ff_rsqrte_div', 'ppc_decode.sv',
   "u.funit = 2'd1; u.fop = 4'(FMA_RSQRTE);", "u.funit = 2'd2; u.fop = 4'(DIV_RSQRTE);", ['core']),
  ('ff_rsqrte_est', 'cflags', '-DPPC_FPU_RSQRTE_EST', '', ['core']),
  # the predecode's raw-word compares (the _nochk forms drop their check)
  ('pd_ctr_bclr_nochk', 'ppc_core.sv',
   ["wire         r_bc    = pd_op == 6'd16 || (r_19 && pd_x == 10'd16);", NOCHK_PD],
   ["wire         r_bc    = pd_op == 6'd16;", "      ;\n"], ['core']),
  ('pd_cr_crlog_nochk', 'ppc_core.sv',
   ["                                  pd_x == 10'd449);", NOCHK_PD],
   ["                                  1'b0);", "      ;\n"], ['core']),
  ('pd_chk', 'ppc_core.sv',
   "wire         r_mcrxr = r_31 && pd_x == 10'd512;", "wire         r_mcrxr = r_31 && pd_x == 10'd513;", ['core']),
  # the timing pass's registered copies (wb_live_q, ex_ffw)
  ('wb_live_clr_nochk', 'ppc_core.sv',
   ["          wb_live_q <= 1'b0;\n", '      if (wb_live_q != (wb_isld && !wb_got)) $fatal(1, "wb_live_q");\n'], ["", ""], ['core']),
  ('ffw_unit_nochk', 'ppc_core.sv',
   ["      ex_ffw   <= id_u.funit == 2'd1 && !q_pd.ifault &&", NOCHK_FFW],
   ["      ex_ffw   <= !q_pd.ifault &&", ""], ['core']),
]

VFLAGS = ['--cc', '--exe', '--build', '-j', '8', '-O3', '-Wno-fatal', '-Wno-UNUSEDSIGNAL', '-Wno-DECLFILENAME']

def build_run(d, bench, drop=''):
    rtl = [os.path.join(d, 'rtl', 'cpu', f + '.sv') for f in CPU_RTL]
    sim = os.path.join(ROOT, 'sim')
    if bench in ('core', 'sub'):
        top = 'ppc_core' if bench == 'core' else 'ppc_cpu'
        defs = '' if bench == 'core' else ' -DPPC_CPU'
        obj = os.path.join(d, 'obj_' + bench)
        cmd = ['verilator'] + VFLAGS + ['--top-module', top]
        if bench == 'sub': cmd += ['+define+PPC_CACHE_SIM', '--assert']
        cf = f'-O2 -std=c++17{defs} -DPPC_FPU_RSQRTE_EST -I{sim}/cpu -I{sim}/fpu -I{sim}/cache'
        if drop: cf = cf.replace(' ' + drop, '')
        cmd += ['-CFLAGS', cf,
                '-LDFLAGS', '-lmpfr -lgmp', '--Mdir', obj] + rtl + [os.path.join(sim, 'cpu', 'tb_ppc_core.cpp')]
        exe = os.path.join(obj, 'V' + top)
        runs = [[exe, '400', '1', '1500', '9'], [exe, '200', '7001', '1500', '9', '--rdy', '25']] if bench == 'core' \
               else [[exe, '80', '1', '1500', '80'], [exe, '80', '9001', '1500', '6']]
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
        shutil.copytree(os.path.join(ROOT, 'rtl'), os.path.join(d, 'rtl'))
        drop = ''
        if f == 'cflags':
            drop = old
        else:
            p = os.path.join(d, 'rtl', 'cpu', f)
            s = open(p).read()
            olds, news = (old, new) if isinstance(old, list) else ([old], [new])
            bad = [o for o in olds if s.count(o) != 1]
            if bad:
                print(f'{name}: pattern not found once in {f}: {bad[0][:60]!r}'); continue
            for o, n in zip(olds, news): s = s.replace(o, n)
            open(p, 'w').write(s)
        res = []
        for b in benches:
            res.append(f'{b}: ' + build_run(d, b, drop))
        print(f'{name}: ' + ' | '.join(res), flush=True)
        for b in benches: shutil.rmtree(os.path.join(d, 'obj_' + b), ignore_errors=True)

if __name__ == '__main__':
    main()
