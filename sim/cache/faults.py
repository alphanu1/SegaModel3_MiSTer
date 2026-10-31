#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Planted faults for the cache benches. Each fault is one textual change to a
# COPY of the RTL under sm3-builds/cache_fault/<name>/ (the working tree is
# never touched); the bench is built there and run seed by seed until it
# fails. Reports caught (with the seed and the first failing check) or missed.
#
# usage: sim/cache/faults.py [name ...]     (from the repository root)

import os, re, shutil, subprocess, sys

ROOT = os.getcwd()
OUT = os.path.join(ROOT, 'sm3-builds', 'cache_fault')
SEEDS = 40
OPS = 100000

FAULTS = [
  # name, cache, what it breaks, file, old, new
  ('d_repl_invalid', 'd', 'replacement: victim ignores invalid ways',
   'ppc_dcache.sv', "wire  [1:0]  d_vic    = d_ff[2] ? d_ff[1:0] : plru_victim(dq_st[2:0]);",
   "wire  [1:0]  d_vic    = plru_victim(dq_st[2:0]);"),
  ('d_repl_plru', 'd', 'replacement: PLRU never updated',
   'ppc_dcache.sv', "touch = {x[8:5], w, plru_touch(x[2:0], w)};", "touch = {x[8:5], w, x[2:0]};"),
  ('d_repl_bypass', 'd', 'replacement: a lookup right after a hit misses its state update',
   'ppc_dcache.sv', "wire  [8:0]  st     = (s_v && s_set == r_set) ? s_new : st_ram;", "wire  [8:0]  st     = st_ram;"),
  ('d_wb_order', 'd', 'dirty write-back: beats offset by one word',
   'ppc_dcache.sv', "cb_zero ? 32'd0 : cbm[wb_cnt];", "cb_zero ? 32'd0 : cbm[wb_cnt + 3'd1];"),
  ('d_wb_victim', 'd', 'dirty write-back: a dirty victim is dropped',
   'ppc_dcache.sv', "wire         d_vdirty = dq_st[5 + d_vic] && !dq_inv[d_vic];",
   "wire         d_vdirty = dq_st[5 + d_vic] && dq_inv[d_vic];"),
  ('d_er_word', 'd', 'early restart: the second beat answers the load',
   'ppc_dcache.sv', "wire er_ans   = beat && f_cnt == 3'd0 && f_k == K_LD;",
   "wire er_ans   = beat && f_cnt == 3'd1 && f_k == K_LD;"),
  ('d_er_addr', 'd', 'early restart: fill requested from word 0, not the critical word',
   'ppc_dcache.sv', "m_cmd <= MC_LINE_RD; m_addr <= {fs_a, 2'd0};", "m_cmd <= MC_LINE_RD; m_addr <= {fs_a[31:5], 5'd0};"),
  ('d_sb_fwd', 'd', 'store buffer: a load does not take the pending store\'s bytes',
   'ppc_dcache.sv', "else if (sb_m  && sb_be[3-l])", "else if (1'b0 && sb_m && sb_be[3-l])"),
  ('d_post_chain', 'd', 'store buffer: a store posted behind a write-through store',
   'ppc_dcache.sv', "wire r_to_sb  = lr && r_post && r_k == K_ST && !r_byp && any && !wt;",
   "wire r_to_sb  = lr && r_post && r_k == K_ST && !r_byp && any;"),
  ('d_stale_sbl', 'd', 'stale read: the store written on the read edge is not merged',
   'ppc_dcache.sv', "else if (sbl_m && sbl_be[3-l])", "else if (1'b0 && sbl_m && sbl_be[3-l])"),
  ('d_stale_tag', 'd', 'stale read: tags written on the read edge are trusted',
   'ppc_dcache.sv', "(tw_en_q != 2'd0 && tw_set_q == r_set) || (tw_last && tw_lset == r_set);",
   "(tw_en_q != 2'd0 && tw_set_q == r_set);"),
  ('d_stale_fill', 'd', 'stale read: a load reads the set being filled',
   'ppc_dcache.sv', "(mq_v && mq_a[11:5] == r_set) || (f_act && f_a[11:5] == r_set) ||",
   "(mq_v && mq_a[11:5] == r_set) ||"),
  ('d_stale_dq', 'd', 'stale read: a lookup of the set DQ is deciding goes ahead',
   'ppc_dcache.sv', "                 (dq_v && dq_set == r_set) ||\n", ""),
  ('d_dcbz_dirty', 'd', 'dcbz: a hit is zeroed but not marked dirty',
   'ppc_dcache.sv', "d_st = 1'b1; d_st_new = touch(dq_st, dq_hw); d_st_new[5 + dq_hw] = 1'b1;\n          end else if (!hid_dlock && !dq_rsv) begin",
   "d_st = 1'b1; d_st_new = touch(dq_st, dq_hw);\n          end else if (!hid_dlock && !dq_rsv) begin"),
  ('d_dcbz_word', 'd', 'dcbz: the last word is not zeroed',
   'ppc_dcache.sv', "wire z_wr     = z_act && !z_wcp && !z_cnt[3];", "wire z_wr     = z_act && !z_wcp && !z_cnt[3] && z_cnt != 4'd7;"),
  ('d_dcbz_copy', 'd', 'dcbz: zeroing does not wait for the victim\'s copy',
   'ppc_dcache.sv', "z_act <= 1'b1; z_set <= dq_set; z_way <= op_way; z_cnt <= 4'd0; z_wcp <= do_cp;",
   "z_act <= 1'b1; z_set <= dq_set; z_way <= op_way; z_cnt <= 4'd0; z_wcp <= 1'b0;"),
  ('d_dcbi_wb', 'd', 'dcbi: a dirty line is written back (it must be discarded)',
   'ppc_dcache.sv', "          if (dq_any && !dq_rsv) begin\n            do_tw = 1'b1;\n",
   "          if (dq_any && !dq_rsv) begin\n            do_tw = 1'b1; do_cp = d_hdirty;\n"),
  ('d_dcbi_none', 'd', 'dcbi: the line is not invalidated',
   'ppc_dcache.sv', "          if (dq_any && !dq_rsv) begin\n            do_tw = 1'b1;\n",
   "          if (dq_any && !dq_rsv) begin\n"),
  ('d_dcbst_inval', 'd', 'dcbst: the line is invalidated as well as written back',
   'ppc_dcache.sv', "            do_cp = 1'b1; d_st = 1'b1; d_st_new[5 + dq_hw] = 1'b0;",
   "            do_cp = 1'b1; do_tw = 1'b1; d_st = 1'b1; d_st_new[5 + dq_hw] = 1'b0;"),
  ('d_byp_load', 'd', 'inhibited bypass: an I = 1 access looks at the arrays',
   'ppc_dcache.sv', "r_byp  <= d_wimg[2] || !hid_dce || d_addr[31:12] == RSV_TAG;",
   "r_byp  <= !hid_dce || d_addr[31:12] == RSV_TAG;"),
  ('d_byp_dce', 'd', 'inhibited bypass: HID0[DCE] = 0 still uses the arrays',
   'ppc_dcache.sv', "r_byp  <= d_wimg[2] || !hid_dce || d_addr[31:12] == RSV_TAG;",
   "r_byp  <= d_wimg[2] || d_addr[31:12] == RSV_TAG;"),
  ('d_byp_size', 'd', 'inhibited bypass: a single access made with all four lanes',
   'ppc_dcache.sv', "m_addr <= {uq_a, 2'd0}; m_be <= uq_be;", "m_addr <= {uq_a, 2'd0}; m_be <= 4'hF;"),
  ('d_dlock', 'd', 'HID0[DLOCK]: a miss allocates',
   'ppc_dcache.sv', "            if (hid_dlock) begin l_uq = 1'b1; l_ph2 = 1'b1; end",
   "            if (1'b0) begin l_uq = 1'b1; l_ph2 = 1'b1; end"),
  ('d_wthru', 'd', 'write-through: a store hit is not sent to memory',
   'ppc_dcache.sv', "l_sb = 1'b1; l_uq = wt; l_ans = 1'b1;", "l_sb = 1'b1; l_uq = 1'b0; l_ans = 1'b1;"),
  ('d_unc_overrun', 'd', 'inhibited order: a second single access is taken while one waits',
   'ppc_dcache.sv', "wire dq_free = !dq_v && !mq_v && !uq_v;", "wire dq_free = !dq_v && !mq_v;"),
  ('d_unc_order', 'd', 'inhibited order: a single access overtakes a pending write-back',
   'ppc_dcache.sv', "          if (wb_pend && !cp_act) begin", "          if (wb_pend && !cp_act && !uq_v) begin"),

  ('i_repl_invalid', 'i', 'replacement: victim ignores invalid ways',
   'ppc_icache.sv', "wire  [1:0]  vic  = ff[2] ? ff[1:0] : plru_victim(st[2:0]);", "wire  [1:0]  vic  = plru_victim(st[2:0]);"),
  ('i_repl_cross', 'i', 'replacement: a crossing touches the next set before it is fetched',
   'ppc_icache.sv', "wire kt_use = kt_v && (fast || (serve_r && p_line && !p_ui));",
   "wire kt_use = kt_v;"),
  ('i_er_word', 'i', 'early restart: a waiting fetch takes any arriving beat',
   'ppc_icache.sv', "serve_r = m_rvalid ? (beat && bw == pa[4:2]) : (fb_v && fb_w == pa[4:2]);",
   "serve_r = m_rvalid ? beat : (fb_v && fb_w == pa[4:2]);"),
  ('i_fill_wait', 'i', 'stale read: a word still being filled is served from the array',
   'ppc_icache.sv', "pfw_q    <= start_fill || (f_act && f_line == pa[31:5] && !avail_nx[pa[4:2]]);",
   "pfw_q    <= start_fill;"),
  ('i_er_addr', 'i', 'early restart: fill requested from word 0, not the critical word',
   'ppc_icache.sv', "m_addr <= {pa, 2'd0}; m_be <= 4'hF; m_cmd <= start_fill ? MC_LINE_RD : MC_RD;",
   "m_addr <= start_fill ? {pa[31:5], 5'd0} : {pa, 2'd0}; m_be <= 4'hF; m_cmd <= start_fill ? MC_LINE_RD : MC_RD;"),
  ('i_stale_fill', 'i', 'stale read: a word read on the edge it was written is served',
   'ppc_icache.sv', "end else serve_r = da == pa[11:2] && da_av;", "end else serve_r = da == pa[11:2];"),
  ('i_stale_tag', 'i', 'stale read: tags written on the read edge are trusted',
   'ppc_icache.sv', "wire         t_haz = (tw_en_q != 2'd0 && tw_set_q == ta_idx) || (tw_last && tw_lset == ta_idx) ||",
   "wire         t_haz = (tw_en_q != 2'd0 && tw_set_q == ta_idx) ||"),
  ('i_waypred', 'i', 'way prediction: the MRU way is served without its tag matching',
   'ppc_icache.sv', "if (WAYPRED && hv[st[4:3]] && da == pa[11:2] && da_av) serve = 1'b1;",
   "if (WAYPRED && da == pa[11:2] && da_av) serve = 1'b1;"),
  ('i_byp', 'i', 'inhibited bypass: an I = 1 fetch looks at the arrays',
   'ppc_icache.sv', "(pend_new && (wi_q || pa[31:12] == RSV_TAG));", "(pend_new && pa[31:12] == RSV_TAG);"),
  ('i_byp_late', 'i', 'inhibited bypass: the I bit is taken from the wrong cycle',
   'ppc_icache.sv', "    wi_q     <= i_wimg[2];", "    if (pend) wi_q <= i_wimg[2];"),
  ('i_rsv_page', 'i', 'reserved page: a fetch from RSV_TAG\'s page looks at the arrays',
   'ppc_icache.sv', "(pend_new && (wi_q || pa[31:12] == RSV_TAG));", "(pend_new && wi_q);"),
  ('i_ilock', 'i', 'HID0[ILOCK]: a miss allocates',
   'ppc_icache.sv', "end else if (hid_ilock) to_unc = 1'b1;", "end else if (1'b0) to_unc = 1'b1;"),
  ('i_icbi_half', 'i', 'icbi: only ways 0 and 1 are invalidated',
   'ppc_icache.sv', "tw_en_q <= 2'b11; tw_set_q <= icbi_set;", "tw_en_q <= 2'b01; tw_set_q <= icbi_set;"),
  ('i_look_page', 'i', 'lookahead: the next line is assumed across a page boundary',
   'ppc_icache.sv', "ta_k_v = kl_v && kl[11:5] != 7'd127;", "ta_k_v = kl_v;"),
]

def run(f):
  name, cache, what, fn, old, new = f
  d = os.path.join(OUT, name)
  shutil.rmtree(d, ignore_errors=True)
  os.makedirs(d)
  for x in ('ppc_cache_pkg.sv', 'ppc_cache_ram.sv', 'ppc_icache.sv', 'ppc_dcache.sv'):
    shutil.copy(os.path.join(ROOT, 'rtl', 'cpu', x), d)
  p = os.path.join(d, fn)
  s = open(p).read()
  n = s.count(old)
  if n != 1:
    return (name, what, 'NOT APPLIED (%d matches)' % n)
  open(p, 'w').write(s.replace(old, new))
  top = 'ppc_dcache' if cache == 'd' else 'ppc_icache'
  tb = 'tb_dcache.cpp' if cache == 'd' else 'tb_icache.cpp'
  cmd = ['verilator', '--cc', '--exe', '--build', '-j', '8', '-O3', '--assert', '-Wno-fatal', '-Wno-UNUSEDSIGNAL',
         '-Wno-DECLFILENAME', '+define+PPC_CACHE_SIM', '--top-module', top,
         '-CFLAGS', '-O2 -std=c++17 -I%s/sim/cache' % ROOT, '--Mdir', os.path.join(d, 'obj'),
         os.path.join(d, 'ppc_cache_pkg.sv'), os.path.join(d, 'ppc_cache_ram.sv'), os.path.join(d, fn),
         os.path.join(ROOT, 'sim', 'cache', tb)]
  r = subprocess.run(cmd, capture_output=True, text=True)
  exe = os.path.join(d, 'obj', 'V' + top)
  if not os.path.exists(exe):
    return (name, what, 'BUILD FAILED')
  for seed in range(1, SEEDS + 1):
    r = subprocess.run([exe, str(OPS), str(seed)], capture_output=True, text=True)
    if r.returncode != 0:
      first = next((l for l in r.stdout.splitlines() + r.stderr.splitlines()
                    if re.search(r'MISMATCH|ALLOC|SINGLE|MEMORY|TIMEOUT|Error|spurious', l)), '?')
      shutil.rmtree(os.path.join(d, 'obj'), ignore_errors=True)
      return (name, what, 'caught, seed %d: %s' % (seed, first.strip()[:110]))
  shutil.rmtree(os.path.join(d, 'obj'), ignore_errors=True)
  return (name, what, 'MISSED in %d seeds x %d' % (SEEDS, OPS))

sel = sys.argv[1:]
for f in FAULTS:
  if sel and f[0] not in sel:
    continue
  name, what, res = run(f)
  print('%-15s %-62s %s' % (name, what, res), flush=True)
