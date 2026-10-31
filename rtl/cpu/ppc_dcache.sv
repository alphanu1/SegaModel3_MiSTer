// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// ---------------------------------------------------------------------------
//
// 603e data cache (manual chapter 3; study D-15): 16 KiB, 4 ways, 128 sets of
// 32-byte lines, write-back, write-allocate, critical word first with early
// restart, a store buffer. Physically tagged, indexed by EA[20-26], which
// translation never changes (4 KiB per way), so the arrays are read from the
// core's request as it leaves EX and compared against a registered tag.
//
// CPU side: ppc_core's pipelined protocol ("Memory ports" there). A request
// may be made in any cycle where d_rdy (a register) is high and is taken on
// that edge; every request is answered by one d_valid pulse, in order. Hits
// go at one a cycle: a load hit answers two cycles after its request, a
// store one (posted) or two. Added: d_wimg (the access's WIMG, with d_req),
// d_op (0 load or store, 1 dcbt/dcbtst, 2 dcbz, 3 dcbst, 4 dcbf, 5 dcbi), the
// HID0 bits, d_idle (for sync) and dcfi_ack. d_be must give a load's lanes
// too: a cache-inhibited load is made with exactly those lanes.
//
// Timing (R32): d_req, d_we and d_addr leave the core straight from EX
// translation. Only d_addr[11:2] reaches a RAM, as a read address. Every
// write enable, write address and tag write comes from a register. The
// request is registered (R) and its tags compared in the next cycle (L); a
// load hit is answered from registers two cycles after its request. The
// data output takes the way the set's MRU field predicts, so the compare
// only decides d_valid; a hit in another way answers two cycles later. Misses
// and cache operations are captured raw (tags, invalid ways, set state) and
// decided one cycle after the compare (DQ), from registers.
//
// Pipelining. R takes a new request every cycle its lookup ends. A request
// whose lookup does not end it (a miss waiting for its word, a single read,
// a blocked set, the other way) moves to X, which retries it as R used to:
// X's fields copy R's every cycle X is free, and only x_v waits on the
// compare. A request taken while X waits stays in R (parked) and is read
// again once X is done; d_rdy is low while R is parked, so there are never
// more than two. The lookup is X's when X was read at the last edge, else
// R's, chosen by registers ahead of the compare.
//
// A store is answered the cycle after its request (posted) whenever the cache
// can take whatever its lookup finds: no fill, copy or zeroing in progress,
// and the single-access and miss holders free. A store that hits is written
// from the store buffer (SB) two edges after it was taken; a load that reads
// that word in between takes SB's bytes. A store that misses waits in the
// miss holder (MQ) and is merged after the line arrives.
//
// Misses: the victim is the first invalid way, else the PLRU one. A dirty
// victim is copied to the cast-out buffer while the fill is in flight (the
// copy reads each word at least two edges before its beat is written) and is
// written back after the fill (manual 8.1.1). Only the set being filled is
// blocked; hits to other sets go on (the 603e blocks the whole cache, 3.2.2;
// R22's CPI model assumes the remainder of a fill is hidden). One memory
// transaction at a time, in order: write-back first, then the miss or the
// single access, which are never pending together.
//
// Cache-inhibited (I = 1), cache disabled (HID0[DCE] = 0, table 2-2) and a
// miss under HID0[DLOCK] (3.2.3.3) are single accesses of the requested lanes,
// in order, never looking at the arrays. Write-through stores (W = 1, 3.5.1)
// update a line that hits, leave its dirty bit alone, and go to memory; one
// that misses does not allocate (table 3-8).
//
// Tag RAM: two M10Ks, each a pair of 20-bit tags per set. There is no valid
// bit: a line is invalid when its tag is RSV_TAG, a page no access is cached
// from (an access to it bypasses the arrays). The dirty bits, MRU and PLRU of
// a set are one MLAB word. Reset and HID0[DCFI] sweep the tags, one set a
// cycle, 128 cycles; nothing is cleared in reset.

module ppc_dcache
  import ppc_cache_pkg::*;
#(
  // a page in Model 3's I/O space (always mapped I = 1): its accesses are single
  parameter logic [19:0] RSV_TAG = 20'hF00FF
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        d_req,
  input  logic        d_we,
  input  logic [31:0] d_addr,
  input  logic [3:0]  d_be,
  input  logic [31:0] d_wdata,
  input  logic [3:0]  d_wimg,       // W I M G
  input  logic [2:0]  d_op,
  output logic        d_rdy,        // a request may be made this cycle (a register)
  output logic        d_valid,
  output logic [31:0] d_rdata,

  input  logic        hid_dce,
  input  logic        hid_dlock,
  input  logic        hid_dcfi,
  input  logic        hid_noopti,
  output logic        dcfi_ack,     // the flash invalidate began: the core clears HID0[DCFI]
  output logic        d_idle,       // nothing posted, buffered or in flight

  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  output logic [31:0] m_wdata,
  input  logic        m_ack,
  input  logic        m_wnext,
  input  logic        m_rvalid,
  input  logic [31:0] m_rdata,

  // verification: a line allocated (fill or dcbz), as it is decided
  output logic        t_alloc,
  output logic [6:0]  t_alloc_set,
  output logic [1:0]  t_alloc_way,
  output logic [19:0] t_alloc_tag
);

  localparam logic [2:0] K_LD = 3'd0, K_ST = 3'd1, K_TCH = 3'd2, K_DCBZ = 3'd3,
                         K_DCBST = 3'd4, K_DCBF = 3'd5, K_DCBI = 3'd6;

  // ================================================================ arrays
  logic [9:0]       rd_a;                       // {set, word}
  logic [9:0]       wr_a;
  logic [31:0]      wr_d;
  logic [3:0][3:0]  wr_en;                      // [way][lane], lane 0 = bits 31:24
  logic [3:0][31:0] dq;                         // each way's word at rd_a
  genvar gw, gl;
  generate
    for (gw = 0; gw < 4; gw++) begin : g_way
      for (gl = 0; gl < 4; gl++) begin : g_lane
        ppc_cache_ram #(.W(8), .D(1024)) u_d (
          .clk (clk), .we (wr_en[gw][gl]), .wa (wr_a), .wd (wr_d[31-8*gl -: 8]),
          .ra (rd_a), .q (dq[gw][31-8*gl -: 8]));
      end
    end
  endgenerate

  // tags: pair p holds way 2p in [39:20], way 2p+1 in [19:0]
  logic [1:0]  tw_en_q;                         // a tag write registered, made at the next edge
  logic [6:0]  tw_set_q;
  logic [79:0] tw_d_q;                          // pair 1 [79:40], pair 0 [39:0]
  logic [39:0] tq0, tq1;
  ppc_cache_ram #(.W(40), .D(128)) u_t0 (.clk (clk), .we (tw_en_q[0]), .wa (tw_set_q), .wd (tw_d_q[39:0]),
                                         .ra (rd_a[9:3]), .q (tq0));
  ppc_cache_ram #(.W(40), .D(128)) u_t1 (.clk (clk), .we (tw_en_q[1]), .wa (tw_set_q), .wd (tw_d_q[79:40]),
                                         .ra (rd_a[9:3]), .q (tq1));
  logic [3:0][19:0] tg;
  assign tg = {tq1[19:0], tq1[39:20], tq0[19:0], tq0[39:20]};

  // set state, {dirty[3:0], mru[1:0], plru[2:0]}: written as presented, read
  // asynchronously at R's set
  (* ramstyle = "MLAB, no_rw_check" *) logic [8:0] stm [0:127];
  logic        st_we;
  logic [6:0]  st_wa;
  logic [8:0]  st_wd;
  always_ff @(posedge clk) if (st_we) stm[st_wa] <= st_wd;

  // cast-out buffer
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] cbm [0:7];
  logic        cp_dv;
  logic [2:0]  cp_dw;
  logic [1:0]  cp_way;
  always_ff @(posedge clk) if (cp_dv) cbm[cp_dw] <= dq[cp_way];

  // ================================================================ state
  // R: the request taken last (fresh: its arrays were read at the last edge)
  logic        r_v, r_fresh, r_post, r_dce;
  logic [2:0]  r_k;
  logic [31:2] r_a;
  logic [3:0]  r_be, r_wimg;
  logic [31:0] r_wd;
  // X: the oldest request, whose lookup did not end it
  logic        x_v, x_byp, x_rsv;
  logic [1:0]  x_ph;                            // 0 look up, 1 wait for the fill's word, 2 wait for a single read
  logic [2:0]  x_k;
  logic [31:2] x_a;
  logic [3:0]  x_be, x_wimg;
  logic [31:0] x_wd;
  logic        x_rwv;                           // the hit way is known (a misprediction)
  logic [1:0]  x_rw;
  logic        l_x;                             // the last edge read X's set
  logic        d_rdy_q;

  // MQ: a miss waiting for the memory port
  logic        mq_v, mq_vdirty;
  logic [2:0]  mq_k;
  logic [31:2] mq_a;
  logic [3:0]  mq_be;
  logic [31:0] mq_wd;
  logic [1:0]  mq_way;
  logic [19:0] mq_vtag, mq_otag;                // the victim's tag, its pair partner's

  // FU: the fill
  logic        f_act, f_wb;
  logic [1:0]  f_ph;                            // 0 beats, 1-3 the end (merge a store, then visible)
  logic [2:0]  f_k;
  logic [31:2] f_a;
  logic [3:0]  f_be;
  logic [31:0] f_wd;
  logic [1:0]  f_way;
  logic [2:0]  f_cnt;
  logic        fb_v;                            // a beat registered, written at the next edge
  logic [2:0]  fb_w;
  logic [31:0] fb_d;

  // CU: a line copied to the cast-out buffer
  logic        cp_act, cp_wb;
  logic [6:0]  cp_set;
  logic [2:0]  cp_base;
  logic [3:0]  cp_cnt;

  // ZU: dcbz's zero writes
  logic        z_act, z_wcp;
  logic [6:0]  z_set;
  logic [1:0]  z_way;
  logic [3:0]  z_cnt;

  // SB: a store that hit, written at the next edge; sbl: the one written last edge
  logic        sb_v, sbl_v;
  logic [31:2] sb_a, sbl_a;
  logic [1:0]  sb_way;
  logic [3:0]  sb_be, sbl_be;
  logic [31:0] sb_wd, sbl_wd;

  // UQ: a single access
  logic        uq_v, uq_we;
  logic [31:2] uq_a;
  logic [3:0]  uq_be;
  logic [31:0] uq_wd;

  // write-back
  logic        wb_pend, cb_zero;
  logic [31:5] wb_a;
  logic [2:0]  wb_cnt;

  // tag write last edge (its set is not readable yet)
  logic        tw_last;
  logic [6:0]  tw_lset;

  // sweep
  logic        wk_act, wk_req, dcfi_q;
  logic [7:0]  wk_cnt;

  // memory port
  typedef enum logic [2:0] { MP_IDLE, MP_RQ, MP_FILL, MP_WB, MP_SRD } mp_t;
  mp_t         mp;

  // ================================================================ lookup (L)
  // The cycle after R's read compares the tags. Only three things act on the
  // compare here: a load hit's answer, a store hit going into SB, and the way
  // of a hit the MRU did not predict. Misses and cache operations are taken
  // raw (tags, invalid ways, state) into DQ and decided in the next cycle
  // from registers: the compare follows an M10K's clock-to-out, and a victim
  // choice or a tag-pair write after it did not close at 120 MHz.
  // the request looked up: X's while X is held (it is read every cycle it
  // can be looked up), else a fresh R's; all from registers
  wire         lx    = x_v && l_x;
  wire         lr    = !x_v && r_v && r_fresh;
  wire         l_v   = lx || lr;
  wire         l_post = !x_v && r_post;
  wire  [2:0]  r_k_l = x_v ? x_k : r_k;
  wire  [31:2] r_a_l = x_v ? x_a : r_a;
  wire  [3:0]  r_be_l = x_v ? x_be : r_be;
  wire  [3:0]  r_wimg_l = x_v ? x_wimg : r_wimg;
  wire  [31:0] r_wd_l = x_v ? x_wd : r_wd;
  // R's bypass and reserved-page bits, formed from R's registers in the
  // lookup cycle: the request's address and WIMG come from the core's EX
  // translation, late in their cycle, and the compare after them did not
  // close in the full build
  wire         r_rsv   = r_a[31:12] == RSV_TAG;
  wire         r_byp   = r_wimg[2] || !r_dce || r_rsv;
  wire         r_byp_l = x_v ? x_byp : r_byp;
  wire         r_rsv_l = x_v ? x_rsv : r_rsv;
  wire  [1:0]  r_ph_l = x_v ? x_ph : 2'd0;
  wire         rw_v_l = x_v && x_rwv;
  wire  [6:0]  r_set = r_a_l[11:5];
  wire  [19:0] r_tag = r_a_l[31:12];

  // a hit's state update is registered (s_*) and written in the next cycle;
  // a lookup of that set in that cycle takes the new value
  logic        s_v, s_dirty;
  logic [6:0]  s_set;
  logic [1:0]  s_way;
  logic [8:0]  s_st;
  function automatic logic [8:0] touch(input logic [8:0] x, input logic [1:0] w);
    touch = {x[8:5], w, plru_touch(x[2:0], w)};
  endfunction
  wire  [8:0]  s_new = touch(s_st, s_way) | ({8'd0, s_dirty} << (5 + s_way));
  wire  [8:0]  st_ram = stm[r_set];
  wire  [8:0]  st     = (s_v && s_set == r_set) ? s_new : st_ram;
  wire  [1:0]  st_mru = st[4:3];

  logic [3:0]  hv, inv;
  always_comb
    for (int w = 0; w < 4; w++) begin
      hv[w]  = tg[w] == r_tag;
      inv[w] = tg[w] == RSV_TAG;
    end
  wire         any    = |hv;
  wire  [1:0]  hw     = enc4(hv);
  wire  [1:0]  pw     = rw_v_l ? x_rw : st_mru; // the way the data output takes
  wire         hp     = hv[pw];
  wire         wt     = r_wimg_l[3];

  // DQ: a miss or a cache operation, decided the cycle after its lookup
  logic        dq_v, dq_any, dq_rsv;
  logic [2:0]  dq_k;
  logic [31:2] dq_a;
  logic [3:0]  dq_be, dq_inv;
  logic [31:0] dq_wd;
  logic [1:0]  dq_hw;
  logic [8:0]  dq_st;
  logic [3:0][19:0] dq_tg;
  wire  [6:0]  dq_set = dq_a[11:5];

  wire mp_idle = (mp == MP_IDLE);
  wire quiet   = !dq_v && !mq_v && !f_act && !cp_act && !z_act && !wb_pend && !uq_v && mp_idle && !sb_v &&
                 tw_en_q == 2'd0;
  // the write port is free for SB: no fill or zeroing, and none about to
  // start (a dcbz in DQ starts ZU at the next edge, when SB would write)
  wire wp_free = !f_act && !z_act && !(dq_v && dq_k == K_DCBZ);
  wire dq_free = !dq_v && !mq_v && !uq_v;       // a miss or a single access can be taken
  wire blk     = wk_act ||
                 (dq_v && dq_set == r_set) ||
                 (mq_v && mq_a[11:5] == r_set) || (f_act && f_a[11:5] == r_set) ||
                 (cp_act && cp_set == r_set) || (z_act && z_set == r_set) ||
                 (tw_en_q != 2'd0 && tw_set_q == r_set) || (tw_last && tw_lset == r_set);
  wire lk      = l_v && r_ph_l == 2'd0 && !blk;

  logic l_ans, l_sb, l_uq, l_dq, l_rw, l_ph1, l_ph2, l_st, l_dirty;
  always_comb begin
    l_ans = 1'b0; l_sb = 1'b0; l_uq = 1'b0; l_dq = 1'b0; l_rw = 1'b0; l_ph1 = 1'b0; l_ph2 = 1'b0;
    l_st = 1'b0; l_dirty = 1'b0;
    if (lk) begin
      case (r_k_l)
        K_LD:
          if (r_byp_l) begin
            if (dq_free) begin l_uq = 1'b1; l_ph2 = 1'b1; end
          end else if (any) begin
            l_st = 1'b1;
            if (hp) l_ans = 1'b1; else l_rw = 1'b1;
          end else if (dq_free) begin
            if (hid_dlock) begin l_uq = 1'b1; l_ph2 = 1'b1; end
            else           begin l_dq = 1'b1; l_ph1 = 1'b1; end
          end
        K_ST:
          if (r_byp_l) begin
            if (dq_free) begin l_uq = 1'b1; l_ans = 1'b1; end
          end else if (any) begin
            if (wp_free && (!wt || dq_free)) begin
              l_sb = 1'b1; l_uq = wt; l_ans = 1'b1; l_st = 1'b1; l_dirty = !wt;
            end
          end else if (dq_free) begin
            l_ans = 1'b1;
            if (wt || hid_dlock) l_uq = 1'b1; else l_dq = 1'b1;
          end
        K_TCH:                                  // 3.2.4, 3.7.2: no-op if locked, disabled, W, I or G
          if (hid_noopti || !hid_dce || hid_dlock || r_byp_l || r_wimg_l[3] || r_wimg_l[0]) l_ans = 1'b1;
          else if (any) begin l_st = 1'b1; l_ans = 1'b1; end
          else if (dq_free) begin l_dq = 1'b1; l_ans = 1'b1; end
        K_DCBZ:                                 // 3.7.4; I or W is the core's alignment exception
          if (r_wimg_l[2] || r_wimg_l[3]) l_ans = 1'b1;
          else if (quiet) begin l_dq = 1'b1; l_ans = 1'b1; end
        K_DCBST, K_DCBF, K_DCBI:
          if (quiet) begin l_dq = 1'b1; l_ans = 1'b1; end
        default: l_ans = 1'b1;
      endcase
    end
  end

  // ================================================================ decision (D)
  wire  [2:0]  d_ff     = first_free(dq_inv);
  wire  [1:0]  d_vic    = d_ff[2] ? d_ff[1:0] : plru_victim(dq_st[2:0]);
  wire         d_vdirty = dq_st[5 + d_vic] && !dq_inv[d_vic];
  wire         d_hdirty = dq_st[5 + dq_hw];
  wire         d_alloc  = dq_v && (dq_k == K_LD || dq_k == K_ST || dq_k == K_TCH);

  logic        do_cp, do_z, do_tw, do_cbz, cp_from_vic, d_st;
  logic [1:0]  op_way;                          // way for CU / ZU / tag write
  logic [19:0] tw_tag;                          // the tag written to op_way
  logic [8:0]  d_st_new;
  always_comb begin
    do_cp = 1'b0; do_z = 1'b0; do_tw = 1'b0; do_cbz = 1'b0; cp_from_vic = 1'b0; d_st = 1'b0;
    op_way = dq_hw; tw_tag = RSV_TAG; d_st_new = dq_st;
    if (dq_v) begin
      case (dq_k)
        K_LD, K_TCH: begin
          d_st = 1'b1; d_st_new = touch(dq_st, d_vic); d_st_new[5 + d_vic] = 1'b0;
        end
        K_ST: begin
          d_st = 1'b1; d_st_new = touch(dq_st, d_vic); d_st_new[5 + d_vic] = 1'b1;
        end
        K_DCBZ:
          if (dq_any && !dq_rsv) begin
            do_z = 1'b1;
            d_st = 1'b1; d_st_new = touch(dq_st, dq_hw); d_st_new[5 + dq_hw] = 1'b1;
          end else if (!hid_dlock && !dq_rsv) begin
            op_way = d_vic; do_z = 1'b1; do_tw = 1'b1; tw_tag = dq_a[31:12];
            do_cp = d_vdirty; cp_from_vic = 1'b1;
            d_st = 1'b1; d_st_new = touch(dq_st, d_vic); d_st_new[5 + d_vic] = 1'b1;
          end else
            do_cbz = 1'b1;                      // locked (or the bypass page): zeros to memory, no allocation
        K_DCBST:                                // 3.7.5
          if (dq_any && !dq_rsv && d_hdirty) begin
            do_cp = 1'b1; d_st = 1'b1; d_st_new[5 + dq_hw] = 1'b0;
          end
        K_DCBF:                                 // 3.7.6
          if (dq_any && !dq_rsv) begin
            do_cp = d_hdirty; do_tw = 1'b1;
            d_st = 1'b1; d_st_new[5 + dq_hw] = 1'b0;
          end
        K_DCBI:                                 // 3.7.1
          if (dq_any && !dq_rsv) begin
            do_tw = 1'b1;
            d_st = 1'b1; d_st_new[5 + dq_hw] = 1'b0;
          end
        default: ;
      endcase
    end
  end

  // ============================================================ posting
  // A store is answered at once when whatever its lookup finds can be taken.
  // R may hold the previous posted store, being absorbed now: only when that
  // one goes to SB is the next one safe. Nothing else may be in R or X: its
  // answer could come in the same cycle as the posted one.
  wire r_to_sb  = lr && r_post && r_k == K_ST && !r_byp && any && !wt;
  wire base_ok  = !wk_act && !wk_req && !dq_v && !mq_v && !f_act && !cp_act && !z_act && !uq_v &&
                  tw_en_q == 2'd0 && !rst;
  wire post     = d_req && d_we && d_op == 3'd0 && base_ok && !x_v && (!r_v || r_to_sb);

  // ============================================================ the fill
  // from MQ (older), or straight from DQ when the port is free
  wire fill_free = mp_idle && !wb_pend && !f_act && !cp_act && !z_act && !uq_v;
  wire fill_go   = fill_free && (mq_v || d_alloc);
  wire fs_mq     = mq_v;                        // the fill's source
  wire [31:2]  fs_a     = fs_mq ? mq_a     : dq_a;
  wire [1:0]   fs_way   = fs_mq ? mq_way   : d_vic;
  wire         fs_vd    = fs_mq ? mq_vdirty : d_vdirty;
  wire [19:0]  fs_vtag  = fs_mq ? mq_vtag  : dq_tg[d_vic];
  wire [19:0]  fs_otag  = fs_mq ? mq_otag  : dq_tg[d_vic ^ 2'd1];
  wire [2:0]   fs_k     = fs_mq ? mq_k     : dq_k;
  wire [3:0]   fs_be    = fs_mq ? mq_be    : dq_be;
  wire [31:0]  fs_wd    = fs_mq ? mq_wd    : dq_wd;
  wire beat     = (mp == MP_FILL) && m_rvalid;
  wire er_ans   = beat && f_cnt == 3'd0 && f_k == K_LD;      // the critical word, as it arrives
  wire srd_ans  = (mp == MP_SRD) && m_rvalid;
  wire cp_rd    = cp_act && !cp_cnt[3];
  wire z_wr     = z_act && !z_wcp && !z_cnt[3];

  // ============================================================ read port
  // By registers alone: the cast-out copy, else X while it can be looked up,
  // else a parked R (also while X waits for memory, so R is fresh when X is
  // answered), else the core's new address.
  localparam logic [1:0] RD_NEW = 2'd0, RD_X = 2'd1, RD_R = 2'd2, RD_CP = 2'd3;
  wire  [1:0]  rd_sel = cp_rd ? RD_CP : (x_v && x_ph == 2'd0) ? RD_X : (r_v && !(r_fresh && !x_v)) ? RD_R : RD_NEW;
  always_comb
    case (rd_sel)
      RD_CP:   rd_a = {cp_set, cp_base + cp_cnt[2:0]};
      RD_X:    rd_a = x_a[11:2];
      RD_R:    rd_a = r_a[11:2];
      default: rd_a = d_addr[11:2];
    endcase

  // ============================================================ write port
  always_comb begin
    wr_a = 10'd0; wr_d = 32'd0; wr_en = '0;
    if (fb_v) begin
      wr_a = {f_a[11:5], fb_w}; wr_d = fb_d; wr_en[f_way] = 4'hF;
    end else if (sb_v) begin
      wr_a = sb_a[11:2]; wr_d = sb_wd;
      wr_en[sb_way] = {sb_be[0], sb_be[1], sb_be[2], sb_be[3]};
    end else if (z_wr) begin
      wr_a = {z_set, z_cnt[2:0]}; wr_en[z_way] = 4'hF;
    end
  end

  // ============================================================ state write
  // one a cycle: the sweep, a hit's (registered), or DQ's decision; DQ's set
  // is blocked to lookups while it is decided, so it needs no forwarding
  always_comb begin
    st_we = 1'b0; st_wa = s_set; st_wd = s_new;
    if (wk_act)    begin st_we = 1'b1; st_wa = wk_cnt[6:0]; st_wd = 9'd0; end
    else if (s_v)  st_we = 1'b1;
    else if (d_st) begin st_we = 1'b1; st_wa = dq_set; st_wd = d_st_new; end
  end

  // ============================================================ output
  // byte sources: the store just written, the store about to be, the word
  wire sb_m  = sb_v  && sb_a  == r_a_l;
  wire sbl_m = sbl_v && sbl_a == r_a_l;
  logic [31:0] ld_word;
  always_comb
    for (int l = 0; l < 4; l++) begin
      if (er_ans || srd_ans)         ld_word[31-8*l -: 8] = m_rdata[31-8*l -: 8];
      else if (sb_m  && sb_be[3-l])  ld_word[31-8*l -: 8] = sb_wd[31-8*l -: 8];
      else if (sbl_m && sbl_be[3-l]) ld_word[31-8*l -: 8] = sbl_wd[31-8*l -: 8];
      else                           ld_word[31-8*l -: 8] = dq[pw][31-8*l -: 8];
    end

  always_ff @(posedge clk) begin
    d_valid <= !rst && (post || (l_ans && !l_post) || er_ans || srd_ans);
    d_rdata <= ld_word;
  end

  // ============================================================ R, X
  // Only the valid bits (and X's phase and known way) wait on the compare.
  wire l_end  = (lk && l_ans) || l_post;        // the lookup ends its request
  wire x_end  = x_v && ((lx && lk && l_ans) || er_ans || srd_ans);
  wire x_take = lr && !l_end;                   // R's lookup did not end it: to X
  wire r_v_n     = d_req || (r_v && !lr);
  wire r_fresh_n = d_req ? rd_sel == RD_NEW : rd_sel == RD_R;
  wire x_v_n     = x_v ? !x_end : x_take;
  always_ff @(posedge clk) begin
    l_x <= rd_sel == RD_X;
    if (rst) begin
      r_v <= 1'b0; r_fresh <= 1'b0; r_post <= 1'b0; x_v <= 1'b0; d_rdy_q <= 1'b0;
    end else begin
      r_v     <= r_v_n;
      r_fresh <= r_fresh_n;
      x_v     <= x_v_n;
      // the next cycle's d_rdy: R will be free at its end (empty, or looked up)
      d_rdy_q <= !r_v_n || (r_fresh_n && !x_v_n);
      if (d_req) r_post <= post;
      else if (lr) r_post <= 1'b0;
    end
    // R's fields load whenever a request may be made (d_rdy, a register): R
    // is then free at this edge (empty, or looked up now and copied to X if
    // its lookup does not end it), and d_req, late from EX, enables nothing
    if (d_rdy_q) begin
      r_k    <= (d_op == 3'd0) ? (d_we ? K_ST : K_LD) :
                (d_op == 3'd1) ? K_TCH : (d_op == 3'd2) ? K_DCBZ :
                (d_op == 3'd3) ? K_DCBST : (d_op == 3'd4) ? K_DCBF : K_DCBI;
      r_a    <= d_addr[31:2];
      r_be   <= d_be;
      r_wd   <= d_wdata;
      r_wimg <= d_wimg;
      r_dce  <= hid_dce;
    end
    if (!x_v) begin                             // X copies R while free
      x_k <= r_k; x_a <= r_a; x_be <= r_be; x_wd <= r_wd; x_wimg <= r_wimg; x_rsv <= r_rsv; x_byp <= r_byp;
      x_ph  <= l_ph1 ? 2'd1 : l_ph2 ? 2'd2 : 2'd0;
      x_rwv <= l_rw;
      x_rw  <= hw;
    end else if (lx) begin
      if (l_ph1) x_ph <= 2'd1;
      if (l_ph2) x_ph <= 2'd2;
      if (l_rw) x_rwv <= 1'b1;
      if (lk) x_rw <= hw;                       // used only with x_rwv, and the same on a retry
    end
  end
  assign d_rdy = d_rdy_q;

  // ============================================================ the rest
  always_ff @(posedge clk) begin
    t_alloc <= 1'b0;
    dcfi_ack <= 1'b0;
    fb_v    <= 1'b0;
    sbl_v   <= sb_v;
    sbl_a   <= sb_a; sbl_be <= sb_be; sbl_wd <= sb_wd;
    tw_last <= tw_en_q != 2'd0;
    tw_lset <= tw_set_q;
    tw_en_q <= 2'd0;
    cp_dv   <= cp_rd;
    cp_dw   <= cp_base + cp_cnt[2:0];
    dcfi_q  <= hid_dcfi;
    // L's registered outcomes
    s_v     <= l_st && !rst;
    s_set   <= r_set; s_st <= st; s_way <= hw; s_dirty <= l_dirty;
    dq_v    <= l_dq && !rst;
    dq_k    <= r_k_l; dq_a <= r_a_l; dq_be <= r_be_l; dq_wd <= r_wd_l; dq_rsv <= r_rsv_l;
    dq_any  <= any; dq_hw <= hw; dq_inv <= inv; dq_st <= st; dq_tg <= tg;

    if (rst) begin
      mq_v <= 1'b0; f_act <= 1'b0; cp_act <= 1'b0; z_act <= 1'b0; sb_v <= 1'b0; sbl_v <= 1'b0;
      uq_v <= 1'b0; wb_pend <= 1'b0; cb_zero <= 1'b0; mp <= MP_IDLE; m_req <= 1'b0;
      wk_act <= 1'b1; wk_cnt <= 8'd0; wk_req <= 1'b0; tw_last <= 1'b0; cp_dv <= 1'b0;
    end else begin
      // ---- sweep (reset, DCFI with the cache enabled: table 2-2)
      if (hid_dcfi && !dcfi_q && hid_dce) wk_req <= 1'b1;
      if (wk_act) begin
        tw_en_q  <= 2'b11;
        tw_set_q <= wk_cnt[6:0];
        tw_d_q   <= {4{RSV_TAG}};
        wk_cnt   <= wk_cnt + 8'd1;
        if (wk_cnt == 8'd127) wk_act <= 1'b0;
      end else if (wk_req && quiet && !r_v && !x_v) begin
        wk_req <= 1'b0; wk_act <= 1'b1; wk_cnt <= 8'd0; dcfi_ack <= 1'b1;
      end

      // ---- SB (its fields load every cycle; only sb_v waits on the compare)
      sb_v <= l_sb || (f_act && f_ph == 2'd1 && f_k == K_ST);
      if (f_act && f_ph == 2'd1) begin
        sb_a <= f_a; sb_way <= f_way; sb_be <= f_be; sb_wd <= f_wd;
      end else begin
        sb_a <= r_a_l; sb_way <= hw; sb_be <= r_be_l; sb_wd <= r_wd_l;
      end

      // ---- UQ (likewise, while it is free)
      if (l_uq) uq_v <= 1'b1;
      if (!uq_v) begin
        uq_we <= (r_k_l == K_ST); uq_a <= r_a_l; uq_be <= r_be_l; uq_wd <= r_wd_l;
      end

      // ---- an allocation, in program order (the fill may start later)
      if (d_alloc || (do_tw && cp_from_vic)) begin
        t_alloc <= 1'b1; t_alloc_set <= dq_set; t_alloc_way <= d_vic; t_alloc_tag <= dq_a[31:12];
      end
      if (d_alloc && !fill_go) begin
        mq_v <= 1'b1; mq_k <= dq_k; mq_a <= dq_a; mq_be <= dq_be; mq_wd <= dq_wd;
        mq_way <= d_vic; mq_vtag <= dq_tg[d_vic]; mq_vdirty <= d_vdirty; mq_otag <= dq_tg[d_vic ^ 2'd1];
      end

      // ---- tag writes decided by DQ (cache operations)
      if (do_tw) begin
        tw_en_q  <= op_way[1] ? 2'b10 : 2'b01;
        tw_set_q <= dq_set;
        tw_d_q   <= {2{op_way[0] ? {dq_tg[op_way ^ 2'd1], tw_tag} : {tw_tag, dq_tg[op_way ^ 2'd1]}}};
      end

      // ---- CU
      if (do_cp) begin
        cp_act <= 1'b1; cp_wb <= 1'b1; cp_set <= dq_set; cp_way <= op_way; cp_base <= 3'd0; cp_cnt <= 4'd0;
        wb_a   <= {cp_from_vic ? dq_tg[op_way] : dq_a[31:12], dq_set};
      end else if (fill_go && fs_vd) begin
        cp_act <= 1'b1; cp_wb <= 1'b0; cp_set <= fs_a[11:5]; cp_way <= fs_way; cp_base <= fs_a[4:2];
        cp_cnt <= 4'd0;
        wb_a   <= {fs_vtag, fs_a[11:5]};
      end else if (cp_act) begin
        if (cp_rd) cp_cnt <= cp_cnt + 4'd1;
        else if (cp_dv) begin                   // the last word written to the buffer this edge
          cp_act <= 1'b0;
          if (cp_wb) wb_pend <= 1'b1;
        end
      end

      // ---- ZU
      if (do_z) begin
        z_act <= 1'b1; z_set <= dq_set; z_way <= op_way; z_cnt <= 4'd0; z_wcp <= do_cp;
      end else if (z_act) begin
        if (z_wcp) z_wcp <= cp_act;
        else if (!z_cnt[3]) z_cnt <= z_cnt + 4'd1;
        else z_act <= 1'b0;                     // the last zero was written an edge ago
      end
      if (do_cbz) begin
        wb_pend <= 1'b1; cb_zero <= 1'b1; wb_a <= dq_a[31:5];
      end

      // ---- FU
      if (fill_go) begin
        if (fs_mq) mq_v <= 1'b0;
        f_act <= 1'b1; f_ph <= 2'd0; f_cnt <= 3'd0;
        f_k   <= fs_k; f_a <= fs_a; f_be <= fs_be; f_wd <= fs_wd; f_way <= fs_way; f_wb <= fs_vd;
        tw_en_q  <= fs_way[1] ? 2'b10 : 2'b01;
        tw_set_q <= fs_a[11:5];
        tw_d_q   <= {2{fs_way[0] ? {fs_otag, fs_a[31:12]} : {fs_a[31:12], fs_otag}}};
      end else if (f_act) begin
        if (beat) begin
          fb_v <= 1'b1; fb_w <= f_a[4:2] + f_cnt; fb_d <= m_rdata;
          f_cnt <= f_cnt + 3'd1;
          if (f_cnt == 3'd7) f_ph <= 2'd1;
        end else if (f_ph != 2'd0) begin
          // 1: last beat writes next edge, a store is merged after it;
          // 2: last write lands; 3: visible to a read from the next edge
          if (f_ph == 2'd3) begin
            f_act <= 1'b0;
            if (f_wb) wb_pend <= 1'b1;
          end else f_ph <= f_ph + 2'd1;
        end
      end

      // ---- memory port
      case (mp)
        MP_IDLE: begin
          if (wb_pend && !cp_act) begin
            mp <= MP_RQ; m_req <= 1'b1; m_cmd <= MC_LINE_WR; m_addr <= {wb_a, 5'd0}; m_be <= 4'hF;
            wb_cnt <= 3'd0;
          end else if (fill_go) begin
            mp <= MP_RQ; m_req <= 1'b1; m_cmd <= MC_LINE_RD; m_addr <= {fs_a, 2'd0}; m_be <= 4'hF;
          end else if (uq_v && !f_act) begin
            mp <= MP_RQ; m_req <= 1'b1; m_cmd <= uq_we ? MC_WR : MC_RD; m_addr <= {uq_a, 2'd0}; m_be <= uq_be;
          end
        end
        MP_RQ: if (m_ack) begin
          m_req <= 1'b0;
          case (m_cmd)
            MC_LINE_WR: mp <= MP_WB;
            MC_LINE_RD: mp <= MP_FILL;
            MC_RD:      mp <= MP_SRD;
            default: begin mp <= MP_IDLE; uq_v <= 1'b0; end
          endcase
        end
        MP_WB: if (m_wnext) begin
          wb_cnt <= wb_cnt + 3'd1;
          if (wb_cnt == 3'd7) begin mp <= MP_IDLE; wb_pend <= 1'b0; cb_zero <= 1'b0; end
        end
        MP_FILL: if (m_rvalid && f_cnt == 3'd7) mp <= MP_IDLE;
        MP_SRD: if (m_rvalid) begin mp <= MP_IDLE; uq_v <= 1'b0; end
        default: mp <= MP_IDLE;
      endcase
    end
  end

  assign m_wdata = (m_cmd == MC_WR) ? uq_wd : cb_zero ? 32'd0 : cbm[wb_cnt];
  assign d_idle  = !r_v && !x_v && quiet && !wk_act && !wk_req && !fb_v;

`ifdef PPC_CACHE_SIM
  // the protocol and the single write port, as the design relies on them
  logic [31:0] sim_cyc = 0;
  always_ff @(posedge clk) sim_cyc <= sim_cyc + 1;
  always_ff @(posedge clk) if (!rst) begin
    if (d_req && !d_rdy) $error("ppc_dcache: request with d_rdy low (cycle %0d)", sim_cyc);
    if (lr && r_post && !(lk && l_ans))
      $error("ppc_dcache: posted store not absorbed (cycle %0d, blk %b k %0d)", sim_cyc, blk, r_k);
    if (fb_v && (sb_v || z_wr)) $error("ppc_dcache: write port conflict (cycle %0d sb %b z %b)", sim_cyc, sb_v, z_wr);
    if (sb_v && z_wr) $error("ppc_dcache: write port conflict sb/z (cycle %0d)", sim_cyc);
    if (s_v && d_st) $error("ppc_dcache: two state writes (cycle %0d)", sim_cyc);
  end
`endif

endmodule
