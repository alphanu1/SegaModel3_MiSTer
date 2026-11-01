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
// 603e instruction cache (manual 3.1; study D-15): 16 KiB, 4 ways, 128 sets
// of 32-byte lines, physically tagged, filled critical word first; the
// waiting word and every later word of the line are forwarded as they arrive
// (3.1.2), and other lines hit while the fill runs.
//
// CPU side: ppc_core's fetch protocol (i_req / i_addr, answered by a one-cycle
// i_valid with i_data, one request outstanding). i_data and i_valid are
// registers (R28: predecode at fetch cannot follow an M10K's clock-to-out).
// Added: i_wimg (the fetch's WIMG; only I is used, 3.5.3), i_seq, the HID0
// bits, icbi, i_idle and icfi_ack.
//
// i_seq: the core asserts it when i_addr is the previous request's + 4 under
// the same translation, so with the same WIMG (it may also be asserted across
// a 4 KiB page boundary; the cache does not trust it there). The core's i_addr[31:12] comes through fetch translation, so it
// is never compared in the request's cycle: a sequential request is answered
// from the predicted next word (nx), whose data and line are already known,
// and every other request is registered and compared a cycle later.
//
//   sequential, line known           i_valid the cycle after the request
//   other request, same line as kl   two cycles
//   other request, new line          three cycles (two when WAYPRED and the
//                                    hit is in the set's MRU way)
//
// The next line's tags are read while the current line streams (the tag
// RAM's port is free then), so a sequential fetch crosses a line without a
// bubble when the next line hits. Without i_seq (tied 0) every fetch is an
// "other request".
//
// After the tag compare (which follows an M10K's clock-to-out) only i_valid
// and registers are decided: the data RAM's next address, i_data's source,
// the set state write and the fill's tag-pair write never wait on it.
//
// Pre-read port (PRE = 1; study D-30, R61 design C): the core's BTB knows,
// with the request for a predicted branch's word B, that the request after it
// is the branch's target T, and the physical line and way T was fetched from
// last time (the entry's payload).
//
//   i_pv, i_pidx, i_pway, i_ppa   with B's request (a sequential one served
//                 by the fast path): the data and tag arrays are read at T's
//                 index at this edge, in place of the next sequential word and
//                 the lookahead.
//   i_pre         with T's request, the cycle after: T is answered at the next
//                 edge when tag[i_pway] = i_ppa (and ICE); otherwise T is an
//                 ordinary request (two or three cycles), so nothing is
//                 trusted that was not compared. kl / nx move to T's line, so
//                 T + 4 is sequential.
//
// i_way and i_arr go with every answer: the way that answered, and whether it
// came from the arrays (the core keeps them in the BTB entry it writes).
// i_ppa is never RSV_TAG for a line that can hit (the core writes RSV_TAG for
// a target answered by a single read).
//
// Two-word fetch (FETCH2 = 1; D-30): the data arrays are two banks per way,
// even and odd words (two 512 x 16 lanes each, the same 16 M10K), read at
// independent indices, so any word and the next one in its line come out
// together. Only the fast path answers two words: i_two says a sequential
// request now would get two (nx and nx + 1 are both in the known line and
// written), and the core asks for them with i_w2; the second word is
// i_data2. Every other answer is one word.
//
// Cache-inhibited fetches (I = 1), HID0[ICE] = 0, and misses under
// HID0[ILOCK] (3.1.3.3) are single reads, never touching the arrays. icbi
// invalidates all four ways of the set its EA indexes (3.7.8); HID0[ICFI]
// and reset sweep every set, 128 cycles. As in ppc_dcache, a line is invalid
// when its tag is RSV_TAG, and fetches from that page are single reads.

module ppc_icache
  import ppc_cache_pkg::*;
#(
  parameter logic [19:0] RSV_TAG = 20'hF00FF,
  parameter bit          WAYPRED = 1'b1,
  parameter bit          PRE     = 1'b1,               // the pre-read port (above)
  parameter bit          FETCH2  = 1'b0                // two-word answers (above; off by default, D-36)
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        i_req,
  input  logic [31:0] i_addr,
  input  logic        i_seq,
  input  logic [3:0]  i_wimg,
  output logic        i_valid,
  output logic [31:0] i_data,
  output logic [1:0]  i_way,        // with i_valid: the way that answered
  output logic        i_arr,        // ... from the arrays (not a single read)
  output logic [19:0] i_pa,         // ... and its physical page
  input  logic        i_pv,         // pre-read T at this edge (with B's request)
  input  logic [11:2] i_pidx,
  input  logic [1:0]  i_pway,
  input  logic [19:0] i_ppa,
  input  logic        i_pre,        // this request is T, pre-read at the last edge
  output logic        i_two,        // a sequential request now gets two words ...
  input  logic        i_w2,         // ... and this one asks for them
  output logic [31:0] i_data2,      // with i_valid after an i_w2 request: the next word

  input  logic        hid_ice,
  input  logic        hid_ilock,
  input  logic        hid_icfi,
  output logic        icfi_ack,
  input  logic        icbi_req,     // held until icbi_ack
  input  logic [6:0]  icbi_set,     // EA[20-26]
  output logic        icbi_ack,
  output logic        i_idle,

  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  input  logic        m_ack,
  input  logic        m_rvalid,
  input  logic [31:0] m_rdata,

  output logic        t_alloc,
  output logic [6:0]  t_alloc_set,
  output logic [1:0]  t_alloc_way,
  output logic [19:0] t_alloc_tag
);

  // ================================================================ arrays
  logic [9:0]       rd_a;                       // data read index
  logic [6:0]       ta_rd;                      // tag read index
  logic             fb_v;                       // a beat registered, written at the next edge
  logic [2:0]       fb_w;
  logic [31:0]      fb_d;
  logic [6:0]       f_set;
  logic [1:0]       f_way;
  logic [3:0][31:0] dq, dq2;                    // the word read and (FETCH2) the next one
  logic [9:0]       da;                         // the data output's index (registered rd_a)
  genvar gw, gl;
  generate
    if (FETCH2) begin : g_two
      // even bank at (w + 1) / 2 within the line, odd bank at w / 2 (for w = 7
      // the even read wraps in the line and is not used)
      // (the fast path's two next words come with their bank indices made from
      // registers, so the late two-word decision only selects)
      wire [8:0] ra_e = pre_take ? {i_pidx[11:5], i_pidx[4:3] + {1'b0, i_pidx[2]}} :
                        fast     ? (pair ? nx2_e : nx1_e) : {rd_a[9:3], rd_a[2:1] + {1'b0, rd_a[0]}};
`ifdef PPC_CACHE_SIM
      always_ff @(posedge clk)
        if (!rst && ra_e != {rd_a[9:3], rd_a[2:1] + {1'b0, rd_a[0]}}) $error("ppc_icache: even-bank index against rd_a");
`endif
      wire [8:0] ra_o = rd_a[9:1];
      logic [3:0][31:0] qe, qo;
      for (gw = 0; gw < 4; gw++) begin : g_way
        for (gl = 0; gl < 2; gl++) begin : g_lane
          ppc_cache_ram #(.W(16), .D(512)) u_e (
            .clk (clk), .we (fb_v && f_way == gw && !fb_w[0]), .wa ({f_set, fb_w[2:1]}), .wd (fb_d[31-16*gl -: 16]),
            .ra (ra_e), .q (qe[gw][31-16*gl -: 16]));
          ppc_cache_ram #(.W(16), .D(512)) u_o (
            .clk (clk), .we (fb_v && f_way == gw && fb_w[0]), .wa ({f_set, fb_w[2:1]}), .wd (fb_d[31-16*gl -: 16]),
            .ra (ra_o), .q (qo[gw][31-16*gl -: 16]));
        end
        assign dq[gw]  = da[0] ? qo[gw] : qe[gw];
        assign dq2[gw] = da[0] ? qe[gw] : qo[gw];
      end
    end else begin : g_one
      for (gw = 0; gw < 4; gw++) begin : g_way
        for (gl = 0; gl < 4; gl++) begin : g_lane
          ppc_cache_ram #(.W(8), .D(1024)) u_d (
            .clk (clk), .we (fb_v && f_way == gw), .wa ({f_set, fb_w}), .wd (fb_d[31-8*gl -: 8]),
            .ra (rd_a), .q (dq[gw][31-8*gl -: 8]));
        end
      end
      assign dq2 = '0;
    end
  endgenerate

  logic [1:0]  tw_en_q;
  logic [6:0]  tw_set_q;
  logic [79:0] tw_d_q;
  logic [39:0] tq0, tq1;
  ppc_cache_ram #(.W(40), .D(128)) u_t0 (.clk (clk), .we (tw_en_q[0]), .wa (tw_set_q), .wd (tw_d_q[39:0]),
                                         .ra (ta_rd), .q (tq0));
  ppc_cache_ram #(.W(40), .D(128)) u_t1 (.clk (clk), .we (tw_en_q[1]), .wa (tw_set_q), .wd (tw_d_q[79:40]),
                                         .ra (ta_rd), .q (tq1));
  logic [3:0][19:0] tg;
  assign tg = {tq1[19:0], tq1[39:20], tq0[19:0], tq0[39:20]};

  // {mru[1:0], plru[2:0]} per set, read at the set whose tags are out
  (* ramstyle = "MLAB, no_rw_check" *) logic [4:0] stm [0:127];
  logic        st_we;
  logic [6:0]  st_wa;
  logic [4:0]  st_wd;
  always_ff @(posedge clk) if (st_we) stm[st_wa] <= st_wd;

  // ================================================================ state
  logic        pend, p_unc;
  logic        wi_q;                            // i_wimg[2] of the last cycle's request
  logic [31:2] pa;
  logic [31:2] nx;                              // the next sequential word
  logic        fs_q;                            // nx can be answered at once
  logic        f2_q;                            // ... with nx + 1 beside it (FETCH2: i_two)
  logic [31:5] kl;                              // the known line: its way is kl_way
  logic        kl_v;
  logic [1:0]  kl_way;
  logic        nl_v;                            // kl + 1 hits in nl_way (same page)
  logic [1:0]  nl_way;
  logic [4:0]  nl_st;                           // kl + 1's set state after that access
  logic        kt_v;                            // kl was entered by a crossing: its set's touch is
  logic [6:0]  kt_set;                          // made at the first word served from it
  logic [4:0]  kt_st;
  logic        da_av;                           // ... and it was written before it was read
  logic        ta_v, ta_look;                   // tags out: valid, and a lookahead
  logic [6:0]  ta_idx;
  logic [19:0] ta_exp;                          // the tag a lookahead compares
  logic        f_act;
  logic [31:5] f_line;
  logic [2:0]  f_cnt, f_crit;
  logic [7:0]  avail;                           // fill words written
  logic        tw_last;
  logic [6:0]  tw_lset;
  logic        wk_act, wk_req, icfi_q;
  logic [7:0]  wk_cnt;

  typedef enum logic [1:0] { MP_IDLE, MP_RQ, MP_FILL, MP_SRD } mp_t;
  mp_t         mp;
  wire         mp_idle = (mp == MP_IDLE);

  // ================================================================ compare
  // a check's or a fill's state update is registered (stp_*) and written in
  // the next cycle. No read of that set can fall in that cycle: the request
  // stays pending (no check) or is answered, and the lookahead reads the
  // next set (asserted below).
  logic        stp_v;
  logic [6:0]  stp_set;
  logic [1:0]  stp_way;
  logic [4:0]  stp_st;
  logic        fw_pend, fw2;                    // the fill's tag write: partner tag taken, then written
  logic [19:0] fw_otag;
  wire  [4:0]  stp_new = {stp_way, plru_touch(stp_st[2:0], stp_way)};
  wire  [4:0]  st     = stm[ta_idx];
  logic [3:0]  hv, inv;
  wire  [19:0] cmp_tag = ta_look ? ta_exp : pa[31:12];
  always_comb
    for (int w = 0; w < 4; w++) begin
      hv[w]  = tg[w] == cmp_tag;
      inv[w] = tg[w] == RSV_TAG;
    end
  wire         any  = |hv;
  wire  [1:0]  hw   = enc4(hv);
  wire  [2:0]  ff   = first_free(inv);
  wire  [1:0]  vic  = ff[2] ? ff[1:0] : plru_victim(st[2:0]);
  // the tags out are stale if their set was written on or after the read edge
  wire         t_haz = (tw_en_q != 2'd0 && tw_set_q == ta_idx) || (tw_last && tw_lset == ta_idx) ||
                       ((fw_pend || fw2) && f_set == ta_idx);

  function automatic logic [4:0] touch(input logic [4:0] s, input logic [1:0] w);
    touch = {w, plru_touch(s[2:0], w)};
  endfunction

  wire         beat    = (mp == MP_FILL) && m_rvalid;
  wire  [2:0]  bw      = f_crit + f_cnt;        // the word this beat carries

  // ================================================================ cycle
  // A request's first pending cycle always goes to the tag check (the MRU
  // way answers it, which is kl_way when pa is in kl). From the second
  // cycle "pa is in kl" and "pa's word is still being filled" are registers,
  // so the data select is one level from registers.
  logic        pend_new, pl_q, pfw_q;
  wire  [7:0]  avail_nx = avail | (fb_v ? 8'd1 << fb_w : 8'd0);    // avail after this edge
  wire         p_line   = !pend_new && pl_q;                     // pa's line is known
  // single read: HID0[ICE] = 0 or ILOCK (p_unc), or, from the first pending
  // cycle, I = 1 or the reserved page. Fetch translation makes i_wimg and
  // i_addr[31:12] late, so they are taken from registers here (wi_q, pa);
  // the first pending cycle is the tag check anyway, so this costs no cycle
`ifdef VERILATOR
  logic [31:2] pa_chk;                          // pa as loaded before, at a request's start only
  always_ff @(posedge clk) if (!rst && !phit && !fast && i_req && !pend) pa_chk <= i_addr[31:2];
  always_ff @(posedge clk) if (!rst && pend && pa != pa_chk) $fatal(1, "icache pa against its start-only load");
`endif
  wire         p_ui     = p_unc || (pend_new && (wi_q || pa[31:12] == RSV_TAG));
  wire         chk      = pend && !p_ui && !p_line && ta_v && !ta_look && ta_idx == pa[11:5] && !t_haz && !wk_act;
  wire         look     = !pend && ta_v && ta_look && kl_v && ta_idx == kl[11:5] + 7'd1 && ta_exp == kl[31:12] &&
                          !t_haz && !wk_act;
  wire         fast     = i_req && i_seq && fs_q && !pend;
  wire         pair     = FETCH2 && fast && i_w2;               // two words answered
  assign i_two = FETCH2 && f2_q;
  wire         fill_ok  = !f_act && mp_idle && !wk_act && !wk_req;

  // the pre-read: taken at B's edge, used by T's request in the next cycle.
  // Only the fast path can answer in a pre_q cycle (fs_q is cleared at the
  // pre-read), so i_data's source is chosen by pre_q, a register.
  logic        pre_q;
  logic [1:0]  pw_q;
  logic [19:0] ppa_q;
  wire         pre_take = PRE && i_pv && fast && !wk_act && !f_act && hid_ice;
  wire         pfast    = PRE && i_req && i_pre && pre_q && !pend && ta_v && !ta_look && !t_haz && !wk_act && !f_act;
  // each way's tag against the page, the stored way choosing last: the way
  // mux was in front of the compare, from the tag M10K into pend and nx (a
  // D-36 cached full build)
  logic [3:0]  pv;
  always_comb
    for (int w = 0; w < 4; w++) pv[w] = tg[w] == ppa_q;
  wire         phit     = pfast && pv[pw_q] && ppa_q != RSV_TAG;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && phit != (pfast && tg[pw_q] == ppa_q && ppa_q != RSV_TAG)) $fatal(1, "phit per way");
`endif

  // The read address and the data select depend on registers only; the
  // compare decides i_valid and the bookkeeping.
  logic        serve;                           // answer pa this cycle
  logic        serve_r;                         // ... decided from registers (no compare)
  logic [1:0]  src;                             // 0 kl_way, 1 mru, 2 m_rdata, 3 fb_d
  logic        start_fill, to_unc, chk_hit;
  always_comb begin
    serve_r = 1'b0; src = 2'd0;
    if (pend && !wk_act) begin
      if (p_ui) begin
        src = 2'd2;
        serve_r = (mp == MP_SRD) && m_rvalid;
      end else if (p_line) begin
        if (pfw_q) begin                        // the word is still on its way
          src     = m_rvalid ? 2'd2 : 2'd3;
          serve_r = m_rvalid ? (beat && bw == pa[4:2]) : (fb_v && fb_w == pa[4:2]);
        end else serve_r = da == pa[11:2] && da_av;
      end else src = 2'd1;
    end
  end
  always_comb begin
    serve = serve_r; start_fill = 1'b0; to_unc = 1'b0; chk_hit = 1'b0;
    if (chk) begin
      if (any) begin
        chk_hit = 1'b1;
        if (WAYPRED && hv[st[4:3]] && da == pa[11:2] && da_av) serve = 1'b1;
      end else if (hid_ilock) to_unc = 1'b1;
      else if (fill_ok) start_fill = 1'b1;
    end
  end

  // availability of word w of line ln for a read made at the coming edge
  function automatic logic av(input logic [31:5] ln, input logic [2:0] w, input logic fa,
                              input logic [31:5] fl, input logic [7:0] avl);
    av = !(fa && ln == fl && !avl[w]);
  endfunction

  // ================================================================ read addresses
  wire [31:2] nx1 = nx + 30'd1;
  wire [31:2] nxn = nx + (pair ? 30'd2 : 30'd1);   // the fast path's next word
  wire        xing = nx[4:2] == 3'd7 || (pair && nx[4:2] == 3'd6);   // the answer ends the line
  wire [31:2] nx2 = nx + 30'd2;
  wire [8:0]  nx1_e = {nx1[11:5], nx1[4:3] + {1'b0, nx1[2]}};   // even-bank indices of nx + 1, nx + 2
  wire [8:0]  nx2_e = {nx2[11:5], nx2[4:3] + {1'b0, nx2[2]}};
  wire [31:2] pa1 = pa + 30'd1;
  always_comb begin
    if (pre_take)            rd_a = i_pidx;
    else if (pfast)          rd_a = i_addr[11:2] + 10'd1;   // a miss is then served a cycle later
    else if (fast)           rd_a = nxn[11:2];
    else if (i_req && !pend) rd_a = i_addr[11:2];
    else if (pend)           rd_a = (serve_r || (chk && WAYPRED)) ? pa1[11:2] : pa[11:2];
    else                     rd_a = nx[11:2];
  end

  logic ta_k_look, ta_k_v;
  always_comb begin
    ta_k_look = 1'b0; ta_k_v = 1'b1;
    if (pre_take)                 ta_rd = i_pidx[11:5];
    else if (pfast)               ta_rd = i_addr[11:5];  // (no lookahead this cycle)
    else if (i_req && !fast && !pend)  ta_rd = i_addr[11:5];
    else if (pend)                ta_rd = pa[11:5];
    else begin
      ta_rd = kl[11:5] + 7'd1; ta_k_look = 1'b1; ta_k_v = kl_v && kl[11:5] != 7'd127;
    end
  end

  // ================================================================ state write
  wire kt_use = kt_v && (fast || (serve_r && p_line && !p_ui));
  always_comb begin
    st_we = 1'b0; st_wa = stp_set; st_wd = stp_new;
    if (wk_act)      begin st_we = 1'b1; st_wa = wk_cnt[6:0]; st_wd = 5'd0; end
    else if (stp_v)  st_we = 1'b1;
    else if (kt_use) begin st_we = 1'b1; st_wa = kt_set; st_wd = kt_st; end
  end

  // ================================================================ sequential
  always_ff @(posedge clk) begin
    i_valid  <= 1'b0;
    t_alloc  <= 1'b0;
    icfi_ack <= 1'b0;
    icbi_ack <= 1'b0;
    fb_v     <= 1'b0;
    tw_last  <= tw_en_q != 2'd0;
    tw_lset  <= tw_set_q;
    tw_en_q  <= 2'd0;
    icfi_q   <= hid_icfi;
    wi_q     <= i_wimg[2];
    // pa loads whenever no request is pending, not only as one starts: it is
    // read only while pending (and in the bookkeeping registered beside it,
    // which the first pending cycle ignores), so the same value is there. As
    // the load of one branch it waited on phit and fast, and the fetch
    // translation into pa led two round-1 cached full builds (p_tgt -> pa)
    if (!pend) pa <= i_addr[31:2];
    da       <= rd_a;
    da_av    <= av({f_line[31:12], rd_a[9:3]}, rd_a[2:0], f_act, f_line, avail);
    ta_idx   <= ta_rd;
    ta_look  <= ta_k_look;
    ta_v     <= ta_k_v;
    ta_exp   <= kl[31:12];

    if (PRE && pre_q) i_data <= dq[pw_q];
    else
      case (src)
        2'd1:    i_data <= dq[st[4:3]];
        2'd2:    i_data <= m_rdata;
        2'd3:    i_data <= fb_d;
        default: i_data <= dq[kl_way];
      endcase
    i_way   <= (PRE && pre_q) ? pw_q : (src == 2'd1) ? st[4:3] : kl_way;
    i_data2 <= dq2[kl_way];
    i_arr   <= !(pend && p_ui);
    i_pa    <= (PRE && pre_q) ? ppa_q : pend ? pa[31:12] : nx[31:12];
    stp_v   <= (chk_hit || start_fill || phit) && !rst && !wk_act;
    stp_set <= ta_idx; stp_st <= st; stp_way <= pfast ? pw_q : start_fill ? vic : hw;
    pre_q   <= pre_take && !rst;
    pw_q    <= i_pway;
    ppa_q   <= i_ppa;
    fw_pend <= start_fill && !rst;
    fw2     <= fw_pend && !rst;
    fw_otag <= tg[f_way ^ 2'd1];                // pa's set is still on the tag output
    if (fw2) begin
      tw_en_q  <= f_way[1] ? 2'b10 : 2'b01;
      tw_set_q <= f_set;
      tw_d_q   <= {2{f_way[0] ? {fw_otag, f_line[31:12]} : {f_line[31:12], fw_otag}}};
    end
    // the next cycle's view of pa against kl and the fill
    pend_new <= i_req && !fast && !pend && !phit;
    pl_q     <= chk_hit || start_fill || (kl_v && kl == pa[31:5]);
    pfw_q    <= start_fill || (f_act && f_line == pa[31:5] && !avail_nx[pa[4:2]]);

    if (rst) begin
      pend <= 1'b0; fs_q <= 1'b0; f2_q <= 1'b0; kl_v <= 1'b0; nl_v <= 1'b0; kt_v <= 1'b0; f_act <= 1'b0; mp <= MP_IDLE; m_req <= 1'b0;
      wk_act <= 1'b1; wk_cnt <= 8'd0; wk_req <= 1'b0; tw_last <= 1'b0; ta_v <= 1'b0;
    end else begin
      begin
        // ---- the pre-read target, compared against the page and way the BTB holds
        if (phit) begin
          i_valid <= 1'b1;
          nx      <= {ppa_q, i_addr[11:2]} + 30'd1;
          kl      <= {ppa_q, i_addr[11:5]}; kl_way <= pw_q; kl_v <= 1'b1; nl_v <= 1'b0; kt_v <= 1'b0;
          fs_q    <= i_addr[4:2] != 3'd7;
          f2_q    <= i_addr[4:2] < 3'd6;
        end else
        // ---- the fast path
        if (fast) begin
          i_valid <= 1'b1;
          nx      <= nxn;
          if (kt_v) kt_v <= 1'b0;                      // this word is kl's first since the crossing
          if (xing) begin                               // crossing into kl + 1
            fs_q <= nl_v && !f_act && hid_ice;
            f2_q <= nl_v && !f_act && hid_ice;          // its words 0 and 1
            if (nl_v) begin
              kl <= kl + 27'd1; kl_way <= nl_way;
              kt_v <= 1'b1; kt_set <= kl[11:5] + 7'd1; kt_st <= touch(nl_st, nl_way);
            end else kl_v <= 1'b0;
            nl_v <= 1'b0;
          end else begin
            fs_q <= av(kl, nxn[4:2], f_act, f_line, avail);
            f2_q <= nxn[4:2] != 3'd7 && av(kl, nxn[4:2], f_act, f_line, avail) &&
                    av(kl, nxn[4:2] + 3'd1, f_act, f_line, avail);
          end
        end else if (i_req && !pend) begin
          pend  <= 1'b1;
          p_unc <= !hid_ice;
          fs_q  <= 1'b0;
          f2_q  <= 1'b0;
        end else if (pend) begin
          fs_q <= 1'b0;
          f2_q <= 1'b0;
          nx   <= pa1;                                   // what follows pa, once it is answered
          p_unc <= p_ui;
          if (serve) begin
            if (p_line && !p_ui && serve_r) kt_v <= 1'b0;
            i_valid <= 1'b1;
            pend    <= 1'b0;
            fs_q    <= !p_ui && pa[4:2] != 3'd7 && hid_ice &&
                       av(pa[31:5], pa1[4:2], f_act, f_line, avail);
            f2_q    <= !p_ui && pa[4:2] < 3'd6 && hid_ice &&
                       av(pa[31:5], pa1[4:2], f_act, f_line, avail) && av(pa[31:5], pa1[4:2] + 3'd1, f_act, f_line, avail);
          end
          if (chk) begin                                 // kl follows the line checked
            kl <= pa[31:5]; kl_way <= start_fill ? vic : hw;
            kl_v <= chk_hit || start_fill; nl_v <= 1'b0; kt_v <= 1'b0;
          end
          if (to_unc) p_unc <= 1'b1;
        end else begin
          fs_q <= fs_q && kl_v && hid_ice;
          f2_q <= f2_q && kl_v && hid_ice;
        end

        if (pre_take) begin fs_q <= 1'b0; f2_q <= 1'b0; end   // T was read, not the next word

        // ---- the lookahead result (kept unless kl moves on this edge)
        if (look && !(fast && xing)) begin
          nl_v <= any; nl_way <= hw; nl_st <= st;        // the set's state, touched at the crossing
        end
      end

      // ---- sweep and icbi (the tag write port); they drop all that is known
      // about lines, so they come after the paths above
      if (hid_icfi && !icfi_q && hid_ice) wk_req <= 1'b1;
      if (wk_act) begin
        tw_en_q <= 2'b11; tw_set_q <= wk_cnt[6:0]; tw_d_q <= {4{RSV_TAG}};
        wk_cnt  <= wk_cnt + 8'd1;
        if (wk_cnt == 8'd127) wk_act <= 1'b0;
        kl_v <= 1'b0; nl_v <= 1'b0; fs_q <= 1'b0; f2_q <= 1'b0; kt_v <= 1'b0; pl_q <= 1'b0;
      end else if (wk_req && !f_act && mp_idle) begin
        wk_req <= 1'b0; wk_act <= 1'b1; wk_cnt <= 8'd0; icfi_ack <= 1'b1;
        kl_v <= 1'b0; nl_v <= 1'b0; fs_q <= 1'b0; f2_q <= 1'b0; kt_v <= 1'b0; pl_q <= 1'b0;
      end else if (icbi_req && !pend && !fw_pend && !fw2 && !icbi_ack) begin
        tw_en_q <= 2'b11; tw_set_q <= icbi_set; tw_d_q <= {4{RSV_TAG}};
        icbi_ack <= 1'b1;
        kl_v <= 1'b0; nl_v <= 1'b0; fs_q <= 1'b0; f2_q <= 1'b0; kt_v <= 1'b0; pl_q <= 1'b0;
      end

      // ---- the fill
      // (its fields load while it is idle; only f_act waits on the compare)
      if (!f_act) begin
        f_line <= pa[31:5]; f_set <= pa[11:5]; f_way <= vic; f_cnt <= 3'd0; f_crit <= pa[4:2];
        avail  <= 8'd0;
        t_alloc_set <= pa[11:5]; t_alloc_way <= vic; t_alloc_tag <= pa[31:12];
      end
      if (start_fill) begin
        f_act <= 1'b1; t_alloc <= 1'b1;
      end else if (f_act) begin
        if (beat) begin
          fb_v <= 1'b1; fb_w <= bw; fb_d <= m_rdata; f_cnt <= f_cnt + 3'd1;
        end
        if (fb_v) begin
          avail[fb_w] <= 1'b1;
          if (avail == (8'hFF ^ (8'd1 << fb_w))) f_act <= 1'b0;   // the last word lands this edge
        end
      end

      // ---- memory port
      case (mp)
        MP_IDLE: begin                                 // a fill or a single read, both of pa
          m_addr <= {pa, 2'd0}; m_be <= 4'hF; m_cmd <= start_fill ? MC_LINE_RD : MC_RD;
          if (start_fill || (pend && p_ui && !f_act && !wk_act)) begin mp <= MP_RQ; m_req <= 1'b1; end
        end
        MP_RQ: if (m_ack) begin
          m_req <= 1'b0;
          mp    <= (m_cmd == MC_LINE_RD) ? MP_FILL : MP_SRD;
        end
        MP_FILL: if (m_rvalid && f_cnt == 3'd7) mp <= MP_IDLE;
        MP_SRD:  if (m_rvalid) mp <= MP_IDLE;
        default: mp <= MP_IDLE;
      endcase
    end
  end

  assign i_idle = !pend && !f_act && mp_idle && !wk_act && !wk_req && tw_en_q == 2'd0;

`ifdef PPC_CACHE_SIM
  always_ff @(posedge clk) if (!rst) begin
    if (i_req && pend) $error("ppc_icache: request while one is outstanding");
    if (fast && i_addr[31:2] != nx) $error("ppc_icache: i_seq with a non-sequential address");
    if (stp_v && kt_use) $error("ppc_icache: two state writes");
    if (stp_v && stp_set == ta_idx && (chk || look)) $error("ppc_icache: state read during its write");
    if (pfast && (ta_idx != i_addr[11:5] || da != i_addr[11:2])) $error("ppc_icache: i_pre for an address not pre-read");
    if (i_pre && i_seq) $error("ppc_icache: i_pre with i_seq");
    if (i_req && i_w2 && !(fast && f2_q)) $error("ppc_icache: i_w2 where two words cannot be answered");
  end
`endif

endmodule
