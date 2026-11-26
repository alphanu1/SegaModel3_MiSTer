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
// Real3D rasteriser: the Z band and the Z walk's control, two Z tests a clock
// (D-28). Bit for bit render_d25's Z test (r3d_rast_rtl.cpp), band by band.
//
// Memory: an 8-row band, two banks by x parity, word {stamp[5:0], z[23:0]} at
// {row[2:0], x[8:1]} (2K x 30, six 2K x 5 M10K a bank). Lane 0 is pixel x,
// lane 1 is x + 1: bank 0 holds the even pixel, at x[8:1] + x[0].
//
// Clear per (band, pass) by generation: a word whose stamp is not the current
// generation reads as the clear value (0 with depth on 1/w, all ones on w).
// s_new on a pair starts a generation: the pipe drains, 32 words of each bank
// are swept to {old generation, clear value}, then the generation steps. A
// swept word reads as the clear value whatever its stamp, and a word a pixel
// wrote is swept within 64 generations, before its 6-bit stamp comes round.
// After reset all 2048 are swept.
//
// Pipe: A (pair in, read issued) -> B (M10K out) -> C (compare, write) -> O.
// C forwards from the two writes it cannot see in the RAM: the one made on the
// previous advance and the one on the advance it was read on. B matches the
// addresses and tests the RAM word's stamp, so C is a select and a compare.
//
// Contour: the Z write waits for shading's alpha test. A passing contour
// pixel goes into the pending FIFO (MLAB) and the scoreboard: per band row a
// count and the x range of the pending pixels. A pair holds in A while its
// row's range covers it, or while a contour pixel of another triangle is in
// B or C. Verdicts come back in order on c_valid; a write takes its bank's
// port the next cycle and stalls the pipe if C holds a pair. A verdict from an
// older generation, or arriving once the sweep has begun, writes nothing: no
// pixel of that generation is left to read it, and a stamp written behind the
// sweep pointer would come round live. The generation steps only on an edge
// with no verdict arriving.
//
// A passing contour pair is registered in P before it enters the FIFO and the
// scoreboard, so the hold also covers P.
//
// TRANS = 1: the 4 x 4 ordered-dither screen door for translucent polygons
// (s_stip), tested before Z as render_d25 does. D-29 makes 0 (plain blend) the
// build; at 0 the door folds away.

module r3d_rast_zband #(
  parameter bit OOW   = 1'b1,     // depth on 1/w: larger nearer, clear 0
  parameter int TRANS = 0,        // 0 MAME blend, 1 stipple
  parameter int PW    = 42,       // payload a lane (the 1/w plane value)
  parameter int TW    = 8,        // triangle tag
  parameter int CD    = 32,       // pending contour pairs
  // SBY (r3d140, Z8): the pair coming in is offered as both of its sources, the caller's skid (s_*_k) and the walk's
  // output (s_*_w), with the select (s_sk): the scoreboard test runs on each side by side and the select comes last,
  // so no 2:1 data select sits in front of the compares. 0: the test reads s_y / s_x / s_on.
  parameter bit SBY   = 1'b0
) (
  input  logic               clk,
  input  logic               rst,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic               s_new,
  input  logic        [8:0]  s_y,
  input  logic        [8:0]  s_x,
  input  logic        [1:0]  s_on,
  input  logic        [23:0] s_z0, s_z1,
  input  logic      [PW-1:0] s_p0, s_p1,
  input  logic        [1:0]  s_zk,     // 0 no Z write, 1 write, 2 contour
  input  logic      [TW-1:0] s_tag,
  input  logic               s_stip,   // TRANS = 1: screen door on
  input  logic        [4:0]  s_trans,
  input  logic               s_pinv,   // header[6] bit 17
  input  logic               s_sk,     // SBY: s_* is the skid's (s_*_k), else the walk's (s_*_w)
  input  logic        [8:0]  s_y_k, s_x_k, s_y_w, s_x_w,
  input  logic        [1:0]  s_on_k, s_on_w,

  output logic               o_valid,
  input  logic               o_ready,
  output logic        [8:0]  o_y,
  output logic        [8:0]  o_x,
  output logic        [1:0]  o_on,     // lanes that passed
  output logic      [PW-1:0] o_p0, o_p1,
  output logic      [TW-1:0] o_tag,
  output logic               o_ctr,    // contour: a verdict per passing lane

  input  logic               c_valid,  // one per passing contour pixel, in order
  input  logic               c_zw
);

  localparam logic [23:0] ZCLR = OOW ? 24'h000000 : 24'hffffff;
  localparam int NSW = 32;

  // ---- memory ------------------------------------------------------------------
  (* ramstyle = "M10K, no_rw_check" *) logic [29:0] zm0 [2048];
  (* ramstyle = "M10K, no_rw_check" *) logic [29:0] zm1 [2048];
  logic        we0, we1, re;
  logic [10:0] wa0, wa1, ra0, ra1;
  logic [29:0] wd0, wd1, q0, q1;
  always_ff @(posedge clk) begin
    if (we0) zm0[wa0] <= wd0;
    if (we1) zm1[wa1] <= wd1;
    if (re) begin q0 <= zm0[ra0]; q1 <= zm1[ra1]; end
  end

  // ---- stages ------------------------------------------------------------------
  logic               a_v, a_new;
  logic        [8:0]  a_y, a_x;
  logic        [1:0]  a_on, a_zk;
  logic        [23:0] a_z0, a_z1;
  logic      [TW-1:0] a_tag;
  logic               a_stip, a_pinv;
  logic        [4:0]  a_tr;

  logic               b_v, c_v;
  logic        [8:0]  b_y, b_x, c_y, c_x;
  logic        [1:0]  b_on, b_zk, c_on, c_zk;   // b_on, c_on: after the screen door
  logic        [23:0] b_z0, b_z1, c_z0, c_z1;
  logic      [PW-1:0] c_p0, c_p1;
  // The payloads ride A, B and C untouched to O, so they wait in an eight-entry ring (area, R179) instead of
  // 6 x PW registers: written from s_p0 / s_p1 as A loads, at pw_i, its index carried through A and B, and read
  // at B's index as C loads (adv, the block's address register and read enable), so the block's output is C's
  // word in C's clock and O stays a register. A loads at most twice between a pair's write and its read (A
  // loads only empty or going, and going needs adv), so a live entry is never written over, and the read is
  // never at the entry being written.
  logic        [2:0]  pw_i, a_i, b_i;
  (* ramstyle = "M10K, no_rw_check" *) logic [2*PW-1:0] pring [8];
  always_ff @(posedge clk) begin
    if (s_valid && s_ready) pring[pw_i] <= {s_p0, s_p1};
    if (adv) {c_p0, c_p1} <= pring[b_i];
  end
`ifdef VERILATOR
  logic      [PW-1:0] a_p0, a_p1, b_p0, b_p1, co_p0, co_p1;   // the old registers, for the check
`endif
  logic      [TW-1:0] b_tag, c_tag;
  logic        [10:0] b_a0, b_a1, c_a0, c_a1;   // bank addresses
  logic        [23:0] c_q0, c_q1;
  logic               c_qv0, c_qv1, c_m10, c_m20, c_m11, c_m21;   // stamp live; matches W1, W2
  logic        [8:0]  c_x1;

  // the two writes C cannot see in the RAM, a bank each
  logic        w1v0, w1v1, w2v0, w2v1;
  logic [10:0] w1a0, w1a1;
  logic [23:0] w1d0, w1d1, w2d0, w2d1;

  logic [5:0]  gen;
  logic [10:0] swp;          // sweep pointer
  logic [11:0] sw_left;      // sweep words left (both banks at once)
  logic        sw_on;

  // ---- the verdict register and the pending FIFO -------------------------------
  // entry: {gen, row, x, mask, z1, z0}
  localparam int EW = 6 + 3 + 9 + 2 + 48;
  localparam int CA = $clog2(CD);
  (* ramstyle = "MLAB, no_rw_check" *) logic [EW-1:0] pf [CD];
  logic [CA-1:0] pf_wp, pf_rp;
  logic [CA:0]   pf_n;
  logic          pf_lane;     // head entry: lane 0 done
  wire  [EW-1:0] pf_h = pf[pf_rp];
  wire  [5:0]    h_gen  = pf_h[67:62];
  wire  [2:0]    h_row  = pf_h[61:59];
  wire  [8:0]    h_x    = pf_h[58:50];
  wire  [1:0]    h_m    = pf_h[49:48];
  wire  [23:0]   h_z1   = pf_h[47:24];
  wire  [23:0]   h_z0   = pf_h[23:0];
  // the head's current pixel: lane 0 first if it passed
  wire           h_l    = pf_lane || !h_m[0];
  wire           h_last = h_l || !h_m[1];
  wire  [8:0]    h_px   = h_x + {8'd0, h_l};

  logic        v_full, v_wr, v_bank;
  logic [5:0]  v_gen;
  logic [10:0] v_a;
  logic [23:0] v_z;
  logic [2:0]  v_row;

  // ---- the scoreboard ----------------------------------------------------------
  logic [6:0] sb_n  [8];
  logic [7:0] sb_nz;          // sb_n[r] != 0, as registers (study R109)
  // P: a passing contour pair on its way into the FIFO and the scoreboard
  logic        p_v;
  logic [TW-1:0] p_tag;
  logic [EW-1:0] p_e;
  logic [8:0]  p_x1;
  wire  [2:0]  p_row = p_e[61:59];
  wire  [1:0]  p_m   = p_e[49:48];
  // the pushed range, from P's own registers: P lives a cycle and is read only on p_v, so
  // it loads every cycle and the C stage's pass bits end at P's D inputs (no c_push enable,
  // no range mux after the compare: a -40C path)
  wire  [8:0]  p_px = p_e[58:50];
  wire  [8:0]  p_lo_q = p_m[0] ? p_px : p_x1;
  wire  [8:0]  p_hi_q = p_m[1] ? p_x1 : p_px;
  logic [8:0] sb_lo [8];
  logic [8:0] sb_hi [8];

  // ---- control -----------------------------------------------------------------
  logic      gen_ok;    // this A's generation step is done
  wire out_free = !o_valid || o_ready;
  wire vport    = v_full && v_wr && !sw_on;
  wire adv      = out_free && !(vport && c_v);

  wire [2:0] a_row = a_y[2:0];
  // the scoreboard test's row as a one-hot and A's last x, registered as A loads (study R109): the add
  // and the row's decode were in front of the compares, and the test is in a_go
  // (r3d140: the row as a one-hot, a_r1, is gone with the side-by-side test)
  logic [8:0] a_xh;
  // each row's test side by side, the row chosen last (the select before the compares was the z band's
  // slowest path once the add and decode had gone)
  // r3d140: the test as a register, sb_q, made a clock ahead for the pair A will hold: against the scoreboard as it
  // stands then (sb_lo / sb_hi / sb_nz, the last push included) and against P's push, which the scoreboard takes at
  // that edge, side by side for A's own pair and for the one coming in, chosen by a_ld last. A decrement at that
  // edge is not seen, so a pair can wait a clock longer than before; a pair between two pending ranges of one row
  // (inside their hull, on none of their pixels) no longer waits. Either way no pair goes onto a pending pixel
  // (asserted below on the FIFO's pixels). The compares and their reduction were in front of a_go, which fans
  // out into A's and B's enables and s_ready: a_x -> a_z0 / a_z1 / the M10K, the rasteriser's worst family at
  // 140 MHz standalone (-1.03 ns).
  // (Off a_go's path, A's row is chosen first and compared once. The pair coming in is not compared: its x and y
  // come through r3d_rast's skid select from the Z walk, and their compares into sb_q were the rasteriser's worst
  // standalone path in a first form, -1.82 ns. It waits a clock when anything is pending (a row's count, or P),
  // and is then tested as A's own; with nothing pending it cannot hit.)
  logic       sb_q;
  wire        hit_c = sb_nz[a_row] && !(a_xh < sb_lo[a_row] || a_x > sb_hi[a_row]);
  wire        hp_c = p_v && p_row == a_row && !(a_xh < p_lo_q || a_x > p_hi_q);
  wire        sb_hit = sb_q;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst) for (int r = 0; r < 8; r++) if (sb_nz[r] != (sb_n[r] != 7'd0)) $fatal(1, "zband: sb_nz");
`endif
  // A's tag against B's, C's and P's, kept as three registered flags (study R109): the 8-bit compares
  // were in front of a_go, and a_go fans out into every A, B, C and O enable. Each flag is the compare of
  // the registers as they will stand, from the compares of the incoming tag (s_tag) against A, B and C
  // and the flags themselves, chosen by the load and advance.
  logic      eq_ab, eq_ac, eq_ap;
  wire       ctr_bc = (b_v && b_zk == 2'd2 && !eq_ab) || (c_v && c_zk == 2'd2 && !eq_ac) || (p_v && !eq_ap);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && a_v && ctr_bc != ((b_v && b_zk == 2'd2 && b_tag != a_tag) || (c_v && c_zk == 2'd2 && c_tag != a_tag) ||
                                  (p_v && p_tag != a_tag)))
      $fatal(1, "zband: the tag flags");
`endif
  wire       pf_full = (a_zk == 2'd2) && (pf_n > (CA+1)'(CD - 4));
  wire       a_gen_wait = a_new && (b_v || c_v || !gen_ok);
  wire       a_go   = a_v && adv && !sw_on && !sb_hit && !ctr_bc && !pf_full && !a_gen_wait;
  assign s_ready = !a_v || a_go;
  wire       a_ld = s_valid && s_ready;
  // (r3d140, Z7) the pair coming in waits a clock only when its own row has something pending (that row's count,
  // or P on that row): the row's bit and P's row compare, not the hull compares. Sound as Z1: what is pending at
  // the next clock on row r is in the scoreboard's row r now or in P now. (Z4's form waited on anything pending
  // anywhere: Daytona 2's f004080 replay +8.4%, 788,863 -> 855,500 clocks, the Z band's stall +68k.)
  // (Z8: and then not Z7: on Daytona 2's f004080 the row test alone gave back 4k of the 67k; the pairs coming in
  // fall on rows with something pending but outside its hull. So the incoming pair has the hull test again, as Z1,
  // but on both of its sources side by side (SBY): Z1's form had r3d_rast's skid select in front of it, -1.82 ns.)
  // (written out per source, no function over the scoreboard's arrays)
  wire [2:0] rk = s_y_k[2:0], rw = s_y_w[2:0], rs = s_y[2:0];
  wire [8:0] xhk = s_x_k + {8'd0, s_on_k[1]}, xhw = s_x_w + {8'd0, s_on_w[1]}, xhs = s_x + {8'd0, s_on[1]};
  wire hit_k = (sb_nz[rk] && !(xhk < sb_lo[rk] || s_x_k > sb_hi[rk])) || (p_v && p_row == rk && !(xhk < p_lo_q || s_x_k > p_hi_q));
  wire hit_w = (sb_nz[rw] && !(xhw < sb_lo[rw] || s_x_w > sb_hi[rw])) || (p_v && p_row == rw && !(xhw < p_lo_q || s_x_w > p_hi_q));
  wire hit_s = (sb_nz[rs] && !(xhs < sb_lo[rs] || s_x   > sb_hi[rs])) || (p_v && p_row == rs && !(xhs < p_lo_q || s_x   > p_hi_q));
  wire hit_i = SBY ? (s_sk ? hit_k : hit_w) : hit_s;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && SBY && s_valid && {s_y, s_x, s_on} != (s_sk ? {s_y_k, s_x_k, s_on_k} : {s_y_w, s_x_w, s_on_w}))
    $fatal(1, "zband: the incoming pair's two sources");
`endif
  always_ff @(posedge clk) sb_q <= a_ld ? hit_i : (hit_c || hp_c);
`ifdef VERILATOR
  // no pair goes onto a pixel still pending: the FIFO's entries (the head's lane 0 done when pf_lane), P, and the
  // verdict register's write
  always_ff @(posedge clk) if (!rst && a_go) begin
    logic [1:0] am;
    am = a_on & a_door;
    for (int k = 0; k < CD; k++) if ((CA+1)'(CA'(CA'(k) - pf_rp)) < pf_n) begin
      logic [EW-1:0] e;
      logic [1:0]    m;
      e = pf[CA'(k)]; m = e[49:48];
      if (CA'(k) == pf_rp && pf_lane) m[0] = 1'b0;
      for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
        if (am[i] && m[j] && e[61:59] == a_row && a_x + 9'(i) == e[58:50] + 9'(j)) $fatal(1, "zband: a pair onto a pending pixel");
    end
    if (p_v) for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
      if (am[i] && p_m[j] && p_row == a_row && a_x + 9'(i) == p_px + 9'(j)) $fatal(1, "zband: a pair onto P's pixel");
    // (the verdict's pixel is {v_a[7:0], v_bank} on row v_row)
    if (v_full && v_wr) for (int i = 0; i < 2; i++)
      if (am[i] && v_row == a_row && a_x + 9'(i) == {v_a[7:0], v_bank}) $fatal(1, "zband: a pair onto the verdict's pixel");
  end
`endif
  wire       sa = s_tag == a_tag, sb = s_tag == b_tag, sc = s_tag == c_tag;
  always_ff @(posedge clk) begin
    // B takes A's tag on a_go, C B's on adv, P C's every cycle
    eq_ab <= a_ld ? (a_go ? sa : sb) : (a_go || eq_ab);
    eq_ac <= a_ld ? (adv ? sb : sc) : (adv ? eq_ab : eq_ac);
    eq_ap <= a_ld ? sc : eq_ac;
  end

  // bank mapping: bank 0 holds the even pixel
  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [10:0] ba0(input logic [8:0] y, input logic [8:0] x);
    ba0 = {y[2:0], x[8:1] + {7'd0, x[0]}};
  endfunction
  function automatic logic [10:0] ba1(input logic [8:0] y, input logic [8:0] x);
    ba1 = {y[2:0], x[8:1]};
  endfunction

  // the screen door: the pixel is drawn where B(x, y) < transparency / 2
  function automatic logic door(input logic [1:0] y, input logic [1:0] x, input logic inv, input logic [4:0] tr);
    logic [3:0] b;
    case ({y, x})
      4'h0: b = 4'd0;  4'h1: b = 4'd8;  4'h2: b = 4'd2;  4'h3: b = 4'd10;
      4'h4: b = 4'd12; 4'h5: b = 4'd4;  4'h6: b = 4'd14; 4'h7: b = 4'd6;
      4'h8: b = 4'd3;  4'h9: b = 4'd11; 4'ha: b = 4'd1;  4'hb: b = 4'd9;
      4'hc: b = 4'd15; 4'hd: b = 4'd7;  4'he: b = 4'd13; default: b = 4'd5;
    endcase
    if (inv) b = ~b;
    door = b < tr[4:1];
  endfunction

  wire [8:0] a_x1 = a_x + 9'd1;
  /* verilator lint_on UNUSEDSIGNAL */
  wire [1:0] a_door = (TRANS == 1 && a_stip) ?
                      {door(a_y[1:0], a_x1[1:0], a_pinv, a_tr), door(a_y[1:0], a_x[1:0], a_pinv, a_tr)} : 2'b11;

  // ---- C: forward, compare --------------------------------------------------------
  wire [23:0] r0 = (w1v0 && c_m10) ? w1d0 : (w2v0 && c_m20) ? w2d0 : c_qv0 ? c_q0 : ZCLR;
  wire [23:0] r1 = (w1v1 && c_m11) ? w1d1 : (w2v1 && c_m21) ? w2d1 : c_qv1 ? c_q1 : ZCLR;
  // the bank's own lane
  // (the lane select is made as C loads, from B's x: c_x[0] through it into the compare and the
  // M10K's write enable was -0.79 ns at 100 MHz; study R104)
  wire        cx0 = c_x[0];
  logic [23:0] cz_b0, cz_b1;
  logic        con_b0, con_b1;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && c_v && {cz_b0, cz_b1, con_b0, con_b1} != {cx0 ? c_z1 : c_z0, cx0 ? c_z0 : c_z1, cx0 ? c_on[1] : c_on[0], cx0 ? c_on[0] : c_on[1]})
      $fatal(1, "zband: the lane select registered");
`endif
  // the compare against each source side by side, chosen as r0 / r1 are (study R109: the forward's
  // select ran into the 24-bit compare and on into the block RAM's write enable)
  function automatic logic zcmp(input logic [23:0] a, input logic [23:0] b);
    zcmp = OOW ? (a >= b) : (a <= b);
  endfunction
  wire        n0 = (w1v0 && c_m10) ? zcmp(cz_b0, w1d0) : (w2v0 && c_m20) ? zcmp(cz_b0, w2d0) :
                   c_qv0 ? zcmp(cz_b0, c_q0) : zcmp(cz_b0, ZCLR);
  wire        n1 = (w1v1 && c_m11) ? zcmp(cz_b1, w1d1) : (w2v1 && c_m21) ? zcmp(cz_b1, w2d1) :
                   c_qv1 ? zcmp(cz_b1, c_q1) : zcmp(cz_b1, ZCLR);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && c_v && {n0, n1} != {zcmp(cz_b0, r0), zcmp(cz_b1, r1)}) $fatal(1, "zband: the compares side by side");
`endif
  wire        pb0 = c_v && con_b0 && n0;
  wire        pb1 = c_v && con_b1 && n1;
  wire [1:0]  c_pass = cx0 ? {pb0, pb1} : {pb1, pb0};
  wire        cw0 = pb0 && c_zk == 2'd1;
  wire        cw1 = pb1 && c_zk == 2'd1;
  wire        c_push = adv && c_v && c_zk == 2'd2 && (c_pass != 2'b00);

  // ---- RAM ports -----------------------------------------------------------------
  wire sw_go = sw_on && !(sw_left == 12'd1 && c_valid);
  always_comb begin
    re  = a_go;
    ra0 = ba0(a_y, a_x);
    ra1 = ba1(a_y, a_x);
    we0 = 1'b0; we1 = 1'b0;
    wa0 = c_a0; wa1 = c_a1;
    wd0 = {gen, cz_b0}; wd1 = {gen, cz_b1};
    if (vport) begin
      if (!v_bank) begin we0 = 1'b1; wa0 = v_a; wd0 = {v_gen, v_z}; end
      else         begin we1 = 1'b1; wa1 = v_a; wd1 = {v_gen, v_z}; end
    end else if (sw_go) begin
      we0 = 1'b1; we1 = 1'b1; wa0 = swp; wa1 = swp; wd0 = {gen, ZCLR}; wd1 = {gen, ZCLR};
    end else if (adv) begin
      we0 = cw0; we1 = cw1;
    end
  end

  // ---- sequencing ---------------------------------------------------------------
  wire v_take = c_valid;   // the verdict register is always free: it commits every cycle it is full
  wire sb_dec = v_full;

  // (r3d140: the reset only on the flags, counts and pointers; the words load in a block of their own, so the reset
  // is in none of their enables: rst_rast -> cz_b0 / cz_b1 / b_z0 / b_z1 and the rest, ~150 endpoints at -1.1 to
  // -1.3 ns in the 140 MHz full fit. A word loaded while the flags are held in reset is not read before it is loaded
  // again: B and C's through b_v / c_v, O's through o_valid, P's through p_v, the verdict's through v_full, a row's
  // range through its count.)
  always_ff @(posedge clk) begin
    if (rst) begin
      a_v <= 1'b0; b_v <= 1'b0; c_v <= 1'b0; o_valid <= 1'b0;
      w1v0 <= 1'b0; w1v1 <= 1'b0; w2v0 <= 1'b0; w2v1 <= 1'b0;
      gen <= 6'd0; swp <= 11'd0; sw_left <= 12'd2048; sw_on <= 1'b1; gen_ok <= 1'b0;
      pf_wp <= '0; pf_rp <= '0; pf_n <= '0; pf_lane <= 1'b0;
      v_full <= 1'b0; p_v <= 1'b0;
      for (int r = 0; r < 8; r++) sb_n[r] <= 7'd0;
      sb_nz <= '0;
    end else begin
      // the generation's first pair holds in A (a_gen_wait) until this is done
      // A
      if (s_valid && s_ready) begin
        a_v <= 1'b1;
        gen_ok <= 1'b0;
      end else if (a_go) a_v <= 1'b0;
      // the generation step: drain, sweep, step
      if (a_v && a_new && !gen_ok && !b_v && !c_v && !sw_on) begin
        sw_on <= 1'b1; sw_left <= 12'(NSW);
      end
      if (sw_go) begin
        swp <= swp + 11'd1;
        sw_left <= sw_left - 12'd1;
        if (sw_left == 12'd1) begin
          sw_on <= 1'b0;
          if (a_v && a_new) begin gen <= gen + 6'd1; gen_ok <= 1'b1; end
        end
      end
      // B, C, O
      if (adv) begin
        b_v <= a_go;
        c_v <= b_v;
        // the writes C made, for the next two
        w2v0 <= w1v0; w2v1 <= w1v1;
        w1v0 <= cw0;  w1v1 <= cw1;
        o_valid <= c_v && (c_pass != 2'b00);
      end else if (o_ready) o_valid <= 1'b0;
      // a new generation forwards nothing from the old one
      if (sw_go && sw_left == 12'd1) begin w1v0 <= 1'b0; w1v1 <= 1'b0; w2v0 <= 1'b0; w2v1 <= 1'b0; end
      // P, then the pending FIFO
      p_v <= c_push;
      if (p_v) pf_wp <= pf_wp + CA'(1);
      if (v_take) begin
        v_full <= 1'b1;
        if (h_last) begin pf_rp <= pf_rp + CA'(1); pf_lane <= 1'b0; end
        else pf_lane <= 1'b1;
      end else v_full <= 1'b0;
      pf_n <= pf_n + (p_v ? (CA+1)'(1) : '0) - ((v_take && h_last) ? (CA+1)'(1) : '0);
      // scoreboard: up on a push, down on a committed verdict
      for (int r = 0; r < 8; r++) begin
        logic [6:0] n;
        n = sb_n[r];
        if (p_v && p_row == 3'(r)) n = n + {6'd0, p_m[0]} + {6'd0, p_m[1]};
        if (sb_dec && v_row == 3'(r)) n = n - 7'd1;
        sb_n[r] <= n;
        sb_nz[r] <= n != 7'd0;
      end
    end
  end
  always_ff @(posedge clk) begin
    // A
    if (s_valid && s_ready) begin
      a_new <= s_new;
      a_y <= s_y; a_x <= s_x; a_on <= s_on; a_zk <= s_zk;
      a_z0 <= s_z0; a_z1 <= s_z1; a_tag <= s_tag;
      a_i <= pw_i; pw_i <= pw_i + 3'd1;
`ifdef VERILATOR
      a_p0 <= s_p0; a_p1 <= s_p1;
`endif
      a_stip <= s_stip; a_pinv <= s_pinv; a_tr <= s_trans;
      a_xh <= s_x + {8'd0, s_on[1]};
    end
    // B, C, O
    if (adv) begin
      // B's words load on every advance, b_v saying whether A went: a_go (the scoreboard's compares)
      // is in b_v's enable alone, not in the ~200 of B's words. B's old words have gone to C at this
      // edge, and B's words are read only through b_v (ctr_bc, a_gen_wait) or by C as c_v
      b_y <= a_y; b_x <= a_x; b_on <= a_on & a_door; b_zk <= a_zk;
      b_z0 <= a_z0; b_z1 <= a_z1; b_i <= a_i; b_tag <= a_tag;
`ifdef VERILATOR
      b_p0 <= a_p0; b_p1 <= a_p1;
`endif
      b_a0 <= ba0(a_y, a_x); b_a1 <= ba1(a_y, a_x);
      c_y <= b_y; c_x <= b_x; c_on <= b_on; c_zk <= b_zk;
      c_z0 <= b_z0; c_z1 <= b_z1; c_tag <= b_tag;
`ifdef VERILATOR
      co_p0 <= b_p0; co_p1 <= b_p1;
`endif
      cz_b0 <= b_x[0] ? b_z1 : b_z0;    cz_b1 <= b_x[0] ? b_z0 : b_z1;
      con_b0 <= b_x[0] ? b_on[1] : b_on[0]; con_b1 <= b_x[0] ? b_on[0] : b_on[1];
      c_a0 <= b_a0; c_a1 <= b_a1; c_q0 <= q0[23:0]; c_q1 <= q1[23:0];
      c_qv0 <= q0[29:24] == gen; c_qv1 <= q1[29:24] == gen;
      c_m10 <= b_a0 == c_a0; c_m20 <= b_a0 == w1a0;
      c_m11 <= b_a1 == c_a1; c_m21 <= b_a1 == w1a1;
      c_x1 <= b_x + 9'd1;
      // the writes C made, for the next two
      w2d0 <= w1d0; w2d1 <= w1d1;
      w1a0 <= c_a0; w1d0 <= cz_b0;
      w1a1 <= c_a1; w1d1 <= cz_b1;
      o_y <= c_y; o_x <= c_x; o_on <= c_pass; o_p0 <= c_p0; o_p1 <= c_p1; o_tag <= c_tag;
      o_ctr <= c_zk == 2'd2;
    end
    // P
    p_tag <= c_tag; p_e <= {gen, c_y[2:0], c_x, c_pass, c_z1, c_z0}; p_x1 <= c_x1;
    if (v_take) begin
      v_wr   <= c_zw && (h_gen == gen);
      v_gen  <= h_gen;
      v_bank <= h_px[0];
      v_a    <= {h_row, h_px[8:1]};
      v_z    <= h_l ? h_z1 : h_z0;
      v_row  <= h_row;
    end
    // the scoreboard's ranges: they restart when the row's count is (or this cycle becomes) zero; a range is read
    // only through its count (sb_nz), which the reset clears
    for (int r = 0; r < 8; r++) begin
      logic z;
      z = (sb_n[r] == 7'd0) || (sb_dec && v_row == 3'(r) && sb_n[r] == 7'd1);
      if (p_v && p_row == 3'(r)) begin
        sb_lo[r] <= (z || p_lo_q < sb_lo[r]) ? p_lo_q : sb_lo[r];
        sb_hi[r] <= (z || p_hi_q > sb_hi[r]) ? p_hi_q : sb_hi[r];
      end
    end
  end

  always_ff @(posedge clk)
    if (p_v) pf[pf_wp] <= p_e;

`ifdef VERILATOR
  // the ring against the old registers wherever C's words are used (R179)
  always_ff @(posedge clk) if (!rst && c_v && {c_p0, c_p1} != {co_p0, co_p1}) $fatal(1, "zband: the payload ring against C's registers");
`endif
endmodule
