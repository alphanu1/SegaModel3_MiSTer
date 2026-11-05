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
  parameter int CD    = 32        // pending contour pairs
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
  logic      [PW-1:0] a_p0, a_p1;
  logic      [TW-1:0] a_tag;
  logic               a_stip, a_pinv;
  logic        [4:0]  a_tr;

  logic               b_v, c_v;
  logic        [8:0]  b_y, b_x, c_y, c_x;
  logic        [1:0]  b_on, b_zk, c_on, c_zk;   // b_on, c_on: after the screen door
  logic        [23:0] b_z0, b_z1, c_z0, c_z1;
  logic      [PW-1:0] b_p0, b_p1, c_p0, c_p1;
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
  wire [8:0] a_xh  = a_x + {8'd0, a_on[1]};
  wire       sb_hit = (sb_n[a_row] != 7'd0) && !(a_xh < sb_lo[a_row] || a_x > sb_hi[a_row]);
  wire       ctr_bc = (b_v && b_zk == 2'd2 && b_tag != a_tag) || (c_v && c_zk == 2'd2 && c_tag != a_tag) ||
                      (p_v && p_tag != a_tag);
  wire       pf_full = (a_zk == 2'd2) && (pf_n > (CA+1)'(CD - 4));
  wire       a_gen_wait = a_new && (b_v || c_v || !gen_ok);
  wire       a_go   = a_v && adv && !sw_on && !sb_hit && !ctr_bc && !pf_full && !a_gen_wait;
  assign s_ready = !a_v || a_go;

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
  wire        n0 = OOW ? (cz_b0 >= r0) : (cz_b0 <= r0);
  wire        n1 = OOW ? (cz_b1 >= r1) : (cz_b1 <= r1);
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

  always_ff @(posedge clk) begin
    if (rst) begin
      a_v <= 1'b0; b_v <= 1'b0; c_v <= 1'b0; o_valid <= 1'b0;
      w1v0 <= 1'b0; w1v1 <= 1'b0; w2v0 <= 1'b0; w2v1 <= 1'b0;
      gen <= 6'd0; swp <= 11'd0; sw_left <= 12'd2048; sw_on <= 1'b1; gen_ok <= 1'b0;
      pf_wp <= '0; pf_rp <= '0; pf_n <= '0; pf_lane <= 1'b0;
      v_full <= 1'b0; p_v <= 1'b0;
      for (int r = 0; r < 8; r++) sb_n[r] <= 7'd0;
    end else begin
      // the generation's first pair holds in A (a_gen_wait) until this is done
      // A
      if (s_valid && s_ready) begin
        a_v <= 1'b1; a_new <= s_new;
        a_y <= s_y; a_x <= s_x; a_on <= s_on; a_zk <= s_zk;
        a_z0 <= s_z0; a_z1 <= s_z1; a_p0 <= s_p0; a_p1 <= s_p1; a_tag <= s_tag;
        a_stip <= s_stip; a_pinv <= s_pinv; a_tr <= s_trans;
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
        if (a_go) begin
          b_y <= a_y; b_x <= a_x; b_on <= a_on & a_door; b_zk <= a_zk;
          b_z0 <= a_z0; b_z1 <= a_z1; b_p0 <= a_p0; b_p1 <= a_p1; b_tag <= a_tag;
          b_a0 <= ba0(a_y, a_x); b_a1 <= ba1(a_y, a_x);
        end
        c_v <= b_v;
        c_y <= b_y; c_x <= b_x; c_on <= b_on; c_zk <= b_zk;
        c_z0 <= b_z0; c_z1 <= b_z1; c_p0 <= b_p0; c_p1 <= b_p1; c_tag <= b_tag;
        cz_b0 <= b_x[0] ? b_z1 : b_z0;    cz_b1 <= b_x[0] ? b_z0 : b_z1;
        con_b0 <= b_x[0] ? b_on[1] : b_on[0]; con_b1 <= b_x[0] ? b_on[0] : b_on[1];
        c_a0 <= b_a0; c_a1 <= b_a1; c_q0 <= q0[23:0]; c_q1 <= q1[23:0];
        c_qv0 <= q0[29:24] == gen; c_qv1 <= q1[29:24] == gen;
        c_m10 <= b_a0 == c_a0; c_m20 <= b_a0 == w1a0;
        c_m11 <= b_a1 == c_a1; c_m21 <= b_a1 == w1a1;
        c_x1 <= b_x + 9'd1;
        // the writes C made, for the next two
        w2v0 <= w1v0; w2d0 <= w1d0;
        w2v1 <= w1v1; w2d1 <= w1d1;
        w1v0 <= cw0;  w1a0 <= c_a0; w1d0 <= cz_b0;
        w1v1 <= cw1;  w1a1 <= c_a1; w1d1 <= cz_b1;
        o_valid <= c_v && (c_pass != 2'b00);
        o_y <= c_y; o_x <= c_x; o_on <= c_pass; o_p0 <= c_p0; o_p1 <= c_p1; o_tag <= c_tag;
        o_ctr <= c_zk == 2'd2;
      end else if (o_ready) o_valid <= 1'b0;
      // a new generation forwards nothing from the old one
      if (sw_go && sw_left == 12'd1) begin w1v0 <= 1'b0; w1v1 <= 1'b0; w2v0 <= 1'b0; w2v1 <= 1'b0; end
      // P, then the pending FIFO
      p_v <= c_push;
      p_tag <= c_tag; p_e <= {gen, c_y[2:0], c_x, c_pass, c_z1, c_z0}; p_x1 <= c_x1;
      if (p_v) pf_wp <= pf_wp + CA'(1);
      if (v_take) begin
        v_full <= 1'b1;
        v_wr   <= c_zw && (h_gen == gen);
        v_gen  <= h_gen;
        v_bank <= h_px[0];
        v_a    <= {h_row, h_px[8:1]};
        v_z    <= h_l ? h_z1 : h_z0;
        v_row  <= h_row;
        if (h_last) begin pf_rp <= pf_rp + CA'(1); pf_lane <= 1'b0; end
        else pf_lane <= 1'b1;
      end else v_full <= 1'b0;
      pf_n <= pf_n + (p_v ? (CA+1)'(1) : '0) - ((v_take && h_last) ? (CA+1)'(1) : '0);
      // scoreboard: up on a push, down on a committed verdict; the range
      // restarts when the row's count is (or this cycle becomes) zero
      for (int r = 0; r < 8; r++) begin
        logic [6:0] n;
        logic       z;
        n = sb_n[r];
        z = (sb_n[r] == 7'd0) || (sb_dec && v_row == 3'(r) && sb_n[r] == 7'd1);
        if (p_v && p_row == 3'(r)) begin
          n = n + {6'd0, p_m[0]} + {6'd0, p_m[1]};
          sb_lo[r] <= (z || p_lo_q < sb_lo[r]) ? p_lo_q : sb_lo[r];
          sb_hi[r] <= (z || p_hi_q > sb_hi[r]) ? p_hi_q : sb_hi[r];
        end
        if (sb_dec && v_row == 3'(r)) n = n - 7'd1;
        sb_n[r] <= n;
      end
    end
  end

  always_ff @(posedge clk)
    if (p_v) pf[pf_wp] <= p_e;

endmodule
