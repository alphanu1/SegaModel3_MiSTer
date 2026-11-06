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
// Real3D rasteriser: the depth plane along a span, two pixels a clock (D-28).
// Bit for bit r3d_rast_rtl.cpp at2 and z_word.
//
// A plane is (c, dx, dy) 40-bit mantissas at exponent e: twice its value at
// pixel centre (x + 1/2, y + 1/2) is v2 = 2c + dx (2x + 1) + dy (2y + 1), times
// 2^(e - 41), exact in 42 bits (make_plane bounds |c| < 2^38 and |dx|, |dy| <
// 2^29). Per span, B = 2c + dy (2y + 1) + dx (2 x0 + 1) takes two products on
// one 40 x 10 multiplier, so a span costs at least two cycles; then the two
// lanes hold v2 at x and x + 1 and add 4 dx a clock. Exact: integer adds of
// the same terms.
//
// The Z word: v2 <= 0 gives 0; else with n the leading one, ex = n + e + 86;
// ex <= 0 gives 0, ex >= 255 all ones; otherwise {ex[7:0], the 16 bits below
// the leading one}, truncated. One leading-one search and shift per lane.
//
// Out, per clock: y, the pair's x (even or odd start), two lane valids, the two
// Z words and v2 of each lane (the shading lane's 1/w when Z is on 1/w).

module r3d_rast_zspan (
  input  logic               clk,
  input  logic               rst,

  input  logic               s_valid,
  output logic               s_ready,
  input  logic        [8:0]  s_y,
  input  logic        [8:0]  s_x0,
  input  logic        [8:0]  s_x1,      // exclusive, x0 < x1
  input  logic signed [39:0] s_c, s_dx, s_dy,
  input  logic signed [8:0]  s_e,

  output logic               p_valid,
  input  logic               p_ready,
  output logic        [8:0]  p_y,
  output logic        [8:0]  p_x,       // lane 0's x; lane 1 is x + 1
  output logic        [1:0]  p_on,      // lane valid
  output logic        [23:0] p_z0, p_z1,
  output logic signed [41:0] p_v0, p_v1,
  output logic               p_last     // the span's last pair
);

  // ---- span set-up: two products on one multiplier ------------------------------
  // A holds the span and issues dy (2y + 1), then dx (2 x0 + 1); N forms
  // B = 2c + both products (the products land two and three cycles after the
  // first issue); W walks. A span every two cycles when W keeps up.

  logic               a_full, a_ph;
  logic        [8:0]  a_y, a_x0, a_x1;
  logic signed [39:0] a_c, a_dx, a_dy;
  logic signed [8:0]  a_e;

  logic               n_busy, n_rdy;
  logic signed [41:0] n_acc;
  /* verilator lint_off UNUSEDSIGNAL */
  logic signed [41:0] n_dx;   // [41] is the sign copy
  logic        [8:0]  n_y, n_x, n_x1;
  logic signed [8:0]  n_e;

  logic               w_act;
  logic signed [41:0] w_l0, w_l1, w_d4;
  logic        [8:0]  w_y, w_x, w_x1;
  logic signed [8:0]  w_e;

  logic signed [39:0] mu_a;
  logic        [9:0]  mu_b;
  logic signed [50:0] mu_p;   // the true product fits 42 bits
  /* verilator lint_on UNUSEDSIGNAL */
  logic               mu_v1, mu_ph1, mu_v2, mu_ph2;

  wire issue    = a_full && (a_ph || !n_busy);
  wire out_free = !p_valid || p_ready;
  wire w_adv    = w_act && out_free;

  logic               q_valid, q_last;
  logic        [8:0]  q_y, q_x;
  logic        [1:0]  q_on;
  logic signed [41:0] q_v0, q_v1;
  logic        [6:0]  q_n0, q_n1;   // leading one; 64 when none
  logic signed [8:0]  q_e;
  logic signed [10:0] q_e86, q_tl, q_th;   // e + 86, and the bounds on n: n <= -(e + 86) gives 0, n >= 255 - (e + 86) all ones
  wire w_lastp  = ({1'b0, w_x} + 10'd2 >= {1'b0, w_x1});
  wire w_take   = n_rdy && (!w_act || (w_adv && w_lastp));

  assign s_ready = !a_full;

  always_ff @(posedge clk) begin
    if (rst) begin
      a_full  <= 1'b0;
      n_busy  <= 1'b0;
      n_rdy   <= 1'b0;
      w_act   <= 1'b0;
      mu_v1   <= 1'b0;
      mu_v2   <= 1'b0;
      p_valid <= 1'b0;
      q_valid <= 1'b0;
    end else begin
      // A
      if (s_valid && s_ready) begin
        a_full <= 1'b1; a_ph <= 1'b0;
        a_y <= s_y; a_x0 <= s_x0; a_x1 <= s_x1;
        a_c <= s_c; a_dx <= s_dx; a_dy <= s_dy; a_e <= s_e;
      end else if (issue) begin
        a_ph <= 1'b1;
        if (a_ph) a_full <= 1'b0;
      end
      // the multiplier
      mu_v1  <= issue;
      mu_ph1 <= a_ph;
      mu_a   <= a_ph ? a_dx : a_dy;
      mu_b   <= a_ph ? {a_x0, 1'b1} : {a_y, 1'b1};
      mu_v2  <= mu_v1;
      mu_ph2 <= mu_ph1;
      mu_p   <= mu_a * $signed({1'b0, mu_b});
      // N
      if (issue && !a_ph) begin
        n_busy <= 1'b1;
        n_acc  <= {a_c[39], a_c, 1'b0};
        n_dx   <= {{2{a_dx[39]}}, a_dx};
        n_y <= a_y; n_x <= a_x0; n_x1 <= a_x1; n_e <= a_e;
      end
      if (mu_v2) n_acc <= n_acc + mu_p[41:0];
      if (mu_v2 && mu_ph2) n_rdy <= 1'b1;
      // W, then Q (the leading ones), then the output (the Z words)
      if (p_valid && p_ready) p_valid <= 1'b0;
      if (out_free) begin
        p_valid <= q_valid;
        p_y  <= q_y;  p_x <= q_x;  p_on <= q_on;  p_last <= q_last;
        p_z0 <= zw3(q_v0, q_n0, q_e86, q_tl, q_th);
        p_z1 <= zw3(q_v1, q_n1, q_e86, q_tl, q_th);
`ifdef VERILATOR
        if (q_valid && (zw3(q_v0, q_n0, q_e86, q_tl, q_th) != zw2(q_v0, q_n0, q_e) || zw3(q_v1, q_n1, q_e86, q_tl, q_th) != zw2(q_v1, q_n1, q_e)))
          $fatal(1, "zspan: the Z word's bounds as compares of n");
`endif
        p_v0 <= q_v0;
        p_v1 <= q_v1;
        q_valid <= w_act;
      end
      if (w_adv) begin
        q_y  <= w_y;
        q_x  <= w_x;
        q_on <= {({1'b0, w_x} + 10'd1 < {1'b0, w_x1}), 1'b1};
        q_last <= w_lastp;
        q_v0 <= w_l0;  q_n0 <= lead1(w_l0);
        q_v1 <= w_l1;  q_n1 <= lead1(w_l1);
        q_e  <= w_e;
        q_e86 <= {{2{w_e[8]}}, w_e} + 11'sd86;
        q_tl  <= 11'sd0 - ({{2{w_e[8]}}, w_e} + 11'sd86);
        q_th  <= 11'sd255 - ({{2{w_e[8]}}, w_e} + 11'sd86);
        w_l0 <= w_l0 + w_d4;
        w_l1 <= w_l1 + w_d4;
        w_x  <= w_x + 9'd2;
        if (w_lastp) w_act <= 1'b0;
      end
      if (w_take) begin
        n_busy <= 1'b0;
        n_rdy  <= 1'b0;
        w_act  <= 1'b1;
        w_y <= n_y; w_x <= n_x; w_x1 <= n_x1; w_e <= n_e;
        w_l0 <= n_acc;
        w_l1 <= n_acc + {n_dx[40:0], 1'b0};
        w_d4 <= {n_dx[39:0], 2'b00};
      end
    end
  end

  // ---- the Z word, in two stages ------------------------------------------------
  // lead1: the leading one of a positive value, 64 for zero or negative
  function automatic logic [6:0] lead1(input logic signed [41:0] v);
    logic [6:0] n;
    n = 7'd64;
    for (int i = 0; i < 41; i++) if (v[i]) n = 7'(i);
    if (v[41]) n = 7'd64;
    return n;
  endfunction

  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [23:0] zw2(input logic signed [41:0] v, input logic [6:0] n, input logic signed [8:0] e);
    logic signed [10:0] ex;
    logic [41:0] sh;
    ex = {5'd0, n[5:0]} + {{2{e[8]}}, e} + 11'sd86;
    sh = 42'(v) << (6'd41 - n[5:0]);   // leading one to bit 41
    if (n[6]) zw2 = 24'd0;
    else if (ex <= 11'sd0) zw2 = 24'd0;
    else if (ex >= 11'sd255) zw2 = 24'hffffff;
    else zw2 = {ex[7:0], sh[40:25]};
  endfunction
  // the same with the bounds as compares of n against the span's registers, side by side with the add
  // (study R109: n + e + 86 and its two compares were in series into p_z)
  function automatic logic [23:0] zw3(input logic signed [41:0] v, input logic [6:0] n, input logic signed [10:0] e86,
                                      input logic signed [10:0] tl, input logic signed [10:0] th);
    logic [7:0]  ex;
    logic [41:0] sh;
    ex = 8'({2'd0, n[5:0]} + e86[7:0]);
    sh = 42'(v) << (6'd41 - n[5:0]);
    if (n[6]) zw3 = 24'd0;
    else if ($signed({5'd0, n[5:0]}) <= tl) zw3 = 24'd0;
    else if ($signed({5'd0, n[5:0]}) >= th) zw3 = 24'hffffff;
    else zw3 = {ex, sh[40:25]};
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
