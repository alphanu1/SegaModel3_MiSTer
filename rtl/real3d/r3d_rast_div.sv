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
// Real3D rasteriser: the three edge slopes of a triangle for r3d_rast_edge,
// s = (dx << 20) / dy, truncated toward zero, 0 when dy is 0. Bit for bit
// sim/real3d/r3d_rast_rtl.cpp rtl_tri::setup.
//
// In: the y-sorted 16.16 vertices, so every dy >= 0. Three r3d_div_lane
// side by side, two quotient bits a cycle. The dividend {0, |dx|, 20 zeros}
// is 54 bits; when dy >= 2^16 (an edge at least a pixel tall) its top 16 bits
// cannot make a quotient bit and go straight into the remainder, so the lane
// runs 19 cycles, else 27. No normaliser and no barrel shift: the 50 MHz
// version's three normalisers (two 53-bit shifts and two leading-one searches
// each) were two thirds of the front's area.
//
// The vertices pass through beside the quotients, so this block sits directly
// in front of the edge walker. The pair's tag rides with them (study R131).

module r3d_rast_div #(
  // PRE = 1 (study R109; the rasteriser's own instance): the inputs hold for at least a cycle before
  // they are taken (r3d_rast's triangle registers are written beats before the record ends and hold
  // until t_go), so the differences' magnitudes and the flags are registered and only taken at go: the
  // subtract, the negate and the select were in front of the lanes' n_r.
  parameter bit PRE = 1'b0,
  parameter int TGW = 8
) (
  input  logic               clk,
  input  logic               rst,

  input  logic               i_valid,
  output logic               i_ready,
  input  logic signed [31:0] i_x1, i_y1, i_x2, i_y2, i_x3, i_y3,
  input  logic     [TGW-1:0] i_tag,

  output logic               o_valid,
  input  logic               o_ready,
  output logic signed [31:0] o_x1, o_y1, o_x2, o_y2, o_x3, o_y3,
  output logic     [TGW-1:0] o_tag,
  output logic signed [53:0] o_s12, o_s13, o_s23   // held while o_valid
);

  logic signed [32:0] dxs [3];
  logic        [31:0] dys [3];
  assign dxs[0] = {i_x2[31], i_x2} - {i_x1[31], i_x1};   // 1 -> 2
  assign dys[0] = i_y2 - i_y1;
  assign dxs[1] = {i_x3[31], i_x3} - {i_x1[31], i_x1};   // 1 -> 3
  assign dys[1] = i_y3 - i_y1;
  assign dxs[2] = {i_x3[31], i_x3} - {i_x2[31], i_x2};   // 2 -> 3
  assign dys[2] = i_y3 - i_y2;

  logic        busy;
  logic [2:0]  l_busy;
  logic [53:0] l_q [3];
  logic [2:0]  neg;
  logic [2:0]  zero;   // dy = 0: the slope is 0

  // the lanes load straight from the inputs on the accepting edge; the slopes
  // are formed from the quotients the lanes hold until the next start
  wire         go = i_valid && i_ready;
  logic [53:0] g_n [3];
  logic [31:0] g_r [3];
  logic [4:0]  g_k [3];

  assign i_ready = !busy && !o_valid;

  always_comb begin
    for (int j = 0; j < 3; j++) begin
      logic [32:0] a;
      logic [53:0] nn;
      a = dxs[j][32] ? (33'd0 - dxs[j]) : dxs[j];
      nn = {1'b0, a, 20'd0};
      if (dys[j][31:16] != 16'd0) begin
        g_r[j] = {16'd0, nn[53:38]};
        g_n[j] = {nn[37:0], 16'd0};
        g_k[j] = 5'd19;
      end else begin
        g_r[j] = 32'd0;
        g_n[j] = nn;
        g_k[j] = (dys[j] == 32'd0) ? 5'd0 : 5'd27;
      end
    end
  end

  // the loads as the lanes take them: straight from the inputs, or (PRE) the same two cycles on
  logic [53:0] h_n [3];
  logic [31:0] h_r [3], h_d [3];
  logic [4:0]  h_k [3];
  logic [2:0]  h_neg, h_zero;
  generate if (PRE) begin : g_pre
    // one registered stage: the magnitudes, the divisors and the flags; the lanes' loads are a 2:1 select
    // of these at go
    logic [32:0] q_a [3];
    logic [31:0] q_dy [3];
    logic [2:0]  q_neg, q_zero, q_hi;
    always_ff @(posedge clk)
      for (int j = 0; j < 3; j++) begin
        q_a[j] <= dxs[j][32] ? (33'd0 - dxs[j]) : dxs[j];
        q_dy[j] <= dys[j]; q_neg[j] <= dxs[j][32]; q_zero[j] <= dys[j] == 32'd0; q_hi[j] <= dys[j][31:16] != 16'd0;
      end
    always_comb for (int j = 0; j < 3; j++) begin
      logic [53:0] nn;
      nn = {1'b0, q_a[j], 20'd0};
      h_n[j] = q_hi[j] ? {nn[37:0], 16'd0} : nn;
      h_r[j] = q_hi[j] ? {16'd0, nn[53:38]} : 32'd0;
      h_k[j] = q_hi[j] ? 5'd19 : q_zero[j] ? 5'd0 : 5'd27;
      h_d[j] = q_dy[j]; h_neg[j] = q_neg[j]; h_zero[j] = q_zero[j];
    end
`ifdef VERILATOR
    always_ff @(posedge clk)
      if (!rst && go) for (int j = 0; j < 3; j++)
        if ({h_n[j], h_r[j], h_k[j], h_d[j], h_neg[j], h_zero[j]} != {g_n[j], g_r[j], g_k[j], dys[j], dxs[j][32], dys[j] == 32'd0})
          $fatal(1, "rast_div: the inputs did not hold a cycle before go");
`endif
  end else begin : g_nopre
    always_comb for (int j = 0; j < 3; j++) begin
      h_n[j] = g_n[j]; h_r[j] = g_r[j]; h_k[j] = g_k[j]; h_d[j] = dys[j]; h_neg[j] = dxs[j][32]; h_zero[j] = dys[j] == 32'd0;
    end
  end endgenerate

  /* verilator lint_off UNUSEDSIGNAL */
  logic [31:0] l_r [3];   // the remainder: the set-up's divides use it
  /* verilator lint_on UNUSEDSIGNAL */
  genvar gj;
  generate for (gj = 0; gj < 3; gj++) begin : g_lane
    r3d_div_lane u_lane (
      .clk, .rst,
      .start(go), .n(h_n[gj]), .r0(h_r[gj]), .d(h_d[gj]), .nb2(h_k[gj]),
      .busy(l_busy[gj]), .q(l_q[gj]), .r(l_r[gj])
    );
  end endgenerate

  assign o_s12 = zero[0] ? 54'sd0 : neg[0] ? (54'sd0 - l_q[0]) : l_q[0];
  assign o_s13 = zero[1] ? 54'sd0 : neg[1] ? (54'sd0 - l_q[1]) : l_q[1];
  assign o_s23 = zero[2] ? 54'sd0 : neg[2] ? (54'sd0 - l_q[2]) : l_q[2];

  // the triangle's fields follow the inputs while the divider takes one (i_ready), so they hold the accepted
  // triangle's from go until the slopes are taken; the edge reads them only with o_valid. They were behind go,
  // the edge's state (study R131's FOV) through t_go: 200 enables.
  always_ff @(posedge clk)
    if (i_ready) begin
      o_x1 <= i_x1; o_y1 <= i_y1; o_x2 <= i_x2; o_y2 <= i_y2; o_x3 <= i_x3; o_y3 <= i_y3; o_tag <= i_tag;
      for (int j = 0; j < 3; j++) begin
        neg[j]  <= h_neg[j];
        zero[j] <= h_zero[j];
      end
    end
`ifdef VERILATOR
  logic [31:0] c_x1, c_y1, c_x2, c_y2, c_x3, c_y3;
  logic [TGW-1:0] c_tag;
  logic [2:0] c_neg, c_zero;
  always_ff @(posedge clk) begin
    if (go) begin
      c_x1 <= i_x1; c_y1 <= i_y1; c_x2 <= i_x2; c_y2 <= i_y2; c_x3 <= i_x3; c_y3 <= i_y3; c_tag <= i_tag;
      c_neg <= h_neg; c_zero <= h_zero;
    end
    if (!rst && o_valid && {o_x1, o_y1, o_x2, o_y2, o_x3, o_y3, o_tag, neg, zero} != {c_x1, c_y1, c_x2, c_y2, c_x3, c_y3, c_tag, c_neg, c_zero})
      $fatal(1, "rast_div: the fields followed the inputs");
  end
`endif

  always_ff @(posedge clk) begin
    if (rst) begin
      busy    <= 1'b0;
      o_valid <= 1'b0;
    end else begin
      if (o_valid && o_ready) o_valid <= 1'b0;
      if (go) begin
        busy    <= 1'b1;
      end else if (busy) begin
        if (l_busy == 3'b000) begin
          busy    <= 1'b0;
          o_valid <= 1'b1;
        end
      end
    end
  end

endmodule
