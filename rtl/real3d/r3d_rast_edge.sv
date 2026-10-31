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
// Real3D rasteriser: the edge walker. One (triangle, band) pair in, the
// triangle's spans in that band out, one row a clock. Bit for bit
// sim/real3d/r3d_rast_rtl.cpp rtl_tri::span, which is MAME's poly.h
// render_triangle scan conversion on fixed-point vertices.
//
// In: vertices sorted by y (MAME's order, made upstream on the FP32 y), 16.16
// fixed point, the three edge slopes s = (dx << 20) / dy, truncated toward
// zero, 0 for a flat edge, and the band's rows [b0, b1).
//
// Rows max(r1, b0) .. min(r3, b1, SCR_H) - 1, with r = round(y) by poly.h's
// rule (floor, up when the fraction is above one half). At each row centre
// fy = r + 1/2:
//   start x = x1 + (fy - y1) s13
//   stop  x = x1 + (fy - y1) s12   while r < r2 (that is, fy < y2)
//             x2 + (fy - y2) s23   after
// in 2^-36 px. The first row's values are products of the row offset (up to
// 32 bits: a band can start far below y1) and the slope, two offset bits a
// cycle, stopping when the rest of both offsets is zero: 8 cycles for a
// triangle's own first row, about 13 for a band start, in the edge registers
// before the walk. Every row after adds s << 16. The span is [round(start), round(stop)), swapped if reversed,
// clamped to [0, SCR_W), and sent only if non-empty.
//
// Arithmetic is modulo 2^56. For every row that is sent the row offset is at
// most the edge's own height, so |offset s| <= |dx| 2^20 <= 2^52, and with
// |x 2^20| < 2^51 the true value is under 2^53: the modular one equals it.
// An edge that is never used may wrap harmlessly.
//
// The row is a two-stage pipeline for 100 MHz: the rounds, then the order,
// clamp and test.

module r3d_rast_edge #(
  parameter int SCR_W = 496
) (
  input  logic               clk,
  input  logic               rst,

  input  logic               t_valid,
  output logic               t_ready,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic signed [31:0] t_x1, t_y1, t_x2, t_y2, t_x3, t_y3,   // x3 is carried by the slopes
  input  logic signed [53:0] t_s12, t_s13, t_s23,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic        [8:0]  t_b0,     // band rows [b0, b1), b1 <= SCR_H
  input  logic        [8:0]  t_b1,

  output logic               s_valid,
  input  logic               s_ready,
  output logic [8:0]         s_y,
  output logic [8:0]         s_x0,
  output logic [8:0]         s_x1,

  output logic               busy
);

  localparam int AW = 56;
  localparam logic signed [19:0] W20 = 20'(SCR_W);

  // ---- helpers -----------------------------------------------------------------

  // poly.h round_coordinate on 16.16: floor, plus one if the fraction > 1/2
  function automatic logic signed [16:0] round16(input logic signed [31:0] v);
    logic signed [16:0] ip;
    ip = {v[31], v[31:16]};
    if (v[15] && (v[14:0] != 15'd0)) ip = ip + 17'sd1;
    return ip;
  endfunction

  // the same on a 2^-36 value: 20-bit integer part, plus one if fraction > 1/2
  function automatic logic signed [19:0] round36(input logic signed [AW-1:0] v);
    logic signed [19:0] ip;
    ip = v[AW-1:36];
    if (v[35] && (v[34:0] != 35'd0)) ip = ip + 20'sd1;
    return ip;
  endfunction

  // ---- stage 0: register the pair --------------------------------------------------

  logic               z_full;
  logic signed [31:0] z_x1, z_x2, z_y1, z_y2;
  logic signed [53:0] z_s12, z_s13, z_s23;
  logic signed [16:0] z_r1, z_r2, z_r3;
  logic        [8:0]  z_b0, z_b1;

  logic               m_load;          // stage 0 moves into the walker

  assign t_ready = !z_full || m_load;

  always_ff @(posedge clk) begin
    if (rst) z_full <= 1'b0;
    else begin
      if (m_load) z_full <= 1'b0;
      if (t_valid && t_ready) begin
        z_full <= 1'b1;
        z_x1 <= t_x1; z_x2 <= t_x2; z_y1 <= t_y1; z_y2 <= t_y2;
        z_s12 <= t_s12; z_s13 <= t_s13; z_s23 <= t_s23;
        z_r1 <= round16(t_y1); z_r2 <= round16(t_y2); z_r3 <= round16(t_y3);
        z_b0 <= t_b0; z_b1 <= t_b1;
      end
    end
  end

  // ---- stage 1: first rows, the products into the walk registers, the walk ----------
  // The products accumulate in the edge registers themselves: a pair loads
  // x1 and x2 there, adds two offset bits of each product a cycle, then walks.
  // The product no longer overlaps the previous pair's walk (it did at 50 MHz,
  // in its own 3 x 56-bit accumulators and step copies); stage 0 holds the next
  // pair meanwhile.

  // first row: max(r1, b0); last + 1: min(r3, b1)
  wire signed [16:0] b0s = {8'd0, z_b0};
  wire signed [16:0] b1s = {8'd0, z_b1};
  wire signed [16:0] ys  = (z_r1 > b0s) ? z_r1 : b0s;
  wire signed [16:0] ye  = (z_r3 < b1s) ? z_r3 : b1s;
  wire signed [16:0] ys2 = (z_r2 > ys) ? z_r2 : ys;   // first row of edge 2 -> 3
  // fy - y = (row << 16) + 2^15 - y >= 0, below 2^32
  /* verilator lint_off UNUSEDSIGNAL */
  wire signed [33:0] d1w = {ys[16], ys, 16'h8000} - {{2{z_y1[31]}}, z_y1};
  wire signed [33:0] d2w = {ys2[16], ys2, 16'h8000} - {{2{z_y2[31]}}, z_y2};
  /* verilator lint_on UNUSEDSIGNAL */

  logic                 m_busy;
  logic [31:0]          m_d1, m_d2;
  logic signed [AW-1:0] m_s12, m_s13, m_s23;    // slopes, shifted left as the product proceeds

  logic                 w_act;
  logic signed [AW-1:0] a12, a13, a23;      // edge x at the current row, 2^-36 px
  logic signed [AW-1:0] d12, d13, d23;      // per-row steps, s << 16
  logic signed [16:0]   w_y, w_end, w_r2;
  logic                 w_up;               // row < r2

  function automatic logic signed [AW-1:0] pp2(input logic [1:0] d, input logic signed [AW-1:0] s);
    logic signed [AW-1:0] a, b;
    a = d[0] ? s : '0;
    b = d[1] ? (s <<< 1) : '0;
    return a + b;
  endfunction

  // pipeline: p1 holds the rounds of a row, then the span register s_*
  logic               p1_v;
  logic signed [19:0] p1_rs, p1_re;
  logic        [8:0]  p1_y;
  wire                adv = !s_valid || s_ready;   // the row pipeline moves

  logic w_last;
  assign w_last = (w_y + 17'sd1 >= w_end);
  wire   w_adv  = w_act && adv;
  assign m_load = z_full && !m_busy && (!w_act || (w_adv && w_last));

  wire signed [19:0] lo0 = (p1_rs > p1_re) ? p1_re : p1_rs;
  wire signed [19:0] hi0 = (p1_rs > p1_re) ? p1_rs : p1_re;
  wire signed [19:0] lo  = (lo0 < 20'sd0) ? 20'sd0 : lo0;
  wire signed [19:0] hi  = (hi0 > W20) ? W20 : hi0;

  always_ff @(posedge clk) begin
    if (rst) begin
      m_busy  <= 1'b0;
      w_act   <= 1'b0;
      p1_v    <= 1'b0;
      s_valid <= 1'b0;
    end else begin
      if (adv) begin
        // stage B: order, clamp, test
        s_valid <= p1_v && (lo < hi);
        s_y     <= p1_y;
        s_x0    <= lo[8:0];
        s_x1    <= hi[8:0];
        // stage A: the rounds of the current row
        p1_v  <= w_act;
        p1_y  <= w_y[8:0];
        p1_rs <= round36(a13);
        p1_re <= round36(w_up ? a12 : a23);
      end
      if (w_adv) begin
        w_y  <= w_y + 17'sd1;
        w_up <= (w_y + 17'sd1 < w_r2);
        a13  <= a13 + d13;
        if (w_up) a12 <= a12 + d12;
        else      a23 <= a23 + d23;
        if (w_last) w_act <= 1'b0;
      end
      if (m_busy) begin
        a12   <= a12 + pp2(m_d1[1:0], m_s12);
        a13   <= a13 + pp2(m_d1[1:0], m_s13);
        a23   <= a23 + pp2(m_d2[1:0], m_s23);
        m_s12 <= m_s12 <<< 2;
        m_s13 <= m_s13 <<< 2;
        m_s23 <= m_s23 <<< 2;
        m_d1  <= m_d1 >> 2;
        m_d2  <= m_d2 >> 2;
        if (m_d1[31:2] == 30'd0 && m_d2[31:2] == 30'd0) begin
          m_busy <= 1'b0;
          w_act  <= (w_y < w_end);
        end
      end
      if (m_load) begin
        m_busy <= 1'b1;
        m_d1   <= d1w[31:0];
        m_d2   <= d2w[31:0];
        m_s12  <= {{(AW-54){z_s12[53]}}, z_s12};
        m_s13  <= {{(AW-54){z_s13[53]}}, z_s13};
        m_s23  <= {{(AW-54){z_s23[53]}}, z_s23};
        d12    <= {z_s12[39:0], 16'd0};
        d13    <= {z_s13[39:0], 16'd0};
        d23    <= {z_s23[39:0], 16'd0};
        a12    <= {{(AW-52){z_x1[31]}}, z_x1, 20'd0};
        a13    <= {{(AW-52){z_x1[31]}}, z_x1, 20'd0};
        a23    <= {{(AW-52){z_x2[31]}}, z_x2, 20'd0};
        w_y    <= ys;
        w_up   <= (ys < z_r2);
        w_r2   <= z_r2;
        w_end  <= ye;
      end
    end
  end

  assign busy = z_full || m_busy || w_act || p1_v || s_valid;

endmodule
