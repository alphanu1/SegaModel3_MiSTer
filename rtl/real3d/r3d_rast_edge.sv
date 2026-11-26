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
// 32 bits: a band can start far below y1) and the slope, PB offset bits a
// cycle, stopping when the rest of both offsets is zero: at PB = 2, 8 cycles
// for a triangle's own first row, about 13 for a band start, in the edge
// registers before the walk. PB = 1 (study R131) takes one bit a cycle through
// the adder each edge register walks with, so the three 3-input adders go. Every row after adds s << 16. The span is [round(start), round(stop)), swapped if reversed,
// clamped to [0, SCR_W), and sent only if non-empty.
//
// Arithmetic is modulo 2^54 (56 before the area130 pass: the same bits, by the bound below). For every row that is sent the row offset is at
// most the edge's own height, so |offset s| <= |dx| 2^20 <= 2^52, and with
// |x 2^20| < 2^51 the true value is under 2^53: the modular one, 54 bits signed, equals it. Every operation on the
// walk's values is an add, a shift left or a product with an offset, all exact modulo 2^54, and the values are read
// only at a row that is sent, by round36.
// An edge that is never used may wrap harmlessly.
//
// The row is a two-stage pipeline for 100 MHz: the rounds, then the order,
// clamp and test. The first rows' compares are made as stage 0 takes the
// pair, side by side, and stage 1 only selects: two muxes and the offsets'
// subtract (R87's -40C path ran from the rounds through two compares, two
// muxes and the subtract). round(y) > b is y > b + 1/2 and round(y) < b is
// y <= b - 1/2 on the 16.16 value, so those compares need no round.

module r3d_rast_edge #(
  parameter int SCR_W = 496,
  parameter int TGW   = 8,        // the pair's tag, carried to its spans (study R131)
  parameter int PB    = 2         // product bits a cycle: 2 (three adders of their own), or 1 on the walk's adders (R131)
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
  input  logic     [TGW-1:0] t_tag,


  output logic               s_valid,
  input  logic               s_ready,
  output logic [8:0]         s_y,
  output logic [8:0]         s_x0,
  output logic [8:0]         s_x1,
  output logic [TGW-1:0]     s_tag,
  output logic [TGW-1:0]     s_tag_n,   // the tag s_tag holds after this edge (a registered read's address)

  output logic               busy
);

  localparam int AW = 54;
  localparam logic signed [19:0] W20 = 20'(SCR_W);

  // ---- helpers -----------------------------------------------------------------

  // poly.h round_coordinate on 16.16: floor, plus one if the fraction > 1/2
  function automatic logic signed [16:0] round16(input logic signed [31:0] v);
    logic signed [16:0] ip;
    ip = {v[31], v[31:16]};
    if (v[15] && (v[14:0] != 15'd0)) ip = ip + 17'sd1;
    return ip;
  endfunction

  // the same on a 2^-36 value: 20-bit integer part, plus one if fraction > 1/2. The walk registers hold the edge's
  // x plus RB = 2^35 - 1 (r3d140: the round's 20-bit carry chain was between a12 and p1_re, the front's worst path
  // at 140 MHz), and floor((v + 2^35 - 1) / 2^36) is the integer part plus one exactly when the fraction is above
  // 2^35, so the round of the biased value is its top bits. |v| < 2^52 + 2^51 (the bound above), so v + RB does
  // not wrap 54 bits.
  function automatic logic signed [19:0] round36(input logic signed [AW-1:0] vb);
    return {{(56-AW){vb[AW-1]}}, vb[AW-1:36]};   // (AW 54: the 18-bit integer part sign-extended to 20)
  endfunction
`ifdef VERILATOR
  function automatic logic signed [19:0] round36_ref(input logic signed [AW-1:0] v);
    logic signed [19:0] ip;
    ip = {{(56-AW){v[AW-1]}}, v[AW-1:36]};
    if (v[35] && (v[34:0] != 35'd0)) ip = ip + 20'sd1;
    return ip;
  endfunction
`endif
  localparam logic signed [AW-1:0] RB = AW'((64'd1 << 35) - 64'd1);

  // ---- stage 0: register the pair --------------------------------------------------

  logic               z_full;
  logic signed [31:0] z_y1, z_y2;
  logic signed [33:0] z_x1b, z_x2b;   // x1, x2 with the round's bias: {sext(x), 20'd0} + RB = {sext34(x) + 0x7FFF, 20'hFFFFF}
  logic signed [53:0] z_s12, z_s13, z_s23;
  logic signed [16:0] z_r1, z_r2, z_r3;
  logic        [8:0]  z_b0, z_b1;
  logic     [TGW-1:0] z_tag, w_tag, p1_tag;
  logic               z_c1, z_c2, z_c21, z_c3;   // r1 > b0, r2 > b0, r2 > r1, r3 < b1

  wire signed [16:0] i_r1 = round16(t_y1), i_r2 = round16(t_y2), i_r3 = round16(t_y3);
  wire signed [32:0] h_b0 = {8'd0, t_b0, 16'h8000};                    // b0 + 1/2
  wire signed [32:0] h_b1 = {8'd0, t_b1, 16'd0} - 33'sh8000;            // b1 - 1/2
  wire signed [32:0] y1x = {t_y1[31], t_y1}, y2x = {t_y2[31], t_y2}, y3x = {t_y3[31], t_y3};

  logic               m_load;          // stage 0 moves into the walker

  assign t_ready = !z_full || m_load;

  // (r3d140: the reset only on the flags; the words load in blocks of their own, so the reset is in none of their
  // enables: rst_rast -> the walk's and the product's registers, ~300 endpoints at -0.8 to -1.2 ns in the 140 MHz full
  // fit. A word loaded while the flags are held in reset is never read: each is loaded again before its flag is set.)
  always_ff @(posedge clk) begin
    if (rst) z_full <= 1'b0;
    else begin
      if (m_load) z_full <= 1'b0;
      if (t_valid && t_ready) z_full <= 1'b1;
    end
  end
  always_ff @(posedge clk) begin
    begin
      if (t_valid && t_ready) begin
        z_x1b <= {{2{t_x1[31]}}, t_x1} + 34'sh7FFF; z_x2b <= {{2{t_x2[31]}}, t_x2} + 34'sh7FFF; z_y1 <= t_y1; z_y2 <= t_y2;
        z_s12 <= t_s12; z_s13 <= t_s13; z_s23 <= t_s23;
        z_r1 <= i_r1; z_r2 <= i_r2; z_r3 <= i_r3; z_b0 <= t_b0; z_b1 <= t_b1; z_tag <= t_tag;
        z_c1  <= y1x > h_b0;
        z_c2  <= y2x > h_b0;
        z_c21 <= i_r2 > i_r1;
        z_c3  <= y3x <= h_b1;
      end
    end
  end

  // ---- stage 1: first rows, the products into the walk registers, the walk ----------
  // The products accumulate in the edge registers themselves: a pair loads
  // x1 and x2 there, adds two offset bits of each product a cycle, then walks.
  // The product no longer overlaps the previous pair's walk (it did at 50 MHz,
  // in its own 3 x 56-bit accumulators and step copies); stage 0 holds the next
  // pair meanwhile.

  // first row: max(r1, b0); last + 1: min(r3, b1); edge 2 -> 3's first row max(r2, ys)
  wire signed [16:0] b0s = {8'd0, z_b0};
  wire signed [16:0] b1s = {8'd0, z_b1};
  wire               up0 = z_c2 && z_c21;          // r2 > max(r1, b0): the first row is above r2
  wire signed [16:0] ys  = z_c1 ? z_r1 : b0s;
  wire signed [16:0] ye  = z_c3 ? z_r3 : b1s;
  wire signed [16:0] ys2 = up0 ? z_r2 : ys;
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
  // r3d140: an output skid. The row pipeline (and the walk, and m_load behind it) moves on a register, !k_v, not on
  // !s_valid || s_ready: that select fanned out into some 250 enables (the walk's three 54-bit registers among
  // them), the front's widest failing family in the 130 MHz full fits (s_valid -> a12, -1.60 ns). When the output
  // is held, the row in stage B goes to the skid (k_*) and the pipeline stops a clock later; the output takes the
  // skid first. The spans leave in the same order, each no later than before.
  logic               k_v;
  logic        [8:0]  k_y, k_x0, k_x1;
  logic     [TGW-1:0] k_tag;
  wire                adv  = !k_v;                     // the row pipeline moves
  wire                o_fr = !s_valid || s_ready;      // the output register takes a word

  // w_last as a register (study R109): w_y + 1 >= w_end was an add and a compare in front of m_load,
  // which loads every walk and product register. Kept as the compare of the registers as they will
  // stand: the pair's first row on m_load, the next row on a step.
  logic w_last;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && w_act && w_last != (w_y + 17'sd1 >= w_end)) $fatal(1, "edge: w_last registered");
`endif
  wire   w_adv  = w_act && adv;
  assign m_load = z_full && !m_busy && (!w_act || (w_adv && w_last));

  // stage B's compares side by side (study R109): the order, both clamps and lo < hi were three 20-bit
  // compares in series into s_valid. With mn = min(rs, re) and mx = max: lo = max(mn, 0), hi = min(mx, W),
  // and max(mn, 0) < min(mx, W) is mn < mx, mn < W and mx > 0 (W > 0): rs != re, either below W, either
  // above 0.
  wire               rgt = p1_rs > p1_re;
  wire               rs0 = p1_rs[19], re0 = p1_re[19];          // below 0
  wire               rsw = p1_rs > W20, rew = p1_re > W20;
  wire        [8:0]  lo  = rgt ? (re0 ? 9'd0 : p1_re[8:0]) : (rs0 ? 9'd0 : p1_rs[8:0]);
  wire        [8:0]  hi  = rgt ? (rsw ? W20[8:0] : p1_rs[8:0]) : (rew ? W20[8:0] : p1_re[8:0]);
  wire               ne  = p1_rs != p1_re;
  wire               bw  = (p1_rs < W20) || (p1_re < W20);
  wire               a0  = (p1_rs > 20'sd0) || (p1_re > 20'sd0);
`ifdef VERILATOR
  /* verilator lint_off UNUSEDSIGNAL */
  wire signed [19:0] lo0c = rgt ? p1_re : p1_rs;
  wire signed [19:0] hi0c = rgt ? p1_rs : p1_re;
  wire signed [19:0] loc  = (lo0c < 20'sd0) ? 20'sd0 : lo0c;
  wire signed [19:0] hic  = (hi0c > W20) ? W20 : hi0c;
  /* verilator lint_on UNUSEDSIGNAL */
  always_ff @(posedge clk)
    if (!rst && p1_v && ((loc < hic) != (ne && bw && a0) || ((loc < hic) && (lo != loc[8:0] || hi != hic[8:0]))))
      $fatal(1, "edge: stage B side by side");
`endif

  // the flags, under the reset
  always_ff @(posedge clk) begin
    if (rst) begin
      m_busy  <= 1'b0;
      w_act   <= 1'b0;
      p1_v    <= 1'b0;
      s_valid <= 1'b0;
      k_v     <= 1'b0;
    end else begin
      if (o_fr) begin
        if (k_v) begin s_valid <= 1'b1; k_v <= 1'b0; end
        else s_valid <= p1_v && ne && bw && a0;
      end else if (!k_v && p1_v && ne && bw && a0) k_v <= 1'b1;
      if (adv) p1_v <= w_act;
      if (w_adv && w_last) w_act <= 1'b0;
      if (m_busy && (m_d1 >> PB) == 32'd0 && (m_d2 >> PB) == 32'd0) begin
        m_busy <= 1'b0;
        w_act  <= (w_y < w_end);
      end
      if (m_load) m_busy <= 1'b1;
    end
  end
  // the words, without it
  always_ff @(posedge clk) begin
    // stage B (order, clamp, test) into the output, or into the skid while the output is held
    if (o_fr) begin
      if (k_v) begin
        s_y <= k_y; s_x0 <= k_x0; s_x1 <= k_x1; s_tag <= k_tag;
      end else begin
        s_y <= p1_y; s_x0 <= lo; s_x1 <= hi; s_tag <= p1_tag;
      end
    end else if (!k_v && p1_v && ne && bw && a0) begin
      k_y <= p1_y; k_x0 <= lo; k_x1 <= hi; k_tag <= p1_tag;
    end
    if (adv) begin
      p1_tag  <= w_tag;
      // stage A: the rounds of the current row
      p1_y  <= w_y[8:0];
      p1_rs <= round36(a13);
      p1_re <= round36(w_up ? a12 : a23);
    end
    if (PB == 1) begin
      // one adder an edge register (study R131): the walk's step, or one offset bit of the product
      logic e12, e13, e23;
      e12 = (w_adv && w_up) || m_busy; e13 = w_adv || m_busy; e23 = (w_adv && !w_up) || m_busy;
      if (e12) a12 <= a12 + (m_busy ? (m_d1[0] ? m_s12 : '0) : d12);
      if (e13) a13 <= a13 + (m_busy ? (m_d1[0] ? m_s13 : '0) : d13);
      if (e23) a23 <= a23 + (m_busy ? (m_d2[0] ? m_s23 : '0) : d23);
    end else begin
      if (w_adv) begin
        a13  <= a13 + d13;
        if (w_up) a12 <= a12 + d12;
        else      a23 <= a23 + d23;
      end
      if (m_busy) begin
        a12   <= a12 + pp2(m_d1[1:0], m_s12);
        a13   <= a13 + pp2(m_d1[1:0], m_s13);
        a23   <= a23 + pp2(m_d2[1:0], m_s23);
      end
    end
    if (w_adv) begin
      w_y  <= w_y + 17'sd1;
      w_last <= (w_y + 17'sd2 >= w_end);
      w_up <= (w_y + 17'sd1 < w_r2);
    end
    if (m_busy) begin
      m_s12 <= m_s12 <<< PB;
      m_s13 <= m_s13 <<< PB;
      m_s23 <= m_s23 <<< PB;
      m_d1  <= m_d1 >> PB;
      m_d2  <= m_d2 >> PB;
    end
    if (m_load) begin
      m_d1   <= d1w[31:0];
      m_d2   <= d2w[31:0];
      m_s12  <= z_s12;                    // (AW 54: the slopes as they come)
      m_s13  <= z_s13;
      m_s23  <= z_s23;
      d12    <= {z_s12[AW-17:0], 16'd0};
      d13    <= {z_s13[AW-17:0], 16'd0};
      d23    <= {z_s23[AW-17:0], 16'd0};
      a12    <= {{(AW-54){z_x1b[33]}}, z_x1b, 20'hFFFFF};   // x1 2^36 + RB
      a13    <= {{(AW-54){z_x1b[33]}}, z_x1b, 20'hFFFFF};
      a23    <= {{(AW-54){z_x2b[33]}}, z_x2b, 20'hFFFFF};
      w_y    <= ys;
      w_last <= (ys + 17'sd1 >= ye);
      w_up   <= up0;
      w_r2   <= z_r2;
      w_end  <= ye;
      w_tag  <= z_tag;
    end
  end

  assign busy = z_full || m_busy || w_act || p1_v || s_valid || k_v;
  assign s_tag_n = !o_fr ? s_tag : k_v ? k_tag : p1_tag;
`ifdef VERILATOR
  logic [TGW-1:0] stn_q;
  logic           rst_q;
  always_ff @(posedge clk) begin
    stn_q <= s_tag_n; rst_q <= rst;
    if (!rst && !rst_q && stn_q != s_tag) $fatal(1, "edge: s_tag_n is not the next s_tag");
  end
  // the biased walk registers' round against the round of the true value, on every row that is sent
  logic signed [19:0] p1_rs_ref, p1_re_ref;
  always_ff @(posedge clk) begin
    if (adv) begin p1_rs_ref <= round36_ref(a13 - RB); p1_re_ref <= round36_ref((w_up ? a12 : a23) - RB); end
    if (!rst && p1_v && ne && bw && a0 && (p1_rs != p1_rs_ref || p1_re != p1_re_ref)) $fatal(1, "edge: the biased round");
  end
`endif

endmodule
