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
// Real3D rasteriser: the bilinear texel, four texels a clock (D-28). Bit for
// bit r3d_rast_rtl.cpp fetch_texel: MAME's TEX_FETCH_BILINEAR addressing, the
// eight formats of get_texture and its SSE2 filter.
//
// Address. u1 = (u >> 8) & m, u2 = (u1 + 1) & m, m = ((mirror ? 64 : 32) << w)
// - 1; a coordinate in the upper half of a mirrored axis reads the lower half
// reflected, 2p - 1 - u. The texel's word is A = ((page 1024 + ty 32 + vv)
// << 11) + tx 32 + uu, mod 2^22: linear, so a texture past x = 2047 goes on
// into the next row, as the model reads it.
//
// Banks by (A[11], A[0]). The two u neighbours differ by an odd amount or not
// at all, the two v neighbours by an odd multiple of 2048 or not at all, so
// the four corners land in four banks, or two corners share one address. A
// bank's word is at {A[21:12], A[10:1]}. The port is read on en and answers
// the next cycle (an M10K, or the cache in front of the texture RAM).
//
// Filter, per channel with a b the u neighbours on row v1 and c d on v2:
// top = 256 a + (b - a) fu, bot = 256 c + (d - c) fu, then
// o = (256 (top >> 1) + ((bot >> 1) - (top >> 1)) fv) >> 15, the same integers
// as a (256 - fu) + b fu and its v form: three products a channel.
//
// MIP = 1 (study D-33, R67): the texel at mip level i_lv (r3d_rast_interp's
// o_lv), as r3d_rast_rtl.cpp texel_addr4 / fetch_texel_lv read it. Level L
// of the page sits at Supermodel's sheet bases, x 2048 - (2048 >> L), y 1024
// - (1024 >> L): the texture at x0 = ((2^L - 1) 2^11 + 32 tx) >> L, y0 =
// page 1024 + (((2^L - 1) 2^10 + 32 ty) >> L), its side p = max(1, (32 << w)
// >> L), and u, v and their fractions are read off u >> L, v >> L. The shifts
// sit in stage 1 with the masks; level 0 is the form above. The MIP form is
// exact for size codes up to 5 and levels up to 9, all the geometry sends (it
// ends a model at a size code of 6 or more, as MAME does): u >> L is taken 19
// bits wide and the bases from 20 and 19 bits (about 40 ALM fewer).

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_texel #(
  parameter int TW = 8,
  parameter bit MIP = 1'b0
) (
  input  logic               clk,
  input  logic               rst,
  input  logic               en,

  input  logic               i_valid,
  input  logic      [TW-1:0] i_tag,
  input  logic        [31:0] i_u, i_v,
  input  logic               i_page,
  input  logic        [5:0]  i_tx,
  input  logic        [4:0]  i_ty,
  input  logic        [2:0]  i_tw, i_th,
  input  logic               i_mu, i_mv,
  input  logic        [2:0]  i_fmt,
  input  logic        [3:0]  i_lv,       // MIP: the mip level

  // the four banks: addresses on en, words the cycle after
  output logic        [19:0] m_a0, m_a1, m_a2, m_a3,
  output logic        [3:0]  m_re,
  input  logic        [15:0] m_q0, m_q1, m_q2, m_q3,

  output logic               o_valid,
  output logic      [TW-1:0] o_tag,
  output logic      [TW-1:0] o_tag_e,    // o_tag a cycle early (the tag o_tag takes at the next en)
  output logic        [31:0] o_texel
);

  // ---- 1: masks, neighbours, fractions ----------------------------------------------------
  logic          v1;
  logic [TW-1:0] tag1;
  logic [12:0]   u1a, u1b, v1a, v1b;   // u1, u2, v1, v2
  logic [12:0]   pw1, ph1;             // 32 << w
  logic          mu1, mv1;
  logic [7:0]    fu1, fv1;
  logic [10:0]   x01;                  // the texture's first column at the level
  logic [10:0]   y01;                  // its first row (page 1024 included)
  logic [2:0]    fmt1;
  generate if (MIP) begin : g_s1m
    always_ff @(posedge clk) begin
      if (rst) v1 <= 1'b0;
      else if (en) v1 <= i_valid;
      if (en) begin
        logic [12:0] pw, ph, um, vm, a, b;
        logic [4:0]  su, sv;
        logic [18:0] ul, vl;
        logic [25:0] xt;
        logic [24:0] yt;
        logic [14:0] th;
        // log2 of the side at the level, floored at 0 (a side of one texel)
        su = (5'd5 + {2'd0, i_tw} >= {1'b0, i_lv}) ? 5'(5'd5 + {2'd0, i_tw} - {1'b0, i_lv}) : 5'd0;
        sv = (5'd5 + {2'd0, i_th} >= {1'b0, i_lv}) ? 5'(5'd5 + {2'd0, i_th} - {1'b0, i_lv}) : 5'd0;
        pw = 13'd1 << su; ph = 13'd1 << sv;
        um = (i_mu ? {pw[11:0], 1'b0} : pw) - 13'd1;
        vm = (i_mv ? {ph[11:0], 1'b0} : ph) - 13'd1;
        ul = 19'(i_u[27:0] >> i_lv); vl = 19'(i_v[27:0] >> i_lv);
        a = {2'd0, ul[18:8]} & um;
        b = {2'd0, vl[18:8]} & vm;
        u1a <= a; u1b <= (a + 13'd1) & um;
        v1a <= b; v1b <= (b + 13'd1) & vm;
        pw1 <= pw; ph1 <= ph;
        mu1 <= i_mu; mv1 <= i_mv;
        fu1 <= ul[7:0]; fv1 <= vl[7:0];
        // the level's base: (2^L - 1) above the texture's place, both >> L
        th = 15'((16'd1 << i_lv) - 16'd1);
        xt = {th, 11'd0} | {15'd0, i_tx, 5'd0};
        yt = {th, 10'd0} | {15'd0, i_ty, 5'd0};
        x01 <= 11'(xt[19:0] >> i_lv);
        y01 <= {i_page, 10'(yt[18:0] >> i_lv)};
        fmt1 <= i_fmt;
        tag1 <= i_tag;
      end
    end
  end else begin : g_s1
    always_ff @(posedge clk) begin
      if (rst) v1 <= 1'b0;
      else if (en) v1 <= i_valid;
      if (en) begin
        logic [12:0] um, vm, a, b;
        um = 13'(({7'd0, i_mu, ~i_mu, 5'd0} << i_tw) - 13'd1);
        vm = 13'(({7'd0, i_mv, ~i_mv, 5'd0} << i_th) - 13'd1);
        a = i_u[20:8] & um;
        b = i_v[20:8] & vm;
        u1a <= a; u1b <= (a + 13'd1) & um;
        v1a <= b; v1b <= (b + 13'd1) & vm;
        pw1 <= 13'd32 << i_tw; ph1 <= 13'd32 << i_th;
        mu1 <= i_mu; mv1 <= i_mv;
        fu1 <= i_u[7:0]; fv1 <= i_v[7:0];
        x01 <= {i_tx, 5'd0}; y01 <= {i_page, i_ty, 5'd0}; fmt1 <= i_fmt;
        tag1 <= i_tag;
      end
    end
  end endgenerate

  // ---- 2: mirrors, the four words, their banks ------------------------------------------
  function automatic logic [12:0] mir(input logic [12:0] c, input logic [12:0] p);
    mir = (c < p) ? c : 13'((p << 1) - 13'd1 - c);
  endfunction

  logic          v2;
  logic [TW-1:0] tag2;
  logic [21:0]   aw [4];               // corner words: 0 (u1,v1), 1 (u2,v1), 2 (u1,v2), 3 (u2,v2)
  logic [7:0]    fu2, fv2;
  logic [2:0]    fmt2;
  always_ff @(posedge clk) begin
    if (rst) v2 <= 1'b0;
    else if (en) v2 <= v1;
    if (en) begin
      logic [13:0] ca, cb;
      logic [10:0] ra, rb;
      ca = {3'd0, x01} + {1'b0, mir(u1a, pw1)};
      cb = {3'd0, x01} + {1'b0, mir(u1b, pw1)};
      ra = 11'(y01 + 11'(mir(v1a, ph1)));
      rb = 11'(y01 + 11'(mir(v1b, ph1)));
      aw[0] <= {ra, 11'd0} + 22'(ca);
      aw[1] <= {ra, 11'd0} + 22'(cb);
      aw[2] <= {rb, 11'd0} + 22'(ca);
      aw[3] <= {rb, 11'd0} + 22'(cb);
      fu2 <= fu1; fv2 <= fv1; fmt2 <= fmt1; tag2 <= tag1;
    end
  end

  // the port: each bank's address from the first corner that lands in it
  logic [1:0] bk [4];
  always_comb for (int k = 0; k < 4; k++) bk[k] = {aw[k][11], aw[k][0]};
  function automatic logic [19:0] bsel(input logic [1:0] b, input logic [1:0] k0, input logic [1:0] k1, input logic [1:0] k2,
                                       input logic [21:0] a0, input logic [21:0] a1, input logic [21:0] a2, input logic [21:0] a3);
    logic [21:0] a;
    if (k0 == b) a = a0; else if (k1 == b) a = a1; else if (k2 == b) a = a2; else a = a3;
    bsel = {a[21:12], a[10:1]};
  endfunction
  assign m_a0 = bsel(2'd0, bk[0], bk[1], bk[2], aw[0], aw[1], aw[2], aw[3]);
  assign m_a1 = bsel(2'd1, bk[0], bk[1], bk[2], aw[0], aw[1], aw[2], aw[3]);
  assign m_a2 = bsel(2'd2, bk[0], bk[1], bk[2], aw[0], aw[1], aw[2], aw[3]);
  assign m_a3 = bsel(2'd3, bk[0], bk[1], bk[2], aw[0], aw[1], aw[2], aw[3]);
  assign m_re = {4{en}};

  // ---- 3: the words back, by corner ---------------------------------------------------------
  logic          v3;
  logic [TW-1:0] tag3;
  logic [1:0]    bk3 [4];
  logic [7:0]    fu3, fv3;
  logic [2:0]    fmt3;
  always_ff @(posedge clk) begin
    if (rst) v3 <= 1'b0;
    else if (en) v3 <= v2;
    if (en) begin
      for (int k = 0; k < 4; k++) bk3[k] <= bk[k];
      fu3 <= fu2; fv3 <= fv2; fmt3 <= fmt2; tag3 <= tag2;
    end
  end

  // ---- 4: decode -------------------------------------------------------------------------------
  function automatic logic [7:0] p4(input logic [3:0] x); p4 = {x, x}; endfunction
  function automatic logic [7:0] p5(input logic [4:0] x); p5 = {x, x[4:2]}; endfunction
  function automatic logic [31:0] dec(input logic [2:0] f, input logic [15:0] p);
    case (f)
      3'd0: dec = {p[15] ? 8'h00 : 8'hff, p5(p[14:10]), p5(p[9:5]), p5(p[4:0])};
      3'd1, 3'd2: dec = {p4(p[7:4]), {3{p4(p[3:0])}}};
      3'd3: dec = {p4(p[15:12]), {3{p4(p[11:8])}}};
      3'd4: dec = {p4(p[11:8]), {3{p4(p[15:12])}}};
      3'd5: dec = {8'hff, {3{p[7:0]}}};
      3'd6: dec = {8'hff, {3{p[15:8]}}};
      default: dec = {p4(p[3:0]), p4(p[15:12]), p4(p[11:8]), p4(p[7:4])};
    endcase
  endfunction

  logic [15:0] mq [4];
  always_comb begin mq[0] = m_q0; mq[1] = m_q1; mq[2] = m_q2; mq[3] = m_q3; end
  logic          v4;
  logic [TW-1:0] tag4;
  logic [31:0]   c4 [4];
  logic signed [8:0] dt4 [4], db4 [4];    // the u lerp's differences, b - a and d - c
  logic [7:0]    fu4, fv4;
  logic [31:0]   d3 [4];
  always_comb for (int k = 0; k < 4; k++) d3[k] = dec(fmt3, mq[bk3[k]]);
  // the differences are taken here, from the decoded texels, so stage 5 is a product and
  // an add (R87: the -40C path ran c4 -> subtract -> product -> add -> bot5)
  always_ff @(posedge clk) begin
    if (rst) v4 <= 1'b0;
    else if (en) v4 <= v3;
    if (en) begin
      for (int k = 0; k < 4; k++) c4[k] <= d3[k];
      for (int c = 0; c < 4; c++) begin
        dt4[c] <= $signed({1'b0, d3[1][8 * c +: 8]}) - $signed({1'b0, d3[0][8 * c +: 8]});
        db4[c] <= $signed({1'b0, d3[3][8 * c +: 8]}) - $signed({1'b0, d3[2][8 * c +: 8]});
      end
      fu4 <= fu3; fv4 <= fv3; tag4 <= tag3;
    end
  end

  // ---- 5: the u lerp of both rows ----------------------------------------------------------------
  logic          v5;
  logic [TW-1:0] tag5;
  logic [15:0]   top5 [4], bot5 [4];
  logic [7:0]    fv5;
  always_ff @(posedge clk) begin
    if (rst) v5 <= 1'b0;
    else if (en) v5 <= v4;
    if (en) begin
      for (int c = 0; c < 4; c++) begin
        logic signed [17:0] pt, pb;
        pt = dt4[c] * $signed({1'b0, fu4});
        pb = db4[c] * $signed({1'b0, fu4});
        top5[c] <= 16'({c4[0][8 * c +: 8], 8'd0} + 16'(pt));
        bot5[c] <= 16'({c4[2][8 * c +: 8], 8'd0} + 16'(pb));
      end
      fv5 <= fv4; tag5 <= tag4;
    end
  end

  // ---- 6: the v lerp's product -------------------------------------------------------------------
  logic          v6;
  logic [TW-1:0] tag6;
  logic [14:0]   t6 [4];
  logic signed [24:0] p6 [4];
  always_ff @(posedge clk) begin
    if (rst) v6 <= 1'b0;
    else if (en) v6 <= v5;
    if (en) begin
      for (int c = 0; c < 4; c++) begin
        logic signed [15:0] d;
        d = $signed({1'b0, bot5[c][15:1]}) - $signed({1'b0, top5[c][15:1]});
        p6[c] <= d * $signed({1'b0, fv5});
        t6[c] <= top5[c][15:1];
      end
      tag6 <= tag5;
    end
  end

  assign o_tag_e = tag6;

  // ---- 7: the add, >> 15, out -------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) o_valid <= 1'b0;
    else if (en) o_valid <= v6;
    if (en) begin
      for (int c = 0; c < 4; c++) begin
        logic [23:0] o;
        o = 24'({1'b0, t6[c], 8'd0} + 24'(p6[c]));
        o_texel[8 * c +: 8] <= (o[23:15] > 9'd255) ? 8'd255 : o[22:15];
      end
      o_tag <= tag6;
    end
  end

endmodule
