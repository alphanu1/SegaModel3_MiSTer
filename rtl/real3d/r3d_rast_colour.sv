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
// Real3D rasteriser: a pixel's colour from its texel, the polygon's colour and
// the Gouraud shade, then fog. Bit for bit r3d_rast_rtl.cpp frag and
// fog_apply (MAME's draw kinds, D-29: translucency is MAME's blend, which the
// band stage applies with fb). One a clock, five stages on en.
//
//   base   the polygon colour (solid kinds) or the texel; contour and colour
//          modulation multiply by the polygon colour, >> 8
//   shade  clamp(int32(base s) >> 8, 0, 255), the product wrapping at 32 bits
//          as MAME's x86 multiply does
//   fog    lin = clamp(S + P, 0, 1024), g = (fogint lin) >> 4,
//          o = clamp(o + (((fc - o) g) >> 10), 0, 255)
//   kinds  contour: texel alpha < 0xf8 discards, else blends by it and writes
//          Z; texture alpha: alpha 0 discards, blends by it; solid
//          translucent: factor (t << 3 | t >> 2) truncated to 8 bits unless
//          0xff. Alpha is not carried: the band writes 0xff.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_colour #(
  parameter int TW = 8
) (
  input  logic               clk,
  input  logic               rst,
  input  logic               en,

  input  logic               i_valid,
  input  logic      [TW-1:0] i_tag,
  input  logic        [2:0]  i_kind,    // r3d::rkind, 1..6
  input  logic        [31:0] i_texel,   // ARGB
  input  logic        [23:0] i_color,   // the polygon's RGB
  input  logic        [5:0]  i_trans,   // 0..32
  input  logic        [31:0] i_shade,
  input  logic        [4:0]  i_fogint,  // 0: no fog
  input  logic        [23:0] i_fc,      // fog colour x ambient, RGB
  input  logic signed [17:0] i_fs,      // fog start, 1/1024
  input  logic        [12:0] i_fp,      // fog depth term P

  output logic               o_valid,
  output logic      [TW-1:0] o_tag,
  output logic               o_ok,      // not discarded
  output logic               o_zw,      // writes Z
  output logic               o_fbv,     // blends
  output logic        [7:0]  o_fb,      // the factor
  output logic        [23:0] o_rgb
);

  localparam logic [2:0] K_SOLID = 3'd1, K_STRANS = 3'd2, K_TEX = 3'd3, K_CMOD = 3'd4, K_CONT = 3'd5, K_TALPHA = 3'd6;

  // ---- 1: base, modulation; alpha tests; fog lin ---------------------------------------
  logic          v1;
  logic [TW-1:0] tag1;
  logic [7:0]    b1 [3];
  logic [31:0]   s1;
  logic          ok1, zw1, fbv1;
  logic [7:0]    fb1;
  logic [4:0]    fi1;
  logic [23:0]   fc1;
  logic [10:0]   lin1;
  always_ff @(posedge clk) begin
    if (rst) v1 <= 1'b0;
    else if (en) v1 <= i_valid;
    if (en) begin
      logic        sol, mod;
      logic [7:0]  ta;
      logic [8:0]  tr;
      logic signed [18:0] l;
      sol = (i_kind == K_SOLID) || (i_kind == K_STRANS);
      mod = (i_kind == K_CMOD) || (i_kind == K_CONT);
      for (int c = 0; c < 3; c++) begin
        logic [7:0]  a, k;
        logic [15:0] m;
        a = sol ? i_color[8 * c +: 8] : i_texel[8 * c +: 8];
        k = i_color[8 * c +: 8];
        m = a * k;
        b1[c] <= mod ? m[15:8] : a;
      end
      ta = i_texel[31:24];
      tr = {i_trans, 3'd0} | {5'd0, i_trans[5:2]};        // t << 3 | t >> 2; t = 32 gives 264
      ok1  <= !((i_kind == K_CONT && ta < 8'hf8) || (i_kind == K_TALPHA && ta == 8'd0));
      zw1  <= (i_kind == K_SOLID) || (i_kind == K_TEX) || (i_kind == K_CMOD) || (i_kind == K_CONT);
      fbv1 <= (i_kind == K_CONT) || (i_kind == K_TALPHA) || (i_kind == K_STRANS && tr != 9'h0ff);
      fb1  <= (i_kind == K_STRANS) ? tr[7:0] : ta;
      tag1 <= i_tag; s1 <= i_shade; fi1 <= i_fogint; fc1 <= i_fc;
      l = {i_fs[17], i_fs} + 19'(i_fp);
      lin1 <= l[18] ? 11'd0 : (l > 19'sd1024) ? 11'd1024 : l[10:0];
    end
  end

  // ---- 2: the shade product, c s[26:0] in one 27 x 27 and the low 5 bits of c s[31:27] -----
  logic          v2;
  logic [TW-1:0] tag2;
  logic [34:0]   lo2 [3];
  logic [4:0]    hi2 [3];
  logic          ok2, zw2, fbv2;
  logic [7:0]    fb2;
  logic          fo2;
  logic [23:0]   fc2;
  logic [10:0]   g2;
  always_ff @(posedge clk) begin
    if (rst) v2 <= 1'b0;
    else if (en) v2 <= v1;
    if (en) begin
      logic [15:0] gg;
      for (int c = 0; c < 3; c++) begin
        logic [12:0] h;
        lo2[c] <= b1[c] * s1[26:0];
        h = b1[c] * s1[31:27];
        hi2[c] <= h[4:0];
      end
      tag2 <= tag1; ok2 <= ok1; zw2 <= zw1; fbv2 <= fbv1; fb2 <= fb1;
      fo2 <= fi1 != 5'd0; fc2 <= fc1;
      gg = fi1 * lin1;
      g2 <= gg[14:4];
    end
  end

  // ---- 3: the low 32 bits, the clamp ---------------------------------------------------------
  logic          v3;
  logic [TW-1:0] tag3;
  logic [7:0]    c3 [3];
  logic          ok3, zw3, fbv3, fo3;
  logic [7:0]    fb3;
  logic [23:0]   fc3;
  logic [10:0]   g3;
  always_ff @(posedge clk) begin
    if (rst) v3 <= 1'b0;
    else if (en) v3 <= v2;
    if (en) begin
      for (int c = 0; c < 3; c++) begin
        logic [31:0] q;
        q = lo2[c][31:0] + {hi2[c], 27'd0};
        if (q[31]) c3[c] <= 8'd0;
        else if (q[30:16] != 15'd0) c3[c] <= 8'd255;
        else c3[c] <= q[15:8];
      end
      tag3 <= tag2; ok3 <= ok2; zw3 <= zw2; fbv3 <= fbv2; fb3 <= fb2; fo3 <= fo2; fc3 <= fc2; g3 <= g2;
    end
  end

  // ---- 4: fog's product ----------------------------------------------------------------------
  logic          v4;
  logic [TW-1:0] tag4;
  logic [7:0]    c4 [3];
  logic signed [19:0] d4 [3];
  logic          ok4, zw4, fbv4, fo4;
  logic [7:0]    fb4;
  always_ff @(posedge clk) begin
    if (rst) v4 <= 1'b0;
    else if (en) v4 <= v3;
    if (en) begin
      for (int c = 0; c < 3; c++) begin
        logic signed [8:0] df;
        df = $signed({1'b0, fc3[8 * c +: 8]}) - $signed({1'b0, c3[c]});
        d4[c] <= df * $signed({1'b0, g3});
      end
      c4 <= c3;
      tag4 <= tag3; ok4 <= ok3; zw4 <= zw3; fbv4 <= fbv3; fb4 <= fb3; fo4 <= fo3;
    end
  end

  // ---- 5: fog's add and clamp, out --------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) o_valid <= 1'b0;
    else if (en) o_valid <= v4;
    if (en) begin
      for (int c = 0; c < 3; c++) begin
        logic signed [10:0] t;
        t = $signed({3'd0, c4[c]}) + 11'(d4[c] >>> 10);
        if (!fo4) o_rgb[8 * c +: 8] <= c4[c];
        else o_rgb[8 * c +: 8] <= t[10] ? 8'd0 : (t > 11'sd255) ? 8'd255 : t[7:0];
      end
      o_tag <= tag4; o_ok <= ok4; o_zw <= zw4; o_fbv <= fbv4; o_fb <= fb4;
    end
  end

endmodule
