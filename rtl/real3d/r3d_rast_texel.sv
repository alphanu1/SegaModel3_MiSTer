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
//
// PART (study R131): r3d_rast runs the address stages (1-2) before the
// texture cache and the filter (3-7) after it, as two instances. With PART = 1
// an instance is stages 1-2 and gives out what stage 3 takes from them (the
// four corners' banks, the fractions, the format: o2_*); with PART = 2 it is
// stages 3-7, with those taken in (i3_*, r3d_rast's ring A read registered at
// o_tagr, stage 1's tag, so they arrive while the pixel is in stage 2 and are
// registered into stage 3 as before: the block RAM's output does not run into
// the decode), so the filter no longer repeats the address arithmetic to find
// its banks. PART = 0 is the whole texel (its unit bench).
//
// L8K (study R134): an L8 texel of 0xFF (formats 5 and 6) decodes with alpha 0,
// as Supermodel's ExtractColour; MAME's get_texture gives every L8 texel alpha
// 0xFF. Alpha is read only by the contour and texture-alpha kinds, so this
// cuts out Lost World's foliage (format 5 under contour), which MAME's form
// draws as a box of the polygon's colour. L8K = 0 is MAME's decode.
//
// SMTX (study R135): the rest of Supermodel's decode (New3D R3DShaderCommon.h
// ExtractColour, New3D.cpp GetTexFormat, texBiLinear). Format 2 is format 1
// with its nibbles swapped (alpha 3:0, luminance 7:4; MAME reads it as format
// 1). With the polygon's contour bit (i_ctr) formats 1-4 are Supermodel's
// types 8-11: the alpha nibble is ignored and luminance 15 is transparent. And
// under contour on any format but 7 (whose alpha test Supermodel turns off)
// every texel's alpha is 0 or 0xFF, and a transparent corner takes the colour
// of an opaque edge neighbour before the filter, in texBiLinear's order: corner
// 0 (u1, v1) from 2, else 1; 1 (u2, v1) from 3, else 0; 2 (u1, v2) from 3,
// else 0; 3 (u2, v2) from 2, else 1. Its alpha stays. That is the dilation, a
// bank select per corner: the corner's colour is read from the neighbour's
// bank. With PART = 2 the selects are formed in stage 2 from registers only:
// r3d_rast reads ring A a stage earlier (o_tagr is then the tag going into
// stage 1, and i3_* arrive while the pixel is in stage 1) and gives each bank
// word's flags (i1_f: bit 15, and each nibble all ones) from a copy of its
// word delay line read a cycle early. Stage 3's data path is the same depth as
// before: a bank select from registers, the decode, the differences. PART = 0
// forms the selects in stage 3 from the words (the unit bench's form).

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_texel #(
  parameter int TW = 8,
  parameter bit MIP = 1'b0,
  parameter int PART = 0,         // 0 whole, 1 stages 1-2, 2 stages 3-7
  parameter bit L8K = 1'b1,       // L8 0xFF transparent (Supermodel); 0: MAME's decode
  parameter bit SMTX = 1'b1       // Supermodel's format 2, contour decode and dilation (R135); 0: MAME's
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
  input  logic               i_ctr,      // SMTX: the polygon's contour bit
  input  logic        [3:0]  i_lv,       // MIP: the mip level

  // the four banks: addresses on en, words the cycle after
  output logic        [19:0] m_a0, m_a1, m_a2, m_a3,
  output logic        [3:0]  m_re,
  input  logic        [15:0] m_q0, m_q1, m_q2, m_q3,

  // PART = 1: stage 2's banks {bk3, bk2, bk1, bk0}, fractions and format; PART = 2: the same in at stage 3
  output logic        [7:0]  o2_bk,
  output logic        [7:0]  o2_fu, o2_fv,
  output logic        [2:0]  o2_fmt,
  output logic               o2_ctr,
  // PART = 2: the tag the caller reads i3_* at (registered): stage 1's; with SMTX the one going into stage 1
  output logic      [TW-1:0] o_tagr,
  input  logic        [7:0]  i3_bk,
  input  logic        [7:0]  i3_fu, i3_fv,
  input  logic        [2:0]  i3_fmt,
  input  logic               i3_ctr,
  input  logic        [19:0] i1_f,       // PART = 2, SMTX: bank b's word's {bit 15, nibbles 3..0 all ones} at 5b, in stage 1

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
  logic          ctr1;
  always_ff @(posedge clk) if (en) ctr1 <= i_ctr;
  generate if (PART == 2) begin : g_s1f
    // the filter alone: stages 1 and 2 carry the valid and the tag
    always_ff @(posedge clk) begin
      if (rst) v1 <= 1'b0;
      else if (en) v1 <= i_valid;
      if (en) tag1 <= i_tag;
    end
    // stage 1's fields are not used (tied off)
    always_comb begin
      u1a = '0; u1b = '0; v1a = '0; v1b = '0; pw1 = '0; ph1 = '0; mu1 = 1'b0; mv1 = 1'b0;
      fu1 = '0; fv1 = '0; x01 = '0; y01 = '0; fmt1 = '0;
    end
  end else if (MIP) begin : g_s1m
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
        // the masks as thermometers of su + mu, sv + mv (= (mu ? 2 pw : pw) - 1), and the neighbour
        // (a + 1) & m as (x + 1) & m, side by side with a (study R109: the table's word ran through
        // the side's subtract, the shift, the mask's subtract, the mask and the add in series)
        for (int k = 0; k < 13; k++) begin
          um[k] = 5'(k) < su + {4'd0, i_mu};
          vm[k] = 5'(k) < sv + {4'd0, i_mv};
        end
        ul = 19'(i_u[27:0] >> i_lv); vl = 19'(i_v[27:0] >> i_lv);
        a = {2'd0, ul[18:8]} & um;
        b = {2'd0, vl[18:8]} & vm;
        u1a <= a; u1b <= ({2'd0, ul[18:8]} + 13'd1) & um;
        v1a <= b; v1b <= ({2'd0, vl[18:8]} + 13'd1) & vm;
`ifdef VERILATOR
        if (um != (i_mu ? {pw[11:0], 1'b0} : pw) - 13'd1 || vm != (i_mv ? {ph[11:0], 1'b0} : ph) - 13'd1 ||
            ((({2'd0, ul[18:8]} + 13'd1) & um) != ((a + 13'd1) & um)) || ((({2'd0, vl[18:8]} + 13'd1) & vm) != ((b + 13'd1) & vm)))
          $fatal(1, "texel: the masks as thermometers");
`endif
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
  // the same without the compare and the subtract (study R109): p is a power of two and c < 2p (c is
  // masked to 2p - 1 when mirrored, p - 1 otherwise), so c >= p is c's bit at p, and 2p - 1 - c is then
  // c with its bits below 2p inverted
  function automatic logic [12:0] mirx(input logic [12:0] c, input logic [12:0] p);
    mirx = ((c & p) != 13'd0) ? (c ^ ({p[11:0], 1'b0} - 13'd1)) : c;
  endfunction

  logic          v2;
  logic [TW-1:0] tag2;
  logic [21:0]   aw [4];               // corner words: 0 (u1,v1), 1 (u2,v1), 2 (u1,v2), 3 (u2,v2)
  logic [7:0]    fu2, fv2;
  logic [2:0]    fmt2;
  logic          ctr2;
  always_ff @(posedge clk) begin
    if (rst) v2 <= 1'b0;
    else if (en) v2 <= v1;
    if (en) tag2 <= tag1;
    if (en) ctr2 <= ctr1;
    if (en && PART != 2) begin
      // ((y01 + v) mod 2^11) 2^11 + x01 + u, mod 2^22, as one three-input add: x01 < 2^11, so {y01, x01} is
      // y01 2^11 + x01 (the row's add and the column's ran into a third add in series: 113 / 107 MHz)
      logic [12:0] mua, mub;
      logic [10:0] mva, mvb;
      mua = mirx(u1a, pw1); mub = mirx(u1b, pw1);
      mva = 11'(mirx(v1a, ph1)); mvb = 11'(mirx(v1b, ph1));
      aw[0] <= {y01, x01} + {mva, 11'd0} + 22'(mua);
      aw[1] <= {y01, x01} + {mva, 11'd0} + 22'(mub);
      aw[2] <= {y01, x01} + {mvb, 11'd0} + 22'(mua);
      aw[3] <= {y01, x01} + {mvb, 11'd0} + 22'(mub);
      fu2 <= fu1; fv2 <= fv1; fmt2 <= fmt1;
    end
  end

`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && en && v1 && PART != 2 && {mirx(u1a, pw1), mirx(u1b, pw1), mirx(v1a, ph1), mirx(v1b, ph1)} !=
                            {mir(u1a, pw1), mir(u1b, pw1), mir(v1a, ph1), mir(v1b, ph1)})
      $fatal(1, "texel: the mirror without the compare");
`endif

`ifdef VERILATOR
  logic [21:0] aw_chk [4];
  always_ff @(posedge clk) if (en && PART != 2) begin
    logic [13:0] ca, cb;
    logic [10:0] ra, rb;
    ca = {3'd0, x01} + {1'b0, mirx(u1a, pw1)};
    cb = {3'd0, x01} + {1'b0, mirx(u1b, pw1)};
    ra = 11'(y01 + 11'(mirx(v1a, ph1)));
    rb = 11'(y01 + 11'(mirx(v1b, ph1)));
    aw_chk[0] <= {ra, 11'd0} + 22'(ca); aw_chk[1] <= {ra, 11'd0} + 22'(cb);
    aw_chk[2] <= {rb, 11'd0} + 22'(ca); aw_chk[3] <= {rb, 11'd0} + 22'(cb);
  end
  always_ff @(posedge clk) if (!rst && PART != 2 && v2 && {aw[0], aw[1], aw[2], aw[3]} != {aw_chk[0], aw_chk[1], aw_chk[2], aw_chk[3]})
    $fatal(1, "texel: the corner words in one add");
`endif

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
  assign o2_bk = {bk[3], bk[2], bk[1], bk[0]};
  assign o2_fu = fu2; assign o2_fv = fv2; assign o2_fmt = fmt2; assign o2_ctr = ctr2;
  assign o_tagr = (PART == 2 && SMTX) ? i_tag : tag1;

  // ---- the dilation's selects (SMTX) ---------------------------------------------------------
  // a bank word's transparency under contour, from its flags {bit 15, nibble 3..0 all ones}
  function automatic logic trf(input logic [2:0] f, input logic c, input logic [4:0] fl);
    case (f)
      3'd0: trf = fl[4];
      3'd1: trf = c && fl[0];
      3'd2: trf = c && fl[1];
      3'd3: trf = c && fl[2];
      3'd4: trf = c && fl[3];
      3'd5: trf = L8K && fl[0] && fl[1];
      3'd6: trf = L8K && fl[2] && fl[3];
      default: trf = 1'b0;
    endcase
  endfunction
  function automatic logic [4:0] flg(input logic [15:0] w);
    flg = {w[15], &w[15:12], &w[11:8], &w[7:4], &w[3:0]};
  endfunction
  // {dilate, corner 3..0's colour bank (2 bits each), corner 3..0 transparent}
  function automatic logic [12:0] dsel(input logic [2:0] f, input logic c, input logic [19:0] fl, input logic [7:0] bkv);
    logic [3:0] tb, tk;
    logic [1:0] b [4];
    logic [1:0] s [4];
    logic       d;
    for (int k = 0; k < 4; k++) begin
      tb[k] = trf(f, c, fl[5 * k +: 5]);
      b[k] = bkv[2 * k +: 2];
    end
    for (int k = 0; k < 4; k++) tk[k] = tb[b[k]];
    d = SMTX && c && f != 3'd7;
    // texBiLinear's order: the last opaque edge neighbour wins
    s[0] = (d && tk[0]) ? (!tk[2] ? b[2] : !tk[1] ? b[1] : b[0]) : b[0];
    s[1] = (d && tk[1]) ? (!tk[3] ? b[3] : !tk[0] ? b[0] : b[1]) : b[1];
    s[2] = (d && tk[2]) ? (!tk[3] ? b[3] : !tk[0] ? b[0] : b[2]) : b[2];
    s[3] = (d && tk[3]) ? (!tk[2] ? b[2] : !tk[1] ? b[1] : b[3]) : b[3];
    dsel = {d, s[3], s[2], s[1], s[0], tk};
  endfunction

  // ---- 3: the words back, by corner ---------------------------------------------------------
  logic          v3;
  logic [TW-1:0] tag3;
  logic [1:0]    bk3 [4];
  logic [7:0]    fu3, fv3;
  logic [2:0]    fmt3;
  logic [1:0]    bk3r [4];
  logic [7:0]    fu3r, fv3r;
  logic [2:0]    fmt3r;
  logic          ctr3;
  logic [1:0]    sb3 [4];              // each corner's colour bank (the dilation's select)
  logic [3:0]    tk3;                  // each corner transparent (under dilation)
  logic          dl3;                  // dilating
  // PART = 2 with SMTX: the caller's read at o_tagr holds this pixel's fields while it is in stage 1;
  // they are registered here, with the bank words' flags, and the selects formed from them
  logic [7:0]    bk2r, fu2r, fv2r;
  logic [2:0]    fmt2r;
  logic          ctr2r;
  logic [19:0]   fl2r;
  logic [12:0]   ds2;
  generate if (PART == 2 && SMTX) begin : g_s2e
    always_ff @(posedge clk) if (en) begin
      bk2r <= i3_bk; fu2r <= i3_fu; fv2r <= i3_fv; fmt2r <= i3_fmt; ctr2r <= i3_ctr; fl2r <= i1_f;
    end
    always_comb ds2 = dsel(fmt2r, ctr2r, fl2r, bk2r);
  end else begin : g_s2n
    always_comb begin bk2r = '0; fu2r = '0; fv2r = '0; fmt2r = '0; ctr2r = 1'b0; fl2r = '0; ds2 = '0; end
  end endgenerate
  logic [12:0]   ds3r;
  always_ff @(posedge clk) begin
    if (rst) v3 <= 1'b0;
    else if (en) v3 <= v2;
    if (en) begin
      // PART = 2: the caller's read at o_tagr holds this pixel's fields while it is in stage 2 (SMTX:
      // stage 1, registered above)
      for (int k = 0; k < 4; k++)
        bk3r[k] <= (PART == 2) ? (SMTX ? bk2r[2 * k +: 2] : i3_bk[2 * k +: 2]) : bk[k];
      fu3r <= (PART == 2) ? (SMTX ? fu2r : i3_fu) : fu2; fv3r <= (PART == 2) ? (SMTX ? fv2r : i3_fv) : fv2;
      fmt3r <= (PART == 2) ? (SMTX ? fmt2r : i3_fmt) : fmt2;
      ctr3 <= (PART == 2) ? (SMTX ? ctr2r : i3_ctr) : ctr2;
      ds3r <= ds2;
      tag3 <= tag2;
    end
  end
  logic [15:0] mq [4];
  always_comb begin mq[0] = m_q0; mq[1] = m_q1; mq[2] = m_q2; mq[3] = m_q3; end
  always_comb begin
    logic [12:0] ds;
    for (int k = 0; k < 4; k++) bk3[k] = bk3r[k];
    fu3 = fu3r; fv3 = fv3r; fmt3 = fmt3r;
    // PART = 2: the selects registered from stage 2; else formed here from the words
    if (PART == 2) ds = ds3r;
    else ds = dsel(fmt3, ctr3, {flg(mq[3]), flg(mq[2]), flg(mq[1]), flg(mq[0])}, {bk3r[3], bk3r[2], bk3r[1], bk3r[0]});
    dl3 = SMTX && ds[12];
    for (int k = 0; k < 4; k++) sb3[k] = SMTX ? ds[4 + 2 * k +: 2] : bk3r[k];
    tk3 = ds[3:0];
  end

  // ---- 4: decode -------------------------------------------------------------------------------
  function automatic logic [7:0] p4(input logic [3:0] x); p4 = {x, x}; endfunction
  function automatic logic [7:0] p5(input logic [4:0] x); p5 = {x, x[4:2]}; endfunction
  // SMTX: format 2 swapped (Supermodel's type 2). Formats 1-4 under contour (types 8-11) are always
  // dilated, so their alpha (luminance 15 transparent) comes from the selects (trf), not from here.
  function automatic logic [31:0] dec(input logic [2:0] f, input logic [15:0] p);
    case (f)
      3'd0: dec = {p[15] ? 8'h00 : 8'hff, p5(p[14:10]), p5(p[9:5]), p5(p[4:0])};
      3'd1: dec = {p4(p[7:4]), {3{p4(p[3:0])}}};
      3'd2: dec = SMTX ? {p4(p[3:0]), {3{p4(p[7:4])}}} : {p4(p[7:4]), {3{p4(p[3:0])}}};
      3'd3: dec = {p4(p[15:12]), {3{p4(p[11:8])}}};
      3'd4: dec = {p4(p[11:8]), {3{p4(p[15:12])}}};
      3'd5: dec = {(L8K && p[7:0] == 8'hff) ? 8'h00 : 8'hff, {3{p[7:0]}}};
      3'd6: dec = {(L8K && p[15:8] == 8'hff) ? 8'h00 : 8'hff, {3{p[15:8]}}};
      default: dec = {p4(p[3:0]), p4(p[15:12]), p4(p[11:8]), p4(p[7:4])};
    endcase
  endfunction

  logic          v4;
  logic [TW-1:0] tag4;
  logic [31:0]   c4 [4];
  logic [7:0]    fu4, fv4;
  logic [31:0]   d3 [4];
  // each corner's colour from its select's bank (its own unless dilated); a dilated corner keeps its own
  // alpha (0 if transparent: under dilation every alpha is 0 or 0xff)
  always_comb for (int k = 0; k < 4; k++) begin
    logic [31:0] w;
    w = dec(fmt3, mq[sb3[k]]);
    d3[k] = dl3 ? {tk3[k] ? 8'h00 : 8'hff, w[23:0]} : w;
  end
  // stage 4 holds the decoded texels; the u lerp's differences (b - a, d - c) are taken in stage 5a, in front
  // of their products (the DSP's pre-adder form, (b - a) x fu): stage 3 is the selects and the decode alone
  // (R87 had moved the differences here from stage 5; with R135's dilation selects in stage 3 the subtract
  // behind them was the rasteriser's worst path, 110.7 MHz standalone, -1.22 ns in c7_s1)
  always_ff @(posedge clk) begin
    if (rst) v4 <= 1'b0;
    else if (en) v4 <= v3;
    if (en) begin
      for (int k = 0; k < 4; k++) c4[k] <= d3[k];
      fu4 <= fu3; fv4 <= fv3; tag4 <= tag3;
    end
  end

  // ---- 5: the u lerp of both rows: the products (5a), then the adds (5b) ------------------------------
  // (study R109: a stage more, so the DSP blocks' products are registered before the add; the DSP's
  // multiply and the 16-bit add were 9.2 ns at -40C. The filter's latency is one cycle longer; its tags
  // move with it, o_tag_e included.)
  logic          v5a, v5;
  logic [TW-1:0] tag5a, tag5;
  logic signed [17:0] pt5a [4], pb5a [4];
  logic [7:0]    ct5a [4], cb5a [4];
  logic [15:0]   top5 [4], bot5 [4];
  logic [7:0]    fv5a, fv5;
  always_ff @(posedge clk) begin
    if (rst) begin v5a <= 1'b0; v5 <= 1'b0; end
    else if (en) begin v5a <= v4; v5 <= v5a; end
    if (en) begin
      for (int c = 0; c < 4; c++) begin
        logic signed [8:0] dt, db;     // b - a and d - c
        dt = $signed({1'b0, c4[1][8 * c +: 8]}) - $signed({1'b0, c4[0][8 * c +: 8]});
        db = $signed({1'b0, c4[3][8 * c +: 8]}) - $signed({1'b0, c4[2][8 * c +: 8]});
        pt5a[c] <= dt * $signed({1'b0, fu4});
        pb5a[c] <= db * $signed({1'b0, fu4});
        ct5a[c] <= c4[0][8 * c +: 8];
        cb5a[c] <= c4[2][8 * c +: 8];
        top5[c] <= 16'({ct5a[c], 8'd0} + 16'(pt5a[c]));
        bot5[c] <= 16'({cb5a[c], 8'd0} + 16'(pb5a[c]));
      end
      fv5a <= fv4; tag5a <= tag4;
      fv5 <= fv5a; tag5 <= tag5a;
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
