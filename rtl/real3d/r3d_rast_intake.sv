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
// Real3D rasteriser: the set-up intake. A set-up triangle record in (the
// geometry's word stream, sim/real3d/m2-notes.md section 14), its fields in
// the rasteriser's fixed point out. Bit for bit r3d_rast_rtl.cpp
// rtl_tri::setup's to_fix and make_plane.
//
// Record: words 0-2 the polygon header (word 0 [4] texture on), 3-8 the
// y-sorted screen x, y (FP32), 9 det, then per parameter pdx, pdy, ps (FP32):
// five parameters textured, two untextured.
//
// One FP32-to-fixed converter serves every field (study 8.4: converters
// time-shared). A vertex is f 2^16 rounded to nearest, ties away from zero,
// saturated to int32. A parameter's three coefficients share one exponent,
//   e = max(E(ps), E(pdx) + 9, E(pdy) + 9) - 124   (biased E of the non-zero
//   ones; 2 if all three are zero)
// which is frexp(max(|ps|, 512 |pdx|, 512 |pdy|)) + 2, and each is
// round(f 2^(40 - e)) the same way: a 40-bit two's-complement mantissa. The
// set-up's units flush to zero, so no operand is subnormal; a non-finite
// coefficient does not occur (none in 4.9 M triangles) and converts as a
// finite one with its exponent.
//
// Out, a field a beat: tag 0-5 vertex x1 y1 x2 y2 x3 y3 (32 bits), tag 6
// det's raw word, then per parameter p tags 8 + 4p .. 11 + 4p: e (signed 9),
// c = ps, dx, dy (signed 40). o_last on the record's last field. The record's
// header words go out first, tag 15 (three beats).

module r3d_rast_intake (
  input  logic        clk,
  input  logic        rst,

  input  logic        i_valid,
  output logic        i_ready,
  input  logic [31:0] i_data,
  input  logic        i_last,

  output logic        o_valid,
  input  logic        o_ready,
  output logic [4:0]  o_tag,
  output logic [39:0] o_data,
  output logic        o_last
);

  // ---- the converter: round(M 2^sh), ties away, sign applied ---------------------
  // Four stages that move together with the output register: F (the field and
  // its shift amount a = 14 - sh), S (Q = M 2^29 >> a = M 2^(sh + 15)), R (the
  // integer part Q >> 15 plus the half bit Q[14]), then the sign and the
  // vertex saturation into o_*. sh <= 14 always (planes by construction,
  // vertices saturate above 7). Header words, det and e pass as raw fields.

  localparam logic [1:0] K_RAW = 2'd0, K_VTX = 2'd1, K_PLN = 2'd2;

  wire adv = !o_valid || o_ready;

  // ---- record state ------------------------------------------------------------

  logic [4:0]  wc;            // word in the record
  logic [2:0]  np_left;       // parameters still to come
  logic [1:0]  pk;            // coefficient within the parameter: 0 pdx, 1 pdy, 2 ps
  logic [2:0]  pnum;          // parameter number
  logic [31:0] pw [3];        // the parameter's words, pdx, pdy, ps
  logic [8:0]  emx;           // running max of E (+9 for the gradients), 0 = none

  // emission: a parameter's four fields go out after its third word
  logic        em_act;
  logic [2:0]  em_k;          // 0 e, 1 c, 2 dx, 3 dy
  logic signed [8:0] em_e;
  logic        em_last;
  logic [2:0]  em_p;

  wire in_take = i_valid && i_ready;
  assign i_ready = adv && !em_act;

  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [8:0] eadj(input logic [31:0] w, input logic grad);
    if (w[30:23] == 8'd0) eadj = 9'd0;
    else eadj = {1'b0, w[30:23]} + (grad ? 9'd9 : 9'd0);
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // stage F
  logic        f_v, f_last;
  logic [4:0]  f_tag;
  logic [1:0]  f_kind;
  logic [31:0] f_w;           // the FP32 word, or the raw field's low 32 bits
  logic [39:0] f_raw;
  logic signed [10:0] f_a;    // 14 - sh

  wire [31:0] em_w = pw[em_k == 3'd1 ? 2 : em_k == 3'd2 ? 0 : 1];

  // stage S
  logic        s_v, s_last, s_sgn, s_sat, s_nan;
  logic [4:0]  s_tag;
  logic [1:0]  s_kind;
  logic [39:0] s_raw;
  logic [38:0] s_q;           // Q[52:14]

  // stage R
  logic        r_v, r_last, r_sgn, r_sat, r_nan;
  logic [4:0]  r_tag;
  logic [1:0]  r_kind;
  logic [39:0] r_raw;
  logic [38:0] r_mag;

  always_ff @(posedge clk) begin
    if (rst) begin
      wc      <= 5'd0;
      em_act  <= 1'b0;
      emx     <= 9'd0;
      pk      <= 2'd0;
      f_v     <= 1'b0;
      s_v     <= 1'b0;
      r_v     <= 1'b0;
      o_valid <= 1'b0;
    end else if (adv) begin
      // ---- F: the record's next field
      f_v <= 1'b0;
      if (em_act) begin
        f_v    <= 1'b1;
        f_tag  <= 5'(5'd8 + {em_p, 2'b00} + {3'd0, em_k[1:0]});
        f_kind <= (em_k == 3'd0) ? K_RAW : K_PLN;
        f_raw  <= {{31{em_e[8]}}, em_e};
        f_w    <= em_w;
        // sh = E - 110 - e, a = 14 - sh = 124 + e - E
        f_a    <= 11'sd124 + {{2{em_e[8]}}, em_e} - $signed({3'b000, em_w[30:23]});
        f_last <= em_last && em_k == 3'd3;
        em_k   <= em_k + 3'd1;
        if (em_k == 3'd3) em_act <= 1'b0;
      end else if (in_take) begin
        wc     <= i_last ? 5'd0 : wc + 5'd1;
        f_w    <= i_data;
        f_raw  <= {8'd0, i_data};
        f_last <= 1'b0;
        // sh = E - 134, a = 148 - E
        f_a    <= 11'sd148 - $signed({3'b000, i_data[30:23]});
        if (wc <= 5'd2) begin
          f_v <= 1'b1; f_tag <= 5'd15; f_kind <= K_RAW;
          if (wc == 5'd0) np_left <= i_data[4] ? 3'd5 : 3'd2;
        end else if (wc <= 5'd8) begin
          f_v <= 1'b1; f_tag <= 5'(wc - 5'd3); f_kind <= K_VTX;
        end else if (wc == 5'd9) begin
          f_v <= 1'b1; f_tag <= 5'd6; f_kind <= K_RAW;
          pk <= 2'd0; pnum <= 3'd0; emx <= 9'd0;
        end else begin
          // a coefficient: keep it, fold its exponent in
          logic [8:0] ea, mx;
          ea = eadj(i_data, pk != 2'd2);
          mx = (ea > emx) ? ea : emx;
          pw[pk] <= i_data;
          if (pk == 2'd2) begin
            em_act  <= 1'b1;
            em_k    <= 3'd0;
            em_p    <= pnum;
            em_e    <= (mx == 9'd0) ? 9'sd2 : $signed(9'(mx - 9'd124));
            em_last <= np_left == 3'd1;
            np_left <= np_left - 3'd1;
            pnum    <= pnum + 3'd1;
            pk      <= 2'd0;
            emx     <= 9'd0;
          end else begin
            pk  <= pk + 2'd1;
            emx <= mx;
          end
        end
      end
      // ---- S: the shift
      s_v <= f_v; s_last <= f_last; s_tag <= f_tag; s_kind <= f_kind; s_raw <= f_raw;
      s_sgn <= f_w[31];
      s_nan <= (f_w[30:23] == 8'hff) && (f_w[22:0] != 23'd0);
      s_sat <= f_w[30:23] >= 8'd142;
      begin
        /* verilator lint_off UNUSEDSIGNAL */
        logic [52:0] pp, qq;
        /* verilator lint_on UNUSEDSIGNAL */
        pp = {f_w[30:23] != 8'd0, f_w[22:0], 29'd0};
        if (f_a > 11'sd52) qq = 53'd0;
        else qq = pp >> f_a[5:0];
        s_q <= qq[52:14];
      end
      // ---- R: the round
      r_v <= s_v; r_last <= s_last; r_tag <= s_tag; r_kind <= s_kind; r_raw <= s_raw;
      r_sgn <= s_sgn; r_sat <= s_sat; r_nan <= s_nan;
      r_mag <= {1'b0, s_q[38:1]} + {38'd0, s_q[0]};
      // ---- out: the sign, the vertex saturation
      o_valid <= r_v;
      o_tag   <= r_tag;
      o_last  <= r_last;
      if (r_kind == K_RAW) o_data <= r_raw;
      else if (r_kind == K_VTX && r_nan) o_data <= 40'h00_8000_0000;
      else if (r_kind == K_VTX && r_sat) o_data <= r_sgn ? 40'h00_8000_0000 : 40'h00_7fff_ffff;
      else o_data <= r_sgn ? (40'd0 - {1'b0, r_mag}) : {1'b0, r_mag};
    end
  end

endmodule
