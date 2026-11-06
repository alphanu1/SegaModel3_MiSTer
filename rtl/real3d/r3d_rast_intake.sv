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
// Record (SETUP4 = 1, the default: R65, m2-notes.md section 15): words 0-3
// the polygon header (word 0 [4] texture on; word 3 fog intensity, layer,
// facing), 4-9 the y-sorted screen x, y (FP32), 10 det, then per parameter
// pdx, pdy, ps (FP32): 1/w, u/w, v/w, shade textured (23 words), 1/w, shade
// untextured (17). SETUP4 = 0 takes R62's record (section 14): a 3-word
// header and five parameters textured, w first.
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
// header words go out first, tag 31 (four beats; three with SETUP4 = 0). No
// parameter tag reaches 31 (the last is 27), so every tag names one field.
//
// MIP = 1 (study D-33, R67): the header has one more word, the geometry's LOD
// word B (r3d_geo LOD_WORD), and its tag-31 beat carries the triangle's LOD
// terms in its place: [16:0] K = 97664 - {E(B), T[B's top six mantissa
// bits]} (T the 64-entry 256 log2 table; 0 when B is not positive and finite,
// which gives level 0), [20:17] the highest level 4 + min(width code, height
// code). r3d_rast_interp's i_lk and i_lvm take them.

module r3d_rast_intake #(
  parameter bit SETUP4 = 1'b1,
  parameter bit MIP = 1'b0
) (
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
  localparam logic [4:0] HW = (SETUP4 ? 5'd4 : 5'd3) + (MIP ? 5'd1 : 5'd0);    // header words

  // MIP: 256 log2(1 + (i + 1/2) / 64), rounded (r3d_rast_interp's table)
  function automatic logic [7:0] log2t(input logic [5:0] i);
    case (i)
      6'd0: log2t = 8'd3;    6'd1: log2t = 8'd9;    6'd2: log2t = 8'd14;   6'd3: log2t = 8'd20;
      6'd4: log2t = 8'd25;   6'd5: log2t = 8'd30;   6'd6: log2t = 8'd36;   6'd7: log2t = 8'd41;
      6'd8: log2t = 8'd46;   6'd9: log2t = 8'd51;   6'd10: log2t = 8'd56;  6'd11: log2t = 8'd61;
      6'd12: log2t = 8'd66;  6'd13: log2t = 8'd71;  6'd14: log2t = 8'd75;  6'd15: log2t = 8'd80;
      6'd16: log2t = 8'd85;  6'd17: log2t = 8'd89;  6'd18: log2t = 8'd94;  6'd19: log2t = 8'd98;
      6'd20: log2t = 8'd103; 6'd21: log2t = 8'd107; 6'd22: log2t = 8'd111; 6'd23: log2t = 8'd116;
      6'd24: log2t = 8'd120; 6'd25: log2t = 8'd124; 6'd26: log2t = 8'd128; 6'd27: log2t = 8'd132;
      6'd28: log2t = 8'd136; 6'd29: log2t = 8'd140; 6'd30: log2t = 8'd144; 6'd31: log2t = 8'd148;
      6'd32: log2t = 8'd152; 6'd33: log2t = 8'd155; 6'd34: log2t = 8'd159; 6'd35: log2t = 8'd163;
      6'd36: log2t = 8'd167; 6'd37: log2t = 8'd170; 6'd38: log2t = 8'd174; 6'd39: log2t = 8'd178;
      6'd40: log2t = 8'd181; 6'd41: log2t = 8'd185; 6'd42: log2t = 8'd188; 6'd43: log2t = 8'd192;
      6'd44: log2t = 8'd195; 6'd45: log2t = 8'd198; 6'd46: log2t = 8'd202; 6'd47: log2t = 8'd205;
      6'd48: log2t = 8'd208; 6'd49: log2t = 8'd212; 6'd50: log2t = 8'd215; 6'd51: log2t = 8'd218;
      6'd52: log2t = 8'd221; 6'd53: log2t = 8'd224; 6'd54: log2t = 8'd228; 6'd55: log2t = 8'd231;
      6'd56: log2t = 8'd234; 6'd57: log2t = 8'd237; 6'd58: log2t = 8'd240; 6'd59: log2t = 8'd243;
      6'd60: log2t = 8'd246; 6'd61: log2t = 8'd249; 6'd62: log2t = 8'd252; default: log2t = 8'd255;
    endcase
  endfunction
  logic [3:0] lvm_r;          // MIP: 4 + min(width code, height code), from word 0

  wire adv = !o_valid || o_ready;

  // ---- record state ------------------------------------------------------------

  logic [4:0]  wc;            // word in the record
  logic [2:0]  np_left;       // parameters still to come
  logic [1:0]  pk;            // coefficient within the parameter: 0 pdx, 1 pdy, 2 ps
  logic [2:0]  pnum;          // parameter number
  logic [31:0] pw [3];        // the parameter's words, pdx, pdy, ps
  logic [8:0]  emx;           // running max of E (+9 for the gradients), 0 = none
  logic signed [8:0] emx_e;   // its e: 2 for none, else emx - 124 (study R109: em_e is chosen, not
                              // formed after the max; the max's select and its zero test were in series)

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
      emx_e   <= 9'sd2;
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
        if (wc < HW) begin
          f_v <= 1'b1; f_tag <= 5'd31; f_kind <= K_RAW;
          if (wc == 5'd0) np_left <= i_data[4] ? (SETUP4 ? 3'd4 : 3'd5) : 3'd2;
          if (MIP && wc == 5'd0) lvm_r <= 4'd4 + {1'b0, (i_data[8:6] < i_data[11:9]) ? i_data[8:6] : i_data[11:9]};
          if (MIP && wc == HW - 5'd1) begin
            logic ok;
            ok = !i_data[31] && i_data[30:23] != 8'hff && i_data[30:0] != 31'd0;
            f_raw <= {19'd0, lvm_r, ok ? 17'(17'd97664 - {1'b0, i_data[30:23], log2t(i_data[22:17])}) : 17'd0};
          end
        end else if (wc < HW + 5'd6) begin
          f_v <= 1'b1; f_tag <= 5'(wc - HW); f_kind <= K_VTX;
        end else if (wc == HW + 5'd6) begin
          f_v <= 1'b1; f_tag <= 5'd6; f_kind <= K_RAW;
          pk <= 2'd0; pnum <= 3'd0; emx <= 9'd0; emx_e <= 9'sd2;
        end else begin
          // a coefficient: keep it, fold its exponent in
          logic [8:0] ea, mx;
          logic signed [8:0] ee;
          logic       gt;
          ea = eadj(i_data, pk != 2'd2);
          gt = ea > emx;
          mx = gt ? ea : emx;
          ee = gt ? $signed(9'(ea - 9'd124)) : emx_e;          // ea > emx: ea is not 0
`ifdef VERILATOR
          if (ee != ((mx == 9'd0) ? 9'sd2 : $signed(9'(mx - 9'd124)))) $fatal(1, "intake: em_e chosen");
`endif
          pw[pk] <= i_data;
          if (pk == 2'd2) begin
            em_act  <= 1'b1;
            em_k    <= 3'd0;
            em_p    <= pnum;
            em_e    <= ee;
            em_last <= np_left == 3'd1;
            np_left <= np_left - 3'd1;
            pnum    <= pnum + 3'd1;
            pk      <= 2'd0;
            emx     <= 9'd0; emx_e <= 9'sd2;
          end else begin
            pk  <= pk + 2'd1;
            emx <= mx; emx_e <= ee;
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
