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
// Frustum set-up, correctly rounded form: result = tanf((float)atan2(y, x) *
// 0.5f), the value MAME's libm calls give (both correctly rounded: tanf on all
// 1.07 billion floats in [0, pi/2), atan2 on 50 M random pairs).
//
// Fixed-point CORDIC on one shared 66-bit adder and one shared arithmetic
// right shifter whose output is registered (SH), about 470 cycles a call; the
// set-up needs four a viewport.
// The integer algorithm is sim/real3d/r3d_cordic.h, step for step:
//   align x, y (the larger at bit 60) -> vectoring, 62 steps: atan2 in Q3.60
//   -> round to FP32, halve -> rotation, 62 steps from (1/K, 0): cos, sin
//   -> tan = sin / cos, 64 restoring steps, Q2.62 -> round to FP32.
// A CORDIC step is three cycles: SH = Y >> i; X += SH while SH = X >> i (the
// old X); Y -= SH while SH = atan(2^-i); the angle add overlaps the next
// step's first shift. Every register loads from the adder or a fixed shift.
// Values are placed by loading the mantissa at bit 60 and shifting right, and
// rounded after a one-bit-a-cycle normalisation, so no left shifter or
// leading-one detector is built. Rounding uses the bits below as sticky; it is
// exact unless the ~2^-54 error of the iterations straddles a rounding
// boundary. Measured: 8 of 19.2 M random inputs differ from libm, all at
// |angle| < 2^-20 rad; frustum angles are ~0.4 rad.
// Range: finite inputs and |tan| < 4; otherwise result 0 and "bad".

module r3d_setup_cr (
  input  logic        clk,
  input  logic        rst,
  input  logic        in_valid,
  input  logic [31:0] y,
  input  logic [31:0] x,
  output logic        busy,
  output logic        out_valid,
  output logic [31:0] result,
  output logic        bad
);
  localparam logic [65:0] KINV = 66'h0_09b7_4eda_8435_e5a7;
  localparam logic [65:0] PI60 = 66'h0_3243_f6a8_885a_308d;

  // atan(2^-i) in Q3.60: a table below 20, 2^(60-i) above (1 at 61)
  function automatic logic [63:0] atan_lo(input logic [4:0] i);
    case (i)
      5'd0: atan_lo = 64'h0c90fdaa22168c23;
      5'd1: atan_lo = 64'h076b19c1586ed3da;
      5'd2: atan_lo = 64'h03eb6ebf25901bac;
      5'd3: atan_lo = 64'h01fd5ba9aac2f6dc;
      5'd4: atan_lo = 64'h00ffaaddb967ef4e;
      5'd5: atan_lo = 64'h007ff556eea5d893;
      5'd6: atan_lo = 64'h003ffeaab776e535;
      5'd7: atan_lo = 64'h001fffd555bbba97;
      5'd8: atan_lo = 64'h000ffffaaaaddddc;
      5'd9: atan_lo = 64'h0007ffff55556eef;
      5'd10: atan_lo = 64'h0003ffffeaaaab77;
      5'd11: atan_lo = 64'h0001fffffd55555c;
      5'd12: atan_lo = 64'h0000ffffffaaaaab;
      5'd13: atan_lo = 64'h00007ffffff55555;
      5'd14: atan_lo = 64'h00003ffffffeaaab;
      5'd15: atan_lo = 64'h00001fffffffd555;
      5'd16: atan_lo = 64'h00000ffffffffaab;
      5'd17: atan_lo = 64'h000007ffffffff55;
      5'd18: atan_lo = 64'h000003ffffffffeb;
      5'd19: atan_lo = 64'h000001fffffffffd;
      default: atan_lo = 64'h0;
    endcase
  endfunction

  typedef enum logic [4:0] {C_IDLE, C_AL0, C_AL1, C_AL2, C_AL3, C_V0, C_V1, C_V2, C_VZ, C_RZ, C_N, C_RND,
                            C_H, C_H1, C_H2, C_H3, C_R0, C_R1, C_R2, C_RND2,
                            C_D0, C_D1, C_D2, C_D3, C_DK, C_RT, C_OUT} cst_t;
  cst_t st, rret;

  logic [65:0] XR, YR, ZR, SH;   // signed, sign-extended to 66 bits
  logic [63:0] T;
  logic [5:0]  i;
  logic [6:0]  k, nsh;
  logic [6:0]  dx, dy;           // right shifts, saturated at 64
  logic [23:0] my;
  logic        xn, yn, d, tneg, rneg, rok;
  logic [5:0]  rfrac;
  logic [31:0] rres;
  logic [24:0] rm;               // rounded mantissa
  logic        rz;

  // ---- shared right shifter (into SH) and adder -----------------------------
  logic [1:0]  sh_src;           // 0 YR, 1 XR, 2 T (signed), 3 2^60 (the angle)
  logic [6:0]  sh_amt, amt_q;
  logic [1:0]  src_q;
  logic [65:0] sh_in, sh_out;
  logic [2:0]  a_src;            // 0 XR, 1 YR, 2 ZR, 3 zero, 4 {YR, T[63]}
  logic [1:0]  b_src;            // 0 SH, 1 YR, 2 ZR, 3 XR
  logic        b_neg;
  logic [65:0] a_op, b_op, sum;
  always_comb begin
    case (sh_src)
      2'd0: sh_in = YR;
      2'd1: sh_in = XR;
      2'd2: sh_in = {{2{T[63]}}, T};
      default: sh_in = 66'h0_1000_0000_0000_0000;
    endcase
    sh_out = (sh_amt[6]) ? {66{sh_in[65]}} : 66'($signed(sh_in) >>> sh_amt[5:0]);
    case (a_src)
      3'd0: a_op = XR;
      3'd1: a_op = YR;
      3'd2: a_op = ZR;
      3'd4: a_op = {YR[64:0], T[63]};
      default: a_op = '0;
    endcase
    case (b_src)
      2'd0: b_op = SH;
      2'd1: b_op = YR;
      2'd2: b_op = ZR;
      default: b_op = XR;
    endcase
    sum = a_op + (b_neg ? ~b_op : b_op) + {65'd0, b_neg};
  end

  // ---- the operand selects of each state; the shifter's source and amount
  //      are registers, set on the way into the state that shifts
  always_comb begin
    sh_src = src_q; sh_amt = amt_q; a_src = 3'd3; b_src = 2'd0; b_neg = 1'b0;
    case (st)
      C_AL0: ;                                                               // SH = x placed
      C_AL1: ;                                                               // XR = SH; SH = y placed
      C_AL2: ;                                                               // YR = SH
      C_AL3: begin b_src = 2'd1; b_neg = 1'b1; end                          // YR = -YR
      // vectoring: d = Y >= 0
      C_V0, C_VZ: begin a_src = 3'd2; b_neg = !d; end        // SH = Y >> i; Z += -/+ SH (angle of i-1)
      C_V1: begin a_src = 3'd0; b_neg = !d; end              // X += SH; SH = X >> i
      C_V2: begin a_src = 3'd1; b_neg = d; end               // Y -= SH; SH = atan
      C_RZ: begin b_src = 2'd2; b_neg = ZR[65]; end                         // |Z|
      C_H3: begin b_src = 2'd2; b_neg = 1'b1; end                           // -Z
      C_H1: ;                                                                // SH = h placed
      C_H2: ;                                                                // ZR = SH
      // rotation: d = A >= 0
      C_R0: begin a_src = 3'd2; b_neg = d; end               // SH = S >> i; A -= +/- angle (of i-1)
      C_R1: begin a_src = 3'd0; b_neg = !ZR[65]; end         // C -= SH; SH = C >> i
      C_R2: begin a_src = 3'd1; b_neg = !d; end              // S += SH; SH = atan
      C_D0: begin b_src = 2'd1; b_neg = YR[65]; end                         // |S|
      C_D1: ;                                                                // SH = |S| >> 2
      C_D2: ;                                                                // YR = SH
      C_D3: begin a_src = 3'd1; b_src = 2'd3; b_neg = 1'b1; end             // r - C: |S| >= 4 C is out of range
      C_DK: begin a_src = 3'd4; b_src = 2'd3; b_neg = 1'b1; end             // r2 - C
      default: ;
    endcase
  end

  assign busy = (st != C_IDLE);

  always_ff @(posedge clk) begin
    out_valid <= 1'b0;
    SH <= (src_q == 2'd3 && i < 6'd20) ? {2'b00, atan_lo(i[4:0])} : (src_q == 2'd3 && i == 6'd61) ? 66'd1 : sh_out;
    if (rst) begin
      st <= C_IDLE;
    end else case (st)
      C_IDLE: if (in_valid) begin
        logic [7:0] emx;
        logic [8:0] ddx, ddy;
        bad <= 1'b0;
        xn <= x[31]; yn <= y[31];
        emx = (x[30:23] > y[30:23]) ? x[30:23] : y[30:23];
        ddx = {1'b0, emx} - {1'b0, x[30:23]};
        ddy = {1'b0, emx} - {1'b0, y[30:23]};
        // a zero (or subnormal) operand places as zero: shift it out
        dx <= (x[30:23] == 8'd0 || ddx > 9'd63) ? 7'd64 : 7'(ddx);
        src_q <= 2'd2; amt_q <= (x[30:23] == 8'd0 || ddx > 9'd63) ? 7'd64 : 7'(ddx);
        dy <= (y[30:23] == 8'd0 || ddy > 9'd63) ? 7'd64 : 7'(ddy);
        T <= {3'b000, 1'b1, x[22:0], 37'd0};
        my <= {1'b1, y[22:0]};
        if (x[30:23] == 8'hff || y[30:23] == 8'hff) begin
          bad <= 1'b1; result <= 32'd0; out_valid <= 1'b1;
        end else st <= C_AL0;
      end
      C_AL0: begin T <= {3'b000, my, 37'd0}; amt_q <= dy; st <= C_AL1; end
      C_AL1: begin XR <= SH; st <= C_AL2; end
      C_AL2: begin
        YR <= SH;
        if (SH == 66'd0 && !xn) begin
          result <= {yn, 31'd0}; out_valid <= 1'b1;            // atan2(+-0, +x) = +-0: tan(+-0)
          st <= C_IDLE;
        end else begin
          // x < 0: start from +-pi with the vector negated
          ZR <= xn ? (yn ? 66'(-PI60) : PI60) : 66'd0;
          i <= '0; src_q <= 2'd0; amt_q <= '0;
          st <= (xn ^ yn) ? C_AL3 : ((XR == 66'd0 && SH == 66'd0) ? C_RZ : C_V0);
        end
      end
      C_AL3: begin YR <= sum; st <= (XR == 66'd0 && YR == 66'd0) ? C_RZ : C_V0; end
      // vectoring: Y >= 0: X += Y >> i, Y -= X >> i, Z += atan; else the other way
      C_V0: begin d <= !YR[65]; if (i != 6'd0) ZR <= sum; src_q <= 2'd1; st <= C_V1; end
      C_V1: begin XR <= sum; src_q <= 2'd3; st <= C_V2; end
      C_V2: begin YR <= sum; i <= i + 6'd1; src_q <= 2'd0; amt_q <= {1'b0, i + 6'd1}; st <= (i == 6'd61) ? C_VZ : C_V0; end
      C_VZ: begin ZR <= sum; st <= C_RZ; end
      // round atan2 to FP32
      C_RZ: begin
        rneg <= ZR[65]; T <= sum[63:0];
        rfrac <= 6'd60; rret <= C_H; nsh <= '0; st <= C_N;
      end
      // half angle into Q3.60: mantissa at bit 60, shifted right by 1 - (e - 127)
      C_H: begin
        if (!rok) begin bad <= 1'b1; result <= 32'd0; out_valid <= 1'b1; st <= C_IDLE; end
        else if (rres[30:0] == 31'd0) begin result <= rres; out_valid <= 1'b1; st <= C_IDLE; end
        else begin
          T <= {3'b000, 1'b1, rres[22:0], 37'd0};
          src_q <= 2'd2;
          amt_q <= (rres[30:23] >= 8'd128) ? 7'd0 : (8'd128 - rres[30:23] >= 8'd64) ? 7'd64 : 7'(8'd128 - rres[30:23]);
          st <= C_H1;
        end
      end
      C_H1: begin XR <= KINV; YR <= '0; st <= C_H2; end
      C_H2: begin ZR <= SH; st <= rres[31] ? C_H3 : C_R0; i <= '0; src_q <= 2'd0; amt_q <= '0; end
      C_H3: begin ZR <= sum; st <= C_R0; end
      // rotation: A >= 0: C -= S >> i, S += C >> i, A -= atan; else the other way
      C_R0: begin if (i != 6'd0) ZR <= sum; src_q <= 2'd1; st <= C_R1; end
      C_R1: begin XR <= sum; d <= !ZR[65]; src_q <= 2'd3; st <= C_R2; end
      C_R2: begin YR <= sum; i <= i + 6'd1; src_q <= 2'd0; amt_q <= {1'b0, i + 6'd1}; st <= (i == 6'd61) ? C_D0 : C_R0; end
      // tan = |S| / C: q = floor(|S| 2^62 / C) by restoring steps
      C_D0: begin
        if (XR[65] || XR == 66'd0) begin bad <= 1'b1; result <= 32'd0; out_valid <= 1'b1; st <= C_IDLE; end
        else begin tneg <= YR[65]; T <= sum[63:0]; src_q <= 2'd2; amt_q <= 7'd2; st <= C_D1; end
      end
      C_D1: st <= C_D2;
      C_D2: begin YR <= SH; ZR <= '0; T <= {T[1:0], 62'd0}; k <= 7'd1; st <= C_D3; end
      C_D3: begin
        if (!sum[65]) begin bad <= 1'b1; result <= 32'd0; out_valid <= 1'b1; st <= C_IDLE; end   // |S| >= 4 C
        else st <= C_DK;
      end
      C_DK: begin
        if (!sum[65]) YR <= sum; else YR <= {YR[64:0], T[63]};
        ZR <= {ZR[64:0], !sum[65]};
        T <= {T[62:0], 1'b0};
        k <= k + 7'd1;
        if (k == 7'd64) st <= C_RT;
      end
      C_RT: begin
        rneg <= tneg; T <= {ZR[63:1], ZR[0] | (YR != 66'd0)};
        rfrac <= 6'd62; rret <= C_OUT; nsh <= '0; st <= C_N;
      end
      C_OUT: begin
        bad <= !rok;
        result <= rok ? rres : 32'd0;
        out_valid <= 1'b1;
        st <= C_IDLE;
      end
      // ---- rounding: T * 2^-rfrac to FP32, nearest even. Normalise one bit a
      //      cycle (T = 0 stops at once), then round at bit 40 and pack
      C_N: begin
        if (T == 64'd0 || T[63]) st <= C_RND;
        else begin T <= {T[62:0], 1'b0}; nsh <= nsh + 7'd1; end
      end
      // round in two cycles: the increment and the zero test, then the pack
      C_RND: begin
        rm <= {1'b0, T[63:40]} + {24'd0, T[39] && ((|T[38:0]) || T[40])};
        rz <= (T == 64'd0);
        st <= C_RND2;
      end
      C_RND2: begin
        logic [9:0]  e;
        e = 10'd63 - {3'd0, nsh} - {4'd0, rfrac} + 10'd127 + (rm[24] ? 10'd1 : 10'd0);
        if (rz) begin rres <= {rneg, 31'd0}; rok <= 1'b1; end
        else if (e[9] || e == 10'd0 || e >= 10'd255) begin rres <= 32'd0; rok <= 1'b0; end
        else begin rres <= {rneg, e[7:0], rm[24] ? rm[23:1] : rm[22:0]}; rok <= 1'b1; end
        st <= rret;
      end
      default: st <= C_IDLE;
    endcase
  end
endmodule
