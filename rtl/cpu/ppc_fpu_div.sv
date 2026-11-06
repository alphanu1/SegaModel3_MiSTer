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
// PowerPC 603e iterative unit: fdiv, fdivs, fres, frsqrte. Blocking: one
// operation at a time, start / busy. The 603e blocks its FPU for fdiv(s) and
// fres the same way [603e 6.4.3].
//
// Restoring digit recurrences on the significands:
//   divide       radix 4, two quotient bits a cycle: the shifted remainder
//                against B, 2B and 3B on three subtractors side by side (3B
//                formed once), the largest that fits taken. Two radix-2 steps
//                chained in one cycle measured 11.3 ns, -3 ns at 120 MHz.
//   square root  radix 2 on the first subtractor: Rs = 4R + next two radicand
//                bits against 4Q + 1, one root bit a cycle. Radix 4 would
//                need 24Q + 9, an add in the loop, to save 28 cycles of frsqrte.
//
// The result is not rounded here: the quotient (56 bits -- 53, guard, round
// and one spare -- with the remainder as sticky) goes into the multiply-add
// unit's back end at its R4 stage, which normalises, denormalises and rounds
// it as it does its own sums. The FPU has one rounder.
//
//   fdiv    a / b, 56 quotient bits (28 cycles of recurrence)
//   fdivs   a / b, 28 quotient bits (14), rounded once to single
//   fres    1.0 / b as fdivs. UNDEFINED bits [PEM 8-79]: the 603e runs fres
//           in fdivs's 18-cycle slot and Supermodel's note reads "on the 603
//           fres behaves the same as fdivs RT, 1.0, RB"; FR and FI from that
//           rounding; XX not changed (fres's own list, PEM 8-80).
//   frsqrte UNDEFINED bits [PEM 8-82]: Supermodel's 1.0 / sqrt(b) -- the
//           root to 56 bits (56 cycles), rounded to double here in the
//           current RN, then 1.0 divided by it (28), rounded to double by the
//           back end. FR and FI from the second rounding; XX not changed
//           (PEM 8-83).
//
// Denormal operands are normalised one bit a cycle before the recurrence
// (the census's divides have none).
//
// Against my Model 1 / Model 2 fp_div (rtl/tgp/fp_div.sv: radix-2 restoring
// on single significands, 29 cycles, its own rounding): the same restoring
// recurrence, radix 4 for the divide here, the square root on the same
// subtractor, and the rounding shared with the multiply-add unit -- the
// reciprocal-on-the-pool idea of m2_fp_pool taken one step further.

module ppc_fpu_div
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW  = 6,
  // frsqrte here (1), or not built (0: ppc_fpu_fast runs it as a pipelined
  // estimate, ppc_fpu_rsqe, study D-22)
  parameter bit RSQRT = 1'b1,
  // R4 = 1: the divide two quotient bits a cycle on three subtractors (2B, 4B,
  // 6B). 0 (study R100): one bit a cycle on one, the same restoring digits
  // (a radix-4 digit is two radix-2 steps), twice the cycles: fdiv 28 -> 56,
  // fdivs and fres 14 -> 28
  parameter bit R4    = 1'b0
) (
  input  logic            clk,
  input  logic            rst,

  input  logic            start,      // operands valid this cycle only
  input  div_op_t         op,
  input  logic            single,     // fdivs (fres is single whatever this says)
  input  logic [7:0]      ctl,        // FPSCR[24:31]
  input  logic [63:0]     a,          // fdiv dividend
  input  logic [63:0]     b,          // divisor / operand
  input  logic [TAGW-1:0] tag,
  output logic            busy,

  output logic            inj_valid,
  output logic            inj_next,   // inj_valid next cycle and not taken by then
  input  logic            inj_ready,  // taken this cycle
  output logic            inj_special,
  output logic [51:0]     inj_payload,
  output logic            inj_sign,
  output logic [55:0]     inj_q,
  output logic            inj_sticky,
  output logic [13:0]     inj_e,
  output logic [13:0]     inj_e0c,    // registered here and not retimed (preserve below)
  output logic [7:0]      inj_sden,
  output logic            inj_cap,
  output logic            inj_single,
  output logic [7:0]      inj_ctl,
  output logic            inj_xx_en,
  output fpu_exc_t        inj_exc,
  output logic            inj_invalid,
  output logic            inj_zdiv,
  output logic [TAGW-1:0] inj_tag
);

  localparam int EW = 14;
  typedef logic signed [EW-1:0] exp_t;

  function automatic exp_t xe(input logic [10:0] e);
    xe = (e == 11'd0) ? exp_t'(-1022) : exp_t'({3'b000, e}) - exp_t'(1023);
  endfunction

  typedef enum logic [3:0] { S_IDLE, S_CHK, S_NORM, S_LOAD, S_ITER, S_FIN, S_RINC, S_RND, S_B3, S_INJ } state_t;
  state_t st;

  div_op_t         r_op;
  logic            r_sgl, r_sqrt;
  logic [7:0]      ctl_q;
  logic [TAGW-1:0] tag_q;
  logic            o_spc, o_sign, o_zero, o_pa, o_pb, o_den;
  exp_t            o_e, ea, eb;
  fpu_exc_t        o_exc;

  logic [59:0] rs;      // partial remainder (divide: twice it; root: at most 2Q < 2^57)
  logic [55:0] q;       // quotient / root bits; a special's payload
  logic [53:0] bx;      // divisor, or the radicand still to bring in
  logic [54:0] b3;      // 3 * divisor
  logic [5:0]  cnt;
  logic        nz;      // rs != 0, registered in S_FIN (the 60-bit OR was on two critical paths)

  // ------------------------------------------- classification, at start
  wire        a_s = a[63], b_s = b[63];
  wire [10:0] a_e = a[62:52], b_e = b[62:52];
  wire [51:0] a_f = a[51:0], b_f = b[51:0];
  wire a_zero = (a_e == 0) && (a_f == 0), b_zero = (b_e == 0) && (b_f == 0);
  wire a_inf  = (a_e == 11'h7FF) && (a_f == 0), b_inf = (b_e == 11'h7FF) && (b_f == 0);
  wire a_nan  = (a_e == 11'h7FF) && (a_f != 0), b_nan = (b_e == 11'h7FF) && (b_f != 0);
  wire a_snan = a_nan & ~a_f[51], b_snan = b_nan & ~b_f[51];

  logic        k_spc, k_zero, k_sign;
  fpu_exc_t    k_exc;
  always_comb begin
    k_spc = 1'b0; k_zero = 1'b0; k_sign = 1'b0; k_exc = '0;
    case (op)
      DIV_DIV: begin
        k_exc.vxsnan = a_snan | b_snan;
        k_exc.vxidi  = a_inf & b_inf;
        k_exc.vxzdz  = a_zero & b_zero;
        k_exc.zx     = b_zero & ~a_zero & ~a_inf & ~a_nan;
        k_sign       = a_s ^ b_s;
        if (a_nan)                        begin k_spc = 1'b1; k_sign = a_s; end
        else if (b_nan)                   begin k_spc = 1'b1; k_sign = b_s; end
        else if ((a_inf & b_inf) | (a_zero & b_zero))
                                          begin k_spc = 1'b1; k_sign = 1'b0; end
        else if (b_zero | a_inf)          k_spc = 1'b1;               // infinity
        else if (b_inf | a_zero)          k_zero = 1'b1;
      end
      DIV_RES: begin
        k_exc.vxsnan = b_snan;
        k_exc.zx     = b_zero;
        k_sign       = b_s;
        if (b_nan)       begin k_spc = 1'b1; end
        else if (b_zero) k_spc = 1'b1;
        else if (b_inf)  k_zero = 1'b1;
      end
      default: begin  // frsqrte
        k_exc.vxsnan = b_snan;
        k_exc.vxsqrt = b_s & ~b_zero & ~b_nan;
        k_exc.zx     = b_zero;
        k_sign       = b_zero & b_s;                                  // sqrt(-0) -> -inf
        if (b_nan)                         begin k_spc = 1'b1; k_sign = b_s; end
        else if (b_s & ~b_zero)            begin k_spc = 1'b1; k_sign = 1'b0; end
        else if (b_zero)                   k_spc = 1'b1;
        else if (b_inf)                    k_zero = 1'b1;
      end
    endcase
  end

  // --------------------------------------------------------- recurrence
  // divide: the remainder is held at twice its value, so the first step (whose
  // digit is q0 q1 of a/b, in (0.5, 2): 2A against dB) is 4R against 2dB like
  // every other, and no mux stands in front of the subtractors (it cost
  // 0.13 ns at 120 MHz); against 2B, 4B, 6B
  wire [59:0] r4  = {rs[57:0], 2'b00};
  wire [59:0] dv1 = {5'd0, bx, 1'b0};
  wire [59:0] dv2 = {4'd0, bx, 2'b00};
  wire [59:0] dv3 = {4'd0, b3, 1'b0};
  // root: Rs = 4R + next two bits against 4Q + 1
  wire [59:0] rq  = {rs[57:0], bx[53:52]};
  wire [59:0] tq  = {2'b00, q, 2'b01};

  wire [59:0] r2  = {rs[58:0], 1'b0};                 // radix 2: 2R against 2B
  wire [60:0] u1 = r_sqrt ? ({1'b0, rq} - {1'b0, tq}) : ({1'b0, R4 ? r4 : r2} - {1'b0, dv1});
  wire [60:0] u2 = {1'b0, r4} - {1'b0, dv2};
  wire [60:0] u3 = {1'b0, r4} - {1'b0, dv3};
  wire ge1 = ~u1[60], ge2 = ~u2[60], ge3 = ~u3[60];

  logic [1:0]  dig;
  logic [59:0] rnext;
  always_comb begin
    if (r_sqrt) begin
      dig   = {1'b0, ge1};
      rnext = ge1 ? u1[59:0] : rq;
    end else if (!R4) begin
      dig   = {1'b0, ge1};
      rnext = ge1 ? u1[59:0] : r2;
    end else if (ge3) begin dig = 2'd3; rnext = u3[59:0]; end
    else if (ge2)     begin dig = 2'd2; rnext = u2[59:0]; end
    else if (ge1)     begin dig = 2'd1; rnext = u1[59:0]; end
    else              begin dig = 2'd0; rnext = r4; end
  end

  // last count: 28 bits (single) or 56, two a cycle dividing; 55 root steps
  // after the first, made when the recurrence is loaded
  wire [5:0] n_iter = r_sqrt ? 6'd55 : !R4 ? (r_sgl ? 6'd27 : 6'd55) : (r_sgl ? 6'd13 : 6'd27);

  // frsqrte: the root rounded to double in the current RN (always positive)
  wire        g_rt   = q[2];
  wire        x_rt   = (q[1:0] != 2'b00) || nz;
  logic       inc_rt;
  always_comb
    case (ctl_q[1:0])
      2'b00:   inc_rt = g_rt & (x_rt | q[3]);
      2'b10:   inc_rt = g_rt | x_rt;
      default: inc_rt = 1'b0;
    endcase
  logic       inc_q;
  wire [53:0] root_r = {1'b0, q[55:3]} + {53'd0, inc_q};

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
    end else begin
      case (st)
        S_IDLE: if (start) begin
          r_op   <= op;
          r_sgl  <= (op == DIV_RES) | ((op == DIV_DIV) & single);
          r_sqrt <= RSQRT && (op == DIV_RSQRTE);
          ctl_q  <= ctl;
          tag_q  <= tag;
          o_exc  <= k_exc;
          o_sign <= k_sign;
          o_spc  <= k_spc;
          o_zero <= k_zero;
          nz     <= 1'b0;
          // The operands are loaded whatever their class and the decision is
          // made in S_CHK from registered flags (taken here, the class of the
          // start cycle's operands steered the state and every working
          // register: -0.07 ns at 120 MHz). frA is loaded as it is; fres's
          // dividend 1.0 is set when the recurrence is loaded, frsqrte loads
          // its own. A NaN result's payload is chosen in S_CHK from these.
          q  <= {4'd0, a_f};
          o_pa <= (op == DIV_DIV) & a_nan;
          o_pb <= b_nan & ~((op == DIV_DIV) & a_nan);
          o_den <= ((op == DIV_DIV) && a_e == 11'd0) || b_e == 11'd0;
          rs <= {7'd0, a_e != 0, a_f};
          ea <= (op == DIV_DIV) ? xe(a_e) : exp_t'(0);
          bx <= {1'b0, b_e != 0, b_f};
          eb <= xe(b_e);
          st <= S_CHK;
        end

        // shift denormal significands up to a leading one (the dividend only
        // for fdiv)
        S_NORM: begin
          if (r_op == DIV_DIV && !rs[52]) begin rs <= {rs[58:0], 1'b0}; ea <= ea - exp_t'(1); end
          if (!bx[52]) begin bx <= {bx[52:0], 1'b0}; eb <= eb - exp_t'(1); end
          if ((r_op != DIV_DIV || rs[52] || rs[51]) && (bx[52] || bx[51])) st <= S_LOAD;
        end

        // S_CHK: a special or zero result takes its fraction -- frA's NaN,
        // frB's NaN (quieted), the default QNaN, or zero -- and is handed
        // over; denormal operands go to S_NORM; anything else loads the
        // recurrence here, as S_LOAD does after S_NORM
        S_CHK, S_LOAD: begin
          if (st == S_CHK && (o_spc || o_zero)) begin
            q  <= {4'd0, o_pa ? (q[51:0] | (52'd1 << 51)) :
                         o_pb ? (bx[51:0] | (52'd1 << 51)) :
                         (o_exc.vxidi | o_exc.vxzdz | o_exc.vxsqrt) ? (52'd1 << 51) : 52'd0};
            st <= S_INJ;
          end else if (st == S_CHK && o_den) begin
            st <= S_NORM;
          end else begin
            if (r_op == DIV_RES) rs <= {7'd0, 53'h10_0000_0000_0000};
            b3    <= {1'b0, bx} + {bx, 1'b0};
            st    <= S_ITER;
            if (r_sqrt) begin
              // radicand in [1, 4): b's significand, doubled when its exponent
              // is odd; the root's exponent is the halved one. The first root
              // bit is always 1: R = top pair - 1.
              q   <= 56'd1;
              cnt <= 6'd1;
              if (eb[0]) begin
                rs  <= {58'd0, bx[52:51]} - 60'd1;
                bx  <= {bx[50:0], 3'b000};
                o_e <= (eb - exp_t'(1)) >>> 1;
              end else begin
                rs  <= {59'd0, bx[52]} - 60'd1;
                bx  <= {bx[51:0], 2'b00};
                o_e <= eb >>> 1;
              end
            end else begin
              q   <= 56'd0;
              cnt <= 6'd0;
              o_e <= ea - eb;
            end
          end
        end

        S_ITER: begin
          rs    <= rnext;
          if (r_sqrt) begin
            q  <= {q[54:0], dig[0]};
            bx <= {bx[51:0], 2'd0};
          end else if (!R4) begin
            q  <= {q[54:0], dig[0]};
          end else begin
            q  <= {q[53:0], dig};
          end
          cnt <= cnt + 6'd1;
          if (cnt == n_iter) st <= S_FIN;
        end

        // frsqrte: round the root to double (the increment decided a cycle
        // ahead), then divide 1.0 by it
        S_FIN: begin
          nz <= (rs != 60'd0);
          st <= r_sqrt ? S_RINC : S_INJ;
        end

        S_RINC: begin
          inc_q <= inc_rt;
          st    <= S_RND;
        end

        S_RND: begin
          r_sqrt <= 1'b0;
          rs     <= {7'd0, 53'h10_0000_0000_0000};
          if (root_r[53]) begin
            bx  <= {1'b0, 53'h10_0000_0000_0000};
            o_e <= exp_t'(0) - (o_e + exp_t'(1));
          end else begin
            bx  <= {1'b0, root_r[52:0]};
            o_e <= exp_t'(0) - o_e;
          end
          st <= S_B3;
        end

        S_B3: begin
          b3    <= {1'b0, bx} + {bx, 1'b0};
          q     <= 56'd0;
          cnt   <= 6'd0;
          st    <= S_ITER;
        end

        S_INJ: if (inj_ready) st <= S_IDLE;

        default: st <= S_IDLE;
      endcase
    end
  end

  assign busy = (st != S_IDLE);

  // The back end's denormal cap for the quotient (doubles, UE=0), formed from
  // o_e two cycles ahead: o_e is last written at least 14 cycles before S_INJ,
  // and a special or zero result does not use it.
  // (two register steps: the clamp, then s_den; o_e is stable long before)
  wire  cap_d = ~r_sgl & ~ctl_q[5] & ~o_spc;
  exp_t e0_d, e0c_d, sden_d;
  // E0 = e - 55, a single's quotient (q[27:0]) e - 27: it enters the back
  // end where it is, no realignment mux
  assign e0_d   = o_e - (r_sgl ? exp_t'(27) : exp_t'(55));
  assign e0c_d  = (cap_d && e0_d < exp_t'(-1182)) ? exp_t'(-1182) : e0_d;
  assign sden_d = e0c_q + exp_t'(1182);
  // kept as registers: retimed back into this logic they put the cap's
  // decode, the injection select and the stopper OR behind one register in
  // the back end (-0.46 ns at 120 MHz)
  (* preserve *) logic [13:0] e0c_q;
  (* preserve *) logic [7:0]  sden_q;
  (* preserve *) logic        cap_q;
  assign inj_e0c  = e0c_q;
  assign inj_sden = sden_q;
  assign inj_cap  = cap_q;
  logic cap_p;
  always_ff @(posedge clk) begin
    e0c_q  <= e0c_d;
    cap_p  <= cap_d;
    sden_q <= (!cap_p || sden_d > exp_t'(161)) ? 8'd255 : sden_d[7:0];
    cap_q  <= cap_p;
  end

  // a single quotient is in q[27:0], a double's in q[55:0]; specials and
  // zeros have q cleared (zero) or carry the payload (special, inj_q unused)
  assign inj_valid   = (st == S_INJ);
  assign inj_next    = (st == S_INJ && !inj_ready) || (st == S_CHK && (o_spc || o_zero)) ||
                       (st == S_FIN && !r_sqrt);
  assign inj_special = o_spc;
  assign inj_payload = q[51:0];
  assign inj_sign    = o_sign;
  assign inj_q       = q;
  assign inj_sticky  = ~o_spc & ~o_zero & nz;
  assign inj_e       = o_e;
  assign inj_single  = r_sgl;
  assign inj_ctl     = ctl_q;
  assign inj_xx_en   = (r_op == DIV_DIV);
  assign inj_exc     = o_exc;
  assign inj_invalid = o_exc.vxsnan | o_exc.vxidi | o_exc.vxzdz | o_exc.vxsqrt;
  assign inj_zdiv    = o_exc.zx;
  assign inj_tag     = tag_q;

endmodule
