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
// PowerPC 603e multiply-add unit: fadd, fsub, fmul, fmadd, fmsub, fnmadd,
// fnmsub in both precisions, frsp, fctiw and fctiwz. Fully pipelined, one
// operation a cycle, fixed latency fma_latency(P1A ... P7) (ppc_fpu_pkg): 12
// with every boundary in. Standalone at 120 MHz (slow 100C) that closed on one
// seed of three (+0.07 ns; the others 116-117 MHz); 11 (P5A = 0) reached
// 113-117 MHz. The stages are routing-bound, about 1.4 ns a LUT level. Its back
// end also rounds the iterative unit's quotients and roots (inj_* port), so
// the FPU has one normaliser, one rounder and one result port.
//
// One rounding of the infinitely precise result [PEM 3.3.5, 4.2.2.2, D.2]:
// the 106-bit product and the aligned addend are added exactly in a 162-bit
// window, normalised, and rounded once to double or single. Single-precision
// operations use the full double operands (the manual leaves non-single
// operands undefined, PEM 3.3.4; the exact result rounded once is what the
// same datapath gives, and that is the choice taken).
//
// The window V[161:0] (V[i] weighs 2^(E0 + i - 1)):
//   V[106:1]   the product. A denormal multiplicand is normalised first, so
//              the product's leading one is at V[106] or V[105].
//   V[161:109] the addend before alignment; shifted right by sh.
//   V[0]       sticky: the OR of every addend bit shifted to or below it.
// The addend starts two places above the product's top. When it dominates
// (would be shifted by less than zero) E0 is anchored on it instead, and the
// product, then placed higher than its weight, still lies wholly below the
// addend's round bit: it only ever acts as a sticky. For that the addend's
// leading one must be at V[161], so a denormal frB is normalised too wherever
// the result is fully normalised (single precision, frsp, UE=1); a double with
// UE=0 keeps frB as it is and its denormal cap (below) fixes the rounding
// place at 2^-1074, under which the product stays.
//
// Denormal results: a tiny double with UE=0 is denormalised by capping the
// normalise shift where V[161] weighs 2^-1022 (s_den). The anchor is raised
// to keep s_den >= 0, which only ever moves values lying wholly below the
// guard bit. A single-precision result is always fully normalised and rounded
// at a place moved up by d = -126 - E, so it stays a normalised double as the
// FPR holds it [PEM D.4.1].
//
// Special operands (NaN, infinity) ride the datapath: the addend is replaced
// by the result's 52-bit fraction, the product forced to zero, the shift and
// the rounding forced off; S8 sets the exponent to all ones. fctiw rides it
// too: frB aligned so that the integer's LSB is the double LSB, rounded with
// the double rounder, range-checked after rounding [PEM D.4.2].
//
// Stages. R1, R4 and R8 are always registers; P1A ... P7 make the others
// optional so the boundaries can move (each removed one is a cycle less):
//   S1a unpack, classify, special results, LZC of the fractions     -> R1a (P1A)
//   S1b normalise A, C (and B), exponents, alignment shift, anchor  -> R1 (DSP in)
//   S2  products on four DSPs, lh + hl in the chain-out adder;
//       coarse alignment (by 16)                                    -> R2 (P2)
//   S3  P = {hh, ll} + (lh + hl) << 27; fine alignment, sticky       -> R3 (P3)
//   S4a low 82 bits of P +- addend and of addend - P                -> R4a (P4A)
//   S4b high 81 bits, sign, select; or the injected quotient; the
//       denormal cap's stopper bit                                  -> R4
//   S5a both counts' first four levels (S | stopper, and reversed S) -> R5a (P5A)
//   S5b the rest: the leading-zero count is the normalise shift;
//       trailing zeros for the sticky                               -> R5 (P5)
//   S6a normalise shift, first two levels                           -> R6a (P6A)
//   S6b last two levels; sticky (tz + s < 106), exponent, tininess,
//       rounding place, guard / round / sticky there                -> R6 (P6)
//   S7  round: the increment, fctiw candidates                       -> R7 (P7)
//   S8  exponent, overflow, defaults, NaN / inf / zero, fctiw, flags  -> R8
//
// Against my Model 1 / Model 2 FP units (rtl/tgp/fp_add.sv, fp_mul.sv, IEEE
// single, a separate adder and multiplier, 4 stages each): the same split of
// the work into align / add / normalise / round, and two of their timing
// lessons reused -- the alignment shift comes from the exponents alone
// (fp_add R739), and no sticky mask is built with a second shifter (fp_add
// R512; here a trailing-zero count against the shift after normalising, and a
// compare per addend bit for the alignment). Different: one fused unit with
// double width, a 162-bit window instead of 27 bits, denormals in and out, all
// four rounding modes and the full FPSCR flag set; theirs flush or round to
// nearest only. Their 4-stage split ran at 80-98 MHz; 120 MHz with double
// width takes 12.

// The side-band fields pass through several registers unchanged, which
// Quartus turns into altshift_taps RAM (three M10K) unless told not to; M10K
// binds as hard as ALM on this part (study 8.4, R26's fetch queue).
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *)
module ppc_fpu_fma
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW = 6,
  parameter bit P1A = 1'b1,
  parameter bit P2  = 1'b1,
  parameter bit P3  = 1'b1,
  parameter bit P4A = 1'b1,
  parameter bit P5A = 1'b1,
  parameter bit P5  = 1'b1,
  parameter bit P6A = 1'b1,
  parameter bit P6  = 1'b1,
  parameter bit P7  = 1'b1
) (
  input  logic            clk,
  input  logic            rst,

  // issue: operands straight from the core's EX-stage registers
  input  logic            in_valid,
  input  fma_op_t         in_op,
  input  logic            in_single,  // opcode 59 form (frsp: 0, it rounds to single anyway)
  input  logic [7:0]      in_ctl,     // FPSCR[24:31]: VE OE UE ZE XE NI RN
  input  logic [63:0]     in_a,
  input  logic [63:0]     in_b,
  input  logic [63:0]     in_c,
  input  logic [TAGW-1:0] in_tag,

  // the iterative unit's result, rounded here (enters at R4)
  input  logic            inj_valid,
  input  logic            inj_next,   // inj_valid will be high next cycle (and not yet taken)
  output logic            inj_ready,  // the injection is taken this cycle
  input  logic            inj_special,// NaN or infinity: inj_payload is the fraction
  input  logic [51:0]     inj_payload,
  input  logic            inj_sign,
  input  logic [55:0]     inj_q,      // q[55] weighs 2^inj_e; 0 with inj_sticky 0 is an exact zero
  input  logic            inj_sticky,
  input  logic [13:0]     inj_e,
  input  logic [13:0]     inj_e0c,    // E0 = inj_e - 55, raised to -1182 under the denormal cap
  input  logic [7:0]      inj_sden,   // the cap's shift limit (255: none)
  input  logic            inj_cap,
  input  logic            inj_single,
  input  logic [7:0]      inj_ctl,
  input  logic            inj_xx_en,  // XX follows FI (0 for fres and frsqrte)
  input  fpu_exc_t        inj_exc,
  input  logic            inj_invalid,
  input  logic            inj_zdiv,
  input  logic [TAGW-1:0] inj_tag,

  // a finished result from another unit (ppc_fpu_fast: the single unit, the
  // estimate pipe), taken into this unit's output register; the issue rules
  // keep it out of cycles that carry a result of this unit's own
  input  logic            x_valid,
  input  logic [TAGW-1:0] x_tag,
  input  logic [63:0]     x_res,
  input  fpu_flags_t      x_flags,

  output logic            out_valid,
  output logic [TAGW-1:0] out_tag,
  output logic            out_div,    // the result is the iterative unit's
  output logic [63:0]     out_res,
  output fpu_flags_t      out_flags
);
  // (R140) the restructurings' own checks; R140_NOCHK turns them off for the planted-fault runs
`ifdef R140_NOCHK
  localparam bit R1C = 1'b0;
`else
  localparam bit R1C = 1'b1;
`endif

  localparam int EW = 14;  // signed exponents: the product of two denormals reaches -2252
  typedef logic signed [EW-1:0] exp_t;

  // unbiased exponent; a denormal (or zero) reads as -1022 with a hidden 0
  function automatic exp_t xe(input logic [10:0] e);
    xe = (e == 11'd0) ? exp_t'(-1022) : exp_t'({3'b000, e}) - exp_t'(1023);
  endfunction

  // everything that travels with an operation to the end
  typedef struct packed {
    logic [TAGW-1:0] tag;
    logic            div;      // the iterative unit's result
    logic            single;   // round to single, single FPRF class
    logic            negr;     // fnmadd / fnmsub
    logic            cvt, cvt_spc, cvt_nan, cvt_big;
    logic            special, sp_isnan;
    logic [1:0]      rmode;
    logic            ve, oe, ue, ze, ni;
    logic            xx_en;
    logic            invalid, zdiv;
    fpu_exc_t        exc;
  } side_t;

  // ================================================================= S1a
  logic has_a, has_b, has_c, subb, negr, cvt, cvtz, rsp;
  always_comb begin
    has_a = 1'b1; has_b = 1'b1; has_c = 1'b1; subb = 1'b0; negr = 1'b0;
    cvt = 1'b0; cvtz = 1'b0; rsp = 1'b0;
    case (in_op)
      FMA_ADD:   has_c = 1'b0;
      FMA_SUB:   begin has_c = 1'b0; subb = 1'b1; end
      FMA_MUL:   has_b = 1'b0;
      FMA_MADD:  ;
      FMA_MSUB:  subb = 1'b1;
      FMA_NMADD: negr = 1'b1;
      FMA_NMSUB: begin negr = 1'b1; subb = 1'b1; end
      FMA_RSP:   begin has_a = 1'b0; has_c = 1'b0; rsp = 1'b1; end
      FMA_CTIW:  begin has_a = 1'b0; has_c = 1'b0; cvt = 1'b1; end
      FMA_CTIWZ: begin has_a = 1'b0; has_c = 1'b0; cvt = 1'b1; cvtz = 1'b1; end
      default:   ;
    endcase
  end

  // fadd and fsub multiply frA by 1.0
  // (frC is classified as it arrives and 1.0 substituted after the tests,
  // so the op decode is not in front of them: 0.85 ns measured)
  wire        a_s = in_a[63], b_s = in_b[63], c_s = has_c & in_c[63];
  wire [10:0] a_e = in_a[62:52], b_e = in_b[62:52];
  wire [51:0] a_f = in_a[51:0], b_f = in_b[51:0], c_f = has_c ? in_c[51:0] : 52'd0;
  wire [10:0] c_er = in_c[62:52];

  wire a_e0 = (a_e == 11'd0), b_e0 = (b_e == 11'd0), c_e0 = has_c & (c_er == 11'd0);
  wire a_e1 = (a_e == 11'h7FF), b_e1 = (b_e == 11'h7FF), c_e1 = has_c & (c_er == 11'h7FF);
  wire a_f0 = (a_f == 52'd0), b_f0 = (b_f == 52'd0), c_f0 = ~has_c | (in_c[51:0] == 52'd0);

  wire a_zero = a_e0 & a_f0, b_zero = b_e0 & b_f0, c_zero = c_e0 & c_f0;
  wire a_inf  = a_e1 & a_f0, b_inf  = b_e1 & b_f0, c_inf  = c_e1 & c_f0;
  wire a_nan  = a_e1 & ~a_f0, b_nan = b_e1 & ~b_f0, c_nan = c_e1 & ~c_f0;
  wire a_snan = a_nan & ~a_f[51], b_snan = b_nan & ~b_f[51], c_snan = c_nan & ~in_c[51];

  // Leading zeros of the 53-bit significands {hidden, fraction}, beside the
  // exponent decode: the normalise shift itself, 0 for a normal number and
  // lz(fraction) + 1 for a denormal (53 for a zero, whose value stays 0), so
  // no denormal test or select follows the count. Counting the fraction alone
  // and selecting by "denormal" after it missed 112 MHz by 0.26 ns once the
  // fast port shared the operand registers.
  logic [5:0] lzfa, lzfb, lzfc;
  ppc_fpu_lzc #(.W(53)) u_lza (.clk (clk), .x({~a_e0, a_f}), .n(lzfa));
  ppc_fpu_lzc #(.W(53)) u_lzb (.clk (clk), .x({~b_e0, b_f}), .n(lzfb));
  // (frC counted raw and 1.0's count, 0, chosen after: the op decode was in
  // front of the count, R63 batch 3)
  logic [5:0] lzfc_r;
  ppc_fpu_lzc #(.W(53)) u_lzc (.clk (clk), .x({c_er != 11'd0, in_c[51:0]}), .n(lzfc_r));
  assign lzfc = has_c ? lzfc_r : 6'd0;

  wire single_e = (in_single & ~cvt) | rsp;
  // frB only where it is normalised, see the header (a zero frB counts too:
  // its value stays 0, and its exponent is then unused)
  wire       bnrm = b_e0 & (single_e | in_ctl[5]) & ~cvt;
  wire [5:0] na = lzfa;
  wire [5:0] nc = lzfc;
  wire [5:0] nb = bnrm ? lzfb : 6'd0;

  exp_t ea0, ec0, eb0, esum, ep00, sh_cvt;
  assign ea0    = xe(a_e);
  assign ec0    = xe(c_er);
  assign eb0    = xe(b_e);
  // both sums formed, frC's (or 1.0's, exponent 0) picked after the adds:
  // the op decode in front of them missed 120 MHz by 0.32 ns
  exp_t esum_c, esum_1, ep00_c, ep00_1;
  // (frC's exponent, the late one, through the stfs and zero logic in front,
  // goes into one add after the others' part: as written left to right it
  // went through three carry chains, a D-36 cached full build's fc_q into
  // esum; the sums are the same modulo the width: one chain after ec0)
  // xe(e) = e' - 1023, e' being e with bit 0 forced to 1 where e = 0 (a zero
  // exponent's -1022), so frB's and frC's exponents go into the sums raw,
  // with no xe adder in front, and each sum is one (ternary) add after them.
  // (Round 3 of the 100 MHz pass: frB's xe, then ea0 - eb0, + 56 and + ec0
  // were four carry chains from fb_q into esum.)
  wire  [10:0] b_x = {b_e[10:1], b_e[0] | b_e0};
  wire  [10:0] c_x = {c_er[10:1], c_er[0] | (c_er == 11'd0)};
  exp_t ea56, ea1079, ea1127;
  assign ea56   = ea0 + exp_t'(56);
  assign ea1079 = ea0 + exp_t'(1079);
  assign ea1127 = ea0 - exp_t'(1127);
  assign esum_1 = ea1079 - exp_t'({3'b000, b_x});
  assign ep00_1 = ea0 - exp_t'(104);
  assign esum_c = ea56 + exp_t'({3'b000, c_x}) - exp_t'({3'b000, b_x});
  assign ep00_c = ea1127 + exp_t'({3'b000, c_x});
`ifdef VERILATOR
  always_ff @(posedge clk)
    if ({esum_1, esum_c, ep00_c} != {ea0 - eb0 + exp_t'(56), ea0 + ec0 - eb0 + exp_t'(56), ea0 + ec0 - exp_t'(104)})
      $fatal(1, "fma exponent sums from raw exponents");
`endif
  assign esum   = has_c ? esum_c : esum_1;
  assign ep00   = has_c ? ep00_c : ep00_1;
  assign sh_cvt = exp_t'(52) - eb0;

  // The operand classes and the decoded operation, registered: the special-
  // operand logic and its consequences are formed in S1b, beside the
  // normalise shift (behind the decode here they missed 120 MHz by 0.34 ns).
  typedef struct packed {
    logic has_a, has_b, has_c, subb, negr, cvt, cvtz, single_e;
    logic a_s, b_s, c_s;
    logic a_zero, a_inf, a_nan, a_snan;
    logic b_zero, b_inf, b_nan, b_snan, b_e1;
    logic c_zero, c_inf, c_nan, c_snan;
    logic [7:0]      ctl;
    logic [TAGW-1:0] tag;
  } cls_t;
  cls_t k1;
  always_comb begin
    k1.has_a = has_a;  k1.has_b = has_b;  k1.has_c = has_c;  k1.subb = subb;  k1.negr = negr;
    k1.cvt = cvt;  k1.cvtz = cvtz;  k1.single_e = single_e;
    k1.a_s = a_s;  k1.b_s = b_s;  k1.c_s = c_s;
    k1.a_zero = a_zero;  k1.a_inf = a_inf;  k1.a_nan = a_nan;  k1.a_snan = a_snan;
    k1.b_zero = b_zero;  k1.b_inf = b_inf;  k1.b_nan = b_nan;  k1.b_snan = b_snan;  k1.b_e1 = b_e1;
    k1.c_zero = c_zero;  k1.c_inf = c_inf;  k1.c_nan = c_nan;  k1.c_snan = c_snan;
    k1.ctl = in_ctl;  k1.tag = in_tag;
  end

  typedef struct packed {
    logic [52:0] a, b, c;
    logic [5:0]  na, nb, nc;
    exp_t        esum, ep00, eb0, sh_cvt;
    cls_t        k;
  } r1a_t;

  r1a_t s1a_d, r1a_q;
  logic r1a_v;
  always_comb begin
    s1a_d.a = {~a_e0, a_f};
    s1a_d.b = {~b_e0, b_f};
    s1a_d.c = {~c_e0, c_f};
    s1a_d.na = na;  s1a_d.nb = nb;  s1a_d.nc = nc;
    s1a_d.esum = esum;  s1a_d.ep00 = ep00;  s1a_d.eb0 = eb0;  s1a_d.sh_cvt = sh_cvt;
    s1a_d.k = k1;
  end

  generate if (P1A) begin : g_r1a
    always_ff @(posedge clk) begin
      if (rst) r1a_v <= 1'b0;
      else     r1a_v <= in_valid;
      r1a_q <= s1a_d;
    end
  end else begin : g_c1a
    assign r1a_v = in_valid;
    assign r1a_q = s1a_d;
  end endgenerate

  // ================================================================= S1b
  cls_t kq;
  assign kq = r1a_q.k;

  // special operands: NaN priority frA, frB, frC [PEM 3.3.1.7]; the product
  // meets the addend as two separate instructions would [PEM D-4]
  wire pa_nan  = kq.has_a & kq.a_nan;
  wire pb_nan  = kq.has_b & kq.b_nan;
  wire pc_nan  = kq.has_c & kq.c_nan;
  wire nan_any = pa_nan | pb_nan | pc_nan;
  wire vxsnan  = (kq.has_a & kq.a_snan) | (kq.has_b & kq.b_snan) | (kq.has_c & kq.c_snan);

  wire sp_  = kq.a_s ^ kq.c_s;                 // product sign
  wire sbe  = kq.b_s ^ kq.subb;                // addend sign as added
  wire imz  = kq.has_a & kq.has_c & ~kq.a_nan & ~kq.c_nan & ((kq.a_inf & kq.c_zero) | (kq.a_zero & kq.c_inf));
  wire pinf = kq.has_a & (kq.a_inf | kq.c_inf) & ~kq.a_nan & ~(kq.has_c & kq.c_nan) & ~imz;
  wire isi  = kq.has_b & pinf & kq.b_inf & (sp_ != sbe);
  wire binf = kq.has_b & kq.b_inf;

  // fctiw: NaN and infinity are invalid converts; |frB| >= 2^53 never fits
  wire cvt_nan = kq.cvt & kq.b_nan;
  wire cvt_spc = kq.cvt & (kq.b_nan | kq.b_inf);
  wire cvt_big = kq.cvt & ~kq.b_e1 & (r1a_q.eb0 > exp_t'(52));

  wire special = ~kq.cvt & (nan_any | imz | isi | pinf | binf);
  // the special result's sign, and which fraction it takes
  logic       sp_sign;
  logic [1:0] sp_sel;   // 0 frA's NaN, 1 frB's, 2 frC's, 3 none (default QNaN or infinity)
  always_comb begin
    if (pa_nan)          begin sp_sign = kq.a_s;  sp_sel = 2'd0; end
    else if (pb_nan)     begin sp_sign = kq.b_s;  sp_sel = 2'd1; end
    else if (pc_nan)     begin sp_sign = kq.c_s;  sp_sel = 2'd2; end
    else if (imz | isi)  begin sp_sign = 1'b0; sp_sel = 2'd3; end
    else if (pinf)       begin sp_sign = sp_;  sp_sel = 2'd3; end
    else                 begin sp_sign = sbe;  sp_sel = 2'd3; end
  end

  // Which of product and addend take part. A zero product still has a sign
  // (the zero-sum rule) but no value; an absent one (frsp, fctiw) takes the
  // addend's sign, so the operation is an addition.
  wire prod_on   = kq.has_a & ~special;
  wire prod_live = prod_on & ~kq.a_zero & ~kq.c_zero;
  wire add_on    = special | (kq.has_b & ~kq.b_zero);
  wire r_sp      = special ? sp_sign : (kq.has_a ? sp_ : sbe);
  wire r_sb      = special ? sp_sign : (kq.has_b ? sbe : sp_);
  wire effsub    = ~special & (r_sp ^ r_sb);

  side_t s1_sd;
  always_comb begin
    s1_sd          = '0;
    s1_sd.tag      = kq.tag;
    s1_sd.single   = kq.single_e;
    s1_sd.negr     = kq.negr;
    s1_sd.cvt      = kq.cvt;
    s1_sd.cvt_spc  = cvt_spc;
    s1_sd.cvt_nan  = cvt_nan;
    s1_sd.cvt_big  = cvt_big;
    s1_sd.special  = special;
    s1_sd.sp_isnan = nan_any | imz | isi;
    s1_sd.rmode    = kq.cvtz ? 2'b01 : kq.ctl[1:0];
    s1_sd.ve       = kq.ctl[7];
    s1_sd.oe       = kq.ctl[6];
    s1_sd.ue       = kq.ctl[5];
    s1_sd.ze       = kq.ctl[4];
    s1_sd.ni       = kq.ctl[2];
    s1_sd.xx_en    = 1'b1;
    s1_sd.invalid  = vxsnan | imz | isi | cvt_spc;
    s1_sd.exc.vxsnan = vxsnan;
    s1_sd.exc.vximz  = imz;
    s1_sd.exc.vxisi  = isi;
    s1_sd.exc.vxcvi  = cvt_spc;
  end

  wire [52:0] a_nrm = r1a_q.a << r1a_q.na;
  wire [52:0] c_nrm = r1a_q.c << r1a_q.nc;
  wire [52:0] b_nrm = r1a_q.b << r1a_q.nb;

  exp_t ep0, ebn, sh_raw;
  // the normalise amounts, summed on 8 bits first so only one 14-bit add follows
  wire [7:0] sac = {2'b00, r1a_q.na} + {2'b00, r1a_q.nc};
  wire [7:0] sbb = {2'b00, r1a_q.nb};
  assign ep0    = r1a_q.ep00 - exp_t'({6'd0, sac});
  assign ebn    = r1a_q.eb0 - exp_t'({6'd0, sbb});
  assign sh_raw = r1a_q.esum + exp_t'({6'd0, sbb}) - exp_t'({6'd0, sac});

  // V[i] weighs 2^(E0 + i - 1). Product-anchored E0 = ep0, the addend's hidden
  // bit then at V[161 - sh], sh = ep0 - eb + 160. Addend-anchored E0 = eb - 160.
  // fctiw: V[161] weighs 2^52, so the integer's LSB lands on the double LSB.
  exp_t e0;
  logic [7:0] sh, sh_o;
  wire spc1 = special;
  always_comb begin
    if (kq.cvt) begin
      e0 = exp_t'(-108);
      sh_o = (r1a_q.sh_cvt > exp_t'(255)) ? 8'd255 : r1a_q.sh_cvt[7:0];
    end else if (spc1 || !prod_live) begin
      e0 = ebn - exp_t'(160);
      sh_o = 8'd0;
    end else if (!add_on || sh_raw < 0) begin
      e0 = add_on ? (ebn - exp_t'(160)) : ep0;
      sh_o = add_on ? 8'd0 : 8'd255;
    end else begin
      e0 = ep0;
      sh_o = (sh_raw > exp_t'(255)) ? 8'd255 : sh_raw[7:0];
    end
  end
  // sh as above with the special-operand decision last (timing, study R104: kq's classes through
  // imz, pinf, isi and special into the coarse shift copies was a 110 MHz system build's FPU path).
  // Where special is clear, prod_live is has_a & ~a_zero & ~c_zero and add_on is has_b & ~b_zero.
  wire       pl_ns = kq.has_a & ~kq.a_zero & ~kq.c_zero;
  wire       ab_ns = kq.has_b & ~kq.b_zero;
  wire [7:0] sh_ns = !pl_ns ? 8'd0 : !ab_ns ? 8'd255 : (sh_raw < 0) ? 8'd0 :
                     (sh_raw > exp_t'(255)) ? 8'd255 : sh_raw[7:0];
  wire [7:0] sh_cv = (r1a_q.sh_cvt > exp_t'(255)) ? 8'd255 : r1a_q.sh_cvt[7:0];
  assign sh = kq.cvt ? sh_cv : (sh_ns & {8{~special}});
`ifdef R104_CHECK
  always_ff @(posedge clk) if (!rst && r1a_v && sh != sh_o) $fatal(1, "fma sh restructured");
`endif

  // a NaN operand quieted, the default QNaN, or zero (infinity)
  logic [51:0] sp_frac;
  always_comb
    case (sp_sel)
      2'd0:    sp_frac = r1a_q.a[51:0] | (52'd1 << 51);
      2'd1:    sp_frac = r1a_q.b[51:0] | (52'd1 << 51);
      2'd2:    sp_frac = r1a_q.c[51:0] | (52'd1 << 51);
      default: sp_frac = s1_sd.sp_isnan ? (52'd1 << 51) : 52'd0;
    endcase
  wire [52:0] b_use = spc1 ? {1'b0, sp_frac} : (add_on ? b_nrm : 53'd0);

  // ------------------------------------------------------------- R1
  // DSP operands: 53 = 26 + 27 bits each. The product is forced to zero in S3
  // (pon), not on the DSP inputs: gated here it sat behind the special-operand
  // logic, 0.4 ns over.
  logic [26:0] r1_alo, r1_clo;
  logic [25:0] r1_ahi, r1_chi;
  logic [52:0] r1_b;
  logic [7:0]  r1_sh;
  // the coarse shift drives 178 muxes and 53 compares: a copy per slice
  // (one register missed 120 MHz by 0.21 ns on its fanout)
  (* preserve *) logic [3:0] r1_shc [4];
  exp_t        r1_e0;
  logic        r1_v, r1_sp, r1_sb, r1_effsub, r1_pon;
  side_t       r1_sd;

  always_ff @(posedge clk) begin
    if (rst) r1_v <= 1'b0;
    else     r1_v <= r1a_v;
    r1_alo    <= a_nrm[26:0];
    r1_ahi    <= a_nrm[52:27];
    r1_pon    <= prod_on;
    r1_clo    <= c_nrm[26:0];
    r1_chi    <= c_nrm[52:27];
    r1_b      <= b_use;
    r1_sh     <= sh;
    for (int k = 0; k < 4; k++) r1_shc[k] <= sh[7:4];
    r1_e0     <= e0;
    r1_sp     <= r_sp;
    r1_sb     <= r_sb;
    r1_effsub <= effsub;
    r1_sd     <= s1_sd;
  end

  // ================================================================== S2
  // Four 27x27 products. The two middle ones have the same weight; written as
  // one sum they map onto two DSP blocks joined by the chain-out adder.
  wire [53:0] pp_ll = r1_alo * r1_clo;
  wire [51:0] pp_hh = r1_ahi * r1_chi;
  wire [53:0] pp_x  = r1_alo * r1_chi + r1_ahi * r1_clo;

  // Coarse alignment: the addend at V[161:109] of a frame extended 16 places
  // below V[0], shifted right by 16 * sh[7:4]. Bits that leave the extension
  // are the coarse sticky -- a compare per addend bit (bit i sits at 125 + i
  // and is lost when that is below 16k), not a second shifter (fp_add R512).
  logic [177:0] alc_k [4];
  logic [177:0] alc;
  always_comb begin
    for (int k = 0; k < 4; k++) alc_k[k] = {r1_b, 125'd0} >> {r1_shc[k], 4'b0000};
    for (int i = 0; i < 178; i++) alc[i] = alc_k[i / 45][i];
  end
  logic [52:0] c_lost;
  always_comb
    for (int i = 0; i < 53; i++) c_lost[i] = (9'(i) + 9'd125 < {1'b0, r1_shc[3], 4'b0000});
  wire cst2 = |(r1_b & c_lost);

  typedef struct packed {
    logic [53:0]  ll, x;
    logic [51:0]  hh;
    logic [177:0] alc;
    logic         cst;
    logic [3:0]   shf;
    logic         pon;
    exp_t         e0;
    logic         sp, sb, effsub;
    side_t        sd;
  } r2_t;

  r2_t  s2_d, r2_q;
  logic r2_v;
  always_comb begin
    s2_d.ll = pp_ll;  s2_d.x = pp_x;  s2_d.hh = pp_hh;
    s2_d.alc = alc;  s2_d.cst = cst2;  s2_d.shf = r1_sh[3:0];  s2_d.pon = r1_pon;
    s2_d.e0 = r1_e0;  s2_d.sp = r1_sp;  s2_d.sb = r1_sb;  s2_d.effsub = r1_effsub;  s2_d.sd = r1_sd;
  end

  generate if (P2) begin : g_r2
    always_ff @(posedge clk) begin
      if (rst) r2_v <= 1'b0;
      else     r2_v <= r1_v;
      r2_q <= s2_d;
    end
  end else begin : g_c2
    assign r2_v = r1_v;
    assign r2_q = s2_d;
  end endgenerate

  // ================================================================== S3
  wire [105:0] prod = r2_q.pon ? ({r2_q.hh, r2_q.ll} + {25'd0, r2_q.x, 27'd0}) : 106'd0;

  // Fine alignment by sh[3:0]. Every bit of the frame that ends at or below
  // V[0] (frame position 16) is sticky: the extension always, and a frame bit
  // j in 17..31 when j - 16 <= shift.
  wire [177:0] alf = r2_q.alc >> r2_q.shf;
  logic [15:0] f_lost;
  always_comb
    for (int j = 0; j < 16; j++) f_lost[j] = (5'(j) + 5'd1 <= {1'b0, r2_q.shf});
  wire ast3 = r2_q.cst | (|r2_q.alc[16:0]) | (|(r2_q.alc[32:17] & f_lost));
  wire [161:0] al3 = {alf[177:17], ast3};

  typedef struct packed {
    logic [105:0] p;
    logic [161:0] al;
    exp_t         e0;
    logic         sp, sb, effsub;
    side_t        sd;
  } r3_t;

  r3_t  s3_d, r3_q;
  logic r3_v;
  always_comb begin
    s3_d.p = prod;  s3_d.al = al3;
    s3_d.e0 = r2_q.e0;  s3_d.sp = r2_q.sp;  s3_d.sb = r2_q.sb;  s3_d.effsub = r2_q.effsub;  s3_d.sd = r2_q.sd;
  end

  generate if (P3) begin : g_r3
    always_ff @(posedge clk) begin
      if (rst) r3_v <= 1'b0;
      else     r3_v <= r2_v;
      r3_q <= s3_d;
    end
  end else begin : g_c3
    assign r3_v = r2_v;
    assign r3_q = s3_d;
  end endgenerate

  // ================================================================= S4a
  // T1 = P + addend or P - addend; T2 = addend - P (used when T1 < 0); 163
  // bits with bit 162 the sign. Two binary adders, cut at bit 82.
  wire [162:0] pw   = {56'd0, r3_q.p, 1'b0};
  wire [162:0] alw  = {1'b0, r3_q.al};
  wire         sub4 = r3_q.effsub;
  wire [162:0] alx  = sub4 ? ~alw : alw;

  wire [82:0]  t1_lo = {1'b0, pw[81:0]} + {1'b0, alx[81:0]} + {82'd0, sub4};
  wire [82:0]  t2_lo = {1'b0, alw[81:0]} + {1'b0, ~pw[81:0]} + 83'd1;

  // The denormal cap (doubles, UE=0): V[161] may weigh no less than 2^-1022.
  // E0 is raised to -1182 if it is lower; s_den = E0 + 1182 is how far the
  // normalise shift may go. Over 161 it never limits. Formed here, a stage
  // before it is used, so S4b only selects it (the iterative unit forms its
  // own the same way).
  wire  cap4a = ~r3_q.sd.single & ~r3_q.sd.ue & ~r3_q.sd.special & ~r3_q.sd.cvt;
  exp_t e0c4a, sden4a;
  assign e0c4a  = (cap4a && r3_q.e0 < exp_t'(-1182)) ? exp_t'(-1182) : r3_q.e0;
  assign sden4a = e0c4a + exp_t'(1182);

  typedef struct packed {
    logic [81:0] t1, t2;
    logic        c1, c2;
    logic [80:0] p_hi, a_hi;         // bits 162:82 of pw, alw
    logic [7:0]  sden;
    logic        cap;
    exp_t        e0;
    logic        sp, sb, effsub;
    side_t       sd;
  } r4a_t;

  r4a_t s4a_d, r4a_q;
  logic r4a_v;
  always_comb begin
    s4a_d.t1 = t1_lo[81:0];  s4a_d.c1 = t1_lo[82];
    s4a_d.t2 = t2_lo[81:0];  s4a_d.c2 = t2_lo[82];
    s4a_d.p_hi = pw[162:82];  s4a_d.a_hi = alw[162:82];
    s4a_d.sden = (!cap4a || sden4a > exp_t'(161)) ? 8'd255 : sden4a[7:0];
    s4a_d.cap = cap4a;
    s4a_d.e0 = e0c4a;  s4a_d.sp = r3_q.sp;  s4a_d.sb = r3_q.sb;  s4a_d.effsub = sub4;  s4a_d.sd = r3_q.sd;
  end

  generate if (P4A) begin : g_r4a
    always_ff @(posedge clk) begin
      if (rst) r4a_v <= 1'b0;
      else     r4a_v <= r3_v;
      r4a_q <= s4a_d;
    end
  end else begin : g_c4a
    assign r4a_v = r3_v;
    assign r4a_q = s4a_d;
  end endgenerate

  // ================================================================= S4b
  wire [80:0] x_hi  = r4a_q.effsub ? ~r4a_q.a_hi : r4a_q.a_hi;
  wire [80:0] t1_hi = r4a_q.p_hi + x_hi + {80'd0, r4a_q.c1};
  wire [80:0] t2_hi = r4a_q.a_hi + ~r4a_q.p_hi + {80'd0, r4a_q.c2};
  wire        neg4  = r4a_q.effsub & t1_hi[80];

  // the injected quotient: q[55] at V[56], so E0 = e - 55; a special puts its
  // fraction where the addend's would be, unshifted
  wire [161:0] inj_s = inj_special ? {1'b0, inj_payload, 109'd0}
                                   : {105'd0, inj_q, inj_sticky};

  side_t inj_sd;
  always_comb begin
    inj_sd          = '0;
    inj_sd.tag      = inj_tag;
    inj_sd.div      = 1'b1;
    inj_sd.single   = inj_single;
    inj_sd.special  = inj_special;
    inj_sd.sp_isnan = inj_special & (inj_payload != 52'd0);
    inj_sd.rmode    = inj_ctl[1:0];
    inj_sd.ve       = inj_ctl[7];
    inj_sd.oe       = inj_ctl[6];
    inj_sd.ue       = inj_ctl[5];
    inj_sd.ze       = inj_ctl[4];
    inj_sd.ni       = inj_ctl[2];
    inj_sd.xx_en    = inj_xx_en;
    inj_sd.invalid  = inj_invalid;
    inj_sd.zdiv     = inj_zdiv;
    inj_sd.exc      = inj_exc;
  end

  // The hand-over takes R4's slot when no operation is in S4b. Decided a
  // cycle ahead, from the valid that will be in S4b then, so the select of
  // the 162-bit mux is a register of its own (the divider's state bit drove it
  // directly and the stage missed by 0.5 ns).
  wire v_pre4b = P4A ? r3_v : P3 ? r2_v : P2 ? r1_v : r1a_v;
  (* preserve *) logic take_q [4];
  always_ff @(posedge clk)
    for (int k = 0; k < 4; k++) take_q[k] <= ~rst & inj_next & ~v_pre4b;
  assign inj_ready = take_q[0];
  wire   take_inj  = take_q[0];

  wire [161:0] s4 = take_q[1] ? inj_s : neg4 ? {t2_hi[79:0], r4a_q.t2} : {t1_hi[79:0], r4a_q.t1};
  side_t sd4;
  assign sd4 = take_q[2] ? inj_sd : r4a_q.sd;
  exp_t e04;
  assign e04 = take_inj ? exp_t'(inj_e0c) : r4a_q.e0;

  // The normalise shift is min(lz, s_den) under the denormal cap, 0 when
  // forced (special, fctiw). Rather than a compare and a mux after the count,
  // a stopper one is planted at bit 161 - s_den (bit 161 when forced) and the
  // count of S | stopper is the shift; the stopper is decoded here, beside
  // the adder. Whether the cap bites -- the result is tiny, lz > s_den -- is
  // S's own bit at the stopper: 0 there and a count of s_den (tested in S6).
  wire [7:0] sden4 = take_q[3] ? inj_sden : r4a_q.sden;
  wire       cap4  = take_q[3] ? inj_cap  : r4a_q.cap;
  wire       force4 = sd4.special | sd4.cvt;
  logic [161:0] stop4;
  always_comb
    for (int i = 0; i < 162; i++)
      stop4[i] = force4 ? (i == 161) : (cap4 && sden4 != 8'd255 && 8'(161 - i) == sden4);

  typedef struct packed {
    logic [161:0] s, stop;
    exp_t         e0;
    logic [7:0]   sden;     // 255: no cap
    logic         cap;
    logic         sign, effsub;
    side_t        sd;
  } r4_t;

  // R4 is kept where it is (preserve: not retimed): moved forward past the
  // stopper and the count's first level, the injected s_den path missed
  // 120 MHz by 0.46 ns
  (* preserve *) r4_t r4;
  logic r4_v;
  always_ff @(posedge clk) begin
    if (rst) r4_v <= 1'b0;
    else     r4_v <= r4a_v | take_inj;
    r4.s      <= s4;
    r4.e0     <= e04;
    r4.sden   <= sden4;
    r4.cap    <= cap4;
    r4.stop   <= stop4;
    r4.sign   <= take_inj ? inj_sign : (neg4 ? r4a_q.sb : r4a_q.sp);
    r4.effsub <= take_inj ? 1'b0 : r4a_q.effsub;
    r4.sd     <= sd4;
  end

  // ================================================================== S5
  // The counts are trees; P5A cuts both after their fourth level (S5a | S5b),
  // the rest of S5 is formed in S5a and registered beside them.
  wire force5 = r4.sd.special | r4.sd.cvt;
  logic [7:0] lz;
  ppc_fpu_lzc #(.W(162), .REG(P5A), .CUT(4)) u_lz (.clk (clk), .x(r4.s | r4.stop), .n(lz));
  wire zero5 = (r4.s == 162'd0);
  // trailing zeros, for the sticky: S6 sets it when the lowest one lands
  // below the kept 56 bits (tz + s < 106); gathering the dropped bits through
  // the shifter's four levels measured 0.4 ns longer
  logic [161:0] s_rev;
  always_comb for (int i = 0; i < 162; i++) s_rev[i] = r4.s[161 - i];
  logic [7:0] tz;
  ppc_fpu_lzc #(.W(162), .REG(P5A), .CUT(4)) u_tz (.clk (clk), .x(s_rev), .n(tz));
  wire sbit5 = |(r4.s & r4.stop);   // S's own bit where the stopper went

  // S6 compares the shift against these instead of forming E = E0 + 160 - s
  // and then comparing E: E < -126 is s > E0 + 286 (and E < -151 is
  // s > E0 + 311, tested as s - (E0 + 286) > 25), E < -1022 is s > E0 + 1182.
  exp_t k286, k1182;
  assign k286  = r4.e0 + exp_t'(286);
  assign k1182 = r4.e0 + exp_t'(1182);

  typedef struct packed {
    logic [161:0] s;
    logic [7:0]   sden;
    logic         zero, sbit, cap;
    exp_t         e0, k286, k1182;
    logic         sign, effsub;
    side_t        sd;
  } r5a_t;
  r5a_t s5a_d, q5;
  logic v5a;
  always_comb begin
    s5a_d.s = r4.s;  s5a_d.sden = r4.sden;
    s5a_d.zero = zero5;  s5a_d.sbit = sbit5;  s5a_d.cap = r4.cap & (r4.sden != 8'd255) & ~force5;
    s5a_d.e0 = r4.e0;  s5a_d.k286 = k286;  s5a_d.k1182 = k1182;
    s5a_d.sign = r4.sign;  s5a_d.effsub = r4.effsub;  s5a_d.sd = r4.sd;
  end
  generate if (P5A) begin : g_r5a
    always_ff @(posedge clk) begin
      if (rst) v5a <= 1'b0;
      else     v5a <= r4_v;
      q5 <= s5a_d;
    end
  end else begin : g_c5a
    assign v5a = r4_v;
    assign q5  = s5a_d;
  end endgenerate

  typedef struct packed {
    logic [161:0] s;
    logic [7:0]   sh;
    logic [7:0]   sden, tz;
    logic         zero, sbit, cap;
    exp_t         e0, k1182;
    logic [8:0]   k286c;    // E0 + 286 clamped to -26 ... 162 (see S6)
    logic         sign, effsub;
    side_t        sd;
  } r5_t;

  r5_t  s5_d, r5_q;
  logic r5_v;
  // the shift drives 162 muxes on each of four levels: a copy per level
  (* preserve *) logic [7:0] shd [4];
  always_comb begin
    s5_d.s = q5.s;  s5_d.sh = lz;  s5_d.sden = q5.sden;  s5_d.tz = tz;
    s5_d.zero = q5.zero;  s5_d.sbit = q5.sbit;  s5_d.cap = q5.cap;
    s5_d.e0 = q5.e0;  s5_d.k1182 = q5.k1182;
    s5_d.k286c = (q5.k286 < exp_t'(-26)) ? 9'h1E6 : (q5.k286 > exp_t'(162)) ? 9'd162 : q5.k286[8:0];
    s5_d.sign = q5.sign;  s5_d.effsub = q5.effsub;  s5_d.sd = q5.sd;
  end

  generate if (P5) begin : g_r5
    always_ff @(posedge clk) begin
      if (rst) r5_v <= 1'b0;
      else     r5_v <= v5a;
      r5_q <= s5_d;
      for (int k = 0; k < 4; k++) shd[k] <= lz;
    end
  end else begin : g_c5
    assign r5_v = v5a;
    assign r5_q = s5_d;
    always_comb for (int k = 0; k < 4; k++) shd[k] = lz;
  end endgenerate


  // ================================================================== S6
  // Normalise: shift left by s, keep V[161:106]. Four 4:1 levels, coarse
  // first; Quartus drops what cannot reach the kept 56 bits. The 162-bit mux
  // tree is routing-bound (about 2.2 ns a level measured), so P6A can cut it
  // after the second level (S6a | S6b).
  logic [161:0] n1, n2, n3;
  logic [55:0]  n4;
  always_comb begin
    case (shd[0][7:6])
      2'd0:    n1 = r5_q.s;
      2'd1:    n1 = r5_q.s << 64;
      default: n1 = r5_q.s << 128;
    endcase
    case (shd[1][5:4])
      2'd0: n2 = n1;
      2'd1: n2 = n1 << 16;
      2'd2: n2 = n1 << 32;
      default: n2 = n1 << 48;
    endcase
  end

  // Everything of S6 that does not need the shifted value -- sticky,
  // exponent, tininess, the rounding place and its masks, the exponent
  // compares -- is formed here beside the first two levels.
  wire st6 = ~r5_q.zero && ({1'b0, r5_q.tz} + {1'b0, r5_q.sh} < 9'd106);

  exp_t em6;
  assign em6 = r5_q.e0 + exp_t'(160) - exp_t'({6'd0, r5_q.sh});

  // tiny under the cap: the count stopped at s_den and S had no one there
  wire capped6 = r5_q.cap && (r5_q.sh == r5_q.sden) && !r5_q.sbit;
  wire force6 = r5_q.sd.special | r5_q.sd.cvt;
  exp_t sh6, dd6;
  assign sh6 = exp_t'({6'd0, r5_q.sh});
  // -126 - E = s - (E0 + 286); with E0 + 286 clamped to -26 ... 162 (s is
  // 0 ... 162) the single tests and the rounding place d come from one short
  // subtract (two compares and a 14-bit subtract in front of the masks missed
  // 112 MHz by 0.07 ns on one seed)
  assign dd6 = sh6 - exp_t'({{5{r5_q.k286c[8]}}, r5_q.k286c});
  wire lt126  = dd6 > 0;              // E < -126
  wire lt151  = dd6 > exp_t'(25);     // E < -151
  wire lt1022 = sh6 > r5_q.k1182;   // E < -1022
  logic tiny6;
  always_comb begin
    if (force6 || r5_q.zero)  tiny6 = 1'b0;
    else if (r5_q.sd.single)  tiny6 = lt126;
    else if (!r5_q.sd.ue)     tiny6 = capped6;
    else                      tiny6 = lt1022;
  end

  // single, UE=0, tiny: the rounding place moves up by d = -126 - E; beyond
  // 25 the value lies wholly below the guard bit and reads as a sticky at
  // E = -151
  wire       sgl_den = r5_q.sd.single & ~r5_q.sd.ue & tiny6;
  wire       far6 = sgl_den & lt151;
  wire [4:0] d6 = !sgl_den ? 5'd0 : far6 ? 5'd25 : dd6[4:0];
  exp_t em6f;
  assign em6f = far6 ? exp_t'(-151) : em6;

  // the rounding place, decoded here for S7: bit k of {00, n}, k = 3 for a
  // double (and fctiw, whose LSB is the integer's), 32 + d for a single
  logic [57:0] keep6, one6;
  always_comb
    for (int i = 0; i < 58; i++) begin
      keep6[i] = r5_q.sd.single ? (7'(i) >= 7'd32 + {2'b00, d6}) : (i >= 3);
      one6[i]  = r5_q.sd.single ? (7'(i) == 7'd32 + {2'b00, d6}) : (i == 3);
    end

  wire sgl6 = r5_q.sd.single;
  exp_t emax6, adj6;
  assign emax6 = sgl6 ? exp_t'(127) : exp_t'(1023);
  assign adj6  = sgl6 ? exp_t'(192) : exp_t'(1536);

  typedef struct packed {
    logic        st, tiny, wfar;   // wfar: wholly below the guard bit
    logic [57:0] keep, one;
    logic        ovf0, ovf_at, den_at;
    exp_t        bias_n, bias_o, bias_u;   // E + 1023, then - adj / + adj
  } m6_t;
  m6_t m6d, m6q;
  always_comb begin
    m6d.st = st6;  m6d.tiny = tiny6;  m6d.wfar = far6;  m6d.keep = keep6;  m6d.one = one6;
    m6d.ovf0 = (em6f > emax6);  m6d.ovf_at = (em6f == emax6);  m6d.den_at = (em6f == exp_t'(-127));
    m6d.bias_n = em6f + exp_t'(1023);
    m6d.bias_o = em6f + exp_t'(1023) - adj6;
    m6d.bias_u = em6f + exp_t'(1023) + adj6;
  end

  // what the rest of S6 needs; the window itself only from bit 91 up
  r5_t         q6;
  logic [70:0] n2q;
  logic        v6a;
  (* preserve *) logic [3:0] shl6 [2];
  generate if (P6A) begin : g_r6a
    always_ff @(posedge clk) begin
      if (rst) v6a <= 1'b0;
      else     v6a <= r5_v;
      q6  <= r5_q;
      m6q <= m6d;
      n2q <= n2[161:91];
      for (int k = 0; k < 2; k++) shl6[k] <= shd[2 + k][3:0];
    end
  end else begin : g_c6a
    assign v6a = r5_v;
    assign q6  = r5_q;
    assign m6q = m6d;
    assign n2q = n2[161:91];
    always_comb for (int k = 0; k < 2; k++) shl6[k] = shd[2 + k][3:0];
  end endgenerate

  always_comb begin
    case (shl6[0][3:2])
      2'd0: n3 = {n2q, 91'd0};
      2'd1: n3 = {n2q, 91'd0} << 4;
      2'd2: n3 = {n2q, 91'd0} << 8;
      default: n3 = {n2q, 91'd0} << 12;
    endcase
    case (shl6[1][1:0])
      2'd0:    n4 = n3[161:106];
      2'd1:    n4 = n3[160:105];
      2'd2:    n4 = n3[159:104];
      default: n4 = n3[158:103];
    endcase
  end
  // Guard, round and sticky at the rounding place [PEM D.4.1 "Round Single"]:
  // LSB at bit k of {00, n} -- k = 3 for a double (and fctiw, whose LSB is the
  // integer's), 32 + d for a single. Each is ORed here in six groups of ten
  // bits and the groups ORed in S7 (the whole 58-bit select behind the last
  // shift levels left 0.18 ns at 120 MHz).
  wire [57:0] kp6 = m6q.keep, on6 = m6q.one;
  wire [57:0] z6  = m6q.wfar ? 58'd0 : {2'b00, n4};
  wire [57:0] g_m = on6 >> 1, r_m = on6 >> 2;
  wire [59:0] z_l = {2'b00, z6 & on6}, z_g = {2'b00, z6 & g_m}, z_r = {2'b00, z6 & r_m};
  wire [59:0] z_x = {2'b00, z6 & ~kp6 & ~g_m & ~r_m};
  logic [5:0] lsb6, g6, rb6, x6;
  always_comb
    for (int k = 0; k < 6; k++) begin
      lsb6[k] = |z_l[10*k +: 10];
      g6[k]   = |z_g[10*k +: 10];
      rb6[k]  = |z_r[10*k +: 10];
      x6[k]   = |z_x[10*k +: 10];
    end


  typedef struct packed {
    logic [57:0] rt, one;       // the value truncated at the rounding place; its LSB
    logic [5:0]  lsb, g, rb, x;  // in groups, ORed in S7
    logic        xst;            // the sticky below the kept bits
    logic        tiny, zero;
    logic        ovf0, ovf_at, den_at;
    exp_t        bias_n, bias_o, bias_u;   // E + 1023, then - adj / + adj
    logic        sign, effsub;
    side_t       sd;
  } r6_t;

  r6_t  s6_d, r6_q;
  logic r6_v;
  always_comb begin
    s6_d.rt = z6 & kp6;  s6_d.one = on6;
    s6_d.lsb = lsb6;  s6_d.g = g6;  s6_d.rb = rb6;  s6_d.x = x6;  s6_d.xst = m6q.st | m6q.wfar;
    s6_d.tiny = m6q.tiny;  s6_d.zero = q6.zero;
    s6_d.ovf0 = m6q.ovf0;  s6_d.ovf_at = m6q.ovf_at;  s6_d.den_at = m6q.den_at;
    s6_d.bias_n = m6q.bias_n;  s6_d.bias_o = m6q.bias_o;  s6_d.bias_u = m6q.bias_u;
    s6_d.sign = q6.sign;  s6_d.effsub = q6.effsub;  s6_d.sd = q6.sd;
  end

  generate if (P6) begin : g_r6
    always_ff @(posedge clk) begin
      if (rst) r6_v <= 1'b0;
      else     r6_v <= v6a;
      r6_q <= s6_d;
    end
  end else begin : g_c6
    assign r6_v = v6a;
    assign r6_q = s6_d;
  end endgenerate

  // ================================================================== S7
  // Round: the increment for the rounding mode, applied beside the truncated
  // value.
  wire lsb7 = |r6_q.lsb, g7 = |r6_q.g, rb7 = |r6_q.rb, x7 = (|r6_q.x) | r6_q.xst;
  wire [57:0] one = r6_q.one;

  wire inexact7 = ~r6_q.sd.special & (g7 | rb7 | x7);
  logic inc7;
  always_comb begin
    case (r6_q.sd.rmode)
      2'b00:   inc7 = g7 & (rb7 | x7 | lsb7);
      2'b01:   inc7 = 1'b0;
      2'b10:   inc7 = ~r6_q.sign & (g7 | rb7 | x7);
      default: inc7 = r6_q.sign & (g7 | rb7 | x7);
    endcase
    if (r6_q.sd.special) inc7 = 1'b0;
  end

  // truncated and incremented values side by side; inc picks one
  wire [57:0] rt7 = r6_q.rt;
  wire [57:0] ri7 = rt7 + one;
  wire [57:0] rr7 = inc7 ? ri7 : rt7;

  // fctiw candidates: the integer magnitude is r[56:3]; its range check and
  // negation for both outcomes of the rounding
  // (fctiw rounds as a double: the integer is rt[56:3]; built beside the
  // rounding add)
  wire [53:0] mt7 = rt7[56:3];
  wire [31:0] mp7 = mt7[31:0] + 32'd1;
  wire [31:0] mn7 = 32'd0 - mt7[31:0];
  // magnitude over 2^31 - 1 (+) or 2^31 (-); rounded up, over by one less
  wire big_t = r6_q.sign ? (mt7 >  54'h8000_0000) : (mt7 >  54'h7FFF_FFFF);
  wire big_i = r6_q.sign ? (mt7 >= 54'h8000_0000) : (mt7 >= 54'h7FFF_FFFF);
  wire [31:0] lo_t = r6_q.sign ? mn7 : mt7[31:0];
  wire [31:0] lo_i = r6_q.sign ? ~mt7[31:0] : mp7;

  // The biased exponent of the delivered value is one of five, picked in S8
  // once the rounding carry is known: E + 1023 (+ 1536 / 192 when tiny with
  // UE=1), plus 0, 1 or 2; or, overflowing with OE=1, E + 1023 - adj plus 0
  // or 1. Formed here with the tests FPRF needs on each, so S8 only selects.
  // Only the low 11 bits are delivered (see the model's pack()).
  typedef struct packed {
    logic [10:0] v;
    logic        e0, e1, lt897;
  } bexp_t;
  function automatic bexp_t bx_of(input exp_t v);
    bx_of.v     = v[10:0];
    bx_of.e0    = (v[10:0] == 11'd0);
    bx_of.e1    = (v[10:0] == 11'h7FF);
    bx_of.lt897 = (v[10:0] < 11'd897);
  endfunction
  exp_t base_nu;
  assign base_nu = (r6_q.tiny & r6_q.sd.ue) ? r6_q.bias_u : r6_q.bias_n;
  bexp_t bnu0, bnu1, bnu2, bo0, bo1;
  assign bnu0 = bx_of(base_nu);
  assign bnu1 = bx_of(base_nu + exp_t'(1));
  assign bnu2 = bx_of(base_nu + exp_t'(2));
  assign bo0  = bx_of(r6_q.bias_o);
  assign bo1  = bx_of(r6_q.bias_o + exp_t'(1));

  typedef struct packed {
    logic [57:0] r;
    logic        inc, inexact, rzero;
    logic        cvt_big;
    logic [31:0] cvt_lo;
    logic        ovf0, ovf_at, den_at;
    bexp_t       bnu0, bnu1, bnu2, bo0, bo1;
    logic        tiny, zero;
    logic        sign, effsub;
    side_t       sd;
  } r7_t;

  r7_t  s7_d, r7_q;
  logic r7_v;
  always_comb begin
    s7_d.r = rr7;  s7_d.inc = inc7;  s7_d.inexact = inexact7;
    s7_d.rzero = ~inc7 & (rt7 == 58'd0);
    s7_d.cvt_big = inc7 ? big_i : big_t;
    s7_d.cvt_lo  = inc7 ? lo_i : lo_t;
    s7_d.ovf0 = r6_q.ovf0;  s7_d.ovf_at = r6_q.ovf_at;  s7_d.den_at = r6_q.den_at;
    s7_d.bnu0 = bnu0;  s7_d.bnu1 = bnu1;  s7_d.bnu2 = bnu2;  s7_d.bo0 = bo0;  s7_d.bo1 = bo1;
    s7_d.tiny = r6_q.tiny;  s7_d.zero = r6_q.zero;
    s7_d.sign = r6_q.sign;  s7_d.effsub = r6_q.effsub;  s7_d.sd = r6_q.sd;
  end

  generate if (P7) begin : g_r7
    always_ff @(posedge clk) begin
      if (rst) r7_v <= 1'b0;
      else     r7_v <= r6_v;
      r7_q <= s7_d;
    end
  end else begin : g_c7
    assign r7_v = r6_v;
    assign r7_q = s7_d;
  end endgenerate

  // ================================================================== S8
  side_t sd8;
  assign sd8 = r7_q.sd;
  wire [57:0] r8 = r7_q.r;
  wire        c2 = r8[57], c1 = r8[56];
  wire        sgl8 = sd8.single;
  wire        cvt8 = sd8.cvt;
  wire        spc8 = sd8.special;

  // overflow on the rounded result, unbounded exponent [PEM 3.3.6.2.1]
  wire ovf     = ~spc8 & ~cvt8 & ~r7_q.zero & ~r7_q.tiny & (r7_q.ovf0 | (r7_q.ovf_at & c1));
  wire ovf_dis = ovf & ~sd8.oe;

  // biased exponent of the delivered value; c2 only for a single rounded up
  // from wholly below 2^-150
  bexp_t bsel;
  assign bsel = (ovf & sd8.oe) ? (c1 ? r7_q.bo1 : r7_q.bo0)
                               : (c2 ? r7_q.bnu2 : c1 ? r7_q.bnu1 : r7_q.bnu0);
  wire        dbl_den = ~sgl8 & (r8[56:55] == 2'b00);
  wire [10:0] bexp  = dbl_den ? 11'd0 : bsel.v;
  wire [51:0] frac8 = sgl8 ? {r8[54:32], 29'd0} : r8[54:3];

  // NI=1: a denormalised result is delivered as a zero of the same sign
  // (603e 2.3.4.2; PROVISIONAL, flags as for NI=0)
  wire res_den = r7_q.tiny & ~sd8.ue & ~r7_q.rzero & (sgl8 ? ~(r7_q.den_at & c1) : dbl_den);
  wire flush   = sd8.ni & res_den;

  // Table 3-15 defaults for a disabled overflow
  wire to_inf = (sd8.rmode == 2'b00) || (sd8.rmode == 2'b10 && !r7_q.sign) ||
                (sd8.rmode == 2'b11 && r7_q.sign);
  wire [62:0] ovf_mag = to_inf ? {11'h7FF, 52'd0}
                               : (sgl8 ? 63'h47EF_FFFF_E000_0000 : 63'h7FEF_FFFF_FFFF_FFFF);

  // exact zero: x + -x is +0 except toward -inf; a zero product keeps its
  // own sign [PEM 3.3.2]
  wire zsign = r7_q.effsub ? (sd8.rmode == 2'b11) : r7_q.sign;

  // fctiw: range checked after rounding [PEM D.4.2]. UNDEFINED frD[0-31]
  // [PEM 8-62]: Supermodel's -- zero for an in-range result, the sign
  // extension of the clamped word otherwise
  wire cvt_inv = cvt8 & (sd8.cvt_spc | sd8.cvt_big | r7_q.cvt_big);
  wire [31:0] cvt_lo = cvt_inv ? ((r7_q.sign | sd8.cvt_nan) ? 32'h8000_0000 : 32'h7FFF_FFFF) : r7_q.cvt_lo;
  wire [31:0] cvt_hi = (cvt_inv & cvt_lo[31]) ? 32'hFFFF_FFFF : 32'd0;

  // fnmadd / fnmsub: rounded, then negated; a NaN is never negated
  wire flip = sd8.negr & ~(spc8 & sd8.sp_isnan);

  // the result and its class side by side, so FPRF does not wait on the mux;
  // cls_n is fprf_of() of the normal-path value from the precomputed tests
  wire [63:0] res_n = {r7_q.sign, bexp, frac8};
  wire n_e0 = dbl_den | bsel.e0, n_e1 = ~dbl_den & bsel.e1, n_f0 = (frac8 == 52'd0);
  wire [4:0] cls_n = (n_e1 & ~n_f0) ? FPRF_QNAN : n_e1 ? FPRF_PINF : (n_e0 & n_f0) ? FPRF_PZERO :
                     (n_e0 | (sgl8 & bsel.lt897)) ? FPRF_PDEN : FPRF_PNORM;
  logic [63:0] res8;
  logic [4:0]  cls8;
  always_comb begin
    if (cvt8)                 begin res8 = {cvt_hi, cvt_lo};             cls8 = 5'd0; end
    else if (spc8)            begin res8 = {r7_q.sign, 11'h7FF, r8[54:3]};
                                    cls8 = sd8.sp_isnan ? FPRF_QNAN : FPRF_PINF; end
    else if (r7_q.zero)       begin res8 = {zsign, 63'd0};               cls8 = FPRF_PZERO; end
    else if (ovf_dis)         begin res8 = {r7_q.sign, ovf_mag};         cls8 = to_inf ? FPRF_PINF : FPRF_PNORM; end
    else if (r7_q.rzero || flush) begin res8 = {r7_q.sign, 63'd0};       cls8 = FPRF_PZERO; end
    else                      begin res8 = res_n;                        cls8 = cls_n; end
    if (!cvt8 && flip) res8[63] = ~res8[63];
    if (sgl8) res8[28:0] = 29'd0;
  end
  // The result as one-hot kinds ANDed with their values and ORed (R140), in place of res8's priority
  // chain: cvt8, spc8, the zero, ovf_dis, rzero / flush, then the normal value, each in front of the next,
  // were eight levels from r7_q into out_res (the 110 MHz full build c7_s1: r7_q.sd.cvt -> ovf -> ovf_dis
  // -> the chain -> out_res, -1.94 ns, and the same into the single unit's operand select once out_res was
  // retimed forward, -2.24). Every kind is formed from r7_q in two or three levels; a fraction bit is then
  // one LUT of the kinds and its values. res8 above still gives cls8 (its value bits are not used).
  (* keep = 1 *) wire k_b   = ~cvt8 & ~spc8 & ~r7_q.zero;                 // none of the first three
  (* keep = 1 *) wire k_ovd = ~r7_q.tiny & ~sd8.oe & (r7_q.ovf0 | (r7_q.ovf_at & c1));   // ovf_dis but for k_b
  (* keep = 1 *) wire k_ove = ~r7_q.tiny &  sd8.oe & (r7_q.ovf0 | (r7_q.ovf_at & c1));   // ovf & oe but for k_b
  (* keep = 1 *) wire k_fla = sd8.ni & r7_q.tiny & ~sd8.ue;
  (* keep = 1 *) wire k_rzf = r7_q.rzero | (k_fla & (sgl8 ? ~(r7_q.den_at & c1) : (~c1 & ~r8[55])));  // rzero | flush
  (* keep = 1 *) wire k_ovf = k_b & k_ovd;
  (* keep = 1 *) wire k_n   = k_b & ~k_ovd & ~k_rzf;
  (* keep = 1 *) wire k_spc = ~cvt8 & spc8;
  (* keep = 1 *) wire k_sn  = k_spc | k_n;                                   // the fraction is r8's
  (* keep = 1 *) wire k_ovm = k_ovf & ~to_inf;                               // ovf_mag's fraction is all ones
  wire [63:0] cvt_v = {cvt_hi, cvt_lo};
  wire [10:0] bsn = k_ove ? (c1 ? r7_q.bo1.v : r7_q.bo0.v) :                // the normal value's exponent
                    (c2 ? r7_q.bnu2.v : c1 ? r7_q.bnu1.v : r7_q.bnu0.v);
  wire        k_ne  = k_n & ~dbl_den;
  // ovf_mag's exponent: 7FF to infinity, else 47E (single) or 7FE
  wire [10:0] ovf_e = to_inf ? 11'h7FF : sgl8 ? 11'h47E : 11'h7FE;
  logic [63:0] res8f;
  always_comb begin
    for (int b = 0; b < 52; b++)
      res8f[b] = ((cvt8 & cvt_v[b]) | (k_sn & r8[b + 3]) | k_ovm) & ~(sgl8 & (b < 29));
    for (int e = 0; e < 11; e++)
      res8f[52 + e] = (cvt8 & cvt_v[52 + e]) | k_spc | (k_ovf & ovf_e[e]) | (k_ne & bsn[e]);
    res8f[63] = cvt8 ? cvt_v[63] : ((k_b ? r7_q.sign : (spc8 ? r7_q.sign : zsign)) ^ flip);
  end
  // (the zero kind is spc8 = 0, cvt8 = 0, zero = 1: its sign is zsign; every other kind's is r7_q.sign)
`ifdef VERILATOR
  always_ff @(posedge clk) if (R1C && !rst && r7_v && res8f != res8) $fatal(1, "ppc_fpu_fma: S8 by kinds %016X against the chain %016X", res8f, res8);
`endif
  // the class with the delivered sign (a NaN's sign is not part of its class)
  wire [4:0] fprf8 = (cls8 == FPRF_QNAN || !res8[63]) ? cls8 :
                     (cls8 == FPRF_PINF)  ? FPRF_NINF  :
                     (cls8 == FPRF_PZERO) ? FPRF_NZERO :
                     (cls8 == FPRF_PDEN)  ? FPRF_NDEN  : FPRF_NNORM;

  wire        invalid8 = sd8.invalid | cvt_inv;
  // VE=1 invalid and ZE=1 zero divide: no result, FPRF unchanged, FR FI
  // cleared [PEM 3.3.6.1]
  wire        wr8 = ~(invalid8 & sd8.ve) & ~(sd8.zdiv & sd8.ze);
  logic       fr8, fi8;
  fpu_exc_t   exc8;
  always_comb begin
    fr8 = r7_q.inc & ~ovf_dis;
    fi8 = r7_q.inexact | ovf_dis;
    if (invalid8 | sd8.zdiv) begin fr8 = 1'b0; fi8 = 1'b0; end
    exc8       = sd8.exc;
    exc8.vxcvi = sd8.exc.vxcvi | cvt_inv;
    exc8.ox    = sd8.exc.ox | ovf;
    exc8.ux    = sd8.exc.ux | (r7_q.tiny & (sd8.ue | r7_q.inexact));
    exc8.xx    = sd8.exc.xx | (sd8.xx_en & fi8);
  end

  fpu_flags_t flags8;
  always_comb begin
    flags8.wr      = wr8;
    flags8.frfi_we = 1'b1;
    flags8.fr      = fr8;
    flags8.fi      = fi8;
    // UNDEFINED FPRF after fctiw [PEM 8-62]: Supermodel leaves it unchanged
    flags8.fprf_we = wr8 & ~cvt8;
    flags8.fpcc_we = 1'b0;
    flags8.fprf    = fprf8;
    flags8.exc     = exc8;
  end

  always_ff @(posedge clk) begin
    if (rst) out_valid <= 1'b0;
    else     out_valid <= r7_v | x_valid;
    out_tag   <= x_valid ? x_tag : sd8.tag;
    out_div   <= ~x_valid & sd8.div;
    out_res   <= x_valid ? x_res : res8f;
    out_flags <= x_valid ? x_flags : flags8;
  end

  // a bench catches a schedule that lets two results meet here
  // synthesis translate_off
  always_ff @(posedge clk)
    if (!rst && r7_v && x_valid) $error("ppc_fpu_fma: result port clash, tags %0d %0d", sd8.tag, x_tag);
  // synthesis translate_on

endmodule
