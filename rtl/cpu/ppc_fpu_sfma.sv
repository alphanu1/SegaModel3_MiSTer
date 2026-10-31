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
// PowerPC 603e single-precision multiply-add path (study D-21): fadds, fsubs,
// fmuls, fmadds, fmsubs, fnmadds, fnmsubs whose operands are all held exactly
// in single (ppc_fpu_pkg::fpu_fast_route). Pipelined, one op a cycle, latency
// sfma_latency(Q1 ... Q5) with the double unit's output register as the last
// stage (6 with every boundary in).
//
// It gives the double unit's results bit for bit, by the same construction:
// the exact product and the aligned addend added in a window, normalised,
// rounded once to single [PEM 3.3.5, D.2]. Only the widths differ, because
// every operand is a 24-bit significand with an exponent in -149 ... 127:
//   V[74:0], V[i] weighs 2^(E0 + i - 1):
//   V[48:1]  the 48-bit product (its leading one at V[48] or V[47]);
//   V[74:51] the addend before alignment, shifted right by sh;
//   V[0]     sticky: every addend bit shifted to or below it.
// The anchors, the special operands riding the datapath, the zero-sum sign,
// the rounding at a place moved up by d = -126 - E for a tiny result with
// UE=0, the flags and FPRF are ppc_fpu_fma's (see its header), restricted to
// single. Two things are done differently for the shorter pipe:
//  - a tiny result's guard, round and LSB are read from the unnormalised sum
//    at the fixed place 2^-149 (V[-148 - E0]), beside the leading-zero count,
//    instead of through masks after the normalise shift;
//  - the rounding carry selects between two finished results, so the add is
//    not in front of the exponent, FPRF and overflow logic.
//
// Stages (R1 ... R5 optional; the last is the double unit's output register):
//   S1  classify, special result, exponent sums, alignment shift, anchor -> R1 (DSP in)
//   S2  24 x 24 product; alignment shift and its sticky                   -> R2 (DSP out)
//   S3  P +- addend and addend - P, sign                                   -> R3
//   S4  select; leading and trailing zeros; a tiny result's guard and LSB  -> R4
//   S5  normalise; rounding place, guard / sticky; exponent candidates     -> R5
//   S6  round, exponent, overflow, defaults, NaN / inf / zero, flags        -> output register

(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF" *)
module ppc_fpu_sfma
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW = 6,
  parameter bit Q1 = 1'b1,
  parameter bit Q2 = 1'b1,
  parameter bit Q3 = 1'b1,
  parameter bit Q4 = 1'b1,
  parameter bit Q5 = 1'b1
) (
  input  logic            clk,
  input  logic            rst,

  input  logic            in_valid,
  input  fma_op_t         in_op,      // FMA_ADD ... FMA_NMSUB, single form
  input  logic [7:0]      in_ctl,     // FPSCR[24:31]: VE OE UE ZE XE NI RN
  input  logic [63:0]     in_a,
  input  logic [63:0]     in_b,
  input  logic [63:0]     in_c,
  input  logic [TAGW-1:0] in_tag,

  // the finished result, combinational from R5: into the output register
  output logic            o_valid,
  output logic [TAGW-1:0] o_tag,
  output logic [63:0]     o_res,
  output fpu_flags_t      o_flags
);

  localparam int EW = 13;
  typedef logic signed [EW-1:0] exp_t;

  // travels with an op to the end
  typedef struct packed {
    logic [TAGW-1:0] tag;
    logic            negr, special, sp_isnan;
    logic [1:0]      rmode;
    logic            ve, oe, ue, ni;
    logic            invalid;
    logic            vxsnan, vximz, vxisi;
  } side_t;

  // ================================================================== S1
  logic has_b, has_c, subb, negr;
  always_comb begin
    has_b = 1'b1; has_c = 1'b1; subb = 1'b0; negr = 1'b0;
    case (in_op)
      FMA_ADD:   has_c = 1'b0;
      FMA_SUB:   begin has_c = 1'b0; subb = 1'b1; end
      FMA_MUL:   has_b = 1'b0;
      FMA_MSUB:  subb = 1'b1;
      FMA_NMADD: negr = 1'b1;
      FMA_NMSUB: begin negr = 1'b1; subb = 1'b1; end
      default:   ;
    endcase
  end

  // An operand is a zero (exponent 0; the route lets no double denormal
  // through), an infinity or NaN (exponent all ones), or a normalised value
  // with a 24-bit significand. fadds multiply frA by 1.0.
  wire [10:0] a_e = in_a[62:52], b_e = in_b[62:52], c_e = in_c[62:52];
  wire a_zero = (a_e == 11'd0), b_zero = (b_e == 11'd0), c_zr = (c_e == 11'd0);
  wire a_e1 = (a_e == 11'h7FF), b_e1 = (b_e == 11'h7FF), c_e1 = (c_e == 11'h7FF);
  wire a_fz = (in_a[51:29] == 23'd0), b_fz = (in_b[51:29] == 23'd0), c_fz = (in_c[51:29] == 23'd0);
  wire a_inf = a_e1 & a_fz, b_inf = b_e1 & b_fz;
  wire a_nan = a_e1 & ~a_fz, b_nan = b_e1 & ~b_fz;
  wire a_snan = a_nan & ~in_a[51], b_snan = b_nan & ~in_b[51];
  wire c_zero = has_c & c_zr, c_inf = has_c & c_e1 & c_fz, c_nan = has_c & c_e1 & ~c_fz;
  wire c_snan = c_nan & ~in_c[51];
  wire a_s = in_a[63], b_s = in_b[63], c_s = has_c & in_c[63];

  // special operands, as ppc_fpu_fma's S1b [PEM 3.3.1.7, D-4]
  wire pb_nan  = has_b & b_nan;
  wire nan_any = a_nan | pb_nan | c_nan;
  wire vxsnan  = a_snan | (has_b & b_snan) | c_snan;
  wire sp_     = a_s ^ c_s;
  wire sbe     = b_s ^ subb;
  wire imz     = has_c & ~a_nan & ~c_nan & ((a_inf & c_zero) | (a_zero & c_inf));
  wire pinf    = (a_inf | c_inf) & ~a_nan & ~c_nan & ~imz;
  wire isi     = has_b & pinf & b_inf & (sp_ != sbe);
  wire binf    = has_b & b_inf;
  // special = nan_any | imz | isi | pinf | binf, which is exactly "an operand
  // the op reads has an all-ones exponent" (each infinity case needs one, and
  // any infinity read is one of them): three levels instead of six
  wire special = a_e1 | (has_c & c_e1) | (has_b & b_e1);
  wire sp_isnan = nan_any | imz | isi;

  logic sp_sign;
  always_comb begin
    if (a_nan)           sp_sign = a_s;
    else if (pb_nan)     sp_sign = b_s;
    else if (c_nan)      sp_sign = c_s;
    else if (imz | isi)  sp_sign = 1'b0;
    else if (pinf)       sp_sign = sp_;
    else                 sp_sign = sbe;
  end

  wire prod_on   = ~special;
  wire prod_live = prod_on & ~a_zero & ~c_zero;
  wire add_on    = special | (has_b & ~b_zero);
  wire r_sp      = special ? sp_sign : sp_;
  wire r_sb      = special ? sp_sign : (has_b ? sbe : sp_);
  wire effsub    = ~special & (r_sp ^ r_sb);

  // unbiased exponents; sh puts the addend's hidden bit at V[74 - sh]
  exp_t ea, ec, eb, ep0, sh_raw, e0;
  assign ea     = exp_t'({2'b00, a_e}) - exp_t'(1023);
  assign ec     = has_c ? (exp_t'({2'b00, c_e}) - exp_t'(1023)) : exp_t'(0);
  assign eb     = exp_t'({2'b00, b_e}) - exp_t'(1023);
  assign ep0    = ea + ec - exp_t'(46);
  assign sh_raw = ea + ec - eb + exp_t'(27);
  logic [6:0] sh;
  always_comb begin
    if (special || !prod_live) begin
      e0 = eb - exp_t'(73);
      sh = 7'd0;
    end else if (!add_on || sh_raw < 0) begin
      e0 = add_on ? (eb - exp_t'(73)) : ep0;
      sh = add_on ? 7'd0 : 7'd127;
    end else begin
      e0 = ep0;
      sh = (sh_raw > exp_t'(127)) ? 7'd127 : sh_raw[6:0];
    end
  end

  // The addend: a NaN operand's payload (quieted), or frB. The other special
  // results (infinity, the default QNaN) are put in place in S2, beside the
  // alignment shift: special is the deepest term here (0.02 ns over).
  wire [22:0] nan_frac = a_nan ? in_a[51:29] : pb_nan ? in_b[51:29] : in_c[51:29];
  wire [23:0] b_use = nan_any ? {1'b0, nan_frac | (23'd1 << 22)}
                              : ({24{has_b & ~b_zero}} & {1'b1, in_b[51:29]});
  wire        spz   = special & ~nan_any;   // payload 0, or the default QNaN's
  wire [23:0] ma = {~a_zero, in_a[51:29]};
  wire [23:0] mc = has_c ? {~c_zr, in_c[51:29]} : 24'h80_0000;

  side_t s1_sd;
  always_comb begin
    s1_sd.tag      = in_tag;
    s1_sd.negr     = negr;
    s1_sd.special  = special;
    s1_sd.sp_isnan = sp_isnan;
    s1_sd.rmode    = in_ctl[1:0];
    s1_sd.ve       = in_ctl[7];
    s1_sd.oe       = in_ctl[6];
    s1_sd.ue       = in_ctl[5];
    s1_sd.ni       = in_ctl[2];
    s1_sd.invalid  = vxsnan | imz | isi;
    s1_sd.vxsnan   = vxsnan;
    s1_sd.vximz    = imz;
    s1_sd.vxisi    = isi;
  end

  typedef struct packed {
    logic [23:0] ma, mc, b;
    logic [6:0]  sh;
    exp_t        e0;
    logic        spz;
    logic        sp, sb, effsub, pon;
    side_t       sd;
  } r1_t;
  // R1 and R4 are kept where they are (preserve): Quartus merged R1 with
  // the double unit's identical operand register (a route across both units,
  // +0.06 ns), and retimed R4 into S5's normalise shift (-0.19 ns)
  r1_t  s1_d;
  (* preserve *) r1_t r1_q;
  logic r1_v;
  always_comb begin
    s1_d.ma = ma;  s1_d.mc = mc;  s1_d.b = b_use;  s1_d.sh = sh;  s1_d.e0 = e0;  s1_d.spz = spz;
    s1_d.sp = r_sp;  s1_d.sb = r_sb;  s1_d.effsub = effsub;  s1_d.pon = prod_on;  s1_d.sd = s1_sd;
  end
  generate if (Q1) begin : g_r1
    always_ff @(posedge clk) begin
      if (rst) r1_v <= 1'b0;
      else     r1_v <= in_valid;
      r1_q <= s1_d;
    end
  end else begin : g_c1
    assign r1_v = in_valid;
    assign r1_q = s1_d;
  end endgenerate

  // ================================================================== S2
  wire [47:0] prod2 = r1_q.ma * r1_q.mc;

  // the addend at V[74:51], shifted right; bit j ends at V[51 + j - sh] and is
  // sticky when that is 0 or below (a compare per bit, not a second shifter)
  // a special result other than a NaN operand's: infinity (payload 0) or the
  // default QNaN (its quiet bit), unshifted (sh = 0)
  wire [23:0] b2  = r1_q.spz ? {1'b0, r1_q.sd.sp_isnan, 22'd0} : r1_q.b;
  wire [73:0] al2 = {b2, 50'd0} >> r1_q.sh;
  logic [23:0] lost2;
  always_comb
    for (int j = 0; j < 24; j++) lost2[j] = (8'(j) + 8'd51 <= {1'b0, r1_q.sh});
  wire st2 = |(b2 & lost2);

  typedef struct packed {
    logic [47:0] p;
    logic [74:0] al;
    exp_t        e0;
    logic        sp, sb, effsub, pon;
    side_t       sd;
  } r2_t;
  r2_t  s2_d, r2_q;
  logic r2_v;
  always_comb begin
    s2_d.p = prod2;  s2_d.al = {al2, st2};  s2_d.e0 = r1_q.e0;
    s2_d.sp = r1_q.sp;  s2_d.sb = r1_q.sb;  s2_d.effsub = r1_q.effsub;  s2_d.pon = r1_q.pon;  s2_d.sd = r1_q.sd;
  end
  generate if (Q2) begin : g_r2
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
  // T1 = P +- addend, T2 = addend - P (used when T1 < 0); 76 bits, bit 75 the
  // sign. Both are registered and the select made in S4: behind the 76-bit
  // carry it missed by 0.11 ns.
  wire [75:0] pw  = {27'd0, (r2_q.pon ? r2_q.p : 48'd0), 1'b0};
  wire [75:0] aw  = {1'b0, r2_q.al};
  wire [75:0] ax  = r2_q.effsub ? ~aw : aw;
  wire [75:0] t1  = pw + ax + {75'd0, r2_q.effsub};
  wire [75:0] t2  = aw + ~pw + 76'd1;
  wire        neg3 = r2_q.effsub & t1[75];

  // a tiny result rounds at 2^-149 = V[-148 - E0]: its guard's place, for S4
  exp_t pd3;
  assign pd3 = exp_t'(-149) - r2_q.e0;

  typedef struct packed {
    logic [74:0] t1, t2;
    logic        neg;
    exp_t        e0, pd;
    logic        sign, effsub;
    side_t       sd;
  } r3_t;
  r3_t  s3_d, r3_q;
  logic r3_v;
  always_comb begin
    s3_d.t1 = t1[74:0];  s3_d.t2 = t2[74:0];  s3_d.neg = neg3;  s3_d.e0 = r2_q.e0;  s3_d.pd = pd3;
    s3_d.sign = neg3 ? r2_q.sb : r2_q.sp;  s3_d.effsub = r2_q.effsub;  s3_d.sd = r2_q.sd;
  end
  generate if (Q3) begin : g_r3
    always_ff @(posedge clk) begin
      if (rst) r3_v <= 1'b0;
      else     r3_v <= r2_v;
      r3_q <= s3_d;
    end
  end else begin : g_c3
    assign r3_v = r2_v;
    assign r3_q = s3_d;
  end endgenerate

  // ================================================================== S4
  wire [74:0] s4 = r3_q.neg ? r3_q.t2 : r3_q.t1;
  // A special result is not shifted: a stopper at V[74] stops the count at 0.
  wire force4 = r3_q.sd.special;
  logic [6:0] lz4, tz4;
  ppc_fpu_lzc #(.W(75)) u_lz (.clk (clk), .x (s4 | {force4, 74'd0}), .n (lz4));
  logic [74:0] s_rev;
  always_comb for (int i = 0; i < 75; i++) s_rev[i] = s4[74 - i];
  ppc_fpu_lzc #(.W(75)) u_tz (.clk (clk), .x (s_rev), .n (tz4));
  wire zero4 = (s4 == 75'd0);

  // a tiny result's guard V[pd] and LSB V[pd + 1], from the unshifted sum
  wire pd_in = (r3_q.pd >= 0) && (r3_q.pd <= exp_t'(74));
  wire pl_in = (r3_q.pd >= exp_t'(-1)) && (r3_q.pd <= exp_t'(73));
  wire [6:0] pdi = r3_q.pd[6:0];
  wire [6:0] pli = pdi + 7'd1;
  wire g_den4 = pd_in & s4[pdi];
  wire l_den4 = pl_in & s4[pli];

  // E = E0 + 73 - lz: the compares S5 needs, as constants against lz
  //   E < -126: lz > E0 + 199      E == -127: lz == E0 + 200
  //   E > 127:  lz < E0 - 54       E == 127:  lz == E0 - 54
  //   E < -151: lz > E0 + 224 (wholly below the guard bit at 2^-150)
  //   E + 1023 (+ 192, - 192) = E0 + 1096 (+ 192, - 192) - lz
  // E0 + 199 is clamped to -26 ... 75 (lz is 0 ... 75), so S5's tests and the
  // rounding place d = lz - (E0 + 199) come from one 8-bit subtract (two
  // 13-bit compares and a 13-bit subtract in front of the masks missed by
  // 0.5 ns on one seed)
  exp_t k199;
  assign k199 = r3_q.e0 + exp_t'(199);
  typedef struct packed {
    logic [74:0] s;
    logic [6:0]  lz, tz;
    logic        zero, g_den, l_den;
    exp_t        pd, k200, k54, kb;
    logic [7:0]  kc;
    logic        sign, effsub;
    side_t       sd;
  } r4_t;
  r4_t  s4_d;
  (* preserve *) r4_t r4_q;
  logic r4_v;
  always_comb begin
    s4_d.s = s4;  s4_d.lz = lz4;  s4_d.tz = tz4;  s4_d.zero = zero4;
    s4_d.g_den = g_den4;  s4_d.l_den = l_den4;  s4_d.pd = r3_q.pd;
    s4_d.k200 = r3_q.e0 + exp_t'(200);
    s4_d.k54  = r3_q.e0 - exp_t'(54);
    s4_d.kc   = (k199 < exp_t'(-26)) ? 8'hE6 : (k199 > exp_t'(75)) ? 8'd75 : k199[7:0];
    s4_d.kb   = r3_q.e0 + exp_t'(1096);
    s4_d.sign = r3_q.sign;  s4_d.effsub = r3_q.effsub;  s4_d.sd = r3_q.sd;
  end
  generate if (Q4) begin : g_r4
    always_ff @(posedge clk) begin
      if (rst) r4_v <= 1'b0;
      else     r4_v <= r3_v;
      r4_q <= s4_d;
    end
  end else begin : g_c4
    assign r4_v = r3_v;
    assign r4_q = s4_d;
  end endgenerate

  // ================================================================== S5
  // Normalise: only the top 27 bits are kept, n[74:48].
  wire [74:0] n5 = r4_q.s << r4_q.lz;

  exp_t lz5;
  assign lz5 = exp_t'({6'd0, r4_q.lz});
  wire force5 = r4_q.sd.special;
  exp_t dd5;                                    // -126 - E
  assign dd5 = lz5 - exp_t'({{5{r4_q.kc[7]}}, r4_q.kc});
  wire lt126  = dd5 > 0;                        // E < -126
  wire lt151  = dd5 > exp_t'(25);               // E < -151
  wire tiny5  = ~force5 & ~r4_q.zero & lt126;
  wire sden5  = ~r4_q.sd.ue & tiny5;          // denormalised: rounds at 2^-149
  wire far5   = sden5 & lt151;
  wire [4:0] d5 = !sden5 ? 5'd0 : far5 ? 5'd25 : dd5[4:0];

  // the rounding place in z = {00, n[74:48]}: LSB at bit 3 + d
  wire [28:0] z5 = {2'b00, n5[74:48]};
  logic [28:0] keep5, one5;
  always_comb
    for (int i = 0; i < 29; i++) begin
      keep5[i] = (6'(i) >= 6'd3 + {1'b0, d5});
      one5[i]  = (6'(i) == 6'd3 + {1'b0, d5});
    end

  // guard, sticky, LSB: a normalised result's at fixed places of n, a
  // denormalised one's from S4 (sticky: a one below V[pd], tz < pd)
  wire [7:0] tzlz = {1'b0, r4_q.tz} + {1'b0, r4_q.lz};
  wire st_n  = ~r4_q.zero & (tzlz < 8'd48);
  wire x_den = ~r4_q.zero & (exp_t'({6'd0, r4_q.tz}) < r4_q.pd);
  wire lsb5  = sden5 ? r4_q.l_den : n5[51];
  wire g5    = sden5 ? r4_q.g_den : n5[50];
  wire x5    = sden5 ? x_den : (n5[49] | n5[48] | st_n);

  // the delivered biased exponent's candidates (FMA's bnu0/1/2, bo0/1):
  // E + 1023 (+ 192 when tiny with UE=1), plus 0, 1 or 2; overflowing with
  // OE=1, E + 1023 - 192 plus 0 or 1. Wholly below the guard, E reads -151.
  // FPRF's single denormal class (FMA: delivered exponent below 897) needs no
  // compare here: with single operands an exponent-adjusted result is never
  // below 897 (a tiny product is at least 2^-298, +192 gives 917; an addend
  // that cancels it is at least 2^-149), so it is exactly a denormalised result
  // that did not round up to 2^-126.
  typedef struct packed {
    logic [10:0] v;
  } bexp_t;
  function automatic bexp_t bx_of(input exp_t v);
    bx_of.v = v[10:0];
  endfunction
  exp_t bn5, bu5, bo5, base5;
  assign bn5   = far5 ? exp_t'(872) : (r4_q.kb - lz5);
  assign bu5   = r4_q.kb + exp_t'(192) - lz5;
  assign bo5   = r4_q.kb - exp_t'(192) - lz5;
  assign base5 = (tiny5 & r4_q.sd.ue) ? bu5 : bn5;

  typedef struct packed {
    logic [28:0] z, keep, one;
    logic        lsb, g, x, rtz;
    logic        tiny, zero, ovf0, ovf_at, den_at;
    bexp_t       bnu0, bnu1, bnu2, bo0, bo1;
    logic        sign, effsub;
    side_t       sd;
  } r5_t;
  r5_t  s5_d, r5_q;
  logic r5_v;
  always_comb begin
    s5_d.z = z5;  s5_d.keep = keep5;  s5_d.one = one5;
    s5_d.lsb = lsb5;  s5_d.g = g5;  s5_d.x = x5 | far5;
    s5_d.rtz = (d5 >= 5'd24);                 // nothing kept above the rounding place
    s5_d.tiny = tiny5;  s5_d.zero = r4_q.zero;
    s5_d.ovf0   = ~far5 & (lz5 < r4_q.k54);
    s5_d.ovf_at = ~far5 & (lz5 == r4_q.k54);
    s5_d.den_at = ~far5 & (lz5 == r4_q.k200);
    s5_d.bnu0 = bx_of(base5);
    s5_d.bnu1 = bx_of(base5 + exp_t'(1));
    s5_d.bnu2 = bx_of(base5 + exp_t'(2));
    s5_d.bo0  = bx_of(bo5);
    s5_d.bo1  = bx_of(bo5 + exp_t'(1));
    s5_d.sign = r4_q.sign;  s5_d.effsub = r4_q.effsub;  s5_d.sd = r4_q.sd;
  end
  generate if (Q5) begin : g_r5
    always_ff @(posedge clk) begin
      if (rst) r5_v <= 1'b0;
      else     r5_v <= r4_v;
      r5_q <= s5_d;
    end
  end else begin : g_c5
    assign r5_v = r4_v;
    assign r5_q = s5_d;
  end endgenerate

  // ================================================================== S6
  side_t sd6;
  assign sd6 = r5_q.sd;
  wire spc6 = sd6.special;
  wire g6 = r5_q.g, x6 = r5_q.x, lsb6 = r5_q.lsb;
  logic inc6;
  always_comb begin
    case (sd6.rmode)
      2'b00:   inc6 = g6 & (x6 | lsb6);
      2'b01:   inc6 = 1'b0;
      2'b10:   inc6 = ~r5_q.sign & (g6 | x6);
      default: inc6 = r5_q.sign & (g6 | x6);
    endcase
    if (spc6) inc6 = 1'b0;
  end
  wire inexact6 = ~spc6 & (g6 | x6);
  wire rzero6   = ~inc6 & r5_q.rtz;

  // the kept bits masked at the adder's inputs (behind the normalise shift
  // in S5 the mask missed by 0.39 ns)
  wire [28:0] rr6 = (r5_q.z & r5_q.keep) + (inc6 ? r5_q.one : 29'd0);
  wire        c2  = rr6[28], c1 = rr6[27];
  wire [22:0] frac6 = rr6[25:3];

  // Table 3-15 defaults for a disabled overflow
  wire to_inf = (sd6.rmode == 2'b00) || (sd6.rmode == 2'b10 && !r5_q.sign) ||
                (sd6.rmode == 2'b11 && r5_q.sign);
  wire [62:0] ovf_mag = to_inf ? {11'h7FF, 52'd0} : 63'h47EF_FFFF_E000_0000;
  wire zsign = r5_q.effsub ? (sd6.rmode == 2'b11) : r5_q.sign;
  wire flip  = sd6.negr & ~(spc6 & sd6.sp_isnan);

  // The two outcomes of the rounding carry c1, side by side; c2 (a single
  // rounded up from wholly below 2^-150) only ever with c1 = 0. The add is
  // then only in front of the fraction and the final select.
  logic [63:0] res_k [2];
  logic [4:0]  cls_k [2];
  logic        ovf_k [2], ovfd_k [2];
  genvar k;
  generate for (k = 0; k < 2; k++) begin : g_fin
    wire   ovf     = ~spc6 & ~r5_q.zero & ~r5_q.tiny & (r5_q.ovf0 | (r5_q.ovf_at & (k == 1)));
    wire   ovf_dis = ovf & ~sd6.oe;
    bexp_t bsel;
    assign bsel    = (ovf & sd6.oe) ? ((k == 1) ? r5_q.bo1 : r5_q.bo0)
                                    : (c2 ? r5_q.bnu2 : (k == 1) ? r5_q.bnu1 : r5_q.bnu0);
    wire   res_den = r5_q.tiny & ~sd6.ue & ~rzero6 & ~(r5_q.den_at & (k == 1));
    wire   flush   = sd6.ni & res_den;
    wire   den_c   = r5_q.tiny & ~sd6.ue & ~(r5_q.den_at & (k == 1)) & ~(ovf & sd6.oe);
    logic [63:0] r;
    logic [4:0]  c;
    always_comb begin
      if (spc6)                 begin r = {r5_q.sign, 11'h7FF, frac6, 29'd0};
                                      c = sd6.sp_isnan ? FPRF_QNAN : FPRF_PINF; end
      else if (r5_q.zero)       begin r = {zsign, 63'd0};  c = FPRF_PZERO; end
      else if (ovf_dis)         begin r = {r5_q.sign, ovf_mag};  c = to_inf ? FPRF_PINF : FPRF_PNORM; end
      else if (rzero6 || flush) begin r = {r5_q.sign, 63'd0};  c = FPRF_PZERO; end
      else                      begin r = {r5_q.sign, bsel.v, frac6, 29'd0};
                                      c = den_c ? FPRF_PDEN : FPRF_PNORM; end
      if (flip) r[63] = ~r[63];
    end
    assign res_k[k]  = r;
    assign cls_k[k]  = c;
    assign ovf_k[k]  = ovf;
    assign ovfd_k[k] = ovf_dis;
  end endgenerate

  wire [63:0] res6 = c1 ? res_k[1] : res_k[0];
  wire [4:0]  cls6 = c1 ? cls_k[1] : cls_k[0];
  wire        ovf6 = c1 ? ovf_k[1] : ovf_k[0];
  wire        ovd6 = c1 ? ovfd_k[1] : ovfd_k[0];
  wire [4:0]  fprf6 = (cls6 == FPRF_QNAN || !res6[63]) ? cls6 :
                      (cls6 == FPRF_PINF)  ? FPRF_NINF  :
                      (cls6 == FPRF_PZERO) ? FPRF_NZERO :
                      (cls6 == FPRF_PDEN)  ? FPRF_NDEN  : FPRF_NNORM;

  wire wr6 = ~(sd6.invalid & sd6.ve);
  logic fr6, fi6;
  always_comb begin
    fr6 = inc6 & ~ovd6;
    fi6 = inexact6 | ovd6;
    if (sd6.invalid) begin fr6 = 1'b0; fi6 = 1'b0; end
  end

  assign o_valid = r5_v;
  assign o_tag   = sd6.tag;
  assign o_res   = res6;
  always_comb begin
    o_flags            = '0;
    o_flags.wr         = wr6;
    o_flags.frfi_we    = 1'b1;
    o_flags.fr         = fr6;
    o_flags.fi         = fi6;
    o_flags.fprf_we    = wr6;
    o_flags.fpcc_we    = 1'b0;
    o_flags.fprf       = fprf6;
    o_flags.exc.vxsnan = sd6.vxsnan;
    o_flags.exc.vximz  = sd6.vximz;
    o_flags.exc.vxisi  = sd6.vxisi;
    o_flags.exc.ox     = ovf6;
    o_flags.exc.ux     = r5_q.tiny & (sd6.ue | inexact6);
    o_flags.exc.xx     = fi6;
  end

endmodule
