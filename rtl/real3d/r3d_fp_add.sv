// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 1 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// IEEE-754 single precision adder / subtractor.
//
// 4-stage pipeline, LATENCY 4. Retimed from 2 stages to meet the M0 Fmax gate.
//
//   A  unpack, order by magnitude, 27-bit align shift
//   B  add or subtract
//   C  leading-zero count and normalise shift
//   D  round and pack
//
// Two separate paths forced this, both measured with make quartus_paths:
// inside the core the ALU operand mux fed stage 1's compare-align-subtract
// chain (~54 MHz), and standalone the old stage 2 ran lzc -> normalise shift ->
// 24-bit round carry chain (~78.7 MHz). Splitting either alone leaves the other
// as the ceiling, so both are cut.
//
// fp_mul carries the same latency so mb86233_alu keeps a single alignment
// depth for its non-FP results.
//
// This is the area-dominant FP block: two barrel shifters and an LZC, all soft
// logic. If the M0 resource gate fails, this module is where to look first.
// The align shifter can be narrowed by capping the shift at 26 and folding
// everything beyond into sticky, which is already done below.
//
// Serves fadd, fsbd, fcpd, fsmd, fmsd, fmrd, and the B+A / B-A forms. The
// caller selects operands and the effective subtract; this module does not
// know about opcodes.

// Copied from my Model 2 core (rtl/tgp/fp_add.sv @47edd9dc00) as r3d_fp_add, timescale removed.

module r3d_fp_add #(
  parameter bit FLUSH_DENORM_IN = 1'b0,
  // R740: 1 = a register between the compare and the align shift -- LATENCY
  // 5, not 4. For m2_fp_pool only (80 MHz: add_b_q -> sA_small -0.462 even
  // after R739); mb86233_alu's alignment depends on 4 and keeps 0.
  parameter bit PIPE_A2 = 1'b0,
  // R852: SUBNORMAL RESULTS, as MAME's host floats give them. 0 is the unit as
  // it was (a sum below 2^-126 flushes to zero). 1 stops the normalising
  // shift at exponent 1 and packs what is left as a subnormal -- exact, since
  // a sum of numbers on the subnormal grid is on it too. Model 2A's TGP
  // (Virtua Fighter 2 accumulates subnormal products).
  parameter bit SUBNORM = 1'b0,
  // DSP_SHIFT (Model 3 study R98): both shifts as multiplies by a power of two,
  // one 27 x 27 DSP each, in place of the soft barrel shifters. Align: p_sh x
  // 2^(26 - shamt), the quotient bits [52:26] are p_sh >> shamt and the low
  // 26 bits are the bits shifted out (sticky). Normalise (SUBNORM = 0 only):
  // s1_sum[26:0] x 2^(lzc - 1), the power formed as the lowest set bit of
  // s1_sum bit-reversed. Same results; 0 keeps the shifters.
  parameter bit DSP_SHIFT = 1'b0,
  // NORM_B (Model 3 study R109; SUBNORM = 0 and DSP_SHIFT = 0 only): the normalise shift split across
  // B and C. B shifts the sum left by whole bytes (0, 8, 16, 24: as many as its leading zero bits allow)
  // and takes the bytes off the exponent; C counts the last zero bits (0 to 7) in the top byte and
  // shifts by those. Same results, same latency; C was the leading-zero count and a 28-bit shift by a
  // 5-bit amount in one stage.
  parameter bit NORM_B = 1'b0,
  // PIPE_B2 (Model 3 study R148, D-50): a register between stage B's add and its byte normalise -- LATENCY one
  // more. B's add (28 bits, with the sticky borrow) and the byte shift's zero tests behind it were the adder's
  // slowest stage at 120 MHz (sA_sticky -> s1_sum). Same results.
  parameter bit PIPE_B2 = 1'b0
) (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic        sub,          // compute a - b

  output logic        out_valid,
  output logic [31:0] result,
  output logic        overflow,
  output logic        underflow,
  output logic        invalid       // inf - inf, or NaN operand
);

  // ------------------------------------------------------------- unpack

  logic [31:0] b_eff;
  assign b_eff = {b[31] ^ sub, b[30:0]};

  logic        a_sign, b_sign;
  logic [7:0]  a_exp,  b_exp;
  logic [22:0] a_frac, b_frac;

  assign a_sign = a[31];      assign a_exp = a[30:23];      assign a_frac = a[22:0];
  assign b_sign = b_eff[31];  assign b_exp = b_eff[30:23];  assign b_frac = b_eff[22:0];

  logic a_is_zero, b_is_zero, a_is_inf, b_is_inf, a_is_nan, b_is_nan;
  assign a_is_zero = (a_exp == 8'h00) && (FLUSH_DENORM_IN ? 1'b1 : (a_frac == 23'd0));
  assign b_is_zero = (b_exp == 8'h00) && (FLUSH_DENORM_IN ? 1'b1 : (b_frac == 23'd0));
  assign a_is_inf  = (a_exp == 8'hff) && (a_frac == 23'd0);
  assign b_is_inf  = (b_exp == 8'hff) && (b_frac == 23'd0);
  assign a_is_nan  = (a_exp == 8'hff) && (a_frac != 23'd0);
  assign b_is_nan  = (b_exp == 8'hff) && (b_frac != 23'd0);

  logic [23:0] a_sig, b_sig;
  assign a_sig = {(a_exp != 8'h00), a_frac};
  assign b_sig = {(b_exp != 8'h00), b_frac};

  logic [7:0] a_e_eff, b_e_eff;
  assign a_e_eff = (a_exp == 8'h00) ? 8'd1 : a_exp;
  assign b_e_eff = (b_exp == 8'h00) ? 8'd1 : b_exp;

  // -------------------------------------------------- order by magnitude

  // R739: THE SHIFT DOES NOT WAIT FOR THE SIGNIFICAND COMPARE. a_ge -- an
  // exponent compare AND a 24-bit significand compare -- selected the operand
  // to shift and the subtraction that made the shift amount, so both sat in
  // front of the 27-bit barrel shifter (s785 at 80 MHz: m2_fp_pool add_a_q ->
  // sA_small -0.499, the TGP ALU's opr_add_a -> sA_small -0.514). But the
  // amount depends on the exponents alone, and when they are equal it is 0:
  // so the shifter is fed by the exponent compare (e_gt), both differences
  // are formed in parallel, the significand compare runs beside the shifter,
  // and the one case it decides -- equal exponents, a >= b -- swaps the
  // unshifted operands with a 2:1 mux after it. Every value below is the
  // same as before, bit for bit.
  logic e_gt, e_eq, sig_ge, a_ge;
  assign e_gt   = (a_e_eff >  b_e_eff);
  assign e_eq   = (a_e_eff == b_e_eff);
  assign sig_ge = (a_sig >= b_sig);
  assign a_ge   = e_gt || (e_eq && sig_ge);

  logic [23:0] big_sig;
  logic [7:0]  big_exp;
  logic        big_sign, small_sign;

  assign big_sig    = a_ge ? a_sig   : b_sig;
  assign big_exp    = a_ge ? a_e_eff : b_e_eff;
  assign big_sign   = a_ge ? a_sign  : b_sign;
  assign small_sign = a_ge ? b_sign  : a_sign;

  logic [8:0] exp_diff_full;
  logic [4:0] shamt;
  logic       shift_saturated;

  // R739: e_gt, not a_ge -- with equal exponents both differences are 0
  logic [8:0] d_ab, d_ba;
  assign d_ab            = {1'b0, a_e_eff} - {1'b0, b_e_eff};
  assign d_ba            = {1'b0, b_e_eff} - {1'b0, a_e_eff};
  assign exp_diff_full   = e_gt ? d_ab : d_ba;
  assign shift_saturated = (exp_diff_full > 9'd26);
  assign shamt           = shift_saturated ? 5'd26 : exp_diff_full[4:0];

  // 3 extra low bits: guard, round, sticky.
  logic [26:0] small_ext, small_aligned;
  logic        sticky_lost;

  // R739: the shifted operand is the one with the smaller exponent (b when
  // they are equal); with equal exponents shamt is 0, so the significand
  // compare only chooses between the two unshifted operands, after the shift.
  logic [23:0] sh_sig;
  assign sh_sig        = e_gt ? b_sig : a_sig;
  // R740: everything the shift and the stage-A register need, at the point
  // PIPE_A2 cuts (p_*): passed straight through when it is 0.
  logic        p_valid, p_sign, p_eff_sub, p_swap, p_sat, p_nan, p_inf, p_both_zero, p_zero_sign;
  logic [7:0]  p_exp;
  logic [4:0]  p_shamt;
  logic [26:0] p_big, p_sh, p_b;
  logic [26:0] p_pow;                // DSP_SHIFT: the align multiplier, decoded before the cut
  wire  [26:0] pow_n = 27'd1 << (5'd26 - shamt);
  assign small_ext     = p_sh;
  logic [53:0] al_prod;
  generate if (DSP_SHIFT) begin : g_al_dsp
    assign al_prod = {27'd0, p_sh} * {27'd0, p_pow};     // p_pow = 2^(26 - shamt), shamt <= 26
    assign small_aligned = p_swap ? p_b : al_prod[52:26];
  end else begin : g_al_sh
    assign al_prod = '0;
    assign small_aligned = p_swap ? p_b : (p_sh >> p_shamt);
  end endgenerate
  // R512: THE STICKY MASK IS A THERMOMETER, NOT A SHIFT AND A SUBTRACT.
  //
  //   m2_geo_xform|sum_bank -> m2_fp_pool|fp_add|sA_sticky    -0.515 on clk_sys
  //
  // was the worst clk_sys path once clk_mem was repaired (R498) and the boot
  // ROM scan was off it (R507), and sA_sticky is the endpoint.
  // `(27'd1 << shamt) - 27'd1` builds the mask of shifted-out bits with a
  // SECOND 27-bit barrel shifter and a 27-bit borrow chain, running beside the
  // real one that produces small_aligned, with an AND and an OR-reduce on top
  // -- all in the cycle the pool's operand mux delivers.
  //
  // The mask is just "bit i is set if i < shamt". Twenty-seven 5-bit compares
  // against the same value, every one independent, replace the shifter and the
  // borrow chain with a single level of logic. Bit-for-bit identical: for
  // shamt in 0..26 the low `shamt` bits are set and the rest clear, which is
  // what the subtract produced, and the saturated arm is untouched.
  //
  // Spike: Fmax 93.23 -> 98.09, 0.54 ns, +15 ALM. tb_fp_add 1,968,564 checks.
  logic [26:0] sticky_mask;
  always_comb
    for (int i = 0; i < 27; i++) sticky_mask[i] = (5'(i) < p_shamt);

  assign sticky_lost   = p_sat ? (|small_ext)
                               : DSP_SHIFT ? (|al_prod[25:0]) : (|(small_ext & sticky_mask));

  logic [26:0] big_ext;
  assign big_ext = {big_sig, 3'b000};

  logic eff_sub;
  assign eff_sub = big_sign ^ small_sign;

  // R740: the cut.
  wire pre_nan  = (a_is_nan | b_is_nan) | (a_is_inf & b_is_inf & (a_sign ^ b_sign));
  wire pre_inf  = (a_is_inf | b_is_inf) & ~(a_is_inf & b_is_inf & (a_sign ^ b_sign));
  generate if (PIPE_A2) begin : g_a2
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) p_valid <= 1'b0;
      else begin
        p_valid <= in_valid;
        p_sign <= big_sign;  p_exp <= big_exp;  p_big <= big_ext;  p_eff_sub <= eff_sub;
        p_sh <= {sh_sig, 3'b000};  p_b <= {b_sig, 3'b000};  p_swap <= e_eq && sig_ge;
        p_shamt <= shamt;  p_sat <= shift_saturated;  p_pow <= pow_n;
        p_nan <= pre_nan;  p_inf <= pre_inf;
        p_both_zero <= a_is_zero & b_is_zero;  p_zero_sign <= a_sign & b_sign;
      end
    end
  end else begin : g_a1
    assign p_valid = in_valid;
    assign p_sign = big_sign;  assign p_exp = big_exp;  assign p_big = big_ext;  assign p_eff_sub = eff_sub;
    assign p_sh = {sh_sig, 3'b000};  assign p_b = {b_sig, 3'b000};  assign p_swap = e_eq && sig_ge;
    assign p_shamt = shamt;  assign p_sat = shift_saturated;  assign p_pow = pow_n;
    assign p_nan = pre_nan;  assign p_inf = pre_inf;
    assign p_both_zero = a_is_zero & b_is_zero;  assign p_zero_sign = a_sign & b_sign;
  end endgenerate

  // ------------------------------------------------- stage A: align only
  //
  // The 27-bit barrel shifter ends here. Everything downstream of it — the
  // add, the normalise, the round — is in a later stage, which is the point
  // of the retime.

  logic        sA_valid, sA_sign, sA_nan, sA_inf, sA_invalid, sA_both_zero;
  logic        sA_zero_sign, sA_eff_sub, sA_sticky;
  logic [7:0]  sA_exp;
  logic [26:0] sA_big, sA_small;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sA_valid <= 1'b0;
    end else begin
      sA_valid   <= p_valid;   // R740: p_* (the inputs themselves when PIPE_A2 is 0)
      sA_sign    <= p_sign;
      sA_exp     <= p_exp;
      sA_big     <= p_big;
      sA_small   <= small_aligned;
      sA_sticky  <= sticky_lost;
      sA_eff_sub <= p_eff_sub;
      sA_invalid <= p_nan;
      sA_nan     <= p_nan;
      sA_inf     <= p_inf;
      sA_both_zero <= p_both_zero;
      // -0 + -0 = -0; every other zero pairing gives +0 under round-to-nearest.
      sA_zero_sign <= p_zero_sign;
    end
  end

  // ------------------------------------------------- stage B: add / subtract
  //
  // On effective subtract the bits discarded by the align shifter represent a
  // positive eps that was never subtracted, so big - small_aligned overshoots.
  // Borrow one LSB and let the round stage see sticky=1: the true remainder is
  // (1 - eps) LSBs, which lies strictly between 0 and 1.
  logic [27:0] sum_raw;
  assign sum_raw = sA_eff_sub
                 ? ({1'b0, sA_big} - {1'b0, sA_small} - {27'd0, sA_sticky})
                 : ({1'b0, sA_big} + {1'b0, sA_small});

  // PIPE_B2: the sum and what travels with it, registered (q_*); passed straight through when it is 0
  logic        q_valid, q_sign, q_eff_sub, q_sticky, q_invalid, q_nan, q_inf, q_both_zero, q_zero_sign;
  logic [7:0]  q_exp;
  logic [27:0] q_sum;
  generate if (PIPE_B2) begin : g_b2
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) q_valid <= 1'b0;
      else begin
        q_valid <= sA_valid;  q_sign <= sA_sign;  q_exp <= sA_exp;  q_sum <= sum_raw;  q_eff_sub <= sA_eff_sub;
        q_sticky <= sA_sticky;  q_invalid <= sA_invalid;  q_nan <= sA_nan;  q_inf <= sA_inf;
        q_both_zero <= sA_both_zero;  q_zero_sign <= sA_zero_sign;
      end
    end
  end else begin : g_b1
    assign q_valid = sA_valid;  assign q_sign = sA_sign;  assign q_exp = sA_exp;  assign q_sum = sum_raw;
    assign q_eff_sub = sA_eff_sub;  assign q_sticky = sA_sticky;  assign q_invalid = sA_invalid;
    assign q_nan = sA_nan;  assign q_inf = sA_inf;  assign q_both_zero = sA_both_zero;  assign q_zero_sign = sA_zero_sign;
  end endgenerate

  logic        s1_valid, s1_sign, s1_nan, s1_inf, s1_invalid, s1_both_zero;
  logic        s1_zero_sign;
  logic [7:0]  s1_exp;
  logic [27:0] s1_sum;
  logic        s1_sticky;
  logic        s1_exact_cancel;
  localparam bit NB = NORM_B && !SUBNORM && !DSP_SHIFT;
  // NORM_B: the byte shift (a carry into bit 27 is left to C)
  logic [27:0] sum_b;
  logic [1:0]  nb_c;                       // bytes shifted
  always_comb begin
    nb_c = 2'd0;
    if (!q_sum[27]) begin
      if (q_sum[26:3] == 24'd0) nb_c = 2'd3;
      else if (q_sum[26:11] == 16'd0) nb_c = 2'd2;
      else if (q_sum[26:19] == 8'd0) nb_c = 2'd1;
    end
    sum_b = q_sum << {nb_c, 3'd0};
  end
  // NORM_B with PIPE_B2: C's exponent less its own count, formed here: exp - 8 bytes + 1 (with a carry out the bytes
  // are 0, so it is also the carry's exp + 1). Only when B is split: in B's single stage the subtract after the
  // byte test lengthened it (the facing unit's adder, 108.2 MHz standalone at -40 C)
  localparam bit XB = NB && PIPE_B2;
  logic signed [9:0] s1_expb;
  // B's bytes, for C's exponent (taken off there, beside C's count: in B the select after the sum's zero
  // tests was the stage's slowest path). A zero sum's exponent differs from the unsplit form's, but a
  // zero sum packs as zero before the exponent is looked at.
  logic [1:0]  s1_c;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s1_valid <= 1'b0;
    end else begin
      s1_valid        <= q_valid;
      s1_sign         <= q_sign;
      s1_exp          <= q_exp;
      s1_sum          <= NB ? sum_b : q_sum;
      s1_c            <= nb_c;
      s1_expb         <= $signed({2'b00, q_exp}) - $signed({5'd0, nb_c, 3'd0}) + 10'sd1;
      s1_sticky       <= q_sticky;
      s1_invalid      <= q_invalid;
      s1_nan          <= q_nan;
      s1_inf          <= q_inf;
      // Exact cancellation returns +0 under round-to-nearest.
      s1_exact_cancel <= q_eff_sub && (q_sum == 28'd0) && !q_sticky;
      s1_both_zero    <= q_both_zero;
      s1_zero_sign    <= q_zero_sign;
    end
  end

  // ------------------------------------------------------------ stage 2

  // Effective-add can carry into bit 27. Effective-sub can cancel arbitrarily.
  logic [4:0]  lzc;
  logic [27:0] shifted;
  logic        lost_bit;
  logic signed [9:0] exp_adj;

  // Leading-zero count as an explicit priority encoder. Written without a
  // loop-with-break so it reads identically to yosys, Verilator and Quartus.
  // lzc == 1 means the leading one already sits at bit 26.
  always_comb begin
    lzc = 5'd27;
    for (int i = 0; i < 27; i++)
      if (s1_sum[i]) lzc = 5'(27 - i);
  end
  // NORM_B: C's own count, 1 to 8 (the leading one is in the top byte, or the sum is 0: 1)
  logic [4:0] lzf;
  always_comb begin
    lzf = 5'd1;
    for (int i = 19; i < 27; i++)
      if (s1_sum[i]) lzf = 5'(27 - i);
  end

  // DSP_SHIFT: 2^(lzc - 1) is the lowest set bit of s1_sum[26:0] reversed
  // (0 for a zero sum, whose shift is 0 either way)
  logic [26:0] nm_rev, nm_pow;
  logic [53:0] nm_prod;
  always_comb begin
    for (int i = 0; i < 27; i++) nm_rev[i] = s1_sum[26 - i];
    nm_pow  = nm_rev & (~nm_rev + 27'd1);
    nm_prod = DSP_SHIFT ? {27'd0, s1_sum[26:0]} * {27'd0, nm_pow} : 54'd0;
  end

  always_comb begin
    if (s1_sum[27]) begin
      // Carry out of the significand: shift right one, exponent up one.
      shifted  = s1_sum >> 1;
      lost_bit = s1_sum[0];
      exp_adj  = XB ? s1_expb : $signed({2'b00, s1_exp}) + 10'sd1;
    end else if (SUBNORM && ({2'b00, s1_exp} < {5'd0, lzc})) begin
      // R852: the full shift would take the exponent below 1 -- shift only to
      // exponent 1 (s1_exp >= 1, so by at most 25); bit 26 stays clear and
      // the pack writes a subnormal.
      shifted  = s1_sum << (s1_exp - 8'd1);
      lost_bit = 1'b0;
      exp_adj  = 10'sd1;
    end else if (DSP_SHIFT && !SUBNORM) begin
      shifted  = nm_prod[27:0];
      lost_bit = 1'b0;
      exp_adj  = $signed({2'b00, s1_exp}) - $signed({5'd0, lzc}) + 10'sd1;
    end else begin
      // lzc == 1 means the leading one already sits at bit 26.
      // (NORM_B: B has shifted by its bytes; a zero sum shifts by nothing more, as it is zero)
      shifted  = NB ? (s1_sum << (lzf - 5'd1)) : (s1_sum << (lzc - 5'd1));
      lost_bit = 1'b0;
      exp_adj  = XB ? s1_expb - $signed({5'd0, lzf})
                    : NB ? $signed({2'b00, s1_exp}) - $signed({5'd0, s1_c, 3'd0}) - $signed({5'd0, lzf}) + 10'sd1
                    : $signed({2'b00, s1_exp}) - $signed({5'd0, lzc}) + 10'sd1;
    end
  end

  // --------------------------------- stage C: register the normalised sum
  //
  // The leading-zero count and the normalise shift end here; the round adder's
  // 24-bit carry chain starts in stage D. Standalone measurement put
  // lzc -> shift -> round-carry on one path at ~78.7 MHz, which was the
  // ceiling once the core-side path was cut.
  logic        sC_valid, sC_sign, sC_nan, sC_inf, sC_invalid;
  logic        sC_both_zero, sC_zero_sign, sC_exact_cancel, sC_zero_result;
  logic [27:0] sC_shifted;
  logic        sC_sticky_in;
  logic signed [9:0] sC_exp_adj;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sC_valid <= 1'b0;
    end else begin
      sC_valid        <= s1_valid;
      sC_sign         <= s1_sign;
      sC_shifted      <= shifted;
      sC_exp_adj      <= exp_adj;
      sC_sticky_in    <= lost_bit | s1_sticky;
      sC_nan          <= s1_nan;
      sC_inf          <= s1_inf;
      sC_invalid      <= s1_invalid;
      sC_both_zero    <= s1_both_zero;
      sC_zero_sign    <= s1_zero_sign;
      sC_exact_cancel <= s1_exact_cancel;
      sC_zero_result  <= (s1_sum == 28'd0) && !s1_sticky;
    end
  end

  // ------------------------------------------------ stage D: round and pack

  logic [22:0] frac_pre;
  logic        guard, round_bit, sticky;

  assign frac_pre  = sC_shifted[25:3];
  assign guard     = sC_shifted[2];
  assign round_bit = sC_shifted[1];
  assign sticky    = sC_shifted[0] | sC_sticky_in;

  logic round_up;
  assign round_up = guard & (round_bit | sticky | frac_pre[0]);

  logic [23:0]       frac_rnd;
  logic signed [9:0] exp_rnd;
  logic [22:0]       frac_final;

  assign frac_rnd   = {1'b0, frac_pre} + {23'd0, round_up};
  assign exp_rnd    = sC_exp_adj + (frac_rnd[23] ? 10'sd1 : 10'sd0);
  assign frac_final = frac_rnd[23] ? 23'd0 : frac_rnd[22:0];

  logic ovf, unf, is_zero_result;
  assign ovf            = (exp_rnd >= 10'sd255);
  assign unf            = (exp_rnd <= 10'sd0);
  assign is_zero_result = sC_zero_result;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_valid <= 1'b0;
      result    <= 32'd0;
      overflow  <= 1'b0;
      underflow <= 1'b0;
      invalid   <= 1'b0;
    end else begin
      out_valid <= sC_valid;
      invalid   <= sC_valid & sC_invalid;
      overflow  <= 1'b0;
      underflow <= 1'b0;

      if (sC_nan) begin
        result <= 32'h7fc00000;
      end else if (sC_inf) begin
        result <= {sC_sign, 8'hff, 23'd0};
      end else if (sC_both_zero) begin
        // -0 + -0 = -0; every other zero pairing gives +0.
        result <= {sC_zero_sign, 8'h00, 23'd0};
      end else if (sC_exact_cancel || is_zero_result) begin
        result <= 32'h00000000;
      end else if (SUBNORM && sC_exp_adj == 10'sd1 && !sC_shifted[26]) begin
        // R852: a subnormal (exponent field 0), or -- rounded up out of it --
        // the smallest normal (exponent 1, fraction 0)
        result <= {sC_sign, 7'd0, frac_rnd[23], frac_final};
      end else if (ovf) begin
        result   <= {sC_sign, 8'hff, 23'd0};
        overflow <= s1_valid;
      end else if (unf) begin
        result    <= {sC_sign, 8'h00, 23'd0};
        underflow <= sC_valid;
      end else begin
        result <= {sC_sign, exp_rnd[7:0], frac_final};
      end
    end
  end

endmodule
