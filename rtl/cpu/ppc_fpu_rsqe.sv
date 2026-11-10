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
// PowerPC 603e frsqrte as a pipelined estimate (study D-22; the 603e issues it
// 1-1-1, 603e Table 6-5). The manual bounds the estimate to one part in 32 of
// 1/sqrt(frB) [PEM 8-82] and documents no table, so the table is my own:
//
//   frB = m * 2^e (a denormal normalised first), x = m for e even, 2m for e
//   odd, so 1/sqrt(frB) = y * 2^-floor(e/2) with y = 1/sqrt(x) in (0.5, 1].
//   128 entries, indexed by e's parity and m's top 6 fraction bits; each a
//   line B - S * dx over its segment, dx the next 12 bits:
//     y_fix = B * 2^10 - S * dx  (units of 2^-32; B 22 bits, S 13 bits)
//   The result is 2y truncated to 23 fraction bits at exponent -floor(e/2)-1:
//   its significand fits a single, so a result in single range feeds the
//   fast path (D-21) unchanged. Worst relative error over every input:
//   1.27e-5 = 2^-16.26 (sim/fpu/gen_rsqe_tab.py, which writes the table here
//   and in the reference model).
//
// Special operands [PEM 8-82, frsqrte table]: -inf and any other negative
// non-NaN -> default QNaN, VXSQRT; -0 / +0 -> -inf / +inf, ZX; +inf -> +0;
// SNaN -> QNaN (quieted), VXSNAN; QNaN -> itself. VE=1 invalid or ZE=1 zero
// divide: no result, FPRF unchanged, FR FI cleared [PEM 3.3.6.1].
// UNDEFINED FR, FI otherwise [PEM 8-83]: Supermodel's frsqrte (1.0 / sqrt) and
// every Supermodel FP op leave them unchanged, so they are not written. XX:
// not in frsqrte's list, unchanged.
//
// Stages (RA, RC optional; DLY extra registers to match the fast port's
// latency; the last register is the double unit's output register):
//   A  classify; leading zeros of a denormal's fraction           -> RA (QA)
//   B  normalise, parity, index, dx, result exponent              -> RB
//   C  table                                                       -> RC (QC, DSP in)
//   D  S * dx                                                      -> RD (DSP out), + DLY
//   E  B * 2^10 - S * dx, result, flags                            -> output register
// Latency 1 + QA + 1 + QC + 1 + DLY: 5 with QA = QC = 1, DLY = 0.

// The table is a 128-entry ROM, below the size Quartus converts unasked: the
// second setting lets it (study R100).
(* altera_attribute = "-name AUTO_SHIFT_REGISTER_RECOGNITION OFF; -name ALLOW_ANY_ROM_SIZE_FOR_RECOGNITION ON" *)
module ppc_fpu_rsqe
  import ppc_fpu_pkg::*;
#(
  parameter int TAGW = 6,
  parameter bit QA   = 1'b1,
  parameter bit QC   = 1'b1,
  parameter int DLY  = 0
) (
  input  logic            clk,
  input  logic            rst,

  input  logic            in_valid,
  input  logic [7:0]      in_ctl,     // FPSCR[24:31]: VE OE UE ZE XE NI RN
  input  logic [63:0]     in_b,
  input  logic [TAGW-1:0] in_tag,

  output logic            o_valid,
  output logic [TAGW-1:0] o_tag,
  output logic [63:0]     o_res,
  output fpu_flags_t      o_flags
);

  // the result when it is not an estimate, and the flags; fixed from stage A
  typedef struct packed {
    logic [TAGW-1:0] tag;
    logic            spc;       // a special result, no estimate:
    logic            sign;      //   {sign, exp, f} below
    logic [10:0]     exp;
    logic [51:0]     f;
    logic [4:0]      fprf;      // its class
    logic            wr, frfi_we;
    logic            vxsnan, vxsqrt, zx;
  } side_t;

  // ================================================================== A
  wire        b_s  = in_b[63];
  wire [10:0] b_e  = in_b[62:52];
  wire [51:0] b_f  = in_b[51:0];
  wire        e0   = (b_e == 11'd0), e1 = (b_e == 11'h7FF), f0 = (b_f == 52'd0);
  wire        zero = e0 & f0, den = e0 & ~f0, inf = e1 & f0, nan = e1 & ~f0;
  wire        snan = nan & ~b_f[51];
  wire        neg  = b_s & ~zero & ~nan;            // -inf included
  wire        ve = in_ctl[7], ze = in_ctl[4];

  logic [5:0] lz;
  ppc_fpu_lzc #(.W(52)) u_lz (.clk (clk), .x (b_f), .n (lz));

  side_t sa;
  always_comb begin
    sa.tag     = in_tag;
    sa.spc     = nan | neg | zero | inf;
    sa.sign    = (nan | zero) & b_s;
    sa.exp     = (nan | neg | zero) ? 11'h7FF : 11'd0;    // +inf -> +0
    // (R140) below bit 51 a NaN's payload is b_f, and with a zero fraction b_f is zero anyway, so those
    // bits need e1 alone, not the 52-bit zero test: nan ? (b_f | bit 51) : neg ? bit 51 : 0, exactly
    // (ppc_cpu spike: ex_fbl -> fb_x -> f0 -> nan -> ra_q.sd.f, -2.10 ns at 120 MHz)
    sa.f       = {nan | neg, {51{e1}} & b_f[50:0]};
    sa.fprf    = (nan | neg) ? FPRF_QNAN : zero ? (b_s ? FPRF_NINF : FPRF_PINF) : FPRF_PZERO;
    sa.wr      = ~((snan | neg) & ve) & ~(zero & ze);
    sa.frfi_we = ~sa.wr;
    sa.vxsnan  = snan;
    sa.vxsqrt  = neg;
    sa.zx      = zero;
  end

  typedef struct packed {
    logic [51:0] f;
    logic [10:0] e;
    logic [5:0]  lz;
    logic        den;
    side_t       sd;
  } ra_t;
  ra_t  sa_d, ra_q;
  logic ra_v;
  always_comb begin
    sa_d.f = b_f;  sa_d.e = b_e;  sa_d.lz = lz;  sa_d.den = den;  sa_d.sd = sa;
  end
  generate if (QA) begin : g_ra
    always_ff @(posedge clk) begin
      if (rst) ra_v <= 1'b0;
      else     ra_v <= in_valid;
      ra_q <= sa_d;
    end
  end else begin : g_ca
    assign ra_v = in_valid;
    assign ra_q = sa_d;
  end endgenerate

  // ================================================================== B
  // A denormal's fraction shifted so its leading one is the hidden bit: by
  // lz + 1; only the 18 bits the table and dx read are formed.
  wire [51:0] fsh = ra_q.f << ra_q.lz;
  wire [17:0] mf  = ra_q.den ? fsh[50:33] : ra_q.f[51:34];
  // e = E - 1023 (a denormal: -1023 - lz, as if E = -lz). The result is
  // 2y * 2^(-floor(e/2) - 1): biased 1022 - floor(e/2) = (3068 - E) >> 1,
  // all unsigned; e is odd when E is even
  wire [11:0] rx2 = ra_q.den ? (12'd3068 + {6'd0, ra_q.lz}) : (12'd3068 - {1'b0, ra_q.e});
  wire        par = ra_q.den ? ~ra_q.lz[0] : ~ra_q.e[0];
  wire [10:0] rex = rx2[11:1];

  typedef struct packed {
    logic [6:0]  idx;
    logic [11:0] dx;
    logic [10:0] rex;
    side_t       sd;
  } rb_t;
  rb_t  rb_q;
  logic rb_v;
  always_ff @(posedge clk) begin
    if (rst) rb_v <= 1'b0;
    else     rb_v <= ra_v;
    rb_q.idx <= {par, mf[17:12]};
    rb_q.dx  <= mf[11:0];
    rb_q.rex <= rex;
    rb_q.sd  <= ra_q.sd;
  end

  // ================================================================== C
  // the table: {B, S}. In an M10K ROM read in stage C (QC = 1; study R100:
  // the 128 x 35 table was about 60 ALM of logic), else logic
  function automatic logic [34:0] rsq_tab(input logic [6:0] i);
    logic [34:0] t;
    case (i)
      // BEGIN rsqe table (sim/fpu/gen_rsqe_tab.py)
      7'd0  : t = {22'd4194253, 13'd8097};
      7'd1  : t = {22'd4161866, 13'd7913};
      7'd2  : t = {22'd4130218, 13'd7735};
      7'd3  : t = {22'd4099281, 13'd7563};
      7'd4  : t = {22'd4069028, 13'd7398};
      7'd5  : t = {22'd4039436, 13'd7239};
      7'd6  : t = {22'd4010481, 13'd7086};
      7'd7  : t = {22'd3982139, 13'd6938};
      7'd8  : t = {22'd3954389, 13'd6795};
      7'd9  : t = {22'd3927212, 13'd6656};
      7'd10 : t = {22'd3900587, 13'd6523};
      7'd11 : t = {22'd3874497, 13'd6394};
      7'd12 : t = {22'd3848924, 13'd6269};
      7'd13 : t = {22'd3823850, 13'd6148};
      7'd14 : t = {22'd3799260, 13'd6031};
      7'd15 : t = {22'd3775138, 13'd5917};
      7'd16 : t = {22'd3751470, 13'd5807};
      7'd17 : t = {22'd3728241, 13'd5701};
      7'd18 : t = {22'd3705439, 13'd5597};
      7'd19 : t = {22'd3683050, 13'd5497};
      7'd20 : t = {22'd3661062, 13'd5400};
      7'd21 : t = {22'd3639463, 13'd5305};
      7'd22 : t = {22'd3618242, 13'd5214};
      7'd23 : t = {22'd3597388, 13'd5125};
      7'd24 : t = {22'd3576891, 13'd5038};
      7'd25 : t = {22'd3556740, 13'd4954};
      7'd26 : t = {22'd3536925, 13'd4872};
      7'd27 : t = {22'd3517438, 13'd4792};
      7'd28 : t = {22'd3498270, 13'd4715};
      7'd29 : t = {22'd3479412, 13'd4639};
      7'd30 : t = {22'd3460855, 13'd4566};
      7'd31 : t = {22'd3442592, 13'd4494};
      7'd32 : t = {22'd3424616, 13'd4425};
      7'd33 : t = {22'd3406918, 13'd4357};
      7'd34 : t = {22'd3389491, 13'd4291};
      7'd35 : t = {22'd3372329, 13'd4226};
      7'd36 : t = {22'd3355426, 13'd4163};
      7'd37 : t = {22'd3338774, 13'd4102};
      7'd38 : t = {22'd3322367, 13'd4042};
      7'd39 : t = {22'd3306200, 13'd3983};
      7'd40 : t = {22'd3290267, 13'd3926};
      7'd41 : t = {22'd3274562, 13'd3871};
      7'd42 : t = {22'd3259079, 13'd3816};
      7'd43 : t = {22'd3243814, 13'd3763};
      7'd44 : t = {22'd3228762, 13'd3711};
      7'd45 : t = {22'd3213917, 13'd3661};
      7'd46 : t = {22'd3199276, 13'd3611};
      7'd47 : t = {22'd3184832, 13'd3562};
      7'd48 : t = {22'd3170582, 13'd3515};
      7'd49 : t = {22'd3156522, 13'd3469};
      7'd50 : t = {22'd3142648, 13'd3423};
      7'd51 : t = {22'd3128954, 13'd3379};
      7'd52 : t = {22'd3115438, 13'd3336};
      7'd53 : t = {22'd3102096, 13'd3293};
      7'd54 : t = {22'd3088924, 13'd3252};
      7'd55 : t = {22'd3075918, 13'd3211};
      7'd56 : t = {22'd3063075, 13'd3171};
      7'd57 : t = {22'd3050392, 13'd3132};
      7'd58 : t = {22'd3037865, 13'd3094};
      7'd59 : t = {22'd3025490, 13'd3056};
      7'd60 : t = {22'd3013266, 13'd3019};
      7'd61 : t = {22'd3001189, 13'd2983};
      7'd62 : t = {22'd2989256, 13'd2948};
      7'd63 : t = {22'd2977464, 13'd2913};
      7'd64 : t = {22'd2965785, 13'd5726};
      7'd65 : t = {22'd2942884, 13'd5595};
      7'd66 : t = {22'd2920505, 13'd5469};
      7'd67 : t = {22'd2898629, 13'd5348};
      7'd68 : t = {22'd2877238, 13'd5231};
      7'd69 : t = {22'd2856313, 13'd5119};
      7'd70 : t = {22'd2835838, 13'd5010};
      7'd71 : t = {22'd2815797, 13'd4906};
      7'd72 : t = {22'd2796175, 13'd4805};
      7'd73 : t = {22'd2776958, 13'd4707};
      7'd74 : t = {22'd2758132, 13'd4612};
      7'd75 : t = {22'd2739683, 13'd4521};
      7'd76 : t = {22'd2721600, 13'd4433};
      7'd77 : t = {22'd2703870, 13'd4347};
      7'd78 : t = {22'd2686482, 13'd4264};
      7'd79 : t = {22'd2669425, 13'd4184};
      7'd80 : t = {22'd2652690, 13'd4106};
      7'd81 : t = {22'd2636265, 13'd4031};
      7'd82 : t = {22'd2620141, 13'd3958};
      7'd83 : t = {22'd2604310, 13'd3887};
      7'd84 : t = {22'd2588762, 13'd3818};
      7'd85 : t = {22'd2573489, 13'd3752};
      7'd86 : t = {22'd2558484, 13'd3687};
      7'd87 : t = {22'd2543738, 13'd3624};
      7'd88 : t = {22'd2529244, 13'd3562};
      7'd89 : t = {22'd2514995, 13'd3503};
      7'd90 : t = {22'd2500984, 13'd3445};
      7'd91 : t = {22'd2487204, 13'd3389};
      7'd92 : t = {22'd2473650, 13'd3334};
      7'd93 : t = {22'd2460316, 13'd3280};
      7'd94 : t = {22'd2447194, 13'd3229};
      7'd95 : t = {22'd2434280, 13'd3178};
      7'd96 : t = {22'd2421569, 13'd3129};
      7'd97 : t = {22'd2409054, 13'd3081};
      7'd98 : t = {22'd2396732, 13'd3034};
      7'd99 : t = {22'd2384597, 13'd2988};
      7'd100: t = {22'd2372644, 13'd2944};
      7'd101: t = {22'd2360869, 13'd2900};
      7'd102: t = {22'd2349268, 13'd2858};
      7'd103: t = {22'd2337836, 13'd2817};
      7'd104: t = {22'd2326570, 13'd2776};
      7'd105: t = {22'd2315465, 13'd2737};
      7'd106: t = {22'd2304517, 13'd2699};
      7'd107: t = {22'd2293723, 13'd2661};
      7'd108: t = {22'd2283080, 13'd2624};
      7'd109: t = {22'd2272583, 13'd2588};
      7'd110: t = {22'd2262229, 13'd2553};
      7'd111: t = {22'd2252016, 13'd2519};
      7'd112: t = {22'd2241940, 13'd2486};
      7'd113: t = {22'd2231998, 13'd2453};
      7'd114: t = {22'd2222187, 13'd2421};
      7'd115: t = {22'd2212505, 13'd2389};
      7'd116: t = {22'd2202948, 13'd2359};
      7'd117: t = {22'd2193513, 13'd2329};
      7'd118: t = {22'd2184199, 13'd2299};
      7'd119: t = {22'd2175003, 13'd2270};
      7'd120: t = {22'd2165921, 13'd2242};
      7'd121: t = {22'd2156953, 13'd2215};
      7'd122: t = {22'd2148095, 13'd2187};
      7'd123: t = {22'd2139345, 13'd2161};
      7'd124: t = {22'd2130701, 13'd2135};
      7'd125: t = {22'd2122161, 13'd2110};
      7'd126: t = {22'd2113723, 13'd2085};
      7'd127: t = {22'd2105390, 13'd2060};
      default: t = '0;
      // END rsqe table
    endcase
    rsq_tab = t;
  endfunction
  logic [34:0] t;

  typedef struct packed {
    logic [21:0] b;
    logic [12:0] s;
    logic [11:0] dx;
    logic [10:0] rex;
    side_t       sd;
  } rc_t;
  rc_t  sc_d, rc_q;
  logic rc_v;
  always_comb begin
    sc_d.b = t[34:13];  sc_d.s = t[12:0];  sc_d.dx = rb_q.dx;  sc_d.rex = rb_q.rex;  sc_d.sd = rb_q.sd;
  end
  generate if (QC) begin : g_rc
    (* romstyle = "M10K" *) logic [34:0] t_q;
    always_ff @(posedge clk) t_q <= rsq_tab(rb_q.idx);
    assign t = rsq_tab(rb_q.idx);                    // (unused here: rc_q's b and s are t_q)
    rc_t rc_r;
    always_ff @(posedge clk) begin
      if (rst) rc_v <= 1'b0;
      else     rc_v <= rb_v;
      rc_r <= sc_d;
    end
    always_comb begin
      rc_q = rc_r;
      rc_q.b = t_q[34:13];  rc_q.s = t_q[12:0];
    end
  end else begin : g_cc
    assign t = rsq_tab(rb_q.idx);
    assign rc_v = rb_v;
    assign rc_q = sc_d;
  end endgenerate

  // ================================================================== D
  typedef struct packed {
    logic [21:0] b;
    logic [24:0] sdx;
    logic [10:0] rex;
    side_t       sd;
  } rd_t;
  rd_t  rd_q [DLY + 1];
  logic rd_v [DLY + 1];
  always_ff @(posedge clk) begin
    if (rst) rd_v[0] <= 1'b0;
    else     rd_v[0] <= rc_v;
    rd_q[0].b   <= rc_q.b;
    rd_q[0].sdx <= rc_q.s * rc_q.dx;
    rd_q[0].rex <= rc_q.rex;
    rd_q[0].sd  <= rc_q.sd;
    for (int k = 1; k <= DLY; k++) begin
      if (rst) rd_v[k] <= 1'b0;
      else     rd_v[k] <= rd_v[k - 1];
      rd_q[k] <= rd_q[k - 1];
    end
  end

  // The side word from RA to E (area pass): instead of the registers rb_q.sd, rc_r.sd and rd_q[0 .. DLY].sd, an
  // M10K written every clock at sd_p and read at sd_p - (SDD - 2) into its address register, then its output
  // register sd_e: every clock the word rd_q[DLY].sd held (the chain is never held or reset). With 2^SDA >= SDD
  // the word read has not been written over, and the read address is never the one being written. (An MLAB
  // here freed no ALMs: a whole LAB used four deep of 32.) Below three registers the chain stays as it was.
  localparam int SDD = 2 + (QC ? 1 : 0) + DLY;       // registers after ra_q: rb_q, rc_r, rd_q[0 .. DLY]
  localparam int SDA = (SDD < 3) ? 1 : $clog2(SDD);
  localparam int SDW = $bits(side_t);
  logic [SDW-1:0] sd_e;
  generate if (SDD >= 3) begin : g_sdm
    logic [SDA-1:0] sd_p;            // any start: the delay holds whatever sd_p starts at
    (* ramstyle = "M10K, no_rw_check" *) logic [SDW-1:0] sd_m [0:(1 << SDA) - 1];
    logic [SDW-1:0] sd_1;
    always_ff @(posedge clk) begin
      sd_p <= sd_p + 1'b1;
      sd_m[sd_p] <= ra_q.sd;
      sd_1 <= sd_m[sd_p - SDA'(SDD - 2)];
      sd_e <= sd_1;
    end
  end else begin : g_sdr
    assign sd_e = rd_q[DLY].sd;
  end endgenerate

  // ================================================================== E
  rd_t re;
  always_comb begin
    re = rd_q[DLY];
    re.sd = sd_e;
  end
  wire [31:0] y   = {re.b, 10'd0} - {7'd0, re.sdx};
  wire [63:0] est = {1'b0, re.rex, y[30:8], 29'd0};
  wire [63:0] res = re.sd.spc ? {re.sd.sign, re.sd.exp, re.sd.f} : est;

  assign o_valid = rd_v[DLY];
  assign o_tag   = re.sd.tag;
  assign o_res   = res;
  always_comb begin
    o_flags            = '0;
    o_flags.wr         = re.sd.wr;
    o_flags.frfi_we    = re.sd.frfi_we;    // FR = FI = 0 with it
    o_flags.fprf_we    = re.sd.wr;
    o_flags.fprf       = re.sd.spc ? re.sd.fprf : FPRF_PNORM;   // an estimate is always normal
    o_flags.exc.vxsnan = re.sd.vxsnan;
    o_flags.exc.vxsqrt = re.sd.vxsqrt;
    o_flags.exc.zx     = re.sd.zx;
  end

endmodule
