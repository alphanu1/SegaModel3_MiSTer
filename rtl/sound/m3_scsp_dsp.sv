// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// THE TWO SCSPs' EFFECTS DSPs AND THEIR EFFECT RETURN, ONE ENGINE (study D-53). The 68000 and the
// slots run on the HPS (arcsnd); each sample the HPS sends the slots' DSP inputs (MIXS), the dry
// sums and the 68000's writes to the DSP's registers. This engine runs both chips' programs, adds
// EFREG x the effect pans to the dry sums and clips them by DAC18B: four channels a sample.
//
// It computes what arcsnd's SCSPDSP_Step (Supermodel's DSP, which follows Mednafen's pipeline) and
// arcsnd's mix compute, NOT MAME's DSP that my Model 2 core's m2_scsp_dsp follows:
//   * an MRD or MWT only starts an access; the next step does it, a read before a waiting write,
//     one access a step; a write left behind a read goes to the next step's address;
//   * IWT takes MEMVAL as it was before this step's access; INPUTS was read before the IWT;
//   * every step computes the address; ACC is 26 bits, SHIFTED saturates or wraps by SHIFT;
//   * ACC, INPUTS, FRC, Y, ADRS, MEMVAL and the pending access carry from sample to sample;
//   * the steps after the program (LastStep's, at most two) run as all-zero instructions, which is
//     what Supermodel's tail computes; a program of 128 steps has none;
//   * a stopped DSP does nothing (DEC stays); MIXS is the HPS's (it accumulates while stopped);
//   * the effect return is (EFREG x pan) >> 12 a term, EFREG as it stands (stopped or not).
// What it does not do, the HPS checks and then runs the DSP itself: a TABLE step, an RBL of 64 K
// words, and RBP moved after the program started (dsplink_guard, arcsnd's dsplink_shm.h).
//
// AREA (study §8.4): every table in block RAM, none in flip-flops; TEMP, MEMS, MIXS and EFREG
// share two M10K (bits 23..8 and 7..0, so a 68000 half-word write is a plain write of one lane);
// COEF, MADRS and the effect pans share one; the second chip's registers wait in a shadow copy
// and swap in, so nothing is selected by chip; PACK's and UNPACK's shifts are a multiply by a power
// of two on a second DSP block; the effect return uses the step multiplier and a running sum.
// The ring buffer is in M10K, 2^RING_BITS words a chip (every set measured: RBL 32 K words).
//
// SIX CLOCKS A STEP; both chips one after the other, then the effect return (22 clocks a channel):
// at most 2 x 128 x 6 + about 90, 1,627 clocks a sample measured, of the 2,494 a 44.1 kHz sample has
// at 110 MHz.
//   P0  the MPRO row is read
//   P1  the row; TEMP and MADRS are read; the pending ring access (a read, or a waiting write);
//       SHIFTED is taken
//   P2  TEMP is taken; MEMS / MIXS and COEF are read; the address; PACK's shift starts
//   P3  INPUTS; X x Y starts; UNPACK's shift starts; EFREG is written
//   P4  TEMP is written; the write's word (PACK) is taken
//   P5  ACC; MEMS is written; FRC, Y, ADRS, INPUTS, MEMVAL; the next access
//
// Commands (c_*), one a clock, only while idle (c_ready), laid out by the HPS as the arrays hold them
// (arcsnd's dsplink_map), so nothing is decoded here:
//   0  both lanes of hi / lo, c_data[23:0] at c_addr[8:0]: TEMP, MEMS, MIXS << 4, the dry sums
//   1  the high lane (bits 23..8) from c_data[15:0]: a 68000 word into TEMP / MEMS, or EFREG
//   2  the low lane (bits 7..0) from c_data[7:0]
//   3  a ring word, c_addr = {chip, offset from RBP << 12} (the HPS sends only the ring's)
//   4  cg (COEF, MADRS, the effect pans) at c_addr[7:0]
//   5  an MPRO word, c_addr = {chip, step 7, word 2} (rows from LastStep on sent as zeros)
//   6  a chip's setting, c_addr = {chip, which 2}: 0 RBL (c_data[1:0]), 1 Start (c_data[15:8] the
//      steps run, LastStep's and its empty ones), 2 DAC18B (c_data[0])
//   7  nothing (the bridge's clear writes it)
// Then go; at done each channel is read by out_sel (front L, front R, rear L, rear R), out_q a clock
// later: out_q[17:2] is the channel at 16 bits, and with DAC18B out_q is the 18-bit value (else its two
// low bits are 0).

`timescale 1ns/1ps

module m3_scsp_dsp #(
  parameter int RING_BITS = 15
) (
  input  logic        clk,
  input  logic        rst_n,

  input  logic        c_valid,
  output logic        c_ready,
  input  logic  [2:0] c_kind,
  input  logic [15:0] c_addr,
  input  logic [23:0] c_data,

  input  logic        go,             // one sample: both chips, then the effect return
  output logic        busy,
  output logic        done,           // pulse

  input  logic  [1:0] out_sel,        // a channel (front L, front R, rear L, rear R), read while idle:
  output logic [17:0] out_q,          // out_q a clock later
  output logic  [1:0] dac18           // each chip's DAC18B (for a bench)
);

  localparam int RW = RING_BITS;

  // ------------------------------------------------------------------ storage
  // MPRO: four 16-bit words a step, one array each (Model 2 R967: one array with byte lanes lost
  // the low bytes in Quartus 17.0), address {chip, step}
  (* ramstyle = "M10K" *) logic [15:0] mpro0 [256];
  (* ramstyle = "M10K" *) logic [15:0] mpro1 [256];
  (* ramstyle = "M10K" *) logic [15:0] mpro2 [256];
  (* ramstyle = "M10K" *) logic [15:0] mpro3 [256];
  // TEMP {0, chip, index 7}, MEMS {10, chip, 0, index 5}, MIXS {10, chip, 10, index 4} (MIXS << 4,
  // as INPUTS takes it), EFREG {11, chip, 00, index 4}, the four channels' sums {11, 0, 01, 00, k}
  // (the dry sums, then the effect return added); bits 23..8 and 7..0 (EFREG has no low lane)
  (* ramstyle = "M10K" *) logic [15:0] hi    [512];
  (* ramstyle = "M10K" *) logic  [7:0] lo    [512];
  // COEF {0, chip, index 6}, MADRS {10, chip, index 5}, the effect pans {11, chip, right, slot 4}
  (* ramstyle = "M10K" *) logic [15:0] cg    [256];
  (* ramstyle = "M10K" *) logic [15:0] ring  [2 ** (RW + 1)];   // {chip, offset}

  // the running chip's registers, and the other chip's in the shadow (swapped at each chip's end)
  logic signed [25:0] acc,  x_acc;
  logic signed [23:0] inp,  x_inp;
  logic        [12:0] frc,  x_frc;
  logic        [23:0] yreg, x_yreg;
  logic        [11:0] adrs, x_adrs;
  logic signed [23:0] memv, x_memv;
  logic        [15:0] dec,  x_dec;
  logic        [15:0] wval, x_wval;
  logic    [RW - 1:0] rwi,  x_rwi;     // the pending access's ring offset
  logic         [1:0] rp,   x_rp;      // read pending: 0, 1 float, 2 NOFL
  logic               wp,   x_wp;
  // per chip, by index (a few bits)
  logic         [1:0] stop_r;
  logic         [7:0] nst_r   [2];       // the steps run: LastStep's and its empty ones (the HPS's)
  logic         [1:0] rbl_r   [2];
  logic         [1:0] dac18_r;

  // ------------------------------------------------------------------ sequencer
  typedef enum logic [1:0] { IDLE, RUN, FIN, MIX } st_t;
  localparam logic [8:0] SUM = 9'h190;     // the channels' sums in hi / lo
  st_t        st;
  logic [2:0] ph;
  logic       ch;                        // the chip running (its registers are the working set)
  logic [7:0] k;                         // the step
  logic [7:0] nsteps;                    // the program and its empty steps
  logic [5:0] mt;                        // the effect return: a channel's clock (14 its sum read,
                                         // 16-31 its terms issued, 19-34 added, 35 its sum written)
  logic [1:0] mk;                        // ... the channel

  // ------------------------------------------------------------------ the row
  logic [15:0] m0_q, m1_q, m2_q, m3_q;
  always_ff @(posedge clk) begin
    m0_q <= mpro0[{ch, k[6:0]}];
    m1_q <= mpro1[{ch, k[6:0]}];
    m2_q <= mpro2[{ch, k[6:0]}];
    m3_q <= mpro3[{ch, k[6:0]}];
  end
  // past LastStep the rows are zero in MPRO (the HPS sends them so, dsplink_prog), so the empty steps
  // are all-zero instructions, which is what Supermodel's tail computes
  wire [63:0] ins = {m3_q, m2_q, m1_q, m0_q};
  wire  [6:0] TRA   = ins[14:8];
  wire        TWT   = ins[7];
  wire  [6:0] TWA   = ins[6:0];
  wire        XSEL  = ins[31];
  wire  [1:0] YSEL  = ins[30:29];
  wire  [5:0] IRA   = ins[27:22];
  wire        IWT   = ins[21];
  wire  [4:0] IWA   = ins[20:16];
  wire        MWT   = ins[46];
  wire        MRD   = ins[45];
  wire        EWT   = ins[44];
  wire  [3:0] EWA   = ins[43:40];
  wire        ADRL  = ins[39];
  wire        FRCL  = ins[38];
  wire  [1:0] SHFT  = ins[37:36];
  wire        YRL   = ins[35];
  wire        NEGB  = ins[34];
  wire        ZERO  = ins[33];
  wire        BSEL  = ins[32];
  wire  [5:0] COEFI = ins[62:57];
  wire        NOFL  = ins[56];
  wire  [4:0] MASA  = ins[54:50];
  wire        ADREB = ins[49];
  wire        NXADR = ins[48];

  // ------------------------------------------------------------------ the RAMs' ports
  logic  [8:0] hl_ra, hl_wa;
  logic        hi_we, lo_we;
  logic [15:0] hi_wd;
  logic  [7:0] lo_wd;
  logic [15:0] hi_q;
  logic  [7:0] lo_q;
  logic  [7:0] cg_ra, cg_wa;
  logic        cg_we;
  logic [15:0] cg_q;
  logic [RW:0] rg_wa;
  logic        rg_we;
  logic [15:0] rg_wd, rg_q;
  logic        mp_we;
  logic  [7:0] mp_wa;

  wire  [6:0] t_idx = (ph == 3'd1 ? TRA : TWA) + dec[6:0];   // TEMP's index: read in P1, written in P4

  // ------------------------------------------------------------------ the step's values
  logic signed [23:0] shifted_c, shifted;    // SHIFTED from ACC, and taken in P0
  logic signed [23:0] tempv;                 // TEMP, taken in P2
  logic signed [23:0] inputs;                // INPUTS after this step's select (P3-P5)
  logic signed [12:0] yv;
  logic    [RW - 1:0] addr_h;                // the next access's offset, taken in P2
  always_comb begin
    logic signed [26:0] a2;
    a2 = {acc, 1'b0};
    case (SHFT)
      2'd0:    shifted_c = (acc > 26'sh7fffff) ? 24'sh7fffff : (acc < -26'sh800000) ? -24'sh800000 : acc[23:0];
      2'd1:    shifted_c = (a2 > 27'sh7fffff) ? 24'sh7fffff : (a2 < -27'sh800000) ? -24'sh800000 : a2[23:0];
      2'd2:    shifted_c = a2[23:0];
      default: shifted_c = acc[23:0];
    endcase
    if (IRA <= 6'h2f)      inputs = {hi_q, lo_q};             // MEMS, or MIXS << 4
    else if (IRA <= 6'h31) inputs = 24'sd0;                    // EXTS: nothing feeds it on Model 3
    else                   inputs = inp;
    case (YSEL)
      2'd0:    yv = $signed(frc);
      2'd1:    yv = $signed(cg_q[15:3]);
      2'd2:    yv = $signed(yreg[23:11]);
      default: yv = $signed({1'b0, yreg[15:4]});
    endcase
  end

  // PACK's exponent: the leading sign bits of SHIFTED, at most 12
  logic [3:0] pk_e;
  always_comb begin
    logic [23:0] t;
    t = shifted ^ {shifted[22:0], 1'b0};
    pk_e = 4'd12;
    for (int i = 11; i >= 0; i--) if (t[23 - i]) pk_e = 4'(i);
  end
  // UNPACK's operand: {sign, ~sign (or sign past exponent 11), mantissa << 11}, shifted by
  // x 2^(11 - e) then >> 11
  wire  [3:0] up_e0 = rg_q[14:11];
  wire  [3:0] up_e  = (up_e0 > 4'd11) ? 4'd11 : up_e0;
  wire [23:0] up_u  = {rg_q[15], (up_e0 > 4'd11) ? rg_q[15] : ~rg_q[15], rg_q[10:0], 11'd0};

  // the step multiplier (X x Y, and the effect return's EFREG x pan) and the shifter (PACK, UNPACK)
  logic signed [23:0] mul_a;
  logic signed [15:0] mul_b;
  (* multstyle = "dsp" *) logic signed [39:0] prod;
  logic signed [23:0] sh_a;
  logic signed [12:0] sh_b;
  (* multstyle = "dsp" *) logic signed [36:0] sh_p;
  always_ff @(posedge clk) begin
    prod <= mul_a * mul_b;
    sh_p <= sh_a * sh_b;
  end

  // ------------------------------------------------------------------ P5: the step's results
  // (sign extensions as concatenations: Quartus 17.0 folds some signed casts wrongly)
  // (in the effect return the same adder sums the channel: TEMP's operand is the running sum)
  wire               mixing = st == MIX;
  wire signed [25:0] b_op  = (ZERO && !mixing) ? 26'sd0 : (BSEL && !mixing) ? acc : {{2{tempv[23]}}, tempv};
  wire signed [25:0] p12   = {prod[36], prod[36:12]};                  // (X x Y) >> 12, 26 bits
  wire signed [25:0] acc_n = (NEGB && !mixing) ? p12 - b_op : p12 + b_op;
  wire        [15:0] adrs_se = {{4{adrs[11]}}, adrs};
  wire        [15:0] addr_n  = cg_q + {15'd0, NXADR} + (ADREB ? adrs_se : 16'd0) + dec;
  wire        [15:0] rblmask = (16'h2000 << rbl_r[ch]) - 16'd1;

  // ------------------------------------------------------------------ the output
  // a channel's sum read from hi / lo (out_sel while idle), clipped as one: (sum >> 2) to 16 bits,
  // the 16-bit output and DAC18B's top 16 bits alike; DAC18B adds the two bits below (11 on the top
  // rail, 00 on the bottom). The sums are 24 bits: the HPS holds the dry sums to +-2^22, and past that
  // the total is on a rail either way (the effect return is under 2^21)
  always_comb begin
    logic signed [21:0] sh;
    logic               top, bot;
    sh  = {hi_q, lo_q[7:2]};
    top = sh > 22'sd32767;
    bot = sh < -22'sd32768;
    out_q[17:2] = top ? 16'h7fff : bot ? 16'h8000 : sh[15:0];
    out_q[1:0]  = !dac18_r[out_chip] ? 2'b00 : top ? 2'b11 : bot ? 2'b00 : lo_q[1:0];
  end
  logic out_chip;                        // the chip of the channel read (the one out_q holds)
  always_ff @(posedge clk) out_chip <= out_sel[1];

  // ------------------------------------------------------------------ the RAMs
  // Commands, as the HPS lays them out (its dsplink_map): 0 both lanes of hi / lo (24 bits), 1 the high
  // lane, 2 the low, 3 the ring {chip, offset}, 4 COEF / MADRS / pans, 5 MPRO {chip, step, word},
  // 6 a chip's setting {chip, which}: 0 RBL, 1 Start {steps run 8, LastStep 8}, 2 DAC18B
  assign c_ready = st == IDLE;
  wire        cv   = c_valid && st == IDLE;

  always_comb begin
    // reads: the row's TEMP in P1, its MEMS / MIXS after; MADRS in P1, COEF after; in the effect
    // return a channel's sum, then EFREG and the pans; idle, the channel out_sel asks for
    hl_ra = (ph == 3'd1) ? {1'b0, ch, t_idx} : {2'b10, ch, IRA};
    cg_ra = (ph == 3'd1) ? {2'b10, ch, MASA} : {1'b0, ch, COEFI};
    if (st == MIX) begin
      hl_ra = mt[4] ? {2'b11, mk[1], 2'b00, mt[3:0]} : SUM | {7'd0, mk};
      cg_ra = {2'b11, mk, mt[3:0]};
    end
    if (st == IDLE) hl_ra = SUM | {7'd0, out_sel};
    // the lanes: EFREG in P3, TEMP in P4, MEMS in P5, a channel's sum at its end, else a command
    hl_wa = {2'b11, ch, 2'b00, EWA}; hi_wd = shifted[23:8]; lo_wd = shifted[7:0];
    hi_we = 1'b0; lo_we = 1'b0;
    if (st == RUN) begin
      case (ph)
        3'd3: hi_we = EWT;
        3'd4: begin hl_wa = {1'b0, ch, t_idx}; hi_we = TWT; lo_we = TWT; end
        3'd5: begin hl_wa = {2'b10, ch, 1'b0, IWA}; hi_wd = memv[23:8]; lo_wd = memv[7:0]; hi_we = IWT; lo_we = IWT; end
        default: ;
      endcase
    end else if (st == MIX) begin
      hl_wa = SUM | {7'd0, mk}; hi_wd = tempv[23:8]; lo_wd = tempv[7:0];
      hi_we = mt == 6'd35; lo_we = mt == 6'd35;
    end else begin
      hl_wa = c_addr[8:0]; hi_wd = c_data[23:8]; lo_wd = c_data[7:0];
      hi_we = cv && (c_kind == 3'd0 || c_kind == 3'd1);
      lo_we = cv && (c_kind == 3'd0 || c_kind == 3'd2);
      if (c_kind == 3'd1) hi_wd = c_data[15:0];
    end
    cg_wa = c_addr[7:0];
    cg_we = cv && c_kind == 3'd4;
    mp_wa = c_addr[9:2];
    mp_we = cv && c_kind == 3'd5;
    // the ring: the run's waiting write in P1 (no read pending), else a command
    rg_wa = (st == RUN) ? {ch, rwi} : c_addr[RW:0];
    rg_wd = (st == RUN) ? wval : c_data[15:0];
    rg_we = (st == RUN) ? (ph == 3'd1 && rp == 2'd0 && wp) : (cv && c_kind == 3'd3);
  end

  always_ff @(posedge clk) begin
    hi_q <= hi[hl_ra];
    lo_q <= lo[hl_ra];
    cg_q <= cg[cg_ra];
    rg_q <= ring[{ch, rwi}];
    if (hi_we) hi[hl_wa] <= hi_wd;
    if (lo_we) lo[hl_wa] <= lo_wd;
    if (cg_we) cg[cg_wa] <= c_data[15:0];
    if (rg_we) ring[rg_wa] <= rg_wd;
    if (mp_we && c_addr[1:0] == 2'd0) mpro0[mp_wa] <= c_data[15:0];
    if (mp_we && c_addr[1:0] == 2'd1) mpro1[mp_wa] <= c_data[15:0];
    if (mp_we && c_addr[1:0] == 2'd2) mpro2[mp_wa] <= c_data[15:0];
    if (mp_we && c_addr[1:0] == 2'd3) mpro3[mp_wa] <= c_data[15:0];
  end

  // the step's temporaries and the multipliers' operands (no reset: they would cost an enable)
  always_ff @(posedge clk) begin
    if (st == RUN) begin
      case (ph)
        3'd1: shifted <= shifted_c;
        3'd2: begin
          tempv <= {hi_q, lo_q};
          addr_h <= RW'(addr_n & rblmask);
          sh_a <= shifted; sh_b <= 13'sd1 <<< ((pk_e == 4'd12) ? 4'd11 : pk_e);       // PACK: x 2^e
        end
        3'd3: begin
          mul_a <= XSEL ? inputs : tempv;
          mul_b <= {{3{yv[12]}}, yv};
          sh_a <= up_u; sh_b <= 13'sd1 <<< (4'd11 - up_e);                               // UNPACK
        end
        default: ;
      endcase
    end
    if (st == MIX) begin
      if (mt == 6'd15) tempv <= {hi_q, lo_q};                                       // the channel's dry sum
      if (mt >= 6'd17 && mt <= 6'd32) begin mul_a <= {8'd0, cg_q}; mul_b <= hi_q; end // pan x EFREG
      if (mt >= 6'd19 && mt <= 6'd34) tempv <= acc_n[23:0];                          // + (EFREG x pan) >> 12
    end
  end

  // ------------------------------------------------------------------ the sequencer
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= IDLE; ph <= '0; ch <= 1'b0; k <= '0; nsteps <= '0; done <= 1'b0;
      mt <= '0; mk <= '0;
      stop_r <= 2'b11; nst_r[0] <= 8'd2; nst_r[1] <= 8'd2;
      rbl_r[0] <= '0; rbl_r[1] <= '0; dac18_r <= '0;
      acc <= '0; inp <= '0; frc <= '0; yreg <= '0; adrs <= '0; memv <= '0; dec <= '0; wval <= '0; rwi <= '0; rp <= '0; wp <= 1'b0;
      x_acc <= '0; x_inp <= '0; x_frc <= '0; x_yreg <= '0; x_adrs <= '0; x_memv <= '0; x_dec <= '0; x_wval <= '0; x_rwi <= '0; x_rp <= '0; x_wp <= 1'b0;
    end else begin
      done <= 1'b0;

      // commands
      if (cv && c_kind == 3'd6)
        case (c_addr[1:0])
          2'd0: rbl_r[c_addr[2]] <= c_data[1:0];
          2'd1: begin stop_r[c_addr[2]] <= 1'b0; nst_r[c_addr[2]] <= c_data[15:8]; end   // (LastStep, 7:0, is the HPS's)
          default: dac18_r[c_addr[2]] <= c_data[0];
        endcase

      case (st)
        IDLE: if (go && !c_valid) begin
          ch <= 1'b0; k <= '0; ph <= '0;
          st <= stop_r[0] ? FIN : RUN;          // chip 0 stopped: straight to its end
          nsteps <= nst_r[0];
        end
        RUN: begin
          ph <= ph + 3'd1;
          case (ph)
            3'd4: if (MWT) wval <= NOFL ? shifted[23:8] : {shifted[23], pk_e, sh_p[21:11]};
            3'd5: begin
              ph <= 3'd0;
              acc <= acc_n;
              inp <= inputs;
              if (YRL)  yreg <= inputs;
              if (FRCL) frc <= (SHFT == 2'd3) ? {1'b0, shifted[11:0]} : shifted[23:11];
              if (ADRL) adrs <= (SHFT == 2'd3) ? shifted[23:12] : {{4{inputs[23]}}, inputs[23:16]};
              // the access done in P1, then this step's
              begin
                logic [1:0] r;
                logic       w;
                r = rp; w = wp;
                if (rp != 2'd0) begin memv <= (rp == 2'd2) ? {rg_q, 8'd0} : sh_p[34:11]; r = 2'd0; end
                else if (w) w = 1'b0;
                rwi <= addr_h;
                if (MRD) r = NOFL ? 2'd2 : 2'd1;
                if (MWT) w = 1'b1;
                rp <= r; wp <= w;
              end
              k <= k + 8'd1;
              if (k + 8'd1 == nsteps) st <= FIN;
            end
            default: ;
          endcase
        end
        FIN: begin   // the chip's DEC, the swap, then the next chip or the effect return
          {acc, inp, frc, yreg, adrs, memv, dec, wval, rwi, rp, wp} <=
            {x_acc, x_inp, x_frc, x_yreg, x_adrs, x_memv, x_dec, x_wval, x_rwi, x_rp, x_wp};
          {x_acc, x_inp, x_frc, x_yreg, x_adrs, x_memv, x_dec, x_wval, x_rwi, x_rp, x_wp} <=
            {acc, inp, frc, yreg, adrs, memv, dec - {15'd0, !stop_r[ch]}, wval, rwi, rp, wp};
          k <= '0; ph <= '0;
          if (ch == 1'b0) begin
            ch <= 1'b1;
            nsteps <= nst_r[1];
            st <= stop_r[1] ? FIN : RUN;
          end else begin
            ch <= 1'b0; st <= MIX; mt <= 6'd14; mk <= '0;
          end
        end
        default: begin   // MIX: each channel's sum, its 16 terms a clock apart, the sum back
          mt <= mt + 6'd1;
          if (mt == 6'd35) begin
            mt <= 6'd14; mk <= mk + 2'd1;
            if (mk == 2'd3) begin st <= IDLE; done <= 1'b1; end
          end
        end
      endcase
    end
  end
  assign busy = st != IDLE;
  assign dac18 = dac18_r;

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, ins[63], ins[55], ins[47], ins[28], ins[15], c_addr[15:10], cg_q[2:0],
                   prod[39:37], prod[11:0], sh_p[36:35], sh_p[10:0], acc_n[25:24], c_data[23:16]};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
