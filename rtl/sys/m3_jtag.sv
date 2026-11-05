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
// The Real3D's JTAG scan chain as the PowerPC sees it through the system
// registers (F010000C: bit 7 TRST, 6 TCK, 5 TDI, 2 TMS; F0100010 bit 5 TDO).
// VF3 bit-bangs it at boot (frames 211-232 of the Supermodel trace, about
// 12,700 TCK cycles): it reads the ID codes, writes and reads back the mode
// words, and moves the Mercury's memory configuration. A game that reads
// the wrong ID or mode word stops, so the chain is built device for device.
//
// After Supermodel's JTAG.cpp (036af1c), IEEE 1149.1, Step 1.0 chain of ten
// devices in TDI-to-TDO order:
//   0 Jupiter  1 Mercury  2 Venus  3 Earth  4 3D-RAM  5 Mars  6 Mars
//   7 3D-RAM   8 3D-RAM   9 3D-RAM
// ASIC: 5-bit IR (capture 00001); DR by instruction: 3 IDCODE (32), 8 the mode
// word (32, read and written), 9 on the Mercury its memory configuration (57
// bits: the three 19-bit Real3D configuration registers), anything else
// BYPASS. 3D-RAM: 4-bit IR (capture 1001), always BYPASS. Venus's polygon
// memory instructions (0C, 0F, 11) are BYPASS, as Supermodel has them unless
// mode-word bit 13 is set, which no Model 3 game sets.
//
// Supermodel's controller: a write with TRST low resets the devices (not the
// TAP); otherwise only a change of TCK counts. A rising edge acts in the TAP
// state it finds (Test-Logic-Reset resets the devices, Capture captures,
// Shift shifts the whole chain one bit) and then moves the TAP; a falling
// edge latches TDO from the last device in Shift, or updates in Update.
//
// Steps (study R100, Supermodel's CJTAG::SetStepping): jstep 0 Step 1.0, 1
// Step 1.5 (the same ten devices, other ID codes), 2 Step 2.0 and 3 Step 2.1:
// seventeen devices,
//   0 Jupiter  1 Mercury  2 Venus  3 Earth  4 3D-RAM  5 Mars  6 Mars  7-9 3D-RAM
//   10 Earth  11 3D-RAM  12 Mars  13 Mars  14-16 3D-RAM
// The second Earth and the last two Mars are Supermodel's dummies: full ASICs
// on the chain whose mode words go nowhere, so no mw_we for them.
//
// Area (study 8.4): the ten devices' shift registers, instructions and mode
// words are one word each in MLAB, and an edge is applied to them one device
// a clock (about 12 clocks an edge). The PowerPC takes far longer than that
// between two writes; `busy` holds the bus's next write meanwhile.

module m3_jtag (
  input  logic        clk,
  input  logic        rst,

  input  logic [1:0]  jstep,         // 0 Step 1.0, 1 Step 1.5, 2 Step 2.0, 3 Step 2.1 (run time)
  input  logic        wr,            // a write of F010000C's byte (one cycle)
  input  logic [7:0]  wdata,
  output logic        busy,
  output logic        tdo,

  input  logic [18:0] cfg0, cfg1, cfg2,   // Real3D configuration registers (9C000000/4/8)
  output logic        cfg_we,             // the Mercury's instruction 9 update
  output logic [18:0] cfg0_w, cfg1_w, cfg2_w,
  output logic        mw_we,              // a mode word updated (stage B uses Mercury's and Mars's)
  output logic [3:0]  mw_dev,
  output logic [31:0] mw_val
);

  // ---------------------------------------------------------------- TAP
  typedef enum logic [3:0] {
    TLR = 4'd0, RTI = 4'd1, SELDR = 4'd2, CAPDR = 4'd3, SHDR = 4'd4, EX1DR = 4'd5, PAUDR = 4'd6,
    EX2DR = 4'd7, UPDDR = 4'd8, SELIR = 4'd9, CAPIR = 4'd10, SHIR = 4'd11, EX1IR = 4'd12,
    PAUIR = 4'd13, EX2IR = 4'd14, UPDIR = 4'd15
  } tap_t;

  function automatic tap_t next_st(input tap_t s, input logic tms);
    case (s)
      TLR:   next_st = tms ? TLR   : RTI;
      RTI:   next_st = tms ? SELDR : RTI;
      SELDR: next_st = tms ? SELIR : CAPDR;
      CAPDR: next_st = tms ? EX1DR : SHDR;
      SHDR:  next_st = tms ? EX1DR : SHDR;
      EX1DR: next_st = tms ? UPDDR : PAUDR;
      PAUDR: next_st = tms ? EX2DR : PAUDR;
      EX2DR: next_st = tms ? UPDDR : SHDR;
      UPDDR: next_st = tms ? SELDR : RTI;
      SELIR: next_st = tms ? TLR   : CAPIR;
      CAPIR: next_st = tms ? EX1IR : SHIR;
      SHIR:  next_st = tms ? EX1IR : SHIR;
      EX1IR: next_st = tms ? UPDIR : PAUIR;
      PAUIR: next_st = tms ? EX2IR : PAUIR;
      EX2IR: next_st = tms ? UPDIR : SHIR;
      default: next_st = tms ? SELDR : RTI;   // UPDIR
    endcase
  endfunction

  // per-device actions
  localparam logic [2:0] A_NONE = 3'd0, A_RESET = 3'd1, A_CAPIR = 3'd2, A_CAPDR = 3'd3,
                         A_SHIFT = 3'd4, A_UPDIR = 3'd5, A_UPDDR = 3'd6;

  // shift register lengths
  localparam logic [2:0] L1 = 3'd0, L4 = 3'd1, L5 = 3'd2, L32 = 3'd3, L57 = 3'd4;

  function automatic logic is_asic(input logic [4:0] k);
    is_asic = (k <= 5'd3) || (k == 5'd5) || (k == 5'd6) || (k == 5'd10) || (k == 5'd12) || (k == 5'd13);
  endfunction
  // ID codes by step (Supermodel): {version, part 16C_7/3/4/5/6, 057}
  function automatic logic [31:0] idcode(input logic [4:0] k, input logic [1:0] js);
    logic [3:0] v;
    logic [3:0] part;
    case (k)
      5'd0:              begin part = 4'h7; v = (js == 2'd0) ? 4'h1 : (js == 2'd1) ? 4'h3 : (js == 2'd2) ? 4'h4 : 4'h5; end  // Jupiter
      5'd1:              begin part = 4'h3; v = (js == 2'd0) ? 4'h2 : (js == 2'd1) ? 4'h3 : 4'h4; end                         // Mercury
      5'd2:              begin part = 4'h4; v = (js == 2'd0) ? 4'h1 : (js == 2'd1) ? 4'h2 : 4'h3; end                         // Venus
      5'd3, 5'd10:       begin part = 4'h5; v = (js == 2'd0) ? 4'h2 : (js == 2'd1) ? 4'h3 : (js == 2'd2) ? 4'h4 : 4'h5; end  // Earth
      default:           begin part = 4'h6; v = (js == 2'd0) ? 4'h1 : (js == 2'd1) ? 4'h2 : 4'h3; end                         // Mars
    endcase
    idcode = {v, 12'h16C, part, 12'h057};
  endfunction

  // ------------------------------------------------------------ the devices
  // {sr[56:0], len[2:0], ir[4:0], mw[31:0]}
  localparam int DW = 57 + 3 + 5 + 32;
  (* ramstyle = "MLAB, no_rw_check" *) logic [DW-1:0] dev [0:31];
  wire  [4:0]  nlast = jstep[1] ? 5'd16 : 5'd9;   // the chain's last device

  tap_t        st;
  logic        last_tck, tdo_q;
  logic        run;                  // a pass over the devices
  logic [2:0]  act;
  logic [4:0]  rk;                   // the device being read
  logic        wv;                   // a read word is present (rk - 1)
  logic [4:0]  wk;
  logic        carry;                // the chain bit into device wk
  logic [DW-1:0] rq;
  logic        last_sr0;             // the last device's TDO (its sr[0]) as written

  assign busy = run;
  assign tdo  = tdo_q;

  always_ff @(posedge clk) rq <= dev[rk];

  // the new word for device wk, from its old word rq
  logic [56:0] sr_o, sr_n;
  logic [2:0]  ln_o, ln_n;
  logic [4:0]  ir_o, ir_n;
  logic [31:0] mw_o, mw_n;
  logic        sh_out;
  always_comb begin
    {sr_o, ln_o, ir_o, mw_o} = rq;
    sr_n = sr_o; ln_n = ln_o; ir_n = ir_o; mw_n = mw_o;
    sh_out = sr_o[0];
    case (act)
      A_RESET: if (is_asic(wk)) begin sr_n = {25'd0, idcode(wk, jstep)}; ln_n = L32; ir_n = 5'd3; end
               else begin sr_n = '0; ln_n = L1; ir_n = 5'd15; end
      A_CAPIR: if (is_asic(wk)) begin sr_n = 57'd1; ln_n = L5; end
               else begin sr_n = 57'd9; ln_n = L4; end
      A_CAPDR: begin
        sr_n = '0; ln_n = L1;                               // BYPASS
        if (is_asic(wk)) begin
          if (ir_o == 5'd3)       begin sr_n = {25'd0, idcode(wk, jstep)}; ln_n = L32; end
          else if (ir_o == 5'd8)  begin sr_n = {25'd0, mw_o}; ln_n = L32; end
          else if (ir_o == 5'd9 && wk == 5'd1) begin sr_n = {cfg0, cfg1, cfg2}; ln_n = L57; end
        end
      end
      A_SHIFT: begin
        sr_n = {1'b0, sr_o[56:1]};
        case (ln_o)
          L1:      sr_n[0]  = carry;
          L4:      sr_n[3]  = carry;
          L5:      sr_n[4]  = carry;
          L32:     sr_n[31] = carry;
          default: sr_n[56] = carry;
        endcase
      end
      A_UPDIR: ir_n = sr_o[4:0];
      A_UPDDR: if (is_asic(wk) && ir_o == 5'd8) mw_n = sr_o[31:0];
      default: ;
    endcase
  end

  wire do_write = run && wv;
  always_ff @(posedge clk) if (do_write) dev[wk] <= {sr_n, ln_n, ir_n, mw_n};

  // ------------------------------------------------------------ control
  always_ff @(posedge clk) begin
    cfg_we <= 1'b0; mw_we <= 1'b0;
    if (rst) begin
      st <= TLR; last_tck <= 1'b0; tdo_q <= 1'b0;
      run <= 1'b1; act <= A_RESET; rk <= 5'd0; wv <= 1'b0; carry <= 1'b0;   // power-on: reset every device
    end else if (run) begin
      // read device rk, write device rk - 1 (its word arrived this cycle)
      if (rk != nlast + 5'd1) rk <= rk + 5'd1;
      wv <= (rk != nlast + 5'd1);
      wk <= rk;
      if (wv) begin
        carry <= sh_out;
        if (wk == nlast) run <= 1'b0;     // (TDO is latched on the falling edge)
        if (act == A_UPDDR && is_asic(wk) && wk < 5'd10 && ir_o == 5'd8) begin
          mw_we <= 1'b1; mw_dev <= wk[3:0]; mw_val <= sr_o[31:0];
        end
        if (act == A_UPDDR && wk == 5'd1 && ir_o == 5'd9) begin
          cfg_we <= 1'b1; cfg0_w <= sr_o[56:38]; cfg1_w <= sr_o[37:19]; cfg2_w <= sr_o[18:0];
        end
      end
    end else if (wr) begin
      // Supermodel's CJTAG::Write
      if (!wdata[7]) begin
        act <= A_RESET; run <= 1'b1; rk <= 5'd0; wv <= 1'b0;
      end else if (wdata[6] != last_tck) begin
        last_tck <= wdata[6];
        if (wdata[6]) begin
          // rising edge: act in this state, then move
          case (st)
            TLR:         begin act <= A_RESET; run <= 1'b1; end
            CAPDR:       begin act <= A_CAPDR; run <= 1'b1; end
            CAPIR:       begin act <= A_CAPIR; run <= 1'b1; end
            SHDR, SHIR:  begin act <= A_SHIFT; run <= 1'b1; end
            default:     act <= A_NONE;
          endcase
          rk <= 5'd0; wv <= 1'b0; carry <= wdata[5];
          st <= next_st(st, wdata[2]);
        end else begin
          // falling edge
          case (st)
            SHDR, SHIR: tdo_q <= last_sr0;
            UPDDR:      begin act <= A_UPDDR; run <= 1'b1; end
            UPDIR:      begin act <= A_UPDIR; run <= 1'b1; end
            default:    ;
          endcase
          rk <= 5'd0; wv <= 1'b0;
        end
      end
    end
  end

  always_ff @(posedge clk) if (do_write && wk == nlast) last_sr0 <= sr_n[0];

endmodule
