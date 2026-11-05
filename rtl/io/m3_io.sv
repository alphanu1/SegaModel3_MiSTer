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
// The Sega 315-5649 I/O chip at F0040000 and the 93C46 EEPROM on its port A
// (study 5.6, D-37: stage A has no cabinet inputs, so every input reads idle).
// Read from my Model 2 core's m2_315_5649 (47edd9dc, the same chip on Model
// 2A); where Model 3 differs:
//   - the bus: MAME maps it with umask64(0xff000000ff000000), so device
//     offset n is the byte at F0040000 + 4n, the high byte (bits 31:24) of
//     that word. The bus hands this module that byte and its offset;
//   - port B is MAME's model3 input_r: PA bit 0 selects bank 1 (IN1: test B
//     bit 7, service B bit 6, the EEPROM's DO in bit 5, bits 4:0 unused and
//     high) or bank 0 (IN0: coins, test A, service A, starts);
//   - port G is the DIP bank. MAME's default is 0xFF; Supermodel returns 0x7F
//     (its comment: a side effect on Ocean Hunter otherwise). The bench's
//     oracle is a Supermodel trace, so the parameter's default is 0x7F
//     (sim/system/notes.md);
//   - the analog channels read 0x00, as Supermodel gives VF3 (the trace's
//     F004003C reads); MAME has the AN ports at their defaults;
//   - no gun board, no lamps or billboard outputs (written, not driven).
// Port semantics (MAME sega/315_5649.cpp): offsets 0-6 ports A-G, a read is
// the input when the direction bit is 1, else the latch; 8 direction (0xFF
// at reset); 0xB 0x00, 0xD 0x0C (MAME's serial hack); 0xE mode; 0xF analog:
// a read returns the channel and steps it, a write selects it. Other reads
// 0xFF. Direction and mode reset with the game; the latches, the analog
// channel and the EEPROM only at power-on.

module m3_io #(
  parameter logic [7:0] DSW = 8'h7F,
  parameter int         EE_TWP = 200000,  // the EEPROM's busy time after a write, clocks (m3_93c46)
  parameter bit         EE_LOAD = 1'b0    // the 93C46's load port (the MRA's <nvram>, study R100)
) (
  input  logic        clk,
  input  logic        por_n,
  input  logic        rst_n,
  input  logic        sel,          // one cycle per access
  input  logic        we,
  input  logic [3:0]  off,          // addr[5:2]
  input  logic [7:0]  wbyte,
  output logic [7:0]  rbyte,
  input  logic [7:0]  in0,          // bank 0: coins, test A, service A, starts (active low)
  input  logic        test_b_n,
  input  logic        service_b_n,
  input  logic [7:0]  in2,          // player 1
  input  logic [7:0]  in3,          // player 2
  output logic [7:0]  pa_q,         // port A's latch (the overlay)
  input  logic        ee_ld_we,     // the 93C46's load port (EE_LOAD), the game in reset
  input  logic [5:0]  ee_ld_a,
  input  logic [15:0] ee_ld_d
);

  logic [7:0] pv [8];
  logic [7:0] dir;
  logic [2:0] an_ch;

  wire ee_do;
  m3_93c46 #(.T_WP(EE_TWP), .LOAD(EE_LOAD)) u_ee (
    .clk, .rst_n(por_n),
    .cs(pv[0][6]), .sk(pv[0][7]), .di(pv[0][5]), .do_o(ee_do),
    .ld_we(ee_ld_we), .ld_a(ee_ld_a), .ld_d(ee_ld_d), .dump_a(6'd0), .dump_q()
  );

  wire [7:0] pb_in = pv[0][0] ? {test_b_n, service_b_n, ee_do, 5'b11111} : in0;

  // the inputs by port (a mux, not a function: Quartus 17 reported pb_in "never
  // read" when a function read it from module scope)
  logic [7:0] pin;
  always_comb begin
    case (off[2:0])
      3'd1:    pin = pb_in;
      3'd2:    pin = in2;
      3'd3:    pin = in3;
      3'd6:    pin = DSW;
      default: pin = 8'hff;
    endcase
    if (off <= 4'd6) rbyte = dir[off[2:0]] ? pin : pv[off[2:0]];
    else case (off)
      4'hb:    rbyte = 8'h00;
      4'hd:    rbyte = 8'h0c;
      4'hf:    rbyte = 8'h00;
      default: rbyte = 8'hff;
    endcase
  end

  always_ff @(posedge clk or negedge por_n) begin
    if (!por_n) begin
      for (int i = 0; i < 8; i++) pv[i] <= 8'hff;
      an_ch <= 3'd0;
    end else if (sel) begin
      if (we && off <= 4'd6) pv[off[2:0]] <= wbyte;
      if (off == 4'hf) an_ch <= we ? wbyte[2:0] : an_ch + 3'd1;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) dir <= 8'hff;
    else if (sel && we && off == 4'h8) dir <= wbyte;
  end

  assign pa_q = pv[0];

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, an_ch};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
