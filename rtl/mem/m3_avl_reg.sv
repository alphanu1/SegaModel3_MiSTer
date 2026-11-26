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
// sys140 (D-81): one register stage on the DDR3 port's command side (Avalon-MM: RD / WE / ADDR / BURSTCNT / DIN /
// BE, held while BUSY), between m3_ddr3p and the framework's DDRAM_* (the HPS's FPGA-to-SDRAM port, a hard block
// in the die's corner). At 130 MHz the paths into that block were combinational from the requesters (the
// rasteriser's band FIFO through m3_ddr_arb and m3_ddr3p's beat select into DDRAM_DIN / BE: -2.9 ns, and
// -2.5 ns from the band's MLAB into the f2sdram registers); with this stage they end at registers the fitter may
// place where it likes, and the stage drives the block with nothing but route.
//
// The upstream side sees BUSY from a register (the skid slot's valid), so m3_ddr3p's own take logic no longer
// waits on the hard block's waitrequest either. A transfer upstream goes into the output register when that is
// free (or being taken), else into the skid slot, which then holds the upstream off until the output register
// takes it: one transfer a clock while the block is not busy, every transfer in order, none lost or doubled.
// Cost: each command reaches the block one clock later (a read's first beat one clock later); the read data
// (DOUT, DOUT_READY) does not pass through here.

module m3_avl_reg (
  input  logic        clk,
  input  logic        rst_n,
  // m3_ddr3p's side
  input  logic        u_rd,
  input  logic        u_we,
  input  logic [28:0] u_addr,
  input  logic [7:0]  u_burstcnt,
  input  logic [63:0] u_din,
  input  logic [7:0]  u_be,
  output logic        u_busy,
  // the framework's DDRAM port
  output logic        DDRAM_RD,
  output logic        DDRAM_WE,
  output logic [28:0] DDRAM_ADDR,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  input  logic        DDRAM_BUSY
);
  localparam int W = 29 + 8 + 64 + 8;
  logic [1:0]    o_c, s_c;                         // {RD, WE} of the output register and of the skid slot
  logic          s_v;
  logic [W-1:0]  o_d, s_d;
  wire  [W-1:0]  u_d   = {u_addr, u_burstcnt, u_din, u_be};
  wire           u_go  = (u_rd || u_we) && !s_v;    // a transfer upstream (its BUSY is s_v)
  wire           o_fr  = (o_c == 2'b00) || !DDRAM_BUSY;   // the output register is free at the edge
  assign u_busy = s_v;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      o_c <= 2'b00; s_c <= 2'b00; s_v <= 1'b0;
    end else if (o_fr) begin
      if (s_v)       begin o_c <= s_c; s_v <= 1'b0; end
      else if (u_go) o_c <= {u_rd, u_we};
      else           o_c <= 2'b00;
    end else if (u_go) begin s_v <= 1'b1; s_c <= {u_rd, u_we}; end
  end
  // the fields without a reset (RD / WE qualify them)
  always_ff @(posedge clk) begin
    if (o_fr) o_d <= s_v ? s_d : u_d;
    if (!o_fr && u_go) s_d <= u_d;
  end
  assign {DDRAM_RD, DDRAM_WE} = o_c;
  assign {DDRAM_ADDR, DDRAM_BURSTCNT, DDRAM_DIN, DDRAM_BE} = o_d;

`ifdef VERILATOR
  always_ff @(posedge clk) if (rst_n) begin
    if (u_rd && u_we) $fatal(1, "m3_avl_reg: RD and WE together");
  end
`endif
endmodule
