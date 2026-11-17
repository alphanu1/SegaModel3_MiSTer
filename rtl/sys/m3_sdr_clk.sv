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
// An SDRAM clock pin from a DDIO output register (study R187), after
// MemTest256's rtl/sdram.v: the register leaves from the pin's own I/O cell,
// as the controller's address, command and DQ registers do (sys.tcl /
// sys_dual_sdram.tcl's FAST_OUTPUT_REGISTER), so the clock and what it samples
// take the same kind of path to the pads, one the fit cannot move.
//
// INV 1 is MemTest256's form: clocked by the controller's own clock with 0 in
// the high half and 1 in the low half, the pin is that clock inverted (180
// degrees at any frequency). INV 0 passes `clk` through (1 high, 0 low): the
// build clocks it from the PLL's outclk_2, set to lead clk_sys's 180 degrees
// by 0.5 ns, the lead c125dp35_s1's 3500 ps gave at 125 MHz and the board's
// dual SDRAM needs (R184, R187: at plain 180 degrees the second SDRAM's read
// sits where 4000 ps put it, which failed).
//
// Before (R184 and earlier) SDRAM_CLK and SDRAM2_CLK were outclk_2 itself,
// routed to the pads on the clock network and general routing; that path was
// not constrained or reported, and the two pins arrived 1.75 ns apart at slow
// 100 C (c120d_s1), so one phase could not suit both. Model3.sdc puts a
// generated clock on each pin and, for TimeQuest, the SDRAMs' I/O delays.
// ---------------------------------------------------------------------------

module m3_sdr_clk #(
  parameter bit INV = 1'b0   // 1: the pin is clk inverted; 0: the pin is clk
) (
  input  logic clk,   // the clock to put on the pin (outclk_2), or the controller's own with INV
  output logic pin    // SDRAM_CLK / SDRAM2_CLK
);
`ifdef VERILATOR
  assign pin = INV ? ~clk : clk;
`else
  altddio_out #(
    .extend_oe_disable("OFF"),
    .intended_device_family("Cyclone V"),
    .invert_output("OFF"),
    .lpm_hint("UNUSED"),
    .lpm_type("altddio_out"),
    .oe_reg("UNREGISTERED"),
    .power_up_high("OFF"),
    .width(1)
  ) u_ddio (
    .datain_h(!INV),
    .datain_l(INV),
    .outclock(clk),
    .dataout(pin),
    .aclr(1'b0),
    .aset(1'b0),
    .oe(1'b1),
    .outclocken(1'b1),
    .sclr(1'b0),
    .sset(1'b0)
  );
`endif
endmodule
