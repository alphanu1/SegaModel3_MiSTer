// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// SIMULATION ONLY. A drop-in for rtl/cpu/ppc_fpu_lzc.sv in the system bench:
// the same ports, parameters and timing (REG: the count a cycle later), as a
// scan with an early exit. Verilator turns the RTL's tree of arrays into
// loops that cleared and rebuilt 256-entry arrays on every evaluation, more
// than half of the whole-system bench's time (gprof, sim/system/notes.md).
// sim/system/tb_lzc_equiv.cpp checks the two against each other.

module ppc_fpu_lzc #(
  parameter int W   = 162,
  parameter int LW  = $clog2(W + 1),
  parameter bit REG = 1'b0,
  parameter int CUT = 4
) (
  input  logic          clk,
  input  logic [W-1:0]  x,
  output logic [LW-1:0] n
);
  logic [W-1:0] xs;
  generate if (REG) begin : g_reg
    always_ff @(posedge clk) xs <= x;
  end else begin : g_comb
    assign xs = x;
  end endgenerate

  always_comb begin
    n = LW'(W);
    for (int i = W - 1; i >= 0; i--)
      if (xs[i]) begin n = LW'(W - 1 - i); break; end
  end

  /* verilator lint_off UNUSEDPARAM */
  localparam int unused_cut = CUT;
  /* verilator lint_on UNUSEDPARAM */
endmodule
