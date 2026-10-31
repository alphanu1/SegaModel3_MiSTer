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
// PowerPC GPR file: 32 x 32 in MLAB, three read ports, one write port, and
// the 603e's four shadow registers TGPR0-3 (603e 5.5.2.1, MSR[TGPR]) as
// entries 32-35. The core forms the sixth address bit: at fetch for the read
// ports (predecode, ppc_core.sv), so no logic is added in front of the read,
// and from MSR[TGPR] for the write. Each copy is 64 deep (MLAB 64 x 10).
//
// Study 8.4: the register file goes in MLAB from the start, never flip-flops
// (Model 1 7d95cdd: -796 ALM on the V60), with whole-word writes only (957fba3:
// a per-bit write mask costs about 1,030 ALM). One write port is enough
// because the decoder cracks every two-result instruction (ppc_decode.sv).
//
// Three copies, written alike, one per read port (rA, rB, rS). The read is
// asynchronous, addressed by the instruction in decode, and the core registers
// the data as the instruction enters EX: the MLAB read sits in ID, not on EX's
// path (R23 timing). The core writes a register as its producer leaves MEM. A
// register written on the same edge the data is registered reads its old
// value; the core forwards that write itself (from MEM), so the arrays need no
// read-during-write logic.

module ppc_regfile (
  input  logic        clk,
  input  logic [5:0]  ra, rb, rs,
  output logic [31:0] qa, qb, qs,
  input  logic        we,
  input  logic [5:0]  wa,
  input  logic [31:0] wd
);

  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_a [0:63];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_b [0:63];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_s [0:63];

  always_ff @(posedge clk) begin
    if (we) begin
      mem_a[wa] <= wd;
      mem_b[wa] <= wd;
      mem_s[wa] <= wd;
    end
  end

  // Before the edge the arrays hold the old value, so a same-edge write is
  // seen only through the core's forward, in simulation as on the MLAB.
  assign qa = mem_a[ra];
  assign qb = mem_b[rb];
  assign qs = mem_s[rs];

endmodule
