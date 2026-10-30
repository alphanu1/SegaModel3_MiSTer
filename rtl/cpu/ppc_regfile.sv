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
// PowerPC GPR file: 32 x 32 in MLAB, three read ports, one write port.
//
// Study 8.4: the register file goes in MLAB from the start, never flip-flops
// (Model 1 7d95cdd: -796 ALM on the V60), with whole-word writes only (957fba3:
// a per-bit write mask costs about 1,030 ALM). One write port is enough
// because the decoder cracks every two-result instruction (ppc_decode.sv).
//
// Three copies, written alike, one per read port (rA, rB, rS). Read addresses
// are registered when the instruction moves from decode to execute and the
// arrays are read asynchronously from them, so a stalled execute stage keeps
// seeing writes as they land. A write on the same edge as the address register
// loads is not relied on: the core forwards that value itself (ppc_core.sv,
// the wb2 path), so the arrays can be built without read-during-write logic.

module ppc_regfile (
  input  logic        clk,
  input  logic        ren,          // load the read addresses
  input  logic [4:0]  ra, rb, rs,
  output logic [31:0] qa, qb, qs,
  input  logic        we,
  input  logic [4:0]  wa,
  input  logic [31:0] wd
);

  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_a [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_b [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] mem_s [0:31];

  logic [4:0] ra_q, rb_q, rs_q;

  always_ff @(posedge clk) begin
    if (we) begin
      mem_a[wa] <= wd;
      mem_b[wa] <= wd;
      mem_s[wa] <= wd;
    end
    if (ren) begin
      ra_q <= ra;
      rb_q <= rb;
      rs_q <= rs;
    end
  end

`ifdef VERILATOR
  // Simulation of the MLAB's undefined case. When a read address loads on the
  // same edge that writes that register, Verilator's array already holds the
  // new value; the MLAB built without read-during-write logic does not promise
  // it. Return the old value for that cycle instead, so a missing forward in
  // the core fails the bench rather than passing it (a deleted wb2 path passed
  // 3 million compared instructions before this was added).
  logic        col_a, col_b, col_s;
  logic [31:0] old_a, old_b, old_s;
  always_ff @(posedge clk) begin
    col_a <= ren && we && wa == ra;  old_a <= mem_a[ra];
    col_b <= ren && we && wa == rb;  old_b <= mem_b[rb];
    col_s <= ren && we && wa == rs;  old_s <= mem_s[rs];
  end
  assign qa = col_a ? old_a : mem_a[ra_q];
  assign qb = col_b ? old_b : mem_b[rb_q];
  assign qs = col_s ? old_s : mem_s[rs_q];
`else
  assign qa = mem_a[ra_q];
  assign qb = mem_b[rb_q];
  assign qs = mem_s[rs_q];
`endif

endmodule
