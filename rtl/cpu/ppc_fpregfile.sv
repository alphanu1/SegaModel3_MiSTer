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
// PowerPC FPR file: 32 x 64 in MLAB, three read ports, one write port, as
// ppc_regfile (study 8.4: MLAB from the start, a copy per read port, whole
// words written). A copy is 64 bits wide in four 32 x 20 MLABs.
//
// Read ports: A (frA), B (frB) and C (frC, or frS for a store: the core
// predecodes which). One write port is enough because the core schedules its
// writers so that no two land in one cycle (ppc_core.sv, "FPR writes").
//
// The read is asynchronous and the core registers the data as the uop enters
// EX. A register written on the same edge reads its old value; the core
// forwards that write itself.

module ppc_fpregfile (
  input  logic        clk,
  input  logic [4:0]  ra, rb, rc,
  output logic [63:0] qa, qb, qc,
  input  logic        we,
  input  logic [4:0]  wa,
  input  logic [63:0] wd
);

  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] mem_a [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] mem_b [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] mem_c [0:31];

  always_ff @(posedge clk) begin
    if (we) begin
      mem_a[wa] <= wd;
      mem_b[wa] <= wd;
      mem_c[wa] <= wd;
    end
  end

  assign qa = mem_a[ra];
  assign qb = mem_b[rb];
  assign qc = mem_c[rc];

endmodule
