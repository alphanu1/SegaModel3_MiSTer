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
//
// Each word carries one more bit, wn: the core writes !fpu_sgl_val(wd), so
// the fast port's route (ppc_fpu_pkg fpu_fast_route) reads it instead of
// testing 64 bits after the read. Inverted, so a word never written (zero)
// reads as a single value, as zero is. 65 bits still fit four MLABs.

module ppc_fpregfile (
  input  logic        clk,
  input  logic [4:0]  ra, rb, rc,
  output logic [63:0] qa, qb, qc,
  output logic        na, nb, nc,     // the words' wn bits
  input  logic        we,
  input  logic [4:0]  wa,
  input  logic [63:0] wd,
  input  logic        wn
);

  (* ramstyle = "MLAB, no_rw_check" *) logic [64:0] mem_a [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [64:0] mem_b [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [64:0] mem_c [0:31];

  always_ff @(posedge clk) begin
    if (we) begin
      mem_a[wa] <= {wn, wd};
      mem_b[wa] <= {wn, wd};
      mem_c[wa] <= {wn, wd};
    end
  end

  // The read addresses through a LUT of their own, so Quartus does not
  // copy the queue head's register into each array's address register (as
  // ppc_regfile.sv; R70's untried patch, sm3-builds/tq_patches/mlab_addr.py)
  (* keep = 1 *) wire [4:0] ra_k = ra;
  (* keep = 1 *) wire [4:0] rb_k = rb;
  (* keep = 1 *) wire [4:0] rc_k = rc;
  assign {na, qa} = mem_a[ra_k];
  assign {nb, qb} = mem_b[rb_k];
  assign {nc, qc} = mem_c[rc_k];

endmodule
