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
// The fetch queue's storage: four entries of instruction, PC and predicted bit,
// in flip-flops, kept in order with the head always in entry 0. Decode reads
// entry 0 directly, with no read mux (R23 timing); a pop shifts the entries
// down. Quartus made a RAM block of the earlier indexed form (from an indexed
// array, a packed array and four named registers alike), so RAM recognition is
// also switched off for this module alone (the core's MLAB files keep it).

(* altera_attribute = "-name AUTO_RAM_RECOGNITION OFF" *)
module ppc_fetchq #(
  parameter int PDW = 5                  // predecode bits per entry
)(
  input  logic        clk,
  input  logic        clear,         // a hard redirect: the queue empties
  input  logic        we,            // push, after the live entries
  input  logic [31:0] w_insn,
  input  logic [31:0] w_pc,
  input  logic        w_pred,
  input  logic [PDW-1:0] w_pd,       // predecode, carried with the word (ppc_core.sv)
  input  logic [3:0]  pop,           // decode took the head: one copy per entry (fanout)
  output logic [2:0]  count,
  output logic [31:0] r_insn,
  output logic [31:0] r_pc,
  output logic        r_pred,
  output logic [PDW-1:0] r_pd,
  output logic [31:0] r1_insn,       // entry 1: the head after a pop
  output logic [PDW-1:0] r1_pd
);

  logic [31:0] qi [0:3];
  logic [31:0] qp [0:3];
  logic [3:0]  qd;
  logic [PDW-1:0] qx [0:3];
  logic [2:0]  cnt;

  // After the pop, entry k holds what was in k+1; the push goes to cnt - pop.
  // Each entry's next value is formed for both cases from the count and the
  // push alone, and the pop, the decode stage's late decision, picks between
  // them as the last mux before the register (R23 timing: the pop went through
  // the write position and a clock enable on every bit).
  always_ff @(posedge clk) begin
    if (clear) cnt <= 3'd0;
    else       cnt <= cnt + {2'd0, we} - {2'd0, pop[0]};
    for (int k = 0; k < 4; k++) begin
      if (pop[k]) begin
        if (we && cnt == 3'(k + 1)) begin
          qi[k] <= w_insn;
          qp[k] <= w_pc;
          qd[k] <= w_pred;
          qx[k] <= w_pd;
        end else if (k < 3) begin
          qi[k] <= qi[(k + 1) % 4];
          qp[k] <= qp[(k + 1) % 4];
          qd[k] <= qd[(k + 1) % 4];
          qx[k] <= qx[(k + 1) % 4];
        end
      end else if (we && cnt == 3'(k)) begin
        qi[k] <= w_insn;
        qp[k] <= w_pc;
        qd[k] <= w_pred;
        qx[k] <= w_pd;
      end
    end
  end

  assign count  = cnt;
  assign r_insn = qi[0];
  assign r_pc   = qp[0];
  assign r_pred = qd[0];
  assign r_pd   = qx[0];
  assign r1_insn = qi[1];
  assign r1_pd   = qx[1];

endmodule
