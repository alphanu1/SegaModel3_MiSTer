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
// The fetch queue's storage: four entries of instruction, PC, predicted bit
// and predecode, in flip-flops, in order with the head always in entry 0.
// Decode reads entry 0 directly, with no read mux (R23 timing), and the early
// hazard logic reads entry 1, the head after a pop. Entries 0 and 1 are
// registers that shift on a pop; entries 2 and 3 are a two-slot ring (rp:
// the slot holding entry 2) that never moves on a pop, so the pop, decode's
// late decision, drives entries 0 and 1 and one pointer bit instead of all
// four entries (R63: the pop's fanout led the full builds' worst paths).
// Quartus made a RAM block of an earlier indexed form, so RAM recognition is
// switched off for this module alone (the core's MLAB files keep it).
//
// F2 = 1 (two-word fetch, D-30): a second word (we2, w2_*) may be pushed
// behind the first, into the entry after it. The core asks for two words
// only when the queue holds at most two at the answer, so a pair lands at
// counts 0, 1 or 2. The ring keeps its rule that its slots are written by the
// count alone: at count 1 the second word goes to slot rp (entry 2 unless a
// pop moves it to entry 1, which h1 takes), at count 2 the pair fills both
// slots, and a pop there turns the ring (rp) so the second word is entry 2.

(* altera_attribute = "-name AUTO_RAM_RECOGNITION OFF" *)
module ppc_fetchq #(
  parameter int PDW = 5,                 // predecode bits per entry
  parameter bit F2  = 1'b0               // a second word per push (above)
)(
  input  logic        clk,
  input  logic        clear,         // a hard redirect: the queue empties
  input  logic        we,            // push, after the live entries
  input  logic [31:0] w_insn,
  input  logic [31:0] w_pc,
  input  logic        w_pred,
  input  logic [PDW-1:0] w_pd,       // predecode, carried with the word (ppc_core.sv)
  input  logic        we2,           // F2: the next word too, behind it
  input  logic [31:0] w2_insn,
  input  logic [31:0] w2_pc,
  input  logic        w2_pred,
  input  logic [PDW-1:0] w2_pd,
  input  logic [3:0]  pop,           // decode took the head: a copy for each half of entries 0, 1
  output logic [2:0]  count,
  output logic [31:0] r_insn,
  output logic [31:0] r_pc,
  output logic        r_pred,
  output logic [PDW-1:0] r_pd,
  output logic [31:0] r1_insn,       // entry 1: the head after a pop
  output logic [PDW-1:0] r1_pd,
  output logic [PDW-1:0] r2_pd,      // entries 2 and 3's predecode (the CR look-ahead's scan)
  output logic [PDW-1:0] r3_pd
);

  // entries 0 and 1: {insn, pc, pred, predecode}; the ring's two slots
  localparam int EW = 65 + PDW;
  logic [EW-1:0] h0, h1;
  logic [EW-1:0] rg [0:1];
  logic          rp;
  logic [2:0]    cnt;
  wire  [EW-1:0] w_e = {w_insn, w_pc, w_pred, w_pd};
  wire  [EW-1:0] w2e = {w2_insn, w2_pc, w2_pred, w2_pd};
  wire           p2  = F2 && we2;
  wire  [EW-1:0] r2  = rg[rp];                         // entry 2

  // Each entry's next value for both cases, the pop choosing last (R23
  // timing). The push goes to entry cnt - pop: entry 0 or 1 is chosen with
  // the pop; the ring's slots are written by count alone (slot rp when cnt =
  // 2, which is free if the pop sends the word to entry 1 instead; the other
  // slot when cnt = 3, entry 3 without a pop or entry 2 after one, where rp
  // moves), so the pop is not in their enables. A push at cnt = 4 does not
  // happen (the core issues a fetch only into a free slot).
  //
  // The arriving word is written into the entry the push would take whether
  // or not there is a push: that entry is free, and stays unused without
  // one. So the push (from i_valid and the redirect, late) is only in the
  // count, not in the entries' selects. Those come from a one-hot copy of
  // the count, o[k] = (cnt == k), registered.
  logic [3:0]    o;
  // (the count after the push for both values of the pop, the pop choosing
  // last: it is decode's late decision, and the one-hot copy below is behind it)
  wire  [2:0]    cnt_a = cnt + {2'd0, we} + {2'd0, p2};
  wire  [2:0]    cnt_b = cnt_a - 3'd1;
  wire  [2:0]    cnt_n = clear ? 3'd0 : pop[0] ? cnt_b : cnt_a;
  always_ff @(posedge clk) begin
    cnt <= cnt_n;
    o   <= clear ? 4'b0001 : pop[0] ? (4'b0001 << cnt_b[1:0] & {4{!cnt_b[2]}}) : (4'b0001 << cnt_a[1:0] & {4{!cnt_a[2]}});
    if (pop[0] && ((!o[0] && !o[1] && !o[2]) || (o[2] && p2))) rp <= !rp;  // cnt >= 3, or a pair at 2
    if (F2 && o[1]) rg[rp] <= w2e;
    if (o[2]) rg[rp]  <= w_e;
    if (F2 && o[2]) rg[!rp] <= w2e;
    if (o[3]) rg[!rp] <= w_e;
  end
  localparam int HW = EW / 2;
  always_ff @(posedge clk) begin
    if (pop[0])     h0[EW-1:HW] <= o[1] ? w_e[EW-1:HW] : h1[EW-1:HW];
    else if (o[0])  h0[EW-1:HW] <= w_e[EW-1:HW];
    if (pop[1])     h0[HW-1:0]  <= o[1] ? w_e[HW-1:0]  : h1[HW-1:0];
    else if (o[0])  h0[HW-1:0]  <= w_e[HW-1:0];
    if (pop[2])     h1[EW-1:HW] <= o[2] ? w_e[EW-1:HW] : (F2 && o[1]) ? w2e[EW-1:HW] : r2[EW-1:HW];
    else if (o[1])  h1[EW-1:HW] <= w_e[EW-1:HW];
    else if (F2 && o[0]) h1[EW-1:HW] <= w2e[EW-1:HW];
    if (pop[3])     h1[HW-1:0]  <= o[2] ? w_e[HW-1:0]  : (F2 && o[1]) ? w2e[HW-1:0] : r2[HW-1:0];
    else if (o[1])  h1[HW-1:0]  <= w_e[HW-1:0];
    else if (F2 && o[0]) h1[HW-1:0] <= w2e[HW-1:0];
  end
`ifdef VERILATOR
  logic [1:0] cleared;                                 // (the copies agree from the first clear on;
  always @(posedge clk) begin                          //  a simulation flag, starts at 0)
    if (clear) cleared <= 2'b11;
    if (cleared == 2'b11 && !clear && o != (cnt[2] ? 4'd0 : 4'b0001 << cnt[1:0])) $fatal(1, "fetch queue count copy");
    if (!clear && p2 && (cnt > 3'd2 || !we)) $fatal(1, "fetch queue: a second word at count %0d", cnt);
    if (!clear && cnt_n > 3'd4) $fatal(1, "fetch queue overflow");
  end
`endif

  assign count  = cnt;
  assign {r_insn, r_pc, r_pred, r_pd} = h0;
  assign r1_insn = h1[EW-1 -: 32];
  assign r1_pd   = h1[PDW-1:0];
  assign r2_pd   = r2[PDW-1:0];
  assign r3_pd   = rg[!rp][PDW-1:0];

endmodule
