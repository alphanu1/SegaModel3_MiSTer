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
// Branch target buffer for the 603e core's fetch (study R52, D-24). The 603e
// has no BTB: it folds branches out of a two-word-a-cycle fetch into a six-
// entry queue (603e manual 6.4.1.1), which hides the redirect. This core
// fetches one word a cycle and pays 2 cycles per predicted-taken branch
// (R42, R48); the BTB is its way to the same effect. Nothing it holds is
// architectural: every prediction is checked by the fetch stage or EX.
//
// Table: N entries (256, 128 or 64), direct mapped by PC[IW+1:2], one entry
// per line of the RAM:
//   v | tag PC[31:IW+2] | ctx MSR[IR, PR] | reg (bclr / bcctr) | target[31:2] | x
// x is XW bits the core may carry with the target (design C below: the
// target's physical line and I-cache way). 56 + XW bits at 256 entries.
// STYLE 0 puts it in M10K (two blocks up to 80 bits, any N <= 256), STYLE 1
// in MLAB. The read address is registered (in the block for M10K) and the
// output is not: the entry comes out in the cycle after rd_pc is presented.
//
// Timing, by cycle:
//   k     rd_pc is the address whose entry is read at the edge ending k.
//         The core gives f_pc's next value: the address the coming cycle's
//         sequential request will carry.
//   k+1   the entry is out and compared with rd_pc (registered here) and with
//         ctx. cap says the request going out in k+1 is that address (an
//         f_pc request, not a redirect). A hit with cap is captured at the
//         edge ending k+1.
//   k+2.. p_v, p_tgt, p_reg, p_x hold the prediction until the core takes it
//         (p_take) or drops it (kill). While p_v is held nothing else is
//         captured, so the request after a predicted one is never looked up.
// A write (allocate with w_v = 1, invalidate with w_v = 0) goes into the
// table at the edge it is presented. A lookup whose read falls on the edge
// of a write to the same entry is suppressed (the M10K's mixed-port result
// is undefined), so a prediction never comes from a half-written entry.
// rst and flush empty the table by a sweep, one entry a cycle (N cycles);
// while it runs (busy) nothing is predicted and writes are dropped. The
// flush is needed only where an entry carries a physical address (design C).
//
// Context (603e 2.3.1, 5.1): the tag includes MSR[IR] and MSR[PR] of the
// fetch, so real-mode and translated code, or user and supervisor, never
// share a prediction; every MSR change already redirects. Translation
// changes (mtsr, mtsrin, tlbie, tlbld, tlbli, IBAT writes) change what an
// effective address fetches; for an entry holding only effective addresses
// (design D) the fetch is translated as normal and the stale prediction is
// caught by the check below, so no flush is needed. isync, rfi and sc need
// nothing: they redirect, and their target is fetched from its own address.
//
// Policy (the core's, measured in tools/mame-census lv2_ and sim/btb):
// written for a taken branch that the static rule predicts taken (b; bc by
// the 603e's y-bit rule, 6.4.1.2) and for a taken bclr or bcctr. A hit
// predicts taken to the stored target. The static rule still decides a bc
// it predicts not taken, so a hit never adds a misprediction for b or bc;
// a hit on a bclr / bcctr that is not taken is a misprediction (EX) and
// invalidates the entry, and a wrong register target is replaced.
//
// ---------------------------------------------------------------------------
// Interface in the core (not connected yet; ppc_core.sv is unchanged)
//
//   rd_pc   = f_pc's D input: f_issue ? fetch_pc + 4 : redir_q ? redir_pc_q :
//             s_ok ? f_rsp_pc : f_pc   (bits 31:2)
//   ctx     = {f_ir, f_pr}
//   cap     = f_issue && !redir_q && !bt_q   (the request is f_pc's own)
//   kill    = redirect || f_soft || s_ok || s_flt   (any change of stream)
//   p_take  = the predicted request goes out
//
// Design D (lookup at the branch's own request; R52 lever "b256" less the
// cases below). Measured with lv2_bD: a hit costs 1 cycle against today's 2,
// because the I-cache answers a non-sequential request in 2 cycles.
//   1. fetch_pc gains p_tgt as a fifth source: redir_q ? redir_pc_q : p_v ?
//      p_tgt : f_pc. The IBAT compares gain p_tgt as a fifth candidate
//      (fc_pc), all from registers. i_seq is 0 for it. p_take = f_issue && p_v
//      && !redir_q.
//   2. A flag goes with the outstanding request (bt_q: "the request after
//      this one went to p_tgt"). When the branch's word arrives, f_soft is
//      suppressed if the word predicts taken to that target; the queue entry
//      gets pred = 1. The check (pw_pred, pw_tgt == target, or a bclr/bcctr
//      for an entry with reg) is registered and acted on the next cycle: on a
//      mismatch the response after the branch is discarded and fetch soft-
//      redirects to pw_pred ? pw_tgt : f_rsp_pc + 4 (a third soft source),
//      and the entry is invalidated. This keeps the 30-bit compare off the
//      path from i_data into the queue (R28, R41).
//   3. A bclr / bcctr predicted by a hit carries its target to EX (30 bits a
//      queue entry, ID, EX) and MEM compares it with the computed target, a
//      cycle after EX's taken decision, redirecting from MEM (the replay's
//      path) on a difference; EX's taken-versus-predicted check is as today.
//   4. Allocation: on a soft redirect (static taken), from the registered
//      branch PC and redir_s_q; on an EX redirect for a taken branch the
//      static rule predicts taken or a bclr / bcctr, from EX's registered PC
//      and redir_a_q; invalidate on the corrections in 2 and 3. One write
//      port: EX's write wins.
//   5. No flush; ctx from f_ir / f_pr.
// Rule broken by a redirect: the first request after any redirect comes from
// redir_pc_q, not f_pc, so a branch at a redirect target is not looked up.
//
// Design C (a lookup a request ahead; R52's "free on a hit"): the entry for a
// branch at B is written at B - 4 and is looked up when B - 4 goes out, so
// the core knows a whole request ahead that the one after B is the target.
// Needs, beyond D's items 2-4:
//   6. XW = 22: the target's physical line PA[31:12] and the I-cache way it
//      was fetched from (a new i_way output of ppc_icache, the way that
//      answered), written when the target's first word arrives.
//   7. ppc_icache: a predicted-next port presented with B's request
//      (registers: index T[11:2], way, PA tag). The cache reads T's data and
//      tags at that edge instead of its lookahead, and answers T's request
//      (flagged) in one cycle when tag[way] = PA and ICE; else T is an
//      ordinary request (2-3 cycles). kl / nx move to T's line, so T + 4 is
//      sequential. This is the cache owner's change.
//   8. flush on every translation change (the same events that empty the
//      instruction micro-TLB, iu_clr), because the cache trusts the PA.
//   Both B - 4 and B must go out as f_pc requests, so the first two words
//   after a redirect cannot be predicted.
//
// Measured (sim/btb on census traces, frames 6000-6299; tools/mame-census
// lv2_ / lv3_ over 18,900 frames a title at 100 MHz, R52's base):
//   - the CPI model's own 256-entry BTB hits 89.07% / 72.96% / 75.14% of taken
//     branches (VF3 / Daytona 2 / Bass Fishing); the RTL gives the same count
//     branch for branch on all three traces;
//   - the policy above: design D right on 64.8% / 44.6% / 35.4% of taken
//     branches, design C on 49.0% / 40.8% / 34.2% (C misses the first two
//     words after a redirect: VF3's short loops); with the direction from
//     the entry (written for every taken branch, invalidated on a hit not
//     taken) 76.1% / 62.6% / 70.6% (D), 60.1% / 57.1% / 67.6% (C);
//   - CPI (VF3): none 1.980, R52's free BTB 1.836, C 1.873, D 1.915; 128 and
//     64 entries add 0.003 and 0.007, so 64 entries keep the hit rate.
//
// Cost, standalone fit (quartus/btb_spike, I/O registered at the top; slow
// 100 C model, BTB logic alone from the entity table):
//   256 x 56, M10K  2 M10K, 47 ALM, 79 regs  177.3 / 189.3 / 197.7 MHz (8.333 ns),
//                                            176.8 / 177.5 / 180.8 MHz (10 ns)
//   128 or 64, M10K 2 M10K, 48 ALM (the entry width, not the depth, sets the blocks)
//   256 x 78, M10K  2 M10K, 62 ALM (design C, XW = 22)        185.7 MHz
//   64, MLAB        0 M10K, 107 ALM                          179.7 MHz
//   128, MLAB       0 M10K, 205 ALM                          180.7 MHz
//   256, MLAB       0 M10K, 388 ALM                          145.0 MHz

module ppc_btb #(
  parameter int N     = 256,                 // entries: 256, 128 or 64
  parameter int XW    = 0,                   // extra payload bits
  parameter int STYLE = 0                    // 0 M10K, 1 MLAB
)(
  input  logic          clk,
  input  logic          rst,

  input  logic [31:2]   rd_pc,
  input  logic [1:0]    ctx,
  input  logic          cap,

  output logic          p_v,
  output logic [31:2]   p_tgt,
  output logic          p_reg,
  output logic [((XW > 0) ? XW : 1)-1:0] p_x,
  input  logic          p_take,
  input  logic          kill,

  input  logic          w_en,
  input  logic          w_v,
  input  logic [31:2]   w_pc,
  input  logic [1:0]    w_ctx,
  input  logic          w_reg,
  input  logic [31:2]   w_tgt,
  input  logic [((XW > 0) ? XW : 1)-1:0] w_x,

  input  logic          flush,
  output logic          busy
);

  localparam int IW = $clog2(N);
  localparam int TW = 30 - IW;                // tag bits PC[31:IW+2]
  localparam int BW = 1 + TW + 2 + 1 + 30;    // the entry without x
  localparam int EW = BW + XW;
  localparam logic [IW-1:0] LAST = IW'(N - 1);

  // ------------------------------------------------------------- sweep
  logic          sw_act;
  logic [IW-1:0] sw_cnt;
  always_ff @(posedge clk) begin
    if (rst || flush) begin
      sw_act <= 1'b1;
      sw_cnt <= '0;
    end else if (sw_act) begin
      sw_cnt <= sw_cnt + 1'b1;
      if (sw_cnt == LAST) sw_act <= 1'b0;
    end
  end
  assign busy = sw_act;

  // ------------------------------------------------------------- write port
  logic          we;
  logic [IW-1:0] wa;
  logic [EW-1:0] wd;
  wire  [BW-1:0] wb = {w_v && !sw_act, w_pc[31:IW+2], w_ctx, w_reg, w_tgt};
  assign we = sw_act || (w_en && !rst && !flush);
  assign wa = sw_act ? sw_cnt : w_pc[IW+1:2];
  generate
    if (XW > 0) begin : g_wx
      assign wd = {wb, w_x};
    end else begin : g_wnx
      assign wd = wb;
    end
  endgenerate

  // ------------------------------------------------------------- read port
  wire  [IW-1:0] ra = rd_pc[IW+1:2];
  logic [EW-1:0] q;
  generate
    if (STYLE == 0) begin : g_m10k
      (* ramstyle = "M10K, no_rw_check" *) logic [EW-1:0] m [0:N-1];
      always_ff @(posedge clk) begin
        if (we) m[wa] <= wd;
        q <= m[ra];
      end
    end else begin : g_mlab
      (* ramstyle = "MLAB, no_rw_check" *) logic [EW-1:0] m [0:N-1];
      logic [IW-1:0] ra_q;
      always_ff @(posedge clk) begin
        if (we) m[wa] <= wd;
        ra_q <= ra;
      end
      assign q = m[ra_q];
    end
  endgenerate

  // the address read and whether a write met it on the read edge
  logic [31:2] rpc_q;
  logic        rdw_q;
  always_ff @(posedge clk) begin
    rpc_q <= rd_pc;
    rdw_q <= we && wa == ra;
  end

  // ------------------------------------------------------------- compare
  wire          e_v   = q[EW-1];
  wire [TW-1:0] e_tag = q[EW-2 -: TW];
  wire [1:0]    e_ctx = q[EW-2-TW -: 2];
  wire          e_reg = q[EW-4-TW];
  wire [31:2]   e_tgt = q[XW+29 -: 30];
  wire          hit   = e_v && e_tag == rpc_q[31:IW+2] && e_ctx == ctx && !rdw_q;   // (a sweep clears p_v below)

  always_ff @(posedge clk) begin
    if (rst || flush || sw_act || kill)
      p_v <= 1'b0;
    else if (cap && hit && (!p_v || p_take)) begin
      p_v   <= 1'b1;
      p_tgt <= e_tgt;
      p_reg <= e_reg;
    end else if (p_take)
      p_v <= 1'b0;
  end
  generate
    if (XW > 0) begin : g_x
      always_ff @(posedge clk)
        if (cap && hit && (!p_v || p_take)) p_x <= q[XW-1:0];
    end else begin : g_nx
      assign p_x = 1'b0;
    end
  endgenerate

endmodule
