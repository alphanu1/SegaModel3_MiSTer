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
// Real3D rasteriser: the texture cache, four texels a clock (D-28), in front
// of r3d_rast_texel's four banks. A pixel brings each bank's word address
// (bank b = {A[11], A[0]}, word {A[21:12], A[10:1]}) and gets the four words
// back in order, exactly the texture RAM's. The addresses are whatever the
// texel path computes, so a mip level's address (a level base and u, v >> L)
// needs nothing here.
//
// Lines are 4 x 4 texels (32 B), {y tile, x tile}, 18 bits; the texture RAM
// holds them tiled, a line one 4-beat burst of 64 bits (beat k = row k). A
// bank keeps a line's 2 x 2 texels of its parity in one 64-bit word, written
// as two 32-bit lanes (one per beat pair). SETS x WAYS lines, set from the low
// tile bits of x and y interleaved.
//
// Non-blocking, in order (Model 2's m2_texel_bl in shape: misses in flight,
// answers in request order):
//   L0  the pixel's line per bank; the four tag copies are read at its sets.
//   C   compare. The tag rows are read again every cycle the pixel waits, and
//       the two tag writes the read cannot see are forwarded into them: the
//       one on the read's own edge (Wq) and the one in flight (W). Hits and
//       ways are registered.
//   A   allocate: each distinct missing line takes a cycle: a victim (a
//       global round-robin way, never one this pixel uses), a fill queued
//       with the pixel's sequence number; the row with the new tag goes to W
//       and is written to all four copies the next cycle. The pixel goes into
//       the FIFO needing every fill allocated so far.
//   R   the FIFO's head reads when that many fills are written; the words
//       leave the cycle after.
// A pixel leaves A only on a cycle with no allocation, so the hits it brings
// from C already hold every earlier allocation: none of its own can take a
// way it uses, and its own lines are marked as they are allocated. Nothing is
// forwarded into A. This is R66's single lookup cycle cut in two (17.5 ns
// from the tag word to the tag write).
//
// A fill's data waits in a buffer until the pixels before its own that read
// its slot have read (they may still want the victim's old line), then writes
// in four cycles. GATE = 1: each slot keeps the sequence number of the last
// pixel to read it (two true-dual-port M10K tables, banks 0-1 and 2-3, written
// a cycle after the pixel enters the FIFO); an allocation reads its slot's two
// numbers the cycle after, and the fill waits until the later has read. A
// stale number can only make a fill wait longer, never past its own pixel.
// GATE = 0: R66's rule, every earlier pixel has read (a frame then costs
// about pixels + 4 x fills; study R87). Fills complete in order either way,
// so "fills allocated so far" is exact for an in-order reader, and the fill
// sequence does not depend on GATE. inval (texture RAM written) re-sweeps the
// tags; C and A hold for the sweep and one cycle after it.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast_tcache #(
  parameter int SB   = 8,    // log2 sets: 8 = 32 KiB, 9 = 64 KiB at four ways
  parameter int WAYS = 4,
  parameter int DB   = 6,    // log2 pixels ahead
  parameter int FB   = 3,    // log2 fills in flight
  parameter int PW   = 32,
  parameter bit GATE = 1'b1  // 1: a fill waits only for earlier readers of its slot
) (
  input  logic          clk,
  input  logic          rst,
  input  logic          inval,

  input  logic          i_valid,
  output logic          i_ready,
  input  logic [19:0]   i_a0, i_a1, i_a2, i_a3,
  input  logic [3:0]    i_use,
  input  logic [PW-1:0] i_pay,

  output logic          o_valid,
  input  logic          o_ready,
  output logic [15:0]   o_q0, o_q1, o_q2, o_q3,
  output logic [PW-1:0] o_pay,

  output logic          f_req,
  output logic [17:0]   f_addr,
  input  logic          f_ack,
  input  logic          f_rvalid,
  input  logic [63:0]   f_rdata
);

  localparam int SETS = 1 << SB;
  localparam int WB   = $clog2(WAYS);
  localparam int LB   = SB + WB;          // slot bits
  localparam int TB   = 18 - SB;          // tag bits
  localparam int EB   = TB + 1;           // tag entry: {valid, tag}
  localparam int RW   = WAYS * EB;        // a set's row
  localparam int D    = 1 << DB;
  localparam int F    = 1 << FB;
  localparam int XB   = (SB + 1) / 2, YB = SB / 2;
  // fill counters. A pixel reads once done_n - need >= 0. With GATE = 0 no fill after an
  // unread pixel's need can be written, so done_n - need lies in [-F, 0]: FB + 2 bits. With
  // GATE = 1 fills allocated behind an unread pixel may be written while it waits (the band
  // can hold the output for thousands of cycles), up to 4 a pixel for the D - 1 behind it and
  // the one in A: under 4D, so DB + 4 bits.
  localparam int NB   = GATE ? DB + 4 : FB + 2;

  function automatic logic [17:0] line_of(input logic [19:0] a);
    line_of = {a[19:11], a[9:1]};
  endfunction
  function automatic logic [SB-1:0] set_of(input logic [17:0] l);
    set_of = {l[9 +: YB], l[0 +: XB]};
  endfunction
  function automatic logic [TB-1:0] tag_of(input logic [17:0] l);
    tag_of = {l[17 : 9 + YB], l[8 : XB]};
  endfunction
  // a row with way w's entry set to {valid, t}: constant indices, a mux a way
  function automatic logic [RW-1:0] put(input logic [RW-1:0] r, input logic [WB-1:0] w, input logic [TB-1:0] t);
    put = r;
    for (int k = 0; k < WAYS; k++) if (w == WB'(k)) put[k * EB +: EB] = {1'b1, t};
  endfunction

  // ---- tags: four copies, a set's row a word ----------------------------------------------
  (* ramstyle = "M10K, no_rw_check" *) logic [RW-1:0] tg0 [SETS];
  (* ramstyle = "M10K, no_rw_check" *) logic [RW-1:0] tg1 [SETS];
  (* ramstyle = "M10K, no_rw_check" *) logic [RW-1:0] tg2 [SETS];
  (* ramstyle = "M10K, no_rw_check" *) logic [RW-1:0] tg3 [SETS];
  logic          twe;
  logic [SB-1:0] tra [4];
  logic [SB-1:0] twa;
  logic [RW-1:0] twd;
  logic [RW-1:0] trq [4];
  always_ff @(posedge clk) begin
    if (twe) begin tg0[twa] <= twd; tg1[twa] <= twd; tg2[twa] <= twd; tg3[twa] <= twd; end
    trq[0] <= tg0[tra[0]]; trq[1] <= tg1[tra[1]]; trq[2] <= tg2[tra[2]]; trq[3] <= tg3[tra[3]];
  end

  // ---- data: four banks, two 32-bit lanes each ---------------------------------------------
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dl0 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dh0 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dl1 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dh1 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dl2 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dh2 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dl3 [1 << LB];
  (* ramstyle = "M10K, no_rw_check" *) logic [31:0] dh3 [1 << LB];
  logic          dre;
  logic [LB-1:0] dra [4];
  logic [63:0]   drq [4];
  logic          dwe;          // a beat: banks {k[0], 0} and {k[0], 1}, lane k[1]
  logic [1:0]    dwk;
  logic [LB-1:0] dwa;
  logic [31:0]   dwd0, dwd1;   // x even, x odd
  always_ff @(posedge clk) begin
    if (dwe && !dwk[0] && !dwk[1]) begin dl0[dwa] <= dwd0; dl1[dwa] <= dwd1; end
    if (dwe && !dwk[0] &&  dwk[1]) begin dh0[dwa] <= dwd0; dh1[dwa] <= dwd1; end
    if (dwe &&  dwk[0] && !dwk[1]) begin dl2[dwa] <= dwd0; dl3[dwa] <= dwd1; end
    if (dwe &&  dwk[0] &&  dwk[1]) begin dh2[dwa] <= dwd0; dh3[dwa] <= dwd1; end
    if (dre) begin
      drq[0] <= {dh0[dra[0]], dl0[dra[0]]}; drq[1] <= {dh1[dra[1]], dl1[dra[1]]};
      drq[2] <= {dh2[dra[2]], dl2[dra[2]]}; drq[3] <= {dh3[dra[3]], dl3[dra[3]]};
    end
  end

  // ---- sequence counters -------------------------------------------------------------------
  logic [NB-1:0] alloc_n, iss_n, done_n;   // fills allocated, issued, written (mod 2^NB)
  logic [DB:0]   pix_n, rd_n;              // pixels into the FIFO, pixels read
  logic [SB:0]   sw_n;                     // tag sweep: sets left
  logic          sw_q;                     // the cycle after the sweep: C's rows were read under it
  wire           sweeping = sw_n != '0;
  wire           hold = sweeping || sw_q;

  // ---- W: the tag write of the last allocation, and Wq the one before -------------------------
  logic          w_v, wq_v;
  logic [SB-1:0] w_set, wq_set;
  logic [WB-1:0] w_way, wq_way;
  logic [TB-1:0] w_tg, wq_tg;
  logic [RW-1:0] w_base;                   // the set's row before W's entry
  always_comb begin
    twe = w_v; twa = w_set; twd = put(w_base, w_way, w_tg);
    if (sweeping) begin twe = 1'b1; twa = SB'(sw_n - 1'b1); twd = '0; end
  end

  // ---- L0 -----------------------------------------------------------------------------------
  logic          l0_v;
  logic [19:0]   l0_a [4];
  logic [3:0]    l0_use;
  logic [PW-1:0] l0_pay;
  logic          c_go;
  // a skid in front of L0, so i_ready is a register (!sk_v) and not the lookup's c_go: the rasteriser's
  // front stalls on i_ready, and c_go through it into every front enable was the slowest path at 110 MHz
  // (study R109). The skid takes a pixel only when L0 is held; L0 takes the skid's first. L0 is loaded on
  // the same cycles as before (each c_go with a pixel waiting), so the lookup sees the same stream.
  logic          sk_v;
  logic [19:0]   sk_a [4];
  logic [3:0]    sk_use;
  logic [PW-1:0] sk_pay;
  logic          sw_nz;                    // = sweeping, as a register
  assign i_ready = !sk_v && !sw_nz;
  wire           l0_free = !l0_v || c_go;
  logic [17:0]   l0_ln [4];
  always_comb for (int b = 0; b < 4; b++) l0_ln[b] = line_of(l0_a[b]);

`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && sw_nz != sweeping) $fatal(1, "tcache: sw_nz");
`endif

  // ---- C: compare -----------------------------------------------------------------------------
  logic          c_v;
  logic [17:0]   c_ln [4];
  logic [3:0]    c_use;
  logic [PW-1:0] c_pay;
  logic [1:0]    c_j [4];
  logic [3:0]    c_eq [4], c_ss [4];       // the pixel's banks: same line, same set
  // the rows as they stand: the read, Wq and W applied. Whether bank b's set is
  // W's or Wq's is registered a cycle early, from the set read that cycle and the
  // allocation's set (the compare was in R87's -40C C path)
  logic [RW-1:0] crow [4];
  logic [3:0]    c_hit;
  logic [WB-1:0] c_hway [4];
  logic [3:0]    c_mw, c_mq;               // = w_v && set_of(c_ln[b]) == w_set; Wq's alike
  // the hit per way from the read's entry, W's and Wq's compared side by side and chosen by the
  // registered set matches (study R109: the rows' two puts were in front of the tag compares)
  always_comb begin
    for (int b = 0; b < 4; b++) begin
      logic tw, tq;
      crow[b] = trq[b];
      if (c_mq[b]) crow[b] = put(crow[b], wq_way, wq_tg);
      if (c_mw[b]) crow[b] = put(crow[b], w_way, w_tg);
      tw = w_tg == tag_of(c_ln[b]);
      tq = wq_tg == tag_of(c_ln[b]);
      c_hit[b] = 1'b0; c_hway[b] = '0;
      for (int w = 0; w < WAYS; w++) begin
        logic [EB-1:0] e;
        logic          m;
        e = trq[b][w * EB +: EB];
        m = e[EB-1] && e[TB-1:0] == tag_of(c_ln[b]);
        if (c_mq[b] && wq_way == WB'(w)) m = tq;
        if (c_mw[b] && w_way == WB'(w)) m = tw;
        if (m) begin c_hit[b] = 1'b1; c_hway[b] = WB'(w); end
      end
    end
    // the read: the next pixel's sets when C moves on, else C's own again
    for (int b = 0; b < 4; b++) tra[b] = c_go ? set_of(l0_ln[b]) : set_of(c_ln[b]);
  end

  // ---- A: allocate ----------------------------------------------------------------------------
  logic          a_v;
  logic [17:0]   a_ln [4];
  logic [3:0]    a_use, a_ok, a_hit;
  logic [WB-1:0] a_way [4], a_hway [4];
  logic [3:0]    a_eq [4], a_ss [4];
  logic [PW-1:0] a_pay;
  logic [1:0]    a_j [4];
  logic [RW-1:0] a_row [4];

  // the first missing bank, one-hot
  logic [3:0]    miss, mbh;
  logic [1:0]    mb;
  always_comb begin
    for (int b = 0; b < 4; b++) miss[b] = a_use[b] && !a_ok[b] && !a_hit[b];
    mbh = miss & (~miss + 4'd1);
    mb = mbh[0] ? 2'd0 : mbh[1] ? 2'd1 : mbh[2] ? 2'd2 : 2'd3;
  end
  wire mneed = miss != 4'd0;
  logic [WB-1:0] way_e [4];
  always_comb for (int b = 0; b < 4; b++) way_e[b] = a_ok[b] ? a_way[b] : a_hway[b];
  logic [WB-1:0] rr, vic;
  logic [3:0]    dup;                      // bank on mb's line
  // each bank's way one-hot and whether it holds one (a_use && (a_ok || a_hit)), kept as registers
  // beside a_way and a_ok (study R109): the way's select and decode were the front of the victim's path
  logic [WAYS-1:0] a_wo [4];
  logic [3:0]      a_qb;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && a_v) for (int b = 0; b < 4; b++)
      if (a_qb[b] != (a_use[b] && (a_ok[b] || a_hit[b])) || (a_qb[b] && a_wo[b] != WAYS'(1) << way_e[b]))
        $fatal(1, "tcache: the ways one-hot");
`endif
  logic [WB-1:0] vic_c [4];                // the victim were bank c the one missing
  always_comb begin
    for (int b = 0; b < 4; b++) dup[b] = |(a_eq[b] & mbh);
    // the ways each candidate's set holds for this pixel, in parallel with the choice of mb
    for (int c = 0; c < 4; c++) begin
      logic [WAYS-1:0] busy;
      busy = '0;
      for (int b = 0; b < 4; b++)
        if (a_qb[b] && a_ss[b][c]) busy = busy | a_wo[b];
      vic_c[c] = rr;
      for (int k = WAYS - 1; k >= 0; k--) if (!busy[WB'(rr + WB'(k))]) vic_c[c] = WB'(rr + WB'(k));
    end
    vic = vic_c[mb];
  end
  // a bank on mb's line has mb's set, so its own candidate's victim is mb's (the busy ways are those of
  // the set): each bank's way is loaded from its own vic_c, not through the select (study R109)
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && a_v && mneed) for (int b = 0; b < 4; b++)
      if (a_use[b] && !a_ok[b] && dup[b] && vic_c[b] != vic) $fatal(1, "tcache: a bank's own victim");
`endif

  // fill queue {line, slot, allocating pixel}, indexed by the counters
  localparam int QW = 18 + LB + DB + 1;
  (* ramstyle = "MLAB, no_rw_check" *) logic [QW-1:0] fq [F];
  // the fill queue's and the pixel FIFO's full flags as registers, from their counts' next values: each count
  // moves by one either way, so the next flag is one of three compares of the present count, chosen by the
  // two moves (c7_s1 and standalone: rd_n through pix_n - rd_n, the compare, a_done and c_go into tra and
  // c_mw, 113.7 MHz standalone)
  logic fq_full, pf_full;
  wire [NB-1:0] fq_c = alloc_n - done_n;

  // pixel FIFO {need, payload, j x4, slot x4}
  localparam int PFW = NB + PW + 8 + 4 * LB;
  (* ramstyle = "M10K, no_rw_check" *) logic [PFW-1:0] pf [D];
  wire  [DB:0] pf_cnt  = pix_n - rd_n;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && (fq_full != (fq_c == NB'(F)) || pf_full != (pf_cnt >= (DB+1)'(D - 1))))
    $fatal(1, "tcache: the full flags registered");
`endif

  wire alloc  = a_v && mneed && !fq_full && !hold;
  wire a_done = a_v && !mneed && !pf_full && !hold;
  wire a_go   = (!a_v || a_done) && !hold;
  assign c_go = (!c_v || a_go) && !hold;

  // A's rows with W applied: W is this pixel's own last allocation (a pixel arrives
  // only after a cycle without one, so W never holds an older pixel's)
  logic [RW-1:0] a_rowe [4];
  // whether bank b's set is W's, a register (study R109): W's set is the allocation's (a_set_mb) on the
  // edge W loads, so the next value is A's own same-set flag a_ss[b][mb], or for a pixel moving into A the
  // compare of its line's set with a_set_mb. The compare after w_set ran into w_base and A's rows.
  logic [3:0] a_wm;
  always_comb for (int b = 0; b < 4; b++)
    a_rowe[b] = (w_v && a_wm[b]) ? put(a_row[b], w_way, w_tg) : a_row[b];
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && a_v && w_v) for (int b = 0; b < 4; b++) if (a_wm[b] != (set_of(a_ln[b]) == w_set)) $fatal(1, "tcache: a_wm");
`endif
  logic [SB-1:0] a_set_mb;
  logic [TB-1:0] a_tag_mb;
  always_comb begin a_set_mb = set_of(a_ln[mb]); a_tag_mb = tag_of(a_ln[mb]); end

  logic [LB-1:0] slot [4];
  always_comb for (int b = 0; b < 4; b++) slot[b] = {set_of(a_ln[b]), way_e[b]};

  always_ff @(posedge clk) begin
    if (a_done) pf[pix_n[DB-1:0]] <= {alloc_n, a_pay, a_j[3], a_j[2], a_j[1], a_j[0], slot[3], slot[2], slot[1], slot[0]};
    if (a_done) pfn[pix_n[DB-1:0]] <= alloc_n;
    if (alloc) fq[alloc_n[FB-1:0]] <= {a_ln[mb], a_set_mb, vic, pix_n};
  end

  // ---- the fill path -------------------------------------------------------------------------
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] bb [4 * F];
  logic [FB+2:0] bb_wp, bb_rp;             // beats in, beats written (mod 8F: the buffer can be full)
  logic [1:0]    wk;
  wire  [QW-1:0] fq_i = fq[iss_n[FB-1:0]];
  wire  [QW-1:0] fq_r = fq[done_n[FB-1:0]];
  assign f_req  = alloc_n != iss_n;
  assign f_addr = fq_i[QW-1 -: 18];
  always_ff @(posedge clk) if (f_rvalid) bb[bb_wp[FB+1:0]] <= f_rdata;
  wire         beats4 = (bb_wp - bb_rp) >= (FB+3)'(4);
  logic        gate_ok;
  wire         wr_go = (alloc_n != done_n) && (wk != 2'd0 || (beats4 && gate_ok));
  wire         fq_dn = wr_go && wk == 2'd3;     // a fill written: done_n moves
  wire  [63:0] bq = bb[bb_rp[FB+1:0]];
  always_comb begin
    dwe = wr_go; dwk = wk; dwa = fq_r[DB+1 +: LB];
    dwd0 = {bq[47:32], bq[15:0]};          // x = 2, x = 0
    dwd1 = {bq[63:48], bq[31:16]};         // x = 3, x = 1
  end

  // ---- the write gate ----------------------------------------------------------------------
  // The fill at the head may write once no pixel before its allocator that reads its slot is
  // still to read. Pixel numbers are DB + 1 bits; the unread ones are [rd_n, P), P the
  // allocator, at most D - 1 of them, so a number G is unread when G - rd_n < P - rd_n.
  // rd_n only rises toward P, so once read stays read.
  generate if (GATE) begin : g_gate
    // last reader of each slot: x through banks 0 and 1, y through 2 and 3. Port a: the
    // first bank's write, or the read for an allocation; port b: the second bank's write
    // when its slot is not the first's. Writes go a cycle after the pixel's a_done, reads
    // a cycle after the allocation: alloc and a_done never share a cycle, so a table is
    // never read and written on one edge, and every pixel before an allocator is written
    // before the allocation's read.
    (* ramstyle = "M10K, no_rw_check" *) logic [DB:0] lsx [1 << LB];
    (* ramstyle = "M10K, no_rw_check" *) logic [DB:0] lsy [1 << LB];
    logic          lw_v;
    logic [3:0]    lw_use;
    logic [LB-1:0] lw_s0, lw_s1, lw_s2, lw_s3;
    logic [DB:0]   lw_px;
    logic [DB:0]   w_px, lr_px;          // the allocator, with W and with the read
    logic          lr_v;                 // the stamps of the allocation before last are out
    logic [DB:0]   qx, qy;
    logic          lq_v;                 // ... and registered once more (the M10K's output
    logic [DB:0]   lq_px, qx2, qy2;      // went straight into the reduction: a -40C path)
    wire  [LB-1:0] ax = lw_v ? lw_s0 : {w_set, w_way};
    wire  [LB-1:0] ay = lw_v ? lw_s2 : {w_set, w_way};
    wire           wxa = lw_v && lw_use[0];
    wire           wxb = lw_v && lw_use[1] && !(lw_use[0] && lw_s1 == lw_s0);
    wire           wya = lw_v && lw_use[2];
    wire           wyb = lw_v && lw_use[3] && !(lw_use[2] && lw_s3 == lw_s2);
    /* verilator lint_off MULTIDRIVEN */
    always @(posedge clk) begin
      if (wxa) lsx[ax] <= lw_px;
      qx <= lsx[ax];
    end
    always @(posedge clk) if (wxb) lsx[lw_s1] <= lw_px;
    always @(posedge clk) begin
      if (wya) lsy[ay] <= lw_px;
      qy <= lsy[ay];
    end
    always @(posedge clk) if (wyb) lsy[lw_s3] <= lw_px;
    /* verilator lint_on MULTIDRIVEN */
    // the later reader: the nearer of the two counting back from P (a stale number may
    // alias nearer, which only waits longer)
    wire [DB:0] dx = lq_px - qx2 - (DB+1)'(1), dy = lq_px - qy2 - (DB+1)'(1);
    // (P - 1 - (P - q - 1) is q, mod 2^(DB + 1): the nearer reader's own number, without the last subtract;
    // qx2 through the two subtracts and the compare into fs was 114.2 MHz at -40C standalone)
    wire [DB:0] gl = (dx < dy) ? qx2 : qy2;
`ifdef VERILATOR
    always_ff @(posedge clk) if (!rst && lq_v && gl != lq_px - (DB+1)'(1) - ((dx < dy) ? dx : dy)) $fatal(1, "tcache: the gate's reader");
`endif
    (* ramstyle = "MLAB, no_rw_check" *) logic [DB:0] fs [F];
    logic [NB-1:0] st_n;                 // fills whose last reader is known
    always_ff @(posedge clk) if (lq_v) fs[st_n[FB-1:0]] <= gl;
    always_ff @(posedge clk) begin
      lw_v <= a_done && !rst; lw_use <= a_use;
      lw_s0 <= slot[0]; lw_s1 <= slot[1]; lw_s2 <= slot[2]; lw_s3 <= slot[3]; lw_px <= pix_n;
      w_px <= pix_n; lr_px <= w_px;
      lr_v <= w_v && !rst;
      lq_v <= lr_v && !rst; lq_px <= lr_px; qx2 <= qx; qy2 <= qy;
      if (rst) st_n <= '0;
      else if (lq_v) st_n <= st_n + 1'b1;
    end
    wire [DB:0] g_dl = fs[done_n[FB-1:0]] - rd_n;
    wire [DB:0] g_dp = fq_r[DB:0] - rd_n;
    assign gate_ok = (st_n != done_n) && (g_dl >= g_dp);
  end else begin : g_all
    assign gate_ok = rd_n == fq_r[DB:0];   // every pixel before the allocator has read
  end endgenerate

  // ---- R: the head, read a cycle ahead into ph --------------------------------------------------
  logic [PFW-1:0] ph;
  logic           ph_v;
  // the head's need from an MLAB of its own, read at rd_n as it stands (study R109): from the block
  // RAM's output (ph) the subtract and r_go ran into the next read address; the MLAB's read is faster
  // and the next address is chosen between rd_n and rd_n + 1, both registers
  (* ramstyle = "MLAB, no_rw_check" *) logic [NB-1:0] pfn [D];
  wire  [NB-1:0]  ph_need = pfn[rd_n[DB-1:0]];
  logic [DB:0]    rd_n1;                   // rd_n + 1
  wire            out_free = !o_valid || o_ready;
  wire            r_go = ph_v && ($signed(done_n - ph_need) >= 0) && out_free;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && ph_v && ph_need != ph[PFW-1 -: NB]) $fatal(1, "tcache: the need's MLAB");
`endif
  logic           r_v;
  logic [PW-1:0]  r_pay;
  logic [1:0]     r_j [4];
  always_comb begin
    dre = r_go;
    for (int b = 0; b < 4; b++) dra[b] = ph[b * LB +: LB];
  end
  wire [DB:0] ra = r_go ? rd_n1 : rd_n;
  always_ff @(posedge clk) ph <= pf[ra[DB-1:0]];

  function automatic logic [15:0] pick(input logic [63:0] w, input logic [1:0] j);
    pick = w[16 * j +: 16];
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      l0_v <= 1'b0; c_v <= 1'b0; a_v <= 1'b0; o_valid <= 1'b0; r_v <= 1'b0; ph_v <= 1'b0; sk_v <= 1'b0;
      sw_nz <= 1'b1;
      w_v <= 1'b0; wq_v <= 1'b0; sw_q <= 1'b0; c_mw <= '0; c_mq <= '0;
      alloc_n <= '0; iss_n <= '0; done_n <= '0; pix_n <= '0; rd_n <= '0; rd_n1 <= (DB+1)'(1);
      fq_full <= 1'b0; pf_full <= 1'b0;
      sw_n <= (SB+1)'(SETS); rr <= '0; bb_wp <= '0; bb_rp <= '0; wk <= 2'd0;
    end else begin
      if (inval && !sweeping) begin sw_n <= (SB+1)'(SETS); sw_nz <= 1'b1; end
      else if (sweeping) begin sw_n <= sw_n - 1'b1; sw_nz <= sw_n != (SB+1)'(1); end
      sw_q <= sweeping;
      // L0 and its skid
      if (l0_free) begin
        if (sk_v) begin
          l0_v <= 1'b1; sk_v <= 1'b0;
          l0_a <= sk_a; l0_use <= sk_use; l0_pay <= sk_pay;
        end else if (i_valid && i_ready) begin
          l0_v <= 1'b1;
          l0_a[0] <= i_a0; l0_a[1] <= i_a1; l0_a[2] <= i_a2; l0_a[3] <= i_a3;
          l0_use <= i_use; l0_pay <= i_pay;
        end else l0_v <= 1'b0;
      end else if (i_valid && i_ready) begin
        sk_v <= 1'b1;
        sk_a[0] <= i_a0; sk_a[1] <= i_a1; sk_a[2] <= i_a2; sk_a[3] <= i_a3;
        sk_use <= i_use; sk_pay <= i_pay;
      end
      // C
      if (c_go) begin
        c_v <= l0_v;
        for (int b = 0; b < 4; b++) begin
          c_ln[b] <= l0_ln[b]; c_j[b] <= {l0_a[b][10], l0_a[b][0]};
          for (int k = 0; k < 4; k++) begin
            c_eq[b][k] <= l0_ln[b] == l0_ln[k];
            c_ss[b][k] <= set_of(l0_ln[b]) == set_of(l0_ln[k]);
          end
        end
        c_use <= l0_use; c_pay <= l0_pay;
      end
      // A
      if (a_go) begin
        a_v <= c_v;
        for (int b = 0; b < 4; b++) begin
          a_ln[b] <= c_ln[b]; a_j[b] <= c_j[b]; a_eq[b] <= c_eq[b]; a_ss[b] <= c_ss[b];
          a_hway[b] <= c_hway[b]; a_row[b] <= crow[b];
          a_way[b] <= '0;
        end
        a_hit <= c_hit; a_ok <= ~c_use; a_use <= c_use; a_pay <= c_pay;
        a_qb <= c_use & c_hit;
        for (int b = 0; b < 4; b++) a_wo[b] <= WAYS'(1) << c_hway[b];
      end else if (sweeping) begin
        for (int b = 0; b < 4; b++) a_row[b] <= '0;   // a row read before the upload is not written back
      end else begin
        for (int b = 0; b < 4; b++) a_row[b] <= a_rowe[b];
        if (alloc)                          // banks on the line are done
          for (int b = 0; b < 4; b++) if (a_use[b] && !a_ok[b] && dup[b]) begin
            a_ok[b] <= 1'b1; a_way[b] <= vic_c[b]; a_qb[b] <= 1'b1; a_wo[b] <= WAYS'(1) << vic_c[b];
          end
      end
      // W, Wq
      w_v <= alloc; w_set <= a_set_mb; w_way <= vic; w_tg <= a_tag_mb; w_base <= a_rowe[mb];
      for (int b = 0; b < 4; b++) a_wm[b] <= a_go ? (set_of(c_ln[b]) == a_set_mb) : a_ss[b][mb];
      wq_v <= w_v && !sweeping; wq_set <= w_set; wq_way <= w_way; wq_tg <= w_tg;
      for (int b = 0; b < 4; b++) begin    // tra is C's set next cycle
        c_mw[b] <= alloc && tra[b] == a_set_mb;
        c_mq[b] <= w_v && !sweeping && tra[b] == w_set;
      end
      if (alloc) begin rr <= vic + WB'(1); alloc_n <= alloc_n + 1'b1; end
      fq_full <= (alloc && !fq_dn) ? (fq_c == NB'(F - 1)) : (!alloc && fq_dn) ? (fq_c == NB'(F + 1)) : (fq_c == NB'(F));
      pf_full <= (a_done && !r_go) ? (pf_cnt >= (DB+1)'(D - 2)) : (!a_done && r_go) ? (pf_cnt >= (DB+1)'(D)) :
                                     (pf_cnt >= (DB+1)'(D - 1));
      if (a_done) pix_n <= pix_n + 1'b1;
      // fills
      if (f_req && f_ack) iss_n <= iss_n + 1'b1;
      if (f_rvalid) bb_wp <= bb_wp + 1'b1;
      if (wr_go) begin
        bb_rp <= bb_rp + 1'b1;
        wk <= wk + 2'd1;
        if (wk == 2'd3) done_n <= done_n + 1'b1;
      end
      // R: the head is valid once written before this edge
      ph_v <= r_go ? (pix_n != rd_n1) : (pix_n != rd_n);
      if (r_go) begin
        rd_n <= rd_n1; rd_n1 <= rd_n1 + 1'b1;
        r_pay <= ph[4 * LB + 8 +: PW];
        for (int b = 0; b < 4; b++) r_j[b] <= ph[4 * LB + 2 * b +: 2];
      end
      if (out_free) begin
        r_v <= r_go;
        o_valid <= r_v;
        if (r_v) begin
          o_q0 <= pick(drq[0], r_j[0]); o_q1 <= pick(drq[1], r_j[1]);
          o_q2 <= pick(drq[2], r_j[2]); o_q3 <= pick(drq[3], r_j[3]);
          o_pay <= r_pay;
        end
      end
    end
  end

endmodule
