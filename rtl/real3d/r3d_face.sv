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
// The back-face test beside the pool (study R62): Supermodel's
// dot(view vertex 0, mat3(modelview) x face normal), facing away when > 0
// (R3DShaderTriangles.h CalcBackFace), one polygon at a time on one FP32
// multiply-add (r3d_fp_mul into r3d_fp_add, flush to zero, as the pool's
// lanes). The order is the model's (sim/real3d/r3d_geo.cpp):
//   V_j = ((x T0j + y T1j) + z T2j) + T3j      j = 0..2, the view vertex
//   N_j = (n0 T0j + n1 T1j) + n2 T2j           the view face normal
//   d   = (V0 N0 + V1 N1) + V2 N2
// with T the model matrix (rows 0..2 the 3x3, row 3 the translation), x y z
// vertex 0's position words (17.7, or 13.11 from Step 1.5) and n the header's
// face normal (s1.22), both converted on the way into the multiplier.
//
// Schedule: a period of four rounds of seven slots. Slots 0-2 run the V
// chains of the polygon in hand, slots 3-5 its N chains (rounds 0-2), slot 6
// the d chain of the polygon before it (rounds 0-2). A chain's steps are seven
// cycles apart, so each step takes the previous step's sum two cycles after it
// leaves the adder: no accumulator registers. 28 cycles a polygon. The words
// are converted on the way to the multiplier, off the decoder's write path.
//
// Memories: A (MLAB) four polygon slots of eight words, the decoder's raw
// position and normal words, then V0..V2 over the position words; B (MLAB)
// the model matrix for two models, {buffer, row, column}, and N0..N2 for two
// polygons in column 3.
//
// LOD = 1 (study D-33, R67): the polygon's LOD word B = d x (NP x C) in the
// idle slots of round 3, FP32 with flush to zero as the model has it
// (sim/real3d/r3d_geo.cpp lod_word). NP is header word 5 bits 8-31 (u12.12),
// written raw at slot word 7 and converted by the same converter; C = -cota
// kz^2 is a register loaded once a viewport (c_we, while the unit is idle).
// Round 3 slot 3 forms Q = NP x C for the polygon in hand, into B at {its N
// buffer, 3, 3}; d is kept in a register as it leaves the adder; round 3 slot
// 4 of the period after forms B = d x Q for the polygon two back. A period
// with only that product left runs as the last. With KZ2 other than 1.0 (MAME's
// frustum, kz a constant) the unit forms C = KZ2 x (-cota) itself, idle, before
// the viewport's first polygon. B leaves in polygon order on b_valid.

module r3d_face #(
  parameter bit          LOD = 1'b0,
  parameter logic [31:0] KZ2 = 32'h3f80_0000
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        step15,          // 13.11 vertex positions (Step 1.5 and later)

  // the model matrix, a word at a time (main, from the pool's TR rows)
  input  logic        t_we,
  input  logic        t_buf,
  input  logic [1:0]  t_row,
  input  logic [1:0]  t_col,
  input  logic [31:0] t_d,
  output logic        t_hold,          // B is written by the unit this cycle: write again
  input  logic        t_clr,           // buffer t_buf is about to be overwritten
  input  logic        t_set,           // buffer t_buf is complete
  output logic [1:0]  mb_busy,         // a pending polygon still uses buffer b

  // polygon inputs (decoder): raw words, then commit
  output logic        in_ready,        // the fill slot is free
  output logic        in_hold,         // A is written by the unit this cycle: write again
  input  logic        in_we,
  input  logic [2:0]  in_word,         // 0..2 x y z, 3..5 n0 n1 n2
  input  logic [23:0] in_d,
  input  logic        in_commit,
  input  logic        in_mb,           // the polygon's model-matrix buffer

  // verdicts, one per committed polygon, in order
  output logic        v_valid,
  output logic        v_gt,            // d > 0
  output logic        v_lt,            // d < 0
  output logic        idle,

  // LOD: the viewport's -cota (loaded while idle), B a polygon in order
  input  logic        c_we,
  input  logic [31:0] c_d,
  output logic        b_valid,
  output logic [31:0] b_word
);

  localparam logic [31:0] F_ONE = 32'h3f80_0000;
  localparam bit          CMUL  = LOD && (KZ2 != F_ONE);   // C formed here: KZ2 x (-cota)

  // ---- slots ---------------------------------------------------------------
  logic [3:0] full /* verilator public_flat_rd */;
  logic [3:0] mbs /* verilator public_flat_rd */;
  logic [1:0] fp /* verilator public_flat_rd */, cp /* verilator public_flat_rd */;                  // fill, next to start
  logic [1:0] tv /* verilator public_flat_rd */;                      // matrix buffer valid

  assign in_ready = !full[fp];
  always_comb for (int b = 0; b < 2; b++) begin
    mb_busy[b] = 1'b0;
    for (int k = 0; k < 4; k++) if (full[k] && mbs[k] == 1'(b)) mb_busy[b] = 1'b1;
  end
  // C (LOD): the register, and with CMUL its product pending / in flight
  logic [31:0] creg /* verilator public_flat_rd */;
  logic        c_pend, c_busy;
  logic        run2;                   // LOD: the polygon two back still has its B product
  assign idle = full == 4'd0 && (!LOD || (!run2 && !c_pend));

  // ---- sequencer -----------------------------------------------------------
  logic        run /* verilator public_flat_rd */;
  logic [1:0]  r /* verilator public_flat_rd */;                      // round
  logic [2:0]  s /* verilator public_flat_rd */;                      // slot
  logic        cur_v /* verilator public_flat_rd */, prev_v /* verilator public_flat_rd */;
  logic [1:0]  cur_slot /* verilator public_flat_rd */, prev_slot /* verilator public_flat_rd */;
  logic        cur_par /* verilator public_flat_rd */, prev_par /* verilator public_flat_rd */;      // N buffer of a polygon (alternates)
  logic        prev2_v, prev2_par;     // LOD: the polygon two back (its B product)
  assign run2 = run && (cur_v || prev_v || prev2_v);
  logic        pend /* verilator public_flat_rd */;                   // the slot at cp is filled but not started
  logic [3:0]  started;
  assign pend = full[cp] && !started[cp];

  // this cycle's operation
  // a: the position, normal or NP word through the converter, one, the FP32
  // word, d (LOD), KZ2 (CMUL); b: the B word, or C (LOD)
  localparam logic [2:0] A_POS = 3'd0, A_NRM = 3'd1, A_ONE = 3'd2, A_RAW = 3'd3, A_NP = 3'd4, A_D = 3'd5, A_KZ = 3'd6;
  // write-back: 0 none, 1 V into A, 2 N into B, 3 verdict (and d), 4 Q into
  // B, 5 B out, 6 C
  localparam logic [2:0] W_NO = 3'd0, W_V = 3'd1, W_N = 3'd2, W_D = 3'd3, W_Q = 3'd4, W_B = 3'd5, W_C = 3'd6;
  logic        op_v, op_c0, op_bc;
  logic [2:0]  op_ak;
  logic [4:0]  op_aa, op_ba;
  logic [2:0]  op_wk;
  logic [1:0]  op_wj, op_wslot;
  logic        op_wpar;
  logic        c_go;                   // CMUL: C's product this cycle
  assign c_go = CMUL && c_pend && !c_busy && !run;
  always_comb begin
    logic [1:0] j;
    j = '0;
    // the adder takes its zero select a cycle after the product it belongs
    // to (c0_d[5] meets the product of the operation one slot earlier), so
    // op_c0 here starts the chain of the operation before: the round-0 steps,
    // the d chain's first step (round 0 slot 6, from round 1 slot 0), and
    // with LOD the two products of round 3 (slots 3, 4, from slots 4, 5)
    op_v = 1'b0; op_bc = 1'b0; op_ak = A_POS; op_aa = '0; op_ba = '0;
    op_c0 = (r == 2'd0) || (r == 2'd1 && s == 3'd0) || (LOD && r == 2'd3 && (s == 3'd4 || s == 3'd5));
    op_wk = W_NO; op_wj = '0; op_wslot = cur_slot; op_wpar = cur_par;
    if (s < 3'd3) begin                                  // V_j
      j = s[1:0];
      op_v = run && cur_v;
      op_aa = {cur_slot, 1'b0, (r == 2'd3) ? 2'd0 : r};
      op_ak = (r == 2'd3) ? A_ONE : A_POS;
      op_ba = {mbs[cur_slot], r, j};
      if (r == 2'd3) begin op_wk = W_V; op_wj = j; end
    end else if (s < 3'd6) begin                         // N_j
      j = 2'(s - 3'd3);
      op_v = run && cur_v && r != 2'd3;
      op_aa = {cur_slot, 1'b0, 2'd3} + {3'd0, r};
      op_ak = A_NRM;
      op_ba = {mbs[cur_slot], r, j};
      if (r == 2'd2) begin op_wk = W_N; op_wj = j; end
    end else begin                                       // d, the polygon before
      op_v = run && prev_v && r != 2'd3;
      op_aa = {prev_slot, 1'b0, r};
      op_ak = A_RAW;
      op_ba = {prev_par, r, 2'd3};
      op_wslot = prev_slot; op_wpar = prev_par;
      if (r == 2'd2) op_wk = W_D;
    end
    if (LOD && r == 2'd3 && s == 3'd3) begin             // Q = NP x C, the polygon in hand
      op_v = run && cur_v; op_bc = 1'b1;
      op_ak = A_NP; op_aa = {cur_slot, 3'd7};
      op_wk = W_Q; op_wslot = cur_slot; op_wpar = cur_par;
    end
    if (LOD && r == 2'd3 && s == 3'd4) begin             // B = d x Q, the polygon two back
      op_v = run && prev2_v;
      op_ak = A_D; op_ba = {prev2_par, 2'd3, 2'd3};
      op_wk = W_B;
    end
    if (c_go) begin                                      // C = KZ2 x (-cota)
      op_v = 1'b1; op_bc = 1'b1; op_ak = A_KZ; op_wk = W_C;   // the next cycle's op_c0 is 1 (idle, round 0)
    end
  end

  // ---- memories ------------------------------------------------------------
  logic        a_we, b_we;
  logic [4:0]  a_wa, b_wa;
  logic [31:0] a_wd, b_wd, a_q, b_q;
  r3d_ram #(.W(32), .A(5)) u_a (.clk, .we(a_we), .wa(a_wa), .wd(a_wd), .ra(op_aa), .rq(a_q));
  r3d_ram #(.W(32), .A(5)) u_b (.clk, .we(b_we), .wa(b_wa), .wd(b_wd), .ra(op_ba), .rq(b_q));

  // ---- the multiply-add ----------------------------------------------------
  // t: read; t+1 convert; t+2 multiplier in; t+11 sum (the adder's align
  // stage split, as the pool's)
  logic [31:0] ar_q, br_q;
  logic [2:0]  ak_q;
  logic        bc_q;
  logic [31:0] dreg;                   // LOD: d of the polygon two back
  logic [31:0] opa, opb;
  logic [5:1]  c0_d;
  logic [31:0] prod, sum /* verilator public_flat_rd */, sd1, sd2;
  typedef struct packed { logic v; logic [2:0] k; logic [1:0] j, slot; logic par; } wb_t;
  wb_t         wb [1:11] /* verilator public_flat_rd */;

  // raw 24-bit integer (signed, or unsigned with !sgd) x 2^(eb - 127) to
  // FP32, exact. One converter for the position, normal and NP words. The
  // sign and magnitude are formed in the read cycle (ar_s, ar_m): the
  // negation's carry chain in front of the leading-one search and the shift
  // was R73's lateack_s3's slowest facing-test path at -40C.
  function automatic logic [31:0] itof(input logic sg, input logic [23:0] mag, input logic [7:0] eb);
    logic        nz;
    logic [23:0] m;
    logic [4:0]  msb;
    begin
      msb = 5'd0; nz = 1'b0;
      for (int i = 0; i < 24; i++) if (mag[i]) begin msb = 5'(i); nz = 1'b1; end
      m = mag << (5'd23 - msb);
      itof = nz ? {sg, eb + {3'd0, msb}, m[22:0]} : 32'd0;
    end
  endfunction

  logic        ar_s;
  logic [23:0] ar_m;
  always_ff @(posedge clk) begin
    logic [7:0] eb;
    logic       sg;
    ar_q <= a_q; br_q <= b_q; ak_q <= op_ak; bc_q <= op_bc;
    sg = !(LOD && op_ak == A_NP) && a_q[23];
    ar_s <= sg; ar_m <= sg ? (~a_q[23:0] + 24'd1) : a_q[23:0];
    eb = (ak_q == A_NRM) ? 8'd105 : (LOD && ak_q == A_NP) ? 8'd115 : step15 ? 8'd116 : 8'd120;
    case (ak_q)
      A_ONE: opa <= F_ONE;
      A_RAW: opa <= ar_q;
      A_D: opa <= LOD ? dreg : ar_q;
      A_KZ: opa <= CMUL ? KZ2 : ar_q;
      default: opa <= itof(ar_s, ar_m, eb);
    endcase
    opb <= (LOD && bc_q) ? creg : br_q;
    c0_d[1] <= op_c0;
    for (int q = 2; q <= 5; q++) c0_d[q] <= c0_d[q - 1];
    sd1 <= sum; sd2 <= sd1;
    wb[1] <= '{v: op_v && op_wk != W_NO, k: op_wk, j: op_wj, slot: op_wslot, par: op_wpar};
    for (int q = 2; q <= 11; q++) wb[q] <= wb[q - 1];
    if (rst) for (int q = 1; q <= 11; q++) wb[q].v <= 1'b0;
  end

  r3d_fp_mul #(.SUBNORM(1'b0)) u_mul (
    .clk, .rst_n(!rst), .in_valid(1'b1), .a(opa), .b(opb),
    .out_valid(), .result(prod), .overflow(), .underflow(), .invalid());
  r3d_fp_add #(.SUBNORM(1'b0), .PIPE_A2(1'b1)) u_add (
    .clk, .rst_n(!rst), .in_valid(1'b1), .a(prod), .b(c0_d[5] ? 32'h8000_0000 : sd2), .sub(1'b0),
    .out_valid(), .result(sum), .overflow(), .underflow(), .invalid());

  // ---- write-back (t+11): V into A, N and Q into B, the verdict, B, C -----
  logic wv /* verilator public_flat_rd */, wn /* verilator public_flat_rd */, wd /* verilator public_flat_rd */;
  logic wq, wbo, wc;
  assign wv = wb[11].v && wb[11].k == W_V;
  assign wn = wb[11].v && wb[11].k == W_N;
  assign wd = wb[11].v && wb[11].k == W_D;
  assign wq = LOD && wb[11].v && wb[11].k == W_Q;
  assign wbo = LOD && wb[11].v && wb[11].k == W_B;
  assign wc = CMUL && wb[11].v && wb[11].k == W_C;
  assign in_hold = wv;
  assign t_hold  = wn || wq;
  always_comb begin
    a_we = wv || in_we;
    a_wa = wv ? {wb[11].slot, 1'b0, wb[11].j} : {fp, in_word};
    a_wd = wv ? sum : {8'd0, in_d};
    b_we = wn || wq || t_we;
    b_wa = wn ? {wb[11].par, wb[11].j, 2'd3} : wq ? {wb[11].par, 2'd3, 2'd3} : {t_buf, t_row, t_col};
    b_wd = (wn || wq) ? sum : t_d;
  end
  logic s_nan, s_zero;
  assign s_nan  = sum[30:23] == 8'hff && sum[22:0] != 23'd0;
  assign s_zero = sum[30:0] == 31'd0;

  always_ff @(posedge clk) begin
    if (rst) begin
      full <= '0; started <= '0; fp <= '0; cp <= '0; tv <= '0;
      run <= 1'b0; r <= '0; s <= '0; cur_v <= 1'b0; prev_v <= 1'b0; cur_par <= 1'b0;
      v_valid <= 1'b0; prev2_v <= 1'b0; b_valid <= 1'b0; c_pend <= 1'b0; c_busy <= 1'b0;
    end else begin
      v_valid <= 1'b0; b_valid <= 1'b0;
      if (LOD) begin
        if (wd) dreg <= sum;
        if (wbo) begin b_valid <= 1'b1; b_word <= sum; end
        if (c_we) begin creg <= c_d; c_pend <= CMUL; end
        if (c_go) c_busy <= 1'b1;
        if (wc) begin creg <= sum; c_pend <= 1'b0; c_busy <= 1'b0; end
      end
      if (t_clr) tv[t_buf] <= 1'b0;
      if (t_set) tv[t_buf] <= 1'b1;
      if (in_commit) begin full[fp] <= 1'b1; mbs[fp] <= in_mb; fp <= fp + 2'd1; end
      if (wd) begin
        v_valid <= 1'b1;
        v_gt <= !sum[31] && !s_zero && !s_nan;
        v_lt <= sum[31] && !s_zero && !s_nan;
        full[wb[11].slot] <= 1'b0; started[wb[11].slot] <= 1'b0;
      end
      // a period starts with the next filled slot whose matrix is in, and
      // runs while it or the polygon before it has work
      if (!run || (r == 2'd3 && s == 3'd6)) begin
        logic nv;
        nv = pend && tv[mbs[cp]] && !(CMUL && c_pend);
        prev_v <= run && cur_v; prev_slot <= cur_slot; prev_par <= cur_par;
        prev2_v <= LOD && run && prev_v; prev2_par <= prev_par;
        cur_v <= nv;
        if (nv) begin cur_slot <= cp; cur_par <= !cur_par; started[cp] <= 1'b1; cp <= cp + 2'd1; end
        run <= nv || (run && (cur_v || (LOD && prev_v)));
        r <= '0; s <= '0;
      end else if (s == 3'd6) begin
        s <= '0; r <= r + 2'd1;
      end else s <= s + 3'd1;
    end
  end

`ifdef VERILATOR
  // flushed or subnormal values at the units (the model assumes none)
  logic [31:0] subn_cnt /* verilator public_flat_rd */;
  function automatic logic issub(input logic [31:0] x);
    issub = (x[30:23] == 8'd0) && (x[22:0] != 23'd0);
  endfunction
  logic [11:1] mv;
  always_ff @(posedge clk) begin
    mv[1] <= op_v;
    for (int q = 2; q <= 11; q++) mv[q] <= mv[q - 1];
    if (rst) subn_cnt <= '0;
    else subn_cnt <= subn_cnt + 32'(mv[2] && (issub(opa) || issub(opb))) + 32'(mv[6] && issub(prod)) + 32'(mv[11] && issub(sum));
  end
`endif

endmodule
