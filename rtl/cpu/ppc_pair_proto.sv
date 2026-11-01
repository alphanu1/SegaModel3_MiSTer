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
// Dual-issue prototype (study D-24), front half: the fetch queue popping one
// or two words, the pairing class predecoded as each word arrives, the pair
// check in ID, and the second decode, for a standalone fit beside
// ppc_dual_proto.sv. Not a core: the words come in from a register, and
// what ID hands on is registered at the output.
//
// The pairing class (cls_t) is formed at the push, as the core's predecode
// is (ppc_core.sv), so the check in ID works from registers:
//   unit: 0 single-cycle integer (add, logical, rotate, shift, compare,
//   cntlzw, exts), 1 other integer (multiply, divide), 2 load / store,
//   3 FP, 4 system register (SPR moves, CR logic, sync ops), 5 branch;
//   the GPRs read and written; CR fields written and read; XER written
//   (CA, OV) and CA read; LR and CTR written and read; one uop or cracked;
//   serialising (603e 6.3.3.2: sc, rfi, isync, sync, mtmsr, mtspr XER,
//   mcrxr, lmw, lswi, lswx, cache and TLB ops).
// The pair is (entry 0, entry 1), in program order, and pairs when
//   the variant allows the two units (VAR below), both are one uop and
//   neither serialises; entry 1 reads no GPR, CR field, CA, LR or CTR that
//   entry 0 writes; the two write no common GPR, CR field, XER, LR or CTR.
//
// VAR (the census model's lv2_ option in brackets):
//   0 one issue slot: the reference (pop one; no check, no second decode)
//   1 any two different units, 603e dispatch (da): two full decoders, and
//     the decoded uops steered to the two slots by unit
//   2 a single-cycle integer op with an op of another unit, either order
//     (di, dn, dl): two full decoders; the integer op is steered to slot B,
//     which takes only the simple integer fields
//   3 as 2, the pair reading at most three GPRs and writing one (dw): the
//     read ports' field map formed here as well
//   4 entry 1 a branch (dm): a branch-only decoder on entry 1; slot A's
//     decoder is untouched, no steering
// FETCH2 = 1 pushes two words a cycle (a two-word I-cache path); 0 one.

module ppc_pair_proto
  import ppc_pkg::*;
#(
  parameter int VAR    = 2,
  parameter int FETCH2 = 0
)(
  input  logic        clk,
  input  logic        rst,
  input  logic [1:0]  push,            // words arriving (word 1 only with FETCH2)
  input  logic [1:0][31:0] w_insn,
  input  logic [1:0][31:0] w_pc,
  input  logic        hold,            // ID cannot take its head (hazards outside)
  input  logic        clear,           // a redirect
  output logic [1:0]  pop,             // taken from the queue this cycle: 0, 1 or 2
  output uop_t        ua,              // slot A's uop, registered
  output uop_t        ub,              // slot B's (VAR 1: whole; 2, 3: simple-integer fields; 4: branch fields)
  output logic        ub_v,
  output logic [2:0][2:0] pmap,        // VAR 3: the read ports' fields
  output logic [31:0] o_pc0, o_pc1     // bench: the PCs of the older and younger op that left ID
);

  typedef struct packed {
    logic [2:0] unit;
    logic       one;            // a single uop
    logic       ser;            // serialising
    logic       wg; logic [4:0] rd;   // GPR written
    logic       ua_, ub_, us_;  // reads rA, rB, rS
    logic [7:0] crw;            // CR fields written
    logic [7:0] crr;            // CR fields read
    logic       xw, car;        // XER written; CA read
    logic       lrw, lrr, ctrw, ctrr;
    logic [1:0] nrd;            // GPRs read (for the three-port rule)
  } cls_t;

  function automatic cls_t classify(input logic [31:0] w);
    cls_t c;
    logic [5:0] p; logic [9:0] x; logic [4:0] rt, ra, rb, bi; logic rc, oe;
    p = w[31:26]; x = w[10:1]; rt = w[25:21]; ra = w[20:16]; rb = w[15:11]; rc = w[0]; oe = w[10];
    bi = w[20:16];
    c = '0; c.one = 1'b1; c.unit = 3'd4;
    case (p)
      6'd14, 6'd15: begin c.unit = 0; c.wg = 1; c.rd = rt; c.ua_ = ra != 0; end            // addi, addis
      6'd12, 6'd13: begin c.unit = 0; c.wg = 1; c.rd = rt; c.ua_ = 1; c.xw = 1; if (p[0]) c.crw = 8'h80; end
      6'd8:         begin c.unit = 0; c.wg = 1; c.rd = rt; c.ua_ = 1; c.xw = 1; end           // subfic
      6'd7:         begin c.unit = 1; c.wg = 1; c.rd = rt; c.ua_ = 1; end                   // mulli
      6'd10, 6'd11: begin c.unit = 0; c.ua_ = 1; c.crw = 8'h80 >> rt[4:2]; end              // cmpli, cmpi
      6'd24, 6'd25, 6'd26, 6'd27, 6'd28, 6'd29:
                    begin c.unit = 0; c.wg = 1; c.rd = ra; c.us_ = 1; if (p == 6'd28 || p == 6'd29) c.crw = 8'h80; end
      6'd20, 6'd21, 6'd23:
                    begin c.unit = 0; c.wg = 1; c.rd = ra; c.us_ = 1; c.ua_ = p == 6'd20; c.ub_ = p == 6'd23; if (rc) c.crw = 8'h80; end
      6'd16:        begin c.unit = 5; c.crr = w[23] ? 8'h00 : (8'h80 >> bi[4:2]); c.ctrr = !w[23]; c.ctrw = !w[23]; c.lrw = w[0]; end
      6'd18:        begin c.unit = 5; c.lrw = w[0]; end
      6'd19: begin
        if (x == 10'd16 || x == 10'd528) begin
          c.unit = 5; c.crr = w[23] ? 8'h00 : (8'h80 >> bi[4:2]); c.lrw = w[0];
          if (x == 10'd16) begin c.lrr = 1; c.ctrr = !w[23]; c.ctrw = !w[23]; end else c.ctrr = 1;
        end else if (x == 10'd150 || x == 10'd50) c.ser = 1;                                // isync, rfi
        else begin c.crw = 8'h80 >> rt[4:2]; c.crr = (8'h80 >> ra[4:2]) | (8'h80 >> rb[4:2]); end
      end
      6'd17: c.ser = 1;
      6'd32, 6'd34, 6'd40, 6'd42, 6'd48, 6'd50:
             begin c.unit = 2; c.wg = p < 6'd48; c.rd = rt; c.ua_ = ra != 0; end
      6'd33, 6'd35, 6'd41, 6'd43, 6'd49, 6'd51, 6'd46, 6'd47:
             begin c.unit = 2; c.one = 0; c.ser = p >= 6'd46; end                            // update loads, lmw, stmw
      6'd36, 6'd38, 6'd44, 6'd52, 6'd54:
             begin c.unit = 2; c.ua_ = ra != 0; c.us_ = p < 6'd52; end
      6'd37, 6'd39, 6'd45, 6'd53, 6'd55:
             begin c.unit = 2; c.wg = 1; c.rd = ra; c.ua_ = 1; c.us_ = p < 6'd52; end       // update stores
      6'd59, 6'd63: begin c.unit = 3; if (rc || (p == 6'd63 && (x == 10'd0 || x == 10'd32))) c.crw = 8'h40;
                     if (p == 6'd63 && (x == 10'd583 || x == 10'd711 || x[4:0] == 5'd6 || x == 10'd64)) c.ser = 1; end
      6'd31: begin
        c.unit = 0; c.wg = 1; c.rd = rt; c.ua_ = 1; c.ub_ = 1;
        if (rc) c.crw = 8'h80;
        case (x)
          10'd28, 10'd60, 10'd124, 10'd284, 10'd316, 10'd412, 10'd444, 10'd476, 10'd24, 10'd536, 10'd792:
            begin c.rd = ra; c.ua_ = 0; c.us_ = 1; c.xw = x == 10'd792; end
          10'd824, 10'd26, 10'd922, 10'd954: begin c.rd = ra; c.ua_ = 0; c.ub_ = 0; c.us_ = 1; c.xw = x == 10'd824; end
          10'd0, 10'd32: begin c.wg = 0; c.crw = 8'h80 >> rt[4:2]; end
          10'd235, 10'd75, 10'd11, 10'd491, 10'd459: c.unit = 1;
          10'd23, 10'd87, 10'd279, 10'd343, 10'd535, 10'd599, 10'd20:
            begin c.unit = 2; c.wg = x < 10'd535 || x == 10'd20; c.ua_ = ra != 0; c.crw = 0; end
          10'd151, 10'd215, 10'd407, 10'd663, 10'd727, 10'd918, 10'd662, 10'd983, 10'd150:
            begin c.unit = 2; c.wg = 0; c.ua_ = ra != 0; c.us_ = x < 10'd663 || x == 10'd918 || x == 10'd662 || x == 10'd150;
                  c.crw = x == 10'd150 ? 8'h80 : 8'h00; end
          10'd339, 10'd371: begin c.unit = 4; c.ua_ = 0; c.ub_ = 0; c.crw = 0;
                  c.lrr = {w[15:11], w[20:16]} == 10'd8; c.ctrr = {w[15:11], w[20:16]} == 10'd9; end
          10'd467: begin c.unit = 4; c.wg = 0; c.ua_ = 0; c.ub_ = 0; c.us_ = 1; c.crw = 0;
                  c.lrw = {w[15:11], w[20:16]} == 10'd8; c.ctrw = {w[15:11], w[20:16]} == 10'd9;
                  c.ser = {w[15:11], w[20:16]} == 10'd1 || {w[15:11], w[20:16]} >= 10'd528; end
          10'd19: begin c.unit = 4; c.ua_ = 0; c.ub_ = 0; c.crr = 8'hFF; c.crw = 0; end
          10'd144: begin c.unit = 4; c.wg = 0; c.ua_ = 0; c.ub_ = 0; c.us_ = 1; c.crw = w[19:12]; end
          10'd83, 10'd146, 10'd210, 10'd242, 10'd306, 10'd598, 10'd854, 10'd978, 10'd1010, 10'd512, 10'd597, 10'd533, 10'd661, 10'd725:
            begin c.unit = 4; c.ser = 1; end
          default: begin
            c.xw = oe || x[8:0] == 9'd8 || x[8:0] == 9'd10 || x[8:0] == 9'd136 || x[8:0] == 9'd138 ||
                   x[8:0] == 9'd200 || x[8:0] == 9'd202 || x[8:0] == 9'd232 || x[8:0] == 9'd234;
            c.car = x[8:0] == 9'd136 || x[8:0] == 9'd138 || x[8:0] == 9'd200 || x[8:0] == 9'd202 ||
                    x[8:0] == 9'd232 || x[8:0] == 9'd234;
          end
        endcase
      end
      default: c.ser = 1;
    endcase
    c.nrd = 2'(c.ua_) + 2'(c.ub_) + 2'(c.us_);
    return c;
  endfunction

  // ---------------------------------------------------------------- queue
  // four entries, head in entry 0; pop 0-2, push 1 (or 2 with FETCH2) after
  // the live entries, as ppc_fetchq.sv
  logic [3:0][31:0] qi, qp;
  cls_t [3:0]       qc;
  logic [2:0]       cnt;
  cls_t [1:0]       wc;
  always_comb begin
    wc[0] = classify(w_insn[0]);
    wc[1] = classify(w_insn[1]);
  end
  wire [2:0] npush = {2'd0, push[0]} + ((FETCH2 != 0) ? {2'd0, push[1]} : 3'd0);
  logic [3:0][2:0] src;                  // the entry after the pop: k + pop, or a pushed word
  always_comb
    for (int k = 0; k < 4; k++) src[k] = 3'(k) + {1'b0, pop};
  always_ff @(posedge clk) begin
    if (rst || clear) cnt <= 3'd0;
    else cnt <= cnt + npush - {1'b0, pop};
    for (int k = 0; k < 4; k++) begin
      if (src[k] < cnt) begin
        qi[k] <= qi[src[k][1:0]]; qp[k] <= qp[src[k][1:0]]; qc[k] <= qc[src[k][1:0]];
      end else if (push[0] && src[k] == cnt) begin
        qi[k] <= w_insn[0]; qp[k] <= w_pc[0]; qc[k] <= wc[0];
      end else if (FETCH2 != 0 && push[1] && src[k] == cnt + 3'd1) begin
        qi[k] <= w_insn[1]; qp[k] <= w_pc[1]; qc[k] <= wc[1];
      end
    end
  end

  // ---------------------------------------------------------------- pair check
  cls_t c0, c1;
  assign c0 = qc[0];
  assign c1 = qc[1];
  wire raw = c0.wg && ((c1.ua_ && qi[1][20:16] == c0.rd) || (c1.ub_ && qi[1][15:11] == c0.rd) ||
                       (c1.us_ && qi[1][25:21] == c0.rd));
  wire waw = c0.wg && c1.wg && c0.rd == c1.rd;
  wire crh = (c0.crw & (c1.crr | c1.crw)) != 8'd0;
  wire xh  = c0.xw && (c1.xw || c1.car);
  wire sh  = (c0.lrw && (c1.lrr || c1.lrw)) || (c0.ctrw && (c1.ctrr || c1.ctrw));
  wire dep = raw || waw || crh || xh || sh;
  wire both = cnt >= 3'd2 && c0.one && c1.one && !c0.ser && !c1.ser && !dep;
  logic units;
  always_comb begin
    case (VAR)
      1:       units = c0.unit != c1.unit && !(c0.unit <= 3'd1 && c1.unit <= 3'd1);
      2:       units = (c0.unit == 3'd0 && c1.unit >= 3'd2) || (c1.unit == 3'd0 && c0.unit >= 3'd2);
      3:       units = ((c0.unit == 3'd0 && c1.unit >= 3'd2 && !c1.wg) || (c1.unit == 3'd0 && c0.unit >= 3'd2 && !c0.wg)) &&
                       {1'b0, c0.nrd} + {1'b0, c1.nrd} <= 3'd3;
      4:       units = c1.unit == 3'd5 && c0.unit != 3'd5;
      default: units = 1'b0;
    endcase
  end
  wire pair = VAR != 0 && both && units;
  wire take = cnt != 3'd0 && !hold;
  assign pop = !take ? 2'd0 : pair ? 2'd2 : 2'd1;

  // ---------------------------------------------------------------- decode
  uop_t d0, d1;
  ppc_decode u_d0 (.insn (qi[0]), .pc (qp[0]), .step (5'd0), .pr (1'b0), .u (d0));
  generate
    if (VAR == 1 || VAR == 2 || VAR == 3) begin : g_d1
      ppc_decode u_d1 (.insn (qi[1]), .pc (qp[1]), .step (5'd0), .pr (1'b0), .u (d1));
    end else if (VAR == 4) begin : g_db
      // branch fields only
      always_comb begin
        d1 = '0;
        d1.valid = 1'b1; d1.pc = qp[1]; d1.insn = qi[1]; d1.op = OP_BRANCH;
        d1.bkind = qi[1][31:26] == 6'd18 ? BR_B : qi[1][31:26] == 6'd16 ? BR_BC : qi[1][10:1] == 10'd16 ? BR_BCLR : BR_BCCTR;
        d1.bo = qi[1][25:21]; d1.bi = qi[1][20:16]; d1.lk = qi[1][0]; d1.aa = qi[1][1];
        d1.imm = qi[1][31:26] == 6'd18 ? {{6{qi[1][25]}}, qi[1][25:2], 2'b00} : {{16{qi[1][15]}}, qi[1][15:2], 2'b00};
      end
    end else begin : g_none
      assign d1 = '0;
    end
  endgenerate

  // steering: slot B takes the single-cycle integer op (VAR 2, 3) or the
  // unit that is not slot A's (VAR 1: entry 1 unless entry 0 is the one A
  // cannot take, here: entry 0 a load / store / FP / branch keeps A)
  wire to_b0 = pair && ((VAR == 2 || VAR == 3) ? c0.unit == 3'd0 : (VAR == 1) ? c0.unit <= 3'd1 && c1.unit >= 3'd2 : 1'b0);
  // slot B keeps only what a simple integer op needs (VAR 2, 3)
  function automatic uop_t simple(input uop_t u);
    uop_t s;
    s = '0;
    s.valid = u.valid; s.op = u.op; s.ra = u.ra; s.rb = u.rb; s.rs = u.rs; s.a_zero = u.a_zero;
    s.x_from_s = u.x_from_s; s.a_used = u.a_used; s.b_used = u.b_used; s.s_used = u.s_used; s.bsel = u.bsel;
    s.imm = u.imm; s.cin = u.cin; s.rd_we = u.rd_we; s.rd = u.rd; s.set_ca = u.set_ca; s.oe = u.oe; s.rc = u.rc;
    s.crf = u.crf; s.mb = u.mb; s.me = u.me; s.rlw_insert = u.rlw_insert;
    return s;
  endfunction
  always_ff @(posedge clk) begin
    if (take) begin
      o_pc0 <= qp[0]; o_pc1 <= pair ? qp[1] : 32'hFFFF_FFFF;
      ua   <= to_b0 ? d1 : d0;
      ub_v <= pair;
      case (VAR)
        2, 3:    ub <= to_b0 ? simple(d0) : simple(d1);
        1:       ub <= to_b0 ? d0 : d1;
        4:       ub <= d1;
        default: ub <= '0;
      endcase
    end else ub_v <= 1'b0;
    // the read ports' fields (VAR 3): A's x and y fields, then B's x
    if (VAR == 3) begin
      pmap[0] <= c0.us_ && !c0.ua_ ? 3'd2 : 3'd0;
      pmap[1] <= c0.ub_ ? 3'd1 : (c1.us_ && !c1.ua_ ? 3'd5 : 3'd3);
      pmap[2] <= c1.ub_ ? 3'd4 : c1.us_ ? 3'd5 : 3'd3;
    end else pmap <= '0;
  end

endmodule
