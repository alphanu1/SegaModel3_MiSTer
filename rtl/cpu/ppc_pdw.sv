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
// The fetch queue's predecode of one arriving word (pd_t, ppc_pkg.sv), moved
// out of ppc_core.sv unchanged so that two-word fetch (D-30) can build it
// twice. w is the word; tg is MSR[TGPR] as fetch sees it; mark says the word
// stands for a fetch fault (its fields are then cleared). See ppc_core.sv,
// "Predecode as the word arrives", for what each field is for.

module ppc_pdw
  import ppc_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic [31:0] w,
  input  logic        tg,
  input  logic        mark,
  output pd_t         pd
);
  uop_t        pd_u;
  ppc_decode #(.FPA (1'b0)) u_pdec (.insn (w), .pc (32'd0), .step (5'd0), .pr (1'b0), .u (pd_u));
  // The fields above that came from that decoder (a_used ... op, and the
  // immediate's form and result kind formed from them, pd_immk and pd_rk
  // below) are taken from its flat tables, ppc_pdec, fewer LUT levels from
  // w; the decoder instance stays for the checks below, where it is
  // compared with the tables on every word, and is trimmed in synthesis
  logic        pr_ua, pr_ub, pr_us, pr_xs, pr_az, pr_yreg, pr_rc, pr_xer, pr_fpw, pr_fhi, pr_fuc, pr_fdbl, pr_ill;
  logic [2:0]  pr_immk;
  logic [4:0]  pr_rk;
  logic [5:0]  pr_op;
  ppc_pdec u_pdr (
    .w (w), .ua (pr_ua), .ub (pr_ub), .us (pr_us), .xs (pr_xs), .az (pr_az), .yreg (pr_yreg), .rc (pr_rc),
    .xer (pr_xer), .fpw (pr_fpw), .fhi (pr_fhi), .fuc (pr_fuc), .fdbl (pr_fdbl), .ill (pr_ill),
    .immk (pr_immk), .rk (pr_rk), .op (pr_op)
  );
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && {pr_ua, pr_ub, pr_us, pr_xs, pr_az, pr_yreg, pr_rc, pr_xer, pr_fpw, pr_fhi, pr_fuc, pr_fdbl, pr_ill,
                 pr_immk, pr_rk, pr_op} !=
                {pd_u.a_used, pd_u.b_used, pd_u.s_used, pd_u.x_from_s, pd_u.a_zero, pd_u.bsel == BSEL_RB, pd_u.rc,
                 pd_u.set_ca || pd_u.oe, pd_u.fpw, pd_u.fhi, pd_u.fuc, pd_u.fdbl, pd_u.op == OP_ILLEGAL,
                 pd_immk, pd_rk, 6'(pd_u.op)})
      $fatal(1, "predecode tables against the decoder, word %h", w);
`endif
  // FP arithmetic is predecoded apart, from the raw word: which FPRs an op of
  // opcode 59 / 63 reads and writes, and whether it writes the CR. Shallow
  // (an encoding the 603e does not have only costs a stall); the decoder
  // above leaves 59 / 63 alone, so FP adds no depth to the integer fields
  // on the fetch path, which limits the full build (R28). FP loads and
  // stores are predecoded by the decoder, as the integer ones.
  wire  [4:0]  pa5 = w[5:1];
  wire         pa_op = pd_op == 6'd59 || pd_op == 6'd63;
  wire         pa_a  = pa5 == 5'd18 || pa5 == 5'd20 || pa5 == 5'd21 || pa5 >= 5'd23;   // A form
  wire         pa_xb = pd_x == 10'd12 || pd_x == 10'd14 || pd_x == 10'd15 ||          // frsp fctiw(z)
                       pd_x == 10'd40 || pd_x == 10'd72 || pd_x == 10'd136 || pd_x == 10'd264;   // moves
  wire         pa_cmp = pd_x == 10'd0 || pd_x == 10'd32;
  wire         pa_mcrfs = pd_x == 10'd64;
  wire         pa_ua = pa_op && (pa_a ? (pa5 != 5'd24 && pa5 != 5'd26) : pa_cmp);
  wire         pa_ub = pa_op && (pa_a ? pa5 != 5'd25 : (pa_cmp || pa_xb || pd_x == 10'd711));
  wire         pa_uc = pa_op && pa_a && (pa5 == 5'd23 || pa5 == 5'd25 || pa5 >= 5'd28);
  wire         pa_w  = pa_op && (pa_a || pa_xb || pd_x == 10'd583);
  wire         pa_rc = pa_op && w[0] && !(!pa_a && (pa_cmp || pa_mcrfs));
  wire         pa_d  = w[28];                     // opcode 63 (59: single)
  wire         pa_mov = pd_x == 10'd40 || pd_x == 10'd72 || pd_x == 10'd136 || pd_x == 10'd264;
  // fadd fsub fmul and the multiply-adds: the single unit's ops
  wire         pa_fs = pa5 == 5'd20 || pa5 == 5'd21 || pa5 == 5'd25 || pa5 >= 5'd28;
  wire         pa_ffma = pa_op && (pa_fs || (pa_d && (pa5 == 5'd26 || pd_x == 10'd12 || pd_x == 10'd14 ||
                                                     pd_x == 10'd15)));
  // a store: frS on the C port
  wire         pd_fst = (pd_op >= 6'd52 && pd_op <= 6'd55) ||
                        (pd_op == 6'd31 && (pd_x == 10'd663 || pd_x == 10'd695 || pd_x == 10'd727 ||
                                            pd_x == 10'd759 || pd_x == 10'd983));
  wire  [5:0]  pd_op = w[31:26];
  wire  [9:0]  pd_x  = w[10:1];
  // 3: lfdu / lfdux, three uops (ppc_decode.sv)
  wire  [1:0]  pd_crack =
          (pd_op == 6'd46 || pd_op == 6'd47) ? 2'd2 :
          (pd_op == 6'd51 || (pd_op == 6'd31 && pd_x == 10'd631)) ? 2'd3 :
          (pd_op == 6'd33 || pd_op == 6'd35 || pd_op == 6'd41 || pd_op == 6'd43 ||
           pd_op == 6'd49 || pd_op == 6'd50 || pd_op == 6'd54 || pd_op == 6'd55 ||
           (pd_op == 6'd31 && (pd_x == 10'd55 || pd_x == 10'd119 || pd_x == 10'd311 || pd_x == 10'd375 ||
                               pd_x == 10'd567 || pd_x == 10'd599 || pd_x == 10'd727 || pd_x == 10'd759))) ? 2'd1 : 2'd0;

  // SPR number to its home: mf_sel picks mfspr's source (0 generic/SRR, 1 XER,
  // 2 LR, 3 CTR, 4 DEC, 5 TBL, 6 TBU, 7 PVR); spr_idx is the SPR array's
  // entry (0 SRR0 and 1 SRR1 are flip-flops, not the array).
  wire  [9:0]  pd_spr = {w[15:11], w[20:16]};
  logic        pd_spr_hit;
  logic [5:0]  pd_spr_idx;
  logic [2:0]  pd_mf_sel;
  always_comb begin
    case (pd_spr)
      10'd1:   pd_mf_sel = 3'd1;  10'd8:   pd_mf_sel = 3'd2;  10'd9:   pd_mf_sel = 3'd3;
      10'd22:  pd_mf_sel = 3'd4;  10'd268: pd_mf_sel = 3'd5;  10'd269: pd_mf_sel = 3'd6;
      10'd287: pd_mf_sel = 3'd7;  default: pd_mf_sel = 3'd0;
    endcase
  end
  always_comb begin
    pd_spr_hit = 1'b1;
    case (pd_spr)
      10'd26:   pd_spr_idx = 6'd0;    // SRR0 (flip-flops, not the array)
      10'd27:   pd_spr_idx = 6'd1;    // SRR1
      10'd272:  pd_spr_idx = 6'd2;    10'd273: pd_spr_idx = 6'd3;      // SPRG0-3
      10'd274:  pd_spr_idx = 6'd4;    10'd275: pd_spr_idx = 6'd5;
      10'd18:   pd_spr_idx = 6'd6;    // DSISR
      10'd19:   pd_spr_idx = 6'd7;    // DAR
      10'd25:   pd_spr_idx = 6'd8;    // SDR1
      10'd1008: pd_spr_idx = 6'd9;    // HID0
      10'd1009: pd_spr_idx = 6'd10;   // HID1
      10'd1010: pd_spr_idx = 6'd11;   // IABR
      10'd282:  pd_spr_idx = 6'd12;   // EAR
      10'd976:  pd_spr_idx = 6'd13;   10'd977: pd_spr_idx = 6'd14;     // DMISS DCMP
      10'd978:  pd_spr_idx = 6'd15;   10'd979: pd_spr_idx = 6'd16;     // HASH1 HASH2
      10'd980:  pd_spr_idx = 6'd17;   10'd981: pd_spr_idx = 6'd18;     // IMISS ICMP
      10'd982:  pd_spr_idx = 6'd19;   // RPA
      default: begin
        if (pd_spr >= 10'd528 && pd_spr <= 10'd543)
          pd_spr_idx = 6'd20 + {2'b00, pd_spr[3:0]};                 // IBAT0U .. DBAT3L
        else begin
          pd_spr_idx = 6'd63;
          pd_spr_hit = 1'b0;
        end
      end
    endcase
  end

  // the immediate's form, as the decoder builds id_u.imm for y
  logic [2:0] pd_immk;
  always_comb begin
    pd_immk = 3'd0;
    case (pd_op)
      6'd10, 6'd24, 6'd26, 6'd28: pd_immk = 3'd1;     // cmpli ori xori andi.
      6'd15, 6'd25, 6'd27, 6'd29: pd_immk = 3'd2;     // addis oris xoris andis.
      6'd20, 6'd21:               pd_immk = 3'd3;     // rlwimi rlwinm
      6'd46, 6'd47:               pd_immk = 3'd4;     // lmw stmw
      6'd31: begin
        if (pd_x == 10'd824) pd_immk = 3'd3;          // srawi
        if (pd_x == 10'd210 || pd_x == 10'd595) pd_immk = 3'd7;   // mtsr mfsr: SR || 28 zeros
        if (pd_u.bsel == BSEL_ZERO)   pd_immk = 3'd5;
        if (pd_u.bsel == BSEL_MINUS1) pd_immk = 3'd6;
      end
      default: ;
    endcase
  end

  // Result kind, five bits in the queue; ID expands it into the one-hot
  // result select and the logical truth table (one LUT level from the queue
  // register, where storing those fourteen bits cost 36 queue flip-flops):
  //   0 sum (add, subtract, compare, EA)  1 slw  2 srw  3 sraw  4 rotate
  //   5 cntlzw  6 extsb  7 extsh  8 multiply/divide  9 move-from
  //   10 none (segment and TLB instructions)  16-23 and or xor nand nor eqv andc orc
  logic [4:0] pd_rk;
  always_comb begin
    case (pd_u.op)
      OP_SLW:    pd_rk = 5'd1;
      OP_SRW:    pd_rk = 5'd2;
      OP_SRAW:   pd_rk = 5'd3;
      OP_RLW:    pd_rk = 5'd4;
      OP_CNTLZW: pd_rk = 5'd5;
      OP_EXTSB:  pd_rk = 5'd6;
      OP_EXTSH:  pd_rk = 5'd7;
      OP_MUL, OP_MULH, OP_MULHU, OP_DIV, OP_DIVU: pd_rk = 5'd8;
      OP_MFSPR, OP_MFCR, OP_MFMSR:               pd_rk = 5'd9;
      OP_MTSR, OP_MFSR, OP_TLBIE, OP_TLBL, OP_FP: pd_rk = 5'd10;  // no ALU group: y is used raw
      OP_AND:    pd_rk = 5'd16;
      OP_OR:     pd_rk = 5'd17;
      OP_XOR:    pd_rk = 5'd18;
      OP_NAND:   pd_rk = 5'd19;
      OP_NOR:    pd_rk = 5'd20;
      OP_EQV:    pd_rk = 5'd21;
      OP_ANDC:   pd_rk = 5'd22;
      OP_ORC:    pd_rk = 5'd23;
      default:   pd_rk = 5'd0;
    endcase
  end

  // The op compares behind the special-register bits, from the raw word: the
  // decoder's op is a four-level mux on the fetch path (w to the queue,
  // R38), and these compares sat behind it. Same values as from pd_u.op
  // (checked in simulation below).
  wire         r_31    = pd_op == 6'd31, r_19 = pd_op == 6'd19;
  wire         r_bc    = pd_op == 6'd16 || (r_19 && pd_x == 10'd16);          // bc bclr
  wire         r_br    = r_bc || pd_op == 6'd18 || (r_19 && pd_x == 10'd528); // and b bcctr
  wire         r_mtspr = r_31 && pd_x == 10'd467;
  wire         r_mtcrf = r_31 && pd_x == 10'd144;
  wire         r_mcrxr = r_31 && pd_x == 10'd512;
  wire         r_stwcx = r_31 && pd_x == 10'd150;
  wire         r_crlog = r_19 && (pd_x == 10'd0 || pd_x == 10'd33 || pd_x == 10'd129 || pd_x == 10'd193 ||
                                  pd_x == 10'd225 || pd_x == 10'd257 || pd_x == 10'd289 || pd_x == 10'd417 ||
                                  pd_x == 10'd449);
  wire         r_cmp   = pd_op == 6'd10 || pd_op == 6'd11 || (r_31 && (pd_x == 10'd0 || pd_x == 10'd32));

  always_comb begin
    pd         = '0;
    pd.ua      = pr_ua;
    pd.ub      = pr_ub;
    pd.us      = pr_us;
    pd.crack   = pd_crack;
    pd.xs      = pr_xs;
    pd.az      = pr_az;
    pd.yreg    = pr_yreg;
    pd.spr_idx = pd_spr_idx;
    pd.mf_sel  = pd_mf_sel;
    pd.mfcr    = r_31 && pd_x == 10'd19;
    pd.mfmsr   = r_31 && pd_x == 10'd83;
    pd.immk    = pr_immk;
    pd.rk      = pr_rk;
    // the special registers this uop writes
    pd.w_cr    = pr_rc || r_cmp || r_crlog || r_mtcrf || r_mcrxr || r_stwcx;
    pd.w_xer   = pr_xer || r_mcrxr || (r_mtspr && pd_spr == 10'd1);
    pd.w_lr    = (r_br && w[0]) || (r_mtspr && pd_spr == 10'd8);
    pd.w_ctr   = (r_bc && !w[23]) || (r_mtspr && pd_spr == 10'd9);
    pd.w_spr   = r_mtspr && pd_spr_hit && pd_spr_idx != 6'd15 && pd_spr_idx != 6'd16;   // HASH1/2 read-only
    // fetch-side state, from registers (no logic on the word's path but the
    // three shadow compares)
    pd.sh      = {3{tg}} & {w[20:18] == 3'd0, w[15:13] == 3'd0, w[25:23] == 3'd0};
    pd.ifault  = mark;
    pd.l0      = pd_crack == 2'd0 || (pd_crack == 2'd2 && w[25:21] == 5'd31);
    pd.ck      = {r_stwcx, r_mtcrf, (r_crlog && pd_x == 10'd0) || r_mcrxr, r_crlog && pd_x != 10'd0};
    // FP: nothing for a fault marker, whose word is not run
    pd.fua     = pa_ua && !mark;
    pd.fub     = pa_ub && !mark;
    pd.fuc     = (pa_uc || pr_fuc) && !mark;
    pd.fpw     = (pa_w || pr_fpw || pr_fhi) && !mark;   // lfd: frD from its first uop on
    pd.fci     = pd_fst ? w[25:21] : w[10:6];
    pd.fwcr    = pa_op && (pa_rc || (!pa_a && (pa_cmp || pa_mcrfs)));
    pd.fck1    = pa_op && (pa_rc || (!pa_a && pa_mcrfs));
    pd.hh      = pd_op == 6'd31 && (pd_x == 10'd339 || pd_x == 10'd371) && (pd_spr == 10'd978 || pd_spr == 10'd979);
    pd.fdx     = pr_fdbl;
    pd.ra0     = w[20:16] == 5'd0;
    pd.ill     = pr_ill && !pa_op;
    pd.pop     = alu_op_t'(pr_op);
    // (ppc_decode's fp_arith and fp_ls, as checked in ID below)
    pd.ffma    = pa_ffma && !mark;
    pd.fdiv    = pa_op && (pa5 == 5'd18 || (pa5 == 5'd24 && !pa_d)) && !mark;
    pd.fmsc    = pa_op && pa_d && (pa5 == 5'd23 || pa_cmp || pa_mov || pd_x == 10'd583) && !mark;
    pd.fmw     = pa_op && pa_d && (pa5 == 5'd23 || pa_mov || pd_x == 10'd583) && !mark;
    pd.ffsx    = pa_op && pa_d && (pd_x == 10'd711 || pd_x == 10'd134 || pd_x == 10'd70 || pd_x == 10'd38 ||
                                     pa_mcrfs) && !mark;
    pd.ff2c    = ((pa_op && pa_d && pa_cmp) ||
                    pd_op == 6'd52 || pd_op == 6'd53 || (pd_op == 6'd31 && (pd_x == 10'd663 || pd_x == 10'd695))) &&
                   !mark;
    pd.frq     = pa_op && pa_d && pa5 == 5'd26 && !mark;
    pd.frs     = pa_op && !pa_d && pa_fs && !mark;
  end
`ifdef VERILATOR
  // the predecode's raw-word compares against the decoder's op, every cycle
  always_ff @(posedge clk)
    if (!rst && (
        pd.w_cr  != (pd_u.rc || pd_u.op == OP_CMP || pd_u.op == OP_CMPL || pd_u.op == OP_CRLOG ||
                       pd_u.op == OP_MTCRF || pd_u.op == OP_MCRXR || (pd_u.op == OP_STORE && pd_u.cond)) ||
        pd.w_xer != (pd_u.set_ca || pd_u.oe || pd_u.op == OP_MCRXR || (pd_u.op == OP_MTSPR && pd_spr == 10'd1)) ||
        pd.w_lr  != ((pd_u.op == OP_BRANCH && pd_u.lk) || (pd_u.op == OP_MTSPR && pd_spr == 10'd8)) ||
        pd.w_ctr != ((pd_u.op == OP_BRANCH && pd_u.bkind != BR_B && pd_u.bkind != BR_BCCTR && !pd_u.bo[2]) ||
                       (pd_u.op == OP_MTSPR && pd_spr == 10'd9)) ||
        pd.w_spr != (pd_u.op == OP_MTSPR && pd_spr_hit && pd_spr_idx != 6'd15 && pd_spr_idx != 6'd16) ||
        pd.mfcr  != (pd_u.op == OP_MFCR) || pd.mfmsr != (pd_u.op == OP_MFMSR) ||
        pd.ck    != {pd_u.op == OP_STORE && pd_u.cond, pd_u.op == OP_MTCRF,
                       (pd_u.op == OP_CRLOG && pd_x == 10'd0) || pd_u.op == OP_MCRXR,
                       pd_u.op == OP_CRLOG && pd_x != 10'd0}))
      $fatal(1, "predecode raw compares against the decoder, word %h", w);
`endif

endmodule
