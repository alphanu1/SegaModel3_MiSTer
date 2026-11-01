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
// PowerPC 603e integer decode, one uop per cycle.
//
// `step` counts uops within the instruction; `last` says this is the final one.
// Cracked instructions, so that no uop writes more than one GPR:
//
//   update loads (lbzu, lhzu, lhau, lwzu and their x forms)
//       step 0: the load into rD        step 1: rA = step 0's EA
//   lmw rD,d(rA)    step n: load r(rD+n) from rA + d + 4n, n = 0 .. 31-rD
//   stmw rS,d(rA)   step n: store r(rS+n) to rA + d + 4n
//
// An update load's step 1 takes step 0's EA as x (ppc_core.sv: EX's sum as
// step 0 leaves EX) and adds the immediate 0. It reads no register: rD may be
// rB (a valid form, PEM 8 lbzux: EA <- (rA) + (rB), rD <- MEM(EA), rA <- EA),
// and step 0 has written rD by then, so rA + rB read again would be rA + the
// loaded value. Update stores write their EA to rA within the store uop. Every non-register
// y operand is in imm, BSEL_ZERO and BSEL_MINUS1 included (0 and -1), and so is
// the rotate amount of rlwinm / rlwimi, so the core captures y with one mux.
//
// Segment and TLB instructions carry their operand in y: rB for mtsrin, mfsrin,
// tlbie, tlbld and tlbli, the SR number in y[31:28] for mtsr and mfsr (the
// immediate is SR || 28 zeros), so the core reads the segment registers and
// indexes the TLBs from one register. Cache-block instructions, sync, eieio and
// tlbsync are no-ops (no caches yet; tlbsync with TLBISYNC negated, 5.4.3.2);
// isync refetches.
//
// Floating point (PEM chapter 8, 603e table 6-5). FP loads and stores are
// load and store uops with a word data bus, so a double takes two beats:
//
//   lfs, lfsx           one uop; the word converted to double in MEM/WB
//   lfsu, lfsux         step 0: the load     step 1: rA = EA
//   lfd, lfdx           step 0: the high word at EA (held)
//                       step 1: the low word at EA + 4; frD written
//   lfdu, lfdux         as lfd, then step 2: rA = EA
//   stfs(u)(x), stfiwx  one uop (an update store writes rA within it)
//   stfd(u)(x)          step 0: the high word at EA (stfdu: rA = EA)
//                       step 1: the low word at EA + 4
//
// A double's step 1 takes its EA from step 0's (ppc_core.sv: EX's sum, or a
// register holding it after a replay) plus the immediate 4, so it needs
// neither rA (an update store has already written it) nor rB again.
// Arithmetic, moves, compares and the FPSCR instructions are OP_FP with the
// unit (funit) and its operation (fop, fsop) for ppc_fpu_pkg's units. fsqrt
// and fsqrts are not on the 603e and are illegal (603e table B-1, 4.5.7.2),
// as is every other unassigned encoding of opcodes 59 and 63, whatever
// MSR[FP] is.

module ppc_decode
  import ppc_pkg::*, ppc_fpu_pkg::*;
#(
  // 0: opcodes 59 and 63 decode as illegal, as before the FPU (the core's
  // fetch predecode, which has a shallow FP decoder of its own beside this)
  parameter bit FPA = 1'b1
)
(
  input  logic [31:0] insn,
  input  logic [31:0] pc,
  input  logic [4:0]  step,
  input  logic        pr,           // MSR[PR]: dcbi is supervisor-only (603e 3.7.1)
  output uop_t        u
);

  wire [5:0]  opcd = insn[31:26];
  wire [4:0]  f_d  = insn[25:21];          // rD / rS / BO / TO / crbD
  wire [4:0]  f_a  = insn[20:16];          // rA / BI / crbA
  wire [4:0]  f_b  = insn[15:11];          // rB / SH / crbB
  wire [9:0]  xo10 = insn[10:1];
  wire [8:0]  xo9  = insn[9:1];
  wire        oe   = insn[10];
  wire        rcb  = insn[0];
  wire [31:0] simm = {{16{insn[15]}}, insn[15:0]};
  wire [31:0] uimm = {16'd0, insn[15:0]};
  wire [31:0] shim = {insn[15:0], 16'd0};
  wire [31:0] step4 = {25'd0, step, 2'b00};

  // XO-form arithmetic: the nine-bit opcode with OE in bit 10.
  logic is_xo;
  always_comb begin
    case (xo9)
      9'd8, 9'd10, 9'd11, 9'd40, 9'd75, 9'd104, 9'd136, 9'd138, 9'd200,
      9'd202, 9'd232, 9'd234, 9'd235, 9'd266, 9'd459, 9'd491: is_xo = 1'b1;
      default: is_xo = 1'b0;
    endcase
  end

  // One FP load or store: st (store), dbl (double), upd (update form), x
  // (indexed), iw (stfiwx). Called from the comb block below; writes u.
  task automatic fp_ls(input logic st, input logic dbl, input logic upd, input logic x, input logic iw);
    u.fp    = 1'b1;
    u.msize = 2'd2;
    u.fdbl  = dbl;
    u.fsl   = !dbl && !iw;
    u.fiw   = iw;
    u.rd_we = 1'b0;
    if (!st && upd && step == (dbl ? 5'd2 : 5'd1)) begin
      // the update: rA = rA + d (or rB). Neither has been written by the
      // earlier steps (frD is an FPR), so they are read again here
      u.op = OP_ADD; u.a_used = 1'b1; u.b_used = x; u.bsel = x ? BSEL_RB : BSEL_IMM; u.imm = simm;
      u.rd = f_a; u.rd_we = 1'b1;
    end else begin
      u.op  = st ? OP_STORE : OP_LOAD;
      u.fld = !st;
      u.fst = st;
      u.fuc = st;                                     // frS, read on the C port
      if (dbl && step != 5'd0) begin
        // the low word at step 0's EA + 4 (the core supplies x)
        u.a_zero = 1'b1; u.bsel = BSEL_IMM; u.imm = 32'd4;
        u.fpw = !st;
        u.last = !(upd && !st);
      end else begin
        u.a_used = (f_a != 5'd0) || upd; u.a_zero = !upd;
        u.b_used = x; u.bsel = x ? BSEL_RB : BSEL_IMM; u.imm = simm;
        u.fhi = dbl && !st;
        u.fpw = !dbl && !st;
        u.last = !dbl && !(upd && !st);
        if (st && upd) begin u.st_update = 1'b1; u.rd = f_a; u.rd_we = 1'b1; end
      end
    end
  endtask

  // Opcodes 59 (s = 0: single) and 63 (s = 1). The A form is told from the X
  // form by xo[5:1]: no X-form extended opcode of opcode 63 has its low five
  // bits in the A-form set (PEM appendix A).
  task automatic fp_arith(input logic dbl);
    logic ok;
    ok = 1'b1;
    u.op   = OP_FP;
    u.fp   = 1'b1;
    u.fsgl = !dbl;
    u.frc  = rcb;
    u.fpw  = 1'b1;
    u.fsu  = 1'b1;
    case (insn[5:1])
      5'd18: begin u.funit = 2'd2; u.fop = 4'(DIV_DIV);   u.fua = 1'b1; u.fub = 1'b1; end   // fdiv(s)
      5'd20: begin u.funit = 2'd1; u.fop = 4'(FMA_SUB);   u.fua = 1'b1; u.fub = 1'b1; end   // fsub(s)
      5'd21: begin u.funit = 2'd1; u.fop = 4'(FMA_ADD);   u.fua = 1'b1; u.fub = 1'b1; end   // fadd(s)
      5'd25: begin u.funit = 2'd1; u.fop = 4'(FMA_MUL);   u.fua = 1'b1; u.fuc = 1'b1; end   // fmul(s)
      5'd28: begin u.funit = 2'd1; u.fop = 4'(FMA_MSUB);  u.fua = 1'b1; u.fub = 1'b1; u.fuc = 1'b1; end
      5'd29: begin u.funit = 2'd1; u.fop = 4'(FMA_MADD);  u.fua = 1'b1; u.fub = 1'b1; u.fuc = 1'b1; end
      5'd30: begin u.funit = 2'd1; u.fop = 4'(FMA_NMSUB); u.fua = 1'b1; u.fub = 1'b1; u.fuc = 1'b1; end
      5'd31: begin u.funit = 2'd1; u.fop = 4'(FMA_NMADD); u.fua = 1'b1; u.fub = 1'b1; u.fuc = 1'b1; end
      5'd24: if (!dbl) begin u.funit = 2'd2; u.fop = 4'(DIV_RES); u.fub = 1'b1; end else ok = 1'b0;     // fres
      5'd26: if (dbl) begin u.funit = 2'd1; u.fop = 4'(FMA_RSQRTE); u.fub = 1'b1; end else ok = 1'b0;   // frsqrte (fast port)
      5'd23: if (dbl) begin                                                                              // fsel
               u.funit = 2'd3; u.fop = 4'(MSC_FSEL); u.fua = 1'b1; u.fub = 1'b1; u.fuc = 1'b1;
               u.fsu = 1'b0; u.fser = rcb;
             end else ok = 1'b0;
      default: begin
        ok = dbl;                                     // opcode 59 has no X form
        u.fsu = 1'b1;
        case (xo10)
          10'd12: begin u.funit = 2'd1; u.fop = 4'(FMA_RSP);   u.fub = 1'b1; end                  // frsp
          10'd14: begin u.funit = 2'd1; u.fop = 4'(FMA_CTIW);  u.fub = 1'b1; end                  // fctiw
          10'd15: begin u.funit = 2'd1; u.fop = 4'(FMA_CTIWZ); u.fub = 1'b1; end                  // fctiwz
          10'd0, 10'd32: begin                                                                    // fcmpu fcmpo
            u.funit = 2'd3; u.fop = xo10[5] ? 4'(MSC_FCMPO) : 4'(MSC_FCMPU);
            u.fua = 1'b1; u.fub = 1'b1; u.fcmp = 1'b1; u.fpw = 1'b0; u.frc = 1'b0; u.fser = 1'b1;
          end
          10'd40, 10'd72, 10'd136, 10'd264: begin                                                 // fneg fmr fnabs fabs
            u.funit = 2'd3; u.fub = 1'b1; u.fsu = 1'b0; u.fser = rcb;
            u.fop = (xo10 == 10'd40) ? 4'(MSC_FNEG) : (xo10 == 10'd72) ? 4'(MSC_FMR) :
                    (xo10 == 10'd136) ? 4'(MSC_FNABS) : 4'(MSC_FABS);
          end
          10'd583: begin u.funit = 2'd3; u.fop = 4'(MSC_MFFS); u.fsu = 1'b0; u.fser = 1'b1; end   // mffs
          10'd711: begin u.fsop = 3'(FS_MTFSF);  u.fub = 1'b1; u.fpw = 1'b0; u.fser = 1'b1; end   // mtfsf
          10'd134: begin u.fsop = 3'(FS_MTFSFI); u.fpw = 1'b0; u.fser = 1'b1; end                 // mtfsfi
          10'd70:  begin u.fsop = 3'(FS_MTFSB0); u.fpw = 1'b0; u.fser = 1'b1; end                 // mtfsb0
          10'd38:  begin u.fsop = 3'(FS_MTFSB1); u.fpw = 1'b0; u.fser = 1'b1; end                 // mtfsb1
          10'd64:  begin u.fsop = 3'(FS_MCRFS);  u.fpw = 1'b0; u.frc = 1'b0; u.fser = 1'b1; end   // mcrfs
          default: ok = 1'b0;
        endcase
      end
    endcase
    // CR1 (Rc = 1) is a field write of field 1; mcrfs and the compares name theirs
    if (u.frc) u.crf = 3'd1;
    if (!ok) begin
      u.op = OP_ILLEGAL; u.fp = 1'b0; u.funit = 2'd0; u.fsop = 3'd0; u.fpw = 1'b0; u.fsu = 1'b0;
      u.fser = 1'b0; u.frc = 1'b0; u.fcmp = 1'b0; u.fua = 1'b0; u.fub = 1'b0; u.fuc = 1'b0;
    end
  endtask

  always_comb begin
    u          = '0;
    u.valid    = 1'b1;
    u.pc       = pc;
    u.insn     = insn;
    u.op       = OP_ILLEGAL;
    u.ra       = f_a;
    u.rb       = f_b;
    u.rs       = f_d;
    u.bsel     = BSEL_RB;
    u.cin      = CIN_ZERO;
    u.rd       = f_d;
    u.crf      = insn[25:23];
    u.crf_src  = insn[20:18];
    u.mb       = insn[10:6];
    u.me       = insn[5:1];
    u.bo       = f_d;
    u.bi       = f_a;
    u.lk       = insn[0];
    u.aa       = insn[1];
    u.crop     = insn[9:6];
    u.crbd     = f_d;
    u.crba     = f_a;
    u.crbb     = f_b;
    u.crm      = insn[19:12];
    u.spr      = {insn[15:11], insn[20:16]};
    u.last     = 1'b1;

    case (opcd)
      // ---------------------------------------------------------- immediates
      6'd3: begin                                     // twi
        u.op = OP_TRAP; u.a_used = 1'b1; u.bsel = BSEL_IMM; u.imm = simm;
      end
      6'd7: begin                                     // mulli
        u.op = OP_MUL; u.a_used = 1'b1; u.bsel = BSEL_IMM; u.imm = simm; u.rd_we = 1'b1;
      end
      6'd8: begin                                     // subfic
        u.op = OP_SUBF; u.a_used = 1'b1; u.bsel = BSEL_IMM; u.imm = simm;
        u.cin = CIN_ONE; u.set_ca = 1'b1; u.rd_we = 1'b1;
      end
      6'd10, 6'd11: begin                             // cmpli, cmpi
        u.op = (opcd == 6'd11) ? OP_CMP : OP_CMPL; u.a_used = 1'b1;
        u.bsel = BSEL_IMM; u.imm = (opcd == 6'd11) ? simm : uimm;
      end
      6'd12, 6'd13: begin                             // addic, addic.
        u.op = OP_ADD; u.a_used = 1'b1; u.bsel = BSEL_IMM; u.imm = simm;
        u.set_ca = 1'b1; u.rd_we = 1'b1; u.rc = opcd[0];
      end
      6'd14, 6'd15: begin                             // addi, addis
        u.op = OP_ADD; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.bsel = BSEL_IMM;
        u.imm = opcd[0] ? shim : simm; u.rd_we = 1'b1;
      end
      6'd24, 6'd25, 6'd26, 6'd27, 6'd28, 6'd29: begin // ori oris xori xoris andi. andis.
        u.op = (opcd[2:1] == 2'b00) ? OP_OR : (opcd[2:1] == 2'b01) ? OP_XOR : OP_AND;
        u.x_from_s = 1'b1; u.s_used = 1'b1; u.bsel = BSEL_IMM; u.imm = opcd[0] ? {insn[15:0], 16'd0} : uimm;
        u.rd = f_a; u.rd_we = 1'b1; u.rc = (opcd[2:1] == 2'b10);
      end
      6'd20, 6'd21, 6'd23: begin                      // rlwimi rlwinm rlwnm
        // the rotate amount is always y: SH as an immediate, or rB for rlwnm
        u.op = OP_RLW; u.x_from_s = 1'b1; u.s_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb;
        u.rlw_insert = (opcd == 6'd20); u.a_used = (opcd == 6'd20);
        u.b_used = (opcd == 6'd23);
        if (opcd != 6'd23) begin u.bsel = BSEL_IMM; u.imm = {27'd0, f_b}; end
      end

      // ------------------------------------------------------------ branches
      6'd16: begin                                    // bc
        u.op = OP_BRANCH; u.bkind = BR_BC; u.imm = {{16{insn[15]}}, insn[15:2], 2'b00};
      end
      6'd18: begin                                    // b
        u.op = OP_BRANCH; u.bkind = BR_B; u.imm = {{6{insn[25]}}, insn[25:2], 2'b00};
      end
      6'd17: u.op = OP_SC;
      6'd19: begin
        case (xo10)
          10'd16:  begin u.op = OP_BRANCH; u.bkind = BR_BCLR;  end
          10'd528: begin u.op = OP_BRANCH; u.bkind = BR_BCCTR; end
          10'd50:  u.op = OP_RFI;
          10'd150: u.op = OP_ISYNC;                   // isync
          10'd0, 10'd33, 10'd129, 10'd193, 10'd225, 10'd257, 10'd289, 10'd417, 10'd449:
                   u.op = OP_CRLOG;                   // mcrf and the eight CR logic ops
          default: u.op = OP_ILLEGAL;
        endcase
      end

      // --------------------------------------------------------- loads/stores
      6'd32, 6'd34, 6'd40, 6'd42,                     // lwz lbz lhz lha
      6'd33, 6'd35, 6'd41, 6'd43: begin               // and their update forms
        u.msize   = (opcd[3]) ? 2'd1 : (opcd[1] ? 2'd0 : 2'd2);
        u.msigned = (opcd[3:1] == 3'b101);
        if (opcd[0] && step == 5'd1) begin            // update: rA = step 0's EA (the core supplies x)
          u.op = OP_ADD; u.a_zero = 1'b1; u.bsel = BSEL_IMM; u.imm = 32'd0; u.rd = f_a; u.rd_we = 1'b1;
        end else begin
          u.op = OP_LOAD; u.a_used = (f_a != 5'd0); u.a_zero = !opcd[0];
          u.bsel = BSEL_IMM; u.imm = simm; u.rd_we = 1'b1; u.last = !opcd[0];
        end
      end
      6'd36, 6'd38, 6'd44, 6'd37, 6'd39, 6'd45: begin // stw stb sth (u)
        u.op = OP_STORE; u.msize = (opcd[3]) ? 2'd1 : (opcd[1] ? 2'd0 : 2'd2);
        u.a_used = (f_a != 5'd0) || opcd[0]; u.a_zero = !opcd[0]; u.s_used = 1'b1;
        u.bsel = BSEL_IMM; u.imm = simm;
        u.st_update = opcd[0]; u.rd = f_a; u.rd_we = opcd[0];
      end
      6'd46: begin                                    // lmw
        u.op = OP_LOAD; u.msize = 2'd2; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1;
        u.bsel = BSEL_IMM; u.imm = simm + step4; u.rd = f_d + step; u.rd_we = 1'b1;
        u.last = (step == ~f_d);                      // rD + step = 31
      end
      6'd47: begin                                    // stmw
        u.op = OP_STORE; u.msize = 2'd2; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1;
        u.s_used = 1'b1; u.rs = f_d + step; u.bsel = BSEL_IMM; u.imm = simm + step4;
        u.last = (step == ~f_d);
      end

      // -------------------------------------------------------------- X form
      6'd31: begin
        if (is_xo) begin
          u.rd_we = 1'b1; u.rc = rcb; u.oe = oe; u.a_used = 1'b1; u.b_used = 1'b1;
          case (xo9)
            9'd266: u.op = OP_ADD;                                            // add
            9'd10:  begin u.op = OP_ADD;  u.set_ca = 1'b1; end                // addc
            9'd138: begin u.op = OP_ADD;  u.set_ca = 1'b1; u.cin = CIN_CA; end // adde
            9'd202: begin u.op = OP_ADD;  u.set_ca = 1'b1; u.cin = CIN_CA; u.bsel = BSEL_ZERO;   u.b_used = 1'b0; end // addze
            9'd234: begin u.op = OP_ADD;  u.set_ca = 1'b1; u.cin = CIN_CA; u.bsel = BSEL_MINUS1; u.b_used = 1'b0; u.imm = 32'hFFFF_FFFF; end // addme
            9'd40:  begin u.op = OP_SUBF; u.cin = CIN_ONE; end                // subf
            9'd8:   begin u.op = OP_SUBF; u.cin = CIN_ONE; u.set_ca = 1'b1; end // subfc
            9'd136: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; end // subfe
            9'd200: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; u.bsel = BSEL_ZERO;   u.b_used = 1'b0; end // subfze
            9'd232: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; u.bsel = BSEL_MINUS1; u.b_used = 1'b0; u.imm = 32'hFFFF_FFFF; end // subfme
            9'd104: begin u.op = OP_SUBF; u.cin = CIN_ONE; u.bsel = BSEL_ZERO; u.b_used = 1'b0; end // neg
            9'd235: u.op = OP_MUL;
            9'd75:  begin u.op = OP_MULH;  u.oe = 1'b0; end
            9'd11:  begin u.op = OP_MULHU; u.oe = 1'b0; end
            9'd491: u.op = OP_DIV;
            9'd459: u.op = OP_DIVU;
            default: u.op = OP_ILLEGAL;
          endcase
        end else begin
          case (xo10)
            10'd0, 10'd32: begin                                              // cmp cmpl
              u.op = xo10[5] ? OP_CMPL : OP_CMP; u.a_used = 1'b1; u.b_used = 1'b1;
            end
            10'd4: begin u.op = OP_TRAP; u.a_used = 1'b1; u.b_used = 1'b1; end // tw
            10'd28, 10'd60, 10'd124, 10'd284, 10'd316, 10'd412, 10'd444, 10'd476: begin
              case (xo10)
                10'd28:  u.op = OP_AND;  10'd60:  u.op = OP_ANDC;
                10'd124: u.op = OP_NOR;  10'd284: u.op = OP_EQV;
                10'd316: u.op = OP_XOR;  10'd412: u.op = OP_ORC;
                10'd444: u.op = OP_OR;   default: u.op = OP_NAND;
              endcase
              u.x_from_s = 1'b1; u.s_used = 1'b1; u.b_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb;
            end
            10'd24, 10'd536, 10'd792: begin                                    // slw srw sraw
              u.op = (xo10 == 10'd24) ? OP_SLW : (xo10 == 10'd536) ? OP_SRW : OP_SRAW;
              u.x_from_s = 1'b1; u.s_used = 1'b1; u.b_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb;
              u.set_ca = (xo10 == 10'd792);
            end
            10'd824: begin                                                    // srawi
              u.op = OP_SRAW; u.x_from_s = 1'b1; u.s_used = 1'b1; u.bsel = BSEL_IMM;
              u.imm = {27'd0, f_b}; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb; u.set_ca = 1'b1;
            end
            10'd26, 10'd922, 10'd954: begin                                   // cntlzw extsh extsb
              u.op = (xo10 == 10'd26) ? OP_CNTLZW : (xo10 == 10'd922) ? OP_EXTSH : OP_EXTSB;
              u.x_from_s = 1'b1; u.s_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb;
            end
            // indexed loads, plain and update
            10'd23, 10'd87, 10'd279, 10'd343, 10'd55, 10'd119, 10'd311, 10'd375: begin
              u.msize   = (xo10[8]) ? 2'd1 : (xo10[6] ? 2'd0 : 2'd2);
              u.msigned = (xo10 == 10'd343) || (xo10 == 10'd375);
              if (xo10[5] && step == 5'd1) begin          // update: rA = step 0's EA (the core supplies x)
                u.op = OP_ADD; u.a_zero = 1'b1; u.bsel = BSEL_IMM; u.imm = 32'd0; u.rd = f_a; u.rd_we = 1'b1;
              end else begin
                u.op = OP_LOAD; u.a_used = (f_a != 5'd0); u.a_zero = !xo10[5]; u.b_used = 1'b1;
                u.rd_we = 1'b1; u.last = !xo10[5];
              end
            end
            10'd20, 10'd534, 10'd790: begin                                   // lwarx lwbrx lhbrx
              u.op = OP_LOAD; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.b_used = 1'b1; u.rd_we = 1'b1;
              u.msize = (xo10 == 10'd790) ? 2'd1 : 2'd2;
              u.brev = (xo10 != 10'd20); u.reserve = (xo10 == 10'd20);
            end
            // indexed stores, plain and update
            10'd151, 10'd215, 10'd407, 10'd183, 10'd247, 10'd439: begin
              u.op = OP_STORE; u.msize = (xo10[8]) ? 2'd1 : (xo10[6] ? 2'd0 : 2'd2);
              u.a_used = (f_a != 5'd0) || xo10[5]; u.a_zero = !xo10[5]; u.b_used = 1'b1; u.s_used = 1'b1;
              u.st_update = xo10[5]; u.rd = f_a; u.rd_we = xo10[5];
            end
            10'd150, 10'd662, 10'd918: begin                                  // stwcx. stwbrx sthbrx
              u.op = OP_STORE; u.msize = (xo10 == 10'd918) ? 2'd1 : 2'd2;
              u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.b_used = 1'b1; u.s_used = 1'b1;
              u.brev = (xo10 != 10'd150); u.cond = (xo10 == 10'd150);
            end
            10'd339, 10'd371: begin u.op = OP_MFSPR; u.rd_we = 1'b1; end      // mfspr mftb
            10'd467: begin u.op = OP_MTSPR; u.s_used = 1'b1; end              // mtspr
            10'd19:  begin u.op = OP_MFCR;  u.rd_we = 1'b1; end
            10'd144: begin u.op = OP_MTCRF; u.s_used = 1'b1; end
            10'd83:  begin u.op = OP_MFMSR; u.rd_we = 1'b1; end
            10'd146: begin u.op = OP_MTMSR; u.s_used = 1'b1; end
            10'd512: u.op = OP_MCRXR;
            // segment registers and TLBs (603e 5.1.8, table 5-5)
            10'd210, 10'd242: begin                                           // mtsr mtsrin
              u.op = OP_MTSR; u.s_used = 1'b1;
              if (xo10 == 10'd242) u.b_used = 1'b1;
              else begin u.bsel = BSEL_IMM; u.imm = {insn[19:16], 28'd0}; end
            end
            10'd595, 10'd659: begin                                           // mfsr mfsrin
              u.op = OP_MFSR; u.rd_we = 1'b1;
              if (xo10 == 10'd659) u.b_used = 1'b1;
              else begin u.bsel = BSEL_IMM; u.imm = {insn[19:16], 28'd0}; end
            end
            // FP loads and stores in the X form
            10'd535, 10'd567, 10'd599, 10'd631,                              // lfsx lfsux lfdx lfdux
            10'd663, 10'd695, 10'd727, 10'd759:                              // stfsx stfsux stfdx stfdux
              fp_ls(xo10[7], xo10[6], xo10[5], 1'b1, 1'b0);
            10'd983: fp_ls(1'b1, 1'b0, 1'b0, 1'b1, 1'b1);                    // stfiwx
            10'd306:            begin u.op = OP_TLBIE; u.b_used = 1'b1; end   // tlbie
            10'd978, 10'd1010:  begin u.op = OP_TLBL;  u.b_used = 1'b1; end   // tlbld tlbli
            // Cache operations (603e 3.7), EA (rA|0) + rB, no GPR written.
            // Translated and protected as a load (dcbt, dcbtst, dcbst, dcbf) or
            // as a store (dcbz, dcbi; table 7-31); the cache gets d_op.
            10'd278, 10'd246, 10'd54, 10'd86: begin                           // dcbt dcbtst dcbst dcbf
              u.op = OP_LOAD; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.b_used = 1'b1; u.msize = 2'd2;
              u.cop = (xo10 == 10'd54) ? 3'd3 : (xo10 == 10'd86) ? 3'd4 : 3'd1;
            end
            10'd1014, 10'd470: begin                                          // dcbz dcbi
              if (xo10 == 10'd470 && pr) begin
                u.op = OP_ILLEGAL; u.priv = 1'b1;                             // supervisor-only (3.7.1)
              end else begin
                u.op = OP_STORE; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.b_used = 1'b1; u.msize = 2'd2;
                u.cop = (xo10 == 10'd1014) ? 3'd2 : 3'd5;
              end
            end
            10'd982: begin                                                    // icbi: not translated (3.7.8)
              u.op = OP_NOP; u.icbi = 1'b1; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1; u.b_used = 1'b1;
            end
            10'd598: begin u.op = OP_NOP; u.sync = 1'b1; end                 // sync
            10'd854, 10'd566:                                                // eieio tlbsync
              u.op = OP_NOP;
            default: u.op = OP_ILLEGAL;
          endcase
        end
      end

      // ------------------------------------------------- FP loads and stores
      6'd48, 6'd49, 6'd50, 6'd51,                     // lfs lfsu lfd lfdu
      6'd52, 6'd53, 6'd54, 6'd55: begin               // stfs stfsu stfd stfdu
        fp_ls(opcd[2], opcd[1], opcd[0], 1'b0, 1'b0);
      end

      // ----------------------------------------------- FP arithmetic, FPSCR
      6'd59, 6'd63: if (FPA) fp_arith(opcd[2]); else u.op = OP_ILLEGAL;

      default: u.op = OP_ILLEGAL;                     // reserved opcodes
    endcase


    // (no clear of rd_we by the op here: every branch that sets rd_we names a
    // legal op that writes rD, and the compare of the op after its priority
    // mux was the deepest path into EX's registers)
  end

endmodule
