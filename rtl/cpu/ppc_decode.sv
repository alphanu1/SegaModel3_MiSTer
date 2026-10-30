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
//       step 0: the load into rD        step 1: rA = rA + d (or rB)
//   lmw rD,d(rA)    step n: load r(rD+n) from rA + d + 4n, n = 0 .. 31-rD
//   stmw rS,d(rA)   step n: store r(rS+n) to rA + d + 4n
//
// Update stores write their EA to rA within the store uop. Floating point,
// translation and cache-block instructions arrive in later steps of M0-A2;
// until then FP opcodes decode as illegal and the TLB, segment and cache ops as
// no-ops.

module ppc_decode
  import ppc_pkg::*;
(
  input  logic [31:0] insn,
  input  logic [31:0] pc,
  input  logic [4:0]  step,
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
    u.sh       = f_b;
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
        u.op = OP_RLW; u.x_from_s = 1'b1; u.s_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1; u.rc = rcb;
        u.rlw_insert = (opcd == 6'd20); u.a_used = (opcd == 6'd20);
        u.sh_from_rb = (opcd == 6'd23); u.b_used = (opcd == 6'd23);
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
          10'd150: u.op = OP_NOP;                     // isync
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
        if (opcd[0] && step == 5'd1) begin            // update: rA = rA + d
          u.op = OP_ADD; u.a_used = 1'b1; u.bsel = BSEL_IMM; u.imm = simm; u.rd = f_a; u.rd_we = 1'b1;
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
        u.last = ((f_d + step) == 5'd31);
      end
      6'd47: begin                                    // stmw
        u.op = OP_STORE; u.msize = 2'd2; u.a_used = (f_a != 5'd0); u.a_zero = 1'b1;
        u.s_used = 1'b1; u.rs = f_d + step; u.bsel = BSEL_IMM; u.imm = simm + step4;
        u.last = ((f_d + step) == 5'd31);
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
            9'd234: begin u.op = OP_ADD;  u.set_ca = 1'b1; u.cin = CIN_CA; u.bsel = BSEL_MINUS1; u.b_used = 1'b0; end // addme
            9'd40:  begin u.op = OP_SUBF; u.cin = CIN_ONE; end                // subf
            9'd8:   begin u.op = OP_SUBF; u.cin = CIN_ONE; u.set_ca = 1'b1; end // subfc
            9'd136: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; end // subfe
            9'd200: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; u.bsel = BSEL_ZERO;   u.b_used = 1'b0; end // subfze
            9'd232: begin u.op = OP_SUBF; u.cin = CIN_CA;  u.set_ca = 1'b1; u.bsel = BSEL_MINUS1; u.b_used = 1'b0; end // subfme
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
              if (xo10[5] && step == 5'd1) begin
                u.op = OP_ADD; u.a_used = 1'b1; u.b_used = 1'b1; u.rd = f_a; u.rd_we = 1'b1;
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
            // cache, TLB and segment operations: no-ops until the MMU and caches land
            10'd54, 10'd86, 10'd246, 10'd278, 10'd470, 10'd982,            // dcbst dcbf dcbtst dcbt dcbi icbi
            10'd598, 10'd854, 10'd566, 10'd306,                              // sync eieio tlbsync tlbie
            10'd210, 10'd242, 10'd978, 10'd1010:                             // mtsr mtsrin tlbld tlbli
              u.op = OP_NOP;
            default: u.op = OP_ILLEGAL;
          endcase
        end
      end

      default: u.op = OP_ILLEGAL;                     // FP and reserved opcodes
    endcase

    if (u.op == OP_ILLEGAL || u.op == OP_SC || u.op == OP_RFI || u.op == OP_TRAP)
      u.rd_we = 1'b0;
  end

endmodule
