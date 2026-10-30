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
// PowerPC 603e core: shared types.
//
// The decoder turns one architectural instruction into one or more uops. Every
// uop writes at most one GPR, so the register file needs a single write port
// (study 8.4: a write port costs a whole copy of the MLAB file). Update loads
// and lmw are cracked; update stores fold their rA write into the store uop.
//
// Behaviour follows the MPC603e User's Manual and the 32-bit Programming
// Environments Manual. Design target: study D-15.

package ppc_pkg;

  // ALU operation. One shared adder, one rotator, one logic unit (study 8.4:
  // arithmetic built once and shared across groups that never overlap).
  typedef enum logic [5:0] {
    OP_ADD,       // a + b (+ carry-in per cin_sel)
    OP_SUBF,      // ~a + b + 1 (+ carry-in per cin_sel), i.e. b - a
    OP_AND, OP_OR, OP_XOR, OP_NAND, OP_NOR, OP_EQV, OP_ANDC, OP_ORC,
    OP_SLW, OP_SRW, OP_SRAW,
    OP_RLW,       // rotate s left by sh, insert under mask into (rlwimi ? a : 0)
    OP_CNTLZW, OP_EXTSB, OP_EXTSH,
    OP_CMP, OP_CMPL,
    OP_MUL, OP_MULH, OP_MULHU,
    OP_DIV, OP_DIVU,
    OP_PASSB,     // result = b (mtspr data, li handled as add)
    OP_MFSPR, OP_MTSPR, OP_MFCR, OP_MTCRF, OP_MFMSR, OP_MTMSR,
    OP_CRLOG,     // crand ... creqv, mcrf
    OP_MCRXR,
    OP_BRANCH,
    OP_LOAD, OP_STORE,
    OP_SC, OP_RFI, OP_TRAP,
    OP_NOP,
    OP_ILLEGAL
  } alu_op_t;

  typedef enum logic [1:0] { CIN_ZERO, CIN_ONE, CIN_CA } cin_sel_t;
  typedef enum logic [1:0] { BSEL_RB, BSEL_IMM, BSEL_ZERO, BSEL_MINUS1 } bsel_t;
  typedef enum logic [1:0] { BR_B, BR_BC, BR_BCLR, BR_BCCTR } br_kind_t;

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] insn;
    alu_op_t     op;

    // operands
    logic [4:0]  ra, rb, rs;      // read port indexes
    logic        a_zero;          // (rA|0): use 0 when rA = 0
    logic        x_from_s;        // first ALU operand is rS (logical, shift, rotate)
    logic        a_used, b_used, s_used;
    bsel_t       bsel;
    logic [31:0] imm;
    cin_sel_t    cin;

    // results
    logic        rd_we;
    logic [4:0]  rd;
    logic        set_ca;          // write XER[CA]
    logic        oe;              // write XER[OV], accumulate SO
    logic        rc;              // write CR0 from the result
    logic [2:0]  crf;             // target CR field of cmp / mcrf / mcrxr
    logic [2:0]  crf_src;         // source field of mcrf

    // rotate
    logic [4:0]  sh;              // from instruction or rB[4:0] (sh_from_rb)
    logic        sh_from_rb;
    logic [4:0]  mb, me;
    logic        rlw_insert;      // rlwimi

    // memory
    logic [1:0]  msize;           // 0 byte, 1 half, 2 word
    logic        msigned;         // lha / lhau / lhax / lhaux
    logic        brev;            // lwbrx, lhbrx, stwbrx, sthbrx
    logic        reserve;         // lwarx
    logic        cond;            // stwcx.
    logic        st_update;       // store with update: write EA to rA

    // branch
    br_kind_t    bkind;
    logic [4:0]  bo, bi;
    logic        lk, aa;

    // condition register logic
    logic [3:0]  crop;            // insn[9:6] of the 19-form CR ops
    logic [4:0]  crbd, crba, crbb;
    logic [7:0]  crm;             // mtcrf field mask

    logic [9:0]  spr;
    logic        last;            // last uop of the architectural instruction
  } uop_t;

endpackage
