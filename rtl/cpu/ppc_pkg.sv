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
    OP_MTSR, OP_MFSR,   // mtsr / mtsrin, mfsr / mfsrin: the SR number is y[31:28]
    OP_TLBIE, OP_TLBL,  // tlbie; tlbld / tlbli (insn[6] = 1: tlbli). The EA is y
    OP_ISYNC,           // refetches: context synchronizing
    OP_FP,              // FP arithmetic, moves, compares and FPSCR ops (funit, fop, fsop)
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

    // rotate (the amount is y: SH as an immediate, or rB)
    logic [4:0]  mb, me;
    logic        rlw_insert;      // rlwimi

    // memory
    logic [1:0]  msize;           // 0 byte, 1 half, 2 word
    logic        msigned;         // lha / lhau / lhax / lhaux
    logic        brev;            // lwbrx, lhbrx, stwbrx, sthbrx
    logic        reserve;         // lwarx
    logic        cond;            // stwcx.
    logic        st_update;       // store with update: write EA to rA
    logic [2:0]  cop;             // a data cache operation, as ppc_dcache's d_op: 1 dcbt / dcbtst,
                                  // 2 dcbz, 3 dcbst, 4 dcbf, 5 dcbi (a load or store uop otherwise)
    logic        icbi;            // icbi: EA (rA|0) + rB, its set index only (603e 3.7.8)
    logic        sync;            // sync: waits for every access to be done
    logic        priv;            // a privileged instruction in user mode (program, SRR1[13])

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

    // floating point (FP loads and stores are OP_LOAD / OP_STORE uops)
    logic        fp;              // an FP instruction: MSR[FP] = 0 takes 0x800
    logic [1:0]  funit;           // 0 none, 1 multiply-add, 2 divider, 3 small unit
    logic [3:0]  fop;             // fma_op_t, div_op_t or misc_op_t (ppc_fpu_pkg)
    logic        fsgl;            // opcode 59: single precision
    logic [2:0]  fsop;            // FPSCR op (fs_op_t): mtfsf, mtfsfi, mtfsb0/1, mcrfs
    logic        fpw;             // this uop writes FPR frD (insn[25:21])
    logic        fua, fub, fuc;   // reads frA, frB, frC (frS for a store: the C port)
    logic        fld, fst;        // FP load / store uop
    logic        fdbl;            // lfd / stfd: two word beats, high word first
    logic        fhi;             // lfd's first beat: the high word, no FPR write
    logic        fsl;             // lfs / stfs: single-format conversion
    logic        fiw;             // stfiwx
    logic        fcmp;            // fcmpu / fcmpo
    logic        frc;             // Rc = 1: CR1 from FPSCR[0-3]
    logic        fsu;             // may change the FPSCR (arithmetic, compares, FPSCR ops)
    logic        fser;            // waits in EX until no older FP op can change the FPSCR
  } uop_t;

  // the fetch queue's predecode of a word (ppc_pdw.sv, ppc_core.sv)
  typedef struct packed {
    logic [3:0] ck;          // CR writer kind: stwcx., mtcrf, a field (mcrf, mcrxr), a bit (CR logic)
    logic       w_spr, w_ctr, w_lr, w_xer, w_cr;   // special registers written
    logic [4:0] rk;          // result kind: group, or logical op (below)
    logic [2:0] immk;        // y's immediate: 0 SIMM, 1 UIMM, 2 shifted, 3 SH, 4 SIMM + 4 step, 5 zero, 6 -1
    logic       mfmsr, mfcr;
    logic [2:0] mf_sel;      // mfspr's source
    logic [5:0] spr_idx;     // 63: not an SPR the core has
    logic       yreg;        // y from rB
    logic       az;          // (rA|0)
    logic       xs;          // first operand from rS
    logic [1:0] crack;       // 0 single uop, 1 update form, 2 lmw / stmw
    logic       ua, ub, us;  // reads rA, rB, rS
    logic       ifault;      // a fetch fault marker, not an instruction
    logic       l0;          // the first uop is the last (not cracked, or lmw / stmw of r31)
    logic       fua, fub, fuc, fpw;   // FPRs read (A, B, C port) and written
    logic [4:0] fci;         // the C port's FPR: frC, or frS for a store
    logic       fdx;         // lfd / stfd: step 1 takes step 0's EA + 4
    logic       fwcr, fck1;  // an FP CR writer: any (a compare, mcrfs, Rc = 1); a field (mcrfs, CR1)
    logic       hh;          // mfspr HASH1 / HASH2 (from the raw word: ID's hash wait is one AND)
    logic       ra0;         // the rA field is 0 (from the raw word: (rA|0) without a compare in ID)
    logic       ill;         // illegal, opcodes 59 and 63 aside (dcbi's privilege is ID's)
    alu_op_t    pop;         // the first uop's op, opcodes 59 and 63 aside (ID's id_opf)
    // EX's FP controls, so they are not behind the decoder: the unit (1 fma,
    // 2 div, 3 misc), a misc op that writes an FPR, an FPSCR op, a compare or
    // stfs (a second EX cycle), and the fast port's route kind (frsqrte; a
    // single op the single unit runs when its operands allow)
    logic       ffma, fdiv, fmsc, fmw, ffsx, ff2c, frq, frs;
    // fetch-side, set by the core (ppc_pdw leaves them 0), for the BTB's keys
    // (D-30): the word's slot in its two-word request, whether the request
    // before it was two words, and whether its request was sequential
    logic       bslot, bpp, bsq;
    // set by the core: the word's direction was resolved at fetch from the
    // committed CR and CTR (the 603e's look-ahead, D-30) and fetch followed it
    // (a bclr / bcctr resolved taken without a BTB target cannot be), for
    // EX's check
    logic       lar;
    // the first uop's GPR result is rA, not rD (logical, shift and rotate
    // ops, update stores): ID's own rd for the early compares without the
    // decoder in front (ppc_core.sv, id_rdf)
    logic       rda;
    // an integer update load (cracked, not an FP double, no FPR written, not a fault marker): its
    // step 1 writes rA from step 0's EA (ppc_core.sv, id_xuea), so ID's operand selects need no
    // decode of the queue head's fields (R108)
    logic       uea;
    // last, at bits 2:0, so the fetch queue can copy it beside the register
    // fields for the GPR file's read addresses (ppc_fetchq r_rf)
    logic [2:0] sh;          // rA, rB, rS is a TGPR shadow: MSR[TGPR] and the field < 4
  } pd_t;
  localparam int PDW = 76;                             // (alu_op_t is 6 bits)

endpackage
