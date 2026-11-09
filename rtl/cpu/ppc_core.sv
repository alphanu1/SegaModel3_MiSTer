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
// PowerPC 603e core: integer pipeline, MMU and FPU. Design target: study D-15.
//
//   F   fetch: one request outstanding, two-entry queue
//   ID  decode, crack, register read addresses, load-use hazard
//   EX  ALU / branch / multiply-divide / address; all special registers are
//       computed here from forwarded values; the data request goes out
//   MEM special registers commit as the instruction leaves MEM
//   WB  a load's or store's answer; GPR and FPR writes as WB is left
//
// Forwarding goes into the operand registers as a uop enters EX: EX's own
// result for a producer one step ahead, else the MEM producer's result (not a
// load's: its data comes in WB), else the WB producer's (a load's data as it
// arrives), else the register file read in ID. EX starts from registers
// alone. A load's consumer enters EX three cycles after the load did: two
// bubbles when it is next in line, one when one instruction separates them.
//
// Branches are predicted at fetch (static, the 603e's rule) and resolved in EX,
// which redirects only on a mispredict. sc, rfi, mtmsr and illegal
// instructions redirect from EX and commit their special-register writes at
// MEM exit; a fired trap commits and redirects from WB. Every redirect is
// registered before it reaches fetch.
//
// Address translation (603e manual chapter 5) costs no cycle when the address
// hits a BAT or a micro-TLB of recent page translations: fetch translates its
// address as it goes out, EX translates the load or store address beside the
// adder. A page translation that misses the micro-TLB is made from the TLB
// from a registered address (fetch: two cycles; data: in MEM, two cycles,
// then the instruction is refetched). The page offset, EA[20-31], never passes
// through translation, so a cache indexed by it (4 KiB per way) can start its
// read from the untranslated bits and compare a physical tag that is already
// registered. See "MMU", "Instruction translation" and "data translation".
//
// Exceptions and interrupts are precise. Fetch faults, interrupts and sc /
// illegal are taken in EX; data translation faults and the refetch after an
// MMU write redirect from MEM. Asynchronous interrupts (external,
// decrementer) replace the first uop of an instruction in EX: the boundary is
// before that instruction.
//
// Floating point (ppc_fpu*.sv; see "Floating point" below): FPRs in MLAB read
// in ID, the multiply-add pipeline (with its fast single-precision port) and
// the divider issued from EX, the small
// unit's results and FP loads written in WB, a scoreboard of pending FPR
// writes, and the FPSCR updated in program order.
//
// Cache operations (603e 3.7): dcbt, dcbtst, dcbst, dcbf, dcbz and dcbi are
// data requests with d_op, translated as loads or stores (table 7-31); a
// touch that would fault is a no-op (3.7.2); dcbz to a W or I page takes the
// alignment exception (3.7.4); dcbi in user mode is a privileged program
// exception (3.7.1); icbi goes to the I-cache from MEM untranslated (3.7.8);
// sync waits in EX until nothing is outstanding and d_idle. HID0's cache
// bits are flip-flops here and go to the caches (rtl/cpu/ppc_cpu.sv).
//
// Misaligned accesses (603e 2.2.3, 4.5.6; see "memory access" below): an
// integer halfword or word that crosses a word boundary is split in EX into
// two word accesses, the first made while EX holds a cycle; one inside its
// word is a single access on its lanes. The alignment exception (0x600) is
// taken for lmw, stmw, lwarx, stwcx. and FP accesses not word-aligned, and for
// an FP double crossing a page with MSR[DR] = 1 and no DBAT hit (R129). An
// integer access crossing a page is split like any other (603e 4.5.3: "The
// 603e supports the crossing of page boundaries"); with MSR[DR] = 1 its second
// word is translated on its own page (study R124, xpage, amending D-39).
//
// Not yet here: privilege checks other than dcbi's (MSR[PR] selects keys and
// BAT valid bits), MSR[LE].
//
// Memory ports. Addresses are physical.
//   Fetch: a request is accepted on the edge where i_req is high; i_valid
//   comes one or more cycles later. One request outstanding.
//   Data, pipelined: d_req may be high in any cycle where d_rdy is high, and
//   the request is taken on that edge; with d_rdy low the core holds the
//   access in EX. d_rdy must come from a register on the memory side (it
//   feeds EX's hold). Every request is answered by one d_valid pulse, in
//   request order, at most one a cycle, no earlier than the cycle after the
//   request; a load's word is on d_rdata with it, a store's pulse only
//   acknowledges. The memory side must take every request made while d_rdy
//   is high, so with one a cycle it has the request of this cycle and up to
//   two older ones unanswered: those in MEM and WB. A uop waits in WB for its
//   answer (the whole pipeline behind it holds); one that arrives while the
//   uop is still in MEM is kept for it. d_req, d_we, d_addr, d_be and
//   d_wdata come from EX's translation and are late in the cycle (R32).
//   d_resv marks an lwarx's request (d_cond an stwcx.'s). res_kill: the
//   reservation has been lost to another master's write of its line (603e
//   2.3.6.1, through ppc_dcache's snoop); a stwcx. in EX then fails. Tie it
//   low without a snooping cache.

module ppc_core
  import ppc_pkg::*, ppc_fpu_pkg::*;
#(
  parameter logic [31:0] RESET_PC = 32'hFFF0_0100,
  parameter logic [31:0] PVR      = 32'h0006_0101,
  // the multiply-add unit's optional stage boundaries (ppc_fpu_fma.sv); all
  // in, L = 12, is what 120 MHz needs
  parameter bit P1A = 1'b1, P2 = 1'b1, P3 = 1'b1, P4A = 1'b1, P5A = 1'b1,
  parameter bit P5  = 1'b1, P6A = 1'b1, P6 = 1'b1, P7 = 1'b1,
  // D-30's fetch additions, each of which can be turned off: BTB = 1 the
  // branch target buffer looked up a request ahead (R61 design C, with the
  // I-cache's pre-read port), BTB_N entries
  parameter bit BTB = 1'b1,
  parameter int BTB_N = 64,
  // FETCH2 = 1: two-word fetch, the I-cache's fast path answering a
  // sequential request with the word and the next (ppc_icache FETCH2); off
  // by default (D-36: about 520 ALM cached for 1-3 frames lost in 56,700)
  parameter bit FETCH2 = 1'b0,
  // LA = 1: the 603e's CR look-ahead (603e 2.3.4.4): a conditional branch
  // whose CR bit and CTR no instruction in flight may change is resolved as
  // its word arrives; otherwise the static rule (the y bit) predicts it
  parameter bit LA = 1'b1,
  // SOFTLU = 1: the BTB also looks up a soft redirect's target (see bt_rd)
  parameter bit SOFTLU = 1'b1,
  // FBYP = 1: a multiply-add or divide result goes into EX's FP operands in the cycle it is written,
  // so its consumer enters EX a cycle before the write and issues in it (see "FP operand bypass")
  parameter bit FBYP = 1'b1,
  // FLW = 1: an FP load leaves WB as its word arrives and writes its FPR from a slot of its own in the
  // next free cycle of the write port (see "FP load write slot"); 0: WB holds it a cycle for that write
  parameter bit FLW = 1'b1,
  // UTS = 1: a TLB or segment register write empties only the micro-TLB entries it can change (see
  // "micro-TLB invalidation"); 0: every MMU write empties both micro-TLBs
  parameter bit UTS = 1'b1,
  // FHD = 1: a head straight from fetch waits on the FP scoreboard only when a field of its word names a
  // pending FPR (hd_fn); 0: it always waits a cycle if it names an FPR
  parameter bit FHD = 1'b1,
  // LU2 = 1: load-use 2 (the 603e's cache-hit load, table 6-6): a load's consumer leaves ID the cycle after the
  // load leaves EX, taking the word the D-cache's RAMs give in the lookup cycle (d_eword, the predicted way) as
  // its operand; EX waits for the answer and d_eok before it runs (see "load-use 2"). 0: it waits in ID for WB
  parameter bit LU2 = 1'b1,
  // LU2's alignment: LU2_AL = 1 any load's bytes are put in place (a 4:1 per byte after the way's word);
  // 0 only a load whose bytes end the word (lwz, lhz at 2, lbz at 3) takes the early word, unshifted
  parameter bit LU2_AL = 1'b1,
  // LU2_FL = 1: the early word joins the operand registers' last LUT (beside EX's own late result);
  // 0: it joins the capture (x_pre), a LUT before
  parameter bit LU2_FL = 1'b1,
  // EWL = 1 (R140): the early word comes as all four ways' words (d_ew4) and the predicted way (d_epw), and
  // each way's word is aligned before the way is chosen, so the way (from the D-cache's state MLAB, the
  // later of the two) is one level from the operand registers' last LUT; 0: d_eword, the way's word
  parameter bit EWL = 1'b0,
  // SQ1 ... SQ5: the single unit's stage boundaries (ppc_fpu_sfma R1 ... R5); each one taken out takes a
  // cycle off the fast port's latency LS (6 with all five)
  parameter bit SQ1 = 1'b1, SQ2 = 1'b1, SQ3 = 1'b1, SQ4 = 1'b1, SQ5 = 1'b1,
  // FEF = 1 (with FBYP and SQ5): the single unit's early forward. A single op of the fast kind whose operand
  // is the result of a single-unit op (not frsqrte) issued with OE = UE = 0 issues LS - 1 cycles after it,
  // a cycle before the result is written, taking it from the single unit's R5 (see "FP operand bypass")
  parameter bit FEF = 1'b1,
  // FWW = 1: a uop whose frD has an FPU-port write outstanding (WAW) waits only until that write lands in its
  // first EX cycle or the next (it then leaves EX a cycle before the write, and that write does not clear frD's
  // pending bit: wsk_q); 0: until the write lands as it enters ID (see "Scoreboard")
  parameter bit FWW = 1'b1,
  // LRA = 1: the LR / CTR look-ahead (603e 6.3.1, 6.4.1.1: the BPU executes a bclr / bcctr as it is fetched,
  // with the LR or CTR from its rename register or the architected one). A bclr or bcctr the rule predicts taken
  // whose target fetch knows is a soft redirect to it, as a b is (see "LR / CTR look-ahead"); 0: it is predicted
  // not taken unless the BTB holds it
  parameter bit LRA = 1'b1,
  // IUN: the instruction micro-TLB's entries (2, 4 or 8), replaced in turn; the 603e's ITLB has 64 (5.4.3.1),
  // and a hit in either costs no cycle, so more entries change only the timing
  parameter int IUN = 4,
  // DUN: the data micro-TLB's entries (2, 4, 6 or 8); the 603e's DTLB has 64 (5.4.3.1) and a hit costs no
  // cycle, so this changes only the timing (a miss is a replay). DRP: its replacement, 0 in turn (a pointer),
  // 1 least recently used (an empty entry first); both are timing only
  parameter int DUN = 4,
  parameter bit DRP = 1'b1,
  // FAST = 0: no single unit (ppc_fpu_fast SF = 0); every multiply-add op goes to the double unit, frsqrte
  // still to its estimate pipe. Same results, single ops at the double unit's latency (study R129)
  parameter bit FAST = 1'b1
)(
  input  logic        clk,
  input  logic        rst,

  output logic        i_req,
  output logic [31:0] i_addr,
  output logic [3:0]  i_wimg,       // the fetch's WIMG (real mode 0011, 603e 3.5)
  output logic        i_seq,        // i_addr is the previous request's + 4, same translation
  input  logic        i_valid,
  input  logic [31:0] i_data,
  // the BTB's side of the I-cache (ppc_icache "Pre-read port"): with an
  // answer, the way, whether it came from the arrays, and the physical page;
  // with B's request the pre-read of T, with T's request i_pre
  input  logic [1:0]  i_way,
  input  logic        i_arr,
  input  logic [19:0] i_pa,
  output logic        i_pv,
  output logic [12:2] i_pidx,       // (bit 12: the I-cache's ISB index bit, EA[19])
  output logic [1:0]  i_pway,
  output logic [19:0] i_ppa,
  output logic        i_pre,
  output logic        i_eix,        // EA[19] of the request (the I-cache's ISB index bit)
  // two-word fetch: the cache offers two words to a sequential request
  // (i_two), the core asks (i_w2), the second comes in i_data2
  input  logic        i_two,
  output logic        i_w2,
  input  logic [31:0] i_data2,

  output logic        d_req,
  output logic        d_we,
  output logic [31:0] d_addr,
  output logic [3:0]  d_be,
  output logic [31:0] d_wdata,
  output logic [3:0]  d_wimg,       // the access's WIMG
  output logic [2:0]  d_op,         // 0 load / store, else a cache operation (ppc_dcache)
  input  logic        d_rdy,        // the memory side takes a request this cycle (from a register)
  input  logic        d_idle,       // nothing buffered or in flight on the memory side (sync)
  input  logic        d_valid,
  input  logic [31:0] d_rdata,
  input  logic [31:0] d_eword,      // LU2: the predicted way's word in a load's lookup cycle (combinational)
  input  logic        d_eok,        // LU2: d_rdata is the d_eword of the cycle before (a register)
  input  logic [3:0][31:0] d_ew4,   // EWL: each way's word at the lookup (d_eword is d_ew4[d_epw])
  input  logic [1:0]  d_epw,        // EWL: the predicted way
  output logic        d_resv,       // with d_req: lwarx (the D-cache snoops its line, D-41)
  output logic        d_cond,       // with d_req: stwcx. (verification)
  input  logic        res_kill,     // the reservation is lost to a snooped write (ppc_dcache)

  // HID0's cache bits (603e table 2-2); ICFI and DCFI are one-cycle pulses as
  // they are written (the bits read 0: they clear as the invalidation begins)
  output logic        hid_ice, hid_dce, hid_ilock, hid_dlock, hid_icfi, hid_dcfi, hid_noopti,
  output logic        icbi_req,     // icbi from MEM, held until icbi_ack
  output logic [6:0]  icbi_set,     // EA[20-26]
  input  logic        icbi_ack,

  input  logic        tb_tick,      // timebase / decrementer enable (bus clock / 4)
  input  logic        ext_int,      // external interrupt, level, active high (603e INT, 4.5.5)

  // performance events, one bit each per cycle, for rtl/dbg/m3_cpu_mon.sv:
  //   0 instruction retired   1 uop retired     2 decode starved (fetch)
  //   3 load-use bubble       4 mul/div wait    5 data memory wait
  //   6 redirect (branch, exception)            7 fetch waiting on memory
  output logic [7:0]  ev,
  //   ev_mmu: 0 DTLB miss taken   1 ITLB miss taken   2 DSI or ISI taken
  //           3 external or decrementer interrupt taken
  //           4 data micro-TLB miss (a replay)   5 fetch translation slow cycle
  output logic [5:0]  ev_mmu,
  //   ev_btb: 0 a BTB prediction's target requested   1 its word failed the check
  //           2 a register target refetched from MEM   3 an entry written
  output logic [3:0]  ev_btb,
  //   ev_xrf: a split's second word replayed after its first was made, and the replay refetched (no fault):
  //           the first word's access is made again (the core bench drops the earlier one). Unconnected
  //           outside the core bench
  output logic        ev_xrf,

  // retire trace: one record per uop leaving WB, with the special registers it
  // committed. Unconnected in the core build, so it costs nothing there.
  output logic        rt_valid,
  output logic        rt_last,
  output logic [31:0] rt_pc,
  output logic [31:0] rt_insn,
  output logic        rt_gpr_we,
  output logic [4:0]  rt_gpr,
  output logic        rt_gpr_sh,    // the write went to TGPR0-3 (MSR[TGPR] = 1)
  output logic [1:0]  rt_irq,       // this record is an interrupt taken: 1 external, 2 decrementer
  output logic [31:0] rt_gpr_val,
  output logic [31:0] rt_cr,
  output logic [31:0] rt_xer,
  output logic [31:0] rt_lr,
  output logic [31:0] rt_ctr,
  output logic [31:0] rt_msr,
  output logic [31:0] rt_srr0,
  output logic [31:0] rt_srr1,
  // floating point: the FPSCR, FP-quiet (no FPR write or FPSCR update
  // outstanding) and the FPR write port, which writes on the coming edge
  output logic [31:0] rt_fpscr,
  output logic        rt_fpq,
  output logic        rt_fw_en,
  output logic [4:0]  rt_fw_idx,
  output logic [63:0] rt_fw_val,
  // cycles this record's uop was held in MEM (saturating): an interrupt, or a
  // DEC / timebase read, was decided in EX that many cycles more than two
  // before the record
  output logic [10:0] rt_dly
);
  // (R140) the restructurings' own checks; R140_NOCHK turns them off for the planted-fault runs
`ifdef R140_NOCHK
  localparam bit R1C = 1'b0;
`else
  localparam bit R1C = 1'b1;
`endif

  localparam logic [31:0] MSR_MASK   = 32'h0007_FF73;  // implemented MSR bits
  localparam logic [31:0] SRR1_KEEP  = 32'h0000_FF73;  // MSR bits saved in SRR1
  localparam logic [31:0] XER_MASK   = 32'hE000_007F;  // SO OV CA, byte count
  localparam int          MSR_IP     = 6;
  localparam int          MSR_ILE    = 16;
  localparam int          MSR_TGPR   = 17;
  localparam int          MSR_EE     = 15;
  localparam int          MSR_PR     = 14;
  localparam int          MSR_IR     = 5;
  localparam int          MSR_DR     = 4;
  localparam logic [31:0] MSR_XCLR   = 32'h0006_EF33;  // bits an exception clears (table 4-8); LE <- ILE

  // ===================================================================== state
  logic [31:0] cr_q, xer_q, lr_q, ctr_q, msr_q;
  logic        ctr1_q;                                 // ctr_q == 1
  logic        res_q;                                  // lwarx reservation
  logic [63:0] tb_q;
  logic [31:0] dec_q;

  // ============================================================== stage regs
  // EX
  logic        ex_valid;
  uop_t        ex_u;
  logic        ex_md_busy;                             // multiply/divide in flight
  // EX's FP controls, registered in ID (none for a fault marker)
  logic        ex_fma, ex_div, ex_misc, ex_fpw, ex_fcmp, ex_fsu, ex_fpser;
  logic        ex_fst, ex_fpun, ex_c2, ex_frc;
  logic        ex_fsw, ex_fsx, ex_f2c, ex_mw;          // waits: FPSCR settled, FPSCR op, second cycle, WB slot
  logic [2:0]  ex_fsfld;                               // the FPSCR op's field: crfS (mcrfs), else crfD
  logic [2:0]  ex_fsop;
  logic        fp_wait;                                // EX waits on the FPU (below)
  logic        m_fsok;                                 // MEM: the FP state it waited for is settled
  logic [31:0] fsv;                                    // FP store data
  logic [31:0] fpscr_q;

  // MEM
  typedef struct packed {
    logic        valid;
    logic [31:0] pc, insn;
    logic        last;
    logic        rd_we;
    logic [4:0]  rd;
    logic [31:0] result;
    logic        is_load, req;
    logic [1:0]  msize, boff;
    logic        msigned, brev;
    logic        cr_we, xer_we, lr_we, ctr_we, msr_we, spr_we, tb_we, dec_we;
    logic [31:0] cr, xer, lr, ctr, msr, sprv;
    logic        ctr1;                                 // ctr == 1, for bdnz / bdz in EX
    logic [5:0]  spr;
    logic        tbu;
    logic        exc;                                  // sc / program: SRR0, SRR1
    logic [31:0] srr0, srr1;
    logic        res_set, res_clr;
    logic        rc0;                                  // CR0 from the result, formed in MEM
    logic        mcrf0;                                // mcrf of CR0: its value is the committed CR0
    logic [31:0] crm;                                  // CR bits this uop writes (cr holds their values)
    logic        cmp_we;                               // cmp / cmpl: the field goes in at MEM
    logic [3:0]  cmpv;
    logic [2:0]  crf;
    logic        is_trap;                              // tw / twi: the compare runs in MEM
    logic [4:0]  to;
    logic [31:0] ta, tb;
    // MMU and interrupts
    logic        xc;                                   // exception in place of the uop, from EX's
                                                       // registered state (fetch fault, interrupt)
    logic [12:0] xvec;                                 // its vector offset
    logic        tmiss, imiss;                         // TLB miss (SRR1 takes CR0); ITLB (and its WAY)
    logic        dsi;                                  // DSISR and DAR written
    logic [2:0]  dsb;                                  // DSISR bits 4, 5, 6
    logic [23:0] vsid;                                 // DCMP's VSID on a DTLB miss
    logic [1:0]  irq;                                  // interrupt taken: {decrementer, external}
    logic        srw, tlbie, tlbl;                     // mtsr(in), tlbie, tlbld/tlbli
    logic [19:0] ya;                                   // y[31:12]: SR number, TLB set and EA[4-14]
    logic        hsrc;                                 // writes an input of HASH1/HASH2
    logic        resync;                               // an MMU write: refetch after it, from MEM
    logic        rp;                                   // a page access to translate in MEM (replay)
    logic        st;                                   // a store
    logic [4:0]  step;                                 // the uop's step in its instruction
    // floating point; frD is rd
    logic        fpw;                                  // writes an FPR in WB (load, small unit)
    logic        fsl;                                  // lfs: the word converted in WB
    logic        fhi;                                  // lfd's first word: held for the second
    logic        fscmp;                                // a compare: its FPSCR flags go in here
    fpu_flags_t  fflags;                               // ... registered from the small unit
    logic        fpser;                                // held until the FP state is settled:
                                                       // precise mode (MSR[FE0/FE1]) or CR1
    logic        fcr1;                                 // CR1 from the FPSCR once settled
    logic        fsx;                                  // an FPSCR instruction (it wrote leaving EX)
    logic        fany;                                 // changed FP state before WB (FPU issue, compare,
                                                       // FPSCR instruction): for the trace's rt_fpq
    logic        dz;                                   // dcbz: the alignment check is made in MEM (m_algc)
    logic        dbany, dbwi, duwi;                    // a DBAT hit; its W or I; the micro-TLB hit's W or I
    logic        fsm;                                  // a small-unit FP result for WB (fpw, not a load)
    logic        fld;                                  // an FP load (fpw and a load)
    logic        crlb;                                 // CR logic: the bit written
    logic [3:0]  crf4;                                 // a field write (mcrf, mcrxr, stwcx., FP): the field
    logic        mtca, mtcb;                           // mtcrf: the bits ID's uop reads (crbA / BI, crbB)
    logic        tch;                                  // dcbt / dcbtst: a translation fault is a no-op
    logic        icbi;                                 // icbi: its set goes to the I-cache from here
    logic [1:0]  hcfi;                                 // mtspr HID0: ICFI, DCFI as written
    logic        rtm;                                  // a bclr / bcctr the BTB predicted to the wrong
                                                       // target: MEM refetches from sprv (its target)
    // misaligned accesses: the first word of a split (no effect of its own; a
    // load's word is kept, mis_w0), a split load's second word (merged with
    // mis_w0), the alignment exception from EX (with lmw / stmw, DAR = EA + 4)
    logic        sp0, sp1, algn, algd4;
  } mem_t;
  mem_t m;
  // the page MEM translates a replay on (EA[0-19], from EX's ea_x): EA's, or for a split's second word once its
  // first has gone (m_x4) EA + 4's, a page crossing's second page (DMISS / DAR its first byte). Registers of their
  // own beside m, loaded as it is (mem_t's layout, and so the system bench's checkpoints, stay as they were)
  logic [19:0] m_xpg;
  logic        m_x4;
  (* preserve *) logic m_rpq;                          // = m.rp, for the queue's hold (below)
  wire         m_c1w = m.valid && m.fcr1 && !m_fsok;    // MEM holds an Rc = 1 arithmetic op: CR1 follows

  // WB
  logic        wb_valid, wb_last, wb_rd_we, wb_rd_sh;
  logic [1:0]  wb_irq;
  logic [4:0]  wb_evm;
  logic [4:0]  wb_rd;
  logic [31:0] wb_result, wb_pc, wb_insn;
  logic        wb_fpw, wb_fsl;                         // an FPR write in WB; lfs's conversion
  logic [4:0]  wb_frd;
  logic [63:0] wb_fdat_q;                              // the small unit's result
  // the data answer: WB waits for it (pipelined loads, see "Memory ports")
  logic        wb_iss, wb_got, wb_isld, wb_fhi, wb_c2; // a request made; answered; a load; lfd's high word; held last cycle
  logic        wb_sp0, wb_sp1;                         // a split's first word; a split load's second (its merge)
  logic        wb_fl;                                  // wb_fpw && wb_isld: an FP load
  logic        wb_live_q;                              // wb_isld && !wb_got, as its own register
  logic [1:0]  wb_msize, wb_boff;
  logic        wb_msigned, wb_brev;
  logic [31:0] wb_dq;                                  // the answer's word, once it has come
  logic [31:0] fhi_q;                                  // lfd's high word, for its second uop
  logic        wb_blk;                                 // WB writers wait for the divider's hand-over (FDL < 4)
  logic [10:0] wb_dly, m_dly;
  logic        wb_hold, wb_adv;
  logic        m_iss, wb_mayh;                         // MEM made a request; WB may yet hold
  logic [31:0] wb_val;                                 // the value WB writes and forwards

  // ====================================================================== MMU
  // 603e chapter 5. Real mode when MSR[IR] / MSR[DR] = 0 (5.2). Otherwise a BAT
  // hit (5.3) takes precedence over the page translation beside it (5.1.3);
  // the page path takes the segment register EA[0-3] selects (5.4) and the two
  // entries of the TLB set EA[15-19] selects, comparing V, the VSID and
  // EA[4-14] (5.4.3.1, figure 5-7). A miss takes one of the 603e's three TLB
  // miss exceptions for a software table search (5.5.2): there is no
  // hardware table walk. Protection: BAT PP alone (PEM 7.4.4); page PP under
  // the key Ks or Kp by MSR[PR] (PEM table 7-10); a store that hits an entry
  // with C = 0 takes the DTLB store miss (5.4.1.2, table 5-7). R is not kept:
  // every entry is referenced (5.4.1.1).
  //
  // Storage (study 8.4): each TLB way is its own MLAB array, so both ways read
  // in the same cycle; the segment registers are held twice, a copy for each
  // MMU, as the 603e shadows them for the IMMU (5.1.8); all are read
  // asynchronously. The BAT compare reads all four entries at once, so the
  // fields it needs are copied into flip-flops as a BAT is written; mfspr reads
  // the full registers from the SPR array. mfsr(in) has a third copy of the
  // segment registers. The replacement order per set is flip-flops.
  //
  // TLB entry: [62] V, [61:38] VSID, [37:27] EA[4-14], [26:7] RPN, [6] C,
  // [5:2] WIMG, [1:0] PP: the PTE's words with EA[10-14] added (5.4.3.1).
  // (R129) The TLB ways are M10K, read at a registered set: the instruction side's is f_rsp_pc's (as it loads),
  // the data side's MEM's page (as m_xpg loads, or kept). Each slow lookup reads them in the
  // cycle it uses them, as the MLABs were read; a TLB write on the edge into that cycle is not yet in what the
  // block gives, so that lookup waits a cycle (itw_q, dtw_q) and reads again.
  (* ramstyle = "M10K, no_rw_check" *) logic [62:0] dtlb0 [0:31];
  (* ramstyle = "M10K, no_rw_check" *) logic [62:0] dtlb1 [0:31];
  (* ramstyle = "M10K, no_rw_check" *) logic [62:0] itlb0 [0:31];
  (* ramstyle = "M10K, no_rw_check" *) logic [62:0] itlb1 [0:31];
  logic        itw_q, dtw_q;
  logic [62:0] it0_q, it1_q, dt0_q, dt1_q;            // the blocks' reads (old data on a write's edge, as M10K)
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] dsr_mem [0:15];
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] fsr_mem [0:15];   // mfsr(in)'s copy
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] isr_mem [0:15];
  logic [31:0] dlru, ilru;                             // per set: the way to replace next (SRR1[WAY])
  // BAT copies, entry {D, n}: BEPI, BL, Vs, Vp from BATnU; BRPN, WIMG, PP from BATnL
  logic [7:0][14:0] bat_bepi, bat_brpn;
  logic [7:0][10:0] bat_bl;
  logic [7:0]       bat_vs, bat_vp;
  logic [7:0][3:0]  bat_wimg;
  logic [7:0][1:0]  bat_pp;
  // The 603e table-search registers, DSISR and DAR are flip-flops: an
  // exception writes two of them on one commit (4.5.3, table 4-17). Zero at
  // reset (table 4-9).
  logic [31:0] dsisr_q, dar_q, dmiss_q, dcmp_q, imiss_q, icmp_q;
  logic        last_i_q;                               // the most recent TLB miss was the ITLB's
  logic [24:0] sdr1_q;                                 // SDR1 HTABORG and HTABMASK, for HASH1/HASH2
  logic [26:0] rpa_q;                                  // RPA's RPN, C, WIMG, PP, for tlbld / tlbli

  // BAT hit (PEM 7.4.3): BEPI against EA[0-14] with the BL bits masked off
  function automatic logic bat_match(input logic [14:0] ea_hi, input logic [14:0] bepi,
                                     input logic [10:0] bl, input logic valid);
    bat_match = valid && (ea_hi[14:11] == bepi[14:11]) && (((ea_hi[10:0] ^ bepi[10:0]) & ~bl) == 11'd0);
  endfunction

  // ==================================================================== fetch
  // Four-entry queue, packed so it stays in flip-flops (Quartus made an indexed
  // array into a RAM block). A request is issued whenever four slots would hold
  // it without counting this cycle's pop, so decode and hazards are off the
  // request path (R23 timing).
  //
  // Branches are predicted here, as each word arrives: b is taken; bc is taken
  // when its condition is always true, otherwise backward-taken with the y bit
  // reversing it (the 603e's static rule); bclr and bcctr are predicted not
  // taken for now. A predicted-taken branch is a soft redirect: fetch moves to
  // its target and anything in flight is discarded, but the queue keeps the
  // branch and what came before it. The prediction travels with the word to EX,
  // which redirects only on a mispredict.
  //
  // Hard redirects (WB: a fired trap; EX: mispredict, sc, rfi, mtmsr, illegal)
  // also clear the queue and hold ID for a cycle. Every redirect is registered
  // (redir_q); the next request takes its address from redir_pc_q, and that
  // request was always going out the cycle after the decision, so the register
  // costs no fetch latency.
  //
  // BTB (BTB = 1; study D-30, R61 design C; ppc_btb.sv). An entry is keyed by
  // the word before a branch B (B - 4) and holds B's target T, with T's
  // physical page and I-cache way. It is read as B - 4's request goes out
  // (rd_pc is f_pc's next value), so a hit is known (bt_pv) by the time B's
  // request goes out: that request carries the cache's pre-read of T (i_pv),
  // and the next request is T itself (pt_q, i_pre), answered in one cycle when
  // the cache's tag for the stored way is the stored page. A taken branch then
  // costs nothing. Both B - 4 and B must go out as f_pc requests, so the first
  // two words after a redirect are never predicted.
  //   - Every prediction is checked as B's word arrives, and the outcome is
  //     registered: the word must be a branch the static rule predicts taken
  //     to T, or for an entry of a bclr / bcctr (reg), such a branch the y-bit
  //     rule predicts taken. Otherwise (a stale entry) the next cycle discards
  //     T's answer and fetch moves to what the word itself asks (bmis_q), and
  //     a stale entry is invalidated.
  //   - A bclr / bcctr's target is checked in EX against the register; a
  //     difference is registered (mx.rtm) and MEM refetches from the right
  //     target, as for a resync. One such prediction is in flight at a time
  //     (rp_v, rp_tgt), so the target needs no space in the queue.
  //   - Entries are written for the branches the 603e's rule predicts taken
  //     (PEM 4.2.4.2: b; bc by S and y; bclr and bcctr when BO says always or
  //     y = 1) as their target's first word arrives, from that answer's way,
  //     page and arrays bit (a single read stores RSV_TAG, which never hits).
  //     The y bit is honoured: a branch the rule predicts not taken is never
  //     written, so a hit never predicts against it.
  //   - The page in an entry is trusted by the cache, so every change to
  //     instruction translation (mtsr, mtsrin, tlbie, tlbli, a BAT write)
  //     empties the table; tlbld does not touch fetch.
  logic [31:0] f_pc;
  logic        f_out, f_discard;
  logic [2:0]  q_cnt;

  logic        redirect;                               // hard, decided this cycle
  logic        redirect_c;                             // ... all but a mispredicted branch
  logic        br_go;                                  // a branch leaving EX
  wire  [31:0] redirect_pc;
  logic        redir_q, redir_hard_q;
  logic [31:0] redir_pc_q;
  // A redirect's address is chosen a cycle late from two registered candidates,
  // so a branch's taken bit lands in one flip-flop, not on 32 (R23 timing).
  logic [31:0] redir_a_q, redir_b_q, redir_s_q;     // hard taken / fall-through / soft
  logic [1:0]  redir_k_q;                            // which: 0 a, 1 b, 2 soft
  logic [31:0] redirect_pc_b;
  logic        redirect_sel;
  logic        id_take;                                // ID consumed the queue head

  // Instruction translation. Every fetch goes out when it would without
  // translation, so the decision to fetch waits on no compare. Its address is
  // translated beside it: real (MSR[IR] = 0), an IBAT that allows the fetch,
  // or the two-entry instruction micro-TLB of recent page translations. When
  // none of these serves it, the request goes to physical page 0 instead
  // (main RAM: harmless to read, and never a guarded or unmapped address),
  // marked f_xm, and its response is not used: fetch then spends two cycles
  // translating that address from the registered f_rsp_pc, the segment
  // register, both ITLB ways and protection (f_slow, the outcome registered),
  // then acting on it (f_slow2). An allowed translation fills the micro-TLB
  // and the address is fetched again; a refused one (ITLB miss, ISI)
  // completes as a fault response (f_fpend), which goes into
  // the queue as a marker in the word's place, so that it is ordered and
  // discarded exactly like a real response. Fetch then stops (f_stall) until a
  // hard redirect: the marker either reaches EX and takes its exception, or an
  // older instruction redirects first (4.1.2: fetch exceptions wait for the
  // stream to drain). The fault's details are held in ifl_*.
  //
  // f_ir / f_pr / f_tg are the MSR bits of the context being fetched: every
  // MSR change redirects, and the redirect loads them with the new value, so
  // the first fetch after an rfi, mtmsr or exception already uses its MSR (the
  // commit of msr_q comes a cycle later). The micro-TLB's entries carry the
  // MSR[PR] they were made under; every write to the MMU's state empties it.
  //
  // The IBATs are compared against each of the four addresses fetch may take
  // (f_pc and the three redirect candidates), all registers, and the result
  // chosen with the address, so the compare is not behind the address mux.
  logic        f_fpend, f_stall, f_slow, f_slow2, f_xm;
  logic        f_ir, f_pr, f_tg;                       // MSR[IR, PR, TGPR] as fetch must see them
  // the BTB's fetch state ("BTB" above; formed below)
  logic        bt_pv, bt_preg;                         // a prediction for the request after the next
  logic [31:2] bt_tgt;                                 // ... its target
  logic [23:0] bt_x;                                   // ... and {its key's size, slot, the target's page, way}
  logic        pt_q;                                   // B's request has gone: the next is bt_tgt
  logic        bq_out;                                 // the outstanding request is B's
  logic        bmis_q;                                 // B's word failed its check: discard, redirect
  logic [31:2] cr_tgt_q;                               // ... to what the word asks
  logic        rp_v;                                   // a bclr / bcctr predicted taken is in flight
  logic [31:2] rp_tgt;                                 // ... to this target
  wire  i_vld   = i_valid || f_fpend;
  wire  rsp     = f_out && i_vld;
  wire  rsp_ok  = rsp && !f_discard && !redir_q && !bmis_q;   // the response is the one wanted
  wire  rsp_xm  = rsp_ok && f_xm;                      // ... but it was not translated: translate it
  wire  rsp_use = rsp_ok && !f_xm;
  assign redir_pc_q = (redir_k_q == 2'd0) ? redir_a_q : (redir_k_q == 2'd1) ? redir_b_q : redir_s_q;
  wire  [31:0] fetch_pc = redir_q ? redir_pc_q : pt_q ? {bt_tgt, 2'b00} : f_pc;

  localparam int IUW = (IUN > 2) ? $clog2(IUN) : 2;   // (2 entries: bit 0 alone counts, as before)
  logic [IUN-1:0]        iu_v, iu_pr;
  logic [IUW-1:0]        iu_nx;
  wire  [$clog2(IUN)-1:0] iu_ix = iu_nx[$clog2(IUN)-1:0];   // the entry replaced next
  logic [IUN-1:0][19:0]  iu_pg, iu_pa;
  logic [IUN-1:0][3:0]   iu_wimg;
  logic [4:0][3:0]   fc_hit;                           // [candidate][IBAT]
  logic [4:0][IUN-1:0] fc_iu;                          // [candidate][micro-TLB entry]
  logic [4:0]        fc_ok;                            // candidate may go out at once
  logic [4:0][31:0]  fc_pc;
  always_comb begin
    fc_pc = {bt_tgt, 2'b00, redir_s_q, redir_b_q, redir_a_q, f_pc};
    for (int c = 0; c < 5; c++) begin
      fc_ok[c] = 1'b0;
      for (int k = 0; k < 4; k++) begin
        fc_hit[c][k] = bat_match(fc_pc[c][31:17], bat_bepi[k], bat_bl[k], f_pr ? bat_vp[k] : bat_vs[k]);
        // a BAT hit that allows the fetch: PP not 00, not guarded (table 5-3)
        fc_ok[c] = fc_ok[c] | (fc_hit[c][k] && bat_pp[k] != 2'd0 && !bat_wimg[k][0]);
      end
      for (int e = 0; e < IUN; e++) begin
        fc_iu[c][e] = iu_v[e] && iu_pr[e] == f_pr && iu_pg[e] == fc_pc[c][31:12];
        fc_ok[c]    = fc_ok[c] | fc_iu[c][e];
      end
    end
  end
  wire  [2:0]  f_sel  = !redir_q ? (pt_q ? 3'd4 : 3'd0) : (redir_k_q == 2'd0) ? 3'd1 : (redir_k_q == 2'd1) ? 3'd2 : 3'd3;
  wire  [3:0]  ib_hit = fc_hit[f_sel];
  // the micro-TLB's hit chosen too, not compared after the address mux (the
  // R63 full build's ROM address path ran through that compare)
  wire  [IUN-1:0] iu_hit = fc_iu[f_sel];
  // with more than two entries the micro-TLB's page and WIMG are formed for each of the five candidates
  // beside its compare, and the address's choice (f_sel, from registers) picks one last: through iu_hit,
  // the choice and an eight-way AND-OR were in front of the I-cache's pa (an IUN = 8 spike's worst I-cache path)
  logic [4:0][19:0] iu_pa_c;
  logic [4:0][3:0]  iu_wg_c;
  always_comb
    for (int c = 0; c < 5; c++) begin
      iu_pa_c[c] = 20'd0; iu_wg_c[c] = 4'd0;
      for (int e = 0; e < IUN; e++) begin
        iu_pa_c[c] = iu_pa_c[c] | ({20{fc_iu[c][e]}} & iu_pa[e]);
        iu_wg_c[c] = iu_wg_c[c] | ({4{fc_iu[c][e]}} & iu_wimg[e]);
      end
    end
  wire         f_fast = !f_ir || fc_ok[f_sel];
  logic [14:0] ib_pa;
  logic [19:0] iu_pahi;
  logic        ib_go;
  logic [3:0]  ib_wimg, iu_wimg_s;
  always_comb begin
    ib_pa = 15'd0; iu_pahi = 20'd0; ib_go = 1'b0; ib_wimg = 4'd0; iu_wimg_s = 4'd0;
    // only a BAT that allows the fetch gives an address (a double BAT hit is a
    // programming error, 5.3: the hits are ORed)
    for (int k = 0; k < 4; k++) begin
      ib_go = ib_go | (ib_hit[k] && bat_pp[k] != 2'd0 && !bat_wimg[k][0]);
      ib_pa = ib_pa | ({15{ib_hit[k] && bat_pp[k] != 2'd0 && !bat_wimg[k][0]}} &
                       (bat_brpn[k] | {4'd0, fetch_pc[27:17] & bat_bl[k]}));
      ib_wimg = ib_wimg | ({4{ib_hit[k]}} & bat_wimg[k] & 4'b1110);   // an IBAT has no G (3.5)
    end
    if (IUN > 2) begin                                  // (formed per candidate, chosen by f_sel last: below)
      iu_pahi = iu_pa_c[f_sel]; iu_wimg_s = iu_wg_c[f_sel];
    end else
      for (int e = 0; e < IUN; e++) begin
        iu_pahi   = iu_pahi   | ({20{iu_hit[e]}} & iu_pa[e]);
        iu_wimg_s = iu_wimg_s | ({4{iu_hit[e]}} & iu_wimg[e]);
      end
  end
  // the fetch's WIMG: real mode 0011 (3.5), an IBAT's, or the micro-TLB's
  // (an untranslated request's goes nowhere: its response is not used)
  assign i_wimg = !f_ir ? 4'b0011 : ib_go ? ib_wimg : iu_wimg_s;
  // with no translation for it, physical page 0
  wire  [19:0] i_pahi = !f_ir ? fetch_pc[31:12] : {ib_pa, fetch_pc[16:12] & {5{ib_go}}} | iu_pahi;

  // the slow cycles: f_rsp_pc's translation, from registers
  wire  [31:0] isr    = isr_mem[f_rsp_pc[31:28]];
  wire  [62:0] it0    = it0_q;
  wire  [62:0] it1    = it1_q;
  logic [1:0]  sb_pp;
  logic        sb_g;
  logic [3:0]  sb_hit, sb_wimg;
  always_comb begin
    sb_pp = 2'd0; sb_g = 1'b0; sb_wimg = 4'd0;
    for (int k = 0; k < 4; k++) begin
      sb_hit[k] = bat_match(f_rsp_pc[31:17], bat_bepi[k], bat_bl[k], f_pr ? bat_vp[k] : bat_vs[k]);
      sb_pp = sb_pp | ({2{sb_hit[k]}} & bat_pp[k]);
      sb_wimg = sb_wimg | ({4{sb_hit[k]}} & bat_wimg[k] & 4'b1110);
      sb_g  = sb_g  | (sb_hit[k] && bat_wimg[k][0]);
    end
  end
  wire         sb_any = |sb_hit;
  wire         ih0    = it0[62] && it0[61:38] == isr[23:0] && it0[37:27] == f_rsp_pc[27:17];
  wire         ih1    = it1[62] && it1[61:38] == isr[23:0] && it1[37:27] == f_rsp_pc[27:17];
  wire  [62:0] ie     = ih0 ? it0 : it1;              // a double hit takes way 0
  wire         i_key  = f_pr ? isr[29] : isr[30];     // Kp : Ks
  wire         i_tn   = isr[31] || isr[28];           // direct-store (T) or no-execute (N) segment
  // ISI (table 5-3): SRR1[3] for T, N or guarded (BAT or PTE G); SRR1[4] for
  // no read access. T and N fault before the TLB is consulted (figure 5-6).
  wire         i_s3   = sb_any ? sb_g : (i_tn || ((ih0 || ih1) && ie[2]));
  wire         i_s4   = sb_any ? (sb_pp == 2'd0) : (!i_tn && (ih0 || ih1) && i_key && ie[1:0] == 2'd0);
  wire         i_miss = !sb_any && !i_tn && !(ih0 || ih1);
  logic        sf_q;                                   // the lookup refused the fetch
  logic [19:0] sf_pa;                                  // or its RPN
  logic [3:0]  sf_wimg;                                // and WIMG
  wire         s_flt  = f_slow2 && !redir_q && sf_q;
  wire         s_ok   = f_slow2 && !redir_q && !sf_q;
  logic        ifl_miss, ifl_key;                      // the held fetch fault: ITLB miss, else ISI
  logic [1:0]  ifl_s34;                                // ISI SRR1[3], SRR1[4]
  logic [23:0] ifl_vsid;                               // ICMP's VSID

  // prediction from the arriving word
  logic [31:0] f_rsp_pc;
  wire  [5:0]  pw_op   = i_data[31:26];
  wire  [4:0]  pw_bo   = i_data[25:21];
  wire  [31:0] pw_disp = (pw_op == 6'd18) ? {{6{i_data[25]}}, i_data[25:2], 2'b00}
                                          : {{16{i_data[15]}}, i_data[15:2], 2'b00};
  // (a fault marker's word is whatever is on the bus, and may predict a
  // branch: harmless, since fetch has stopped and the marker's exception in EX
  // overrides any redirect of its own)
  // Direction (D-30), the 603e's rule: a conditional branch is resolved from
  // the committed CR and CTR when no instruction in flight writes them (the
  // queue's entries, EX, MEM: 603e 2.3.4.4 "scans the execution pipelines
  // ... if no interlock is found, the branch can be resolved immediately");
  // otherwise the static rule predicts it (PEM 4.2.4.2: S and the y bit).
  // The scan is per register, not per CR bit (a writer of any field counts),
  // which only resolves fewer branches. A word behind another in a two-word
  // answer counts that word as in flight too.
  // (declared here, before the look-ahead's scan uses them: Quartus 17 wants
  // a struct declared before its fields are named)
  pd_t         pd_w, pd_w2, q_pd, q1_pd;
  logic [17:0] q_rf;                                   // the queue head's {rS, rA, rB, sh}, a copy for the GPR file
  logic [PDW-1:0] q2_pdv, q3_pdv;
  pd_t         q2_pd, q3_pd;
  assign q2_pd = pd_t'(q2_pdv);
  assign q3_pd = pd_t'(q3_pdv);
  wire         la_crb = (q_cnt > 3'd0 && (q_pd.w_cr  || q_pd.fwcr))  || (q_cnt > 3'd1 && (q1_pd.w_cr || q1_pd.fwcr)) ||
                        (q_cnt > 3'd2 && (q2_pd.w_cr || q2_pd.fwcr)) || (q_cnt > 3'd3 && (q3_pd.w_cr || q3_pd.fwcr)) ||
                        (ex_valid && ex_w_cr) || (m.valid && m.cr_we);
  wire         la_ctb = (q_cnt > 3'd0 && q_pd.w_ctr)  || (q_cnt > 3'd1 && q1_pd.w_ctr) ||
                        (q_cnt > 3'd2 && q2_pd.w_ctr) || (q_cnt > 3'd3 && q3_pd.w_ctr) ||
                        (ex_valid && ex_w_ctr) || (m.valid && m.ctr_we);
  // resolved (r) and its direction (d) for a word w with BO bo, BI bi: a bc,
  // bclr or bcctr that is not "branch always"
  function automatic logic [1:0] la_f(input logic [31:0] w, input logic crb, input logic ctb,
                                      input logic [31:0] cr, input logic c1);
    logic [4:0] bo;
    logic cond, r, d;
    bo   = w[25:21];
    cond = w[31:26] == 6'd16 || (w[31:26] == 6'd19 && (w[10:1] == 10'd16 || (w[10:1] == 10'd528 && bo[2])));
    r    = cond && !(bo[4] && bo[2]) && (bo[4] || !crb) && (bo[2] || !ctb);
    d    = (bo[2] || (!c1 ^ bo[1])) && (bo[4] || cr[5'd31 - w[20:16]] == bo[3]);
    la_f = {r, d};
  endfunction
  wire  [1:0]  la1 = LA ? la_f(i_data, la_crb, la_ctb, cr_q, ctr1_q) : 2'b00;
  wire         pw_spred = (pw_op == 6'd18) ||
                          (pw_op == 6'd16 && ((pw_bo[4] && pw_bo[2]) || (i_data[15] ^ pw_bo[0])));
  wire         pw_pred = la1[1] ? (pw_op == 6'd16 && la1[0]) : pw_spred;
  wire  [31:0] pw_tgt  = (i_data[1] ? 32'd0 : f_rsp_pc) + pw_disp;
  // a word the BTB already redirected for (bq_out) makes no soft redirect of
  // its own: its check is registered (bmis_q, below)
  wire         f_soft1 = rsp_use && (pw_pred || pw_lrs) && !bq_out;
  wire         f_lra   = rsp_use && pw_lrs && !bq_out;           // (pw_pred is 0 for a bclr / bcctr)
  wire         f_soft  = f_soft1 || f_soft2;
  // bclr, or bcctr (BO[2] = 1), and the y-bit rule's taken for it (PEM 4.2.4.2)
  wire         pw_reg  = pw_op == 6'd19 && (i_data[10:1] == 10'd16 || (i_data[10:1] == 10'd528 && pw_bo[2]));
  wire         pw_rstk = la1[1] ? la1[0] : (pw_bo[4] && pw_bo[2]) || pw_bo[0];
  // LR / CTR look-ahead (LRA). Fetch keeps what the LR will be once the words it has pushed into the queue
  // have executed: lw_k 0, none of them writes the LR (since the queue was last emptied by a hard redirect),
  // so the LR is lr_q once no older writer is in EX or MEM (it commits as MEM is left); 1, the youngest writer
  // pushed is a branch with LK = 1, so the LR is its address + 4 (lw_lr, the 603e's LR rename register for a
  // branch); 2, another writer (mtlr): not known until it executes (the 603e waits; this core predicts as
  // before). The CTR is known when no writer of it is in the queue, EX or MEM (la_ctb): ctr_q. A bclr / bcctr
  // first word that branches always (BO = 1z1zz) whose target is known, with no other register prediction in
  // flight (rp_v), is a soft redirect to that target (f_lra), checked in EX as a BTB register prediction is
  // (rp_tgt, rtm). With a BTB register prediction for it, the BTB's target must be that one (bq_ok1).
  logic [1:0]  lw_k;
  logic [31:2] lw_lr;
  wire         pw_bcr  = pw_op == 6'd19 && (i_data[10:1] == 10'd16 || i_data[10:1] == 10'd528);   // bclr / bcctr, any BO
  wire         pw_lk   = (pw_op == 6'd18 || pw_op == 6'd16 || pw_bcr) && i_data[0];               // a branch with LK = 1
  wire         lr_old  = (ex_valid && ex_w_lr) || (m.valid && m.lr_we);
  wire         pw_isl  = i_data[10:1] == 10'd16;
  // (with the BTB only: its rp_v / rp_tgt carry the prediction to EX's check; one-word fetch only)
  // (only a bclr / bcctr that branches always, BO = 1z1zz: its direction needs nothing from the look-ahead's CR
  // scan, so the decision is two LUTs from the word and registers; a conditional one is predicted as before)
  wire         lr_kn   = lw_k == 2'd1 || (lw_k == 2'd0 && !lr_old);
  wire         pw_kn   = LRA && BTB && !FETCH2 && pw_reg && pw_bo[4] && pw_bo[2] && (pw_isl ? lr_kn : !la_ctb);
  wire  [31:2] pw_rt   = pw_isl ? ((lw_k == 2'd1) ? lw_lr : lr_q[31:2]) : ctr_q[31:2];
  wire         pw_lrs  = pw_kn && !rp_v;                           // a soft redirect if the BTB has not redirected
  // the queue's predicted bit: the static rule's, or a register branch the BTB
  // redirected for, whose word passes its check
  wire         bt_slot = FETCH2 && bt_x[22];            // the branch is the second word of its request
  wire         w_prd   = pw_pred || (bq_out && !bt_slot && bt_preg && pw_reg && pw_rstk) || (pw_lrs && !bq_out);
  // two-word fetch: the second word's prediction, from i_data2 at f_rsp_pc + 4.
  // A first word predicted taken drops the second (it is not pushed).
  logic        f_w2_q;                                 // the outstanding request asked for two words
  wire  [31:2] f_rsp4  = f_rsp_pc[31:2] + 30'd1;
  wire  [31:2] f_rsp8  = f_rsp_pc[31:2] + 30'd2;
  wire  [5:0]  pw2_op  = i_data2[31:26];
  wire  [4:0]  pw2_bo  = i_data2[25:21];
  wire  [31:0] pw2_disp = (pw2_op == 6'd18) ? {{6{i_data2[25]}}, i_data2[25:2], 2'b00}
                                            : {{16{i_data2[15]}}, i_data2[15:2], 2'b00};
  // the first word as a writer, from its raw fields: a superset of the CR and
  // CTR writers, shallow (the predecode's own bits sat behind its tables and
  // led a cached full build, i_data through la2 into the queue's bit)
  function automatic logic [1:0] w_may(input logic [31:0] w);
    logic [5:0] op;
    logic [9:0] x;
    op = w[31:26]; x = w[10:1];
    w_may[1] = op == 6'd10 || op == 6'd11 || op == 6'd13 || op == 6'd19 || op == 6'd28 || op == 6'd29 ||
               op == 6'd59 || op == 6'd63 || ((op == 6'd20 || op == 6'd21 || op == 6'd23) && w[0]) ||
               (op == 6'd31 && (w[0] || x == 10'd0 || x == 10'd32 || x == 10'd144 || x == 10'd512));
    w_may[0] = ((op == 6'd16 || op == 6'd19) && !w[23]) || (op == 6'd31 && x == 10'd467);
  endfunction
  wire  [1:0]  w1_may = w_may(i_data);
  wire  [1:0]  la2 = (LA && FETCH2) ? la_f(i_data2, la_crb || w1_may[1], la_ctb || w1_may[0], cr_q, ctr1_q) : 2'b00;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && rsp_use && !f_fpend && (((pd_w.w_cr || pd_w.fwcr) && !w1_may[1]) || (pd_w.w_ctr && !w1_may[0])))
      $fatal(1, "w_may misses a CR or CTR writer: %08X", i_data);
`endif
  wire         pw2_spred = (pw2_op == 6'd18) ||
                           (pw2_op == 6'd16 && ((pw2_bo[4] && pw2_bo[2]) || (i_data2[15] ^ pw2_bo[0])));
  wire         pw2_pred = FETCH2 && (la2[1] ? (pw2_op == 6'd16 && la2[0]) : pw2_spred);
  wire  [31:0] pw2_tgt  = (i_data2[1] ? 32'd0 : {f_rsp4, 2'b00}) + pw2_disp;
  wire         pw2_reg  = pw2_op == 6'd19 && (i_data2[10:1] == 10'd16 || (i_data2[10:1] == 10'd528 && pw2_bo[2]));
  wire         pw2_rstk = la2[1] ? la2[0] : (pw2_bo[4] && pw2_bo[2]) || pw2_bo[0];
  wire         w2_in    = FETCH2 && f_w2_q;            // the answer has a second word
  wire         f_soft2  = rsp_use && w2_in && !pw_pred && pw2_pred && !bq_out;
  wire         w2_prd   = pw2_pred || (bq_out && bt_slot && bt_preg && pw2_reg && pw2_rstk);

  // A fetch goes out (f_issue) as without translation, except while
  // translating and on the cycle its untranslated response comes back (the
  // address is fetched again after). A fault in the slow cycles stands for a
  // request (its response is the marker, next cycle). None goes out on a fault
  // response's cycle, so the held fault is never overwritten before its marker
  // is in the queue.
  // (an answer counts as two words if two were asked for, pushed or not)
  wire  [2:0] f_nin  = {1'b0, rsp_use && w2_in, rsp_use && !w2_in};
  wire  f_issue = !rst && !f_stall && !f_fpend && !f_slow && !f_slow2 && !rsp_xm && (!f_out || rsp) &&
                  (redir_hard_q || ({1'b0, q_cnt} + {1'b0, f_nin} < 4'd4));
  wire  f_room2 = {1'b0, q_cnt} + {1'b0, f_nin} < 4'd3;
  assign i_addr = {i_pahi, fetch_pc[11:0]};
  assign i_eix  = fetch_pc[12];
  assign i_req  = f_issue;
  // i_seq: the request follows the previous one at + 4, both translated at
  // once (a request that went untranslated, f_xm, is fetched again from a
  // redirect-like reload, s_ok, which is never sequential)
  // (the translation part is f_xm's: a request that went out translated
  // has f_xm = 0, so f_fast drives one register, not two)
  logic f_sq;
  assign i_seq  = !redir_q && !pt_q && f_sq && !f_xm;
  always_ff @(posedge clk)
    if (rst)                         f_sq <= 1'b0;
    else if (f_issue)                f_sq <= 1'b1;
    else if (redir_q || s_ok || s_flt) f_sq <= 1'b0;

  // micro-TLB invalidation (UTS). An entry for page p (EA[0-19]) holds what the TLB set p's EA[15-19]
  // and segment register EA[0-3] gave (5.4.3: the TLBs are indexed by EA[15-19]). tlbie invalidates both
  // ways of one set and tlbld / tlbli load one entry of one set, of one TLB (603e 5.4.3.2, 2.3.8,
  // 5.5.2.2.2); mtsr / mtsrin change one segment register. Only the entries of that set (tlbie, and the
  // load of this side's TLB) or of that segment are then emptied; the others still hold what the TLB and
  // the segment registers give, as the 603e's other TLB entries stay valid. A BAT write empties all.
  // (R140: from registers, m_dvok last)
  (* keep = 1 *) wire iu_clr_e = m_okb && (m_bat || (!UTS && (m.srw || m.tlbie || m.tlbl)));
  (* keep = 1 *) wire iu_sel_e = UTS && m_okb && (m.srw || m.tlbie || (m.tlbl && m.insn[6]));
  wire  iu_clr  = iu_clr_e && m_dvok;
  wire  iu_sel  = iu_sel_e && m_dvok;
  always_ff @(posedge clk) begin
    if (rst) begin
      f_fpend <= 1'b0;
      f_stall <= 1'b0;
      f_xm    <= 1'b0;
      f_slow  <= 1'b0;
      f_slow2 <= 1'b0;
      iu_v    <= '0;
      iu_nx   <= '0;
    end else begin
      f_fpend <= s_flt;
      if (f_issue)    f_xm <= !f_fast;
      else if (s_flt) f_xm <= 1'b0;                     // the marker's response is used
      f_slow  <= rsp_xm || (f_slow && itw_q);       // (an ITLB write on the edge into it: read again)
      f_slow2 <= f_slow && !itw_q && !redir_q;
      if (redir_hard_q)                  f_stall <= 1'b0;
      else if (rsp_use && f_fpend)       f_stall <= 1'b1;
      if (iu_clr) iu_v <= '0;
      else if (iu_sel) begin                            // (a fill in this cycle is dropped: it is refetched)
        for (int e = 0; e < IUN; e++)
          if (m.srw ? iu_pg[e][19:16] == m.ya[19:16] : iu_pg[e][4:0] == m.ya[4:0]) iu_v[e] <= 1'b0;
      end else if (s_ok) begin
        iu_v[iu_ix]    <= 1'b1;
        iu_pg[iu_ix]   <= f_rsp_pc[31:12];
        iu_pa[iu_ix]   <= sf_pa;
        iu_pr[iu_ix]   <= f_pr;
        iu_wimg[iu_ix] <= sf_wimg;
        iu_nx          <= (IUN > 2) ? IUW'(iu_nx + 1'b1) : IUW'({1'b0, !iu_nx[0]});
      end
    end
    if (f_slow) begin
      sf_q     <= i_s3 || i_s4 || i_miss;
      sf_pa    <= ie[26:7];
      sf_wimg  <= sb_any ? sb_wimg : ie[5:2];
      ifl_miss <= i_miss;
      ifl_s34  <= {i_s3, i_s4};
      ifl_vsid <= isr[23:0];
      ifl_key  <= i_key;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc         <= RESET_PC;
      f_out        <= 1'b0;
      f_discard    <= 1'b0;
      redir_q      <= 1'b0;
      redir_hard_q <= 1'b0;
    end else begin
      // (a failed check made in the cycle of a hard redirect is dropped here,
      // a cycle on, so EX's late decision does not reach bmis_q)
      redir_q      <= rdf_x || f_soft || (bmis_q && !redir_hard_q);
      redir_hard_q <= rdf_x;
      // the candidates load every cycle; only the 2-bit choice waits on the
      // redirect decision (R23 timing). A failed BTB check's redirect is a
      // soft one, its address loaded from a register (cr_tgt_q).
      redir_a_q    <= redirect_pc;
      redir_b_q    <= redirect_pc_b;
      redir_s_q    <= bmis_q ? {cr_tgt_q, 2'b00} : (LRA && !pw_pred && pw_lrs) ? {pw_rt, 2'b00} : (pw_pred || !FETCH2) ? pw_tgt : pw2_tgt;
      redir_k_q    <= rdk_x;
      if (f_issue)      f_pc <= fetch_pc + (i_w2 ? 32'd8 : 32'd4);
      else if (redir_q) f_pc <= redir_pc_q;
      else if (s_ok)    f_pc <= f_rsp_pc;              // fetch it again, now translated
      if (redir_q) begin
        f_out     <= f_issue || (f_out && !i_vld);
        f_discard <= f_out && !i_vld;                // an older request still in flight
      end else begin
        if (rsp) begin
          f_out     <= 1'b0;
          f_discard <= 1'b0;
        end
        if (f_issue || s_flt) f_out <= 1'b1;
      end
    end
  end

  // the LR look-ahead's state, by the words pushed (LRA, above)
  always_ff @(posedge clk)
    if (rst || redir_hard_q) lw_k <= 2'd0;
    else if (rsp_use && !f_fpend) begin
      if (pw_lk) begin lw_k <= 2'd1; lw_lr <= f_rsp4; end
      else if (pd_w.w_lr) lw_k <= 2'd2;
    end
`ifdef R104_CHECK
  // every LR writer the predecode names is a branch with LK = 1 or not, and pw_lk names only LR writers
  always_ff @(posedge clk)
    if (!rst && LRA && rsp_use && !f_fpend && pw_lk && !pd_w.w_lr) $fatal(1, "LRA: a branch with LK = 1 the predecode does not mark as an LR writer");
`endif

  // Queue writes go to the slot after the live entries, counted before any pop.
  logic [31:0] q_hi, q_hp;
  logic        q_hd;
  always_ff @(posedge clk) if (f_issue) f_rsp_pc <= fetch_pc;
  // the outstanding request: two words; the one before it was two; it was
  // sequential (the BTB's key for a branch in it is the request before)
  logic        rq_pp_q, rq_sq_q;
  always_ff @(posedge clk) begin
    if (rst || s_flt) f_w2_q <= 1'b0;                   // (a fault marker is one word)
    else if (f_issue) f_w2_q <= i_w2;
    if (f_issue) begin
      rq_pp_q <= f_w2_q;
      rq_sq_q <= i_seq;
    end
  end
  wire  [31:2] f_rkey = f_rsp_pc[31:2] - (rq_pp_q ? 30'd2 : 30'd1);

  // ---------------------------------------------------------------- BTB
  // (see "BTB" above). b_go: B's request goes out now, with the pre-read;
  // t_go: T's. A register prediction waits while another is in flight.
  // With two-word fetch an entry also says which word of B's request is the
  // branch (bt_slot): the first, and that request asks for one word; the
  // second, and it must be a two-word request, or the prediction is dropped.
  logic        bt_kill;
  wire         bt_use = bt_pv && !(bt_preg && rp_v);
  // (never across a line, so never across a page: the second word is
  // translated with the first; the cache does not offer it there anyway)
  wire         w2_can = FETCH2 && i_two && i_seq && f_room2 && fetch_pc[4:2] != 3'd7;   // this request may ask for two
  // the entry also holds the size of its key request (one or two words): the
  // request after the key is then the one the entry was written for, so the
  // slot means the same word. f_w2_q is the key request's size while B's
  // request is still to go (no request goes out in between).
  wire         bt_xok = !FETCH2 || bt_x[23] == f_w2_q;
  wire         b_ok   = bt_use && bt_xok && !pt_q && !redir_q && (!bt_slot || w2_can);
  wire         b_go   = f_issue && b_ok;
  wire         t_go   = f_issue && pt_q && !redir_q;
  assign i_w2   = w2_can && !(b_ok && !bt_slot);
  assign i_pv   = b_go;
  assign i_pidx = bt_tgt[12:2];
  assign i_pway = bt_x[1:0];
  assign i_ppa  = bt_x[21:2];
  assign i_pre  = pt_q && !redir_q;
  // any change of stream drops a prediction (and one whose slot cannot be met)
  // (a hard redirect kills from its registered copy, a cycle on: until then
  // b_go and t_go wait on redir_q, and EX's late decision stays off these)
  assign bt_kill = redir_hard_q || f_soft || bmis_q || s_ok || s_flt || rsp_xm || (bt_pv && bt_preg && rp_v) ||
                   (f_issue && bt_use && !pt_q && !redir_q && bt_slot && !w2_can) || (bt_pv && !pt_q && !bt_xok);
  // B's check, from its word: a branch the rule predicts taken to T (pw_tgt
  // is T's adder, the compare goes to one register), or for a register entry
  // a bclr / bcctr the y-bit rule predicts taken; in the second slot, with a
  // first word that predicts nothing
  wire         bq_ok1 = bt_preg ? (pw_reg && pw_rstk && (!pw_kn || pw_rt == bt_tgt)) : (pw_pred && pw_tgt[31:2] == bt_tgt);
  wire         bq_ok2 = w2_in && !pw_pred && (bt_preg ? (pw2_reg && pw2_rstk) : (pw2_pred && pw2_tgt[31:2] == bt_tgt));
  wire         bq_ok  = bt_slot ? bq_ok2 : bq_ok1;
  logic        bmr_q;                                  // the failed check was a register entry's
  logic        cr_lrk_q;                               // ... and the word's target is known (LRA): a register prediction to it
  logic        al_lr1;                                 // LRA: the soft redirect was a bclr / bcctr's (a register entry)
  logic        rp_lq;                                  // LRA: the register prediction in flight is the look-ahead's
  logic        cr_pred_q;                              // ... and a word predicts taken (to cr_tgt_q)
  logic        cr_slot_q, cr_sq_q, cr_pp_q;            // ... which word; its request was sequential; the key's size
  logic [31:2] cr_key_q;                               // ... the key of B's request (the one before it)
  // the BTB's writes: an allocation as a redirect target's first word
  // arrives; an invalidation for a stale entry. Keys are the request before
  // the branch's (the word before it with one word a request).
  logic        al_v, al_reg, al_slot, al_pp;
  logic        al_sq, al_s2q;                          // a soft redirect last cycle; for the second word
  logic        al_pp1, al_sv1;                         // ... and its key, registered every cycle
  logic [31:2] al_k1;
  logic [31:2] al_key;
  logic        rx_q, rt_q;                             // last cycle: EX redirected for a taken bclr / bcctr
                                                       // the rule predicts taken; MEM refetched (rtm)
  logic [31:2] rx_key_q;                               // the key of the last uop to leave EX
  logic        rx_slot_q, rx_kv_q, rx_pp_q;
  wire         bw_al  = rsp_use && al_v;
  wire         bw_inv = bmis_q && !cr_pred_q;
  wire  [31:2] bw_key = bw_inv ? cr_key_q : al_key;
  wire         bt_flush = m_ok && (m.srw || m.tlbie || (m.tlbl && m.insn[6]) || m_bat);
  // the address the coming request is expected to carry, so its entry is out
  // when it goes: after a soft redirect, B's request or a failed check, their
  // target; else the next sequential one. A wrong guess only misses a lookup
  // (the BTB compares the request's address, CURPC). A hard redirect's
  // target is not known in time, so the request after one is not looked up.
  // A soft redirect's target (SOFTLU = 1) is presented on the static rule's
  // guess, not on the look-ahead's direction: with the look-ahead's CR select
  // in it, the arriving word into the table's address led the first cached
  // full build (-0.44 ns). A wrong guess only misses the lookup.
  wire         g_soft1 = rsp_use && pw_spred && !bq_out;
  wire         g_soft  = g_soft1 || (rsp_use && w2_in && !pw_spred && pw2_spred && !bq_out);
  wire  [31:2] bt_rd = (SOFTLU && g_soft) ? (g_soft1 ? pw_tgt[31:2] : pw2_tgt[31:2]) : bmis_q ? cr_tgt_q :
                       (b_go || (pt_q && !f_issue)) ? bt_tgt :
                       f_issue ? fetch_pc[31:2] + (i_w2 ? 30'd2 : 30'd1) : redir_q ? redir_pc_q[31:2] :
                       s_ok ? f_rsp_pc[31:2] : f_pc[31:2];
  assign ev_btb = {bw_al, rt_q && redir_hard_q, bmis_q, t_go};
  genvar bce;
  generate
    if (BTB) begin : g_btb
      // the request's address against the lookup's (the BTB's CURPC compare), made for each of fetch's
      // five candidates and chosen by f_sel as the address is: redir_a_q through fetch_pc's mux and the
      // 30-bit compare into the prediction's enables was -0.57 ns at 100 MHz (study R104)
      logic [31:2] bt_rdq;
      always_ff @(posedge clk) bt_rdq <= bt_rd;
      // (kept: without it synthesis shared one compare behind the candidates' mux again)
      (* keep = 1 *) wire [4:0] bt_ceq;
      for (bce = 0; bce < 5; bce++) begin : g_bceq
        assign bt_ceq[bce] = fc_pc[bce][31:2] == bt_rdq;
      end
      ppc_btb #(.N (BTB_N), .XW (24), .STYLE (0), .CURPC (1'b1), .CUREQ (1'b1), .TD (2)) u_btb (
        .clk (clk), .rst (rst),
        .rd_pc (bt_rd),
        .ctx ({f_ir, f_pr}),
        .cap (f_issue), .cur_pc (fetch_pc[31:2]), .cur_eq (bt_ceq[f_sel]),
        .p_v (bt_pv), .p_tgt (bt_tgt), .p_reg (bt_preg), .p_x (bt_x),
        .p_take (t_go), .kill (bt_kill),
        .w_en (bw_al || bw_inv), .w_v (bw_al), .w_pc (bw_key), .w_ctx ({f_ir, f_pr}), .w_reg (al_reg),
        .w_tgt (f_rsp_pc[31:2]), .w_x ({FETCH2 && al_pp, al_slot, i_arr ? i_pa : 20'hF00FF, i_way}),
        .flush (bt_flush), .busy ()
      );
      always_ff @(posedge clk) begin
        if (rst || bt_kill) pt_q <= 1'b0;
        else if (b_go)      pt_q <= 1'b1;
        else if (t_go)      pt_q <= 1'b0;
        if (rst || bt_kill) bq_out <= 1'b0;
        else if (f_issue)   bq_out <= b_go;
        // (a hard redirect in the same cycle discards B's word: its
        // correction is dropped at redir_q, below)
        bmis_q    <= rsp_use && bq_out && !bq_ok && !rst;
        bmr_q     <= bt_preg;
        cr_pred_q <= pw_pred || (w2_in && pw2_pred) || pw_kn;
        cr_lrk_q  <= pw_kn;
        cr_slot_q <= !pw_pred;
        cr_tgt_q  <= pw_pred ? pw_tgt[31:2] : pw_kn ? pw_rt : (w2_in && pw2_pred) ? pw2_tgt[31:2] : w2_in ? f_rsp8 : f_rsp4;
        cr_key_q  <= f_rkey;
        cr_sq_q   <= rq_sq_q;
        cr_pp_q   <= rq_pp_q;
        // the register prediction in flight: from T's request until the
        // branch leaves EX (its target is checked there), or its word fails
        if (rst || redir_hard_q)                                rp_v <= 1'b0;
        else if (t_go && bt_preg)                               rp_v <= 1'b1;
        else if (LRA && f_lra)                                  rp_v <= 1'b1;
        else if (LRA && bmis_q && bmr_q && cr_lrk_q)            rp_v <= 1'b1;   // the BTB's target replaced by the known one
        else if ((bmis_q && bmr_q) || (ex_adv && ex_u.op == OP_BRANCH && ex_pred &&
                                       (ex_u.bkind == BR_BCLR || ex_u.bkind == BR_BCCTR))) rp_v <= 1'b0;
        if (rst || redir_hard_q || t_go) rp_lq <= 1'b0;
        else if (LRA && (f_lra || (bmis_q && bmr_q && cr_lrk_q))) rp_lq <= 1'b1;
        if (t_go) rp_tgt <= bt_tgt;
        else if (LRA && f_lra) rp_tgt <= pw_rt;
        else if (LRA && bmis_q && bmr_q && cr_lrk_q) rp_tgt <= cr_tgt_q;
        // allocation. The key of the last uop to leave EX is kept until another
        // leaves (a MEM refetch, rt_q, uses it too: nothing younger leaves EX
        // past a uop MEM kills)
        rx_q    <= ex_adv && ex_u.op == OP_BRANCH && !ex_xcm && (ex_u.bkind == BR_BCLR || ex_u.bkind == BR_BCCTR) &&
                   ((ex_u.bo[4] && ex_u.bo[2]) || ex_u.bo[0]);
        if (ex_adv && !m_kp) begin
          rx_key_q  <= ex_u.pc[31:2] - {29'd0, ex_bk[2]} - (ex_bk[1] ? 30'd2 : 30'd1);
          rx_slot_q <= ex_bk[2];
          rx_kv_q   <= ex_bk[0];
          rx_pp_q   <= ex_bk[1];
        end
        rt_q    <= m_kill && m.rtm;
        al_sq   <= f_soft && !rst;
        al_lr1  <= LRA && f_lra;
        al_s2q  <= !f_soft1;
        al_k1   <= f_rkey;
        al_pp1  <= rq_pp_q;
        al_sv1  <= rq_sq_q;
        if (rst) al_v <= 1'b0;
        else if (redir_hard_q) begin
          al_v    <= ((redir_k_q == 2'd0 && rx_q) || (redir_k_q == 2'd1 && rt_q)) && rx_kv_q;
          al_key  <= rx_key_q;
          al_slot <= rx_slot_q;
          al_pp   <= rx_pp_q;
          al_reg  <= 1'b1;
        end else if (al_sq) begin                            // a soft redirect last cycle: its key
          al_v    <= al_sv1;                                 // was registered then (a request may
          al_key  <= al_k1;                                  // have gone out since)
          al_slot <= al_s2q;
          al_pp   <= al_pp1;
          al_reg  <= al_lr1;                                 // (LRA: a register entry for a bclr / bcctr)
        end else if (bmis_q) begin
          al_v    <= cr_pred_q && cr_sq_q;
          al_key  <= cr_key_q;
          al_slot <= cr_slot_q;
          al_pp   <= cr_pp_q;
          al_reg  <= LRA && cr_lrk_q;
        end else if (rsp_use) al_v <= 1'b0;
      end
`ifdef VERILATOR
      always_ff @(posedge clk)
        if ($test$plusargs("btbdbg") && (f_issue || rsp || bmis_q || redirect_f))
          $display("  f: iss %b pc %h w2 %b b_go %b t_go %b pt %b rq %b k %0d | rsp %b use %b w %h %h @%h two %b bq %b ok %b soft %b | bmis %b hard %b | bt %b %h reg %b sl %b rd %h",
                   f_issue, fetch_pc, i_w2, b_go, t_go, pt_q, redir_q, redir_k_q, rsp, rsp_use, i_data, i_data2, f_rsp_pc, w2_in, bq_out, bq_ok, f_soft,
                   bmis_q, redirect_f, bt_pv, {bt_tgt, 2'b0}, bt_preg, bt_slot, {bt_rd, 2'b0});
`endif
    end else begin : g_nobtb
      assign bt_pv = 1'b0; assign bt_preg = 1'b0; assign bt_tgt = '0; assign bt_x = '0;
      assign pt_q = 1'b0; assign bq_out = 1'b0; assign bmis_q = 1'b0; assign cr_tgt_q = '0;
      assign rp_v = 1'b0; assign rp_tgt = '0;
      assign cr_lrk_q = 1'b0; assign al_lr1 = 1'b0; assign rp_lq = 1'b0;
      assign bmr_q = 1'b0; assign cr_pred_q = 1'b0; assign cr_slot_q = 1'b0; assign cr_sq_q = 1'b0; assign cr_key_q = '0;
      assign cr_pp_q = 1'b0;
      assign al_v = 1'b0; assign al_reg = 1'b0; assign al_slot = 1'b0; assign al_key = '0; assign al_pp = 1'b0;
      assign al_sq = 1'b0; assign al_s2q = 1'b0; assign al_pp1 = 1'b0; assign al_sv1 = 1'b0; assign al_k1 = '0;
      assign rx_q = 1'b0; assign rt_q = 1'b0; assign rx_key_q = '0; assign rx_slot_q = 1'b0; assign rx_kv_q = 1'b0;
      assign rx_pp_q = 1'b0;
    end
  endgenerate

  // Predecode as the word arrives, so the queue's pop needs no decoder (R23
  // timing): which of rA, rB, rS the first uop reads, and how it cracks
  // (0 single uop, 1 update form, 2 lmw / stmw). A second decoder instance;
  // all but these few outputs are trimmed. Everything else ID hands to EX
  // that does not change between a cracked instruction's uops comes from here
  // too: the operand capture's selects, the immediate's form, the result
  // select and truth table, the special registers written, the CR writer's
  // kind, the SPR number's decode and the move-from source. Each was four or
  // five LUT levels of the decoder in front of an EX register (R23 timing).
  logic [31:0] q1_hi;                                  // queue entry 1: the head after a pop
  // the predecode (ppc_pdw.sv): the arriving word's fields for the queue
  ppc_pdw u_pdw (.clk (clk), .rst (rst), .w (i_data), .tg (f_tg), .mark (f_fpend), .pd (pd_w));
  // the second word of a two-word answer (never a fault marker)
  generate
    if (FETCH2) begin : g_pdw2
      ppc_pdw u_pdw2 (.clk (clk), .rst (rst), .w (i_data2), .tg (f_tg), .mark (1'b0), .pd (pd_w2));
    end else begin : g_nopdw2
      assign pd_w2 = '0;
    end
  endgenerate

  // the pop, one copy for each half of queue entries 0 and 1 (ppc_fetchq:
  // entries 2 and 3 do not move on a pop), from a copy of EX's hold of its
  // own (q_hold, below): ex_hold drives some 700 enables across the core, and
  // the queue's pop was behind that net
  logic q_hold;
  wire  q_take = id_valid && !q_hold && !load_use && !fp_haz && id_last;
  (* keep = 1 *) wire q_pop0 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop1 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop2 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop3 = q_take && !redir_hard_q;
  wire [3:0] q_pop = {q_pop3, q_pop2, q_pop1, q_pop0};
  // the key bits go with each word (pd_t's bslot, bpp, bsq)
  pd_t pd_w1, pd_w2f;
  always_comb begin
    pd_w1 = pd_w;   pd_w1.bslot  = 1'b0; pd_w1.bpp  = FETCH2 && rq_pp_q; pd_w1.bsq  = rq_sq_q; pd_w1.lar  = la1[1] && (pw_op == 6'd16 || !la1[0] || (bq_out && !bt_slot && bt_preg));
    pd_w2f = pd_w2; pd_w2f.bslot = 1'b1; pd_w2f.bpp = FETCH2 && rq_pp_q; pd_w2f.bsq = rq_sq_q; pd_w2f.lar = la2[1] && (pw2_op == 6'd16 || !la2[0] || (bq_out && bt_slot && bt_preg));
  end
  ppc_fetchq #(.PDW (PDW), .F2 (FETCH2)) u_q (
    .clk (clk), .clear (rst || redir_hard_q), .we (rsp_use && !redir_hard_q),
    .w_insn (i_data), .w_pc (f_rsp_pc), .w_pred (w_prd), .w_pd (pd_w1),
    .we2 (rsp_use && w2_in && !pw_pred && !redir_hard_q),
    .w2_insn (i_data2), .w2_pc ({f_rsp4, 2'b00}), .w2_pred (w2_prd), .w2_pd (pd_w2f),
    .pop (q_pop), .count (q_cnt),
    .r_insn (q_hi), .r_pc (q_hp), .r_pred (q_hd), .r_pd (q_pd),
    .r1_insn (q1_hi), .r1_pd (q1_pd), .r2_pd (q2_pdv), .r3_pd (q3_pdv), .r_rf (q_rf)
  );

  // ==================================================================== decode
  logic [4:0]  id_step;
  uop_t        id_u;
  wire         id_valid = (q_cnt != 3'd0) && !redir_hard_q;
  wire         id_pred  = q_hd;
  logic        ex_pred;

  ppc_decode u_dec (
    .insn (q_hi),
    .pc   (q_hp),
    .step (id_step),
    .pr   (msr_q[MSR_PR]),                            // committed: every MSR write redirects
    .u    (id_u)
  );

  // load-use: a load in EX or MEM writes a register this uop reads (its data
  // reaches the forward only in WB). From the predecode and the raw fields,
  // not the decoder; every uop of a cracked instruction reads the registers
  // its first does, with rS counted for stmw (id_rs), so lmw and stmw run a
  // word a cycle. mfsr / mfsrin count as loads in EX only: their result is
  // formed as EX is left, and MEM forwards it.
  logic ex_ld, ex_md;                                  // EX is a load / multiply-divide (registered in ID)
  logic ex_ls;                                         // EX is a load or store (waits for d_rdy)
  logic ex_isync;
  wire ex_is_load = ex_valid && ex_ld;
  wire m_ldw      = m.valid && m.is_load && m.rd_we;
  logic [4:0] id_rs;
  // mfspr HASH1/HASH2 is formed in ID from DMISS/IMISS, DCMP/ICMP and SDR1,
  // so it waits while a write to one of them is in EX or MEM (603e-specific
  // and rare: the TLB miss exceptions write them long before)
  logic ex_hsrc;
  wire hash_wait  = q_pd.hh && ((ex_valid && ex_hsrc) || (m.valid && m.hsrc));
  // WB's load forwards only in the cycle its answer arrives: with MEM empty,
  // WB's wait does not hold EX, so a consumer would otherwise enter EX with
  // the port's word before it is the load's
  wire wb_ldw     = wb_valid && wb_live_q && wb_rd_we && (!d_valid || wb_sp1);
  // (the register compares are made a cycle early: qe_*, below)
  logic [3:0] qe_e, qe_m, qe_w;
  function automatic logic lu_q(input logic [3:0] qe, input logic ua, input logic ub, input logic us);
    lu_q = (ua && qe[0]) || (ub && qe[1]) || (us && qe[3]);
  endfunction
  logic id_s0, id_s1, id_lst_q;
  // An integer update load's step 1 (lbzu lhzu lhau lwzu and the x forms:
  // cracked, not an FP double, no FPR written) writes rA from step 0's EA,
  // EX's sum as step 0 leaves EX (x_ex, below), not from rA + d / rB read
  // again: rD may be rB (a valid form, PEM 8 lbzux: EA <- (rA) + (rB),
  // rD <- MEM(EA), rA <- EA), and step 0 has written rD by then (VF3's lbzux
  // r4,r6,r4 at 0x4D040). It reads no register, so it waits on no load, its
  // own step 0's above all. It never resumes from a replay (an add makes no
  // access), so EX always holds its step 0 as it leaves ID. From predecode
  // flops already in the queue: no register of its own.
  wire id_xuea    = id_s1 && q_pd.uea;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && id_valid && q_pd.uea != (q_pd.crack == 2'd1 && !q_pd.fdx && !q_pd.fpw && !q_pd.ifault))
      $fatal(1, "predecoded update-load kind against the queue head's fields");
`endif
  // formed for both values of d_valid and chosen by it last (timing, study R104: the D-cache's
  // d_valid through wb_ldw and the hazard terms into the queue's pop and its 60-bit head was the 100 MHz
  // system build's worst path, six LUTs; now d_valid is one LUT from load_use)
  // LU2 (load-use 2, below): the MEM load's wait is waived when its word can be taken early (lu2_m, from
  // registers) by every operand that waits on it, which is every one but an FP double's second word's
  // (id_xfea: EX's sum), and the answer this cycle, if any, is WB's (lu2_wbo), so the MEM load's own comes
  // next cycle at the earliest. lu2_m implies m_ldw.
  logic lu2_m, lu2_wbo;
  wire lu2_w0     = lu2_m && !(q_pd.fdx && id_s1);
  wire lu_m       = m_ldw && lu_q(qe_m, q_pd.ua, q_pd.ub, q_pd.us);
  wire lu_e       = (ex_is_load && ex_u.rd_we && lu_q(qe_e, q_pd.ua, q_pd.ub, q_pd.us)) || hash_wait;
  wire lu_rest    = lu_e || (lu_m && !lu2_w0);                 // d_valid = 0
  wire lu_rest1   = lu_e || (lu_m && !(lu2_w0 && lu2_wbo));    // d_valid = 1
  wire lu_wb      = wb_valid && wb_live_q && wb_rd_we && lu_q(qe_w, q_pd.ua, q_pd.ub, q_pd.us);
  (* keep = 1 *) wire load_use_d0 = !id_xuea && (lu_rest || lu_wb);
  // (R140) d1 with WB's request state last: lu_rest1's term behind lu2_wbo (wb_out) as its own part, so
  // wb_valid / wb_iss / wb_got are one LUT from load_use_d1 (ppc_cpu spike: wb_iss -> lu2_wbo -> lu_rest1 ->
  // load_use_d1 -> h_tk -> q_pop -> the queue's entry 1, -1.18 ns at 120 MHz)
  (* keep = 1 *) wire lu_d1e = !id_xuea && (lu_e || (lu_m && !lu2_w0) || (lu_wb && wb_sp1));
  (* keep = 1 *) wire lu_d1o = !id_xuea && lu_m && lu2_w0;            // waits unless WB's answer is out
  (* keep = 1 *) wire load_use_d1 = lu_d1e || (lu_d1o && !(wb_valid && wb_iss && !wb_got));
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && load_use_d1 != (!id_xuea && (lu_rest1 || (lu_wb && wb_sp1)))) $fatal(1, "load_use_d1 by parts");
`endif
  wire load_use   = d_valid ? load_use_d1 : load_use_d0;
`ifdef R104_CHECK
  // (the equivalence checks of the R104 restructuring: off in the planted-fault runs, which would
  // otherwise be caught here rather than by the benches)
  always_ff @(posedge clk)
    if (!rst && load_use != (!id_xuea && (hash_wait || (lu_q(qe_e, q_pd.ua, q_pd.ub, q_pd.us) && ex_is_load && ex_u.rd_we) ||
                                          (lu_q(qe_m, q_pd.ua, q_pd.ub, q_pd.us) && m_ldw &&
                                           !(lu2_m && !(q_pd.fdx && id_s1) && (!d_valid || lu2_wbo))) ||
                                          (lu_q(qe_w, q_pd.ua, q_pd.ub, q_pd.us) && wb_ldw))))
      $fatal(1, "load_use by d_valid");
`endif

  // the uop that finishes the instruction: on a first uop from the predecode;
  // after it from a flag registered as the step advances (an update form's
  // second uop; lmw / stmw at rD + step = 31, from rS's counted index)
  wire  id_last = id_s0 ? q_pd.l0 : id_lst_q;

  logic ex_hold, mem_hold, mem_adv;
  logic fp_haz;                                        // FP scoreboard stall (below)
  wire  id_adv = id_valid && !ex_hold && !load_use && !fp_haz && !redirect;
  // the pop does not wait on a redirect: a redirect clears the queue next cycle
  assign id_take = id_valid && !ex_hold && !load_use && !fp_haz && id_last;

  always_ff @(posedge clk) begin
    // a replay resumes its instruction at the uop that replayed (an lmw or
    // stmw does not repeat the words it has done)
    if (rst)               id_step <= 5'd0;
    else if (redirect)     id_step <= m_kp && m.rp && !sl_flt ? m.step : 5'd0;   // (m_kp: m_kill here)
    else if (id_adv)       id_step <= id_last ? 5'd0 : id_step + 5'd1;
    if (rst)               id_s0   <= 1'b1;
    else if (redirect)     id_s0   <= !(m_kp && m.rp && !sl_flt) || m.step == 5'd0;
    else if (id_adv)       id_s0   <= id_last;
    // step 1, where an FP double's second word takes step 0's EA + 4
    if (rst)               id_s1   <= 1'b0;
    else if (redirect)     id_s1   <= m_kp && m.rp && !sl_flt && m.step == 5'd1;
    else if (id_adv)       id_s1   <= id_s0 && !id_last;
  end

  // result group and logical truth table from the result kind. One-hot select:
  //   0 sum  1 slw  2 srw  3 sraw  4 rotate  5 cntlzw  6 extsb/extsh
  //   7 multiply/divide  8 move-from
  // The logical group has no select bit: its truth table is zero otherwise.
  logic [8:0] id_rsel;
  logic [3:0] id_ltt;
  wire        id_ext_h = q_pd.rk == 5'd7;
  always_comb begin
    id_rsel = 9'd0;
    id_ltt  = 4'b0000;
    case (q_pd.rk)
      5'd16:   id_ltt = 4'b1000;                       // and
      5'd17:   id_ltt = 4'b1110;                       // or
      5'd18:   id_ltt = 4'b0110;                       // xor
      5'd19:   id_ltt = 4'b0111;                       // nand
      5'd20:   id_ltt = 4'b0001;                       // nor
      5'd21:   id_ltt = 4'b1001;                       // eqv
      5'd22:   id_ltt = 4'b0100;                       // andc
      5'd23:   id_ltt = 4'b1101;                       // orc
      5'd1:    id_rsel[1] = 1'b1;
      5'd2:    id_rsel[2] = 1'b1;
      5'd3:    id_rsel[3] = 1'b1;
      5'd4:    id_rsel[4] = 1'b1;
      5'd5:    id_rsel[5] = 1'b1;
      5'd6, 5'd7: id_rsel[6] = 1'b1;
      5'd8:    id_rsel[7] = 1'b1;
      5'd9:    id_rsel[8] = 1'b1;
      5'd10:   ;                                       // none
      default: id_rsel[0] = 1'b1;
    endcase
  end
  // EX's serializing and MSR-writer kinds from the raw word and the
  // predecode, not the decoder's op (whose priority mux was in front of
  // these registers: up to eight levels from the queue head). FP arithmetic's
  // legality and FPSCR waits are the decoder's fp_arith, shallow here.
  wire [5:0] h_op = q_hi[31:26];
  wire [9:0] h_x  = q_hi[10:1];
  function automatic logic fp_ok(input logic [31:0] w);
    logic dbl;
    dbl = w[28];                                       // opcode 63 (59: single)
    case (w[5:1])
      5'd18, 5'd20, 5'd21, 5'd25, 5'd28, 5'd29, 5'd30, 5'd31: fp_ok = 1'b1;
      5'd24:   fp_ok = !dbl;
      5'd23, 5'd26: fp_ok = dbl;
      default: fp_ok = dbl && (w[10:1] == 10'd12 || w[10:1] == 10'd14 || w[10:1] == 10'd15 || w[10:1] == 10'd0 ||
                               w[10:1] == 10'd32 || w[10:1] == 10'd40 || w[10:1] == 10'd72 || w[10:1] == 10'd136 ||
                               w[10:1] == 10'd264 || w[10:1] == 10'd583 || w[10:1] == 10'd711 || w[10:1] == 10'd134 ||
                               w[10:1] == 10'd70 || w[10:1] == 10'd38 || w[10:1] == 10'd64);
    endcase
  endfunction
  wire       h_fpa   = h_op == 6'd59 || h_op == 6'd63;
  wire       h_rfi   = h_op == 6'd19 && h_x == 10'd50;
  wire       h_isync = h_op == 6'd19 && h_x == 10'd150;
  wire       h_mtmsr = h_op == 6'd31 && h_x == 10'd146;
  wire       h_ill   = q_pd.ill || (h_op == 6'd31 && h_x == 10'd470 && msr_q[MSR_PR]) || (h_fpa && !fp_ok(q_hi));
  wire       h_kmsr2 = h_op == 6'd17 || h_op == 6'd3 || (h_op == 6'd31 && h_x == 10'd4) || h_ill;
  wire       h_fser  = h_op == 6'd63 && (h_x == 10'd0 || h_x == 10'd32 || h_x == 10'd583 || h_x == 10'd711 ||
                                         h_x == 10'd134 || h_x == 10'd70 || h_x == 10'd38 || h_x == 10'd64 ||
                                         (q_hi[0] && (h_x == 10'd40 || h_x == 10'd72 || h_x == 10'd136 ||
                                                      h_x == 10'd264 || q_hi[5:1] == 5'd23)));
  wire       h_mmu   = h_op == 6'd31 && (h_x == 10'd210 || h_x == 10'd242 || h_x == 10'd306 || h_x == 10'd978 ||
                                         h_x == 10'd1010);
  wire       h_mfsr  = h_op == 6'd31 && (h_x == 10'd595 || h_x == 10'd659);
  // the uop's op without the decoder's priority mux: the predecoded first
  // uop's, FP arithmetic's and dcbi's privilege from the word here, and a
  // cracked instruction's later uops by kind (lmw / stmw and an FP double's
  // second word keep the first's op; an update's last uop is an add)
  alu_op_t   id_opf;
  always_comb
    if (h_fpa)                                                id_opf = fp_ok(q_hi) ? OP_FP : OP_ILLEGAL;
    else if (h_op == 6'd31 && h_x == 10'd470 && msr_q[MSR_PR]) id_opf = OP_ILLEGAL;
    else if (id_s0 || q_pd.crack == 2'd2 || (q_pd.fdx && id_s1)) id_opf = q_pd.pop;
    else                                                      id_opf = OP_ADD;
  // EX's redirect kind, one-hot, registered beside ex_u.op (R108): branch, sc, illegal, trap / mtmsr /
  // isync (pc + 4), rfi. The redirect's address and decision took ex_u.op through a 6-bit compare in
  // front of their selects (the 100 MHz build's ex_u.op -> redir_a_q, seven levels). Formed as id_opf
  // is, from the predecoded op through a LUT beside the overrides
  function automatic logic [4:0] rk_of(input alu_op_t op);
    rk_of = {op == OP_RFI, op == OP_TRAP || op == OP_MTMSR || op == OP_ISYNC, op == OP_ILLEGAL, op == OP_SC,
             op == OP_BRANCH};
  endfunction
  logic [4:0] id_rk, ex_rk;
  always_comb
    if (h_fpa)                                                id_rk = fp_ok(q_hi) ? 5'd0 : 5'b00100;
    else if (h_op == 6'd31 && h_x == 10'd470 && msr_q[MSR_PR]) id_rk = 5'b00100;
    else if (id_s0 || q_pd.crack == 2'd2 || (q_pd.fdx && id_s1)) id_rk = rk_of(q_pd.pop);
    else                                                      id_rk = 5'd0;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_rk != rk_of(ex_u.op)) $fatal(1, "EX's redirect kind %b against ex_u.op %0d", ex_rk, ex_u.op);
`endif
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && id_valid &&
        (({h_kmsr2, h_rfi, h_mtmsr, h_isync, h_mmu, h_fser, h_mfsr} !=
          {id_u.op == OP_SC || id_u.op == OP_ILLEGAL || id_u.op == OP_TRAP, id_u.op == OP_RFI, id_u.op == OP_MTMSR,
           id_u.op == OP_ISYNC, id_u.op == OP_MTSR || id_u.op == OP_TLBIE || id_u.op == OP_TLBL, id_u.fser,
           id_u.op == OP_MFSR}) || id_opf != id_u.op))
      $fatal(1, "raw decode of %08X (step %0d) differs from the decoder's", q_hi, id_step);
  always_ff @(posedge clk)
    if (!rst && id_valid && id_u.rd_we &&
        (id_u.op == OP_ILLEGAL || id_u.op == OP_SC || id_u.op == OP_RFI || id_u.op == OP_TRAP))
      $fatal(1, "%08X decodes with rd_we and no GPR result", q_hi);
  always_ff @(posedge clk)
    if (!rst && id_valid && id_u.fpw && id_u.rd != q_hi[25:21])
      $fatal(1, "%08X: an FPR write whose frD is not the word's [25:21]", q_hi);
`endif
  // an integer word / halfword load or store that may be split (EX's ex_msw
  // / ex_msh), from the raw word (through the decoder it led a round-6 full
  // build, h0 into ex_msw); checked against the decoder's fields in EX
  wire [9:0]  id_xo  = q_hi[10:1];
  wire        id_x31 = q_hi[31:26] == 6'd31;
  wire        id_mw  = q_hi[31:26] == 6'd32 || q_hi[31:26] == 6'd33 || q_hi[31:26] == 6'd36 || q_hi[31:26] == 6'd37 ||
                       (id_x31 && (id_xo == 10'd23 || id_xo == 10'd55 || id_xo == 10'd151 || id_xo == 10'd183 ||
                                   id_xo == 10'd534 || id_xo == 10'd662));
  wire        id_mh  = (q_hi[31:26] >= 6'd40 && q_hi[31:26] <= 6'd45) ||
                       (id_x31 && (id_xo == 10'd279 || id_xo == 10'd311 || id_xo == 10'd343 || id_xo == 10'd375 ||
                                   id_xo == 10'd407 || id_xo == 10'd439 || id_xo == 10'd790 || id_xo == 10'd918));
  logic [8:0] ex_rsel;
  logic [3:0] ex_ltt;
  logic       ex_ext_h;

  // ================================================================ registers
  logic [31:0] ld_val;                                 // WB's load data as it arrives, aligned
  logic [31:0] ld_val_m;                               // MEM's (an early answer), into wb_result

  logic [31:0] rf_ra, rf_rb, rf_rs;                   // read in ID

  // rS's index without the decoder's adder: the raw field on a first uop, and
  // for stmw's later uops rS + step, kept in a register that counts with the
  // step (the same value the decoder forms, id_u.rs)
  logic [4:0] id_rs_q;
  assign id_rs = id_s0 ? q_hi[25:21] : id_rs_q;
  always_ff @(posedge clk)
    if (redirect) begin                                // a replay resuming at its step
      id_rs_q  <= m.insn[25:21] + m.step;
      id_lst_q <= m.last;                              // the resumed uop is the one that replayed
    end else if (id_adv) begin
      id_rs_q  <= id_rs + 5'd1;
      // the next uop is the last: an update form's or a double's second;
      // lfdu's third; lmw / stmw at r31
      id_lst_q <= q_pd.crack == 2'd1 || (q_pd.crack == 2'd3 && !id_s0) || (q_pd.crack == 2'd2 && id_rs == 5'd30);
    end

  // The register compares of the forward selects and the load-use hazard,
  // made a cycle early (R40's queue-head path: the compares and the select
  // after them were in line from the queue register to the operand
  // registers). qe_e, qe_m, qe_w [f]: the rd of the uop in EX, MEM, WB equals
  // field f of the uop in ID, f = 0 rA, 1 rB, 2 rS as in the word, 3 rS as
  // counted (id_rs). Each is compared for every uop the stage may hold next
  // cycle (EX: ID's or its own; MEM: EX's or its own; WB: MEM's or its own)
  // against every head ID may hold (entry 0, entry 1 after a pop, the word
  // arriving from fetch into an empty queue), and the holds and the pop
  // choose as the last level. ID qualifies them with the stages' valid and
  // write bits, from registers. Where ID holds no uop next cycle (a redirect,
  // an empty queue) the bits are not used.
  logic [2:0][4:0] fq0, fq1, fin;                     // [rA, rB, rS] of each candidate head
  assign fq0 = {q_hi[25:21], q_hi[15:11], q_hi[20:16]};
  assign fq1 = {q1_hi[25:21], q1_hi[15:11], q1_hi[20:16]};
  assign fin = {i_data[25:21], i_data[15:11], i_data[20:16]};
  // ID takes the head (id_take) only where EX does not hold, so each compare
  // is chosen for both cases, the head ID would take with EX free (h_tk, the
  // take without the hold) and the head it keeps, and the holds pick last:
  // EX's hold is then not in front of the head's choice (R63: the queue head
  // and ex_valid into qe_e)
  wire        h_adv = id_valid && !load_use && !fp_haz;   // id_advr where EX is free
  wire        h_tk  = h_adv && id_last;                    // id_take where EX is free
  // id_s0, id_rs_q next cycle; a redirect is left out (ID then holds no uop)
  wire        hn_s0 = h_adv ? id_last : id_s0;             // (EX free; held: id_s0, id_rs_q)
  logic [3:0][4:0] qe_rd;                              // the producers: ID, EX, MEM, WB
  // ID's own rd without the decoder in front (the decoder's rd selection was
  // five LUT levels from the queue head in front of these compares: the D-36
  // full builds' h0 -> qe_e). The first uop's is rA or rD by the predecode
  // (pd.rda); a later uop's is rA (an update) or, for lmw, rD + step, which
  // is id_rs_q. Equal to id_u.rd wherever the uop writes a GPR, the only
  // case in which qe_e is used (ex_w, and the load-use hazard's rd_we)
  wire  [4:0] id_rdf = id_s0 ? (q_pd.rda ? q_hi[20:16] : q_hi[25:21])
                             : (q_pd.crack == 2'd2 ? id_rs_q : q_hi[20:16]);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && id_valid && id_u.rd_we && id_rdf != id_u.rd)
      $fatal(1, "id_rdf %0d against the decoder's rd %0d (%08X, step %0d)", id_rdf, id_u.rd, q_hi, id_step);
`endif
  assign qe_rd = {wb_rd, m.rd, ex_u.rd, id_rdf};
  // [producer][field] for next cycle's head: qe_hk where EX holds (entry 0,
  // else the arriving word), qe_hf where it is free (entry 1 after a take)
  // (every candidate compared, then chosen: the take is not in front of a
  // compare)
  logic [3:0][3:0] qe_hk, qe_hf;
  wire  [4:0] id_rs1 = id_rs + 5'd1;
  always_comb
    for (int r = 0; r < 4; r++) begin
      for (int f = 0; f < 3; f++) begin
        qe_hk[r][f] = q_cnt != 3'd0 ? qe_rd[r] == fq0[f] : qe_rd[r] == fin[f];
        qe_hf[r][f] = !h_tk ? qe_hk[r][f] : q_cnt >= 3'd2 ? qe_rd[r] == fq1[f] : qe_rd[r] == fin[f];
      end
      qe_hk[r][3] = id_s0 ? qe_hk[r][2] : qe_rd[r] == id_rs_q;
      qe_hf[r][3] = hn_s0 ? qe_hf[r][2] : h_adv ? qe_rd[r] == id_rs1 : qe_rd[r] == id_rs_q;
    end
  // qf_m, qf_w: the same, with the producer's GPR write folded in (MEM:
  // rd_we and not a load; WB: rd_we), so that a forward select is one LUT of
  // the stage's valid bit and these (registered below, after EX). (A MEM
  // hold implies EX's; a WB hold does not, with MEM empty.)
  logic [3:0] qf_m, qf_w;
  always_ff @(posedge clk) begin
    qe_e <= ex_hold  ? qe_hk[1] : qe_hf[0];
    qe_m <= mem_hold ? qe_hk[2] : ex_hold ? qe_hk[1] : qe_hf[1];
    qe_w <= ex_hold ? (wb_hold ? qe_hk[3] : qe_hk[2]) : (wb_hold ? qe_hf[3] : qe_hf[2]);
  end

  // the sixth address bit selects TGPR0-3: predecoded for the reads; for the
  // write, MSR[TGPR] as committed when the writer left MEM, which is the
  // writer's own (every change of TGPR redirects, so no uop of the other mode
  // is in flight)
  wire m_sh = msr_q[MSR_TGPR] && m.rd[4:2] == 3'd0;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && id_valid && ({q_rf[2], q_rf[12:8], q_rf[1], q_rf[7:3], q_rf[0], id_s0 ? q_rf[17:13] : id_rs_q} !=
                             {q_pd.sh[2], id_u.ra, q_pd.sh[1], id_u.rb, q_pd.sh[0], id_rs}))
      $fatal(1, "GPR read addresses from the queue's copy against the head's");
`endif
  ppc_regfile u_rf (
    .clk (clk),
    .ra  ({q_rf[2], q_rf[12:8]}), .rb ({q_rf[1], q_rf[7:3]}), .rs ({q_rf[0], id_s0 ? q_rf[17:13] : id_rs_q}),
    .qa  (rf_ra),   .qb (rf_rb),   .qs (rf_rs),
    .we  (wb_adv && wb_rd_we),                        // written as WB is left
    .wa  ({wb_rd_sh, wb_rd}),
    .wd  (wb_val)
  );

  // Forwarding into the operand registers. As a uop moves from ID to EX each
  // operand register captures its final value: the EX producer's result as it
  // leaves EX, the MEM producer's result (or its load data), the WB producer's
  // result, else the register file, which ID has already read; immediates, zero
  // and -1 are captured the same way, and the ALU's first operand (rA or rS) is
  // chosen here too. EX then starts from the registers with no mux in front of
  // its units: the producer one step ahead goes in through the last 2:1 mux at
  // the end of its own EX cycle, not through a mux at the start of the
  // consumer's (R23 timing: that mux and its fanout led every ALU path). While
  // EX is held the registers keep their values; every producer they needed had
  // already been captured.
  //
  // x: the ALU's first operand (rA, (rA|0) or rS); y: rB, the immediate, 0 or -1;
  // a: rA for rlwimi's insert; s: rS, the store and move-to source.
  logic [31:0] op_x, op_y, op_a, op_s;

  wire        ex_w   = ex_valid && ex_u.rd_we;
  wire        mem_w  = m.valid && m.rd_we && !m.is_load;   // a load's value is not here yet
  wire        wb_w   = wb_valid && wb_rd_we;
  // WB's value: a load's word as it arrives (wb_live), else the registered
  // result (a load answered earlier is in wb_result, aligned)
  // (wb_live_q: the forward selects' input straight from a register; wb_isld
  // also fans out to the FP write port)
  wire        wb_live = wb_live_q;
  assign      wb_val  = wb_live ? ld_val : wb_result;

  // The selects come from the predecode and the raw register fields, not the
  // decoder; the immediate (zero and -1 included, ppc_decode.sv) or the
  // register file is the base, MEM's or WB's producer is chosen with it in one
  // 4:1 (a 6-input LUT: two select bits formed from registers beside the
  // register file's read), and EX's producer is the last mux (R23 timing). The
  // register file is written as a uop leaves WB, where a load's data arrives;
  // nothing after MEM can cancel a uop (a fired trap writes no GPR and nothing
  // younger has issued). A producer that has left WB is in the file.
  wire        id_xs    = q_pd.xs;                      // first operand from rS
  wire [31:0] id_xrf   = id_xs ? rf_rs : rf_ra;
  wire        id_xzero = !id_xs && q_pd.az && q_pd.ra0;
  // An FP double's second word (step 1) is at step 0's EA + 4: x is EX's sum
  // as step 0 leaves EX, y the immediate 4 (ppc_decode.sv); after a replay
  // resumed at step 1, x is 0 and y the immediate fea4_q, step 0's EA + 4 kept
  // for it. Either way nothing is added in front of x's register read.
  // An integer update load's step 1 (rA = EA) takes x the same way, EX's sum
  // as step 0 leaves EX, with the immediate 0 (id_xuea, above).
  logic       ex_fd0;                                  // EX holds an FP double's step 0
  logic [31:0] fea_q, fea4_q;
  logic [31:0] fimm_q;                                 // step 1's immediate: 4, or after a replay fea4_q
  wire        id_xfea  = q_pd.fdx && id_s1;
  // (R140: one LUT of the head's predecode and id_s1, keep; through id_xfea it was two, in front of x's
  // forward select: ppc_cpu spike, h0 -> id_xfea -> id_xz -> x_k -> x_pre -> op_x, -1.36 ns at 120 MHz)
  (* keep = 1 *) wire id_xz = (!q_pd.xs && q_pd.az && q_pd.ra0) || (id_s1 && (q_pd.fdx || q_pd.uea));
`ifdef VERILATOR
  always_ff @(posedge clk) if (R1C && !rst && id_xz != (id_xzero || id_xfea || id_xuea)) $fatal(1, "id_xz flat");
`endif
  wire        id_yreg  = q_pd.yreg && !(id_s1 && (q_pd.fdx || q_pd.uea));   // !id_xfea && !id_xuea
  // y's immediate, by the predecoded form (the decoder's id_u.imm, built here
  // from the raw fields)
  logic [31:0] id_imm;
  // lmw / stmw: d + 4 x step without an adder in ID, counted as the step
  // advances like id_rs_q (the adder was in front of y's capture)
  logic [31:0] imm4_q;
  // (R140) imm4_q is read only in a later step (!id_s0). Every redirect but a replay's leaves id_s0 set, and the
  // next later step follows a step 0 leaving ID, which loads imm4_q from the word, so its value after any other
  // redirect is never read: it loads on the replay's redirect alone (im_rp, from registers and WB's hold; with
  // m_kp and WB held the only redirect is a fired trap's) and on ID's advance without the redirect. The
  // redirect (MEM's zero detect, the branch's mispredict) was three levels in front of its 32 enables (c7_s1:
  // m.result -> the zero detect -> br_mis -> redirect -> imm4_q, -1.38 ns at 110 MHz)
  wire         im_rp  = m_kp && m.rp && !sl_flt && (!wb_hold || wb_trap);
  wire         im_adv = id_valid && !ex_hold && !load_use && !fp_haz;
`ifdef VERILATOR
  logic [31:0] imm4_c;                                 // the old form, for the check
  always_ff @(posedge clk)
    if (redirect)    imm4_c <= {{16{m.insn[15]}}, m.insn[15:0]} + {25'd0, m.step, 2'b00};
    else if (id_adv) imm4_c <= (id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_c) + 32'd4;
  always_ff @(posedge clk) begin
    if (R1C && !rst && im_rp != (redirect && m_kp && m.rp && !sl_flt)) $fatal(1, "imm4_q: the replay's redirect");
    if (R1C && !rst && id_valid && !id_s0 && q_pd.immk == 3'd4 && imm4_q != imm4_c)
      $fatal(1, "imm4_q %08X read against the old form %08X", imm4_q, imm4_c);
  end
`endif
  always_ff @(posedge clk)
    if (im_rp)       imm4_q <= {{16{m.insn[15]}}, m.insn[15:0]} + {25'd0, m.step, 2'b00};   // a replay resumes here
    else if (im_adv) imm4_q <= (id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_q) + 32'd4;
  always_comb begin
    case (q_pd.immk)
      3'd1:    id_imm = {16'd0, q_hi[15:0]};
      3'd2:    id_imm = {q_hi[15:0], 16'd0};
      3'd3:    id_imm = {27'd0, q_hi[15:11]};
      3'd4:    id_imm = id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_q;   // d + 4 x step
      3'd5:    id_imm = 32'd0;
      3'd6:    id_imm = 32'hFFFF_FFFF;
      3'd7:    id_imm = {q_hi[19:16], 28'd0};
      default: id_imm = {{16{q_hi[15]}}, q_hi[15:0]};
    endcase
    if (id_xfea) id_imm = fimm_q;
    if (id_xuea) id_imm = 32'd0;                      // an update load's rA = step 0's EA + 0
  end

  // (both fields compared, then chosen: the compare does not wait on the
  // field mux)
  wire        x_ex  = (!id_xz && ex_w && (id_xs ? qe_e[2] : qe_e[0])) ||
                      (id_xfea && ex_valid && ex_fd0) || (id_xuea && ex_valid);
  wire        y_ex  = id_yreg && ex_w && qe_e[1];
  wire        a_ex  = ex_w && qe_e[0];
  wire        s_ex  = ex_w && qe_e[3];
  // (MEM is always empty when a resumed step 1 is in ID: no id_xfea here)
  wire        x_m   = !id_xzero && m.valid && (id_xs ? qf_m[2] : qf_m[0]);
  wire        y_m   = id_yreg && m.valid && qf_m[1];
  wire        a_m   = m.valid && qf_m[0];
  wire        s_m   = m.valid && qf_m[3];
  wire        x_wb  = !id_xz && wb_valid && (id_xs ? qf_w[2] : qf_w[0]);
  wire        y_wb  = id_yreg && wb_valid && qf_w[1];
  wire        a_wb  = wb_valid && qf_w[0];
  wire        s_wb  = wb_valid && qf_w[3];

  // ---------------------------------------------------------------- load-use 2 (LU2)
  // The 603e's cache-hit load is 2:1 (table 6-6): its consumer executes two cycles after the load does.
  // Here a load's word arrives in WB, so a consumer captured from WB enters EX three cycles after the
  // load. With LU2 the consumer leaves ID while the load is in MEM, the cycle of its D-cache lookup, and
  // takes the predicted way's word as the data RAMs give it (d_eword), aligned by the load's byte selects
  // (e_jq, e_dq: WB's la_ix and mg_isd, registered a stage early), through the capture's x_pre (EX's
  // forward and the last LUT are as before). Whether that word was the answer is known in the next
  // cycle, from registers: the load is then in WB, and its answer comes with d_eok (no fill word,
  // single read or store-buffer byte in it, ppc_dcache). EX then runs as if the consumer had been
  // captured from WB. Otherwise (a miss, a hit in the other way, a store's bytes in the word) EX holds
  // (lsh) until the answer, and the operands that took the early word are loaded with WB's word in the
  // cycle it arrives (rl_*, through the capture's WB input), as ID would have captured it; EX runs the
  // cycle after, as the consumer would have entered EX from ID. So a miss costs what it did, a hit a
  // cycle less. The load: an integer load whose request was made as it left EX, in one access, not
  // halfword-algebraic (the sign would be a level more), not answered yet (lu2_m). An operand takes it
  // where it uses the field (the predecode's ua, ub, us), the field is the load's rd and EX's uop does
  // not write that register (EX's forward is younger).
  logic [3:0][1:0] e_jq;                               // MEM's load: the byte of d_eword for result byte r
  logic [3:0]      e_dq;                               // ... and whether result byte r is data (else zero)
  logic            m_lsp_q;                            // MEM's uop is such a load (with m.valid)
  logic            ex_lsw, ex_lsf;                     // EX's uop took the early word: until confirmed or
                                                       // reloaded; its first cycle in EX
  logic [3:0]      ex_lso;                             // the operands that took it: x, y, a, s
  wire        x_ld  = lu2_m && !id_xz && (id_xs ? q_pd.us && qe_m[2] : q_pd.ua && qe_m[0]) && !x_ex;
  wire        y_ld  = lu2_m && id_yreg && q_pd.ub && qe_m[1] && !y_ex;
  wire        a_ld  = lu2_m && q_pd.xs && q_pd.ua && qe_m[0] && !a_ex;   // rlwimi's insert
  wire        s_ld  = lu2_m && q_pd.us && qe_m[3] && !s_ex;
  wire  [3:0] lu2_ld = {s_ld, a_ld, y_ld, x_ld};
  logic [31:0] e_al;                                   // d_eword aligned
  logic [3:0]  x_le, y_le, a_le, s_le;                 // [r]: result byte r takes e_al
  logic [3:0]  x_lz, y_lz, a_lz, s_lz;                 // ... takes zero
  // (EWL: each way's word aligned, keep, then the way)
  wire [3:0][31:0] e_alw;
  genvar ewg, ewr;
  generate if (EWL) begin : g_ewl
    for (ewg = 0; ewg < 4; ewg = ewg + 1) begin : g_ew
      for (ewr = 0; ewr < 4; ewr = ewr + 1) begin : g_r
        (* keep = 1 *) wire [7:0] a = LU2_AL ? d_ew4[ewg][31 - 8*e_jq[ewr] -: 8] : d_ew4[ewg][31 - 8*ewr -: 8];
        assign e_alw[ewg][31 - 8*ewr -: 8] = a;
      end
    end
  end else begin : g_newl
    assign e_alw = '0;
  end
  endgenerate
  always_comb
    for (int r = 0; r < 4; r++) begin
      e_al[31 - 8*r -: 8] = EWL ? e_alw[d_epw][31 - 8*r -: 8] :
                            LU2_AL ? d_eword[31 - 8*e_jq[r] -: 8] : d_eword[31 - 8*r -: 8];
      x_le[r] = x_ld && e_dq[r]; x_lz[r] = x_ld && !e_dq[r];
      y_le[r] = y_ld && e_dq[r]; y_lz[r] = y_ld && !e_dq[r];
      a_le[r] = a_ld && e_dq[r]; a_lz[r] = a_ld && !e_dq[r];
      s_le[r] = s_ld && e_dq[r]; s_lz[r] = s_ld && !e_dq[r];
    end
  // lu2_cf: confirmed (EX's first cycle, the load's answer in WB, and it is the early word); lsh: EX holds;
  // lu2_rl: the answer is here and is not the early word: reload (all three below, after ans_wb)
  logic       lu2_cf, lsh, lu2_rl;
  // (R140) each operand's reload is one LUT of the D-cache's d_valid and d_eok and registers (rl_e*), and it
  // chooses WB's word last, at the operand register's input: through ans_wb and lu2_rl it was two levels
  // in front of the forward select and the capture (ppc_cpu spike: d_valid_q -> ans_wb -> rl_x -> x_k ->
  // x_pre -> op_x, -2.02 ns at 120 MHz)
  (* keep = 1 *) wire rl_ex = LU2 && ex_lsw && wb_out && ex_lso[0];
  (* keep = 1 *) wire rl_ey = LU2 && ex_lsw && wb_out && ex_lso[1];
  (* keep = 1 *) wire rl_ea = LU2 && ex_lsw && wb_out && ex_lso[2];
  (* keep = 1 *) wire rl_es = LU2 && ex_lsw && wb_out && ex_lso[3];
  wire        rl_x = rl_ex && d_valid && !(ex_lsf && d_eok), rl_y = rl_ey && d_valid && !(ex_lsf && d_eok),
              rl_a = rl_ea && d_valid && !(ex_lsf && d_eok), rl_s = rl_es && d_valid && !(ex_lsf && d_eok);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && {rl_s, rl_a, rl_y, rl_x} != ({4{lu2_rl}} & ex_lso)) $fatal(1, "LU2 reloads by operand against lu2_rl");
`endif
`ifdef VERILATOR
`ifndef LU2_NOCHK
  // (LU2_NOCHK: off in the planted-fault runs, which the benches' comparison must catch)
  // every operand that waits on the MEM load and leaves ID with the wait waived takes the early word
  // or EX's younger result; a confirmed early word is WB's word, as ID would have captured it
  always_ff @(posedge clk)
    if (!rst && LU2 && id_adv && lu_m && lu2_w0 && (!d_valid || lu2_wbo) &&
        ((q_pd.ua && qe_m[0] && !(q_pd.xs ? (a_ld || a_ex) : (x_ld || x_ex || id_xz))) ||
         (q_pd.ub && qe_m[1] && !(y_ld || y_ex)) || (q_pd.us && qe_m[3] && !(s_ld || s_ex))))
      $fatal(1, "LU2: %08X leaves ID on the early word with an operand not covered (x %b y %b a %b s %b)", q_hi, x_ld, y_ld, a_ld, s_ld);
  always_ff @(posedge clk)
    if (!rst && LU2 && lu2_cf && ((ex_lso[0] && op_x != ld_val) || (ex_lso[1] && op_y != ld_val) ||
                                  (ex_lso[2] && op_a != ld_val) || (ex_lso[3] && op_s != ld_val)))
      $fatal(1, "LU2: confirmed early word against WB's: x %08X y %08X a %08X s %08X, WB %08X (%b)", op_x, op_y, op_a, op_s, ld_val, ex_lso);
  always_ff @(posedge clk)
    if (!rst && LU2 && ex_lsw && ex_lsf && !(wb_valid && wb_isld && wb_out))
      $fatal(1, "LU2: an early word in EX with no load waiting in WB");
`endif
`endif
`ifdef VERILATOR
  // an integer update load's step 1 leaves ID only behind its own step 0 in EX
  always_ff @(posedge clk)
    if (!rst && id_adv && id_xuea && !(ex_valid && ex_first && !ex_ifault && ex_u.op == OP_LOAD && ex_u.insn == q_hi))
      $fatal(1, "update load %08X: step 1 leaves ID without its step 0 in EX", q_hi);
  // id_xuea from the predecode against the raw word, wherever ID holds a uop
  always_ff @(posedge clk)
    if (!rst && id_valid && !q_pd.ifault &&
        id_xuea != (id_s1 && (q_hi[31:26] == 6'd33 || q_hi[31:26] == 6'd35 || q_hi[31:26] == 6'd41 || q_hi[31:26] == 6'd43 ||
                              (q_hi[31:26] == 6'd31 && (q_hi[10:1] == 10'd55 || q_hi[10:1] == 10'd119 ||
                                                        q_hi[10:1] == 10'd311 || q_hi[10:1] == 10'd375)))))
      $fatal(1, "update load kind of %08X (step 1 %b) against the raw word", q_hi, id_s1);
  // the early compares against the live ones, wherever ID holds a uop
  always_ff @(posedge clk)
    if (!rst && id_valid)
      for (int p = 0; p < 3; p++) begin
        automatic logic [4:0] rd = p == 0 ? ex_u.rd : p == 1 ? m.rd : wb_rd;
        automatic logic [3:0] qe = p == 0 ? qe_e : p == 1 ? qe_m : qe_w;
        // (EX's from id_rdf: the decoder's rd where a valid EX uop writes a GPR)
        if ((p != 0 || (ex_valid && ex_u.rd_we)) && qe != {rd == id_rs, rd == q_hi[25:21], rd == q_hi[15:11], rd == q_hi[20:16]})
          $fatal(1, "early register compare, stage %0d: %b against %b", p, qe,
                 {rd == id_rs, rd == q_hi[25:21], rd == q_hi[15:11], rd == q_hi[20:16]});
      end
  always_ff @(posedge clk)
    if (!rst && id_valid && (qf_m != (qe_m & {4{m.rd_we && !m.is_load}}) || qf_w != (qe_w & {4{wb_rd_we}})))
      $fatal(1, "early forward selects: MEM %b / %b, WB %b / %b", qf_m, qe_m, qf_w, qe_w);
`endif
  // 0 base, 1 MEM, 2 WB's arriving load word, 3 WB's result; MEM is younger
  function automatic logic [1:0] fw_k(input logic in_m, input logic in_wb, input logic live);
    fw_k = in_m ? 2'd1 : !in_wb ? 2'd0 : live ? 2'd2 : 2'd3;
  endfunction
  function automatic logic [31:0] fw_mux(input logic [1:0] k, input logic [31:0] b, input logic [31:0] vm,
                                         input logic [31:0] vl, input logic [31:0] vw);
    fw_mux = (k == 2'd0) ? b : (k == 2'd1) ? vm : (k == 2'd2) ? vl : vw;
  endfunction
  (* keep = 1 *) wire [1:0] x_k = fw_k(x_m, x_wb, wb_live);   // (LU2's reload, WB's arriving word: at op_x, R140)
  // y's select (R108): ID's term (id_yreg) one LUT from the queue head (the update-load kind
  // predecoded, pd.uea), WB's terms from registers, and MEM's valid bit and select into y_k itself:
  // y_k is two LUTs from the head and one from m.valid (it was three from the head: h0 -> id_yreg ->
  // y_k -> op_y led the 100 MHz release build; with MEM's term formed beside WB's, m.valid was three)
  (* keep = 1 *) wire       y_id = id_yreg;
  (* keep = 1 *) wire       y_wl = wb_valid && qf_w[1] && !wb_live;
  (* keep = 1 *) wire       y_w  = wb_valid && qf_w[1];
  (* keep = 1 *) wire [1:0] y_k = {y_id && !(m.valid && qf_m[1]) && y_w, y_id && ((m.valid && qf_m[1]) || y_wl)};
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && y_k != fw_k(y_m, y_wb, wb_live)) $fatal(1, "y_k by ID's term last against fw_k");
`endif
  (* keep = 1 *) wire [1:0] a_k = fw_k(a_m, a_wb, wb_live);
  (* keep = 1 *) wire [1:0] s_k = fw_k(s_m, s_wb, wb_live);
  // The capture as two LUTs side by side, the base (register file or
  // immediate) and the forward (MEM, WB's arriving word, WB), and the choice
  // between them in x_pre below: one LUT of five inputs, so a forward select
  // is three levels from MEM's valid bit, not five (R63 batch 6 full build:
  // m.valid into op_x through the 4:1 built three deep). Same values as
  // fw_mux.
  (* keep = 1 *) wire [31:0] x_b   = id_xz ? 32'd0 : id_xrf;
  (* keep = 1 *) wire [31:0] y_b   = id_yreg ? rf_rb : id_imm;
  wire [31:0] a_b   = rf_ra;
  wire [31:0] s_b   = rf_rs;
  function automatic logic [31:0] fw_fwd(input logic [1:0] k, input logic [31:0] vm, input logic [31:0] vl,
                                         input logic [31:0] vw);
    fw_fwd = !k[1] ? vm : !k[0] ? vl : vw;           // k = 1, 2, 3
  endfunction
  (* keep = 1 *) wire [31:0] x_fw = fw_fwd(x_k, m.result, ld_val, wb_result);
  (* keep = 1 *) wire [31:0] y_fw = fw_fwd(y_k, m.result, ld_val, wb_result);
  (* keep = 1 *) wire [31:0] a_fw = fw_fwd(a_k, m.result, ld_val, wb_result);
  (* keep = 1 *) wire [31:0] s_fw = fw_fwd(s_k, m.result, ld_val, wb_result);
  wire [31:0] x_cap = x_k == 2'd0 ? x_b : x_fw;
  wire [31:0] y_cap = y_k == 2'd0 ? y_b : y_fw;
  wire [31:0] a_cap = a_k == 2'd0 ? a_b : a_fw;
  wire [31:0] s_cap = s_k == 2'd0 ? s_b : s_fw;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (x_cap != fw_mux(x_k, x_b, m.result, ld_val, wb_result) || y_cap != fw_mux(y_k, y_b, m.result, ld_val, wb_result) ||
        a_cap != fw_mux(a_k, a_b, m.result, ld_val, wb_result) || s_cap != fw_mux(s_k, s_b, m.result, ld_val, wb_result))
      $fatal(1, "operand capture against fw_mux");
`endif

  wire [31:0] x_val = op_x;
  wire [31:0] y_val = op_y;
  wire [31:0] a_val = op_a;
  wire [31:0] s_val = op_s;

  // special registers as EX sees them: the MEM stage's pending write, else committed
  // CR0 from Rc is formed here, from MEM's registered result, not after EX's adder
  // CR0 from Rc and a compare's field are both put into CR here, in MEM, from
  // registered values, not after EX's adder and comparator (R23 timing)
  // A CR writer carries only the bits it writes (crm) and their values; they
  // are merged into the committed CR here, so no uop copies the whole
  // forwarded CR into MEM (R23 timing). m_cr is CR as it will be once this
  // uop commits: what EX's readers see while it is in MEM.
  //
  // CR0 from Rc (m_cr0) waits on the result's zero detect, the latest input of
  // all, so it is kept apart (m_crn is CR without it) and each reader in EX
  // takes it through a registered select of its own, as the last level before
  // its result (R23 timing).
  wire        m_z   = (m.result == 32'd0);
  wire [3:0]  m_cr0 = {m.result[31], !m.result[31] && !m_z, m_z, m.xer[31]};
  wire [3:0]  m_cr0z0 = {m.result[31], !m.result[31], 1'b0, m.xer[31]};   // m_cr0 for m_z = 0
  wire [3:0]  m_cr0z1 = {m.result[31], 1'b0, 1'b1, m.xer[31]};            // ... for m_z = 1
  logic [31:0] m_crn, m_cr;
  always_comb begin
    // a compare's field: crm already marks it, so its value goes in under
    // the same mask, with no decode of the field number here
    m_crn = (cr_q & ~m.crm) | ((m.cmp_we ? {8{m.cmpv}} : m.mcrf0 ? {8{cr_q[31:28]}} : m.cr) & m.crm);
    m_cr = m_crn;
    if (m.rc0)
      m_cr[31:28] = m_cr0;
  end
  // Special registers forward from MEM only (they commit as MEM is left). As
  // for the GPRs, ID decides from the producer's decoded write flags and hands
  // EX a registered select, so MEM's own flags are off EX's paths (R23 timing).
  // sc, illegal and fired traps are never forwarded: each redirects, so
  // nothing that could read SRR0/SRR1/MSR is in EX before they commit.
  logic fs_cr, fs_rc0, fs_xer, fs_lr, fs_ctr, fs_srr0, fs_srr1;
  logic ex_w_cr, ex_w_xer, ex_w_lr, ex_w_ctr, ex_w_spr;
  wire [31:0] cr_in_n = fs_cr  ? m_crn : cr_q;          // CR0 from Rc not included (fs_rc0)
  wire [31:0] xer_in = fs_xer ? m.xer : xer_q;
  wire [31:0] lr_in  = fs_lr  ? m.lr  : lr_q;
  wire [31:0] ctr_in = fs_ctr ? m.ctr : ctr_q;
  // the BTB's register-target check (mx.rtm): the compares made on both
  // sources of the forward, from registers, and chosen by it (the compare
  // behind the forward mux led the first BTB spike's paths)
  wire        rt_eq  = (ex_u.bkind == BR_BCLR) ? (fs_lr  ? m.lr[31:2]  == rp_tgt : lr_q[31:2]  == rp_tgt)
                                               : (fs_ctr ? m.ctr[31:2] == rp_tgt : ctr_q[31:2] == rp_tgt);
  wire        ctr1_in = fs_ctr ? m.ctr1 : ctr1_q;

  // The two CR bits a branch (BI) or a CR logic op (crbA, crbB) reads. In ID,
  // each captures its value once the uop in MEM has committed; a writer one
  // step ahead, in EX now, is given a registered select by the kind of its
  // write: CR0 from Rc, a compare's field, or bits it carries in m.cr. EX then
  // has one mux per bit, with the zero detect at its last input, where it had
  // the zero detect, a 32-bit merge and a 32:1 select in line (R23 timing).
  logic       ex_cra, ex_crb;                          // the bit, as of MEM's commit
  logic [4:0] ex_cra_s, ex_crb_s;                      // writer ahead: Rc, compare, bit, field, mtcrf
  wire  [4:0] id_cba = id_u.crba;                      // also BI
  wire  [4:0] id_cbb = id_u.crbb;
  // does the writer in EX put bit b into m.cr (CR logic, mcrf, mcrxr, mtcrf,
  // stwcx.), from its registered kind and raw fields
  logic [3:0] ex_ck;
  logic [2:0] ex_kmsr;                                 // MSR writer: sc/illegal/trap, rfi, mtmsr
  // rlw*'s mask, MB..ME, formed in ID from the raw fields and registered: in EX
  // its compare and shifts were in front of the rotators' last level (R23 timing)
  wire  [31:0] id_m_lo = 32'hFFFF_FFFF >> q_hi[10:6];          // bits mb..31
  wire  [31:0] id_m_hi = 32'hFFFF_FFFF << (5'd31 - q_hi[5:1]); // bits 0..me
  wire  [31:0] id_mask = (q_hi[10:6] <= q_hi[5:1]) ? (id_m_lo & id_m_hi) : (id_m_lo | id_m_hi);
  logic [31:0] ex_mask;
  function automatic logic ck_hit(input logic [3:0] ck, input logic [4:0] b, input logic [4:0] crbd,
                                  input logic [2:0] crf, input logic [7:0] crm);
    ck_hit = (ck[0] && crbd == b) || (ck[1] && crf == b[4:2]) ||
             (ck[2] && crm[3'd7 - b[4:2]]) || (ck[3] && b[4:2] == 3'd0);
  endfunction
  wire [31:0] m_crv  = m.valid ? m_crn : cr_q;
  wire        id_cra = (m.valid && m.rc0 && id_cba[4:2] == 3'd0) ? m_cr0[2'd3 - id_cba[1:0]] : m_crv[5'd31 - id_cba];
  wire        id_crb = (m.valid && m.rc0 && id_cbb[4:2] == 3'd0) ? m_cr0[2'd3 - id_cbb[1:0]] : m_crv[5'd31 - id_cbb];
  // (BI's value without the CR0-from-Rc case, for the branch's two decisions)
  wire        cr_an  = ex_cra_s[1] ? m.cmpv[2'd3 - ex_u.crba[1:0]] :
                       ex_cra_s[2] ? m.crlb : ex_cra_s[3] ? m.crf4[2'd3 - ex_u.crba[1:0]] :
                       ex_cra_s[4] ? m.mtca : ex_cra;
  wire        cr_a   = ex_cra_s[0] ? m_cr0[2'd3 - ex_u.crba[1:0]] : cr_an;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && ((ex_cra_s[4:2] != 3'd0 && cr_a != m.cr[5'd31 - ex_u.crba]) ||
                             (ex_crb_s[4:2] != 3'd0 && cr_b != m.cr[5'd31 - ex_u.crbb])))
      $fatal(1, "CR bit from the writer ahead: %b%b against %b%b (kinds %b %b)", cr_a, cr_b,
             m.cr[5'd31 - ex_u.crba], m.cr[5'd31 - ex_u.crbb], ex_cra_s, ex_crb_s);
`endif
  wire        cr_bn  = ex_crb_s[1] ? m.cmpv[2'd3 - ex_u.crbb[1:0]] :
                       ex_crb_s[2] ? m.crlb : ex_crb_s[3] ? m.crf4[2'd3 - ex_u.crbb[1:0]] :
                       ex_crb_s[4] ? m.mtcb : ex_crb;
  wire        cr_b   = ex_crb_s[0] ? m_cr0[2'd3 - ex_u.crbb[1:0]] : cr_bn;
  wire        res_in = !res_kill && ((m.valid && m.res_set) ? 1'b1 : (m.valid && m.res_clr) ? 1'b0 : res_q);
  // EX is an mcrf of CR0 with CR0's Rc producer in MEM (its value taken in MEM)
  wire        ex_mcrf0 = ex_valid && ex_u.op == OP_CRLOG && ex_u.insn[10:1] == 10'd0 &&
                         fs_rc0 && ex_u.crf_src == 3'd0;
  wire        id_a0  = ex_mcrf0 && id_cba[4:2] == ex_u.crf;
  wire        id_b0  = ex_mcrf0 && id_cbb[4:2] == ex_u.crf;

  // Generic SPRs in MLAB (study 8.4): SPRGs, BATs, DSISR, DAR, SDR1, HID0, the
  // 603e TLB-miss registers. SRR0 and SRR1 are flip-flops: an exception writes
  // both on one commit, and a second array write port would cost a copy.
  logic [31:0] srr0_q, srr1_q;
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] spr_mem [0:63];

  // The SPR number is decoded in ID and registered into EX with the uop, so the
  // decode is off EX's result path (R23 timing). mf_sel picks mfspr's source:
  // 0 generic/SRR, 1 XER, 2 LR, 3 CTR, 4 DEC, 5 TBL, 6 TBU, 7 PVR.
  // The SPR number is decoded at fetch (predecode) and registered into EX with
  // the uop, so the decode is off EX's result path (R23 timing).
  wire        id_spr_hit = q_pd.spr_idx != 6'd63;
  wire [5:0]  id_spr_idx = q_pd.spr_idx;
  wire [2:0]  id_mf_sel  = q_pd.mf_sel;
  logic [5:0] ex_spr_idx;

  wire [31:0] srr0_in  = fs_srr0 ? m.sprv : srr0_q;
  wire [31:0] srr1_in  = fs_srr1 ? m.sprv : srr1_q;

  // Move-from (mfspr, mftb, mfcr, mfmsr). Its source is read in ID and the
  // value captured into EX as it will stand once the uop in MEM commits, with
  // MEM's write folded in, as for the GPRs: the SPR array, SRR0/SRR1, XER, LR,
  // CTR, CR, MSR and PVR. Only a writer one step ahead is taken from MEM in EX,
  // each kind through a registered one-hot select, so EX's move-from value is
  // one AND-OR level from registers (R23 timing: the MLAB read and then an 8:1
  // select by SPR were on EX's result path). DEC and the timebase count every
  // tick, so they are still read in EX, as before.
  wire        id_k_spr = q_pd.rk == 5'd9 && !q_pd.mfcr && !q_pd.mfmsr;   // mfspr, mftb
  // the flip-flop SPRs: DSISR, DAR, the 603e table-search registers, and
  // HASH1/HASH2, formed here (2.1.2.4): HTABORG[0-6], then HTABORG[7-15] OR
  // the hash's upper nine bits under HTABMASK, then its lower ten, then zeros;
  // the primary hash is VSID[5-23] XOR the page index, the secondary its
  // complement (PEM 7.6.1.1)
  wire [31:0] h_ea   = last_i_q ? imiss_q : dmiss_q;
  wire [31:0] h_cmp  = last_i_q ? icmp_q : dcmp_q;
  wire [18:0] h_pri  = h_cmp[25:7] ^ {3'd0, h_ea[27:12]};
  wire [18:0] h_sec  = ~h_pri;
  wire [31:0] hash1  = {sdr1_q[24:18], sdr1_q[17:9] | (h_pri[18:10] & sdr1_q[8:0]), h_pri[9:0], 6'd0};
  wire [31:0] hash2  = {sdr1_q[24:18], sdr1_q[17:9] | (h_sec[18:10] & sdr1_q[8:0]), h_sec[9:0], 6'd0};
  logic [31:0] id_fspr;
  logic        id_isf;
  always_comb begin
    id_isf = 1'b1;
    case (id_spr_idx)
      6'd6:    id_fspr = dsisr_q;
      6'd7:    id_fspr = dar_q;
      6'd13:   id_fspr = dmiss_q;
      6'd14:   id_fspr = dcmp_q;
      6'd15:   id_fspr = hash1;
      6'd16:   id_fspr = hash2;
      6'd17:   id_fspr = imiss_q;
      6'd18:   id_fspr = icmp_q;
      default: begin id_fspr = 32'd0; id_isf = 1'b0; end
    endcase
  end
  wire [31:0] id_spr_v = (m.valid && m.spr_we && m.spr == id_spr_idx) ? m.sprv :
                         (id_spr_idx == 6'd0) ? srr0_q : (id_spr_idx == 6'd1) ? srr1_q :
                         id_isf ? id_fspr : spr_mem[spr_ra];
  wire [31:0] xer_nx   = (m.valid && m.xer_we) ? m.xer : xer_q;
  wire [31:0] lr_nx    = (m.valid && m.lr_we)  ? m.lr  : lr_q;
  wire [31:0] ctr_nx   = (m.valid && m.ctr_we) ? m.ctr : ctr_q;
  // id_mfv as one AND-OR of every source under a select formed from the
  // queue head's predecode alone (two levels), MEM's SPR forward choosing
  // last: as the case, the SPR index's decode and the forward compare were
  // five levels of select in front of ex_mfv (a round-2 cached full build,
  // h0 into ex_mfv, -0.57 ns). id_mfv_c is the case form, kept for the check.
  wire        mk0   = !q_pd.mfcr && !q_pd.mfmsr;
  wire        ksp   = mk0 && id_mf_sel == 3'd0 && id_spr_hit;
  wire [5:0]  ix    = id_spr_idx;
  // the array's read address through a LUT of its own (as the register
  // files'): Quartus otherwise copies the queue head into the array's
  // address register, and the pop's next state then reaches it (a round-2
  // cached full build, wk_act into spr_mem's rdaddr_reg)
  (* keep = 1 *) wire [5:0] spr_ra = id_spr_idx;
  wire        k_arr = ksp && !(ix == 6'd0 || ix == 6'd1 || ix == 6'd6 || ix == 6'd7 || (ix >= 6'd13 && ix <= 6'd18));
  wire [31:0] id_mfb = ({32{q_pd.mfcr && !q_pd.mfmsr}} & (m.valid ? m_cr : cr_q)) | ({32{q_pd.mfmsr}} & msr_q) |
                       ({32{mk0 && id_mf_sel == 3'd1}} & xer_nx) | ({32{mk0 && id_mf_sel == 3'd2}} & lr_nx) |
                       ({32{mk0 && id_mf_sel == 3'd3}} & ctr_nx) | ({32{mk0 && id_mf_sel == 3'd7}} & PVR) |
                       ({32{ksp && ix == 6'd0}} & srr0_q) | ({32{ksp && ix == 6'd1}} & srr1_q) |
                       ({32{ksp && ix == 6'd6}} & dsisr_q) | ({32{ksp && ix == 6'd7}} & dar_q) |
                       ({32{ksp && ix == 6'd13}} & dmiss_q) | ({32{ksp && ix == 6'd14}} & dcmp_q) |
                       ({32{ksp && ix == 6'd15}} & hash1) | ({32{ksp && ix == 6'd16}} & hash2) |
                       ({32{ksp && ix == 6'd17}} & imiss_q) | ({32{ksp && ix == 6'd18}} & icmp_q) |
                       ({32{k_arr}} & spr_mem[spr_ra]);
  wire        mfwd  = ksp && m.valid && m.spr_we && m.spr == ix;
  wire [31:0] id_mfv = mfwd ? m.sprv : id_mfb;
  logic [31:0] id_mfv_c;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && id_valid && id_mfv != id_mfv_c) $fatal(1, "flat move-from value %08X against %08X", id_mfv, id_mfv_c);
`endif
  always_comb begin
    case (id_mf_sel)
      3'd1:    id_mfv_c = xer_nx;
      3'd2:    id_mfv_c = lr_nx;
      3'd3:    id_mfv_c = ctr_nx;
      3'd7:    id_mfv_c = PVR;
      3'd0:    id_mfv_c = id_spr_hit ? id_spr_v : 32'd0;
      default: id_mfv_c = 32'd0;                         // DEC, TBL, TBU: read in EX
    endcase
    if (q_pd.mfcr)  id_mfv_c = m.valid ? m_cr : cr_q;
    if (q_pd.mfmsr) id_mfv_c = msr_q;
  end
  // EX's selects: mf_b the captured value; mf_f a writer one step ahead of
  // XER, LR, CTR, an SPR, CR; mf_c0 its CR0 from Rc; mf_t DEC, TBL, TBU
  logic [31:0] ex_mfv;
  logic        mf_b, mf_c0;
  logic [4:0]  mf_f;
  logic [2:0]  mf_t;
  wire  [4:0]  id_mf_f = {q_pd.mfcr && ex_valid && ex_w_cr,
                          id_k_spr && id_mf_sel == 3'd0 && ex_valid && ex_w_spr && ex_spr_idx == id_spr_idx,
                          id_k_spr && id_mf_sel == 3'd3 && ex_valid && ex_w_ctr,
                          id_k_spr && id_mf_sel == 3'd2 && ex_valid && ex_w_lr,
                          id_k_spr && id_mf_sel == 3'd1 && ex_valid && ex_w_xer};
  wire  [2:0]  id_mf_t = {id_k_spr && id_mf_sel == 3'd6, id_k_spr && id_mf_sel == 3'd5, id_k_spr && id_mf_sel == 3'd4};
  // (mf_c0's CR0 from Rc, behind MEM's zero detect, joins EX's result in its
  // late part, r_late, not here)
  wire  [31:0] mf_c0v  = {{4{mf_c0}} & m_cr0, 28'd0};
  // mfcr after a CR writer in MEM: mf_f[4] & m_crn with the select folded
  // into the mask a cycle early (mf4c = mf_f[4] & m.crm, mf4o = mf_f[4] &
  // ~m.crm, registered as EX is loaded), so the writer's bits are one LUT from
  // MEM's registers: through m_crn's merge and then the select, m.cmp_we
  // into EX's result led a round-1 uncached full build (into op_a)
  logic [31:0] mf4c, mf4o;
  wire  [31:0] m_crw   = m.cmp_we ? {8{m.cmpv}} : m.mcrf0 ? {8{cr_q[31:28]}} : m.cr;
  wire  [31:0] mf_nowe = ({32{mf_b}} & ex_mfv) | ({32{mf_f[0]}} & m.xer) | ({32{mf_f[1]}} & m.lr) |
                         ({32{mf_f[2]}} & m.ctr) | ({32{mf_f[3]}} & m.sprv) | (mf4o & cr_q) | (mf4c & m_crw);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && (mf4c != ({32{mf_f[4]}} & m.crm) || mf4o != ({32{mf_f[4]}} & ~m.crm)))
      $fatal(1, "mfcr's registered CR masks against mf_f[4] and m.crm");
`endif
  // (R140) the move-from value as four groups of at most six inputs (keep), each one LUT from registers, so
  // EX's early result is two levels from them: as mf_nowe, then mf_val, then r_early, MEM's LR into op_y was
  // five (ppc_cpu spike: mf_f -> mf_nowe -> r_early -> y_pre -> op_y, -1.53 ns at 120 MHz)
  (* keep = 1 *) wire [31:0] mfg_a = ({32{mf_b}} & ex_mfv) | ({32{mf_f[0]}} & m.xer) | ({32{mf_f[1]}} & m.lr);
  (* keep = 1 *) wire [31:0] mfg_b = ({32{mf_f[2]}} & m.ctr) | ({32{mf_f[3]}} & m.sprv) | (mf4o & cr_q);
  (* keep = 1 *) wire [31:0] mfg_c = mf4c & m_crw;
  (* keep = 1 *) wire [31:0] mfg_d = ({32{mf_t[0]}} & dec_q) | ({32{mf_t[1]}} & tb_q[31:0]) | ({32{mf_t[2]}} & tb_q[63:32]);
  wire  [31:0] mf_now  = mfg_a | mfg_b | mfg_c | mf_c0v;
  wire  [31:0] mf_val  = mfg_a | mfg_b | mfg_c | mfg_d;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && (mf_now != (mf_nowe | mf_c0v) || mf_val != (mf_nowe | ({32{mf_t[0]}} & dec_q) | ({32{mf_t[1]}} & tb_q[31:0]) |
                                                              ({32{mf_t[2]}} & tb_q[63:32]))))
      $fatal(1, "the move-from value by groups");
`endif

  // ALU

  logic [31:0] alu_res, r_sum, r_rl, r_rr, r_clz, r_mid;
  logic        alu_ca, alu_ov;
  logic [3:0]  alu_cmp;

  ppc_alu u_alu (
    .op (ex_u.op), .x (x_val), .y (y_val), .a_ins (a_val),
    .cin (ex_u.cin), .ca_in (xer_in[29]), .so_in (xer_in[31]),
    .mask (ex_mask), .rlw_insert (ex_u.rlw_insert),
    .ltt (ex_ltt), .ext_h (ex_ext_h), .sel (ex_rsel[6:0]),
    .r_sum (r_sum), .r_rl (r_rl), .r_rr (r_rr), .r_clz (r_clz), .r_mid (r_mid),
    .ca_out (alu_ca), .ov_out (alu_ov), .cmp_out (alu_cmp)
  );

  // multiply / divide
  wire ex_is_md = ex_valid && ex_md;
  logic        md_done, md_ov;
  logic [31:0] md_res;
  logic        md_have;                                // result captured, waiting on MEM
  logic [31:0] md_res_q;
  logic        md_ov_q;

  ppc_muldiv u_md (
    .clk (clk), .rst (rst),        // a redirect never meets a divide in EX
    .start (md_start), .en (ex_md),
    .op (ex_u.op), .x (x_val), .y (y_val),
    .done (md_done), .result (md_res), .ov (md_ov)
  );

  // A multiply/divide killed from MEM (m_kill) while it runs is let finish
  // and its result dropped (md_drop), so the kill needs nothing in EX's hold.
  logic       md_drop;
  wire        md_ready = (md_done && !md_drop) || md_have;
  wire        md_start = ex_is_md && !ex_md_busy && !md_have && !md_drop && !lsh;   // (LU2: not on an early word)
  wire [31:0] md_val   = md_res | ({32{md_have}} & md_res_q);   // md_res is zero unless done
  wire        md_ovv   = md_done ? md_ov  : md_ov_q;

  // branch
  wire        bo_noctr  = ex_u.bo[2];
  wire        bo_ctrz   = ex_u.bo[1];
  wire        bo_nocond = ex_u.bo[4];
  wire        bo_cond   = ex_u.bo[3];
  wire [31:0] ctr_dec   = ctr_in - 32'd1;
  wire        br_ctr_ok = bo_noctr || (!ctr1_in ^ bo_ctrz);         // CTR-1 != 0, from a flag
  // The decision for both values of MEM's zero detect (m_z, CR0[EQ] and
  // CR0[GT] of an Rc producer one step ahead), chosen by it last: the
  // detect was in front of BI's select, the decision and the redirect
  // (R63: MEM's result into redir_q and its fanout)
  function automatic logic br_tk(input logic cb, input br_kind_t k, input logic ctr_ok, input logic nocond,
                                 input logic cond);
    case (k)
      BR_B:              br_tk = 1'b1;
      BR_BC, BR_BCLR:    br_tk = ctr_ok && (nocond || cb == cond);
      default:           br_tk = nocond || cb == cond;
    endcase
  endfunction
  wire        cra_z0    = ex_cra_s[0] ? m_cr0z0[2'd3 - ex_u.crba[1:0]] : cr_an;
  wire        cra_z1    = ex_cra_s[0] ? m_cr0z1[2'd3 - ex_u.crba[1:0]] : cr_an;
  wire        brt_z0    = br_tk(cra_z0, ex_u.bkind, br_ctr_ok, bo_nocond, bo_cond);
  wire        brt_z1    = br_tk(cra_z1, ex_u.bkind, br_ctr_ok, bo_nocond, bo_cond);
  wire        br_taken  = m_z ? brt_z1 : brt_z0;
  // b's and bc's displacement, from the raw word as it enters EX (ex_bd):
  // the decoder's imm, used in EX for nothing else, was through its case
  // structure from the queue head into ex_u.imm (a D-36 cached full build)
  logic [31:0] ex_bd;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && ex_u.op == OP_BRANCH && (ex_u.bkind == BR_B || ex_u.bkind == BR_BC) && ex_bd != ex_u.imm)
      $fatal(1, "branch displacement %08X against the decoder's %08X", ex_bd, ex_u.imm);
`endif
`ifdef VERILATOR
  // ex_fsfld from the raw word against the decoder's form, for FPSCR ops
  logic [2:0] ex_fsfld_d;
  always_ff @(posedge clk) begin
    if (!ex_hold) ex_fsfld_d <= id_u.fsop == 3'(FS_MCRFS) ? id_u.insn[20:18] : id_u.insn[25:23];
    if (!rst && ex_valid && ex_fsop != 3'd0 && ex_fsfld != ex_fsfld_d)
      $fatal(1, "FPSCR op field %0d against the decoder's %0d", ex_fsfld, ex_fsfld_d);
  end
`endif
  logic [31:0] br_target;
  always_comb begin
    case (ex_u.bkind)
      BR_B, BR_BC: br_target = (ex_u.aa ? 32'd0 : ex_u.pc) + ex_bd;
      BR_BCLR:     br_target = {lr_in[31:2], 2'b00};
      default:     br_target = {ctr_in[31:2], 2'b00};
    endcase
  end
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && ex_u.op == OP_BRANCH) begin
      automatic logic cnd = bo_nocond || (cr_a == bo_cond);
      if (br_taken != (ex_u.bkind == BR_B ? 1'b1 : ex_u.bkind == BR_BC || ex_u.bkind == BR_BCLR ? br_ctr_ok && cnd : cnd))
        $fatal(1, "branch decision against BI");
    end
`endif

  // memory access
  assign alu_res = r_sum;
  wire [31:0] ea      = r_sum;
  wire        is_ld   = ex_u.op == OP_LOAD;
  wire        is_st   = ex_u.op == OP_STORE;
  wire        st_ok   = !ex_u.cond || res_in;           // stwcx. stores only when reserved
  // the byte offset in the word, any size: from the operands' own two bits,
  // not the ALU's sum (its input logic for subtract was in front of the
  // split's hold and EX's redirect in a round-5 cached full build)
  wire [1:0]  boff    = op_x[1:0] + op_y[1:0];
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && (is_ld || is_st) && boff != ea[1:0]) $fatal(1, "boff against the EA");
`endif
`ifdef R104_CHECK
  // (page crossings, R124) ea_x is the EA but in a split's second cycle once its first word has gone; that
  // second word, made, is in the first's page or the next; one whose first word replayed makes no request;
  // MEM's translation page is the EA's but for that second word
  always_ff @(posedge clk) begin
    if (!rst && ex_valid && (is_ld || is_st) && ea_x != ea + {29'd0, ex_s0ok, 2'b00}) $fatal(1, "ea_x against the EA");
    if (!rst && ex_s0ok && !ex_sp1) $fatal(1, "ex_s0ok outside a split's second cycle");
    if (!rst && ex_valid && ex_sp1 && !ex_s0ok && d_req) $fatal(1, "a split's second word requested without its first");
    if (!rst && m.valid && (m.is_load || m.req || m.rp) && !m_x4 && m_xpg != m.result[31:12])
      $fatal(1, "MEM's translation page is not the EA's");
    if (!rst && m.valid && m.rp && m_x4 && m_xpg != m.result[31:12] + 20'd1)
      $fatal(1, "a split's second word replayed in its first word's page (EA %08X)", m.result);
  end
`endif

  // Misaligned accesses (603e 2.2.3, 4.5.6). An integer halfword or word that
  // crosses a word boundary (mis_x) is split: in its first cycle in EX (mis_hold,
  // EX holding) the word at EA goes out on the lanes from EA on, as a uop of
  // its own into MEM with no effect but the access (m.sp0); in the next
  // (ex_sp1) the uop itself goes out at EA + 4 (ea_x's carry-in, once the
  // first word has gone: ex_s0ok) on the rest.
  // A load's first word is kept as it leaves WB (mis_w0) and merged with the
  // second, which WB takes a cycle for (wb_sp1). The alignment exception
  // instead (algn, taken from MEM as dcbz's, m_algc), before any access:
  // lmw / stmw (at step 0), lwarx, stwcx. and FP accesses not word-aligned;
  // with MSR[DR] = 1 an FP double crossing a page with no DBAT hit for its EA
  // (4.5.6.1.1: double-word operands "cause an alignment exception if the EA
  // ends in 0xFF9-FFF"). A halfword or word crossing a page is split as any
  // other, with DR = 1 too (4.5.3: "The 603e supports the crossing of page
  // boundaries"; study R124, xpage): its second word, at EA + 4, is translated
  // on its own page by the DBATs and the micro-TLB, and on a micro-TLB miss
  // replays from MEM with that page (m_xpg), taking its own DTLB miss or DSI
  // with DMISS / DAR its page's first byte ("a byte address in the first word
  // of the offending page", 4.5.3, table 4-11). The first word has then been
  // made: a store's bytes before the boundary are in memory (PEM 6.1.4), a
  // load writes nothing (the instruction is run again by the handler's rfi,
  // or the replay's refetch). String and LE accesses do not exist here.
  logic        ex_sp1, ex_s0ok, ex_lsm;                // the split's second cycle; its first word went; lmw / stmw
  // (ex_msw / ex_msh: an integer word / halfword access that may split,
  // registered from ID, so the split's hold is one LUT of registers and the
  // sum's low bits: from the op decode it led a round-4 full build, ex_u.op
  // through EX's hold into the fetch queue)
  logic        ex_msw, ex_msh;
  wire        ls_mem  = (is_ld || is_st) && ex_u.cop == 3'd0;
  wire        mis_x   = ex_msw ? boff != 2'd0 : ex_msh && boff == 2'd3;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && !ex_ifault && {ex_msw, ex_msh} != ({2{ls_mem && !ex_u.fp && !ex_u.reserve && !ex_u.cond && !ex_lsm}} &
                                                 {ex_u.msize == 2'd2, ex_u.msize == 2'd1}))
      $fatal(1, "the split kind's registered bits: %b%b insn %08X op %0d cop %0d msize %0d step %0d first %b ifault %b", ex_msw, ex_msh, ex_u.insn, ex_u.op, ex_u.cop, ex_u.msize, ex_step, ex_first, ex_ifault);
`endif
  // (an exception in the uop's place, ex_xcm, is not held for; a replay or
  // the alignment exception, known late, holds the cycle and then leaves as
  // without a split, with no access: ex_s0ok says the first word went)
  // (R140: ex_xcm's terms in two parts, so ex_s0ok and the interrupt are one level from mis_hold, and
  // mis_hold one from each copy of EX's hold: ppc_cpu spike, ex_s0ok -> ex_xcm -> mis_hold -> fh_e ->
  // f_hold -> fa_q's enable, -1.63 ns at 120 MHz)
  (* keep = 1 *) wire xcm_ni = ex_ifault || xc_fpx || ex_fpun;     // ex_xcm without the interrupt
  (* keep = 1 *) wire xci0   = irq_req && ex_first;                // xc_irq but for ex_s0ok
  wire        mis_hold = ex_valid && mis_x && !ex_sp1 && !xcm_ni && !(xci0 && !ex_s0ok);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && mis_hold != (ex_valid && mis_x && !ex_sp1 && !ex_xcm)) $fatal(1, "mis_hold by parts");
`endif
  wire        algn    = ls_mem && (((ex_u.fp || ex_u.reserve || ex_u.cond || (ex_lsm && ex_first)) && boff != 2'd0) ||
                                   (msr_q[MSR_DR] && !db_any && ex_u.fp && ex_fd0 && ea_lo[11:0] == 12'hFFC));
  // a store's lanes and data: the value left-justified (vl), shifted to EA's
  // lane, or for a split's second word the rest from lane 0
  logic [31:0] st_data, sv, vl;
  logic [3:0]  st_be, smask;
  always_comb begin
    sv = ex_fst ? fsv : s_val;
    vl    = (ex_u.msize == 2'd2) ? (ex_u.brev ? {sv[7:0], sv[15:8], sv[23:16], sv[31:24]} : sv)
                                 : {ex_u.brev ? {sv[7:0], sv[15:8]} : sv[15:0], 16'd0};
    smask = (ex_u.msize == 2'd2) ? 4'b1111 : 4'b1100;
    // (one rotation serves both words of a split: each takes its own lanes)
    case (ex_u.msize)
      2'd2, 2'd1: begin
        st_data = (boff == 2'd0) ? vl : (boff == 2'd1) ? {vl[7:0], vl[31:8]} :
                  (boff == 2'd2) ? {vl[15:0], vl[31:16]} : {vl[23:0], vl[31:24]};
        st_be   = ex_sp1 ? 4'(8'(smask) << (3'd4 - {1'b0, boff})) : smask >> boff;
      end
      default: begin
        st_data = {4{sv[7:0]}};
        st_be   = 4'b1000 >> boff;
      end
    endcase
  end

  // ------------------------------------------------------- data translation
  // In EX, on an address adder of its own (x + y, no carry-in, no gating:
  // a load's or store's EA is the sum's value without the ALU's input logic),
  // two translations that need no array read: the four DBATs (5.3, PEM 7.4)
  // and a four-entry micro-TLB of recent page translations, each a page's
  // EA[0-19], its RPN and whether a store may use it (PP, key and C all
  // allowing it). Either gives the physical address in EX, so a hit costs no
  // cycle. Any other access (a page access the micro-TLB cannot serve, or a
  // BAT protection fault) goes to MEM without a request (m.rp), and MEM
  // translates it from the registered EA (BATs, segment register, both TLB
  // ways, protection, C): a fault takes its exception there; a translation
  // that is allowed fills the micro-TLB and the instruction is fetched again,
  // resuming at the uop that replayed (a replay, about eight cycles), to hit
  // it. A write to the MMU's state empties the entries it can change (UTS:
  // that TLB set's, or that segment's; a BAT write all of them), and the
  // entries carry the MSR[PR] they were made under, so it never holds what
  // the TLB, the segment registers or the key would no longer give.
  logic        ex_mfsr;                                // mfsr(in) in EX
  // (+ 4 for a split's second word once its first has gone, ex_s0ok: its word
  // index and its page, which the DBAT and micro-TLB compares below translate;
  // a second cycle whose first word replayed, ex_sp1 && !ex_s0ok, is EA's and
  // replays again, so MEM translates the first word's page)
  wire  [31:0] ea_x    = op_x + op_y + {29'd0, ex_s0ok, 2'b00};   // = ea for loads and stores
  wire  [31:0] fsr     = fsr_mem[op_y[31:28]];         // mfsr(in)'s copy of the segment registers
  logic [3:0]  db_hit, db_go;
  logic [14:0] db_pa;
  logic [3:0]  db_wimg;
  // The DBAT compare without most of the adder in front (R63: the address
  // add, this compare and the address mux led two full builds). EA[i] =
  // BEPI[i] for every unmasked bit i holds exactly when, in each run of
  // unmasked bits, the run's lowest bit matches as summed (ea_x) and every
  // bit above it in the run passes the carry-free test of sum_is (below),
  // which is exact once the bits below it in the run match. Bit 17 + j is
  // BEPI bit j (BL masks bits 17 to 27; 28 to 31 are always compared).
  function automatic logic dbat_hit(input logic [31:17] x, input logic [31:17] y, input logic [31:17] s,
                                    input logic [14:0] bepi, input logic [10:0] bl, input logic valid);
    logic [14:0] um, rs, ok;                          // unmasked; a run's lowest; the bit's test
    logic [14:0] gx;
    um = {4'b1111, ~bl};
    for (int j = 0; j < 15; j++) begin
      rs[j] = um[j] && (j == 0 || !um[(j + 14) % 15]);
      gx[j] = (x[17 + j] & y[17 + j]) | ((x[17 + j] ^ y[17 + j]) & ~bepi[j]);
    end
    for (int j = 0; j < 15; j++)
      if (!um[j])     ok[j] = 1'b1;
      else if (rs[j]) ok[j] = s[17 + j] == bepi[j];
      else            ok[j] = (x[17 + j] ^ y[17 + j] ^ bepi[j]) == gx[(j + 14) % 15];
    dbat_hit = valid && &ok;
  endfunction
  // dbat_hit was laid out by hand (keep), as the micro-TLB's compare below: each
  // bit's carry-free test as one LUT of x, y and BEPI at the bit and the one
  // below (db_cf), then the bit's outcome by its static kind (masked, a run's
  // lowest against the sum, else the test: db_ok), then the AND. Left to
  // Quartus it took five levels from op_y, and a round-3 cached full build
  // ran op_y through it and xl_rp into the data cache's posted answer (d_valid).
  // The compare split in two (R108), so the hit is one level shallower than with each bit's outcome
  // (db_ok) in front of the AND: a bit passes exactly when (cf | ML) & (!L | sum == BEPI), ML: the bit
  // is masked or a run's lowest, L: it is a run's lowest. A_k ANDs the first factor (the carry-free
  // tests, three bits a LUT, then the groups with the valid bit), B_k the second (two of the run's
  // lowest bits a LUT, from the sum). ML and L are LUTs of BL alone.
  (* keep = 1 *) wire [59:0] db_cf;                // [15k + j]: DBAT k, BEPI bit j (EA bit 17 + j); j = 0 unused
  (* keep = 1 *) wire [59:0] db_ml;                // masked or a run's lowest
  (* keep = 1 *) wire [47:0] db_lo;                // [12k + j], j < 12: a run's lowest (none above bit 28)
  (* keep = 1 *) wire [19:0] db_ag;                // [5k + g]: A's group g, bits j = 3g + 1 .. 3g + 3
  (* keep = 1 *) wire [3:0]  db_vl;                // the valid bit under MSR[PR]
  (* keep = 1 *) wire [3:0]  db_a;
  (* keep = 1 *) wire [23:0] db_bp;                // [6k + q]: B's pair q, bits j = 2q, 2q + 1
  (* keep = 1 *) wire [3:0]  db_b;
  genvar dbk, dbj, dbq;
  generate
  for (dbk = 0; dbk < 4; dbk = dbk + 1) begin : g_db
    wire [14:0] blx = {4'b0000, bat_bl[4 + dbk]};    // masked (BL bits 0-10 mask EA bits 17-27)
    for (dbj = 0; dbj < 15; dbj = dbj + 1) begin : g_b
      if (dbj == 0) begin : g_0
        assign db_cf[15*dbk + dbj] = 1'b0;
        assign db_ml[15*dbk + dbj] = 1'b1;             // (bit 17 is masked or a run's lowest)
        assign db_lo[12*dbk + dbj] = !blx[0];
      end else begin : g_j
        assign db_cf[15*dbk + dbj] = (op_x[17+dbj] ^ op_y[17+dbj] ^ bat_bepi[4 + dbk][dbj]) ==
                                     ((op_x[16+dbj] & op_y[16+dbj]) | ((op_x[16+dbj] ^ op_y[16+dbj]) & ~bat_bepi[4 + dbk][dbj-1]));
        assign db_ml[15*dbk + dbj] = blx[dbj] || blx[dbj-1];
        if (dbj < 12) begin : g_l
          assign db_lo[12*dbk + dbj] = !blx[dbj] && blx[dbj-1];
        end
      end
    end
    for (dbj = 0; dbj < 5; dbj = dbj + 1) begin : g_g
      localparam int J0 = 3 * dbj + 1;
      localparam int JN = (dbj == 4) ? 2 : 3;          // (bits 1-14)
      wire [2:0] t;
      for (dbq = 0; dbq < 3; dbq = dbq + 1) begin : g_t
        if (dbq < JN) begin : g_u
          assign t[dbq] = db_cf[15*dbk + J0 + dbq] || db_ml[15*dbk + J0 + dbq];
        end else begin : g_n
          assign t[dbq] = 1'b1;
        end
      end
      assign db_ag[5*dbk + dbj] = &t;
    end
    assign db_vl[dbk] = msr_q[MSR_PR] ? bat_vp[4 + dbk] : bat_vs[4 + dbk];
    assign db_a[dbk]  = db_vl[dbk] && &db_ag[5*dbk +: 5];
    for (dbj = 0; dbj < 6; dbj = dbj + 1) begin : g_p
      assign db_bp[6*dbk + dbj] =
        (!db_lo[12*dbk + 2*dbj]     || ea_x[17 + 2*dbj]     == bat_bepi[4 + dbk][2*dbj]) &&
        (!db_lo[12*dbk + 2*dbj + 1] || ea_x[17 + 2*dbj + 1] == bat_bepi[4 + dbk][2*dbj + 1]);
    end
    assign db_b[dbk] = &db_bp[6*dbk +: 6];
  end
  endgenerate
  always_comb begin
    db_pa = 15'd0; db_wimg = 4'd0;
    for (int k = 0; k < 4; k++) begin
      db_hit[k] = db_a[k] && db_b[k];
      // BAT protection (PEM 7.4.4): PP = 00 none, 10 read/write, x1 read only
      db_go[k]  = db_hit[k] && (is_st ? bat_pp[4 + k] == 2'd2 : bat_pp[4 + k] != 2'd0);
      db_pa   = db_pa   | ({15{db_hit[k]}} & (bat_brpn[4 + k] | {4'd0, ea_x[27:17] & bat_bl[4 + k]}));
      db_wimg = db_wimg | ({4{db_hit[k]}} & bat_wimg[4 + k]);
    end
  end
  wire         db_any  = |db_hit;
  // D-39 as amended (R124, R129): an FP double crossing a page whose EA hits a DBAT is two word uops, each
  // translated on its own (ea_x), whether or not its second word leaves the block: 4.5.6.1.1's exception is
  // for a page-address translation access ("there is not a match in the BAT"), and "there is no special
  // handling for accesses that fall into BAT regions". With no BAT hit it takes the alignment exception
  // (algn). Integer crossings are split and translated word by word, so need neither. (Until R129 a double
  // leaving its block trapped, which took a block-end compare per DBAT: db_lt, db_lv, db_leave.)
`ifdef VERILATOR
  always_ff @(posedge clk)
    for (int k = 0; k < 4; k++)
      if (!rst && db_hit[k] != bat_match(ea_x[31:17], bat_bepi[4 + k], bat_bl[4 + k],
                                         msr_q[MSR_PR] ? bat_vp[4 + k] : bat_vs[4 + k]))
        $fatal(1, "DBAT %0d compare against bat_match", k);
`endif
  // micro-TLB: entries filled by MEM's page translation (below), each tagged
  // with the MSR[PR] it was made under (the key depends on it). The tag is
  // compared against x + y without waiting for the adder: x + y = K holds
  // exactly when every bit i has x ^ y ^ K = the carry into i that the sum K
  // would need, (x & y) | ((x ^ y) & ~K) of bit i-1, which needs only bits i
  // and i-1; the carry into bit 12 is the low part's own.
  localparam int DUW = $clog2(DUN);
  logic [DUN-1:0]        du_v, du_wr, du_pr;
  logic [DUN-1:0][19:0]  du_pg, du_pa;
  logic [DUN-1:0][3:0]   du_wimg;
  logic [DUW-1:0]        du_nx;                        // next entry to fill
  logic [DUN-1:0]        du_use;                       // hit, and this access may use it
  logic [19:0]       du_pahi;
  logic [3:0]        du_wimg_s;
  // (the split's + 4 as ea_x's: its carry into bit 12 is the second word's
  // page, which differs from the first's only for a word crossing the page)
  wire  [12:0]       ea_lo = {1'b0, op_x[11:0]} + {1'b0, op_y[11:0]} + {10'd0, ex_s0ok, 2'b00};
  function automatic logic sum_is(input logic [19:0] x, input logic [19:0] y, input logic [19:0] k,
                                  input logic cin);
    logic [19:0] c;
    c = {(x[18:0] & y[18:0]) | ((x[18:0] ^ y[18:0]) & ~k[18:0]), cin};
    sum_is = (x ^ y ^ k) == c;
  endfunction
  // The address and WIMG are taken from an entry's page match alone
  // (du_hit), without the load / store permission, which only decides
  // whether there is a request (du_use): the op decode was in front of the
  // address (R63 batch-8 full build, ex_u.op into the RAM's address). An
  // entry refused for a store is never beside another for the same page:
  // the refusal is a replay that faults (DSI, DTLB store miss), and the
  // handler's tlbld or tlbie empties the micro-TLB before a refill.
  // The compare is laid out by hand, each level a LUT (keep): a bit's test
  // (x ^ y ^ K against the carry the sum K needs, six inputs), five bits ANDed
  // (the entry's live bit, valid under this MSR[PR], in the first group), the four groups
  // ANDed into the hit. Left to synthesis the bit's test came out as two
  // levels and the hit as five, and the micro-TLB hit into the data cache's
  // address register led all three D-36 cached full builds (op_y -> r_a).
  logic [DUN-1:0]  du_hit;
  (* keep = 1 *) wire [20*DUN-1:0] du_mb;            // [20k + i]: entry k, page bit i
  (* keep = 1 *) wire [4*DUN-1:0]  du_g;             // [4k + j]: entry k, bits 5j .. 5j + 4
  (* keep = 1 *) wire [DUN-1:0]    du_lv;            // entry k is valid under this MSR[PR]
  genvar duk, dui;
  generate
  for (duk = 0; duk < DUN; duk = duk + 1) begin : g_du
    for (dui = 0; dui < 20; dui = dui + 1) begin : g_b
      if (dui == 0) begin : g_0
        assign du_mb[20*duk + dui] = (op_x[12] ^ op_y[12] ^ du_pg[duk][0]) == ea_lo[12];
      end else begin : g_i
        assign du_mb[20*duk + dui] = (op_x[12+dui] ^ op_y[12+dui] ^ du_pg[duk][dui]) ==
                                 ((op_x[11+dui] & op_y[11+dui]) | ((op_x[11+dui] ^ op_y[11+dui]) & ~du_pg[duk][dui-1]));
      end
    end
    assign du_lv[duk]      = du_v[duk] && du_pr[duk] == msr_q[MSR_PR];   // registers only
    assign du_g[4*duk]     = &du_mb[20*duk +: 5] && du_lv[duk];
    assign du_g[4*duk + 1] = &du_mb[20*duk + 5 +: 5];
    assign du_g[4*duk + 2] = &du_mb[20*duk + 10 +: 5];
    assign du_g[4*duk + 3] = &du_mb[20*duk + 15 +: 5];
  end
  endgenerate
  always_comb begin
    du_pahi = 20'd0; du_wimg_s = 4'd0;
    for (int k = 0; k < DUN; k++) begin
      du_hit[k] = &du_g[4*k +: 4];
      du_use[k] = du_hit[k] && (is_ld || du_wr[k]);
      du_pahi   = du_pahi | ({20{du_hit[k]}} & du_pa[k]);
      du_wimg_s = du_wimg_s | ({4{du_hit[k]}} & du_wimg[k]);
    end
  end
`ifdef VERILATOR
  always_ff @(posedge clk)
    for (int k = 0; k < DUN; k++)
      if (!rst && du_hit[k] != (du_v[k] && du_pr[k] == msr_q[MSR_PR] &&
                                sum_is(op_x[31:12], op_y[31:12], du_pg[k], ea_lo[12])))
        $fatal(1, "micro-TLB %0d: the laid-out compare against sum_is", k);
`endif
  // the page's AND-OR of entries 0-2 as one level, so the page and the real
  // mode select after it are a second (d_pahi below). (DUN = 2: entries 0-1 as
  // that level; DUN = 6 or 8: entries 3-5 and 6-7 as more groups beside it, and the
  // low five bits take EA[15-19] by the DBAT hit, db_any, which never comes with a
  // micro-TLB hit, in place of "no micro-TLB hit": where neither hits there is no
  // request, and the page is not used)
  localparam int DUA = (DUN < 3) ? DUN : 3;           // entries in the first group
  logic [19:0] du_p012_c;
  logic        du_h012_c;
  always_comb begin
    du_p012_c = 20'd0; du_h012_c = 1'b0;
    for (int k = 0; k < DUA; k++) begin
      du_p012_c = du_p012_c | ({20{du_hit[k]}} & du_pa[k]);
      du_h012_c = du_h012_c || du_hit[k];
    end
  end
  (* keep = 1 *) wire [19:0] du_p012 = du_p012_c;
  (* keep = 1 *) wire        du_n012 = !du_h012_c;
  wire [19:0] du_p345, du_p67;
  generate if (DUN == 8) begin : g_du8
    (* keep = 1 *) wire [19:0] p345 = ({20{du_hit[3]}} & du_pa[3]) | ({20{du_hit[4]}} & du_pa[4]) |
                                      ({20{du_hit[5]}} & du_pa[5]);
    (* keep = 1 *) wire [19:0] p67  = ({20{du_hit[6]}} & du_pa[6]) | ({20{du_hit[7]}} & du_pa[7]);
    assign du_p345 = p345;
    assign du_p67  = p67;
  end else if (DUN == 6) begin : g_du6
    (* keep = 1 *) wire [19:0] p345 = ({20{du_hit[3]}} & du_pa[3]) | ({20{du_hit[4]}} & du_pa[4]) |
                                      ({20{du_hit[5]}} & du_pa[5]);
    assign du_p345 = p345;
    assign du_p67  = 20'd0;
  end else begin : g_du4
    assign du_p345 = 20'd0;
    assign du_p67  = 20'd0;
  end endgenerate
  wire         d_xl    = msr_q[MSR_DR] && (is_ld || is_st);
  // the access goes ahead from EX when a BAT allows it or the micro-TLB does;
  // anything else, a BAT protection fault included, is translated in MEM
  wire         xl_rp   = d_xl && !((|db_go) || (|du_use));
  // A DBAT hit and a micro-TLB hit never coincide: an entry is filled only by
  // a replay, which no allowing DBAT hit makes and a refusing one turns into
  // a DSI; a BAT write empties the micro-TLB, and its entries carry MSR[PR],
  // which picks the BATs' valid bits. So the two translations are ORed, with
  // no select by db_any (the select was a level of the R63 full builds' EX
  // address paths); where neither hits there is no request.
  wire         du_any  = |du_hit;
  wire  [19:0] du_p3   = (DUN == 4) ? {20{du_hit[DUN-1]}} & du_pa[DUN-1] : 20'd0;
  wire         du_n3   = (DUN == 4) ? !du_hit[DUN-1] : 1'b1;
  wire  [19:0] d_pahi  = !msr_q[MSR_DR] ? ea_x[31:12] :
                         (DUN > 4) ? {db_pa | du_p012[19:5] | du_p345[19:5] | du_p67[19:5],
                                       du_p012[4:0] | du_p345[4:0] | du_p67[4:0] | ({5{db_any}} & ea_x[16:12])} :
                         {db_pa | du_p012[19:5] | du_p3[19:5],
                          du_p012[4:0] | du_p3[4:0] | ({5{du_n012 && du_n3}} & ea_x[16:12])};
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && msr_q[MSR_DR] && (DUN <= 4 || db_any || du_any) &&
        d_pahi != {db_pa | du_pahi[19:5], du_any ? du_pahi[4:0] : ea_x[16:12]})
      $fatal(1, "d_pahi's laid-out form against the plain one");
`endif
  // the access's WIMG: real mode 0011 (603e 3.5), the DBAT's, the micro-TLB's
  wire  [3:0]  d_wimg_x = !msr_q[MSR_DR] ? 4'b0011 : db_wimg | du_wimg_s;
`ifdef VERILATOR
  // against the selected form from the permitted entries (du_use)
  always_ff @(posedge clk)
    if (!rst && d_req && msr_q[MSR_DR]) begin
      automatic logic [19:0] up = 20'd0;
      automatic logic [3:0]  uw = 4'd0;
      for (int k = 0; k < DUN; k++) begin
        up = up | ({20{du_use[k]}} & du_pa[k]);
        uw = uw | ({4{du_use[k]}} & du_wimg[k]);
      end
      if (db_any && du_any || d_pahi != (db_any ? {db_pa, ea_x[16:12]} : up) ||
          d_wimg_x != (db_any ? db_wimg : uw))
        $fatal(1, "a DBAT and a micro-TLB hit together, or the ORed translation differs");
    end
`endif
  // dcbz to a W or I page takes the alignment exception from MEM (3.7.4,
  // table 3-8). The request goes out regardless (the data cache leaves a dcbz
  // to a W or I page alone, ppc_dcache.sv), so the translation's WIMG is not
  // in front of the request or MEM's fields: EX registers the W or I of the
  // DBAT and of the micro-TLB entry that hit, and MEM chooses (m_algc). A
  // replay finds the WIMG on its refetch.
  logic        db_wi, du_wi;
  always_comb begin
    db_wi = 1'b0; du_wi = 1'b0;
    for (int k = 0; k < 4; k++)                        // (the four DBATs)
      db_wi = db_wi | (db_hit[k] && (bat_wimg[4 + k][3] || bat_wimg[4 + k][2]));
    for (int k = 0; k < DUN; k++)
      du_wi = du_wi | (du_use[k] && (du_wimg[k][3] || du_wimg[k][2]));
  end

  // Asynchronous interrupts (4.5.5, 4.5.9), maskable by MSR[EE]; the external
  // interrupt has priority over the decrementer (table 4-3). Taken in place of
  // the first uop of an instruction as it leaves EX, so the boundary is before
  // that instruction. A kill from MEM (m_kill) is older and wins.
  logic        int_q, dec_pend;
  logic        ex_first, ex_ifault, ex_mmuw;
  logic [2:0]  ex_bk;                                  // the BTB key bits {bslot, bpp, bsq} (fetch)
  logic        ex_lar;                                 // resolved at fetch (LA): EX must agree
  logic [4:0]  ex_step;
  logic        m_fpxr;                                 // MEM's FP op takes its program exception
  logic        tnop;                                   // MEM's touch faulted: it completes as a no-op
  // (not while WB waits for an older answer: the kill and its commit go
  // together as MEM is left)
  // (a touch whose translation faults completes as a no-op: no kill, tnop)
  wire         m_algc   = (m.dz && !m.rp && msr_q[MSR_DR] && (m.dbany ? m.dbwi : m.duwi)) || m.algn;
  // (as two products: a replay's, then the rest)
  (* keep = 1 *) wire m_kp1 = m.valid && m.rp && sl_done && !(m.tch && (sl_dsi_q || sl_tm_q));
  (* keep = 1 *) wire m_kp2 = m.valid && (m.resync || m.rtm || m_fpxr || m_algc);
  wire         m_kp     = m_kp1 || m_kp2;
  wire         m_kill   = m_kp && !wb_hold;
  // (where EX advances, MEM does too, so WB is not holding: there m_kill is
  // m_kp, which is formed from registers alone and not behind WB's wait)
  // MSR[EE] registered for the request (timing, study R104: msr_q through the request, EX's exception
  // and the redirect into fetch's registers was -1.11 ns at 100 MHz). Every MSR write redirects, so no
  // instruction reaches EX in the clock where the copy lags an EE change. The sources stay live: a first
  // version registered the whole request, and an interrupt line that dropped while EE was 1 was then
  // still seen for a clock, taken with int_q low, as a decrementer interrupt (0x900); Daytona 2 hung on
  // its warning screen with it (study R104).
  logic        ee_q;
  always_ff @(posedge clk) ee_q <= !rst && msr_q[MSR_EE];
  wire         irq_req  = ee_q && (int_q || dec_pend);
`ifdef R104_CHECK
  always_ff @(posedge clk) if (!rst && ex_valid && ee_q != msr_q[MSR_EE]) $fatal(1, "EE copy behind EX");
`endif

  // EX holds for multiply/divide, for a load or store the memory side cannot
  // take (d_rdy), and whenever MEM holds.
  //
  // A uop that redirects from EX and commits MSR, SRR0 or SRR1 as it leaves
  // MEM (sc, illegal, trap, rfi, mtmsr, isync, the exceptions taken in EX)
  // leaves EX only when nothing older can hold it in MEM: no request in MEM
  // and none WB may wait for. The instructions fetched after its redirect
  // read those registers committed (msr_q in ID and EX, SRR0/SRR1 in ID);
  // with a MEM held behind a load they could reach ID first.
  logic ser_wait;                                      // below, after fpx_cond
  wire md_wait = ex_is_md && !md_ready;
  // sync waits until every access before it is done (an answer is only an
  // acknowledge: a store may still be buffered on the memory side, d_idle)
  wire sync_wait = ex_valid && ex_u.sync && (wb_mayh || (m.valid && m_iss) || !d_idle);
  // (ex_hold_o: every hold but a split's first cycle, mis_hold, which still
  // makes its request and sends its first word to MEM)
  // (R140) Each copy of EX's hold as three parts: one of every term formed inside the core (from registers,
  // two or three levels), then the D-cache's registers last: d_rdy and d_idle (h*_a), d_valid and d_eok (h*_b).
  // Left to synthesis, d_rdy and d_idle went in at the bottom of the OR tree, three or four levels before the
  // hold (ppc_cpu spike: d_idle -> sync_wait_q -> qh_x -> f_hold -> fa_q's enable; the full build's worst,
  // d_rdy's logic -> two LUTs -> qh_x -> f_hold -> fc_q, then into the single unit's operand select).
  (* keep = 1 *) wire ex_vls  = ex_valid && ex_ls;
  (* keep = 1 *) wire ex_vsy  = ex_valid && ex_u.sync;
  (* keep = 1 *) wire m_vdw   = m.valid && mh_dw;                     // MEM waits on WB's unanswered request
  (* keep = 1 *) wire lsf_wo  = ex_lsf && wb_out;                     // lu2_cf but for d_valid and d_eok
  // ex_hold's copy
  (* keep = 1 *) wire hx_r = mh_e || fp_wait || (ex_vsy && (wb_mayh || (m.valid && m_iss)));   // (from registers)
  (* keep = 1 *) wire hx_a = (ex_vls && !d_rdy) || (ex_vsy && !d_idle);
  (* keep = 1 *) wire hx_b = (m_vdw && !d_valid) || (LU2 && ex_lsw && !(lsf_wo && d_valid && d_eok));
  wire   ex_hold_o = hx_r || md_wait || ser_wait || hx_a || hx_b;
  assign ex_hold = ex_hold_o || mis_hold;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && ex_hold_o != (mem_hold || md_wait || fp_wait || ser_wait || sync_wait || (ex_valid && ex_ls && !d_rdy) || lsh))
      $fatal(1, "ex_hold_o by parts against its terms");
`endif
  // the queue's copy (q_take above): the same terms, kept apart
  (* keep = 1 *) wire qh_1 = m.valid && wb_valid && wb_iss && !wb_got && wb_sp1;
  (* keep = 1 *) wire qh_2 = m.valid && wb_valid && wb_fl && (FLW ? fl_v && (fr_at[0] || wb_blk) : !(wb_got && wb_c2 && !fr_at[0] && !wb_blk));
  (* keep = 1 *) wire qh_3 = m.valid && ((m_rpq && !sl_done) || (m.fsm && (fr_at[1] || wb_blk)));
  (* keep = 1 *) wire qh_4 = m.valid && ((m.fpser && !m_fsok) || (m.icbi && !m_ibdone && !icbi_ack));
  // (the queue's copies of ser_wait and sync_wait take MEM's request from
  // m_rpq, a copy of m.rp of their own: m.rp drives some 65 loads, and led
  // the queue pop's paths in the R63 batch-7 full builds)
  wire  m_issq      = m.req && !m_rpq;
  wire  ser_wait_q  = ex_valid && ex_serz && (wb_mayh || (m.valid && (m_issq || m.fld)));
  wire  sync_wait_q = ex_valid && ex_u.sync && (wb_mayh || (m.valid && m_issq) || !d_idle);
  // (the terms from registers in one part, qh_r; the deeper ones, mis_hold, ser_wait_q and md_wait, beside
  // it in the last LUT)
  (* keep = 1 *) wire qh_r = qh_1 || qh_2 || qh_3 || qh_4 || fp_wait || (ex_vsy && (wb_mayh || (m.valid && m_issq)));
  (* keep = 1 *) wire qh_a = (ex_vls && !d_rdy) || (ex_vsy && !d_idle);
  (* keep = 1 *) wire qh_b = (m_vdw && !d_valid) || (LU2 && ex_lsw && !(lsf_wo && d_valid && d_eok));
  assign q_hold = qh_r || mis_hold || ser_wait_q || md_wait || qh_a || qh_b;
  // a third copy for the FP operand registers' 194 enables (R63 batch 3:
  // EX's hold into fa_q / fb_q / fc_q)
  (* keep = 1 *) wire fh_r = qh_1 || qh_2 || qh_3 || qh_4 || fp_wait || (ex_vsy && (wb_mayh || (m.valid && m_issq)));
  (* keep = 1 *) wire fh_a = (ex_vls && !d_rdy) || (ex_vsy && !d_idle);
  (* keep = 1 *) wire fh_b = (m_vdw && !d_valid) || (LU2 && ex_lsw && !(lsf_wo && d_valid && d_eok));
  (* keep = 1 *) wire f_hold = fh_r || mis_hold || ser_wait_q || md_wait || fh_a || fh_b;
`ifdef VERILATOR
  always_ff @(posedge clk) if (R1C && !rst && f_hold != ex_hold) $fatal(1, "f_hold");
  always_ff @(posedge clk)
    if (R1C && !rst && q_hold != (qh_1 || qh_2 || qh_3 || qh_4 || md_wait || fp_wait || ser_wait_q || sync_wait_q ||
                           (ex_valid && ex_ls && !d_rdy) || mis_hold || lsh || (m.valid && mh_dw && !d_valid)))
      $fatal(1, "q_hold by parts against its terms");
`endif
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && q_hold != ex_hold) $fatal(1, "q_hold");
  always_ff @(posedge clk) if (!rst && m.valid && m_rpq != m.rp) $fatal(1, "m_rpq");
`endif
  wire  ex_adv  = ex_valid && !ex_hold;
  // A program exception for an enabled FP exception left pending by an MSR
  // write: FPSCR[FEX] = 1 when MSR[FE0/FE1] become nonzero (mtmsr, rfi, both
  // waiting for the FP state to settle) is taken before the next instruction,
  // with SRR0 that instruction (PEM table 6-16); an FP instruction in precise
  // mode takes its own from MEM (below), so FEX is never set here otherwise.
  wire  fpx_cond = (msr_q[11] || msr_q[8]) && fpscr_q[30];
  logic ex_serc;                                       // ex_kmsr != 0, isync, a fault marker, FP unavailable
  wire  ex_serz  = ex_serc || irq_req || fpx_cond;
  assign ser_wait = ex_valid && ex_serz && (wb_mayh || (m.valid && (m_iss || m.fld)));
  // EX's uop becomes an exception: a fetch fault marker, FP unavailable, or
  // an interrupt or a pending FP program exception before the first uop of
  // an instruction. Only where the uop leaves EX (ex_adv) and MEM does not
  // kill it does this act, so it is formed from registers alone and EX's hold
  // and MEM's kill are not in front of MEM's fields
  // (not in a split's second cycle once its first word has gone, ex_s0ok:
  // the instruction completes, and the interrupt is taken before the next)
  wire  xc_irq   = irq_req && ex_first && !ex_s0ok;
  wire  xc_fpx   = fpx_cond && ex_first;
  wire  ex_xcm   = ex_ifault || xc_irq || xc_fpx || ex_fpun;

  // the early forward selects' write bits (qe_hk / qe_hf, qf_m, qf_w above)
  wire        qf_mx = ex_u.rd_we && !ex_xcm && !is_ld && !mis_hold && !algn;   // mx.rd_we && !mx.is_load
  wire        qf_wm = m.rd_we && !m.rp;                  // wb_rd_we as WB takes MEM's uop
  always_ff @(posedge clk) begin
    qf_m <= mem_hold ? qe_hk[2] & {4{m.rd_we && !m.is_load}} : (ex_hold ? qe_hk[1] : qe_hf[1]) & {4{qf_mx}};
    qf_w <= wb_hold  ? (ex_hold ? qe_hk[3] : qe_hf[3]) & {4{wb_rd_we}} : (ex_hold ? qe_hk[2] : qe_hf[2]) & {4{qf_wm}};
  end

  // (R140) as three parts and the D-cache's own registers' hold terms (hx_a, hx_b) at the last LUT: EX's own
  // state (dq_e), the translation (dq_t, late from EX's sum) and the core's holds (dq_h)
  (* keep = 1 *) wire dq_e = ex_valid && (is_ld || (is_st && st_ok)) && !ex_xcm && !m_kp;
  (* keep = 1 *) wire dq_t = !xl_rp && !algn;
  (* keep = 1 *) wire dq_h = !(hx_r || md_wait || ser_wait);
  assign d_req   = dq_e && dq_t && dq_h && !hx_a && !hx_b;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && d_req != (ex_valid && !ex_hold_o && (is_ld || (is_st && st_ok)) && !xl_rp && !ex_xcm && !m_kp && !algn))
      $fatal(1, "d_req by parts");
`endif
  // a split's first word goes into MEM as a uop of its own (m.sp0) when it
  // makes its request; otherwise (a replay, an exception) MEM takes nothing
  // this cycle, and the uop goes next cycle as without a split
  wire   mis_go = mis_hold && !ex_hold_o && !xl_rp && !ex_xcm && !algn;
  // (R140) MEM's valid bit with the translation's replay (xl_rp, late from EX's sum) at the last LUT: through
  // mis_go and d_req's shared logic it was two levels in front of m.valid (ppc_cpu spike: op_y -> the
  // micro-TLB compare -> du_use -> xl_rp -> d_req -> m.valid, -1.88 ns at 120 MHz)
  (* keep = 1 *) wire mv_a = ex_adv && !m_kp;
  (* keep = 1 *) wire mv_b = mis_hold && !ex_hold_o && !ex_xcm && !algn && !m_kp;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && (mv_a || (mv_b && !xl_rp)) != ((ex_adv || mis_go) && !m_kp)) $fatal(1, "m.valid's next value by parts");
`endif
  assign d_we    = is_st && ex_u.cop == 3'd0;
  assign d_addr  = {d_pahi, ea_x[11:2], 2'b00};
  assign d_be    = st_be;                              // a load's lanes too (an inhibited load is made with them)
  assign d_wdata = st_data;
  assign d_wimg  = d_wimg_x;
  assign d_op    = ex_u.cop;
  assign d_resv  = ex_u.reserve;
  assign d_cond  = ex_u.cond;

  // ============================================================ floating point
  // 603e chapter 2.3.4.2 and 6.4.3; the units are ppc_fpu*.sv.
  //
  // FPRs (ppc_fpregfile, MLAB) are read in ID as the GPRs are and registered
  // into EX, with the write port forwarded. The multiply-add pipeline and the
  // divider are issued from EX as the uop leaves it (so an older instruction
  // that faults in MEM kills it first: once issued nothing older can fault);
  // their results come back on one port, FL cycles later for the multiply-add
  // unit, the divider's after its hand-over. The small unit (fmr, fneg, fabs,
  // fnabs, fsel, mffs; two cycles) and FP loads write their FPR in WB.
  //
  // Fast port (ppc_fpu_fast, study D-21, D-22): a single op whose operands
  // are all single values (fpu_fast_route, registered with the operands as
  // ex_ffast) and frsqrte write LS cycles after issue instead of FL. A fast
  // op issues only when no result is due LS or more cycles ahead (ffbl_q), so
  // results still come in issue order.
  //
  // FPR writes. One write port: an FPU result and a WB write never land in one
  // cycle, because fr_at says which cycles the FPU port will write and a WB
  // writer is held instead (a small-unit op in EX, an FP load in MEM). The
  // multiply-add unit's results come exactly FL (fast: LS) cycles after
  // issue; the divider's FDL cycles after it hands its result over, the last
  // cycle of d_inj_req (WB writers wait while it asks, and the cycle after).
  //
  // Scoreboard. pend[r]: a write of FPR r is outstanding, set as its producer
  // leaves EX, cleared as it is written (or as an FP load is replayed in MEM).
  // A first uop waits in ID while an FPR it reads is pending (RAW), or one it
  // writes is (WAW), unless that write lands this cycle; the producer in EX is
  // compared directly. With FWW a WAW wait on an FPU-port write ends when that
  // write lands in the uop's first EX cycle or the next, and a port write older
  // than a uop that has left EX for the same FPR does not clear its bit (wsk_q). A read whose write lands this cycle takes the write
  // port's data, except lfs's, which is converted in WB and read a cycle later.
  //
  // FPSCR in program order. Multiply-add results come in issue order; a
  // divide could finish after younger ones, so no multiply-add op issues while
  // a divide is between its start and its hand-over (the 603e blocks its FPU
  // for fdiv, fdivs and fres, 603e 6.4.3), and a divide starts only once the
  // multiply-add ops ahead of it are past the point where its result joins
  // them. Compares, mffs, mcrfs, the
  // mtfs* instructions and Rc = 1 moves wait
  // in EX until nothing older can change the FPSCR (fs_idle): "every FPSCR
  // instruction appears to synchronize the effects of all floating-point
  // instructions" (PEM 4.2.2.5, 603e 2.3.4.2.5), and R30's census reads FPRF
  // through mcrfs straight after an op. A compare's flags enter the FPSCR in
  // MEM; the mtfs* instructions and mcrfs write it as they leave EX.
  //
  // CR1 (Rc = 1) is FPSCR[0-3] after the instruction (PEM table 2-2): known in
  // EX for the moves and FPSCR instructions; an arithmetic op with Rc = 1 is
  // held in MEM until its result is in and takes it then.
  //
  // Exceptions. MSR[FP] = 0: FP unavailable (0x800) in place of the first uop
  // in EX (603e 4.5.8). MSR[FE0/FE1] = 00: enabled exceptions are ignored, as
  // in all three games (R30); an enabled invalid or zero divide still writes
  // no frD (the units' flags.wr). Any other FE setting is precise mode on the
  // 603e (603e 4.1, 4.5.7.1: the imprecise modes are implemented as precise):
  // every op that can change the FPSCR is then held in MEM until it has, and
  // if FEX is set the program exception (0x700, SRR1[11]) is taken with SRR0
  // the instruction, which has executed (PEM 3.3.6, table 6-16).
  localparam int FL  = 3 + int'(P1A) + int'(P2) + int'(P3) + int'(P4A) + int'(P5A) + int'(P5) + int'(P6A) + int'(P6) + int'(P7);
  localparam int FDL = 2 + int'(P5A) + int'(P5) + int'(P6A) + int'(P6) + int'(P7);   // hand-over to result
  // cycles from issue to S4b, where the divider's result joins: a divide
  // starts only once every multiply-add op ahead of it is past S4b, or a
  // short one (special operands) could hand over in front of an older op
  localparam int FDP = 1 + int'(P1A) + int'(P2) + int'(P3) + int'(P4A);
  localparam logic [FL-1:0] FPRE = ~({FL{1'b1}} >> (FDP - 1));    // fr_at[FL-1 : FL-FDP+1]
  // the fast port's latency: ppc_fpu_sfma's stage boundaries (SQ1 ... SQ5; all five, LS = 6: R53, 6
  // closes, 5 does not)
  localparam int LS  = sfma_latency(SQ1, SQ2, SQ3, SQ4, SQ5);

  // -------------------------------------------------- FPR file, operands
  logic [63:0] rf_fa, rf_fb, rf_fc;
  logic        fo_valid;
  logic [4:0]  fo_tag;
  logic [63:0] fo_res;
  fpu_flags_t  fo_flags;
  logic [63:0] lfs_dbl;
  wire  [4:0]  id_fa = q_hi[20:16], id_fb = q_hi[15:11], id_fc = q_pd.fci;
  wire         fw_fpu = fo_valid && fo_flags.wr;
  // a WB writer writes as it leaves: the small unit's at once (MEM let it go
  // only into a free port cycle), an FP load once its word has been here a
  // cycle (lfs's conversion) and the FPU port is free; from registers
  wire         wb_fpok = wb_got && wb_c2 && !fr_at[0] && !wb_blk;
  // FP load write slot (FLW). The 603e's FP load is 2:1 like its integer load (603e table 6-6); here the
  // word arrives in WB, and lfs's conversion takes that cycle and the next (u_lfs). With FLW the load
  // leaves WB as its word arrives (as an integer load does) into fl_*, which writes the FPR in the next
  // cycle the port is free: after the FPU port's result (fr_at[0]) and a small-unit result leaving WB
  // (fw_wbs), which come first. In its first cycle the slot's lfs value is u_lfs's output, from the word
  // registered as the load left WB; if the port is busy it is kept (fl_dat, fl_p2). A second FP load
  // waits in WB while the slot is full. The scoreboard keeps frD pending until the slot writes it.
  logic        fl_v, fl_p2, fl_sl;
  logic [4:0]  fl_frd;
  logic [31:0] fl_hi, fl_lo;
  logic [63:0] fl_dat;
  wire         fw_wbs = wb_valid && wb_fpw && !wb_isld;          // a small-unit result, written as WB is left
  wire         fl_go  = FLW && fl_v && !fr_at[0] && !fw_wbs && !wb_blk;
  wire  [63:0] fl_val = fl_sl ? (fl_p2 ? fl_dat : lfs_dbl) : {fl_hi, fl_lo};
  // WB lets an FP load go: with FLW once the slot is free or frees now, else once written (wb_fpok)
  wire         wb_flok = FLW ? (!fl_v || fl_go) : wb_fpok;
  wire         fw_wbp = FLW ? fw_wbs : wb_valid && wb_fpw && (!wb_isld || wb_fpok);
  // the WB-side write this cycle (at most one: the slot waits for a small-unit result), for the scoreboard
  wire         fw_wv  = fw_wbp || fl_go;
  // (R140, FLW) the WB side's index and data chosen by fw_wbs (one LUT of WB's registers) where they were
  // chosen by fl_go: wherever there is a WB-side write it is fw_wbs's (a small-unit result) or the slot's
  // (fl_go, never with fw_wbs), so the two agree wherever the port writes. Through fl_go (81 loads), WB's
  // wb_fpw reached the FP register file's write data four levels deep (c7_s1: wb_fpw -> fl_go -> fw_dat ->
  // the MLAB, -1.46 ns at 110 MHz)
  wire  [4:0]  fw_wr  = FLW ? (fw_wbs ? wb_frd : fl_frd) : (fl_go ? fl_frd : wb_frd);
  wire         fw_we  = fw_fpu || fw_wv;
  wire  [4:0]  fw_idx = fo_valid ? fo_tag : fw_wr;
  wire  [63:0] fw_dat = fo_valid ? fo_res : FLW ? (fw_wbs ? wb_fdat_q : fl_val) :
                       fl_go ? fl_val : !wb_isld ? wb_fdat_q : wb_fsl ? lfs_dbl : {fhi_q, wb_dq};
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && fw_we && !fo_valid &&
        ({fw_wr, fw_dat} != {fl_go ? fl_frd : wb_frd, fl_go ? fl_val : !wb_isld ? wb_fdat_q : wb_fsl ? lfs_dbl : {fhi_q, wb_dq}}))
      $fatal(1, "FPR write port: the WB side by fw_wbs against fl_go");
`endif
  // the forward into ID's capture: an FPU result or a small-unit result (both
  // from registers); not a load's word (lfs is converted in WB, lfd's second
  // word arrives in MEM), which is read from the file a cycle later
  wire  [63:0] fw_fdt = fo_valid ? fo_res : wb_fdat_q;

  // the written value's !fpu_sgl_val, per source (fw_dat's select after)
  logic        rf_na, rf_nb, rf_nc;
  // (lfs's word converted is always a single value: 1, checked at every read
  // below; its conversion is in front of the write, R63 batch 4)
  wire         fw_nsv = !(fo_valid ? sv_fo : FLW ? (fw_wbs ? sv_wbf : (fl_sl || fpu_sgl_val({fl_hi, fl_lo}))) :
                          fl_go ? (fl_sl || fpu_sgl_val({fl_hi, fl_lo})) : !wb_isld ? sv_wbf :
                          wb_fsl ? 1'b1 : fpu_sgl_val({fhi_q, wb_dq}));
  ppc_fpregfile u_frf (
    .clk (clk), .ra (id_fa), .rb (id_fb), .rc (id_fc),
    .qa (rf_fa), .qb (rf_fb), .qc (rf_fc), .na (rf_na), .nb (rf_nb), .nc (rf_nc),
    .we (fw_we), .wa (fw_idx), .wd (fw_dat), .wn (fw_nsv)
  );

  // ------------------------------------------------------------ scoreboard
  // Everything ID needs is formed a cycle ahead, for both uops that can be the
  // queue head next cycle (entry 0 if ID does not take it, entry 1 if it
  // does), and registered: the hazard and the forward selects then come from
  // registers (the forward select drives 64 bits of each operand copy). A
  // head that arrives straight from fetch has none and waits a cycle. The
  // early forms are conservative where the exact one depends on this cycle's
  // late signals: a producer in EX counts as issuing, a load's write in WB
  // never forwards. An FPU result forwards only while FPSCR[VE] = FPSCR[ZE]
  // = 0, when its write enable is always 1 (ppc_fpu_fma: flags.wr is 0 only
  // for an enabled invalid or zero divide); those bits change only from an
  // FPSCR instruction, which waits for the FPU to empty.
  logic [31:0] pend;
  logic        ex_frw;                                 // = ex_fpw (the scoreboard's copy)
  logic [31:0] ex_froh;                                // ex_frw, as a one-hot of frD (pend's set is then
                                                       // one LUT after the late fp_go)
  wire  [4:0]  ex_frd = ex_u.rd;
  wire         fp_go  = ex_adv && !m_kp && !ex_xcm;    // an FP uop leaves EX for good
  logic [FL-1:0]      fr_at;                           // [k]: the FPU port writes k cycles from now
  logic [FL-1:0][4:0] fr_ix;                           // ... that write's FPR
  wire         fok    = !fpscr_q[7] && !fpscr_q[4];    // VE, ZE: an FPU result forwards
  // FWW: wsk_q, the port's write this cycle is older than a uop writing the same FPR that left EX last cycle
  // (with the write then a cycle ahead), so it does not clear the FPR's pending bit (fo_clr)
  logic        wsk_q;
  logic [4:0]  wsk_r;
  wire         fo_clr = fo_valid && !(FWW && wsk_q && wsk_r == fo_tag);
  // a small-unit result in WB next cycle: only where MEM surely advances (no
  // FPR port wait, no answer WB could still wait for)
  assign       wb_mayh = wb_valid && ((wb_iss && !wb_got) || wb_fl);
  wire         m_mwb  = m.valid && m.fpw && !m.is_load && !fr_at[1] && !wb_blk && !wb_mayh &&
                        !(m.fpser && !m_fsok);
  // FPR r pending next cycle, conservatively: pend[r] looked up first, and
  // this cycle's clears and EX's producer applied by comparing r, so the
  // lookup and the compares run side by side
  function automatic logic f_pn(input logic [31:0] pd, input logic [4:0] r, input logic fv, input logic [4:0] ft,
                                input logic wv, input logic [4:0] wr, input logic xv, input logic [4:0] xr);
    f_pn = (pd[r] && !(fv && ft == r) && !(wv && wr == r)) || (xv && xr == r);
  endfunction
  wire         ex_fh  = ex_valid && ex_frw;
  function automatic logic f_hit(input logic [4:0] r, input logic fa, input logic [4:0] fi, input logic mw, input logic [4:0] mr);
    f_hit = (fa && fi == r) || (mw && mr == r);
  endfunction
  // per candidate head h (0: entry 0, 1: entry 1): {A, B, C} forward hit and
  // ready, frD free, and the whole hazard. Entry 1 is the head only after a
  // pop, when today's head has gone to EX: its frD counts as pending too.
  logic [1:0][2:0] nx_hit, nx_rdy, hd_hit;
  // FP operand bypass (FBYP): an operand whose FPU-port write lands the cycle after next (fr_at[2]) is ready
  // for a head that would enter EX next cycle and takes the result in its first EX cycle from fo_res
  // (ex_fbl). Only for the FPU's own consumers (multiply-add port, divider, small unit): a store's data
  // (fsv, d_wdata) and mtfsf's frB stay on the operand registers. Forwarded only while VE = ZE = 0 (fok).
  logic [1:0][2:0] nx_byp, hd_byp;
  wire  [1:0]      nx_bok = {2{FBYP && fok}} & {q1_pd.ffma || q1_pd.fdiv || q1_pd.fmsc, q_pd.ffma || q_pd.fdiv || q_pd.fmsc};
  // FP early forward (FEF): an operand whose write lands three cycles ahead (fr_at[3]) from a single-unit op
  // issued with OE = UE = 0 (fr_sf) is ready for a head of the fast kind (fadds ... fnmsubs) that would enter EX
  // next cycle; it issues in its first EX cycle and the single unit takes the operand from its R5 (ex_fbe)
  logic [FL-1:0]   fr_sf;                              // [k]: the port's write k cycles from now is the single unit's
  logic [1:0][2:0] nx_bye, hd_bye;
  wire  [1:0]      nx_boe = {2{FEF && FBYP && SQ5 && FAST && fok}} & {q1_pd.frs, q_pd.frs};
  logic [1:0]      nx_wfr, nx_haz;
  logic            hd_fz;
  logic            hd_sel, hd_val;
  wire             id_fpw = id_valid && q_pd.fpw;
  wire  [1:0]      nx_fua = {q1_pd.fua, q_pd.fua}, nx_fub = {q1_pd.fub, q_pd.fub};
  wire  [1:0]      nx_fuc = {q1_pd.fuc, q_pd.fuc}, nx_fpw = {q1_pd.fpw, q_pd.fpw};
  logic [1:0][2:0][4:0] nx_ix;                         // [h][A, B, C]
  logic [1:0][4:0]      nx_d;
  assign nx_ix = {{q1_pd.fci, q1_hi[15:11], q1_hi[20:16]}, {q_pd.fci, q_hi[15:11], q_hi[20:16]}};
  assign nx_d  = {q1_hi[25:21], q_hi[25:21]};
  always_comb
    for (int h = 0; h < 2; h++) begin
      for (int p = 0; p < 3; p++) begin
        nx_hit[h][p] = f_hit(nx_ix[h][p], fr_at[1] && fok, fr_ix[1], m_mwb, m.rd);
        nx_byp[h][p] = nx_bok[h] && fr_at[2] && fr_ix[2] == nx_ix[h][p];
        nx_bye[h][p] = nx_boe[h] && fr_at[3] && fr_sf[3] && fr_ix[3] == nx_ix[h][p];
        // (FWW: with a writer of the FPR in EX, any port write to it in the schedule is older)
        nx_rdy[h][p] = (((nx_hit[h][p] || nx_byp[h][p] || nx_bye[h][p]) && !(FWW && ex_fh && ex_frd == nx_ix[h][p])) ||
                        !f_pn(pend, nx_ix[h][p], fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd)) &&
                       !(h == 1 && id_fpw && q_hi[25:21] == nx_ix[h][p]);
      end
      nx_wfr[h] = (!f_pn(pend, nx_d[h], fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd) ||
                   (((fr_at[1] && fr_ix[1] == nx_d[h]) || (FWW && fr_at[2] && fr_ix[2] == nx_d[h]) ||
                     (FWW && fr_at[3] && fr_ix[3] == nx_d[h])) && !(FWW && ex_fh && ex_frd == nx_d[h])) ||
                   (m_mwb && m.rd == nx_d[h])) &&
                  !(h == 1 && id_fpw && q_hi[25:21] == nx_d[h]);
      nx_haz[h] = (nx_fua[h] && !nx_rdy[h][0]) || (nx_fub[h] && !nx_rdy[h][1]) ||
                  (nx_fuc[h] && !nx_rdy[h][2]) || (nx_fpw[h] && !nx_wfr[h]);
    end
  wire         id_pop = id_take && !redir_hard_q;
  always_ff @(posedge clk) begin
    hd_hit <= nx_hit;
    hd_byp <= nx_byp;
    hd_bye <= nx_bye;
    hd_fz  <= id_pop ? nx_haz[1] : nx_haz[0];          // hd_haz chosen by hd_sel, as registered
    hd_sel <= id_pop;
    hd_val <= !rst && !redir_hard_q && (id_pop ? q_cnt >= 3'd2 : q_cnt >= 3'd1);
  end
  wire  [2:0]  f_hit_id = {3{hd_val}} & (hd_sel ? hd_hit[1] : hd_hit[0]);
  wire  [2:0]  f_byp_id = {3{hd_val}} & (hd_sel ? hd_byp[1] : hd_byp[0]);   // taken from fo_res in EX (ex_fbl)
  wire  [2:0]  f_bye_id = {3{hd_val}} & (hd_sel ? hd_bye[1] : hd_bye[0]);   // taken from the single unit's R5 (ex_fbe)
  // A head with nothing formed for it (straight from fetch) waits a cycle if it names an FPR, unless
  // (FHD) the word, as it arrived, was an FP instruction none of whose four register fields (rD/rS, rA,
  // rB, rC: a superset of the FPRs it reads or writes) names an FPR pending next cycle, counting the
  // head that leaves ID with it (hd_fn = 0). Such a head reads every operand from the file: nothing it
  // names is written in its ID cycle or later, so no forward is needed. From the raw fields, not the
  // predecode (the word is late, R40).
  function automatic logic hd_pn(input logic [4:0] r, input logic [31:0] pd, input logic fv, input logic [4:0] ft,
                                 input logic wv, input logic [4:0] wr, input logic xv, input logic [4:0] xr,
                                 input logic iv, input logic [4:0] ir);
    hd_pn = f_pn(pd, r, fv, ft, wv, wr, xv, xr) || (iv && ir == r);
  endfunction
  wire  [5:0]  hf_op  = i_data[31:26];
  wire         hf_fp  = hf_op == 6'd59 || hf_op == 6'd63 || hf_op[5:3] == 3'b110;     // arithmetic, or 48-55
  wire         hf_pn  = hd_pn(i_data[25:21], pend, fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd, id_fpw, q_hi[25:21]) ||
                        hd_pn(i_data[20:16], pend, fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd, id_fpw, q_hi[25:21]) ||
                        hd_pn(i_data[15:11], pend, fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd, id_fpw, q_hi[25:21]) ||
                        hd_pn(i_data[10:6],  pend, fo_clr, fo_tag, fw_wv, fw_wr, ex_fh, ex_frd, id_fpw, q_hi[25:21]);
  logic        hd_fn;
  always_ff @(posedge clk) hd_fn <= !FHD || !rsp_use || !hf_fp || hf_pn;
  always_comb
    fp_haz = id_valid && id_s0 && (hd_val ? hd_fz : hd_fn && (q_pd.fua || q_pd.fub || q_pd.fuc || q_pd.fpw));
`ifdef R104_CHECK
  // (FHD) a fresh head let through: every FPR it reads or writes is free, the file's read is its value
  always_ff @(posedge clk)
    if (!rst && id_valid && id_s0 && !hd_val && !hd_fn && (q_pd.fua || q_pd.fub || q_pd.fuc || q_pd.fpw) &&
        ((q_pd.fua && (pend[id_fa] || (ex_fh && ex_frd == id_fa))) || (q_pd.fub && (pend[id_fb] || (ex_fh && ex_frd == id_fb))) ||
         (q_pd.fuc && (pend[id_fc] || (ex_fh && ex_frd == id_fc))) || (q_pd.fpw && (pend[q_hi[25:21]] || (ex_fh && ex_frd == q_hi[25:21])))))
      $fatal(1, "fresh FP head %08X let through with an FPR pending", q_hi);
`endif
  // the set's gate as one LUT of the MEM kill's two products (keep), and
  // each bit's held part apart (keep), so the kill is two LUTs from pend:
  // through m_kp, fp_go and a shared AND it was four (a round-2 cached full
  // build, m.duwi into pend)
  // (R140: EX's hold last, from its FP copy f_hold, = ex_hold; through ex_adv, MEM's m.rp reached pend five
  // levels deep in a ppc_cpu spike, -1.52 ns at 120 MHz)
  (* keep = 1 *) wire        pg_e   = ex_valid && !m_kp1 && !m_kp2 && !ex_xcm && ex_frw;
  (* keep = 1 *) wire        pend_g = pg_e && !f_hold;                                   // = fp_go && ex_frw
  (* keep = 1 *) logic [31:0] pend_h;
  always_comb
    for (int i = 0; i < 32; i++)
      pend_h[i] = pend[i] && !(fo_clr && fo_tag == 5'(i)) && !(fw_wv && fw_wr == 5'(i)) &&
                  !(mem_adv && m.fpw && (m.rp || m.algn) && m.rd == 5'(i));
  always_ff @(posedge clk)
    for (int i = 0; i < 32; i++)
      if (rst) pend[i] <= 1'b0;
      else     pend[i] <= pend_h[i] || (pend_g && ex_froh[i]);
  always_ff @(posedge clk) begin
    if (rst) wsk_q <= 1'b0;
    else     wsk_q <= FWW && pend_g && fr_at[1] && fr_ix[1] == ex_frd;
    wsk_r <= ex_frd;
  end

  wire  [63:0] fa_cap = f_hit_id[0] ? fw_fdt : rf_fa;
  wire  [63:0] fb_cap = f_hit_id[1] ? fw_fdt : rf_fb;
  wire  [63:0] fc_cap = f_hit_id[2] ? fw_fdt : rf_fc;
  // The fast route (fpu_fast_route) for these operands, without testing 64
  // bits after the file's read: each FPR carries !fpu_sgl_val of its value
  // (ppc_fpregfile, written below), the forward's values are tested from
  // their registers, and the op's kind is predecoded (frsqrte; a single op
  // reading A with B, C or both, q_pd.fub / fuc)
  // the route the checks expect: fpu_fast_route, or with FAST = 0 frsqrte's alone
  function automatic logic rt_ref(input fma_op_t op, input logic sg, input logic [63:0] a, b, c);
    return FAST ? fpu_fast_route(op, sg, a, b, c) : (op == FMA_RSQRTE);
  endfunction
  wire         sv_fo  = fpu_sgl_val(fo_res);
  wire         sv_wbf = fpu_sgl_val(wb_fdat_q);
  wire         sv_fw  = fo_valid ? sv_fo : sv_wbf;
  // a bypassed operand's value is not here yet: its single-value bit is the one predicted for the result
  // landing next cycle (fr_sv[1]; 1 only where the result is surely a single value, checked below)
  logic [FL-1:0] fr_sv;
  wire         sv_by  = fr_sv[1];
  wire         sv_be  = fr_sv[2];                      // ... and an early-forwarded one's, landing the cycle after
  wire         sv_a   = f_hit_id[0] ? sv_fw : f_byp_id[0] ? sv_by : f_bye_id[0] ? sv_be : !rf_na;
  wire         sv_b   = f_hit_id[1] ? sv_fw : f_byp_id[1] ? sv_by : f_bye_id[1] ? sv_be : !rf_nb;
  wire         sv_c   = f_hit_id[2] ? sv_fw : f_byp_id[2] ? sv_by : f_bye_id[2] ? sv_be : !rf_nc;
  wire         id_frt = q_pd.frq || (FAST && q_pd.frs && sv_a && (!q_pd.fub || sv_b) && (!q_pd.fuc || sv_c));
  // One register per operand for every FP unit and the store data. Separate
  // copies per unit (the FPU's standalone fits pointed at the operand
  // fanout) measured worse in the core on all three seeds and cost 357
  // registers (fit batches b5 and b7)
  // ex_ffast: the fast port's route for these operand values (D-21)
  // (ex_ffast is 0 for an op that is not the multiply-add port's, which
  // fpu_fast_route need not be: it is used only with ex_fma)
  logic [63:0] fa_q, fb_q, fc_q;
  (* preserve *) logic ex_ffast;
  (* preserve *) logic ex_ffw;                         // = ex_ffast, for fp_wait alone (fanout)
  // FP operand bypass: ex_fbl[p] in a consumer's first EX cycle, when operand p is the FPU result written
  // in this cycle (fo_res). The fast port, the divider's copy and the small unit read fa_x / fb_x / fc_x:
  // fo_res in that cycle, and, while the uop is then held in EX, a copy taken in it (fx_*, ex_fbh). The
  // operand registers keep EX's hold on their enables alone (a hold-chosen data input was an earlier
  // form's ppc_cpu spike worst path, the D-cache's dq_v into fa_q, -1.95 ns); they take fo_res only for
  // the double unit (ex_fbd), whose wait cycle is a sure hold
  logic [2:0]  ex_fbl, ex_fbh;
  logic [63:0] fx_a, fx_b, fx_c;
  logic        ex_svb;                                 // frB's single-value bit (frsqrte's result prediction)
  logic        ex_fbu;                                 // EX's uop took a bypassed operand (the route checks)
  // ex_fbd: EX's first cycle holds a bypassed multiply-add op routed to the double unit, which reads the
  // operand registers: it waits that cycle (fp_wait), as they take fo_res (a register of its own, so the
  // wait is not formed from ex_fbl and the route in front of EX's hold)
  logic        ex_fbd;
  // FEF: ex_fbe[p] in a consumer's first EX cycle, operand p from the single unit's R5 (its result is written
  // next cycle). Held there, it becomes ex_fbl[p] in the next cycle (the result is in fo_res then). ex_fs[p]: the
  // single unit's select for operand p, as a register of its own (0 fa_q, 1 fx_a, 2 fo_res, 3 R5), so its
  // operand is one 4:1 select. ex_fbed: an early-forwarded op routed to the double unit waits, as ex_fbd
  // does, for its operand registers to take fo_res (two cycles)
  logic [2:0]       ex_fbe;
  logic [2:0][1:0]  ex_fs;
  logic             ex_fbed;
  wire  [63:0] fa_x = ex_fbl[0] ? fo_res : ex_fbh[0] ? fx_a : fa_q;
  wire  [63:0] fb_x = ex_fbl[1] ? fo_res : ex_fbh[1] ? fx_b : fb_q;
  wire  [63:0] fc_x = ex_fbl[2] ? fo_res : ex_fbh[2] ? fx_c : fc_q;
  wire  [2:0]  fd_l = {3{ex_fbd}} & ex_fbl;            // operand registers that take fo_res (double unit)
  always_ff @(posedge clk) begin
    if (rst) begin
      ex_fbl  <= 3'b000;
      ex_fbd  <= 1'b0;
      ex_fbe  <= 3'b000;
      ex_fbed <= 1'b0;
    end else if (f_hold) begin
      ex_fbl  <= ex_fbe;                               // (0 without FEF)
      ex_fbd  <= ex_fbed;
      ex_fbe  <= 3'b000;
      ex_fbed <= 1'b0;
    end else begin
      ex_fbl  <= f_byp_id;
      ex_fbd  <= f_byp_id != 3'b000 && q_pd.ffma && !id_frt;
      ex_fbe  <= f_bye_id;
      ex_fbed <= f_bye_id != 3'b000 && !id_frt;
    end
    for (int k = 0; k < 3; k++)
      if (rst)         ex_fs[k] <= 2'd0;
      else if (f_hold) ex_fs[k] <= (ex_fs[k] == 2'd3) ? 2'd2 : (ex_fs[k] == 2'd2) ? 2'd1 : ex_fs[k];
      else             ex_fs[k] <= f_bye_id[k] ? 2'd3 : f_byp_id[k] ? 2'd2 : 2'd0;
    if (rst || !f_hold) ex_fbh <= 3'b000;
    else                ex_fbh <= ex_fbh | ex_fbl;
    if (ex_fbl[0]) fx_a <= fo_res;
    if (ex_fbl[1]) fx_b <= fo_res;
    if (ex_fbl[2]) fx_c <= fo_res;
  end
  // (R140) each operand register's enable as two copies, one a half (keep): one enable LUT drove 64 to 75
  // enables with a 2.7 ns route (ppc_cpu spike: op_y -> mis_hold -> f_hold -> fa_q's enable)
  (* keep = 1 *) wire [5:0] f_en = {{2{!f_hold || fd_l[2]}}, {2{!f_hold || fd_l[1]}}, {2{!f_hold || fd_l[0]}}};
  always_ff @(posedge clk) begin
    if (f_en[0]) fa_q[31:0]  <= fd_l[0] ? fo_res[31:0]  : fa_cap[31:0];
    if (f_en[1]) fa_q[63:32] <= fd_l[0] ? fo_res[63:32] : fa_cap[63:32];
    if (f_en[2]) fb_q[31:0]  <= fd_l[1] ? fo_res[31:0]  : fb_cap[31:0];
    if (f_en[3]) fb_q[63:32] <= fd_l[1] ? fo_res[63:32] : fb_cap[63:32];
    if (f_en[4]) fc_q[31:0]  <= fd_l[2] ? fo_res[31:0]  : fc_cap[31:0];
    if (f_en[5]) fc_q[63:32] <= fd_l[2] ? fo_res[63:32] : fc_cap[63:32];
  end
  always_ff @(posedge clk)
    if (!f_hold) begin
      ex_svb <= sv_b;
      ex_fbu <= f_byp_id != 3'b000 || f_bye_id != 3'b000;
      ex_ffast <= id_frt;
      ex_ffw   <= id_frt;
    end

  // ------------------------------------------------------------- the units
  // The divider starts the cycle after its uop leaves EX, from plain copies
  // of the operands and fields (no enable): its start then comes from a
  // register, not from EX's late leaving decision, which drove the
  // divider's operand-register enables (one cycle a divide)
  logic        d_busy, d_inj_req;
  logic        d_go;                                   // start the divider this cycle
  logic [63:0] dq_a, dq_b;
  logic [1:0]  dq_op;
  logic        dq_sgl;
  logic [4:0]  dq_frd;
  always_ff @(posedge clk) begin
    if (rst) d_go <= 1'b0;
    else     d_go <= ex_div && fp_go;
    dq_a   <= fa_x;
    dq_b   <= fb_x;
    dq_op  <= ex_u.fop[1:0];
    dq_sgl <= ex_u.fsgl;
    dq_frd <= ex_frd;
  end
  logic f_fast_ok;                                     // the unit's own issue check (simulation only)
  ppc_fpu_fast #(.TAGW (5), .P1A (P1A), .P2 (P2), .P3 (P3), .P4A (P4A), .P5A (P5A), .P5 (P5), .P6A (P6A), .P6 (P6), .P7 (P7),
                 .SQ1 (SQ1), .SQ2 (SQ2), .SQ3 (SQ3), .SQ4 (SQ4), .SQ5 (SQ5), .EF (FEF && FBYP && SQ5 && FAST), .SF (FAST)) u_fpu (
    .clk (clk), .rst (rst),
    .f_valid (ex_fma && fp_go), .f_op (fma_op_t'(ex_u.fop)), .f_single (ex_u.fsgl), .f_fast (ex_ffast),
    .f_ctl (fpscr_q[7:0]), .f_a (fa_x), .f_b (fb_x), .f_c (fc_x), .f_ad (fa_q), .f_bd (fb_q), .f_cd (fc_q), .f_tag (ex_frd),
    .f_sel (ex_fs), .f_aq (fa_q), .f_bq (fb_q), .f_cq (fc_q), .f_ax (fx_a), .f_bx (fx_b), .f_cx (fx_c), .f_fast_ok (f_fast_ok),
    .d_start (d_go), .d_op (div_op_t'(dq_op)), .d_single (dq_sgl), .d_ctl (fpscr_q[7:0]),
    .d_a (dq_a), .d_b (dq_b), .d_tag (dq_frd), .d_busy (d_busy), .d_inj_req (d_inj_req),
    .out_valid (fo_valid), .out_tag (fo_tag), .out_div (), .out_res (fo_res), .out_flags (fo_flags)
  );

  // the small unit, two cycles: its operands registered at the end of EX
  // (the unit's own register), its result in MEM for WB; a compare's result
  // is taken in a second EX cycle instead, so its CR field and flags go to MEM
  // registered
  logic [63:0] ms_res;
  logic [3:0]  ms_cr;
  fpu_flags_t  ms_flags;
  ppc_fpu_misc #(.PIPE (1'b1)) u_fmisc (
    .clk (clk), .op (misc_op_t'(ex_u.fop[2:0])), .ctl (fpscr_q[7:0]), .a (fa_x), .b (fb_x), .c (fc_x),
    .fpscr (fpscr_q), .res (ms_res), .cr (ms_cr), .flags (ms_flags)
  );

  // lfs: WB's word as it arrives (or as held), converted over that cycle and
  // the next; the load leaves WB in the next at the earliest (wb_fpok)
  logic        m_got;                                  // MEM's answer arrived in an earlier cycle
  logic [31:0] m_dq;
  wire  [31:0] ldw = wb_got ? wb_dq : d_rdata;
  ppc_fpu_ldst #(.PIPE (1'b1)) u_lfs (.clk (clk), .lfs_word (ldw), .lfs_dbl (lfs_dbl), .stfs_dbl (64'd0), .stfs_word ());
  // stfs: from the store operand, a second EX cycle
  logic [31:0] stfs_w;
  ppc_fpu_ldst #(.PIPE (1'b1)) u_stfs (.clk (clk), .lfs_word (32'd0), .lfs_dbl (), .stfs_dbl (fc_q), .stfs_word (stfs_w));
  always_comb
    fsv = ex_u.fiw ? fc_q[31:0] : ex_u.fdbl ? (ex_first ? fc_q[63:32] : fc_q[31:0]) : stfs_w;

  // ------------------------------------------- FPU port schedule, FPSCR order
  logic          div_out, inj_q;                       // a divide between its start and its hand-over
  logic [FL-1:0] fr_at_n;                              // fr_at next cycle
  logic [4:0]    div_frd;
  logic          div_sgl;                              // the divide in flight is single (fr_sv)
  wire           sv_ok    = !fpscr_q[6] && !fpscr_q[5];  // OE = UE = 0: a single result is a single value
  wire           inj_fall = inj_q && !d_inj_req;       // the hand-over was last cycle
  // The divider's write is in fr_at from the cycle after its hand-over, FDL-2
  // cycles ahead: soon enough for a WB writer's check when FDL >= 4 (L = 12:
  // FDL = 7); with fewer stages WB writers also wait while it hands over
  assign         wb_blk   = (FDL < 4) && (d_inj_req || inj_q);
  wire           f_iss    = ex_fma && fp_go;             // an op leaves for the multiply-add port
  assign fr_at_n = (fr_at >> 1) | ({FL{f_iss && !ex_ffast}} & (FL'(1) << (FL - 1))) |
                   ({FL{f_iss && ex_ffast}} & (FL'(1) << (LS - 1))) |
                   ({FL{inj_fall}} & (FL'(1) << (FDL - 2)));
  always_ff @(posedge clk) begin
    if (rst) begin
      fr_at   <= '0;
      div_out <= 1'b0;
      inj_q   <= 1'b0;
    end else begin
      fr_at   <= fr_at_n;
      if (d_go)          div_out <= 1'b1;
      else if (inj_fall) div_out <= 1'b0;
      inj_q   <= d_inj_req;
    end
    for (int k = 0; k < FL - 1; k++) fr_ix[k] <= fr_ix[k + 1];
    fr_ix[FL - 1] <= ex_frd;
    if (f_iss && ex_ffast) fr_ix[LS - 1] <= ex_frd;
    if (inj_fall) fr_ix[FDL - 2] <= div_frd;
    if (d_go) div_frd <= dq_frd;
    // the result surely a single value (FP operand bypass, sv_by): a single-precision op's, frsp's, and
    // frsqrte's of a single value; a divide's when single (fdivs, fres)
    for (int k = 0; k < FL - 1; k++) fr_sv[k] <= fr_sv[k + 1];
    // (with OE = UE = 0: an enabled overflow or underflow delivers the exponent adjusted by 192,
    // PEM 3.3.6.2; the FPSCR changes only with the FPU empty)
    fr_sv[FL - 1] <= sv_ok && (ex_u.fsgl || ex_u.fop == 4'(FMA_RSP) || (ex_u.fop == 4'(FMA_RSQRTE) && ex_svb));
    if (f_iss && ex_ffast) fr_sv[LS - 1] <= sv_ok && (ex_u.fsgl || (ex_u.fop == 4'(FMA_RSQRTE) && ex_svb));
    if (inj_fall) fr_sv[FDL - 2] <= sv_ok && div_sgl;
    // FEF: the write is a single-unit op's (not frsqrte's) issued with OE = UE = 0
    for (int k = 0; k < FL - 1; k++) fr_sf[k] <= fr_sf[k + 1];
    fr_sf[FL - 1] <= 1'b0;
    if (f_iss && ex_ffast) fr_sf[LS - 1] <= sv_ok && ex_u.fop != 4'(FMA_RSQRTE);
    if (inj_fall) fr_sf[FDL - 2] <= 1'b0;
    if (d_go) div_sgl <= dq_sgl;
  end
  // FPSCR settled: no FPU result or compare flags still to come. Registered,
  // and conservative by a cycle when an FPU op in EX, or a compare in EX that
  // could leave (it waits for this itself), does not leave; but not while MEM
  // holds an op waiting for this (then nothing in EX can leave, and it would
  // wait for ever).
  logic m_c2;                                          // MEM holds the same uop as last cycle
  logic fs_idle;
  wire  mfp_hold = m.valid && m.fpser && !m_fsok;
  always_ff @(posedge clk)
    if (rst) fs_idle <= 1'b1;
    else     fs_idle <= (fr_at >> 1) == '0 && !div_out && !d_go &&
                        !(ex_valid && (ex_fma || ex_div || (ex_fcmp && fs_idle)) && !mfp_hold);

  // (d_busy is not needed: the divider is busy only inside div_out)
  // EX's FP wait, from few registered inputs (it goes through ex_hold to
  // the fetch queue's pop): the divider's block (div_out || d_go) and "an
  // older multiply-add op is still before S4b" as next-state registers, the
  // rarer instructions' waits as capture bits (ex_fsw, ex_fsx, ex_f2c).
  // ffbl_q: an FPU result is due LS or more cycles ahead, so a fast op waits
  logic fblk_q, fpre_q, ffbl_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      fblk_q <= 1'b0;
      fpre_q <= 1'b0;
      ffbl_q <= 1'b0;
    end else begin
      fblk_q <= (ex_div && fp_go) || d_go || (div_out && !inj_fall);
      fpre_q <= (fr_at_n & FPRE) != '0;
      ffbl_q <= (fr_at_n >> LS) != '0;
    end
  end
  // All of it but the fast op's term is registered as fwq, formed for both
  // cases of this cycle's EX hold: EX keeps its uop (no FP op leaves, ex_c2
  // and ex_fs2 are set) or takes ID's (EX's FP op leaves if it is not killed;
  // ID's controls are predecoded). The hold picks as the last level, so EX's
  // wait is one LUT after registers (it led the queue pop's paths, R63).
  // ex_valid is left out of fwq (EX's controls load with any uop) and ANDed
  // here. With FDL < 4 the WB slot's divider term stays out (wb_blk).
  logic fwq;
  wire  fw_go  = ex_valid && !m_kp && !ex_xcm;         // fp_go where EX advances
  wire  fw_bk  = d_go || (div_out && !inj_fall);       // fblk_q next cycle, no divide leaving EX
  wire  fw_ba  = (ex_div && fw_go) || fw_bk;           // ... a divide in EX leaving
  wire  [FL-1:0] fw_rk = (fr_at >> 1) | ({FL{inj_fall}} & (FL'(1) << (FDL - 2)));   // fr_at next, EX held
  wire  [FL-1:0] fw_ra = fw_rk | ({FL{ex_fma && fw_go && !ex_ffast}} & (FL'(1) << (FL - 1))) |
                                 ({FL{ex_fma && fw_go &&  ex_ffast}} & (FL'(1) << (LS - 1)));
  wire  fw_idl = (fr_at >> 1) == '0 && !div_out && !d_go &&
                 !(ex_valid && (ex_fma || ex_div || (ex_fcmp && fs_idle)) && !mfp_hold);   // fs_idle next
  wire  fwk_h   = (ex_fma && fw_bk) || (ex_div && (fw_bk || (fw_rk & FPRE) != '0)) || (ex_fsw && !fw_idl) ||
                 (ex_fsx && !(ex_valid && ex_fsop != 3'd0 && fs_idle)) || (ex_f2c && !ex_valid) ||
                 (ex_mw && fw_rk[2]);
  wire  fw_n   = (q_pd.ffma && fw_ba) || (q_pd.fdiv && (fw_ba || (fw_ra & FPRE) != '0)) ||
                 (((h_fser && !q_pd.ifault) || h_rfi || h_mtmsr) && !fw_idl) || q_pd.ffsx || q_pd.ff2c ||
                 (q_pd.fmw && fw_ra[2]);
  always_ff @(posedge clk) fwq <= ex_hold ? fwk_h : fw_n;
  always_comb
    fp_wait = ex_valid && (fwq || (ex_ffw && ffbl_q) || (ex_mw && wb_blk) || ex_fbd || ex_fbed);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && fwq != ((ex_fma && fblk_q) || (ex_div && (fblk_q || fpre_q)) || (ex_fsw && !fs_idle) ||
                                    (ex_fsx && !ex_fs2) || (ex_f2c && !ex_c2) || (ex_mw && fr_at[2])))
      $fatal(1, "fwq against EX's FP wait");
`endif

  // FPSCR: one update a cycle, from the FPU port, a compare in MEM or an FPSCR
  // instruction leaving EX; the waits above keep them apart
  wire        fs_cmp = m.valid && m.fscmp && !m_c2;     // in its first MEM cycle
  wire        fs_ex  = ex_valid && ex_fsop != 3'd0;
  // Two instances of the FPSCR logic: one applies a result's or a compare's
  // flags to fpscr_q, the other forms an FPSCR instruction's result for
  // fs_new_q (below), so EX's instruction fields are not in front of fpscr_q
  fpu_flags_t fs_flags;
  logic [31:0] fs_flg, fs_new;
  logic [3:0]  fs_cr1, fs_crf;
  assign fs_flags = fo_valid ? fo_flags : m.fflags;
  ppc_fpu_fpscr u_fpscr (
    .op (FS_FLAGS), .fpscr_in (fpscr_q), .flags (fs_flags), .fm (8'd0), .frb_lo (32'd0),
    .fld (3'd0), .imm (4'd0), .bitn (5'd0), .fpscr_out (fs_flg), .cr1 (), .crf ()
  );
  ppc_fpu_fpscr u_fpscr_x (
    .op (fs_op_t'(ex_fsop)), .fpscr_in (fpscr_q), .flags ('0), .fm (ex_u.insn[24:17]), .frb_lo (fb_q[31:0]),
    .fld (ex_fsfld), .imm (ex_u.insn[15:12]),
    .bitn (ex_u.insn[25:21]), .fpscr_out (fs_new), .cr1 (fs_cr1), .crf (fs_crf)
  );
  // An FPSCR instruction's result is registered in a cycle where the FPSCR
  // is settled (fs_idle: nothing else updates it then) and written from the
  // register as the instruction leaves EX, so the FPSCR logic is not in front
  // of fpscr_q behind EX's late leaving decision; it costs these rare
  // instructions a cycle (ex_fs2)
  logic [31:0] fs_new_q;
  logic [3:0]  fs_cr1_q, fs_crf_q;
  logic        ex_fs2;
  always_ff @(posedge clk) begin
    fs_new_q <= fs_new;
    fs_cr1_q <= fs_cr1;
    fs_crf_q <= fs_crf;
    if (rst) ex_fs2 <= 1'b0;
    else     ex_fs2 <= fs_ex && fs_idle && ex_hold;
    if (rst)                       fpscr_q <= 32'd0;                     // 603e table 4-9
    else if (fo_valid || fs_cmp)   fpscr_q <= fs_flg;
    else if (fs_ex && fp_go)       fpscr_q <= fs_new_q;
  end

  // FP double: step 0's EA, for step 1 after a replay. Not from a step 0 that
  // MEM kills as it leaves EX: when MEM's own step 1 replays, that is the next
  // instruction's, and step 1 resumes from this register
  always_ff @(posedge clk) begin
    if (ex_adv && ex_fd0 && !m_kp) fea_q <= r_sum;
    fea4_q <= fea_q + 32'd4;
    // step 0's EA + 4 from a replay that resumes at step 1 until the next
    // FP double's step 0 enters EX, then 4 again (step 1 behind it takes EX's
    // sum); nothing here waits on EX's late hold beyond its own enable
    if (m_kill && m.rp && !sl_flt && m.step == 5'd1)                       fimm_q <= fea4_q;
    else if (!ex_hold && id_u.fdbl && id_s0 && (id_u.fld || id_u.fst))     fimm_q <= 32'd4;
  end
  always_ff @(posedge clk) begin
    if (rst) ex_c2 <= 1'b0;
    else     ex_c2 <= ex_valid && ex_hold;
  end

`ifdef VERILATOR
  // the schedule's invariants, checked in simulation
  always_ff @(posedge clk)
    if (!rst) begin
      if (fo_valid != fr_at[0])          $fatal(1, "FPU result %0d against the schedule %0d", fo_valid, fr_at[0]);
      if (fo_valid && fo_tag != fr_ix[0]) $fatal(1, "FPU result's FPR %0d against the schedule's %0d", fo_tag, fr_ix[0]);
`ifdef R104_CHECK
      // FP operand bypass: a result predicted a single value is one, and a live bypass is the result
      // written now to the operand's FPR (off in the planted-fault runs, as R104's checks)
      if (fo_valid && fr_sv[0] && !fpu_sgl_val(fo_res)) $fatal(1, "FPU result %016X predicted a single value", fo_res);
      for (int p = 0; p < 3; p++)
        if (ex_valid && ex_fbl[p] && !(fo_valid && fo_tag == (p == 0 ? ex_u.insn[20:16] : p == 1 ? ex_u.insn[15:11] : ex_u.insn[10:6])))
          $fatal(1, "FP operand bypass %0d without its result (tag %0d)", p, fo_tag);
      if (ex_valid && ex_fbd != (ex_fma && !ex_ffast && ex_fbl != 3'b000)) $fatal(1, "ex_fbd");
      if (ex_valid && ex_fbd && !f_hold) $fatal(1, "ex_fbd without EX's hold");
      // FEF: an early-forwarded operand is the single unit's R5 now (its write next cycle, a single-unit op's
      // with OE = UE = 0), the uop is a single op of the fast kind, and the selects agree with the bypass's
      for (int p = 0; p < 3; p++) begin
        if (ex_valid && ex_fbe[p] && !(fr_at[1] && fr_sf[1] && fr_ix[1] == (p == 0 ? ex_u.insn[20:16] : p == 1 ? ex_u.insn[15:11] : ex_u.insn[10:6])))
          $fatal(1, "FP early forward %0d without its producer in R5 (fr_at %b fr_sf %b)", p, fr_at, fr_sf);
        if (ex_valid && ex_fbe[p] && !(ex_fma && ex_u.fsgl && ex_u.fop != 4'(FMA_RSQRTE) && ex_u.fop <= 4'(FMA_NMSUB)))
          $fatal(1, "FP early forward into a uop not of the fast kind");
        if (ex_fs[p] != (ex_fbe[p] ? 2'd3 : ex_fbl[p] ? 2'd2 : ex_fbh[p] ? 2'd1 : 2'd0))
          $fatal(1, "ex_fs[%0d] %0d against ex_fbe %b ex_fbl %b ex_fbh %b", p, ex_fs[p], ex_fbe, ex_fbl, ex_fbh);
      end
      if (ex_valid && ex_fbed != (ex_fma && !ex_ffast && ex_fbe != 3'b000)) $fatal(1, "ex_fbed");
      // (FWW) an FPR with a port write still to come is pending
      for (int k = 1; k < FL; k++)
        if (fr_at[k] && !pend[fr_ix[k]]) $fatal(1, "FPR %0d not pending with its port write %0d cycles ahead", fr_ix[k], k);
`endif
      if (fs_idle && !((fr_at == '0) && !div_out && !d_go && !(m.valid && m.fscmp && !m_c2))) $fatal(1, "fs_idle with FP updates outstanding");
      if (fo_valid && fw_wbp)            $fatal(1, "FPR write port: FPU and WB in one cycle");
`ifdef R104_CHECK
      // (FP load write slot; off in the planted-fault runs)
      if (int'(fo_valid) + int'(fw_wbp) + int'(fl_go) > 1) $fatal(1, "FPR write port: the slot and another writer in one cycle");
      if (fl_go && fl_sl && !fpu_sgl_val(fl_val)) $fatal(1, "FP load slot: lfs value %016X not a single value", fl_val);
`endif
      if (int'(fo_valid) + int'(fs_cmp) + int'(fs_ex && fp_go) > 1) $fatal(1, "two FPSCR updates in one cycle: fpu %0d cmp %0d ex %0d (op %0d, m.fpser %0d m_c2 %0d)", fo_valid, fs_cmp, fs_ex && fp_go, ex_fsop, m.fpser, m_c2);
      if (ex_fma && fp_go && d_inj_req && !inj_q) $fatal(1, "multiply-add issued as the divider asks to hand over");
      if (d_go && d_busy)                $fatal(1, "divider started while busy");
      if (fblk_q != (div_out || d_go))   $fatal(1, "fblk_q");
      if (fpre_q != ((fr_at & FPRE) != '0)) $fatal(1, "fpre_q");
      if (ffbl_q != ((fr_at >> LS) != '0)) $fatal(1, "ffbl_q");
      if (ex_ffw != ex_ffast || (ex_ffast && !ex_fma)) $fatal(1, "ex_ffw");
      // (with a bypassed operand the route is from its predicted single-value bit: fast only where
      // the values allow it, and not always where they do)
      if (ex_valid && ex_fma && !ex_ifault && ex_fbe == 3'b000 && (ex_fbu ? (ex_ffast && !rt_ref(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_x, fb_x, fc_x))
                                                                : ex_ffast != rt_ref(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_x, fb_x, fc_x)))
        $fatal(1, "fast route %0d for EX's operands", ex_ffast);
      if (rf_na != !fpu_sgl_val(rf_fa) || rf_nb != !fpu_sgl_val(rf_fb) || rf_nc != !fpu_sgl_val(rf_fc))
        $fatal(1, "FPR single-value bit against the value read");
      if (id_valid && {q_pd.ffma, q_pd.fdiv, q_pd.fmsc, q_pd.fmw, q_pd.ffsx, q_pd.ff2c} !=
          ({6{!q_pd.ifault}} & {id_u.funit == 2'd1, id_u.funit == 2'd2, id_u.funit == 2'd3,
                                id_u.funit == 2'd3 && id_u.fpw, id_u.fsop != 3'd0, id_u.fcmp || (id_u.fst && id_u.fsl)}))
        $fatal(1, "FP predecode of %08X against the decoder", q_hi);
      if (id_valid && !q_pd.ifault && id_u.funit == 2'd1 &&
          (q_pd.frq || q_pd.frs) != (id_u.fop == 4'(FMA_RSQRTE) ||
                                     (id_u.fsgl && (id_u.fop == 4'(FMA_ADD) || id_u.fop == 4'(FMA_SUB) ||
                                                    id_u.fop == 4'(FMA_MUL) || id_u.fop == 4'(FMA_MADD) ||
                                                    id_u.fop == 4'(FMA_MSUB) || id_u.fop == 4'(FMA_NMADD) ||
                                                    id_u.fop == 4'(FMA_NMSUB)))))
        $fatal(1, "fast route kind of %08X against the decoder", q_hi);
      if (id_valid && (q_pd.frq || q_pd.frs) && (q_pd.ifault || id_u.funit != 2'd1))
        $fatal(1, "fast route kind of %08X off the multiply-add port", q_hi);
      if (wb_live_q != (wb_isld && !wb_got)) $fatal(1, "wb_live_q");
      if (f_iss && ex_ffast && !f_fast_ok) $fatal(1, "fast op issued against the unit's own check");
      if (f_iss && ex_fbe == 3'b000 && (ex_fbu ? (ex_ffast && !rt_ref(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_x, fb_x, fc_x))
                                     : ex_ffast != rt_ref(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_x, fb_x, fc_x)))
        $fatal(1, "fast route %0d is not for EX's operands", ex_ffast);
    end
`endif

  // results and special-register updates
  //
  // FP integration point: FP loads and stores take the data path below as the
  // integer ones do (translation included; a 64-bit lfd/stfd is two word
  // accesses or a wider port); FP arithmetic joins as another multi-cycle
  // unit beside u_md, its FPR writes on a port of its own; MSR[FP] = 0 gives
  // the FP unavailable exception (0x800, 603e 4.5.8) in EX, where the fetch
  // fault markers and interrupts are taken (ex_xcm).
  // The result in two parts: the ALU's late groups, and everything early (the
  // ALU's early groups, multiply/divide, move-from), from registers through
  // two or three levels. For each operand register, the early part and the
  // capture from ID are muxed first, and the late part joins as the last LUT:
  // op = (ex & late) | (ex ? early : cap), one LUT of six inputs after the
  // rotators' last level and the adder (R23 timing). keep holds Quartus to
  // that split; left to itself it merged the result into a tree three levels
  // deep after the rotators and only then muxed the capture in.
  wire [31:0]                r_late  = r_sum | r_rl | r_rr | r_clz | mf_c0v;
  (* keep = 1 *) wire [31:0] r_mdv   = {32{ex_rsel[7]}} & md_val;
  (* keep = 1 *) wire [31:0] r_early = r_mid | r_mdv | mfg_a | mfg_b | mfg_c | mfg_d;   // (R140: mf_val's groups)
  // (LU2: the early word, aligned, or zero for a byte it does not give, beside the capture, and EX's own
  // forward off while an operand is reloaded; x_se ... from registers and the queue head, ahead of the data)
  wire        x_se = x_ex, y_se = y_ex, a_se = a_ex, s_se = s_ex;   // (R140: a reload chooses at op_*)
  (* keep = 1 *) logic [31:0] x_pre, y_pre, a_pre, s_pre;
  logic [31:0] x_fl, y_fl, a_fl, s_fl;
  always_comb
    for (int b = 0; b < 32; b++) begin
      x_pre[b] = x_se ? r_early[b] : x_le[b/8 ^ 3] ? (LU2_FL ? 1'b0 : e_al[b]) : x_lz[b/8 ^ 3] ? 1'b0 : x_cap[b];
      y_pre[b] = y_se ? r_early[b] : y_le[b/8 ^ 3] ? (LU2_FL ? 1'b0 : e_al[b]) : y_lz[b/8 ^ 3] ? 1'b0 : y_cap[b];
      a_pre[b] = a_se ? r_early[b] : a_le[b/8 ^ 3] ? (LU2_FL ? 1'b0 : e_al[b]) : a_lz[b/8 ^ 3] ? 1'b0 : a_cap[b];
      s_pre[b] = s_se ? r_early[b] : s_le[b/8 ^ 3] ? (LU2_FL ? 1'b0 : e_al[b]) : s_lz[b/8 ^ 3] ? 1'b0 : s_cap[b];
      // (LU2_FL: the early word's bits, gated, for the last LUT)
      x_fl[b] = LU2_FL && x_le[b/8 ^ 3] && e_al[b];
      y_fl[b] = LU2_FL && y_le[b/8 ^ 3] && e_al[b];
      a_fl[b] = LU2_FL && a_le[b/8 ^ 3] && e_al[b];
      s_fl[b] = LU2_FL && s_le[b/8 ^ 3] && e_al[b];
    end

  mem_t mx;
  // FP CR writes: a compare's field (as cmp's), mcrfs's field, CR1 for Rc = 1
  // (an arithmetic op's CR1 MEM fills in, m.fcr1)
  logic [31:0] fp_crv, fp_crm;
  always_comb begin
    fp_crv = 32'd0;
    fp_crm = 32'd0;
    if (ex_fcmp || ex_fsop == 3'(FS_MCRFS)) fp_crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
    if (ex_fsop == 3'(FS_MCRFS))            fp_crv[5'd31 - {ex_u.crf, 2'b00} -: 4] = fs_crf_q;
    if (ex_frc) begin
      fp_crm[27:24] = 4'hF;
      fp_crv[27:24] = fs_ex ? fs_cr1_q : fpscr_q[31:28];
    end
  end
  // CR values written, without the CR logic op's bit (cr_f), and that bit
  // (cr_lb): for mx.cra / mx.crb
  // Formed for both values of MEM's zero detect, as the branch's decision
  // (cra_z0 / cra_z1 above), m_z choosing last: a CR logic op reading CR0
  // from an Rc producer in MEM had the zero detect in front of its crop
  // lookup (a round-2 uncached full build, m.result into m.cr)
  wire         cna_z0 = ex_cra_s[0] ? m_cr0z0[2'd3 - ex_u.crba[1:0]] : cr_an;
  wire         cna_z1 = ex_cra_s[0] ? m_cr0z1[2'd3 - ex_u.crba[1:0]] : cr_an;
  wire         cnb_z0 = ex_crb_s[0] ? m_cr0z0[2'd3 - ex_u.crbb[1:0]] : cr_bn;
  wire         cnb_z1 = ex_crb_s[0] ? m_cr0z1[2'd3 - ex_u.crbb[1:0]] : cr_bn;
  wire         clb_z0 = ex_u.crop[{cna_z0, cnb_z0}];
  wire         clb_z1 = ex_u.crop[{cna_z1, cnb_z1}];
  wire         cr_lb = m_z ? clb_z1 : clb_z0;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && cr_lb != ex_u.crop[{cr_a, cr_b}]) $fatal(1, "cr_lb by m_z against crop");
`endif
  logic [3:0]  cr_f4;
  always_comb begin
    cr_f4 = 4'd0;
    case (ex_u.op)
      OP_CRLOG: cr_f4 = cr_in_n[5'd31 - {ex_u.crf_src, 2'b00} -: 4];    // mcrf (a bit op takes cr_lb)
      OP_MCRXR: cr_f4 = xer_in[31:28];
      OP_STORE: cr_f4 = {2'b00, res_in, xer_in[31]};                   // stwcx.: CR0
      default: ;
    endcase
    if (ex_fsop == 3'(FS_MCRFS)) cr_f4 = fs_crf_q;
    else if (ex_frc)             cr_f4 = fs_ex ? fs_cr1_q : fpscr_q[31:28];
  end
  logic [3:0]  c0;
  logic [31:0] res, xer_n, cr_n, msk;
  logic        so_n;
  always_comb begin
    msk        = 32'd0;
    mx         = '0;
    mx.valid   = ex_valid;
    mx.pc      = ex_u.pc;
    mx.insn    = ex_u.insn;
    mx.last    = ex_u.last;
    mx.rd      = ex_u.rd;
    mx.rd_we   = ex_u.rd_we;
    mx.is_load = is_ld;
    mx.req     = is_ld || (is_st && st_ok);
    mx.msize   = ex_u.msize;
    mx.boff    = boff;
    mx.sp0     = mis_hold;
    mx.sp1     = ex_sp1 && is_ld;
    mx.algd4   = ex_lsm;
    if (mis_hold) begin
      mx.rd_we = 1'b0;                                   // (an update store writes rA from the second)
      mx.last  = 1'b0;
    end
    mx.msigned = ex_u.msigned;
    mx.brev    = ex_u.brev;
    mx.spr     = ex_spr_idx;

    xer_n = xer_in;
    cr_n  = 32'd0;                                       // values of the bits written (mx.crm)
    mx.crm = 32'd0;

    // one-hot result select, registered in decode (R23 timing); the update
    // form of a store writes its EA (the sum) to rA. The ALU's groups and the
    // move-from value come gated; only multiply/divide is gated here.
    res = r_late | r_early;

    // XER: carry, overflow, summary overflow
    if (ex_u.set_ca) xer_n[29] = alu_ca;
    if (ex_u.oe) begin
      xer_n[30] = (ex_u.op == OP_MUL || ex_u.op == OP_DIV || ex_u.op == OP_DIVU) ? md_ovv : alu_ov;
      if (xer_n[30]) xer_n[31] = 1'b1;
    end
    so_n = xer_n[31];
    mx.xer_we = ex_u.set_ca || ex_u.oe;

    // CR
    c0 = 4'd0;
    if (ex_u.rc) begin
      mx.rc0   = 1'b1;                                   // CR0 formed in MEM (m_cr)
      mx.crm[31:28] = 4'hF;
      mx.cr_we = 1'b1;
    end
    case (ex_u.op)
      OP_CMP, OP_CMPL: begin
        mx.cmp_we = 1'b1;                                // field inserted in MEM (m_cr)
        mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
        mx.cmpv   = alu_cmp;
        mx.crf    = ex_u.crf;
        mx.cr_we  = 1'b1;
      end
      OP_CRLOG: begin
        if (ex_u.insn[10:1] == 10'd0)                        // mcrf
          begin
            // CR0 from an Rc producer one step ahead is taken in MEM, from the
            // committed CR, where that producer is by then: its zero detect
            // stays off EX's path
            cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4]   = cr_in_n[5'd31 - {ex_u.crf_src, 2'b00} -: 4];
            mx.mcrf0 = ex_mcrf0;
            mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
          end
        else
          begin
            cr_n[5'd31 - ex_u.crbd]   = cr_lb;
            mx.crm[5'd31 - ex_u.crbd] = 1'b1;
          end
        mx.cr_we = 1'b1;
      end
      OP_MTCRF: begin
        for (int f = 0; f < 8; f++) msk[31 - 4*f -: 4] = {4{ex_u.crm[7 - f]}};
        cr_n   = s_val;
        mx.crm = msk;
        mx.cr_we = 1'b1;
      end
      OP_MCRXR: begin
        cr_n[5'd31 - {ex_u.crf, 2'b00} -: 4]   = xer_in[31:28];
        mx.crm[5'd31 - {ex_u.crf, 2'b00} -: 4] = 4'hF;
        xer_n[31:28] = 4'd0;
        mx.cr_we = 1'b1;
        mx.xer_we = 1'b1;
      end
      OP_STORE: if (ex_u.cond) begin                       // stwcx.
        cr_n[31:28]   = {2'b00, res_in, xer_in[31]};
        mx.crm[31:28] = 4'hF;
        mx.cr_we = 1'b1;
      end
      default: ;
    endcase

    // floating point: a compare's field like cmp's (its value from the small
    // unit, in EX's second cycle); mcrfs's field; CR1 for Rc = 1, known here
    // except for an arithmetic op, whose CR1 MEM fills in (m.fcr1)
    mx.fpw    = ex_fpw && (ex_misc || is_ld);              // the WB writers: small unit, FP loads
    mx.fsl    = ex_u.fsl;
    mx.fhi    = ex_u.fhi && ex_fd0;
    mx.fscmp  = ex_fcmp;
    mx.fflags = ms_flags;
    mx.fpser  = ex_fpser;
    mx.fcr1   = ex_frc && (ex_fma || ex_div);
    mx.fsx    = fs_ex;
    mx.fany   = ex_fma || ex_div || ex_fcmp || fs_ex;
    // The FP CR values come from registers and are ORed in, not muxed after
    // the integer ones: cr_n's CR-logic bit and cmp's field are behind EX's
    // zero detect and compare (R27)
    mx.cmp_we = mx.cmp_we || ex_fcmp;
    mx.cmpv   = mx.cmpv | ({4{ex_fcmp}} & ms_cr);
    mx.crf    = ex_u.crf;
    mx.crm    = mx.crm | fp_crm;
    mx.cr_we  = mx.cr_we || fp_crm != 32'd0;
    cr_n      = cr_n | fp_crv;

    mx.result = res | ({32{ex_mfsr}} & fsr);             // mfsr(in): the sum is zero
    mx.cr     = cr_n;
    mx.xer    = xer_n & XER_MASK;
    mx.lr     = lr_in;
    mx.ctr    = ctr_in;
    mx.ctr1   = ctr1_in;
    mx.msr    = msr_q;

    // branch side effects. A bclr / bcctr the BTB predicted (ex_pred: the
    // static rule never predicts them) that is taken to another target than
    // the prediction's refetches from MEM (mx.rtm), from its target in sprv
    if (ex_u.op == OP_BRANCH) begin
      mx.sprv = br_target;
      mx.rtm  = BTB && ex_pred && (ex_u.bkind == BR_BCLR || ex_u.bkind == BR_BCCTR) && br_taken && !rt_eq;
      if (ex_u.lk) begin mx.lr = ex_u.pc + 32'd4; mx.lr_we = 1'b1; end
      if (ex_u.bkind != BR_B && ex_u.bkind != BR_BCCTR && !bo_noctr) begin
        mx.ctr = ctr_dec; mx.ctr1 = (ctr_in == 32'd2); mx.ctr_we = 1'b1;
      end
    end

    // mtspr
    if (ex_u.op == OP_MTSPR) begin
      case (ex_u.spr)
        10'd1:   begin mx.xer = s_val & XER_MASK; mx.xer_we = 1'b1; end
        10'd8:   begin mx.lr  = s_val;           mx.lr_we  = 1'b1; end
        10'd9:   begin mx.ctr = s_val;           mx.ctr1 = (s_val == 32'd1); mx.ctr_we = 1'b1; end
        10'd22:  begin mx.dec_we = 1'b1;         mx.sprv = s_val; end
        10'd284: begin mx.tb_we  = 1'b1;         mx.sprv = s_val; mx.tbu = 1'b0; end
        10'd285: begin mx.tb_we  = 1'b1;         mx.sprv = s_val; mx.tbu = 1'b1; end
        // HID0: ICFI and DCFI read 0 (table 2-2); as written they go in hcfi
        default: begin mx.spr_we = ex_w_spr;     mx.sprv = (ex_spr_idx == 6'd9) ? (s_val & ~32'h0000_0C00) : s_val;
                       mx.hcfi = {2{ex_spr_idx == 6'd9}} & s_val[11:10]; end
      endcase
    end
    // The new MSR's value follows from the opcode alone; only its write enable
    // waits on a trap's compare (R23 timing).
    // (by kinds registered in ID, not the opcode compare: R23 timing)
    if (ex_kmsr[0])      mx.msr = s_val & MSR_MASK;                                   // mtmsr
    else if (ex_kmsr[1]) mx.msr = (srr1_in & SRR1_KEEP) | (msr_q & 32'h0001_0000);    // rfi
    else if (ex_kmsr[2]) mx.msr = (msr_q & ~32'h0006_EF33) | {31'd0, msr_q[MSR_ILE]}; // sc, program
    if (ex_u.op == OP_MTMSR) mx.msr_we = 1'b1;

    // reservation
    mx.res_set = ex_u.reserve;
    mx.res_clr = ex_u.cond;

    // exceptions: sc and program (illegal, trap); rfi restores MSR
    // SRR0/SRR1 values do not depend on whether the trap fires; only `exc` does,
    // so they are not held behind the trap compare (R23 timing)
    mx.srr0 = (ex_u.op == OP_SC) ? ex_u.pc + 32'd4 : ex_u.pc;
    mx.srr1 = (msr_q & SRR1_KEEP) |
              ((ex_u.op == OP_ILLEGAL) ? (ex_u.priv ? 32'h0004_0000 : 32'h0008_0000) :
               (ex_u.op == OP_TRAP) ? 32'h0002_0000 : 32'd0);
    // tw / twi: the operands go to MEM, which compares them (R23 timing)
    mx.is_trap = (ex_u.op == OP_TRAP);
    mx.to      = ex_u.bo;
    mx.ta      = x_val;
    mx.tb      = y_val;
    if (ex_u.op == OP_SC || ex_u.op == OP_ILLEGAL) begin
      mx.exc    = 1'b1;
      mx.msr_we = 1'b1;
      mx.res_clr = 1'b0;
    end
    if (ex_u.op == OP_RFI) mx.msr_we = 1'b1;

    // segment and TLB instructions commit as MEM is left
    mx.srw   = ex_u.op == OP_MTSR;
    mx.tlbie = ex_u.op == OP_TLBIE;
    mx.tlbl  = ex_u.op == OP_TLBL;
    mx.ya    = op_y[31:12];
    if (ex_u.op == OP_MTSR) mx.sprv = s_val;
    mx.hsrc  = ex_hsrc;
    mx.st    = is_st;
    mx.step  = ex_step;
    // A write to the MMU's state (mtsr(in), tlbie, tlbld, tlbli, a BAT) with
    // translation on refetches what follows, from MEM, so every later access,
    // fetch included, translates with it. The architecture asks software for a
    // context-synchronizing instruction (PEM table 2-23); here it is implied,
    // and its cost is paid only with translation on.
    mx.resync = ex_mmuw && (msr_q[MSR_IR] || msr_q[MSR_DR]);

    // Exceptions in place of the uop, from EX's registered state: a fetch
    // fault marker (ITLB miss 0x1000, ISI 0x400) or an interrupt (external
    // 0x500, decrementer 0x900); or a BAT protection fault (DSI 0x300, from
    // the BAT compare). SRR0 is this instruction (4.5.3-4.5.14); SRR1[16-31]
    // the MSR; SRR1[0-15] per table 4-17 for a TLB miss (CR0 and the WAY are
    // added at commit, from the committed CR and replacement order), table 5-3
    // for ISI, else zero. The MSR per table 4-8, with TGPR set for the three
    // TLB misses. A page access the micro-TLB cannot serve becomes a replay
    // (mx.rp): MEM translates it, and either takes its exception or refetches.
    // A replay leaves EX as one flag (m.rp) beside the request it did not
    // make; everything else it needs is formed in MEM from registers, so the
    // compares here feed one flip-flop and the request.
    // an op that may set FPSCR[FEX]: its program exception, should precise
    // mode take one in MEM (SRR1[11], PEM table 6-16)
    if (ex_fsu) begin
      mx.srr1 = (msr_q & SRR1_KEEP) | 32'h0010_0000;
      mx.msr  = (msr_q & ~MSR_XCLR) | {31'd0, msr_q[MSR_ILE]};
      mx.xvec = 13'h0700;
    end
    mx.req  = mx.req && !ex_xcm && !algn;
    mx.rp   = xl_rp && !ex_xcm && !algn;
    // the alignment exception in place of the uop (its FPR write is dropped
    // in WB, and its scoreboard bit cleared, as a replay's)
    mx.algn = algn && !ex_xcm;
    if (algn && !ex_xcm) begin
      mx.last    = 1'b1;
      mx.rd_we   = 1'b0;
      mx.is_load = 1'b0;
      mx.sp0     = 1'b0;
      mx.sp1     = 1'b0;
    end
    mx.dz   = is_st && ex_u.cop == 3'd2 && !ex_xcm;
    mx.dbany = db_any;
    mx.dbwi = db_wi;
    mx.duwi = du_wi;
    mx.tch  = ex_u.cop == 3'd1;
    mx.icbi = ex_u.icbi && !ex_xcm;
    if (ex_xcm) begin
      mx.xc      = 1'b1;
      mx.last    = 1'b1;
      mx.rd_we   = 1'b0;
      mx.req     = 1'b0;
      mx.is_trap = 1'b0;
      mx.exc     = 1'b0;
      mx.srw     = 1'b0;
      mx.resync  = 1'b0;
      mx.rtm     = 1'b0;
      mx.tlbie   = 1'b0;
      mx.tlbl    = 1'b0;
      mx.fpw     = 1'b0;
      mx.fscmp   = 1'b0;
      mx.fpser   = 1'b0;
      mx.fcr1    = 1'b0;
      mx.fsx     = 1'b0;
      mx.fany    = 1'b0;
      mx.dz      = 1'b0;
      mx.icbi    = 1'b0;
      mx.srr0    = ex_u.pc;
      mx.msr     = (msr_q & ~MSR_XCLR) | {31'd0, msr_q[MSR_ILE]};
      mx.msr_we  = 1'b1;
      if (xc_fpx) begin                                // SRR0 is a later instruction: SRR1[15]
        mx.xvec = 13'h0700;
        mx.srr1 = (msr_q & SRR1_KEEP) | 32'h0011_0000;
      end else if (xc_irq) begin
        mx.irq  = {!int_q, int_q};
        mx.xvec = int_q ? 13'h0500 : 13'h0900;
        mx.srr1 = msr_q & SRR1_KEEP;
      end else if (ex_ifault && ifl_miss) begin
        mx.tmiss = 1'b1;
        mx.imiss = 1'b1;
        mx.xvec  = 13'h1000;
        mx.vsid  = ifl_vsid;
        mx.srr1  = (msr_q & SRR1_KEEP) | {12'd0, ifl_key, 1'b1, 18'd0};
        mx.msr[MSR_TGPR] = 1'b1;
      end else if (ex_ifault) begin
        mx.xvec = 13'h0400;
        mx.srr1 = (msr_q & SRR1_KEEP) | {3'd0, ifl_s34, 27'd0};
      end else begin                                   // FP unavailable
        mx.xvec = 13'h0800;
        mx.srr1 = msr_q & SRR1_KEEP;
      end
    end
    mx.fsm = mx.fpw && !mx.is_load;
    mx.fld = mx.fpw && mx.is_load;
    // what a reader one step behind takes from this uop's CR write, by kind
    // (registered selects, ex_cra_s[4:2], choose in its EX): a CR logic op's
    // bit, a field's four bits (the reader picks one: a 4:1), or for mtcrf
    // the reader's own bit, picked here from rS by ID's fields. R23's
    // m.cr[31 - crbA], a 32:1 select in the reader's EX, was in front of its
    // branch decision.
    mx.crlb = cr_lb;
    mx.crf4 = cr_f4;
    mx.mtca = s_val[5'd31 - id_cba];
    mx.mtcb = s_val[5'd31 - id_cbb];
  end

  // redirect from EX (younger instructions in ID and fetch are discarded)
  logic [31:0] exc_vec;
  always_comb begin
    exc_vec = msr_q[MSR_IP] ? 32'hFFF0_0000 : 32'd0;
    redirect_c    = 1'b0;
    br_go         = 1'b0;
    // b: a branch that falls through, or a redirect from MEM or WB. Which
    // one is chosen by registers: MEM can kill (m_kp) only where EX cannot
    // redirect (if MEM does not advance, EX holds), and a fired trap reaches
    // WB with EX empty (it waits in EX for MEM and WB to drain), so the late
    // kill decides only the 2-bit choice, not 32 address bits
    redirect_pc_b = wb_trap ? exc_vec | 32'h0700 :
                    m_kp    ? (m.resync ? m.pc + 32'd4 : m.rtm ? m.sprv : (m.rp && !sl_flt) ? m.pc : exc_vec | {19'd0, m_xvec}) :
                              ex_u.pc + 32'd4;
    redirect_sel  = 1'b1;
    if (ex_adv) begin
      if (ex_rk[0]) begin                              // a branch
        redirect_sel = br_taken;
        br_go        = 1'b1;                           // redirects on a mispredict (below)
      end
      // sc, illegal, trap, rfi, mtmsr, isync. A trap redirects to the next instruction from EX,
      // decided by the opcode alone; if it fired, MEM redirects again to the vector one cycle later
      // (below). The operand compare is then off the fetch path entirely (R23 timing). Nothing
      // younger can be in EX or ID by then: the first redirect flushed them. Traps do not appear in
      // VF3's census (R12).
      if (|ex_rk[4:1]) redirect_c = 1'b1;
      if (ex_xcm) begin                                // (a kill from MEM overrides it below)
        redirect_c = 1'b1; redirect_sel = 1'b1;
      end
    end
    // from MEM: a data translation fault, one cycle on, or the refetch after
    // an MMU write (its commit is this cycle, so the refetch, next cycle,
    // translates with it)
    if (m_kill || wb_trap) begin
      redirect_c   = 1'b1;
      redirect_sel = 1'b0;                             // redirect_pc_b
    end
  end

  // EX's redirect address, without ex_adv: redir_a_q (its only reader) is chosen only when EX
  // advanced (redir_k_q = 0 needs a redirect with redirect_sel set, which only an advancing EX makes),
  // so its value in any other cycle is never used. ex_valid -> ex_adv -> the 32-bit mux into
  // redir_a_q was a 110 MHz system build's worst core path (timing, study R104)
  // The exception's vector is formed here for both values of ex_s0ok (it decides only the
  // interrupt, xc_irq) and chosen by it last, so the register is one LUT from the 32-bit mux
  // (the 100 MHz build's ex_s0ok -> redir_a_q, -0.62 ns). The vectors are mx.xvec's (checked below).
  // (by ex_rk, one-hot: an AND-OR of the five sources, 0 for any other op)
  wire  [31:0] rp_case = ({32{ex_rk[0]}} & br_target) | ({32{ex_rk[1]}} & (exc_vec | 32'h0C00)) |
                         ({32{ex_rk[2]}} & (exc_vec | 32'h0700)) | ({32{ex_rk[3]}} & (ex_u.pc + 32'd4)) |
                         ({32{ex_rk[4]}} & {srr0_in[31:2], 2'b00});
  wire        xc_noirq = ex_ifault || xc_fpx || ex_fpun;           // ex_xcm without the interrupt
  wire        xc_irq0  = irq_req && ex_first;                      // xc_irq where ex_s0ok = 0
  wire [12:0] xv_noirq = xc_fpx ? 13'h0700 : (ex_ifault && ifl_miss) ? 13'h1000 : ex_ifault ? 13'h0400 : 13'h0800;
  wire [12:0] xv_s0    = xc_fpx ? 13'h0700 : xc_irq0 ? (int_q ? 13'h0500 : 13'h0900) : xv_noirq;
  (* keep = 1 *) wire [31:0] rp_s1 = xc_noirq ? (exc_vec | {19'd0, xv_noirq}) : rp_case;
  (* keep = 1 *) wire [31:0] rp_s0 = (xc_noirq || xc_irq0) ? (exc_vec | {19'd0, xv_s0}) : rp_case;
  assign redirect_pc = ex_s0ok ? rp_s1 : rp_s0;
`ifdef R104_CHECK
  always_ff @(posedge clk)
    if (!rst && redirect_pc != (ex_xcm ? (exc_vec | {19'd0, mx.xvec}) : rp_case)) $fatal(1, "redirect_pc by ex_s0ok");
  always_ff @(posedge clk)
    if (!rst && redir_q && redir_k_q == 2'd0 && !ra_adv_q) $fatal(1, "redir_a_q chosen without EX advancing");
  logic ra_adv_q;
  always_ff @(posedge clk) ra_adv_q <= ex_adv;
`endif

  // A mispredict from both values of MEM's zero detect, chosen by it last
  // (br_tk above); the redirect for fetch's registers is a copy of its own
  // (the redirect drives some 50 loads, 1.1 ns of the R63 batch-4 spike's
  // MEM result into redir_q path)
  wire br_mis = m_z ? (brt_z1 != ex_pred) : (brt_z0 != ex_pred);
  assign redirect = redirect_c || (br_go && br_mis);
`ifdef VERILATOR
  // a branch resolved at fetch (LA) was resolved on the CR and CTR it meets
  // here: nothing that could change them was in flight
  always_ff @(posedge clk)
    if (!rst && ex_valid && ex_adv && ex_u.op == OP_BRANCH && ex_lar && br_mis && !ex_xcm)
      $fatal(1, "branch at %08X resolved at fetch, mispredicted in EX", ex_u.pc);
`ifdef R104_CHECK
  // a register target fetch knew (LRA) is the register EX reads
  always_ff @(posedge clk)
    if (!rst && LRA && ex_valid && ex_adv && ex_u.op == OP_BRANCH && ex_pred && rp_lq && !ex_xcm &&
        (ex_u.bkind == BR_BCLR || ex_u.bkind == BR_BCCTR) && br_taken && !rt_eq)
      $fatal(1, "LRA: branch at %08X: the look-ahead's target %08X is not the register's", ex_u.pc, {rp_tgt, 2'b00});
`endif
`endif
  (* keep = 1 *) wire redirect_f = redirect_c || (br_go && br_mis);
  // fetch's own three registers (redir_q, redir_hard_q, redir_k_q) take the
  // redirect for both values of m_z and choose by it last: through
  // redirect_f's 58 loads, m.result's zero detect into redir_k_q led a
  // round-2 uncached full build
  (* keep = 1 *) wire rdf_z0 = redirect_c || (br_go && (brt_z0 != ex_pred));
  (* keep = 1 *) wire rdf_z1 = redirect_c || (br_go && (brt_z1 != ex_pred));
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && (m_z ? rdf_z1 : rdf_z0) != redirect) $fatal(1, "redirect by m_z");
`endif
  // fetch's three registers again, now formed for both values of EX's advance as well and chosen by
  // it (ex_valid && !ex_hold) and m_z last: WB's wb_got through ex_hold, the redirect and the choice
  // into redir_k_q was a 100 MHz system build's worst path, -1.11 ns (study R104). Where EX does not
  // advance only MEM's kill or a fired trap redirects (to redirect_pc_b).
  // (R108: the op's kind from ex_rk, registered, not a compare of ex_u.op; MEM's kill and a fired trap
  // (rd_kn, from WB's hold) chosen last: with either, the choice is b and there is a redirect, for
  // both values of EX's advance. WB's hold through m_kill into rc_a / sl_a and the choice was a
  // two-level detour into redir_k_q)
  wire              bg_a    = ex_rk[0];
  wire              rc_a    = (|ex_rk[4:1]) || ex_xcm;
  wire              rd_kn   = m_kill || wb_trap;
  (* keep = 1 *) wire       rdfa_z0 = rc_a || (bg_a && (brt_z0 != ex_pred));
  (* keep = 1 *) wire       rdfa_z1 = rc_a || (bg_a && (brt_z1 != ex_pred));
  // redirect_sel: a branch's taken bit (by m_z), else 1
  (* keep = 1 *) wire [1:0] rdka_z0 = !rdfa_z0 ? 2'd2 : (!bg_a || ex_xcm || brt_z0) ? 2'd0 : 2'd1;
  (* keep = 1 *) wire [1:0] rdka_z1 = !rdfa_z1 ? 2'd2 : (!bg_a || ex_xcm || brt_z1) ? 2'd0 : 2'd1;
  wire              ex_advx = ex_valid && !ex_hold;
  wire              rdf_x   = rd_kn || (ex_advx && (m_z ? rdfa_z1 : rdfa_z0));
  wire [1:0]        rdk_x   = rd_kn ? 2'd1 : ex_advx ? (m_z ? rdka_z1 : rdka_z0) : 2'd2;
`ifdef R104_CHECK
  always_ff @(posedge clk)
    if (!rst && (rdf_x != (m_z ? rdf_z1 : rdf_z0) ||
                 rdk_x != (m_z ? (!rdf_z1 ? 2'd2 : redirect_sel ? 2'd0 : 2'd1) : (!rdf_z0 ? 2'd2 : redirect_sel ? 2'd0 : 2'd1))))
      $fatal(1, "fetch's redirect registers by ex_adv");
`endif

  // the MSR that fetch continues under, loaded with every redirect (a trap's
  // first redirect, to the next instruction, keeps the MSR; a fired trap's
  // second brings its own)
  wire [31:0] redirect_msr = wb_trap ? wb_tmsr : m_kill ? m_msrx :
                             (ex_u.op == OP_TRAP && !ex_xcm) ? msr_q : mx.msr;
  // (R108) fetch's three MSR bits by the same choice as fetch's redirect registers (rdf_x): MEM's kill or
  // a fired trap (rd_kn) last, then EX's advance and m_z, each case's value formed beside them; EX's
  // exception (ex_xcm, from the FPSCR and the interrupt) is one level from them. Through redirect and
  // redirect_msr, fpscr_q and m.result reached f_ir / f_pr / f_tg four levels deep (a CPU subsystem
  // spike's fpscr_q -> f_pr). The values: MEM's or WB's MSR (fm_p); EX's exception's, which clears
  // IR, PR and TGPR and sets TGPR for an ITLB miss (fm_mx); else EX's new MSR, or a trap's msr_q (fm_n).
  wire [31:0] fm_xc  = (msr_q & ~MSR_XCLR) | {31'd0, msr_q[MSR_ILE]};
  wire [31:0] fm_mn  = ex_fsu ? fm_xc : ex_kmsr[0] ? s_val & MSR_MASK :
                       ex_kmsr[1] ? (srr1_in & SRR1_KEEP) | (msr_q & 32'h0001_0000) : ex_kmsr[2] ? fm_xc : msr_q;
  wire [31:0] fm_r   = (ex_u.op == OP_TRAP) ? msr_q : fm_mn;
  wire [2:0]  fm_q   = {f_tg, f_pr, f_ir};
  wire [2:0]  fm_p   = wb_trap ? {wb_tmsr[MSR_TGPR], wb_tmsr[MSR_PR], wb_tmsr[MSR_IR]}
                               : {m_msrx[MSR_TGPR], m_msrx[MSR_PR], m_msrx[MSR_IR]};
  wire [2:0]  fm_mx  = {fm_xc[MSR_TGPR] || (!xc_fpx && !xc_irq && ex_ifault && ifl_miss), fm_xc[MSR_PR], fm_xc[MSR_IR]};
  wire [2:0]  fm_n   = {fm_r[MSR_TGPR], fm_r[MSR_PR], fm_r[MSR_IR]};
  wire        rdfn_z0 = (|ex_rk[4:1]) || (bg_a && (brt_z0 != ex_pred));   // rdfa without ex_xcm
  wire        rdfn_z1 = (|ex_rk[4:1]) || (bg_a && (brt_z1 != ex_pred));
  (* keep = 1 *) wire [2:0] fm_u0 = ex_xcm ? fm_mx : rdfn_z0 ? fm_n : fm_q;
  (* keep = 1 *) wire [2:0] fm_u1 = ex_xcm ? fm_mx : rdfn_z1 ? fm_n : fm_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      f_ir <= 1'b0;
      f_pr <= 1'b0;
      f_tg <= 1'b0;
    end else if (rd_kn || ex_advx) begin
      {f_tg, f_pr, f_ir} <= rd_kn ? fm_p : m_z ? fm_u1 : fm_u0;
    end
  end
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ((rd_kn || ex_advx) ? (rd_kn ? fm_p : m_z ? fm_u1 : fm_u0) : fm_q) !=
                (redirect ? {redirect_msr[MSR_TGPR], redirect_msr[MSR_PR], redirect_msr[MSR_IR]} : fm_q))
      $fatal(1, "fetch's MSR bits by case against redirect_msr");
`endif


  // the operand registers: loaded as a uop enters EX, or (LU2) an operand's reload while EX holds
  always_ff @(posedge clk) begin
    if (!ex_hold || rl_x) op_x <= rl_x ? ld_val : (({32{x_se}} & r_late) | x_fl | x_pre);
    if (!ex_hold || rl_y) op_y <= rl_y ? ld_val : (({32{y_se}} & r_late) | y_fl | y_pre);
    if (!ex_hold || rl_a) op_a <= rl_a ? ld_val : (({32{a_se}} & r_late) | a_fl | a_pre);
    if (!ex_hold || rl_s) op_s <= rl_s ? ld_val : (({32{s_se}} & r_late) | s_fl | s_pre);
  end
  // LU2: EX's uop that took the early word, until it is confirmed (lu2_cf), reloaded (lu2_rl) or killed
  always_ff @(posedge clk)
    if (rst || !LU2) begin
      ex_lsw <= 1'b0; ex_lsf <= 1'b0; ex_lso <= 4'd0;
    end else if (!ex_hold) begin
      ex_lsw <= id_adv && (|lu2_ld);
      ex_lsf <= id_adv && (|lu2_ld);
      ex_lso <= lu2_ld;
    end else begin
      ex_lsf <= 1'b0;
      if (m_kill || lu2_cf || lu2_rl) ex_lsw <= 1'b0;
    end

  always_ff @(posedge clk) begin
    if (rst) begin
      ex_valid   <= 1'b0;
      ex_sp1     <= 1'b0;
      ex_s0ok    <= 1'b0;
      ex_md_busy <= 1'b0;
      md_have    <= 1'b0;
      md_drop    <= 1'b0;
    end else begin
      if (md_start) ex_md_busy <= 1'b1;
      if (md_done) begin
        ex_md_busy <= 1'b0;
        md_have    <= !md_drop;
        md_drop    <= 1'b0;
        md_res_q   <= md_res;
        md_ov_q    <= md_ov;
      end
      if (m_kill && ex_is_md) begin                    // killed: drop what it computes
        md_have <= 1'b0;
        if ((ex_md_busy || md_start) && !md_done) md_drop <= 1'b1;
      end
      if (ex_hold && m_kill) ex_valid <= 1'b0;         // a held uop killed from MEM leaves
      // the split's second word: from the cycle after its first (made, or
      // given up for a replay or exception) until the uop leaves EX
      if (!ex_hold || m_kill)                  ex_sp1 <= 1'b0;
      else if (mis_hold && !ex_hold_o && !m_kp) ex_sp1 <= 1'b1;
      if (!ex_hold || m_kill)                  ex_s0ok <= 1'b0;
      else if (mis_go && !m_kp)                ex_s0ok <= 1'b1;
      if (!ex_hold) begin
        md_have  <= 1'b0;
        ex_valid <= id_adv;
        // the uop's data loads whenever EX is free; only ex_valid waits on the
        // redirect decision (R23 timing)
        ex_u       <= id_u;
        ex_u.op    <= id_opf;                          // (= id_u.op)
        ex_rk      <= id_rk;
        ex_bd      <= q_hi[27] ? {{6{q_hi[25]}}, q_hi[25:2], 2'b00}            // b (18), else bc (16)
                               : {{16{q_hi[15]}}, q_hi[15:2], 2'b00};
        ex_pred    <= id_pred;
        ex_rsel    <= id_rsel;
        ex_ld      <= id_opf == OP_LOAD || h_mfsr;
        ex_ls      <= id_opf == OP_LOAD || id_opf == OP_STORE;
        ex_isync   <= h_isync;
        ex_md      <= id_rsel[7] && !q_pd.ifault;       // a marker's word is not run
        ex_first   <= id_s0;
        ex_lsm     <= q_pd.crack == 2'd2;                 // lmw / stmw
        ex_msw     <= id_mw && id_s0 && !q_pd.ifault;   // (an update load's second uop is its add)
        ex_msh     <= id_mh && id_s0 && !q_pd.ifault;
        ex_step    <= id_step;
        ex_ifault  <= q_pd.ifault;
        ex_bk      <= {q_pd.bslot, q_pd.bpp, q_pd.bsq};
        ex_lar     <= q_pd.lar && !q_pd.ifault;
        // FP: none of it for a fault marker
        ex_fma     <= q_pd.ffma;                       // (predecoded, a marker's cleared)
        ex_div     <= q_pd.fdiv;
        ex_misc    <= q_pd.fmsc;
        ex_fpw     <= id_u.fpw && !q_pd.ifault;
        ex_frw     <= id_u.fpw && !q_pd.ifault;
        ex_froh    <= 32'd1 << q_hi[25:21];             // frD (pend's set is gated by ex_frw)
        ex_fcmp    <= id_u.fcmp && !q_pd.ifault;
        ex_fsu     <= id_u.fsu && !q_pd.ifault;
        ex_frc     <= id_u.frc && !q_pd.ifault;
        // held in MEM until settled: anything that may change the FPSCR in
        // precise mode, and an arithmetic op with Rc = 1
        ex_fpser   <= !q_pd.ifault && ((id_u.fsu && (msr_q[11] || msr_q[8])) ||
                                       (id_u.frc && (id_u.funit == 2'd1 || id_u.funit == 2'd2)));
        ex_fsop    <= q_pd.ifault ? 3'd0 : id_u.fsop;
        ex_fst     <= id_u.fst && !q_pd.ifault;
        ex_fd0     <= id_u.fdbl && id_s0 && (id_u.fld || id_u.fst) && !q_pd.ifault;
        // MSR[FP] as committed: every MSR write redirects (603e 4.5.8)
        ex_fpun    <= id_u.fp && id_s0 && !msr_q[13] && !q_pd.ifault;
        ex_mfsr    <= h_mfsr;
        ex_mmuw    <= h_mmu || (q_pd.w_spr && id_spr_idx >= 6'd20 && id_spr_idx <= 6'd35);   // (w_spr: an mtspr)
        // writes an input of HASH1/HASH2: SDR1, DMISS, DCMP, IMISS, ICMP
        ex_hsrc    <= q_pd.w_spr && (id_spr_idx == 6'd8 || id_spr_idx == 6'd13 || id_spr_idx == 6'd14 ||
                                     id_spr_idx == 6'd17 || id_spr_idx == 6'd18);
        ex_mfv     <= id_mfv;
        mf_f       <= id_mf_f;
        mf4c       <= {32{id_mf_f[4]}} & mx.crm;           // (m.crm next cycle: EX moves on)
        mf4o       <= {32{id_mf_f[4]}} & ~mx.crm;
        mf_c0      <= q_pd.mfcr && ex_valid && ex_u.rc;
        mf_b       <= id_rsel[8] && id_mf_t == 3'd0 && id_mf_f == 5'd0;
        mf_t       <= id_mf_t;
        ex_ltt     <= id_ltt;
        ex_ext_h   <= id_ext_h;
        ex_spr_idx <= id_spr_idx;
        ex_w_cr    <= q_pd.w_cr || q_pd.fwcr;
        ex_w_xer   <= q_pd.w_xer;
        ex_w_lr    <= q_pd.w_lr;
        ex_w_ctr   <= q_pd.w_ctr;
        ex_w_spr   <= q_pd.w_spr;
        ex_ck      <= q_pd.ck | {2'b00, q_pd.fck1, 1'b0};
        ex_mask    <= id_mask;
        ex_kmsr    <= {h_kmsr2, h_rfi, h_mtmsr};
        ex_serc    <= h_kmsr2 || h_rfi || h_mtmsr || h_isync || q_pd.ifault ||
                      (id_u.fp && id_s0 && !msr_q[13] && !q_pd.ifault);   // as ex_kmsr, ex_isync, ex_ifault, ex_fpun
        ex_fsw     <= (h_fser && !q_pd.ifault) || h_rfi || h_mtmsr;
        ex_fsx     <= q_pd.ffsx;
        ex_f2c     <= q_pd.ff2c;
        ex_mw      <= q_pd.fmw;
        // (mcrfs from the raw word, not the decoder's fsop: the field is used
        // only by an FPSCR op, and of those only mcrfs has opcode 63, xo 64)
        ex_fsfld   <= (q_hi[31:26] == 6'd63 && q_hi[10:1] == 10'd64) ? q_hi[20:18] : q_hi[25:23];
        fs_cr      <= ex_valid && ex_w_cr;
        fs_rc0     <= ex_valid && ex_u.rc;
        // a bit of the field an mcrf of CR0 writes (its value taken in MEM,
        // mx.mcrf0) is that CR0's bit, known here: the Rc producer is in MEM
        ex_cra     <= id_a0 ? m_cr0[2'd3 - id_cba[1:0]] : id_cra;
        ex_crb     <= id_b0 ? m_cr0[2'd3 - id_cbb[1:0]] : id_crb;
        ex_cra_s   <= {ex_valid && ex_ck[2] && ex_u.crm[3'd7 - id_cba[4:2]] && !id_a0,
                       ex_valid && ((ex_ck[1] && ex_u.crf == id_cba[4:2]) || (ex_ck[3] && id_cba[4:2] == 3'd0)) && !id_a0,
                       ex_valid && ex_ck[0] && ex_u.crbd == id_cba && !id_a0,
                       ex_valid && mx.cmp_we && ex_u.crf == id_cba[4:2],
                       ex_valid && ex_u.rc && id_cba[4:2] == 3'd0};
        ex_crb_s   <= {ex_valid && ex_ck[2] && ex_u.crm[3'd7 - id_cbb[4:2]] && !id_b0,
                       ex_valid && ((ex_ck[1] && ex_u.crf == id_cbb[4:2]) || (ex_ck[3] && id_cbb[4:2] == 3'd0)) && !id_b0,
                       ex_valid && ex_ck[0] && ex_u.crbd == id_cbb && !id_b0,
                       ex_valid && mx.cmp_we && ex_u.crf == id_cbb[4:2],
                       ex_valid && ex_u.rc && id_cbb[4:2] == 3'd0};
        fs_xer     <= ex_valid && ex_w_xer;
        fs_lr      <= ex_valid && ex_w_lr;
        fs_ctr     <= ex_valid && ex_w_ctr;
        fs_srr0    <= ex_valid && ex_w_spr && ex_spr_idx == 6'd0;
        fs_srr1    <= ex_valid && ex_w_spr && ex_spr_idx == 6'd1;
      end else begin
        // Held: the operands are final and stay. The special registers'
        // selects drop once their producer in MEM commits.
        ex_mfv <= mf_now;
        if (mem_adv) begin
          mf_b  <= mf_b || mf_f != 5'd0 || mf_c0;
          mf_f  <= 5'd0;
          mf4c  <= 32'd0;
          mf4o  <= 32'd0;
          mf_c0 <= 1'b0;
        end
        // the producer in MEM commits when MEM advances; then read the register
        fs_cr   <= fs_cr   && !mem_adv;
        fs_rc0  <= fs_rc0  && !mem_adv;
        ex_cra  <= cr_a;
        ex_crb  <= cr_b;
        ex_cra_s <= ex_cra_s & {5{!mem_adv}};
        ex_crb_s <= ex_crb_s & {5{!mem_adv}};
        fs_xer  <= fs_xer  && !mem_adv;
        fs_lr   <= fs_lr   && !mem_adv;
        fs_ctr  <= fs_ctr  && !mem_adv;
        fs_srr0 <= fs_srr0 && !mem_adv;
        fs_srr1 <= fs_srr1 && !mem_adv;
      end
    end
  end

  // ====================================================================== MEM
  // A load or store does not wait here for its answer: WB does. MEM holds
  // while WB does; for a replay's translation; for a small-unit result that
  // would reach WB in a cycle the FPU port writes (it writes in WB, which
  // never holds it); and for an op held until the FP state is settled
  // (m.fpser), which leaves the cycle after fs_idle was seen.
  //
  // Answers, in request order: d_valid is WB's while WB waits for one, else
  // MEM's (an answer a cycle after the request, or behind a WB that held).
  assign m_iss = m.req && !m.rp;                        // MEM's uop made a request
  wire wb_out = wb_valid && wb_iss && !wb_got;
  wire m_out  = m.valid && m_iss && !m_got;
  wire ans_wb = d_valid && wb_out;
  // LU2 (see "load-use 2"): from registers
  assign lu2_wbo = wb_out;
  assign lu2_m   = LU2 && m.valid && m_lsp_q && !m_got;
  assign lu2_cf  = ex_lsf && ans_wb && d_eok;
  assign lsh     = LU2 && ex_lsw && !lu2_cf;
  assign lu2_rl  = LU2 && ex_lsw && ans_wb && !lu2_cf;
  wire ans_m  = d_valid && !wb_out && m_out;
  // an FP load leaves once its word has been here a cycle (lfs's conversion
  // is registered: wb_c2) and the FPU port is free (wb_fpok)
  // (a split load's second word holds WB the cycle it arrives, its merge
  // registered: wb_sp1)
  assign wb_hold  = wb_valid && ((wb_out && (!d_valid || wb_sp1)) || (wb_fl && !wb_flok));
  assign wb_adv   = wb_valid && !wb_hold;
  // icbi: its set to the I-cache, MEM held until it is taken
  logic m_ibdone;
  assign icbi_req = m.valid && m.icbi && !m_ibdone;
  assign icbi_set = m.result[11:5];
  // as four products of at most six inputs, then their OR (it feeds EX's
  // hold and the fetch queue's pop): a small-unit result (m.fsm) is never a
  // replay, so !m.rp is not needed with it
  // (R140) d_valid is MEM's hold's only late input, so it comes in last: mh_dw (WB's request waits for
  // its answer, from registers) and m_dvok (d_valid's whole part, one LUT from the D-cache's register); the
  // rest is mh_e, from registers. Every commit enable as MEM is left is then its own decode and m_okb from
  // registers, and m_dvok at the last level (the spike's d_valid_q -> mh_1 -> mem_hold -> iu_clr -> the
  // BAT decode -> bat_*'s enables, -1.67 ns at 120 MHz, was four levels after the register)
  (* keep = 1 *) wire mh_1 = m.valid && wb_valid && wb_iss && !wb_got && wb_sp1;
  (* keep = 1 *) wire mh_2 = m.valid && wb_valid && wb_fl && (FLW ? fl_v && (fr_at[0] || wb_blk) : !(wb_got && wb_c2 && !fr_at[0] && !wb_blk));
  (* keep = 1 *) wire mh_3 = m.valid && ((m.rp && !sl_done) || (m.fsm && (fr_at[1] || wb_blk)));
  (* keep = 1 *) wire mh_4 = m.valid && ((m.fpser && !m_fsok) || (m.icbi && !m_ibdone && !icbi_ack));
  (* keep = 1 *) wire mh_e = mh_1 || mh_2 || mh_3 || mh_4;
  (* keep = 1 *) wire mh_dw = wb_valid && wb_iss && !wb_got && !wb_sp1;
  (* keep = 1 *) wire m_dvok = !(wb_valid && wb_iss && !wb_got && !wb_sp1) || d_valid;   // (!mh_dw || d_valid, one LUT)
  assign mem_hold = mh_e || (m.valid && !m_dvok);
  // (R140) mh_2 / qh_2 with FLW: an FP load in WB (wb_fl) is never a small-unit result leaving WB (fw_wbs needs
  // !wb_isld), so fl_v && !fl_go is fl_v && (fr_at[0] || wb_blk), one LUT; through fl_go, WB's wb_fpw reached
  // MEM's commits six levels deep (c7_s1: wb_fpw -> du_pg, bat_brpn, fimm_q; -1.0 to -1.15 ns at 110 MHz)
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (R1C && !rst && FLW && wb_valid && wb_fl && (fl_v && !fl_go) != (fl_v && (fr_at[0] || wb_blk)))
      $fatal(1, "mh_2 without fl_go");
`endif
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && mem_hold != (m.valid && (wb_hold || (m.rp && !sl_done) || (m.icbi && !m_ibdone && !icbi_ack) ||
                                         (m.fpw && !m.is_load && !m.rp && (fr_at[1] || wb_blk)) || (m.fpser && !m_fsok))))
      $fatal(1, "mem_hold products");
`endif
  // the small unit's result is valid in MEM's first cycle only (its operand
  // register takes EX's next uop), so it is kept for a MEM that holds
  logic [63:0] ms_res_q;
  wire  [63:0] ms_res_m = m_c2 ? ms_res_q : ms_res;
  always_ff @(posedge clk) begin
    if (rst || mem_adv) m_ibdone <= 1'b0;
    else if (icbi_ack)  m_ibdone <= 1'b1;
    if (rst || mem_adv) begin
      m_got  <= 1'b0;
      m_fsok <= 1'b0;
      m_fpxr <= 1'b0;
    end else begin
      if (ans_m) m_got <= 1'b1;
      if (m.valid && m.fpser && fs_idle) begin
        m_fsok <= 1'b1;
        m_fpxr <= fpx_cond;
      end
    end
    if (rst) m_c2 <= 1'b0;
    else     m_c2 <= m.valid && mem_hold;
    if (ans_m) m_dq <= d_rdata;
    if (!m_c2) ms_res_q <= ms_res;
    if (rst || mem_adv) m_dly <= 11'd0;
    else if (m.valid && m_dly != 11'h7FF) m_dly <= m_dly + 11'd1;
  end

  // tw / twi decided here, on registered operands; a fired trap redirects to
  // 0x700 from MEM and commits like any other exception
  wire m_trap = m.is_trap && (
                  (m.to[4] && $signed(m.ta) <  $signed(m.tb)) || (m.to[3] && $signed(m.ta) > $signed(m.tb)) ||
                  (m.to[2] && m.ta == m.tb) || (m.to[1] && m.ta < m.tb) || (m.to[0] && m.ta > m.tb));
  wire m_exc  = m.exc;                                  // sc / illegal commit at MEM exit

  // A fired trap commits and redirects from WB, one cycle on, from registered
  // state (R23 timing): nothing younger can have issued, since EX already
  // redirected the trap to its next instruction.
  logic        wb_trap;
  logic [31:0] wb_tsrr0, wb_tsrr1, wb_tmsr;
  always_ff @(posedge clk) begin
    if (rst) wb_trap <= 1'b0;
    else     wb_trap <= mem_adv && m_trap;
    wb_tsrr0 <= m.srr0;
    wb_tsrr1 <= m.srr1;
    wb_tmsr  <= m.msr;
  end
  (* keep = 1 *) wire m_advb = m.valid && !mh_e;
  assign mem_adv  = m_advb && m_dvok;

  // a load's word to its value: lane, byte reversal, extension
  // (bo: the byte offset; a halfword at 1 is bytes 1-2, inside its word)
  function automatic logic [31:0] ld_align(input logic [31:0] w, input logic [1:0] sz, input logic [1:0] bo,
                                           input logic sx, input logic rv);
    logic [15:0] h;
    logic [7:0]  b;
    h = (bo == 2'd0) ? w[31:16] : (bo == 2'd1) ? w[23:8] : w[15:0];
    if (rv) h = {h[7:0], h[15:8]};
    b = w[5'd31 - {bo, 3'b000} -: 8];
    case (sz)
      2'd2:    ld_align = rv ? {w[7:0], w[15:8], w[23:16], w[31:24]} : w;
      2'd1:    ld_align = sx ? {{16{h[15]}}, h} : {16'd0, h};
      default: ld_align = {24'd0, b};
    endcase
  endfunction
  // a split load's value from its two words: bytes bo .. bo + size - 1 of
  // {w0, w1}, then byte reversal and extension
  function automatic logic [31:0] ld_mrg(input logic [31:0] w0, input logic [31:0] w1, input logic [1:0] sz,
                                         input logic [1:0] bo, input logic sx, input logic rv);
    logic [63:0] d;
    logic [31:0] v;
    logic [15:0] h;
    d = {w0, w1} << {bo, 3'b000};
    v = d[63:32];
    h = rv ? {v[23:16], v[31:24]} : v[31:16];
    ld_mrg = (sz == 2'd2) ? (rv ? {v[7:0], v[15:8], v[23:16], v[31:24]} : v) :
             (sz == 2'd1) ? (sx ? {{16{h[15]}}, h} : {16'd0, h}) : {24'd0, v[31:24]};
  endfunction
  logic [31:0] mis_w0;                                 // a split load's first word
  // WB's, on the forward path: the word straight from the port
  // (as an aligner, R108: each result byte one byte of the port's word by a select of WB's registers,
  // then zero or the sign, so the port's word is one LUT from a data bit of ld_val)
  function automatic logic [1:0] la_ix(input logic [1:0] r, input logic [1:0] sz, input logic [1:0] bo,
                                       input logic rv);
    logic [1:0] hb;                                    // the halfword's first byte: bo 0, 1, else 2
    hb = bo[1] ? 2'd2 : bo;
    case (sz)
      2'd2:    la_ix = rv ? 2'd3 - r : r;
      2'd1:    la_ix = hb + ((r[0] ^ rv) ? 2'd1 : 2'd0);
      default: la_ix = bo;
    endcase
  endfunction
  // (the byte selects, data and sign bits registered beside WB's fields, so wb_msize is not in front
  // of the port's word: la_jq, la_dq, la_sq, loaded as wb_msize is)
  logic [3:0][1:0] la_jq;
  logic [3:0]      la_dq;
  logic            la_sq;
  logic [3:0][7:0] la_dat;
  genvar lar;
  generate
  for (lar = 0; lar < 4; lar = lar + 1) begin : g_la
    wire [1:0] j = la_jq[lar];
    assign la_dat[lar] = d_rdata[31 - 8*j -: 8];
  end
  endgenerate
  wire         la_sgn = la_sq && la_dat[2][7];
  assign ld_val = {la_dq[0] ? la_dat[0] : {8{la_sgn}}, la_dq[1] ? la_dat[1] : {8{la_sgn}},
                   la_dq[2] ? la_dat[2] : 8'd0,        la_dq[3] ? la_dat[3] : 8'd0};
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && wb_valid && ld_val != ld_align(d_rdata, wb_msize, wb_boff, wb_msigned, wb_brev))   // (fields loaded)
      $fatal(1, "WB's aligner against ld_align: %08X %08X", ld_val, ld_align(d_rdata, wb_msize, wb_boff, wb_msigned, wb_brev));
`endif
  // MEM's, for an answer that came while the uop was in MEM: into wb_result
  // (the first word as it leaves WB, in the cycle it does: the second's answer
  // may come in that cycle, while it is in MEM)
  wire  [31:0] mis_w0_m = (wb_adv && wb_sp0) ? ldw : mis_w0;
  // One merge serves MEM's early answer (a split's second word, or any load:
  // its word twice) and WB's split second word arriving there (mg_wb): WB
  // then holds, so MEM's uop is not taking its value in that cycle.
  wire         mg_wb = wb_valid && wb_sp1 && !wb_got;
  wire  [31:0] mg_w1 = mg_wb ? d_rdata : (m_got ? m_dq : d_rdata);
  wire  [31:0] mg_w0 = mg_wb ? mis_w0 : m.sp1 ? mis_w0_m : mg_w1;
  wire  [1:0]  mg_sz = mg_wb ? wb_msize : m.msize;
  wire  [1:0]  mg_bo = mg_wb ? wb_boff : m.boff;
  wire         mg_sx = mg_wb ? wb_msigned : m.msigned;
  wire         mg_rv = mg_wb ? wb_brev : m.brev;
  // The merge as a load aligner (R108): each result byte is one byte of one of four words (the port's
  // d_rdata, m_dq, wb_dq, mis_w0), chosen by selects formed from registers, d_valid only choosing
  // between the port and mis_w0 for a split's first word still in WB; then zero or the sign. ld_mrg
  // on mg_w0 / mg_w1 put the word muxes, the 64-bit shift and the extension in line, seven levels
  // from the port into wb_result (the 100 MHz release build's d_rdata and d_valid -> wb_result).
  // (wb_adv && wb_sp0 is wb_valid && wb_sp0 && (wb_got || d_valid): a split's first word is an
  // integer access that made its request, asserted below)
  wire         mg_sp = !mg_wb && m.sp1;                // w0 is a split's first word
  wire         mg_hd = wb_valid && wb_sp0;             // ... and it is in WB
  wire         mg_1q = !mg_wb && m_got;                // w1 is m_dq, else the port's word
  // a result byte's source byte in {w0, w1} (0-7) for data, else zero or the sign (byte 2's top bit)
  function automatic logic [2:0] mg_ix(input logic [1:0] r, input logic [1:0] sz, input logic [1:0] bo,
                                       input logic rv);
    case (sz)
      2'd2:    mg_ix = {1'b0, rv ? 2'd3 - r : r} + {1'b0, bo};
      2'd1:    mg_ix = {1'b0, (r[0] ^ rv) ? 2'd1 : 2'd0} + {1'b0, bo};
      default: mg_ix = {1'b0, bo};
    endcase
  endfunction
  function automatic logic mg_isd(input logic [1:0] r, input logic [1:0] sz);
    mg_isd = sz == 2'd2 || (sz == 2'd1 && r[1]) || r == 2'd3;
  endfunction
  logic [3:0][7:0] mg_dat;                             // [r]: result byte r (0: bits 31:24), as data
  logic [3:0][1:0] mg_ws;                              // its word: 0 d_rdata, 1 m_dq, 2 wb_dq, 3 mis_w0
  logic [3:0][2:0] mg_bx;
  logic [3:0][2:0] mbx_q, wbx_q;                       // mg_ix of MEM's fields, of WB's
  (* keep = 1 *) wire [31:0] mg_dp;                    // the port's byte for each result byte
  genvar mgr;
  generate
  for (mgr = 0; mgr < 4; mgr = mgr + 1) begin : g_mg
    assign mg_bx[mgr] = mg_wb ? wbx_q[mgr] : mbx_q[mgr];   // (registered beside m and WB's fields)
    wire [1:0] j = mg_bx[mgr][1:0];
    wire [1:0] ws1 = mg_1q ? 2'd1 : 2'd0;              // w1's word
    // (a byte of w0 when the index is below 4)
    (* keep = 1 *) wire [1:0] ws = mg_bx[mgr][2] ? ws1 :
                                    mg_wb ? 2'd3 : !mg_sp ? ws1 : !mg_hd ? 2'd3 : wb_got ? 2'd2 :
                                    d_valid ? 2'd0 : 2'd3;
    assign mg_ws[mgr] = ws;
    assign mg_dp[31 - 8*mgr -: 8] = d_rdata[31 - 8*j -: 8];
    wire [7:0] bq = m_dq[31 - 8*j -: 8], bw = wb_dq[31 - 8*j -: 8], b0 = mis_w0[31 - 8*j -: 8];
    assign mg_dat[mgr] = ws == 2'd0 ? mg_dp[31 - 8*mgr -: 8] : ws == 2'd1 ? bq : ws == 2'd2 ? bw : b0;
  end
  endgenerate
  wire         mg_sgn = mg_sz == 2'd1 && mg_sx && mg_dat[2][7];
  wire  [31:0] ld_mg = {mg_isd(2'd0, mg_sz) ? mg_dat[0] : {8{mg_sgn}}, mg_isd(2'd1, mg_sz) ? mg_dat[1] : {8{mg_sgn}},
                        mg_isd(2'd2, mg_sz) ? mg_dat[2] : 8'd0,        mg_isd(2'd3, mg_sz) ? mg_dat[3] : 8'd0};
  assign ld_val_m = ld_mg;
`ifdef VERILATOR
  always_ff @(posedge clk) begin
    for (int r = 0; r < 4; r++)
      if (!rst && mg_bx[r] != mg_ix(r[1:0], mg_sz, mg_bo, mg_rv)) $fatal(1, "the merge's registered byte index");
    if (!rst && ld_mg != ld_mrg(mg_w0, mg_w1, mg_sz, mg_bo, mg_sx, mg_rv))
      $fatal(1, "the merge as an aligner against ld_mrg: %08X %08X", ld_mg, ld_mrg(mg_w0, mg_w1, mg_sz, mg_bo, mg_sx, mg_rv));
    if (!rst && wb_valid && wb_sp0 && (!wb_iss || wb_fl || wb_sp1))
      $fatal(1, "a split's first word in WB without its request, or an FP load");
  end
`endif
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && !mg_wb && m.valid && m.is_load && m.req && !m.rp && !m.xc && !m.sp1 && !m.sp0 &&
        ld_mg != ld_align(m_got ? m_dq : d_rdata, m.msize, m.boff, m.msigned, m.brev))
      $fatal(1, "the shared merge against ld_align for an unsplit load: %08X %08X sz %0d bo %0d rv %b w0 %08X w1 %08X", ld_mg, ld_align(m_got ? m_dq : d_rdata, m.msize, m.boff, m.msigned, m.brev), m.msize, m.boff, m.brev, mg_w0, mg_w1);
`endif

  always_ff @(posedge clk) begin
    if (rst) begin
      m.valid <= 1'b0;
    end else if (!mem_hold) begin
      m <= mx;
      m_xpg <= ea_x[31:12];
      m_x4  <= ex_s0ok;
      for (int r = 0; r < 4; r++) mbx_q[r] <= mg_ix(r[1:0], mx.msize, mx.boff, mx.brev);
      for (int r = 0; r < 4; r++) e_jq[r] <= la_ix(r[1:0], mx.msize, mx.boff, mx.brev);
      for (int r = 0; r < 4; r++) e_dq[r] <= mg_isd(r[1:0], mx.msize);
      m_lsp_q <= LU2 && d_req && !d_we && d_op == 3'd0 && mx.is_load && mx.rd_we && !mx.fpw && !mx.sp0 && !mx.sp1 &&
                 !(mx.msize == 2'd1 && mx.msigned) &&
                 (LU2_AL || (!mx.brev && (mx.msize == 2'd2 || (mx.msize == 2'd1 && mx.boff == 2'd2) || (mx.msize == 2'd0 && mx.boff == 2'd3))));
      m.valid <= mv_a || (mv_b && !xl_rp);              // ((ex_adv || mis_go) && !m_kp, R140)
    end
    if (!mem_hold) m_rpq <= mx.rp;
    // CR1 as the FPSCR settles, for an arithmetic op with Rc = 1 held in MEM:
    // chosen by registers (the op is held until m_fsok), not by mem_hold
    if (m_c1w) begin
      m.cr[27:24] <= fpscr_q[31:28];
      m.crf4      <= fpscr_q[31:28];
    end
  end

  // ------------------------------------------- MEM: page translation (replay)
  // The 603e's page translation for an access the micro-TLB could not serve
  // (5.4.3, figures 5-6, 5-7), from the registered EA: the segment register
  // EA[0-3] selects, then V, the VSID and EA[4-14] against both entries of the
  // set EA[15-19] selects. Direct-store segment: DSI, DSISR[5]; protection
  // (PEM table 7-10: key 0 reads all and writes all but PP = 11, key 1 reads
  // all but PP = 00 and writes only PP = 10): DSI, DSISR[4]; no entry: DTLB
  // miss on load or store; a store to an entry with C = 0: DTLB miss on store
  // (table 5-7), behind protection (table 4-3, priorities 10 and 11). Else the
  // translation fills the micro-TLB and the instruction is refetched.
  //
  // Two cycles: MEM holds the replay one cycle while the lookup's outcome is
  // registered (sl_*_q), then acts on it, so the arrays and compares feed
  // flip-flops only.
  // (a load's or store's EA, or for a split's second word that page's first byte; the page from EX's ea_x,
  // m_xpg, and DMISS / DAR from m_xea)
  wire [31:0] sl_ea   = {m_xpg, m.result[11:0]};
  wire [31:0] m_xea   = {m_xpg, m.result[11:0] & {12{!m_x4}}};
  wire [31:0] sl_sr   = dsr_mem[sl_ea[31:28]];
  wire [62:0] sl_t0   = dt0_q;
  wire [62:0] sl_t1   = dt1_q;
  wire        sl_h0   = sl_t0[62] && sl_t0[61:38] == sl_sr[23:0] && sl_t0[37:27] == sl_ea[27:17];
  wire        sl_h1   = sl_t1[62] && sl_t1[61:38] == sl_sr[23:0] && sl_t1[37:27] == sl_ea[27:17];
  wire        sl_hit  = sl_h0 || sl_h1;
  wire [62:0] sl_e    = sl_h0 ? sl_t0 : sl_t1;         // a double hit takes way 0
  wire        sl_key  = msr_q[MSR_PR] ? sl_sr[29] : sl_sr[30];
  wire        sl_t    = sl_sr[31];
  wire        sl_rok  = !sl_key || sl_e[1:0] != 2'd0;
  wire        sl_wpp  = sl_key ? sl_e[1:0] == 2'd2 : sl_e[1:0] != 2'd3;
  // a DBAT hit here is a BAT protection fault: one that allowed the access
  // would have served it in EX (DSI, DSISR[4]; the BAT outranks the page path)
  logic [3:0] sl_bh;
  always_comb
    for (int k = 0; k < 4; k++)
      sl_bh[k] = bat_match(sl_ea[31:17], bat_bepi[4 + k], bat_bl[4 + k], msr_q[MSR_PR] ? bat_vp[4 + k] : bat_vs[4 + k]);
  wire        sl_bat  = |sl_bh;
  wire        sl_prot = sl_bat || (!sl_t && sl_hit && (m.st ? !sl_wpp : !sl_rok));
  // The registered outcomes below from each way's own fields, the hit
  // choosing last: through sl_e (the hit's mux of the entry) the protection
  // and C terms sat behind the tag compare, and the DTLB read address into
  // sl_tm_q / sl_dsi_q led a D-36 cached full build. pd_w: the access is
  // refused by way w's PP under the key; ok_w: a store may use way w (PP,
  // key, C), as sl_w_q.
  function automatic logic sl_pf(input logic [1:0] pp, input logic key, input logic st);
    sl_pf = st ? !(key ? pp == 2'd2 : pp != 2'd3) : !(!key || pp != 2'd0);
  endfunction
  wire        sl_pd0  = sl_pf(sl_t0[1:0], sl_key, m.st);
  wire        sl_pd1  = sl_pf(sl_t1[1:0], sl_key, m.st);
  wire        sl_cs0  = m.st && !sl_pd0 && !sl_t0[6];    // a store to an allowed entry with C = 0
  wire        sl_cs1  = m.st && !sl_pd1 && !sl_t1[6];
  wire        sl_prw  = sl_h0 ? sl_pd0 : sl_h1 && sl_pd1;   // the hit's PP refuses it
  wire        sl_csw  = sl_h0 ? sl_cs0 : sl_h1 ? sl_cs1 : 1'b1;   // a miss, or C = 0 on a store
  wire        sl_dsi_n = sl_bat || (!sl_t ? sl_prw : 1'b1);
  wire        sl_tm_n  = !sl_bat && !sl_t && sl_csw;
  wire        sl_prot_n = sl_bat || (!sl_t && sl_prw);
  function automatic logic sl_wf(input logic [1:0] pp, input logic key, input logic c);
    sl_wf = (key ? pp == 2'd2 : pp != 2'd3) && c;
  endfunction
  wire        sl_w_n   = sl_h0 ? sl_wf(sl_t0[1:0], sl_key, sl_t0[6]) : sl_wf(sl_t1[1:0], sl_key, sl_t1[6]);
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst &&
        (sl_dsi_n != ((!sl_bat && sl_t) || sl_prot) ||
         sl_tm_n != (!sl_bat && !sl_t && (!sl_hit || (m.st && !sl_prot && !sl_e[6]))) ||
         sl_prot_n != sl_prot || sl_w_n != (sl_wpp && sl_e[6])))
      $fatal(1, "slow translation's per-way outcome against the hit's entry");
`endif
  logic       sl_done, sl_dsi_q, sl_tm_q, sl_prot_q, sl_t_q, sl_key_q, sl_way_q, sl_w_q;
  logic [19:0] sl_pa0_q, sl_pa1_q;                      // both ways' RPN, chosen at the fill
  logic [3:0]  sl_wimg0_q, sl_wimg1_q;                  // and WIMG
  logic        sl_h0_q;
  logic [23:0] sl_vsid_q;
  always_ff @(posedge clk) begin
    if (rst || mem_adv) sl_done <= 1'b0;
    else if (m.valid && m.rp && !dtw_q) sl_done <= 1'b1;   // (a DTLB write on the edge into it: read again)
    if (!sl_done) begin
      sl_t_q    <= !sl_bat && sl_t;
      sl_prot_q <= sl_prot_n;
      sl_dsi_q  <= sl_dsi_n;
      sl_tm_q   <= sl_tm_n;
      sl_key_q  <= sl_key;
      // SRR1[WAY]: replacement order, or for a store that hit an entry with
      // C = 0 the way that hit, so the handler's tlbld replaces it
      sl_way_q  <= (sl_hit && m.st) ? !sl_h0 : dlru[sl_ea[16:12]];
      sl_w_q    <= sl_w_n;                  // a store may use it: PP, key, C
      sl_pa0_q  <= sl_t0[26:7];
      sl_pa1_q  <= sl_t1[26:7];
      sl_wimg0_q <= sl_t0[5:2];
      sl_wimg1_q <= sl_t1[5:2];
      sl_h0_q   <= sl_h0;
      sl_vsid_q <= sl_sr[23:0];
    end
  end
  wire        sl_flt  = m.rp && sl_done && (sl_dsi_q || sl_tm_q) && !m.tch;
  // dcbt / dcbtst whose translation faults: a no-op (603e 3.7.2)
  assign      tnop    = m.rp && sl_done && (sl_dsi_q || sl_tm_q) && m.tch;

  // what commits for an exception, from EX or from MEM's translation
  wire        m_noef  = m.xc || m.rp || m_algc;        // in place of the uop: none of its own effects
  wire        m_fpxk  = m.valid && m_fpxr;
  wire        m_alg   = m.valid && m_algc;             // alignment (dcbz), 4.5.6
  wire        m_xcp   = m_exc || m.xc || sl_flt || m_fpxk || m_alg;
  wire        m_tmiss = m.rp ? sl_flt && sl_tm_q : m.xc && m.tmiss;
  wire        m_dsi   = m.rp ? sl_flt && sl_dsi_q : m.xc && m.dsi;
  wire [2:0]  m_dsb   = m.rp ? {sl_prot_q, sl_t_q, m.st} : m.dsb;
  wire [23:0] m_vsid  = m.rp ? sl_vsid_q : m.vsid;
  wire [12:0] m_xvec  = m_algc ? 13'h0600 : !m.rp ? m.xvec : sl_dsi_q ? 13'h0300 : m.st ? 13'h1200 : 13'h1100;
  wire [31:0] m_exmsr = (msr_q & ~MSR_XCLR) | {14'd0, sl_flt && sl_tm_q, 16'd0, msr_q[MSR_ILE]};
  wire [31:0] m_msrx  = (sl_flt || m_algc) ? m_exmsr : m.rp ? msr_q : m.msr;
  wire        m_s1tm  = m.rp && sl_tm_q;                // SRR1 per table 4-17
  wire [31:0] m_srr1x = m.rp ? (msr_q & SRR1_KEEP) |
                                         {12'd0, m_s1tm && sl_key_q, 1'b0, m_s1tm && sl_way_q, m_s1tm && m.st, 16'd0}
                               : m.srr1;
  // a TLB miss's SRR1 gets CR0 (table 4-17) from the committed CR, and an
  // ITLB miss its WAY from the committed replacement order, so both include
  // everything older
  wire [31:0] m_srr1c = m_srr1x | {m_tmiss ? cr_q[31:28] : 4'd0, 10'd0, m.imiss && ilru[m.pc[16:12]], 17'd0};

  // commit of special registers as the instruction leaves MEM; an exception or
  // a replay in a uop's place (m.xc, m.rp) commits only the exception's
  // (R140: m_dvok last; and from MEM's terms directly, not through mh_e and m_advb: with m.rp clear,
  // mh_3 is the small unit's term alone. m.rp -> mh_3 -> mh_e -> m_advb -> m_okb was four levels in front of
  // the BAT and micro-TLB enables, -1.78 ns in a ppc_cpu spike)
  (* keep = 1 *) wire m_ok1 = m.valid && !m.rp && !m.xc && !m_algc;
  (* keep = 1 *) wire m_ok2 = !mh_1 && !mh_2 && !mh_4 && !(m.fsm && (fr_at[1] || wb_blk));
  (* keep = 1 *) wire m_okb = m_ok1 && m_ok2;
`ifdef VERILATOR
  always_ff @(posedge clk) if (R1C && !rst && m_okb != (m_advb && !m_noef)) $fatal(1, "m_okb by parts");
`endif
  wire        m_ok   = m_okb && m_dvok;
  wire [31:0] dec_nx = (m_ok && m.dec_we) ? m.sprv : tb_tick ? dec_q - 32'd1 : dec_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      cr_q  <= 32'd0;
      xer_q <= 32'd0;
      lr_q  <= 32'd0;
      ctr_q <= 32'd0;
      ctr1_q <= 1'b0;
      msr_q <= 32'h0000_0040;                            // IP: vectors at 0xFFF00000
      res_q <= 1'b0;
      tb_q  <= 64'd0;
      dec_q <= 32'hFFFF_FFFF;
    end else begin
      if (tb_tick) tb_q <= tb_q + 64'd1;
      dec_q <= dec_nx;
      if (wb_trap) msr_q <= wb_tmsr;
      if (mem_adv) begin
        if (m.msr_we || sl_flt || m_fpxk || m_alg) msr_q <= m_msrx;
        if (!m_noef) begin                             // an exception in its place has no other effect
          if (m.cr_we)  cr_q  <= m_cr;
          if (m.xer_we) xer_q <= m.xer;
          if (m.lr_we)  lr_q  <= m.lr;
          if (m.ctr_we) begin ctr_q <= m.ctr; ctr1_q <= m.ctr1; end
          if (m.res_set) res_q <= 1'b1;
          if (m.res_clr) res_q <= 1'b0;
          if (m.tb_we) begin
            if (m.tbu) tb_q[63:32] <= m.sprv;
            else       tb_q[31:0]  <= m.sprv;
          end
        end
      end
    end
  end

  // Decrementer (PEM 2.3.13.1): a request whenever DEC[0] goes from 0 to 1,
  // counting through zero or by mtdec; one request however many arrive, and
  // taking the exception cancels it (603e 4.5.9). The external interrupt is a
  // level, sampled once.
  always_ff @(posedge clk) begin
    if (rst) begin
      int_q    <= 1'b0;
      dec_pend <= 1'b0;
    end else begin
      int_q <= ext_int;
      if (!dec_q[31] && dec_nx[31]) dec_pend <= 1'b1;
      if (mem_adv && m.irq[1])      dec_pend <= 1'b0;
    end
  end

  // ------------------------------------------------- MMU and exception commit
  // SRR1 as an exception commits: a TLB miss adds CR0 (table 4-17) from the
  // committed CR, and an ITLB miss its WAY from the committed LRU, so both
  // include everything older
  wire        m_bat   = m.spr_we && m.spr >= 6'd20 && m.spr <= 6'd35;
  wire [3:0]  m_batn  = m.spr[3:0] - 4'd4;              // SPR index 20-35: {D, n, L}
  // (R140) each BAT register's write from registers (m_okb, MEM's SPR index), m_dvok joined at the enable
  (* keep = 1 *) logic [15:0] bat_we_e;
  always_comb
    for (int k = 0; k < 16; k++) bat_we_e[k] = m_okb && m.spr_we && m.spr == 6'(20 + k);
`ifdef VERILATOR
  always_ff @(posedge clk)
    for (int k = 0; k < 16; k++)
      if (R1C && !rst && (bat_we_e[k] && m_dvok) != (m_ok && m_bat && m_batn == 4'(k))) $fatal(1, "BAT write %0d by index", k);
`endif
  wire [62:0] m_tlbe  = {m.tlbl && (m.insn[6] ? icmp_q[31] : dcmp_q[31]),
                         m.insn[6] ? icmp_q[30:7] : dcmp_q[30:7], m.ya[15:5], rpa_q};
  wire        m_tway  = srr1_q[17];                     // SRR1[WAY]
  always_ff @(posedge clk) begin
    if (m_ok && m.srw) begin
      dsr_mem[m.ya[19:16]] <= m.sprv;
      fsr_mem[m.ya[19:16]] <= m.sprv;
      isr_mem[m.ya[19:16]] <= m.sprv;
    end
    // tlbie: both ways of the set in both TLBs (5.4.3.2); tlbld / tlbli: the
    // set's way SRR1[WAY] (2.3.8)
    if (m_ok && (m.tlbie || (m.tlbl && !m.insn[6] && !m_tway))) dtlb0[m.ya[4:0]] <= m_tlbe;
    if (m_ok && (m.tlbie || (m.tlbl && !m.insn[6] &&  m_tway))) dtlb1[m.ya[4:0]] <= m_tlbe;
    if (m_ok && (m.tlbie || (m.tlbl &&  m.insn[6] && !m_tway))) itlb0[m.ya[4:0]] <= m_tlbe;
    if (m_ok && (m.tlbie || (m.tlbl &&  m.insn[6] &&  m_tway))) itlb1[m.ya[4:0]] <= m_tlbe;
  end
  // the TLB blocks' reads, every cycle at the set the next cycle's lookup uses (f_rsp_pc's, as it loads; MEM's
  // page, as m_xpg loads or holds), and the write flags for the lookup cycle
  wire [4:0] itr_n = f_issue ? fetch_pc[16:12] : f_rsp_pc[16:12];
  wire [4:0] dtr_n = mem_hold ? m_xpg[4:0] : ea_x[16:12];
  always_ff @(posedge clk) begin
    it0_q <= itlb0[itr_n];
    it1_q <= itlb1[itr_n];
    dt0_q <= dtlb0[dtr_n];
    dt1_q <= dtlb1[dtr_n];
    itw_q <= m_ok && (m.tlbie || (m.tlbl && m.insn[6]));
    dtw_q <= m_ok && (m.tlbie || (m.tlbl && !m.insn[6]));
  end
`ifdef VERILATOR
  logic tq_v;                                          // (a cycle after reset or a checkpoint's restore)
  always_ff @(posedge clk) tq_v <= !rst;
  always_ff @(posedge clk)
    if (!rst && tq_v && ((!itw_q && (it0_q != itlb0[f_rsp_pc[16:12]] || it1_q != itlb1[f_rsp_pc[16:12]])) ||
                 (m.valid && (!dtw_q && (dt0_q != dtlb0[m_xpg[4:0]] || dt1_q != dtlb1[m_xpg[4:0]])))))
      $fatal(1, "TLB block reads against the arrays at f_rsp_pc's set and MEM's page");
`endif
  always_ff @(posedge clk) begin
    if (rst) begin
      dlru <= 32'd0;
      ilru <= 32'd0;
    end else begin
      // replacement order: the way not loaded most recently
      if (m_ok && m.tlbl) begin
        if (m.insn[6]) ilru[m.ya[4:0]] <= !m_tway;
        else           dlru[m.ya[4:0]] <= !m_tway;
      end
    end
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      dsisr_q  <= 32'd0;
      dar_q    <= 32'd0;
      dmiss_q  <= 32'd0;
      dcmp_q   <= 32'd0;
      imiss_q  <= 32'd0;
      icmp_q   <= 32'd0;
      last_i_q <= 1'b0;
    end else if (mem_adv) begin
      if (m_dsi) begin                                  // table 4-12
        dsisr_q <= {4'd0, m_dsb, 25'd0};
        dar_q   <= m_xea;                                 // a crossing's second page: its first byte (4.5.3)
      end
      if (m_alg) begin                                  // table 4-14, X form; [22-26] from the field
        dsisr_q <= (m.insn[31:26] == 6'd31)
                   ? {15'd0, m.insn[2:1], m.insn[6], m.insn[10:7], m.insn[25:21], m.insn[20:16]}
                   : {17'd0, m.insn[26], m.insn[30:27], m.insn[25:21], m.insn[20:16]};   // D form
        dar_q   <= m.result + {29'd0, m.algn && m.algd4, 2'b00};   // lmw / stmw: EA + 4 (4.5.6.2)
      end
      if (m_tmiss) begin                                // tables 5-9, 5-11
        if (m.imiss) begin
          imiss_q <= m.pc;
          icmp_q  <= {1'b1, m_vsid, 1'b0, m.pc[27:22]};
        end else begin
          dmiss_q <= m_xea;
          dcmp_q  <= {1'b1, m_vsid, 1'b0, m_xea[27:22]};
        end
        last_i_q <= m.imiss;
      end
      if (!m_noef && m.spr_we) begin
        case (m.spr)
          6'd6:  dsisr_q <= m.sprv;
          6'd7:  dar_q   <= m.sprv;
          6'd13: dmiss_q <= m.sprv;
          6'd14: dcmp_q  <= m.sprv;
          6'd17: imiss_q <= m.sprv;
          6'd18: icmp_q  <= m.sprv;
          default: ;
        endcase
      end
    end
  end
`ifdef R104_CHECK
  // (micro-TLB invalidation) every valid entry of either micro-TLB is a translation the TLB and the segment
  // registers give now: its set's way that hits for the page (way 0 first, as a lookup) has its RPN
  always_ff @(posedge clk)
    if (!rst) begin
      for (int k = 0; k < DUN; k++)
        if (du_v[k]) begin
          automatic logic [31:0] sr = dsr_mem[du_pg[k][19:16]];
          automatic logic [62:0] t0 = dtlb0[du_pg[k][4:0]], t1 = dtlb1[du_pg[k][4:0]];
          automatic logic h0 = t0[62] && t0[61:38] == sr[23:0] && t0[37:27] == du_pg[k][15:5];
          automatic logic h1 = t1[62] && t1[61:38] == sr[23:0] && t1[37:27] == du_pg[k][15:5];
          if (!(h0 ? t0[26:7] == du_pa[k] : h1 && t1[26:7] == du_pa[k]))
            $fatal(1, "data micro-TLB %0d (page %05X) is not the TLB's translation", k, du_pg[k]);
        end
      for (int e = 0; e < IUN; e++)
        if (iu_v[e]) begin
          automatic logic [31:0] sr = isr_mem[iu_pg[e][19:16]];
          automatic logic [62:0] t0 = itlb0[iu_pg[e][4:0]], t1 = itlb1[iu_pg[e][4:0]];
          automatic logic h0 = t0[62] && t0[61:38] == sr[23:0] && t0[37:27] == iu_pg[e][15:5];
          automatic logic h1 = t1[62] && t1[61:38] == sr[23:0] && t1[37:27] == iu_pg[e][15:5];
          if (!(h0 ? t0[26:7] == iu_pa[e] : h1 && t1[26:7] == iu_pa[e]))
            $fatal(1, "instruction micro-TLB %0d (page %05X) is not the TLB's translation", e, iu_pg[e]);
        end
    end
`endif
  // the data micro-TLB: filled by a replay's translation, emptied by a write to the MMU's state (with UTS
  // only the entries it can change; its entries carry the MSR[PR] they were made under)
  //
  // Replacement (DRP). 0: in turn, du_nx. 1: least recently used, an empty entry first (the lowest). The
  // use order is a matrix of DUN x DUN bits, du_ord[i][j] = entry i was used after entry j (i != j): a use
  // of entry i sets its row and clears its column, and the oldest entry is the one whose row is all zero.
  // An entry is used by a page-translated access that leaves EX with its page matched (du_hit, registered as
  // MEM takes the uop, du_hq, and applied while MEM holds it, so nothing of it is in front of EX's request)
  // and by its fill.
  // Only the choice of entry changes; what an entry holds, and when it is emptied, is as with DRP = 0
  logic [DUN-1:0]          du_hq;                      // the entries the uop in MEM matched in EX
  logic [DUN-1:0][DUN-1:0] du_ord;
  logic [DUW-1:0]          du_fx;                      // the entry the next fill takes
  always_comb begin
    du_fx = du_nx;
    if (DRP) begin
      for (int k = DUN - 1; k >= 0; k--)
        if (!(|(du_ord[k] & ~(DUN'(1) << k)))) du_fx = DUW'(k);   // the oldest
      for (int k = DUN - 1; k >= 0; k--)
        if (!du_v[k]) begin du_fx = DUW'(k); end                 // an empty one before it
    end
  end
  always_ff @(posedge clk) if (!mem_hold) du_hq <= du_hit;
  wire              du_fill = mem_adv && m.rp && !sl_flt && !tnop;
  generate if (DRP) begin : g_drp
    always_ff @(posedge clk) begin
      if (rst) du_ord <= '0;
      else begin
        automatic logic [DUN-1:0] u = du_fill ? (DUN'(1) << du_fx) :
                                      (m.valid && (m.req || m.rp) && msr_q[MSR_DR]) ? du_hq : '0;
        for (int i = 0; i < DUN; i++)
          if (u[i])
            for (int j = 0; j < DUN; j++) begin
              du_ord[i][j] <= (i != j);
              du_ord[j][i] <= 1'b0;
            end
      end
    end
  end else begin : g_drn
    always_ff @(posedge clk) du_ord <= '0;
  end endgenerate
`ifdef VERILATOR
  // (DRP) the use order is an order: no two entries each used after the other, and some entry is the oldest
  always_ff @(posedge clk)
    if (!rst && DRP) begin
      automatic int n = 0;
      for (int k = 0; k < DUN; k++) if (!(|(du_ord[k] & ~(DUN'(1) << k)))) n++;
      for (int i = 0; i < DUN; i++) for (int j = 0; j < DUN; j++)
        if (i != j && du_ord[i][j] && du_ord[j][i]) $fatal(1, "data micro-TLB use order %0d %0d", i, j);
      if (n == 0) $fatal(1, "data micro-TLB use order: no oldest entry");
    end
`endif
  always_ff @(posedge clk) begin
    if (rst || (m_ok && (m_bat || (!UTS && (m.srw || m.tlbie || m.tlbl))))) begin
      du_v  <= '0;
      du_nx <= '0;
    end else if (UTS && m_ok && (m.srw || m.tlbie || (m.tlbl && !m.insn[6]))) begin
      // (micro-TLB invalidation, above "fetch": that segment's entries, or that TLB set's)
      for (int k = 0; k < DUN; k++)
        if (m.srw ? du_pg[k][19:16] == m.ya[19:16] : du_pg[k][4:0] == m.ya[4:0]) du_v[k] <= 1'b0;
    end else if (du_fill) begin
      du_v[du_fx]  <= 1'b1;
      du_wimg[du_fx] <= sl_h0_q ? sl_wimg0_q : sl_wimg1_q;
      du_pg[du_fx] <= sl_ea[31:12];
      du_pa[du_fx] <= sl_h0_q ? sl_pa0_q : sl_pa1_q;
      du_wr[du_fx] <= sl_w_q;
      du_pr[du_fx] <= msr_q[MSR_PR];
      du_nx        <= ((DUN & (DUN - 1)) != 0 && du_nx == DUW'(DUN - 1)) ? '0 : DUW'(du_nx + 1'b1);   // (6: wraps at 6)
    end
  end

  // HID0's cache bits as flip-flops (zero at reset, table 2-2), and the
  // flash-invalidate pulses as HID0 is written with ICFI or DCFI set
  logic [31:0] hid0_q;
  // HID0[DCE] for the data cache, a copy of its own loaded as hid0_q[14] is (preserve): the bit reaches
  // the snoop's decision and the tag write's select there (R108: hid0_q[14] -> tw_d_q, a 1.8 ns route)
  (* preserve *) logic hid_dce_q;
  logic        icfi_q, dcfi_q;
  wire         m_hid0 = m_ok && m.spr_we && m.spr == 6'd9;
  always_ff @(posedge clk) begin
    if (rst) begin
      hid0_q <= 32'd0;
      hid_dce_q <= 1'b0;
      icfi_q <= 1'b0;
      dcfi_q <= 1'b0;
    end else begin
      if (m_hid0) hid0_q <= m.sprv;
      if (m_hid0) hid_dce_q <= m.sprv[14];
      icfi_q <= m_hid0 && m.hcfi[1];
      dcfi_q <= m_hid0 && m.hcfi[0];
    end
  end
  assign hid_ice    = hid0_q[15];
  assign hid_dce    = hid_dce_q;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && hid_dce_q != hid0_q[14]) $fatal(1, "HID0[DCE] copy");
`endif
  assign hid_ilock  = hid0_q[13];
  assign hid_dlock  = hid0_q[12];
  assign hid_icfi   = icfi_q;
  assign hid_dcfi   = dcfi_q;
  assign hid_noopti = hid0_q[0];

  // copies of SDR1, RPA and the BAT fields the translation reads
  always_ff @(posedge clk) begin
    if (m_ok && m.spr_we && m.spr == 6'd8)  sdr1_q <= {m.sprv[31:16], m.sprv[8:0]};
    if (m_ok && m.spr_we && m.spr == 6'd19) rpa_q  <= {m.sprv[31:12], m.sprv[7:3], m.sprv[1:0]};
    for (int k = 0; k < 8; k++) begin
      if (bat_we_e[2*k] && m_dvok) begin                // BATnU: BEPI, BL, Vs, Vp
        bat_bepi[k] <= m.sprv[31:17];
        bat_bl[k]   <= m.sprv[12:2];
        bat_vs[k]   <= m.sprv[1];
        bat_vp[k]   <= m.sprv[0];
      end
      if (bat_we_e[2*k + 1] && m_dvok) begin            // BATnL: BRPN, WIMG, PP
        bat_brpn[k] <= m.sprv[31:17];
        bat_wimg[k] <= m.sprv[6:3];
        bat_pp[k]   <= m.sprv[1:0];
      end
    end
  end

  // the array's write address through a LUT of its own (keep): fed straight
  // from a duplicate of m.spr placed beside it, the MLAB's write address
  // register missed hold by 0.085 ns (a round-2 cached full build); the
  // setup side of that path has some 8 ns to spare
  (* keep = 1 *) wire [5:0] spr_wa = m.spr;
  always_ff @(posedge clk) begin
    if (m_ok && m.spr_we && m.spr >= 6'd2) spr_mem[spr_wa] <= m.sprv;
    if (rst) begin
      srr0_q <= 32'd0;
      srr1_q <= 32'd0;
    end else if (wb_trap) begin                        // a fired trap, one cycle on
      srr0_q <= wb_tsrr0;
      srr1_q <= wb_tsrr1;
    end else if (mem_adv) begin
      if (m_xcp) begin
        srr0_q <= m.srr0;
        srr1_q <= m_srr1c;
      end else if (m.spr_we && m.spr == 6'd0) srr0_q <= m.sprv;
      else if (m.spr_we && m.spr == 6'd1)     srr1_q <= m.sprv;
    end
  end

  wire wbr_ld = !rst && ((!wb_hold && m.is_load) || (wb_hold && ans_wb && wb_sp1));
  wire wbr_rs = !rst && !wb_hold && !m.is_load;

  // ======================================================================= WB
  // A load or store waits here for its answer, and the GPR or FPR write is
  // made as WB is left. A load's word that arrives here is aligned on the
  // forward path (ld_val); one that came in MEM is in wb_result already.
  always_ff @(posedge clk) begin
    if (rst) begin
      wb_valid <= 1'b0;
      wb_c2    <= 1'b0;
    end else begin
      wb_c2 <= wb_hold;
      if (!wb_hold) begin
        wb_valid  <= mem_adv;
        wb_last   <= m.rp ? sl_flt || tnop : m.last;    // a replay is not the instruction's end
        wb_rd_we  <= m.rd_we && !m.rp;
        wb_rd     <= m.rd;
        wb_rd_sh  <= m_sh;
        wb_irq    <= mem_adv ? m.irq : 2'd0;
        wb_evm    <= !mem_adv ? 5'd0 : {m.rp, m.irq != 2'd0, m_dsi || (m.xc && m.xvec == 13'h0400),
                                        m.xc && m.imiss, m_tmiss && !m.imiss};
        wb_iss    <= m_iss;
        wb_got    <= m_got || ans_m;
        wb_live_q <= m.is_load && !(m_got || ans_m);
        wb_dq     <= m_got ? m_dq : d_rdata;
        wb_isld   <= m.is_load;
        wb_fhi    <= m.fhi && !m.rp && !m.algn;
        wb_sp0    <= m.sp0 && !m.rp;
        wb_sp1    <= m.sp1 && !m.rp;
        wb_msize  <= m.msize;
        wb_boff   <= m.boff;
        wb_msigned <= m.msigned;
        wb_brev   <= m.brev;
        for (int r = 0; r < 4; r++) begin
          la_jq[r] <= la_ix(r[1:0], m.msize, m.boff, m.brev);
          la_dq[r] <= mg_isd(r[1:0], m.msize);
          wbx_q[r] <= mg_ix(r[1:0], m.msize, m.boff, m.brev);
        end
        la_sq     <= m.msize == 2'd1 && m.msigned;
        wb_fpw    <= mem_adv && m.fpw && !m.rp && !m.algn;
        wb_fl     <= mem_adv && m.fld && !m.rp && !m.algn;
        wb_fsl    <= m.is_load && m.fsl;
        wb_frd    <= m.rd;
        wb_fdat_q <= ms_res_m;
        wb_pc     <= m.pc;
        wb_insn   <= m.insn;
        wb_dly    <= m_dly;
      end else begin
        if (ans_wb) begin
          wb_got    <= 1'b1;
          wb_live_q <= 1'b0;
          wb_dq     <= d_rdata;
          // a split load's second word, merged a cycle (WB holds for it): wb_result, below
        end
      end
    end
    if (wb_adv && wb_fhi) fhi_q <= ldw;
    // FP load write slot (FLW): taken as the load leaves WB, its word as it is (lfs converts it in u_lfs
    // over this cycle and the next); kept converted (fl_dat) while the port is busy
    if (rst || !FLW)          fl_v <= 1'b0;
    else if (wb_adv && wb_fl) fl_v <= 1'b1;
    else if (fl_go)           fl_v <= 1'b0;
    if (wb_adv && wb_fl) begin
      fl_frd <= wb_frd;
      fl_sl  <= wb_fsl;
      fl_hi  <= fhi_q;
      fl_lo  <= ldw;
      fl_p2  <= 1'b0;
    end else if (fl_v && !fl_go) begin
      fl_p2  <= 1'b1;
      if (!fl_p2) fl_dat <= lfs_dbl;
    end
    // wb_result: MEM's result or load value as WB takes MEM's uop, or a split load's second word
    // merged while WB holds; its two loads formed from d_valid beside the merge (R108)
    if (wbr_ld)      wb_result <= ld_mg;
    else if (wbr_rs) wb_result <= m.result;
    if (wb_adv && wb_sp0) mis_w0 <= ldw;                 // a split's first word, kept for its second
  end

  // =================================================================== events
  assign ev[0] = wb_adv && wb_last;
  assign ev[1] = wb_adv;
  assign ev[2] = !id_valid && !ex_hold && !redirect;
  assign ev[3] = id_valid && load_use && !ex_hold;
  assign ev[4] = md_wait && !mem_hold;
  assign ev[5] = mem_hold || (!m.valid && wb_hold);   // MEM or WB waits (an answer, mostly)
  assign ev[6] = redirect;
  assign ev[7] = f_out && !i_valid;
  assign ev_mmu = {f_slow, wb_evm};
  always_ff @(posedge clk) ev_xrf <= !rst && mem_adv && m.rp && m_x4 && !sl_flt && !tnop;

  // ==================================================================== trace
  logic [31:0] tr_cr, tr_xer, tr_lr, tr_ctr, tr_msr, tr_srr0, tr_srr1;
  always_ff @(posedge clk) begin
    if (mem_adv) begin
      tr_cr   <= (m.cr_we  && !m_noef) ? m_cr  : cr_q;
      tr_xer  <= (m.xer_we && !m_noef) ? m.xer : xer_q;
      tr_lr   <= (m.lr_we  && !m_noef) ? m.lr  : lr_q;
      tr_ctr  <= (m.ctr_we && !m_noef) ? m.ctr : ctr_q;
      tr_msr  <= m_trap ? m.msr : (m.msr_we || sl_flt || m_fpxk || m_alg) ? m_msrx : msr_q;
      tr_srr0 <= (m_xcp || m_trap) ? m.srr0 : (m.spr_we && m.spr == 6'd0) ? m.sprv : srr0_q;
      tr_srr1 <= m_xcp ? m_srr1c : m_trap ? m.srr1 : (m.spr_we && m.spr == 6'd1) ? m.sprv : srr1_q;
    end
  end

  assign rt_valid   = wb_adv;
  assign rt_last    = wb_last;
  assign rt_pc      = wb_pc;
  assign rt_insn    = wb_insn;
  assign rt_gpr_we  = wb_rd_we;
  assign rt_gpr     = wb_rd;
  assign rt_gpr_sh  = wb_rd_sh;
  assign rt_irq     = wb_irq;
  assign rt_gpr_val = wb_val;
  assign rt_cr      = tr_cr;
  assign rt_xer     = tr_xer;
  assign rt_lr      = tr_lr;
  assign rt_ctr     = tr_ctr;
  assign rt_msr     = tr_msr;
  assign rt_srr0    = tr_srr0;
  assign rt_srr1    = tr_srr1;
  assign rt_fpscr   = fpscr_q;
  // (with WB waiting for an answer, a younger FP op in MEM may already have
  // changed FP state: not settled for this record then)
  assign rt_fpq     = (pend & ~({32{fl_go}} & (32'd1 << fl_frd))) == 32'd0 && fs_idle && !(m.valid && (m.fsx || m.fany));
  assign rt_fw_en   = fw_we;
  assign rt_fw_idx  = fw_idx;
  assign rt_fw_val  = fw_dat;
  assign rt_dly     = wb_dly;

endmodule
