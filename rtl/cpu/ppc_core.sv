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
// Not yet here: misaligned accesses (forced aligned), the alignment
// exception for them, privilege checks other than dcbi's (MSR[PR] selects
// keys and BAT valid bits), MSR[LE].
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

module ppc_core
  import ppc_pkg::*, ppc_fpu_pkg::*;
#(
  parameter logic [31:0] RESET_PC = 32'hFFF0_0100,
  parameter logic [31:0] PVR      = 32'h0006_0101,
  // the multiply-add unit's optional stage boundaries (ppc_fpu_fma.sv); all
  // in, L = 12, is what 120 MHz needs
  parameter bit P1A = 1'b1, P2 = 1'b1, P3 = 1'b1, P4A = 1'b1, P5A = 1'b1,
  parameter bit P5  = 1'b1, P6A = 1'b1, P6 = 1'b1, P7 = 1'b1
)(
  input  logic        clk,
  input  logic        rst,

  output logic        i_req,
  output logic [31:0] i_addr,
  output logic [3:0]  i_wimg,       // the fetch's WIMG (real mode 0011, 603e 3.5)
  output logic        i_seq,        // i_addr is the previous request's + 4, same translation
  input  logic        i_valid,
  input  logic [31:0] i_data,

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
  } mem_t;
  mem_t m;
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
  (* ramstyle = "MLAB, no_rw_check" *) logic [62:0] dtlb0 [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [62:0] dtlb1 [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [62:0] itlb0 [0:31];
  (* ramstyle = "MLAB, no_rw_check" *) logic [62:0] itlb1 [0:31];
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
  logic [31:0] f_pc;
  logic        f_out, f_discard;
  logic [2:0]  q_cnt;

  logic        redirect;                               // hard, decided this cycle
  logic [31:0] redirect_pc;
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
  wire  i_vld   = i_valid || f_fpend;
  wire  rsp     = f_out && i_vld;
  wire  rsp_ok  = rsp && !f_discard && !redir_q;       // the response is the one wanted
  wire  rsp_xm  = rsp_ok && f_xm;                      // ... but it was not translated: translate it
  wire  rsp_use = rsp_ok && !f_xm;
  assign redir_pc_q = (redir_k_q == 2'd0) ? redir_a_q : (redir_k_q == 2'd1) ? redir_b_q : redir_s_q;
  wire  [31:0] fetch_pc = redir_q ? redir_pc_q : f_pc;

  logic [1:0]        iu_v, iu_nx, iu_pr;
  logic [1:0][19:0]  iu_pg, iu_pa;
  logic [1:0][3:0]   iu_wimg;
  logic [3:0][3:0]   fc_hit;                           // [candidate][IBAT]
  logic [3:0]        fc_ok;                            // candidate may go out at once
  logic [3:0][31:0]  fc_pc;
  always_comb begin
    fc_pc = {redir_s_q, redir_b_q, redir_a_q, f_pc};
    for (int c = 0; c < 4; c++) begin
      fc_ok[c] = 1'b0;
      for (int k = 0; k < 4; k++) begin
        fc_hit[c][k] = bat_match(fc_pc[c][31:17], bat_bepi[k], bat_bl[k], f_pr ? bat_vp[k] : bat_vs[k]);
        // a BAT hit that allows the fetch: PP not 00, not guarded (table 5-3)
        fc_ok[c] = fc_ok[c] | (fc_hit[c][k] && bat_pp[k] != 2'd0 && !bat_wimg[k][0]);
      end
      for (int e = 0; e < 2; e++) fc_ok[c] = fc_ok[c] | (iu_v[e] && iu_pr[e] == f_pr && iu_pg[e] == fc_pc[c][31:12]);
    end
  end
  wire  [1:0]  f_sel  = !redir_q ? 2'd0 : (redir_k_q == 2'd0) ? 2'd1 : (redir_k_q == 2'd1) ? 2'd2 : 2'd3;
  wire  [3:0]  ib_hit = fc_hit[f_sel];
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
    for (int e = 0; e < 2; e++) begin
      iu_pahi   = iu_pahi   | ({20{iu_v[e] && iu_pr[e] == f_pr && iu_pg[e] == fetch_pc[31:12]}} & iu_pa[e]);
      iu_wimg_s = iu_wimg_s | ({4{iu_v[e] && iu_pr[e] == f_pr && iu_pg[e] == fetch_pc[31:12]}} & iu_wimg[e]);
    end
  end
  // the fetch's WIMG: real mode 0011 (3.5), an IBAT's, or the micro-TLB's
  // (an untranslated request's goes nowhere: its response is not used)
  assign i_wimg = !f_ir ? 4'b0011 : ib_go ? ib_wimg : iu_wimg_s;
  // with no translation for it, physical page 0
  wire  [19:0] i_pahi = !f_ir ? fetch_pc[31:12] : {ib_pa, fetch_pc[16:12] & {5{ib_go}}} | iu_pahi;

  // the slow cycles: f_rsp_pc's translation, from registers
  wire  [31:0] isr    = isr_mem[f_rsp_pc[31:28]];
  wire  [62:0] it0    = itlb0[f_rsp_pc[16:12]];
  wire  [62:0] it1    = itlb1[f_rsp_pc[16:12]];
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
  wire         pw_pred = (pw_op == 6'd18) ||
                         (pw_op == 6'd16 && ((pw_bo[4] && pw_bo[2]) || (i_data[15] ^ pw_bo[0])));
  wire  [31:0] pw_tgt  = (i_data[1] ? 32'd0 : f_rsp_pc) + pw_disp;
  wire         f_soft  = rsp_use && pw_pred;

  // A fetch goes out (f_issue) as without translation, except while
  // translating and on the cycle its untranslated response comes back (the
  // address is fetched again after). A fault in the slow cycles stands for a
  // request (its response is the marker, next cycle). None goes out on a fault
  // response's cycle, so the held fault is never overwritten before its marker
  // is in the queue.
  wire  f_issue = !rst && !f_stall && !f_fpend && !f_slow && !f_slow2 && !rsp_xm && (!f_out || rsp) &&
                  (redir_hard_q || ({1'b0, q_cnt} + {3'd0, rsp_use} < 4'd4));
  assign i_addr = {i_pahi, fetch_pc[11:0]};
  assign i_req  = f_issue;
  // i_seq: the request follows the previous one at + 4, both translated at
  // once (a request that went untranslated, f_xm, is fetched again from a
  // redirect-like reload, s_ok, which is never sequential)
  // (the translation part is f_xm's: a request that went out translated
  // has f_xm = 0, so f_fast drives one register, not two)
  logic f_sq;
  assign i_seq  = !redir_q && f_sq && !f_xm;
  always_ff @(posedge clk)
    if (rst)                         f_sq <= 1'b0;
    else if (f_issue)                f_sq <= 1'b1;
    else if (redir_q || s_ok || s_flt) f_sq <= 1'b0;

  wire  iu_clr  = m_ok && (m.srw || m.tlbie || m.tlbl || m_bat);
  always_ff @(posedge clk) begin
    if (rst) begin
      f_fpend <= 1'b0;
      f_stall <= 1'b0;
      f_xm    <= 1'b0;
      f_slow  <= 1'b0;
      f_slow2 <= 1'b0;
      iu_v    <= 2'd0;
      iu_nx   <= 2'd0;
    end else begin
      f_fpend <= s_flt;
      if (f_issue)    f_xm <= !f_fast;
      else if (s_flt) f_xm <= 1'b0;                     // the marker's response is used
      f_slow  <= rsp_xm;
      f_slow2 <= f_slow && !redir_q;
      if (redir_hard_q)                  f_stall <= 1'b0;
      else if (rsp_use && f_fpend)       f_stall <= 1'b1;
      if (iu_clr) iu_v <= 2'd0;
      else if (s_ok) begin
        iu_v[iu_nx[0]]  <= 1'b1;
        iu_pg[iu_nx[0]] <= f_rsp_pc[31:12];
        iu_pa[iu_nx[0]] <= sf_pa;
        iu_pr[iu_nx[0]] <= f_pr;
        iu_wimg[iu_nx[0]] <= sf_wimg;
        iu_nx           <= {1'b0, !iu_nx[0]};
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
      redir_q      <= redirect || f_soft;
      redir_hard_q <= redirect;
      // the candidates load every cycle; only the 2-bit choice waits on the
      // redirect decision (R23 timing)
      redir_a_q    <= redirect_pc;
      redir_b_q    <= redirect_pc_b;
      redir_s_q    <= pw_tgt;
      redir_k_q    <= !redirect ? 2'd2 : redirect_sel ? 2'd0 : 2'd1;
      if (f_issue)      f_pc <= fetch_pc + 32'd4;
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

  // Queue writes go to the slot after the live entries, counted before any pop.
  logic [31:0] q_hi, q_hp;
  logic        q_hd;
  always_ff @(posedge clk) if (f_issue) f_rsp_pc <= fetch_pc;

  // Predecode as the word arrives, so the queue's pop needs no decoder (R23
  // timing): which of rA, rB, rS the first uop reads, and how it cracks
  // (0 single uop, 1 update form, 2 lmw / stmw). A second decoder instance;
  // all but these few outputs are trimmed. Everything else ID hands to EX
  // that does not change between a cracked instruction's uops comes from here
  // too: the operand capture's selects, the immediate's form, the result
  // select and truth table, the special registers written, the CR writer's
  // kind, the SPR number's decode and the move-from source. Each was four or
  // five LUT levels of the decoder in front of an EX register (R23 timing).
  uop_t        pd_u;
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
    logic [2:0] sh;          // rA, rB, rS is a TGPR shadow: MSR[TGPR] and the field < 4
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
  } pd_t;
  localparam int PDW = 62;                             // (alu_op_t is 6 bits)
  pd_t         pd_w, q_pd, q1_pd;
  logic [31:0] q1_hi;                                  // queue entry 1: the head after a pop
  ppc_decode #(.FPA (1'b0)) u_pdec (.insn (i_data), .pc (32'd0), .step (5'd0), .pr (1'b0), .u (pd_u));
  // FP arithmetic is predecoded apart, from the raw word: which FPRs an op of
  // opcode 59 / 63 reads and writes, and whether it writes the CR. Shallow
  // (an encoding the 603e does not have only costs a stall); the decoder
  // above leaves 59 / 63 alone, so FP adds no depth to the integer fields
  // on the fetch path, which limits the full build (R28). FP loads and
  // stores are predecoded by the decoder, as the integer ones.
  wire  [4:0]  pa5 = i_data[5:1];
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
  wire         pa_rc = pa_op && i_data[0] && !(!pa_a && (pa_cmp || pa_mcrfs));
  // a store: frS on the C port
  wire         pd_fst = (pd_op >= 6'd52 && pd_op <= 6'd55) ||
                        (pd_op == 6'd31 && (pd_x == 10'd663 || pd_x == 10'd695 || pd_x == 10'd727 ||
                                            pd_x == 10'd759 || pd_x == 10'd983));
  wire  [5:0]  pd_op = i_data[31:26];
  wire  [9:0]  pd_x  = i_data[10:1];
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
  wire  [9:0]  pd_spr = {i_data[15:11], i_data[20:16]};
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
  // decoder's op is a four-level mux on the fetch path (i_data to the queue,
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
    pd_w         = '0;
    pd_w.ua      = pd_u.a_used;
    pd_w.ub      = pd_u.b_used;
    pd_w.us      = pd_u.s_used;
    pd_w.crack   = pd_crack;
    pd_w.xs      = pd_u.x_from_s;
    pd_w.az      = pd_u.a_zero;
    pd_w.yreg    = pd_u.bsel == BSEL_RB;
    pd_w.spr_idx = pd_spr_idx;
    pd_w.mf_sel  = pd_mf_sel;
    pd_w.mfcr    = r_31 && pd_x == 10'd19;
    pd_w.mfmsr   = r_31 && pd_x == 10'd83;
    pd_w.immk    = pd_immk;
    pd_w.rk      = pd_rk;
    // the special registers this uop writes
    pd_w.w_cr    = pd_u.rc || r_cmp || r_crlog || r_mtcrf || r_mcrxr || r_stwcx;
    pd_w.w_xer   = pd_u.set_ca || pd_u.oe || r_mcrxr || (r_mtspr && pd_spr == 10'd1);
    pd_w.w_lr    = (r_br && i_data[0]) || (r_mtspr && pd_spr == 10'd8);
    pd_w.w_ctr   = (r_bc && !i_data[23]) || (r_mtspr && pd_spr == 10'd9);
    pd_w.w_spr   = r_mtspr && pd_spr_hit && pd_spr_idx != 6'd15 && pd_spr_idx != 6'd16;   // HASH1/2 read-only
    // fetch-side state, from registers (no logic on the word's path but the
    // three shadow compares)
    pd_w.sh      = {3{f_tg}} & {i_data[20:18] == 3'd0, i_data[15:13] == 3'd0, i_data[25:23] == 3'd0};
    pd_w.ifault  = f_fpend;
    pd_w.l0      = pd_crack == 2'd0 || (pd_crack == 2'd2 && i_data[25:21] == 5'd31);
    pd_w.ck      = {r_stwcx, r_mtcrf, (r_crlog && pd_x == 10'd0) || r_mcrxr, r_crlog && pd_x != 10'd0};
    // FP: nothing for a fault marker, whose word is not run
    pd_w.fua     = pa_ua && !f_fpend;
    pd_w.fub     = pa_ub && !f_fpend;
    pd_w.fuc     = (pa_uc || pd_u.fuc) && !f_fpend;
    pd_w.fpw     = (pa_w || pd_u.fpw || pd_u.fhi) && !f_fpend;   // lfd: frD from its first uop on
    pd_w.fci     = pd_fst ? i_data[25:21] : i_data[10:6];
    pd_w.fwcr    = pa_op && (pa_rc || (!pa_a && (pa_cmp || pa_mcrfs)));
    pd_w.fck1    = pa_op && (pa_rc || (!pa_a && pa_mcrfs));
    pd_w.hh      = pd_op == 6'd31 && (pd_x == 10'd339 || pd_x == 10'd371) && (pd_spr == 10'd978 || pd_spr == 10'd979);
    pd_w.fdx     = pd_u.fdbl;
    pd_w.ra0     = i_data[20:16] == 5'd0;
    pd_w.ill     = pd_u.op == OP_ILLEGAL && !pa_op;
    pd_w.pop     = pd_u.op;
  end

  // the pop, one copy for each entry (each copy drives an entry's 120 bits),
  // from a copy of EX's hold of its own (q_hold, below): ex_hold drives some
  // 700 enables across the core, and the queue's pop was behind that net
  logic q_hold;
  wire  q_take = id_valid && !q_hold && !load_use && !fp_haz && id_last;
  (* keep = 1 *) wire q_pop0 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop1 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop2 = q_take && !redir_hard_q;
  (* keep = 1 *) wire q_pop3 = q_take && !redir_hard_q;
  wire [3:0] q_pop = {q_pop3, q_pop2, q_pop1, q_pop0};
  ppc_fetchq #(.PDW (PDW)) u_q (
    .clk (clk), .clear (rst || redir_hard_q), .we (rsp_use && !redir_hard_q),
    .w_insn (i_data), .w_pc (f_rsp_pc), .w_pred (pw_pred), .w_pd (pd_w),
    .pop (q_pop), .count (q_cnt),
    .r_insn (q_hi), .r_pc (q_hp), .r_pred (q_hd), .r_pd (q_pd),
    .r1_insn (q1_hi), .r1_pd (q1_pd)
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
  wire wb_ldw     = wb_valid && wb_live_q && wb_rd_we && !d_valid;
  // (the register compares are made a cycle early: qe_*, below)
  logic [3:0] qe_e, qe_m, qe_w;
  function automatic logic lu_q(input logic [3:0] qe, input logic ua, input logic ub, input logic us);
    lu_q = (ua && qe[0]) || (ub && qe[1]) || (us && qe[3]);
  endfunction
  wire load_use   = (ex_is_load && ex_u.rd_we && lu_q(qe_e, q_pd.ua, q_pd.ub, q_pd.us)) ||
                    (m_ldw && lu_q(qe_m, q_pd.ua, q_pd.ub, q_pd.us)) ||
                    (wb_ldw && lu_q(qe_w, q_pd.ua, q_pd.ub, q_pd.us)) ||
                    hash_wait;

  // the uop that finishes the instruction: on a first uop from the predecode;
  // after it from a flag registered as the step advances (an update form's
  // second uop; lmw / stmw at rD + step = 31, from rS's counted index)
  logic id_s0, id_s1, id_lst_q;
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
  wire        hn_q0 = !id_take && q_cnt != 3'd0;       // the head next cycle: entry 0
  wire        hn_q1 = id_take && q_cnt >= 3'd2;        // entry 1; else the arriving word
  // id_s0, id_rs_q next cycle; a redirect is left out (ID then holds no uop)
  wire        id_advr = id_valid && !ex_hold && !load_use && !fp_haz;
  wire        hn_s0 = id_advr ? id_last : id_s0;
  wire  [4:0] hn_rq = id_advr ? id_rs + 5'd1 : id_rs_q;
  logic [3:0][4:0] qe_rd;                              // the producers: ID, EX, MEM, WB
  assign qe_rd = {wb_rd, m.rd, ex_u.rd, id_u.rd};
  logic [3:0][3:0] qe_hc;                              // [producer][field], for next cycle's head
  always_comb
    for (int r = 0; r < 4; r++) begin
      for (int f = 0; f < 3; f++)
        qe_hc[r][f] = hn_q0 ? qe_rd[r] == fq0[f] : hn_q1 ? qe_rd[r] == fq1[f] : qe_rd[r] == fin[f];
      qe_hc[r][3] = hn_s0 ? qe_hc[r][2] : qe_rd[r] == hn_rq;
    end
  // qf_m, qf_w: the same, with the producer's GPR write folded in (MEM:
  // rd_we and not a load; WB: rd_we), so that a forward select is one LUT of
  // the stage's valid bit and these (registered below, after EX)
  logic [3:0] qf_m, qf_w;
  always_ff @(posedge clk) begin
    qe_e <= ex_hold  ? qe_hc[1] : qe_hc[0];
    qe_m <= mem_hold ? qe_hc[2] : qe_hc[1];
    qe_w <= wb_hold  ? qe_hc[3] : qe_hc[2];
  end

  // the sixth address bit selects TGPR0-3: predecoded for the reads; for the
  // write, MSR[TGPR] as committed when the writer left MEM, which is the
  // writer's own (every change of TGPR redirects, so no uop of the other mode
  // is in flight)
  wire m_sh = msr_q[MSR_TGPR] && m.rd[4:2] == 3'd0;
  ppc_regfile u_rf (
    .clk (clk),
    .ra  ({q_pd.sh[2], id_u.ra}), .rb ({q_pd.sh[1], id_u.rb}), .rs ({q_pd.sh[0], id_rs}),
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
  logic       ex_fd0;                                  // EX holds an FP double's step 0
  logic [31:0] fea_q, fea4_q;
  logic [31:0] fimm_q;                                 // step 1's immediate: 4, or after a replay fea4_q
  wire        id_xfea  = q_pd.fdx && id_s1;
  wire        id_xz    = id_xzero || id_xfea;
  wire        id_yreg  = q_pd.yreg && !id_xfea;
  // y's immediate, by the predecoded form (the decoder's id_u.imm, built here
  // from the raw fields)
  logic [31:0] id_imm;
  // lmw / stmw: d + 4 x step without an adder in ID, counted as the step
  // advances like id_rs_q (the adder was in front of y's capture)
  logic [31:0] imm4_q;
  always_ff @(posedge clk)
    if (redirect)    imm4_q <= {{16{m.insn[15]}}, m.insn[15:0]} + {25'd0, m.step, 2'b00};   // a replay resumes here
    else if (id_adv) imm4_q <= (id_s0 ? {{16{q_hi[15]}}, q_hi[15:0]} : imm4_q) + 32'd4;
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
  end

  // (both fields compared, then chosen: the compare does not wait on the
  // field mux)
  wire        x_ex  = (!id_xz && ex_w && (id_xs ? qe_e[2] : qe_e[0])) ||
                      (id_xfea && ex_valid && ex_fd0);
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
`ifdef VERILATOR
  // the early compares against the live ones, wherever ID holds a uop
  always_ff @(posedge clk)
    if (!rst && id_valid)
      for (int p = 0; p < 3; p++) begin
        automatic logic [4:0] rd = p == 0 ? ex_u.rd : p == 1 ? m.rd : wb_rd;
        automatic logic [3:0] qe = p == 0 ? qe_e : p == 1 ? qe_m : qe_w;
        if (qe != {rd == id_rs, rd == q_hi[25:21], rd == q_hi[15:11], rd == q_hi[20:16]})
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
  (* keep = 1 *) wire [1:0] x_k = fw_k(x_m, x_wb, wb_live);
  (* keep = 1 *) wire [1:0] y_k = fw_k(y_m, y_wb, wb_live);
  (* keep = 1 *) wire [1:0] a_k = fw_k(a_m, a_wb, wb_live);
  (* keep = 1 *) wire [1:0] s_k = fw_k(s_m, s_wb, wb_live);
  wire [31:0] x_b   = id_xz ? 32'd0 : id_xrf;
  wire [31:0] y_b   = id_yreg ? rf_rb : id_imm;
  wire [31:0] a_b   = rf_ra;
  wire [31:0] s_b   = rf_rs;
  wire [31:0] x_cap = fw_mux(x_k, x_b, m.result, ld_val, wb_result);
  wire [31:0] y_cap = fw_mux(y_k, y_b, m.result, ld_val, wb_result);
  wire [31:0] a_cap = fw_mux(a_k, a_b, m.result, ld_val, wb_result);
  wire [31:0] s_cap = fw_mux(s_k, s_b, m.result, ld_val, wb_result);

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
  wire        cr_a   = ex_cra_s[0] ? m_cr0[2'd3 - ex_u.crba[1:0]] :
                       ex_cra_s[1] ? m.cmpv[2'd3 - ex_u.crba[1:0]] :
                       ex_cra_s[2] ? m.crlb : ex_cra_s[3] ? m.crf4[2'd3 - ex_u.crba[1:0]] :
                       ex_cra_s[4] ? m.mtca : ex_cra;
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && ex_valid && ((ex_cra_s[4:2] != 3'd0 && cr_a != m.cr[5'd31 - ex_u.crba]) ||
                             (ex_crb_s[4:2] != 3'd0 && cr_b != m.cr[5'd31 - ex_u.crbb])))
      $fatal(1, "CR bit from the writer ahead: %b%b against %b%b (kinds %b %b)", cr_a, cr_b,
             m.cr[5'd31 - ex_u.crba], m.cr[5'd31 - ex_u.crbb], ex_cra_s, ex_crb_s);
`endif
  wire        cr_b   = ex_crb_s[0] ? m_cr0[2'd3 - ex_u.crbb[1:0]] :
                       ex_crb_s[1] ? m.cmpv[2'd3 - ex_u.crbb[1:0]] :
                       ex_crb_s[2] ? m.crlb : ex_crb_s[3] ? m.crf4[2'd3 - ex_u.crbb[1:0]] :
                       ex_crb_s[4] ? m.mtcb : ex_crb;
  wire        res_in = (m.valid && m.res_set) ? 1'b1 : (m.valid && m.res_clr) ? 1'b0 : res_q;
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
                         id_isf ? id_fspr : spr_mem[id_spr_idx];
  wire [31:0] xer_nx   = (m.valid && m.xer_we) ? m.xer : xer_q;
  wire [31:0] lr_nx    = (m.valid && m.lr_we)  ? m.lr  : lr_q;
  wire [31:0] ctr_nx   = (m.valid && m.ctr_we) ? m.ctr : ctr_q;
  logic [31:0] id_mfv;
  always_comb begin
    case (id_mf_sel)
      3'd1:    id_mfv = xer_nx;
      3'd2:    id_mfv = lr_nx;
      3'd3:    id_mfv = ctr_nx;
      3'd7:    id_mfv = PVR;
      3'd0:    id_mfv = id_spr_hit ? id_spr_v : 32'd0;
      default: id_mfv = 32'd0;                         // DEC, TBL, TBU: read in EX
    endcase
    if (q_pd.mfcr)  id_mfv = m.valid ? m_cr : cr_q;
    if (q_pd.mfmsr) id_mfv = msr_q;
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
  wire  [31:0] mf_now  = ({32{mf_b}} & ex_mfv) | ({32{mf_f[0]}} & m.xer) | ({32{mf_f[1]}} & m.lr) |
                         ({32{mf_f[2]}} & m.ctr) | ({32{mf_f[3]}} & m.sprv) | ({32{mf_f[4]}} & m_crn) |
                         {{4{mf_c0}} & m_cr0, 28'd0};
  wire  [31:0] mf_val  = mf_now | ({32{mf_t[0]}} & dec_q) | ({32{mf_t[1]}} & tb_q[31:0]) | ({32{mf_t[2]}} & tb_q[63:32]);

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
  wire        md_start = ex_is_md && !ex_md_busy && !md_have && !md_drop;
  wire [31:0] md_val   = md_res | ({32{md_have}} & md_res_q);   // md_res is zero unless done
  wire        md_ovv   = md_done ? md_ov  : md_ov_q;

  // branch
  wire        bo_noctr  = ex_u.bo[2];
  wire        bo_ctrz   = ex_u.bo[1];
  wire        bo_nocond = ex_u.bo[4];
  wire        bo_cond   = ex_u.bo[3];
  wire [31:0] ctr_dec   = ctr_in - 32'd1;
  wire        cr_bit    = cr_a;                                     // BI
  wire        br_ctr_ok = bo_noctr || (!ctr1_in ^ bo_ctrz);         // CTR-1 != 0, from a flag
  wire        br_cnd_ok = bo_nocond || (cr_bit == bo_cond);
  logic       br_taken;
  logic [31:0] br_target;
  always_comb begin
    case (ex_u.bkind)
      BR_B:     begin br_taken = 1'b1;                    br_target = (ex_u.aa ? 32'd0 : ex_u.pc) + ex_u.imm; end
      BR_BC:    begin br_taken = br_ctr_ok && br_cnd_ok;  br_target = (ex_u.aa ? 32'd0 : ex_u.pc) + ex_u.imm; end
      BR_BCLR:  begin br_taken = br_ctr_ok && br_cnd_ok;  br_target = {lr_in[31:2], 2'b00};  end
      default:  begin br_taken = br_cnd_ok;               br_target = {ctr_in[31:2], 2'b00}; end
    endcase
  end

  // memory access
  assign alu_res = r_sum;
  wire [31:0] ea      = r_sum;
  wire        is_ld   = ex_u.op == OP_LOAD;
  wire        is_st   = ex_u.op == OP_STORE;
  wire        st_ok   = !ex_u.cond || res_in;           // stwcx. stores only when reserved
  wire [1:0]  boff    = (ex_u.msize == 2'd2) ? 2'd0 : (ex_u.msize == 2'd1) ? {ea[1], 1'b0} : ea[1:0];
  logic [31:0] st_data, sv;
  logic [3:0]  st_be;
  always_comb begin
    sv = ex_fst ? fsv : s_val;
    case (ex_u.msize)
      2'd2: begin
        st_data = ex_u.brev ? {sv[7:0], sv[15:8], sv[23:16], sv[31:24]} : sv;
        st_be   = 4'b1111;
      end
      2'd1: begin
        st_data = ex_u.brev ? {2{sv[7:0], sv[15:8]}} : {2{sv[15:0]}};
        st_be   = boff[1] ? 4'b0011 : 4'b1100;
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
  // it. Every write to the MMU's state empties the micro-TLB, and its entries
  // carry the MSR[PR] they were made under, so it never holds what the TLB,
  // the segment registers or the key would no longer give.
  logic        ex_mfsr;                                // mfsr(in) in EX
  wire  [31:0] ea_x    = op_x + op_y;                // = ea for loads and stores
  wire  [31:0] fsr     = fsr_mem[op_y[31:28]];         // mfsr(in)'s copy of the segment registers
  logic [3:0]  db_hit, db_go;
  logic [14:0] db_pa;
  logic [3:0]  db_wimg;
  always_comb begin
    db_pa = 15'd0; db_wimg = 4'd0;
    for (int k = 0; k < 4; k++) begin
      db_hit[k] = bat_match(ea_x[31:17], bat_bepi[4 + k], bat_bl[4 + k],
                            msr_q[MSR_PR] ? bat_vp[4 + k] : bat_vs[4 + k]);
      // BAT protection (PEM 7.4.4): PP = 00 none, 10 read/write, x1 read only
      db_go[k]  = db_hit[k] && (is_st ? bat_pp[4 + k] == 2'd2 : bat_pp[4 + k] != 2'd0);
      db_pa   = db_pa   | ({15{db_hit[k]}} & (bat_brpn[4 + k] | {4'd0, ea_x[27:17] & bat_bl[4 + k]}));
      db_wimg = db_wimg | ({4{db_hit[k]}} & bat_wimg[4 + k]);
    end
  end
  wire         db_any  = |db_hit;
  // micro-TLB: entries filled by MEM's page translation (below), each tagged
  // with the MSR[PR] it was made under (the key depends on it). The tag is
  // compared against x + y without waiting for the adder: x + y = K holds
  // exactly when every bit i has x ^ y ^ K = the carry into i that the sum K
  // would need, (x & y) | ((x ^ y) & ~K) of bit i-1, which needs only bits i
  // and i-1; the carry into bit 12 is the low part's own.
  logic [3:0]        du_v, du_wr, du_pr;
  logic [3:0][19:0]  du_pg, du_pa;
  logic [3:0][3:0]   du_wimg;
  logic [1:0]        du_nx;                            // next entry to fill
  logic [3:0]        du_use;                           // hit, and this access may use it
  logic [19:0]       du_pahi;
  logic [3:0]        du_wimg_s;
  wire  [12:0]       ea_lo = {1'b0, op_x[11:0]} + {1'b0, op_y[11:0]};
  function automatic logic sum_is(input logic [19:0] x, input logic [19:0] y, input logic [19:0] k,
                                  input logic cin);
    logic [19:0] c;
    c = {(x[18:0] & y[18:0]) | ((x[18:0] ^ y[18:0]) & ~k[18:0]), cin};
    sum_is = (x ^ y ^ k) == c;
  endfunction
  always_comb begin
    du_pahi = 20'd0; du_wimg_s = 4'd0;
    for (int k = 0; k < 4; k++) begin
      du_use[k] = du_v[k] && du_pr[k] == msr_q[MSR_PR] && (is_ld || du_wr[k]) &&
                  sum_is(op_x[31:12], op_y[31:12], du_pg[k], ea_lo[12]);
      du_pahi   = du_pahi | ({20{du_use[k]}} & du_pa[k]);
      du_wimg_s = du_wimg_s | ({4{du_use[k]}} & du_wimg[k]);
    end
  end
  wire         d_xl    = msr_q[MSR_DR] && (is_ld || is_st);
  // the access goes ahead from EX when a BAT allows it or the micro-TLB does;
  // anything else, a BAT protection fault included, is translated in MEM
  wire         xl_rp   = d_xl && !((|db_go) || (|du_use));
  wire  [19:0] d_pahi  = !msr_q[MSR_DR] ? ea_x[31:12] : db_any ? {db_pa, ea_x[16:12]} : du_pahi;
  // the access's WIMG: real mode 0011 (603e 3.5), the DBAT's, the micro-TLB's
  wire  [3:0]  d_wimg_x = !msr_q[MSR_DR] ? 4'b0011 : db_any ? db_wimg : du_wimg_s;
  // dcbz to a W or I page takes the alignment exception from MEM (3.7.4,
  // table 3-8). The request goes out regardless (the data cache leaves a dcbz
  // to a W or I page alone, ppc_dcache.sv), so the translation's WIMG is not
  // in front of the request or MEM's fields: EX registers the W or I of the
  // DBAT and of the micro-TLB entry that hit, and MEM chooses (m_algc). A
  // replay finds the WIMG on its refetch.
  logic        db_wi, du_wi;
  always_comb begin
    db_wi = 1'b0; du_wi = 1'b0;
    for (int k = 0; k < 4; k++) begin
      db_wi = db_wi | (db_hit[k] && (bat_wimg[4 + k][3] || bat_wimg[4 + k][2]));
      du_wi = du_wi | (du_use[k] && (du_wimg[k][3] || du_wimg[k][2]));
    end
  end

  // Asynchronous interrupts (4.5.5, 4.5.9), maskable by MSR[EE]; the external
  // interrupt has priority over the decrementer (table 4-3). Taken in place of
  // the first uop of an instruction as it leaves EX, so the boundary is before
  // that instruction. A kill from MEM (m_kill) is older and wins.
  logic        int_q, dec_pend;
  logic        ex_first, ex_ifault, ex_mmuw;
  logic [4:0]  ex_step;
  logic        m_fpxr;                                 // MEM's FP op takes its program exception
  logic        tnop;                                   // MEM's touch faulted: it completes as a no-op
  // (not while WB waits for an older answer: the kill and its commit go
  // together as MEM is left)
  // (a touch whose translation faults completes as a no-op: no kill, tnop)
  wire         m_algc   = m.dz && !m.rp && msr_q[MSR_DR] && (m.dbany ? m.dbwi : m.duwi);
  // (as two products: a replay's, then the rest)
  (* keep = 1 *) wire m_kp1 = m.valid && m.rp && sl_done && !(m.tch && (sl_dsi_q || sl_tm_q));
  (* keep = 1 *) wire m_kp2 = m.valid && (m.resync || m_fpxr || m_algc);
  wire         m_kp     = m_kp1 || m_kp2;
  wire         m_kill   = m_kp && !wb_hold;
  // (where EX advances, MEM does too, so WB is not holding: there m_kill is
  // m_kp, which is formed from registers alone and not behind WB's wait)
  wire         irq_req  = msr_q[MSR_EE] && (int_q || dec_pend);

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
  assign ex_hold = mem_hold || md_wait || fp_wait || ser_wait || sync_wait || (ex_valid && ex_ls && !d_rdy);
  // the queue's copy (q_take above): the same terms, kept apart
  (* keep = 1 *) wire qh_1 = m.valid && wb_valid && wb_iss && !wb_got && !d_valid;
  (* keep = 1 *) wire qh_2 = m.valid && wb_valid && wb_fl && !(wb_got && wb_c2 && !fr_at[0] && !wb_blk);
  (* keep = 1 *) wire qh_3 = m.valid && ((m.rp && !sl_done) || (m.fsm && (fr_at[1] || wb_blk)));
  (* keep = 1 *) wire qh_4 = m.valid && ((m.fpser && !m_fsok) || (m.icbi && !m_ibdone && !icbi_ack));
  (* keep = 1 *) wire qh_x = md_wait || fp_wait || ser_wait || sync_wait || (ex_valid && ex_ls && !d_rdy);
  assign q_hold = qh_1 || qh_2 || qh_3 || qh_4 || qh_x;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && q_hold != ex_hold) $fatal(1, "q_hold");
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
  wire  xc_irq   = irq_req && ex_first;
  wire  xc_fpx   = fpx_cond && ex_first;
  wire  ex_xcm   = ex_ifault || xc_irq || xc_fpx || ex_fpun;

  // the early forward selects' write bits (qe_hc, qf_m, qf_w above)
  wire        qf_mx = ex_u.rd_we && !ex_xcm && !is_ld;   // mx.rd_we && !mx.is_load
  wire        qf_wm = m.rd_we && !m.rp;                  // wb_rd_we as WB takes MEM's uop
  always_ff @(posedge clk) begin
    qf_m <= mem_hold ? qe_hc[2] & {4{m.rd_we && !m.is_load}} : qe_hc[1] & {4{qf_mx}};
    qf_w <= wb_hold  ? qe_hc[3] & {4{wb_rd_we}}              : qe_hc[2] & {4{qf_wm}};
  end

  assign d_req   = ex_adv && (is_ld || (is_st && st_ok)) && !xl_rp && !ex_xcm && !m_kp;
  assign d_we    = is_st && ex_u.cop == 3'd0;
  assign d_addr  = {d_pahi, ea_x[11:2], 2'b00};
  assign d_be    = st_be;                              // a load's lanes too (an inhibited load is made with them)
  assign d_wdata = st_data;
  assign d_wimg  = d_wimg_x;
  assign d_op    = ex_u.cop;

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
  // compared directly. A read whose write lands this cycle takes the write
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
  // the fast port's latency: ppc_fpu_sfma with all five stage boundaries
  // (SQ1 ... SQ5 = 1, R53: L = 6 closes, 5 does not)
  localparam int LS  = 6;

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
  wire         fw_wbp = wb_valid && wb_fpw && (!wb_isld || wb_fpok);
  wire         fw_we  = fw_fpu || fw_wbp;
  wire  [4:0]  fw_idx = fo_valid ? fo_tag : wb_frd;
  wire  [63:0] fw_dat = fo_valid ? fo_res : !wb_isld ? wb_fdat_q : wb_fsl ? lfs_dbl : {fhi_q, wb_dq};
  // the forward into ID's capture: an FPU result or a small-unit result (both
  // from registers); not a load's word (lfs is converted in WB, lfd's second
  // word arrives in MEM), which is read from the file a cycle later
  wire  [63:0] fw_fdt = fo_valid ? fo_res : wb_fdat_q;

  ppc_fpregfile u_frf (
    .clk (clk), .ra (id_fa), .rb (id_fb), .rc (id_fc),
    .qa (rf_fa), .qb (rf_fb), .qc (rf_fc),
    .we (fw_we), .wa (fw_idx), .wd (fw_dat)
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
  logic [1:0]      nx_wfr, nx_haz, hd_haz;
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
        nx_rdy[h][p] = (nx_hit[h][p] || !f_pn(pend, nx_ix[h][p], fo_valid, fo_tag, fw_wbp, wb_frd, ex_fh, ex_frd)) &&
                       !(h == 1 && id_fpw && q_hi[25:21] == nx_ix[h][p]);
      end
      nx_wfr[h] = (!f_pn(pend, nx_d[h], fo_valid, fo_tag, fw_wbp, wb_frd, ex_fh, ex_frd) ||
                   (fr_at[1] && fr_ix[1] == nx_d[h]) || (m_mwb && m.rd == nx_d[h])) &&
                  !(h == 1 && id_fpw && q_hi[25:21] == nx_d[h]);
      nx_haz[h] = (nx_fua[h] && !nx_rdy[h][0]) || (nx_fub[h] && !nx_rdy[h][1]) ||
                  (nx_fuc[h] && !nx_rdy[h][2]) || (nx_fpw[h] && !nx_wfr[h]);
    end
  wire         id_pop = id_take && !redir_hard_q;
  always_ff @(posedge clk) begin
    hd_hit <= nx_hit;
    hd_haz <= nx_haz;
    hd_sel <= id_pop;
    hd_val <= !rst && !redir_hard_q && (id_pop ? q_cnt >= 3'd2 : q_cnt >= 3'd1);
  end
  wire  [2:0]  f_hit_id = {3{hd_val}} & (hd_sel ? hd_hit[1] : hd_hit[0]);
  // a head with nothing formed for it (straight from fetch) waits a cycle if
  // it names an FPR
  always_comb
    fp_haz = id_valid && id_s0 && (hd_val ? (hd_sel ? hd_haz[1] : hd_haz[0]) :
                                            (q_pd.fua || q_pd.fub || q_pd.fuc || q_pd.fpw));
  always_ff @(posedge clk)
    for (int i = 0; i < 32; i++)
      if (rst) pend[i] <= 1'b0;
      else     pend[i] <= (pend[i] && !(fo_valid && fo_tag == 5'(i)) && !(fw_wbp && wb_frd == 5'(i)) &&
                           !(mem_adv && m.fpw && m.rp && m.rd == 5'(i))) ||
                          (fp_go && ex_frw && ex_froh[i]);

  wire  [63:0] fa_cap = f_hit_id[0] ? fw_fdt : rf_fa;
  wire  [63:0] fb_cap = f_hit_id[1] ? fw_fdt : rf_fb;
  wire  [63:0] fc_cap = f_hit_id[2] ? fw_fdt : rf_fc;
  // One register per operand for every FP unit and the store data. Separate
  // copies per unit (the FPU's standalone fits pointed at the operand
  // fanout) measured worse in the core on all three seeds and cost 357
  // registers (fit batches b5 and b7)
  // ex_ffast: the fast port's route for these operand values (D-21)
  logic [63:0] fa_q, fb_q, fc_q;
  logic        ex_ffast;
  logic        ex_ffw;                                 // ex_fma && ex_ffast, for fp_wait alone (fanout)
  always_ff @(posedge clk)
    if (!ex_hold) begin
      fa_q <= fa_cap; fb_q <= fb_cap; fc_q <= fc_cap;
      ex_ffast <= fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_cap, fb_cap, fc_cap);
      ex_ffw   <= id_u.funit == 2'd1 && !q_pd.ifault &&
                  fpu_fast_route(fma_op_t'(id_u.fop), id_u.fsgl, fa_cap, fb_cap, fc_cap);
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
    dq_a   <= fa_q;
    dq_b   <= fb_q;
    dq_op  <= ex_u.fop[1:0];
    dq_sgl <= ex_u.fsgl;
    dq_frd <= ex_frd;
  end
  logic f_fast_ok;                                     // the unit's own issue check (simulation only)
  ppc_fpu_fast #(.TAGW (5), .P1A (P1A), .P2 (P2), .P3 (P3), .P4A (P4A), .P5A (P5A), .P5 (P5), .P6A (P6A), .P6 (P6), .P7 (P7),
                 .SQ1 (1'b1), .SQ2 (1'b1), .SQ3 (1'b1), .SQ4 (1'b1), .SQ5 (1'b1)) u_fpu (
    .clk (clk), .rst (rst),
    .f_valid (ex_fma && fp_go), .f_op (fma_op_t'(ex_u.fop)), .f_single (ex_u.fsgl), .f_fast (ex_ffast),
    .f_ctl (fpscr_q[7:0]), .f_a (fa_q), .f_b (fb_q), .f_c (fc_q), .f_tag (ex_frd), .f_fast_ok (f_fast_ok),
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
    .clk (clk), .op (misc_op_t'(ex_u.fop[2:0])), .ctl (fpscr_q[7:0]), .a (fa_q), .b (fb_q), .c (fc_q),
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
  always_comb
    fp_wait = ex_valid && (
              (ex_fma && fblk_q) ||
              (ex_ffw && ffbl_q) ||
              (ex_div && (fblk_q || fpre_q)) ||
              (ex_fsw && !fs_idle) ||
              (ex_fsx && !ex_fs2) ||
              (ex_f2c && !ex_c2) ||
              (ex_mw && (fr_at[2] || wb_blk)));

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
  // the predecode's raw-word compares against the decoder's op, every cycle
  always_ff @(posedge clk)
    if (!rst && (
        pd_w.w_cr  != (pd_u.rc || pd_u.op == OP_CMP || pd_u.op == OP_CMPL || pd_u.op == OP_CRLOG ||
                       pd_u.op == OP_MTCRF || pd_u.op == OP_MCRXR || (pd_u.op == OP_STORE && pd_u.cond)) ||
        pd_w.w_xer != (pd_u.set_ca || pd_u.oe || pd_u.op == OP_MCRXR || (pd_u.op == OP_MTSPR && pd_spr == 10'd1)) ||
        pd_w.w_lr  != ((pd_u.op == OP_BRANCH && pd_u.lk) || (pd_u.op == OP_MTSPR && pd_spr == 10'd8)) ||
        pd_w.w_ctr != ((pd_u.op == OP_BRANCH && pd_u.bkind != BR_B && pd_u.bkind != BR_BCCTR && !pd_u.bo[2]) ||
                       (pd_u.op == OP_MTSPR && pd_spr == 10'd9)) ||
        pd_w.w_spr != (pd_u.op == OP_MTSPR && pd_spr_hit && pd_spr_idx != 6'd15 && pd_spr_idx != 6'd16) ||
        pd_w.mfcr  != (pd_u.op == OP_MFCR) || pd_w.mfmsr != (pd_u.op == OP_MFMSR) ||
        pd_w.ck    != {pd_u.op == OP_STORE && pd_u.cond, pd_u.op == OP_MTCRF,
                       (pd_u.op == OP_CRLOG && pd_x == 10'd0) || pd_u.op == OP_MCRXR,
                       pd_u.op == OP_CRLOG && pd_x != 10'd0}))
      $fatal(1, "predecode raw compares against the decoder, word %h", i_data);
  // the schedule's invariants, checked in simulation
  always_ff @(posedge clk)
    if (!rst) begin
      if (fo_valid != fr_at[0])          $fatal(1, "FPU result %0d against the schedule %0d", fo_valid, fr_at[0]);
      if (fo_valid && fo_tag != fr_ix[0]) $fatal(1, "FPU result's FPR %0d against the schedule's %0d", fo_tag, fr_ix[0]);
      if (fs_idle && !((fr_at == '0) && !div_out && !d_go && !(m.valid && m.fscmp && !m_c2))) $fatal(1, "fs_idle with FP updates outstanding");
      if (fo_valid && fw_wbp)            $fatal(1, "FPR write port: FPU and WB in one cycle");
      if (int'(fo_valid) + int'(fs_cmp) + int'(fs_ex && fp_go) > 1) $fatal(1, "two FPSCR updates in one cycle: fpu %0d cmp %0d ex %0d (op %0d, m.fpser %0d m_c2 %0d)", fo_valid, fs_cmp, fs_ex && fp_go, ex_fsop, m.fpser, m_c2);
      if (ex_fma && fp_go && d_inj_req && !inj_q) $fatal(1, "multiply-add issued as the divider asks to hand over");
      if (d_go && d_busy)                $fatal(1, "divider started while busy");
      if (fblk_q != (div_out || d_go))   $fatal(1, "fblk_q");
      if (fpre_q != ((fr_at & FPRE) != '0)) $fatal(1, "fpre_q");
      if (ffbl_q != ((fr_at >> LS) != '0)) $fatal(1, "ffbl_q");
      if (ex_ffw != (ex_fma && ex_ffast)) $fatal(1, "ex_ffw");
      if (wb_live_q != (wb_isld && !wb_got)) $fatal(1, "wb_live_q");
      if (f_iss && ex_ffast && !f_fast_ok) $fatal(1, "fast op issued against the unit's own check");
      if (f_iss && ex_ffast != fpu_fast_route(fma_op_t'(ex_u.fop), ex_u.fsgl, fa_q, fb_q, fc_q))
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
  wire [31:0]                r_late  = r_sum | r_rl | r_rr | r_clz;
  (* keep = 1 *) wire [31:0] r_early = r_mid | ({32{ex_rsel[7]}} & md_val) | mf_val;
  (* keep = 1 *) wire [31:0] x_pre   = x_ex ? r_early : x_cap;
  (* keep = 1 *) wire [31:0] y_pre   = y_ex ? r_early : y_cap;
  (* keep = 1 *) wire [31:0] a_pre   = a_ex ? r_early : a_cap;
  (* keep = 1 *) wire [31:0] s_pre   = s_ex ? r_early : s_cap;

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
  wire         cr_lb = ex_u.crop[{cr_a, cr_b}];
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
            cr_n[5'd31 - ex_u.crbd]   = ex_u.crop[{cr_a, cr_b}];
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

    // branch side effects
    if (ex_u.op == OP_BRANCH) begin
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
    mx.req  = mx.req && !ex_xcm;
    mx.rp   = xl_rp && !ex_xcm;
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
    redirect      = 1'b0;
    redirect_pc   = 32'd0;
    // b: a branch that falls through, or a redirect from MEM or WB. Which
    // one is chosen by registers: MEM can kill (m_kp) only where EX cannot
    // redirect (if MEM does not advance, EX holds), and a fired trap reaches
    // WB with EX empty (it waits in EX for MEM and WB to drain), so the late
    // kill decides only the 2-bit choice, not 32 address bits
    redirect_pc_b = wb_trap ? exc_vec | 32'h0700 :
                    m_kp    ? (m.resync ? m.pc + 32'd4 : (m.rp && !sl_flt) ? m.pc : exc_vec | {19'd0, m_xvec}) :
                              ex_u.pc + 32'd4;
    redirect_sel  = 1'b1;
    if (ex_adv) begin
      case (ex_u.op)
        OP_BRANCH:  begin
                      redirect_pc  = br_target;          // not behind the taken decision
                      redirect_sel = br_taken;
                      if (br_taken != ex_pred) redirect = 1'b1;
                    end
        OP_SC:      begin redirect = 1'b1; redirect_pc = exc_vec | 32'h0C00; end
        OP_ILLEGAL: begin redirect = 1'b1; redirect_pc = exc_vec | 32'h0700; end
        // A trap redirects to the next instruction from EX, decided by the
        // opcode alone; if it fired, MEM redirects again to the vector one cycle
        // later (below). The operand compare is then off the fetch path entirely
        // (R23 timing). Nothing younger can be in EX or ID by then: the first
        // redirect flushed them. Traps do not appear in VF3's census (R12).
        OP_TRAP:    begin redirect = 1'b1; redirect_pc = ex_u.pc + 32'd4; end
        OP_RFI:     begin redirect = 1'b1; redirect_pc = {srr0_in[31:2], 2'b00}; end
        OP_MTMSR, OP_ISYNC:
                    begin redirect = 1'b1; redirect_pc = ex_u.pc + 32'd4; end
        default: ;
      endcase
      if (ex_xcm) begin                                // (a kill from MEM overrides it below)
        redirect = 1'b1; redirect_pc = exc_vec | {19'd0, mx.xvec}; redirect_sel = 1'b1;
      end
    end
    // from MEM: a data translation fault, one cycle on, or the refetch after
    // an MMU write (its commit is this cycle, so the refetch, next cycle,
    // translates with it)
    if (m_kill || wb_trap) begin
      redirect     = 1'b1;
      redirect_sel = 1'b0;                             // redirect_pc_b
    end
  end

  // the MSR that fetch continues under, loaded with every redirect (a trap's
  // first redirect, to the next instruction, keeps the MSR; a fired trap's
  // second brings its own)
  wire [31:0] redirect_msr = wb_trap ? wb_tmsr : m_kill ? m_msrx :
                             (ex_u.op == OP_TRAP && !ex_xcm) ? msr_q : mx.msr;
  always_ff @(posedge clk) begin
    if (rst) begin
      f_ir <= 1'b0;
      f_pr <= 1'b0;
      f_tg <= 1'b0;
    end else if (redirect) begin
      f_ir <= redirect_msr[MSR_IR];
      f_pr <= redirect_msr[MSR_PR];
      f_tg <= redirect_msr[MSR_TGPR];
    end
  end


  always_ff @(posedge clk) begin
    if (rst) begin
      ex_valid   <= 1'b0;
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
      if (!ex_hold) begin
        md_have  <= 1'b0;
        ex_valid <= id_adv;
        // the uop's data loads whenever EX is free; only ex_valid waits on the
        // redirect decision (R23 timing)
        ex_u       <= id_u;
        ex_u.op    <= id_opf;                          // (= id_u.op)
        ex_pred    <= id_pred;
        ex_rsel    <= id_rsel;
        ex_ld      <= id_opf == OP_LOAD || h_mfsr;
        ex_ls      <= id_opf == OP_LOAD || id_opf == OP_STORE;
        ex_isync   <= h_isync;
        ex_md      <= id_rsel[7] && !q_pd.ifault;       // a marker's word is not run
        ex_first   <= id_s0;
        ex_step    <= id_step;
        ex_ifault  <= q_pd.ifault;
        // FP: none of it for a fault marker
        ex_fma     <= id_u.funit == 2'd1 && !q_pd.ifault;
        ex_div     <= id_u.funit == 2'd2 && !q_pd.ifault;
        ex_misc    <= id_u.funit == 2'd3 && !q_pd.ifault;
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
        mf_c0      <= q_pd.mfcr && ex_valid && ex_u.rc;
        mf_b       <= id_rsel[8] && id_mf_t == 3'd0 && id_mf_f == 5'd0;
        mf_t       <= id_mf_t;
        ex_ltt     <= id_ltt;
        ex_ext_h   <= id_ext_h;
        op_x       <= ({32{x_ex}} & r_late) | x_pre;
        op_y       <= ({32{y_ex}} & r_late) | y_pre;
        op_a       <= ({32{a_ex}} & r_late) | a_pre;
        op_s       <= ({32{s_ex}} & r_late) | s_pre;
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
        ex_fsx     <= id_u.fsop != 3'd0 && !q_pd.ifault;
        ex_f2c     <= (id_u.fcmp || (id_u.fst && id_u.fsl)) && !q_pd.ifault;
        ex_mw      <= id_u.funit == 2'd3 && id_u.fpw && !q_pd.ifault;
        ex_fsfld   <= id_u.fsop == 3'(FS_MCRFS) ? id_u.insn[20:18] : id_u.insn[25:23];
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
  wire ans_m  = d_valid && !wb_out && m_out;
  // an FP load leaves once its word has been here a cycle (lfs's conversion
  // is registered: wb_c2) and the FPU port is free (wb_fpok)
  assign wb_hold  = wb_valid && ((wb_out && !d_valid) || (wb_fl && !wb_fpok));
  assign wb_adv   = wb_valid && !wb_hold;
  // icbi: its set to the I-cache, MEM held until it is taken
  logic m_ibdone;
  assign icbi_req = m.valid && m.icbi && !m_ibdone;
  assign icbi_set = m.result[11:5];
  // as four products of at most six inputs, then their OR (it feeds EX's
  // hold and the fetch queue's pop): a small-unit result (m.fsm) is never a
  // replay, so !m.rp is not needed with it
  (* keep = 1 *) wire mh_1 = m.valid && wb_valid && wb_iss && !wb_got && !d_valid;
  (* keep = 1 *) wire mh_2 = m.valid && wb_valid && wb_fl && !(wb_got && wb_c2 && !fr_at[0] && !wb_blk);
  (* keep = 1 *) wire mh_3 = m.valid && ((m.rp && !sl_done) || (m.fsm && (fr_at[1] || wb_blk)));
  (* keep = 1 *) wire mh_4 = m.valid && ((m.fpser && !m_fsok) || (m.icbi && !m_ibdone && !icbi_ack));
  assign mem_hold = mh_1 || mh_2 || mh_3 || mh_4;
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
  assign mem_adv  = m.valid && !mem_hold;

  // a load's word to its value: lane, byte reversal, extension
  function automatic logic [31:0] ld_align(input logic [31:0] w, input logic [1:0] sz, input logic [1:0] bo,
                                           input logic sx, input logic rv);
    logic [15:0] h;
    logic [7:0]  b;
    h = bo[1] ? w[15:0] : w[31:16];
    if (rv) h = {h[7:0], h[15:8]};
    b = w[5'd31 - {bo, 3'b000} -: 8];
    case (sz)
      2'd2:    ld_align = rv ? {w[7:0], w[15:8], w[23:16], w[31:24]} : w;
      2'd1:    ld_align = sx ? {{16{h[15]}}, h} : {16'd0, h};
      default: ld_align = {24'd0, b};
    endcase
  endfunction
  // WB's, on the forward path: the word straight from the port
  assign ld_val   = ld_align(d_rdata, wb_msize, wb_boff, wb_msigned, wb_brev);
  // MEM's, for an answer that came while the uop was in MEM: into wb_result
  assign ld_val_m = ld_align(m_got ? m_dq : d_rdata, m.msize, m.boff, m.msigned, m.brev);

  always_ff @(posedge clk) begin
    if (rst) begin
      m.valid <= 1'b0;
    end else if (!mem_hold) begin
      m <= mx;
      m.valid <= ex_adv && !m_kp;
    end
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
  wire [31:0] sl_ea   = m.result;                      // a load's or store's EA
  wire [31:0] sl_sr   = dsr_mem[sl_ea[31:28]];
  wire [62:0] sl_t0   = dtlb0[sl_ea[16:12]];
  wire [62:0] sl_t1   = dtlb1[sl_ea[16:12]];
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
  logic       sl_done, sl_dsi_q, sl_tm_q, sl_prot_q, sl_t_q, sl_key_q, sl_way_q, sl_w_q;
  logic [19:0] sl_pa0_q, sl_pa1_q;                      // both ways' RPN, chosen at the fill
  logic [3:0]  sl_wimg0_q, sl_wimg1_q;                  // and WIMG
  logic        sl_h0_q;
  logic [23:0] sl_vsid_q;
  always_ff @(posedge clk) begin
    if (rst || mem_adv) sl_done <= 1'b0;
    else if (m.valid && m.rp) sl_done <= 1'b1;
    if (!sl_done) begin
      sl_t_q    <= !sl_bat && sl_t;
      sl_prot_q <= sl_prot;
      sl_dsi_q  <= (!sl_bat && sl_t) || sl_prot;
      sl_tm_q   <= !sl_bat && !sl_t && (!sl_hit || (m.st && !sl_prot && !sl_e[6]));
      sl_key_q  <= sl_key;
      // SRR1[WAY]: replacement order, or for a store that hit an entry with
      // C = 0 the way that hit, so the handler's tlbld replaces it
      sl_way_q  <= (sl_hit && m.st) ? !sl_h0 : dlru[sl_ea[16:12]];
      sl_w_q    <= sl_wpp && sl_e[6];                  // a store may use it: PP, key, C
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
  wire        m_ok   = mem_adv && !m_noef;
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
        dar_q   <= m.result;
      end
      if (m_alg) begin                                  // table 4-14, X form; [22-26] from the field
        dsisr_q <= {15'd0, m.insn[2:1], m.insn[6], m.insn[10:7], m.insn[25:21], m.insn[20:16]};
        dar_q   <= m.result;
      end
      if (m_tmiss) begin                                // tables 5-9, 5-11
        if (m.imiss) begin
          imiss_q <= m.pc;
          icmp_q  <= {1'b1, m_vsid, 1'b0, m.pc[27:22]};
        end else begin
          dmiss_q <= m.result;
          dcmp_q  <= {1'b1, m_vsid, 1'b0, m.result[27:22]};
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
  // the data micro-TLB: filled by a replay's translation, emptied by any write
  // to the MMU's state (its entries carry the MSR[PR] they were made under)
  always_ff @(posedge clk) begin
    if (rst || (m_ok && (m.srw || m.tlbie || m.tlbl || m_bat))) begin
      du_v  <= 4'd0;
      du_nx <= 2'd0;
    end else if (mem_adv && m.rp && !sl_flt && !tnop) begin
      du_v[du_nx]  <= 1'b1;
      du_wimg[du_nx] <= sl_h0_q ? sl_wimg0_q : sl_wimg1_q;
      du_pg[du_nx] <= sl_ea[31:12];
      du_pa[du_nx] <= sl_h0_q ? sl_pa0_q : sl_pa1_q;
      du_wr[du_nx] <= sl_w_q;
      du_pr[du_nx] <= msr_q[MSR_PR];
      du_nx        <= du_nx + 2'd1;
    end
  end

  // HID0's cache bits as flip-flops (zero at reset, table 2-2), and the
  // flash-invalidate pulses as HID0 is written with ICFI or DCFI set
  logic [31:0] hid0_q;
  logic        icfi_q, dcfi_q;
  wire         m_hid0 = m_ok && m.spr_we && m.spr == 6'd9;
  always_ff @(posedge clk) begin
    if (rst) begin
      hid0_q <= 32'd0;
      icfi_q <= 1'b0;
      dcfi_q <= 1'b0;
    end else begin
      if (m_hid0) hid0_q <= m.sprv;
      icfi_q <= m_hid0 && m.hcfi[1];
      dcfi_q <= m_hid0 && m.hcfi[0];
    end
  end
  assign hid_ice    = hid0_q[15];
  assign hid_dce    = hid0_q[14];
  assign hid_ilock  = hid0_q[13];
  assign hid_dlock  = hid0_q[12];
  assign hid_icfi   = icfi_q;
  assign hid_dcfi   = dcfi_q;
  assign hid_noopti = hid0_q[0];

  // copies of SDR1, RPA and the BAT fields the translation reads
  always_ff @(posedge clk) begin
    if (m_ok && m.spr_we && m.spr == 6'd8)  sdr1_q <= {m.sprv[31:16], m.sprv[8:0]};
    if (m_ok && m.spr_we && m.spr == 6'd19) rpa_q  <= {m.sprv[31:12], m.sprv[7:3], m.sprv[1:0]};
    if (m_ok && m_bat) begin
      if (!m_batn[0]) begin                             // BATnU: BEPI, BL, Vs, Vp
        bat_bepi[m_batn[3:1]] <= m.sprv[31:17];
        bat_bl[m_batn[3:1]]   <= m.sprv[12:2];
        bat_vs[m_batn[3:1]]   <= m.sprv[1];
        bat_vp[m_batn[3:1]]   <= m.sprv[0];
      end else begin                                    // BATnL: BRPN, WIMG, PP
        bat_brpn[m_batn[3:1]] <= m.sprv[31:17];
        bat_wimg[m_batn[3:1]] <= m.sprv[6:3];
        bat_pp[m_batn[3:1]]   <= m.sprv[1:0];
      end
    end
  end

  always_ff @(posedge clk) begin
    if (m_ok && m.spr_we && m.spr >= 6'd2) spr_mem[m.spr] <= m.sprv;
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
        wb_result <= m.is_load ? ld_val_m : m.result;
        wb_iss    <= m_iss;
        wb_got    <= m_got || ans_m;
        wb_live_q <= m.is_load && !(m_got || ans_m);
        wb_dq     <= m_got ? m_dq : d_rdata;
        wb_isld   <= m.is_load;
        wb_fhi    <= m.fhi && !m.rp;
        wb_msize  <= m.msize;
        wb_boff   <= m.boff;
        wb_msigned <= m.msigned;
        wb_brev   <= m.brev;
        wb_fpw    <= mem_adv && m.fpw && !m.rp;
        wb_fl     <= mem_adv && m.fld && !m.rp;
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
        end
      end
    end
    if (wb_adv && wb_fhi) fhi_q <= ldw;
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
  assign rt_fpq     = pend == 32'd0 && fs_idle && !(m.valid && (m.fsx || m.fany));
  assign rt_fw_en   = fw_we;
  assign rt_fw_idx  = fw_idx;
  assign rt_fw_val  = fw_dat;
  assign rt_dly     = wb_dly;

endmodule
