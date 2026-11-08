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
// ppc_core against the reference model (ppc_iss.h), instruction by instruction.
//
// Each seed builds a random program, loads it into two copies of memory, and
// runs the RTL with random wait states on both ports. Every time the RTL
// retires the last uop of an instruction, the model steps once and the two are
// compared: the retired PC, all 32 GPRs and the four TGPR shadows, CR, XER, LR,
// CTR, MSR, SRR0, SRR1. All of memory is compared when the program reaches its
// final `b .`, and so is the ordered stream of data accesses on both sides
// (each load's word address, each stored byte), which catches an access that
// should not have happened even when it changed nothing. Any difference stops
// the run with both states printed.
//
// Four kinds of program, by seed (seed % 4, or forced with --mode):
//   0 integer      the integer slice in real mode (the board test's kind)
//   1 translated   boot code sets up BATs, segment registers and a hashed
//                  page table, then runs the program with IR = DR = 1 through
//                  the 603e's software TLB reload (the manual's example
//                  handlers, 603e 5.5.2.2.2), with random mapping layouts,
//                  protection faults, page faults, user mode, tlbie, segment
//                  register traffic and fetches that fault
//   2 translated with interrupts: as 1, with the external interrupt and the
//                  decrementer
//   3 interrupts in real mode
//
// Floating point (in every kind of program, 7 seeds in 8): the FPRs are seeded
// from a pool of special values in data memory; FP arithmetic, compares with
// branches on them, FPSCR instructions, mcrfs straight after an op, FP loads
// and stores in every form (a few doubles across a page in translated
// programs, so the second word replays alone), Rc = 1, MSR[FP] = 0 (0x800,
// whose handler turns FP on) and MSR[FE0/FE1] nonzero with random enables
// (precise program exceptions, whose handler returns with FE = 00). FPU
// results land after younger instructions retire, so FPR writes are compared
// per register as the RTL's write port makes them, against the model's in
// program order; whenever the RTL reports its FP state settled (rt_fpq) all
// 32 FPRs and the FPSCR are compared too, and at the end. FPTRACE=1 prints
// every FPR write and FPSCR change on both sides.
//
// Asynchronous interrupts: the RTL chooses the instruction boundary and the
// model takes the interrupt at that same boundary. Each is checked legal when
// it is taken: MSR[EE] = 1, the interrupt requested as the RTL could see it
// (the input two cycles before, the bench's own decrementer model), the
// external interrupt first when both are pending (603e table 4-3); and none
// may stay pending with EE = 1 for more than a few instructions. Instructions
// lost or repeated show as a PC difference. The decrementer and timebase are
// modelled here, cycle by cycle, from the same tick the RTL sees, and supply
// the model's mfdec and mftb.
//
// Data port, pipelined (ppc_core.sv, "Memory ports"): every request is
// queued with its own wait (1 to max_wait, or --dlat n fixed) and answered in
// order, one a cycle, at the request's cycle plus its wait or the cycle after
// the previous answer, whichever is later. --rdy p holds d_rdy low in p% of
// cycles (from a register, as the protocol asks); a request made while it is
// low fails the seed. --ilat n fixes the fetch wait. A record's rt_dly (the
// cycles its uop waited in MEM) moves the cycle at which an interrupt or a
// DEC / timebase read was decided.
//
//   tb_ppc_core [seeds] [first_seed] [length] [max_wait] [--mode n] [--dlat n] [--ilat n] [--rdy p] [--loops] [--fpburst] [--mis] [--xpage]
//
// --alias: update forms whose registers overlap, a third of the memory ops
// (alias_mem, and FP in fp_mem): loads with rD = rB and rA = rB, stores with
// rS = rB, rS = rA and rS = rA = rB, back-to-back updates of one base, some
// misaligned. rD = rB is a valid form (PEM 8, lbzux: EA <- (rA) + (rB),
// rD <- MEM(EA), rA <- EA). Before it the generator never gave a load with
// update an rD equal to its rB (dst() leaves out r3 and r6), so a step 1 that
// read rB after the load had written it went unseen (VF3, lbzux r4,r6,r4).
// The program also ends with the reproducer for each integer load with
// update, D and X form (real mode: lis r4,1; li r6,0x10; lbzux r4,r6,r4 with
// the byte at 0x10010 = 0x55; translated: through 8 seg 6 pages of its own
// (Layout::vrep), each after a tlbie so that step 0 takes a DTLB miss), and
// the bench checks the RTL's
// and the model's rD and rA against the expected values (Gen::expect).

#ifdef PPC_CPU
// rtl/cpu/ppc_cpu.sv: the core with both caches, behind the caches' memory
// agent (sim/cache/cache_mem.h) at first-word latencies 1 to max_wait. Every
// retirement is compared as for the core alone. Not compared: the data
// access stream (the port carries lines) and the final memory (dirty lines
// stay in the D-cache).
//   --snoop n (n = 1 reads, 2 reads and writes; study D-41): a DMA master on
// the data cache's snoop port. A shadow of memory takes every store request
// the core makes, in order; after each sn_ack the master reads the line from
// memory, and each byte must be the shadow's after every request answered by
// the cycle sn_req rose, or a value a request unanswered then or made before
// sn_ack gave it (lines ever accessed cache-inhibited are not checked: an
// inhibited access to a resident line is boundedly undefined, 603e 3.6.4.1).
// A write snoop is followed, as sn_ack comes, by the master's write of the
// line: in real-mode programs, to a data line no request has touched for 400
// cycles, new words, which the model's memory takes too; otherwise the line
// as the requests answered by then left it, which no access can tell apart.
// Reservations: a stwcx. that stores while the snoop rule (ppc_dcache.sv)
// has its reservation lost fails the seed; one that fails where the model
// would succeed is accepted only if that rule cleared the reservation since
// its lwarx, and the model's reservation is then cleared.
#include "Vppc_cpu.h"
#include "cache_mem.h"
using Top = Vppc_cpu;
#else
#include "Vppc_core.h"
using Top = Vppc_core;
#endif
#include "verilated.h"
#include "ppc_iss.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <deque>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {

bool dtrace = getenv("DTRACE") != nullptr;   // every data request, answer and retirement
int g_dlat = 0, g_ilat = 0, g_rdy = 0;
bool g_board = false;
bool g_mis = false;                           // --mis: misaligned accesses (mis_mem)
bool g_xpage = false;                         // --xpage: integer accesses crossing a page, translated (xpage_mem)
bool g_alias = false;                         // --alias: overlapping registers in update forms (alias_mem)
bool g_fpburst = false;                       // --fpburst: multiply-adds, then FP loads behind them (fp_burst, cpi112)
bool g_fpchain = false;                       // --fpchain: chains of dependent single multiply-adds (fp_chain, fp117)
bool g_loops = false;
uint32_t g_intdrop = 40000;   // the interrupt line drops before it is taken, 1 in N clocks (--intdrop, study R104)
int g_snoop = 0;                              // --snoop n (PPC_CPU only)                         // --loops: backward branches, calls and (real mode) rewritten code                         // --emit: the board harness without caches cannot dcbz       // fixed data / fetch wait (0: random), % of cycles with d_rdy low

// ---------------------------------------------------------------- memory
struct Mem {
  std::vector<uint8_t> lo = std::vector<uint8_t>(0x200000);     // 0x00000000 - 0x001FFFFF
  std::vector<uint8_t> hi = std::vector<uint8_t>(0x10000);      // 0xFFF00000 - 0xFFF0FFFF
  uint8_t *at(uint32_t a) {
    if (a < lo.size()) return &lo[a];
    if (a >= 0xFFF00000u && a - 0xFFF00000u < hi.size()) return &hi[a - 0xFFF00000u];
    return nullptr;
  }
  uint8_t rd8(uint32_t a) { uint8_t *p = at(a); return p ? *p : 0xFF; }
  void wr8(uint32_t a, uint8_t v) { if (uint8_t *p = at(a)) *p = v; }
  uint32_t rd32(uint32_t a) { return (rd8(a) << 24) | (rd8(a + 1) << 16) | (rd8(a + 2) << 8) | rd8(a + 3); }
  void wr32(uint32_t a, uint32_t v) { wr8(a, v >> 24); wr8(a + 1, v >> 16); wr8(a + 2, v >> 8); wr8(a + 3, v); }
};

// -------------------------------------------------------------- encoders
uint32_t D(int op, int rt, int ra, uint32_t imm) { return (op << 26) | (rt << 21) | (ra << 16) | (imm & 0xFFFF); }
uint32_t X(int rt, int ra, int rb, int xo, int rc = 0) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (xo << 1) | rc; }
uint32_t XO(int rt, int ra, int rb, int xo9, int oe, int rc) { return (31u << 26) | (rt << 21) | (ra << 16) | (rb << 11) | (oe << 10) | (xo9 << 1) | rc; }
uint32_t M(int op, int rs, int ra, int sh, int mb, int me, int rc) { return (op << 26) | (rs << 21) | (ra << 16) | (sh << 11) | (mb << 6) | (me << 1) | rc; }
uint32_t XL(int bt, int ba, int bb, int xo, int lk = 0) { return (19u << 26) | (bt << 21) | (ba << 16) | (bb << 11) | (xo << 1) | lk; }
uint32_t SPR(int rt, int spr, int xo) { return X(rt, spr & 31, spr >> 5, xo); }
uint32_t B(int32_t off, int aa = 0, int lk = 0) { return (18u << 26) | (uint32_t(off) & 0x03FFFFFC) | (aa << 1) | lk; }
uint32_t BC(int bo, int bi, int32_t off, int lk = 0) { return (16u << 26) | (bo << 21) | (bi << 16) | (uint32_t(off) & 0xFFFC) | lk; }
uint32_t MFSPR(int rt, int n) { return SPR(rt, n, 339); }
uint32_t MTSPR(int n, int rs) { return SPR(rs, n, 467); }
uint32_t MTSR(int sr, int rs) { return X(rs, sr, 0, 210); }
uint32_t MTSRIN(int rs, int rb) { return X(rs, 0, rb, 242); }
uint32_t MFSR(int rt, int sr) { return X(rt, sr, 0, 595); }
uint32_t MFSRIN(int rt, int rb) { return X(rt, 0, rb, 659); }
uint32_t TLBIE(int rb) { return X(0, 0, rb, 306); }
uint32_t TLBLD(int rb) { return X(0, 0, rb, 978); }
uint32_t TLBLI(int rb) { return X(0, 0, rb, 1010); }
uint32_t MTMSR(int rs) { return X(rs, 0, 0, 146); }
uint32_t MFMSR(int rt) { return X(rt, 0, 0, 83); }
uint32_t MTCRF(int crm, int rs) { return (31u << 26) | (rs << 21) | (uint32_t(crm) << 12) | (144u << 1); }
const uint32_t RFI = XL(0, 0, 0, 50), ISYNC = XL(0, 0, 0, 150), SYNC = X(0, 0, 0, 598);
// floating point: A form (frD frA frB frC), X form of opcode 63
uint32_t FA(int op, int d, int a, int b, int c, int xo, int rc = 0) { return (uint32_t(op) << 26) | (d << 21) | (a << 16) | (b << 11) | (c << 6) | (xo << 1) | rc; }
uint32_t FX(int d, int a, int b, int xo, int rc = 0) { return (63u << 26) | (d << 21) | (a << 16) | (b << 11) | (xo << 1) | rc; }
uint32_t MTFSF(int fm, int b, int rc = 0) { return (63u << 26) | (uint32_t(fm) << 17) | (b << 11) | (711u << 1) | rc; }
uint32_t MTFSFI(int crf, int imm, int rc = 0) { return (63u << 26) | (crf << 23) | (imm << 12) | (134u << 1) | rc; }
constexpr uint32_t M_FP = 0x2000, M_FE = 0x0900;

constexpr uint32_t CODE = 0x1000, DATA = 0x100000, END_INSN = 0x48000000;
// FP constant pool in data memory: 64 doubles (special classes, ties, the
// fctiw edges, single-format values and random ones), FPRs seeded from it
constexpr uint32_t FPOOL = DATA + 0x800, FPOOL_N = 64;
// data span the programs use: 64 KiB in the bench, 16 KiB for the board's RAM (--emit)
uint32_t g_span = 0x10000;

// SPR numbers (603e 2.1.2)
enum { DSISR_ = 18, DAR_ = 19, DEC_ = 22, SDR1_ = 25, SRR0_ = 26, SRR1_ = 27, DMISS_ = 976, DCMP_ = 977,
       HASH1_ = 978, HASH2_ = 979, IMISS_ = 980, ICMP_ = 981, RPA_ = 982 };
constexpr uint32_t M_EE = 0x8000, M_PR = 0x4000, M_IP = 0x40, M_IR = 0x20, M_DR = 0x10;
constexpr uint32_t ACK = 0x00180000;          // a store here acknowledges the external interrupt

// --------------------------------------------------------- an assembler
// For the exception handlers: words at addresses, labels, branches resolved last.
struct Asm {
  std::map<uint32_t, uint32_t> w;
  std::map<std::string, uint32_t> lab;
  struct Fix { uint32_t at; std::string to; bool cond; };
  std::vector<Fix> fix;
  uint32_t pc = 0;
  void org(uint32_t a) { pc = a; }
  void l(const std::string &n) { lab[n] = pc; }
  void e(uint32_t x) { w[pc] = x; pc += 4; }
  void b(const std::string &n) { fix.push_back({pc, n, false}); e(B(0)); }
  void bc(int bo, int bi, const std::string &n) { fix.push_back({pc, n, true}); e(BC(bo, bi, 0)); }
  void li32(int r, uint32_t v) { e(D(15, r, 0, v >> 16)); e(D(24, r, r, v & 0xFFFF)); }
  void done() {
    for (auto &f : fix) {
      int32_t off = int32_t(lab.at(f.to) - f.at);
      w[f.at] = f.cond ? (w[f.at] & ~0xFFFCu) | (uint32_t(off) & 0xFFFC) : (w[f.at] & ~0x03FFFFFCu) | (uint32_t(off) & 0x03FFFFFC);
    }
  }
};

// The 603e's three TLB miss handlers, as the manual gives them (603e
// 5.5.2.2.2), at the high vectors. Corrected from the printed code: the SPR
// numbers (976-982, not 1010-1015), `srw r1,r1,8` is srwi, and the TGPR flip
// on the ISI path is xoris 0x0002 (as the DSI path has it, not 0x8000).
void tlb_handlers(Asm &a, uint32_t base) {
  // instruction TLB miss
  a.org(base + 0x1000);
  a.e(MFSPR(2, HASH1_)); a.e(D(14, 1, 0, 8)); a.e(MFSPR(0, 9)); a.e(MFSPR(3, ICMP_)); a.e(D(14, 2, 2, uint32_t(-8)));
  a.l("im0"); a.e(MTSPR(9, 1));
  a.l("im1"); a.e(D(33, 1, 2, 8)); a.e(X(0, 1, 3, 0)); a.bc(0, 2, "im1"); a.bc(4, 2, "isec");
  a.e(D(32, 1, 2, 4)); a.e(D(28, 1, 3, 8)); a.bc(4, 2, "isip");           // guarded: ISI
  a.e(MTSPR(9, 0)); a.e(MFSPR(0, IMISS_)); a.e(MFSPR(3, SRR1_)); a.e(MTCRF(0x80, 3)); a.e(MTSPR(RPA_, 1));
  a.e(D(24, 1, 1, 0x100)); a.e(M(21, 1, 1, 24, 8, 31, 0)); a.e(TLBLI(0)); a.e(D(38, 1, 2, 6)); a.e(RFI);
  a.l("isec"); a.e(D(28, 3, 1, 0x40)); a.bc(4, 2, "isi"); a.e(MFSPR(2, HASH2_)); a.e(D(24, 3, 3, 0x40));
  a.e(D(14, 1, 0, 8)); a.e(D(14, 2, 2, uint32_t(-8))); a.b("im0");
  a.l("isip"); a.e(MFSPR(3, SRR1_)); a.e(D(28, 3, 2, 0xFFFF)); a.e(D(15, 2, 2, 0x0800)); a.b("isi1");
  a.l("isi");  a.e(MFSPR(3, SRR1_)); a.e(D(28, 3, 2, 0xFFFF)); a.e(D(15, 2, 2, 0x4000));
  a.l("isi1"); a.e(MTSPR(9, 0)); a.e(MTSPR(SRR1_, 2)); a.e(MFMSR(0)); a.e(D(27, 0, 0, 0x0002));
  a.e(MTCRF(0x80, 3)); a.e(MTMSR(0)); a.e(B(int32_t(base + 0x400), 1));
  // data TLB miss on load
  a.org(base + 0x1100);
  a.e(MFSPR(2, HASH1_)); a.e(D(14, 1, 0, 8)); a.e(MFSPR(0, 9)); a.e(MFSPR(3, DCMP_)); a.e(D(14, 2, 2, uint32_t(-8)));
  a.l("dm0"); a.e(MTSPR(9, 1));
  a.l("dm1"); a.e(D(33, 1, 2, 8)); a.e(X(0, 1, 3, 0)); a.bc(0, 2, "dm1"); a.bc(4, 2, "dsec");
  a.e(D(32, 1, 2, 4)); a.e(MTSPR(9, 0)); a.e(MFSPR(0, DMISS_)); a.e(MFSPR(3, SRR1_)); a.e(MTCRF(0x80, 3));
  a.e(MTSPR(RPA_, 1)); a.e(D(24, 1, 1, 0x100)); a.e(M(21, 1, 1, 24, 8, 31, 0)); a.e(TLBLD(0)); a.e(D(38, 1, 2, 6)); a.e(RFI);
  a.l("dsec"); a.e(D(28, 3, 1, 0x40)); a.bc(4, 2, "dodsi"); a.e(MFSPR(2, HASH2_)); a.e(D(24, 3, 3, 0x40));
  a.e(D(14, 1, 0, 8)); a.e(D(14, 2, 2, uint32_t(-8))); a.b("dm0");
  // data TLB miss on store, or a store to an entry with C = 0
  a.org(base + 0x1200);
  a.e(MFSPR(2, HASH1_)); a.e(D(14, 1, 0, 8)); a.e(MFSPR(0, 9)); a.e(MFSPR(3, DCMP_)); a.e(D(14, 2, 2, uint32_t(-8)));
  a.l("ceq0"); a.e(MTSPR(9, 1));
  a.l("ceq1"); a.e(D(33, 1, 2, 8)); a.e(X(0, 1, 3, 0)); a.bc(0, 2, "ceq1"); a.bc(4, 2, "csec");
  a.e(D(32, 1, 2, 4)); a.e(D(28, 1, 3, 0x80)); a.bc(12, 2, "cchk");
  a.l("ceq2"); a.e(MTSPR(9, 0)); a.e(MFSPR(0, DMISS_)); a.e(MFSPR(3, SRR1_)); a.e(MTCRF(0x80, 3));
  a.e(MTSPR(RPA_, 1)); a.e(TLBLD(0)); a.e(RFI);
  a.l("csec"); a.e(D(28, 3, 1, 0x40)); a.bc(4, 2, "dodsi"); a.e(MFSPR(2, HASH2_)); a.e(D(24, 3, 3, 0x40));
  a.e(D(14, 1, 0, 8)); a.e(D(14, 2, 2, uint32_t(-8))); a.b("ceq0");
  a.l("cchk"); a.e(M(21, 1, 3, 30, 0, 1, 1)); a.bc(4, 0, "chk0"); a.e(D(28, 1, 3, 1)); a.bc(12, 2, "chk2"); a.b("dodsip");
  a.l("chk0"); a.e(MFSPR(3, SRR1_)); a.e(D(29, 3, 3, 0x0008)); a.bc(12, 2, "chk2"); a.b("dodsip");
  a.l("chk2"); a.e(D(24, 1, 1, 0x180)); a.e(D(44, 1, 2, 6)); a.b("ceq2");
  a.l("dodsi");  a.e(MFSPR(3, SRR1_)); a.e(M(21, 3, 1, 9, 6, 6, 0)); a.e(D(15, 1, 1, 0x4000)); a.b("dsi1");
  a.l("dodsip"); a.e(MFSPR(3, SRR1_)); a.e(M(21, 3, 1, 9, 6, 6, 0)); a.e(D(15, 1, 1, 0x0800));
  a.l("dsi1"); a.e(MTSPR(9, 0)); a.e(D(28, 3, 2, 0xFFFF)); a.e(MTSPR(SRR1_, 2)); a.e(MTSPR(DSISR_, 1));
  a.e(MFSPR(1, DMISS_)); a.e(M(21, 2, 2, 0, 31, 31, 1)); a.bc(12, 2, "dsi2"); a.e(D(26, 1, 1, 0x07));
  a.l("dsi2"); a.e(MTSPR(DAR_, 1)); a.e(MFMSR(0)); a.e(D(27, 0, 0, 0x0002)); a.e(MTCRF(0x80, 3)); a.e(MTMSR(0));
  a.e(B(int32_t(base + 0x300), 1));
}

// ------------------------------------------------------- program layout
// Translated programs (modes 1, 2), physical memory:
//   0x001000  boot code, real mode         0x020000  main code (IBAT or pages)
//   0x050000  frames behind fetch probes   0x100000  data frames (pages)
//   0x140000  data behind DBAT0            0x1C0000 / 0x1F0000  hashed page table
// Virtual:
//   seg 4  0x40020000  main code through IBAT0 (256 KiB block at PA 0), or
//   seg 5  0x50020000  main code through pages, and the same pages again at
//                      0x50000000 (the same ITLB sets: both ways in use)
//   seg 6  r1 0x60340000 and r2 0x60360000: 8 pages each, the same TLB sets
//          (so both ways are used), two VSIDs that map them differently
//   seg 7  0x70080000  DBAT0, 128 KiB at 0x140000
//   seg 3  0x30000000  IBAT1, a fetch probe (PP = 00 or guarded)
//   seg 8  direct-store (T = 1)            seg 9  no page table entries
//   seg 10 no-execute (N = 1)              seg 11 a guarded page and an empty one
struct Layout {
  bool code_bat = false;
  uint32_t vmain = 0;
  uint32_t sr[16] = {};
  uint32_t vsid6[2] = {};
  uint32_t sdr1 = 0;
  uint32_t bat[16] = {};                     // IBAT0U, IBAT0L, ... DBAT3L
  uint32_t vd1 = 0x60340000, vd1b = 0x60360000, vd2 = 0x70080000;
  uint32_t vrep = 0x60350000;                // --alias: the reproducer's 8 pages, read/write under both VSIDs
  uint32_t code_frame[32] = {};
};

struct Gen {
  std::mt19937 rng;
  std::vector<uint32_t> code;                // the main program
  std::vector<uint32_t> boot;                // translated modes: real-mode boot code at CODE
  std::map<uint32_t, uint32_t> image;        // other words: handlers, page table
  int mode;
  bool xl, irq;
  uint32_t base = CODE;                      // the main program's (virtual) address
  uint32_t cur_msr = M_IP;                   // translated and interrupt modes: the MSR here, statically
  uint32_t dec_reload = 0;
  Layout L;
  std::map<uint32_t, uint32_t> pteg_used;    // PTEG address -> slots taken
  bool fp = true;                            // FP instructions in the mix
  std::vector<std::pair<int, uint32_t>> expect;   // --alias: GPR values the program must end with
  uint64_t fpool[FPOOL_N] = {};
  explicit Gen(uint32_t seed, int mode_) : rng(seed), mode(mode_), xl(mode_ == 1 || mode_ == 2), irq(mode_ >= 2) {
    fp = rng() % 8 != 0;
  }

  uint32_t r32() { return rng(); }
  int rnd(int n) { return int(rng() % n); }
  // destinations avoid the bases (r1 r2 r3), the update base (r6) and r9 (handlers)
  int dst() { static const int ok[] = {0,4,5,7,8,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}; return ok[rnd(27)]; }
  int src() { return rnd(32); }
  uint32_t edge() {
    static const uint32_t e[] = {0, 1, 0xFFFFFFFF, 0x80000000, 0x7FFFFFFF, 2, 0xFFFFFFFE, 0x10000};
    return rnd(3) ? r32() : e[rnd(8)];
  }
  void emit(uint32_t w) { code.push_back(w); }
  void li32(int r, uint32_t v) { emit(D(15, r, 0, v >> 16)); emit(D(24, r, r, v & 0xFFFF)); }
  uint32_t here() const { return base + 4 * uint32_t(code.size()); }

  void alu_imm() {
    static const int ops[] = {14, 15, 12, 13, 8, 7, 24, 25, 26, 27, 28, 29};
    int op = ops[rnd(12)];
    if (op >= 24) emit(D(op, src(), dst(), r32()));
    else          emit(D(op, dst(), src(), r32()));
  }
  void alu_xo() {
    static const int ops[] = {266, 10, 138, 202, 234, 40, 8, 136, 200, 232, 104, 235, 75, 11, 491, 459};
    int x = ops[rnd(16)];
    int oe = (x == 75 || x == 11) ? 0 : rnd(2);
    emit(XO(dst(), src(), src(), x, oe, rnd(2)));
  }
  void alu_x() {
    static const int ops[] = {28, 60, 124, 284, 316, 412, 444, 476, 24, 536, 792, 824, 26, 922, 954};
    emit(X(src(), dst(), src(), ops[rnd(15)], rnd(2)));
  }
  void rotate() {
    static const int ops[] = {20, 21, 23};
    emit(M(ops[rnd(3)], src(), dst(), rnd(32), rnd(32), rnd(32), rnd(2)));
  }
  void compare() {
    int bf = rnd(8) << 2;
    switch (rnd(4)) {
      case 0: emit(X(bf, src(), src(), 0)); break;
      case 1: emit(X(bf, src(), src(), 32)); break;
      case 2: emit(D(11, bf, src(), r32())); break;
      default: emit(D(10, bf, src(), r32())); break;
    }
  }
  void crops() {
    static const int ops[] = {33, 129, 193, 225, 257, 289, 417, 449};
    switch (rnd(5)) {
      case 0: emit(XL(rnd(8) << 2, rnd(8) << 2, 0, 0)); break;                   // mcrf
      case 1: emit((31u << 26) | (src() << 21) | (uint32_t(rnd(256)) << 12) | (144u << 1)); break; // mtcrf
      case 2: emit(X(dst(), 0, 0, 19)); break;                                   // mfcr
      case 3: emit(X(rnd(8) << 2, 0, 0, 512)); break;                            // mcrxr
      default: emit(XL(rnd(32), rnd(32), rnd(32), ops[rnd(8)])); break;
    }
  }
  void spr() {
    static const int sprs[] = {1, 8, 9, 272, 273, 274, 275, 18, 19, 25, 1008, 976, 982, 528, 535, 543, 26, 27, 287, 999};
    int n = sprs[rnd(20)];
    // translated: no writes to the BATs in use or SDR1 (the mapping lives there)
    if (rnd(2) || (xl && (n == 25 || n == 528))) emit(SPR(dst(), n, 339));
    else if (n == 1008) { li32(12, hid0_val()); emit(SPR(12, n, 467)); }
    else if (n != 287) emit(SPR(src(), n, 467));
  }
  // HID0 (603e table 2-2): DCE stays on and DCFI off, since what a disabled
  // or flash-invalidated data cache would lose depends on what it holds,
  // which the model does not know; ICE, the locks, ICFI and NOOPTI at random
  // (none of them changes what a program computes)
  uint32_t hid0_val() {
    uint32_t v = (r32() & ~0xFC00u) | 0x4000u;
    if (rnd(8)) v |= 0x8000u;                                   // ICE
    if (!rnd(4)) v |= (rnd(2) ? 0x2000u : 0) | (rnd(2) ? 0x1000u : 0);   // ILOCK, DLOCK
    if (!rnd(6)) v |= 0x0800u;                                  // ICFI
    return v;
  }
  // The MSR the translated and interrupt programs move to: translation as the
  // mode has it, EE mostly on with interrupts, sometimes user mode
  uint32_t pick_msr() {
    uint32_t v = M_IP | (r32() & 0x00002F00);                 // FP FE0 SE BE FE1 at random
    if (rnd(4)) v &= ~M_FE;                                   // mostly FP exceptions ignored
    if (xl) v |= M_IR | M_DR;
    if (irq && rnd(5)) v |= M_EE;
    if (xl && rnd(4) == 0) v |= M_PR;
    return v;
  }
  void set_msr(uint32_t v) { li32(12, v); emit(MTMSR(12)); cur_msr = v; }
  void msr() {
    if (rnd(2)) emit(X(dst(), 0, 0, 83));
    else if (xl || irq) set_msr(pick_msr());
    else {
      // MSR[16-25] at random, IP kept; IR, DR, TGPR and POW stay clear (a
      // random TGPR would put r0-r3 on their shadows, whose use with r4-r31
      // is undefined, 603e table 4-6)
      int r = src(); emit(M(21, r, 7, 0, 15, 25, 0));
      if (rnd(4)) emit(M(21, 7, 7, 0, 24, 19, 0));            // mostly FE0 = FE1 = 0
      emit(D(24, 7, 7, 0x0040)); emit(X(7, 0, 0, 146));
    }
  }
  // an rfi to the next instruction with the given MSR; EE off while SRR0/SRR1 are live
  void rfi_to(uint32_t new_msr, bool srr1_last) {
    // With the code in pages, keep the sequence inside one page: a fetch that
    // misses between the SRR writes and the rfi would overwrite them, on the
    // 603e as here (its handler returns to the rfi with SRR0 pointing at it)
    if (xl && !L.code_bat) {
      uint32_t n = (irq && (cur_msr & M_EE)) ? 10 : 7;
      while (((here() & 0xFFF) + 4 * n) > 0x1000) emit(D(24, 0, 0, 0));           // nop
    }
    if (irq && (cur_msr & M_EE)) set_msr(cur_msr & ~M_EE);
    uint32_t tgt = here() + 4 * 7;
    if (srr1_last) { li32(12, tgt); emit(SPR(12, 26, 467)); li32(12, new_msr); emit(SPR(12, 27, 467)); }
    else           { li32(12, new_msr); emit(SPR(12, 27, 467)); li32(12, tgt); emit(SPR(12, 26, 467)); }
    emit(RFI);
    cur_msr = new_msr;
  }
  uint32_t data_base(int &reg) {           // translated: r1, r2, or r12 loaded with DBAT0's region
    if (!xl) { reg = rnd(2) ? 1 : 2; return reg == 1 ? DATA : DATA + g_span / 2; }
    int k = rnd(3);
    if (k == 2) { li32(12, L.vd2); reg = 12; return L.vd2; }
    reg = k ? 1 : 2;
    return reg == 1 ? L.vd1 : L.vd1b;
  }
  // --mis: misaligned accesses (603e 2.2.3, 4.5.6): halfwords and words of
  // every form, 1-3 bytes past a word, doubleword, cache-line or page
  // boundary (so most cross it), and an in-word halfword at offset 1; with
  // them the forms that take the alignment exception when misaligned (lmw,
  // stmw, lwarx, stwcx.) and, in fp_mem, FP accesses not word-aligned
  uint32_t mis_off(uint32_t room) {
    static const uint32_t bnd[] = {4, 8, 32, 0x1000};
    uint32_t b = bnd[rnd(4)];
    if (rnd(6) == 0) return ((r32() % room) & ~3u) + 1;                       // a halfword inside its word
    return ((r32() % room) & ~(b - 1)) + b - 1 - uint32_t(rnd(3));            // EA & 3 = 3, 2 or 1
  }
  void mis_mem() {
    int base_r;
    uint32_t bv = data_base(base_r);
    int k = rnd(15);
    if (k == 14) {                                       // D-39: across a page inside DBAT0's block, or out of it
      if (!xl) return;
      uint32_t ea = L.vd2 + (rnd(2) ? 0x1000u * uint32_t(1 + rnd(30)) : 0x20000u) - 1 - uint32_t(rnd(3));
      li32(6, ea - 0x100);
      static const int ops[] = {32, 40, 36, 44};
      int op = ops[rnd(4)];
      int rt = (op < 36 || op == 40) ? dst() : src(); if (rt == 6) rt = 7;
      if ((op == 40) && (ea & 3) != 3) { emit(D(op, rt, 6, 0x100 + (3 - (ea & 3)))); return; }   // a halfword at 0xFFF
      emit(D(op, rt, 6, 0x100));
      return;
    }
    if (k >= 12) {                                       // a word stored, then loaded back (VF3's own check):
      // offset 1, 2 or 3 inside a doubleword or across a 32-byte line, D or X form
      uint32_t room = g_span / 4 - 0x2000;
      uint32_t off = (k == 12) ? ((r32() % room) & ~7u) + 1 + uint32_t(rnd(3))
                               : ((r32() % room) & ~31u) + 29 + uint32_t(rnd(3));
      int rt = dst(); if (base_r == 12 && rt == 12) rt = 13;
      if (rnd(2)) { emit(D(36, src(), base_r, off)); emit(D(32, rt, base_r, off)); }
      else { emit(D(14, 3, 0, off)); emit(X(src(), base_r, 3, 151)); if (rt == 3) rt = 7; emit(X(rt, base_r, 3, 23)); }
      return;
    }
    if (k < 5) {                                         // D form
      static const int ops[] = {32, 40, 42, 36, 44};
      int op = ops[rnd(5)];
      bool isld = op == 32 || op == 40 || op == 42;
      int rt = isld ? dst() : src();
      if (base_r == 12 && isld && rt == 12) rt = 13;
      emit(D(op, rt, base_r, mis_off(g_span / 2 - 0x2000)));
    } else if (k < 7) {                                  // update forms through r6
      static const int ops[] = {33, 41, 43, 37, 45};
      int op = ops[rnd(5)];
      li32(6, bv + mis_off(g_span / 2 - 0x2000) - 0x100);
      int rt = dst(); if (rt == 6) rt = 7;
      emit(D(op, (op & 4) ? src() : rt, 6, 0x100));
    } else if (k < 10) {                                 // indexed through r3, plain, update, byte-reversed
      static const int ops[] = {23, 279, 343, 151, 407, 55, 311, 375, 183, 439, 534, 790, 662, 918};
      int x = ops[rnd(14)];
      bool isst = (x == 151 || x == 407 || x == 183 || x == 439 || x == 662 || x == 918);
      bool upd = (x == 55 || x == 311 || x == 375 || x == 183 || x == 439);
      emit(D(14, 3, 0, mis_off(g_span / 4 - 0x2000)));
      int rt = isst ? src() : dst();
      if (upd) { li32(6, bv); if (rt == 6 || rt == 3) rt = 7; emit(X(rt, 6, 3, x)); }
      else { if (base_r == 12 && !isst && rt == 12) rt = 13; emit(X(rt, base_r, 3, x)); }
    } else if (k < 11) {                                 // lwarx / stwcx. misaligned: alignment exception
      emit(D(14, 3, 0, ((r32() % (g_span / 4 - 0x100)) & ~3u) + 1 + uint32_t(rnd(3))));
      if (rnd(2)) { int rt = dst(); if (base_r == 12 && rt == 12) rt = 13; emit(X(rt, base_r, 3, 20)); }
      else emit(X(src(), base_r, 3, 150, 1));
    } else {                                             // lmw / stmw misaligned: alignment exception
      emit(D(rnd(2) ? 46 : 47, 24 + rnd(8), 1, ((r32() % (g_span / 2 - 0x800)) & ~3u) + 1 + uint32_t(rnd(3))));
    }
  }
  // --xpage (translated programs; study R124, xpage): an integer halfword or
  // word crossing a 4 KiB page with MSR[DR] = 1, which the 603e splits with
  // the second word translated on its own page (4.5.3). The crossing is into
  // one of seg 6's data pages (both pages mapped but where the layout left
  // one out), into a page with no entry (the second word's DTLB miss, then the
  // handler's DSI), from a page into DBAT0's block, out of the block into a
  // page, or inside the block (D-39's split), or across a 4 MiB line in seg 6
  // (the second page's API, DCMP, differs). Half the time the second page is
  // first taken out of the TLB (tlbie: its own DTLB miss) and the first
  // page's word touched (so the first word hits); a third of the time the
  // access is made twice (both pages in the micro-TLB). Every load and store
  // form, D and X, update and byte-reversed, and Scud Race's lwzx rD,rD,rB
  // (rD = rA) at its EA FF292FFE's page offset.
  void xpage_mem() {
    int kind = rnd(9);
    uint32_t pgb;                                        // the second page
    if (kind == 8)      pgb = 0x60800000u;                                       // across a 4 MiB line: the API
    else if (kind < 4)  pgb = (rnd(2) ? L.vd1 : L.vd1b) + 0x1000u * uint32_t(1 + rnd(7));
    else if (kind == 4) pgb = (rnd(2) ? L.vd1 : L.vd1b) + 0x8000u;              // no entry
    else if (kind == 5) pgb = L.vd2;                                             // a page into the block
    else if (kind == 6) pgb = L.vd2 + 0x20000u;                                  // out of the block
    else                pgb = L.vd2 + 0x1000u * uint32_t(1 + rnd(31));           // inside it
    bool half = rnd(3) == 0;
    uint32_t ea = half ? pgb - 1 : pgb - 1 - uint32_t(rnd(3));
    if (rnd(4) == 0 && !half) ea = pgb - 2;                                      // Scud Race's offset, FFE
    if (rnd(2)) { li32(12, pgb); emit(TLBIE(12)); emit(SYNC); }
    if (rnd(2)) { int rt = dst(); if (rt == 12) rt = 13; li32(12, ea & ~3u); emit(D(32, rt, 12, 0)); }
    // a quarter of them an FP double at the page's last word (lfd, lfdu, stfd, stfdu, lfdx, lfdux, stfdx,
    // stfdux): it takes the alignment exception when its EA has no DBAT hit (4.5.6.1.1, a page-address
    // translation access), and is two words, each translated on its own, when it has one (inside the block,
    // or leaving it into a page: "no special handling for accesses that fall into BAT regions")
    if (rnd(4) == 0) {
      static const int fdd[] = {50, 51, 54, 55}, fdx[] = {599, 631, 727, 759};
      uint32_t fea = pgb - 4;
      int fr = rnd(32);
      for (int i = 0, n = rnd(3) == 0 ? 2 : 1; i < n; i++) {
        if (rnd(2)) { li32(6, fea - 0x100); emit(D(fdd[rnd(4)], fr, 6, 0x100)); }
        else        { li32(6, fea - 0x40); emit(D(14, 3, 0, 0x40)); emit(X(fr, 6, 3, fdx[rnd(4)])); }
      }
      return;
    }
    static const int wl[] = {32, 33}, ws[] = {36, 37}, hl[] = {40, 41, 42, 43}, hs[] = {44, 45};
    static const int wlx[] = {23, 55, 534}, wsx[] = {151, 183, 662}, hlx[] = {279, 311, 343, 375, 790}, hsx[] = {407, 439, 918};
    bool st = rnd(2), xf = rnd(2);
    int reps = rnd(3) == 0 ? 2 : 1;
    if (!xf) {
      int op = half ? (st ? hs[rnd(2)] : hl[rnd(4)]) : (st ? ws[rnd(2)] : wl[rnd(2)]);
      int rt = st ? src() : dst(); if (rt == 6) rt = 7;
      for (int i = 0; i < reps; i++) { li32(6, ea - 0x100); emit(D(op, rt, 6, 0x100)); }
      return;
    }
    int xo = half ? (st ? hsx[rnd(3)] : hlx[rnd(5)]) : (st ? wsx[rnd(3)] : wlx[rnd(3)]);
    bool upd = xo == 55 || xo == 183 || xo == 311 || xo == 375 || xo == 439;
    if (!st && !upd && rnd(2)) {                         // rD = rA, as Scud Race's lwzx r16,r16,r5
      int rx = dst(), rb = 3; if (rx == 12) rx = 13;
      if (xo == 23 && rnd(2)) { rx = 16; rb = 5; }       // the word itself: 7E10282E
      for (int i = 0; i < reps; i++) { li32(rx, ea - 0x40); emit(D(14, rb, 0, 0x40)); emit(X(rx, rx, rb, xo)); }
      return;
    }
    int rt = st ? src() : dst(); if (rt == 6 || rt == 3) rt = 7;
    for (int i = 0; i < reps; i++) { li32(6, ea - 0x40); emit(D(14, 3, 0, 0x40)); emit(X(rt, 6, 3, xo)); }
  }
  // --alias: the update forms with overlapping registers. Every EA is in the
  // data region; rB, when it is not rA, is a random value with rA = EA - rB.
  void alias_mem() {
    int base_r;
    uint32_t bv = data_base(base_r);
    static const int lx[] = {55, 119, 311, 375}, sx[] = {183, 247, 439}, ld[] = {33, 35, 41, 43}, sd[] = {37, 39, 45};
    auto xsz = [](int xo) { return (xo & 256) ? 2 : (xo & 64) ? 1 : 4; };
    auto dsz = [](int op) { return (op & 8) ? 2 : (op & 2) ? 1 : 4; };
    bool mis = rnd(4) == 0;                              // a quarter misaligned: the split (R86)
    auto ea_of = [&](int sz) {
      uint32_t room = g_span / 2 - 0x2000;
      return bv + (mis && sz > 1 ? mis_off(room) : (r32() % room) & ~uint32_t(sz - 1));
    };
    int k = rnd(8);
    if (k < 2) {                                         // load, rD = rB
      int x = lx[rnd(4)]; uint32_t ea = ea_of(xsz(x)), b = rnd(2) ? r32() : uint32_t(rnd(0x800));
      int rt = dst();
      li32(6, ea - b); li32(rt, b); emit(X(rt, 6, rt, x));
    } else if (k == 2) {                                 // load, rA = rB: EA = 2 x rA
      int x = lx[rnd(4)]; uint32_t ea = ea_of(xsz(x)) & ~1u;
      li32(6, ea >> 1); emit(X(dst(), 6, 6, x));
    } else if (k == 3) {                                 // store, rS = rB
      int x = sx[rnd(3)]; uint32_t ea = ea_of(xsz(x)), b = rnd(2) ? r32() : uint32_t(rnd(0x800));
      int rs = dst();
      li32(6, ea - b); li32(rs, b); emit(X(rs, 6, rs, x));
    } else if (k == 4) {                                 // store, rS = rA: the old rA is stored
      if (rnd(2)) { int op = sd[rnd(3)]; uint32_t ea = ea_of(dsz(op)); li32(6, ea - 0x100); emit(D(op, 6, 6, 0x100)); }
      else { int x = sx[rnd(3)]; uint32_t ea = ea_of(xsz(x)); uint32_t b = uint32_t(rnd(0x4000));
             li32(6, ea - b); li32(3, b); emit(X(6, 6, 3, x)); }
    } else if (k == 5) {                                 // store, rS = rA = rB
      int x = sx[rnd(3)]; uint32_t ea = ea_of(xsz(x)) & ~1u;
      li32(6, ea >> 1); emit(X(6, 6, 6, x));
    } else if (k == 6) {                                 // two loads, rD = rB, back to back through r6, then both used
      int x1 = lx[rnd(4)], x2 = lx[rnd(4)];
      uint32_t ea = ea_of(4) & ~7u, b1 = r32(), b2 = 8 * uint32_t(rnd(64));
      int r1 = dst(), r2 = dst(); while (r2 == r1) r2 = dst();   // (r1 + 1 could be a base: r1, r6)
      li32(6, ea - b1); li32(r1, b1); li32(r2, b2);
      emit(X(r1, 6, r1, x1)); emit(X(r2, 6, r2, x2));
      emit(XO(dst(), 6, r2, 266, 0, rnd(2)));            // add: r6 (the second EA) and the second load
    } else {                                             // D form: two updates of r6 back to back, then r6 as rB
      int o1 = ld[rnd(4)], o2 = ld[rnd(4)];
      uint32_t ea = ea_of(4) & ~7u;
      li32(6, ea - 0x40);
      emit(D(o1, dst(), 6, 0x20)); emit(D(o2, dst(), 6, 0x20));
      int rt = dst(); li32(rt, 0); emit(X(rt, 6, rt, lx[rnd(4)]));   // an X form, rD = rB, from the updated r6
    }
  }
  // --alias: the reproducer for each integer load with update, at the end of
  // the program, with the values rD and rA must hold (expect). In translated
  // programs each goes through its own seg 6 page after a tlbie, so its step
  // 0 takes a DTLB miss and the instruction is run again by the handler's rfi.
  void repro() {
    if (xl && (cur_msr & M_PR)) { emit(17u << 26 | 2); cur_msr &= ~M_PR; }   // sc: supervisor, for tlbie
    static const int op[8]  = {119, 311, 375, 55, 35, 41, 43, 33};   // lbzux lhzux lhaux lwzux, lbzu lhzu lhau lwzu
    static const int rdk[8] = {4, 5, 8, 11, 14, 15, 16, 17};
    static const int rak[8] = {6, 7, 10, 13, 18, 19, 20, 21};
    static const uint32_t w[8] = {0x55667788, 0x8899AABB, 0x8899AABB, 0x12345678,
                                  0xC3A5F00F, 0xF1E2D3C4, 0x8ED0BA98, 0x0BADF00D};
    for (int k = 0; k < 8; k++) {
      uint32_t ea = xl ? L.vrep + 0x10 + 0x1000 * uint32_t(k) : 0x10010 + 0x20 * uint32_t(k);
      li32(12, ea); li32(22, w[k]); emit(D(36, 22, 12, 0));           // stw: the word at EA
      if (xl) { emit(TLBIE(12)); emit(SYNC); }
      int rd = rdk[k], ra = rak[k];
      bool xf = k < 4;
      int sz = xf ? ((op[k] & 256) ? 2 : (op[k] & 64) ? 1 : 4) : ((op[k] & 8) ? 2 : (op[k] & 2) ? 1 : 4);
      bool alg = xf ? op[k] == 375 : op[k] == 43;
      if (xf) {                                          // lis rD,1; addi rA = EA - 0x10000; op rD,rA,rD
        emit(D(15, rd, 0, 1));
        if (!xl && k == 0) emit(D(14, ra, 0, 0x10));     // li r6,0x10: the reproducer as found
        else li32(ra, ea - 0x10000);
        emit(X(rd, ra, rd, op[k]));
      } else {
        li32(ra, ea - 0x10); emit(D(op[k], rd, ra, 0x10));
      }
      uint32_t v = sz == 4 ? w[k] : sz == 2 ? w[k] >> 16 : w[k] >> 24;
      if (alg) v = uint32_t(int32_t(int16_t(v)));
      expect.push_back({rd, v}); expect.push_back({ra, ea});
    }
  }
  void mem() {
    if (g_xpage && xl && rnd(3) == 0) { xpage_mem(); return; }
    if (g_mis && rnd(3) == 0) { mis_mem(); return; }
    if (g_alias && rnd(3) == 0) { alias_mem(); return; }
    int base_r;
    uint32_t bv = data_base(base_r);
    int k = rnd(16);
    if (k < 6) {                                         // D-form loads and stores
      static const int ld[] = {32, 34, 40, 42}, st[] = {36, 38, 44};
      bool isld = rnd(2);
      int op = isld ? ld[rnd(4)] : st[rnd(3)];
      int sz = (op & 8) ? 2 : (op & 2) ? 1 : 4;
      uint32_t off = (r32() % (g_span / 2 - 0x100)) & ~uint32_t(sz - 1);
      int rt = isld ? dst() : src();
      if (base_r == 12 && isld && rt == 12) rt = 13;
      emit(D(op, rt, base_r, off));
    } else if (k < 8) {                                  // update forms through r6
      static const int ops[] = {33, 35, 41, 43, 37, 39, 45};
      int op = ops[rnd(7)];
      int sz = (op & 8) ? 2 : (op & 2) ? 1 : 4;
      li32(6, bv + ((r32() % (g_span / 2 - 0x800)) & ~3u));
      int rt = dst(); if (rt == 6) rt = 7;
      emit(D(op, (op & 4) ? src() : rt, 6, (r32() % 0x400) & ~uint32_t(sz - 1)));
    } else if (k < 11) {                                 // indexed through r3
      static const int ops[] = {23, 87, 279, 343, 151, 215, 407, 534, 790, 662, 918};
      int x = ops[rnd(11)];
      bool isst = (x == 151 || x == 215 || x == 407 || x == 662 || x == 918);
      emit(D(14, 3, 0, (r32() % (g_span / 4 - 0x100)) & ~3u));   // li r3, aligned offset
      int rt = isst ? src() : dst();
      if (base_r == 12 && !isst && rt == 12) rt = 13;
      emit(X(rt, base_r, 3, x));
    } else if (k < 12) {                                 // lwarx / stwcx.
      emit(D(14, 3, 0, (r32() % (g_span / 4 - 0x100)) & ~3u));
      if (rnd(2)) { int rt = dst(); if (base_r == 12 && rt == 12) rt = 13; emit(X(rt, base_r, 3, 20)); }
      emit(X(src(), base_r, 3, 150, 1));
    } else if (k < 14) {                                 // lmw / stmw
      int r = 24 + rnd(8);
      emit(D(rnd(2) ? 46 : 47, r, 1, (r32() % (g_span / 2 - 0x800)) & ~3u));
    } else {                                             // cache operations (603e 3.7), EA r(base) + r3
      emit(D(14, 3, 0, (r32() % (g_span / 2 - 0x100)) & ~3u));
      static const int xo[] = {278, 246, 54, 86, 982, 1014};
      int c = rnd(7);
      if (g_board && c == 5) c = 0;
      // dcbi only of a block dcbf has just made clean: with a cache, what
      // dcbi throws away is then nothing the model would see
      if (c == 6) { emit(X(0, base_r, 3, 86)); emit(X(0, base_r, 3, 470)); }
      else emit(X(0, base_r, 3, xo[c]));
    }
  }
  void muldiv() {
    li32(10, edge()); li32(11, edge());
    static const int ops[] = {235, 75, 11, 491, 459};
    int x = ops[rnd(5)];
    emit(XO(dst(), 10, 11, x, (x == 75 || x == 11) ? 0 : rnd(2), rnd(2)));
  }
  void filler(int n) { for (int i = 0; i < n; i++) alu_imm(); }
  void branch() {
    int k = rnd(6);
    int skip = 1 + rnd(3);
    switch (k) {
      case 0: emit(B(4 * (skip + 1))); filler(skip); break;
      case 1: case 2: emit(BC(rnd(32), rnd(32), 4 * (skip + 1), rnd(2))); filler(skip); break;
      case 3: {                                          // bclr through a computed LR
        emit(BC(20, 31, 4, 1));                          // bcl 20,31,$+4: LR = next
        emit(SPR(12, 8, 339));                           // mflr r12
        emit(D(14, 12, 12, 20));                         // addi r12,r12,20
        emit(SPR(12, 8, 467));                           // mtlr r12
        emit(XL(rnd(32), rnd(32), 0, 16, rnd(2)));       // bclr / bclrl
        filler(1);
        break;
      }
      case 4: {                                          // bcctr through a computed CTR
        emit(BC(20, 31, 4, 1));
        emit(SPR(12, 8, 339));
        emit(D(14, 12, 12, 20));
        emit(SPR(12, 9, 467));                           // mtctr r12
        emit(XL(rnd(32) | 4, rnd(32), 0, 528, rnd(2)));  // bcctr (BO_2 set)
        filler(1);
        break;
      }
      default: {                                         // bdnz-style with a small CTR
        emit(D(14, 12, 0, rnd(3)));
        emit(SPR(12, 9, 467));
        emit(BC(rnd(2) ? 16 : 18, rnd(32), 4 * (skip + 1)));
        filler(skip);
        break;
      }
    }
  }
  // --loops (never in --emit's board program): code run more than once, so the
  // BTB sees its branches again (rtl/cpu/ppc_btb.sv). The bodies are plain
  // operations, so nothing in them writes CTR or LR or branches.
  void body(int n) {
    for (int i = 0; i < n; i++) {
      int k = rnd(10);
      if (fp && k == 0) fp_insn();
      else if (k < 4) alu_imm(); else if (k < 6) alu_xo(); else if (k < 7) alu_x();
      else if (k < 8) rotate(); else if (k < 9) compare(); else mem();
    }
  }
  void loop_form() {
    switch (rnd(xl ? 4 : 5)) {
      case 0: {                                          // counted: bdnz back
        emit(D(14, 12, 0, 2 + rnd(4))); emit(MTSPR(9, 12));
        uint32_t top = here();
        body(1 + rnd(5));
        emit(BC(16, 0, int32_t(top - here())));
        break;
      }
      case 1: {                                          // through LR: blr back, bdz out
        uint32_t top = here() + 20;
        li32(12, top); emit(MTSPR(8, 12)); emit(D(14, 12, 0, 2 + rnd(3))); emit(MTSPR(9, 12));
        body(1 + rnd(4));
        emit(BC(18, 0, 8));
        emit(XL(20, 0, 0, 16));
        break;
      }
      case 2: {                                          // a subroutine called from several sites
        size_t at = code.size(); emit(0);
        uint32_t sub = here();
        body(1 + rnd(3));
        // sometimes first a conditional return the y-bit rule predicts taken
        // (BO 01101, 00101: on a CR bit, y = 1), not always taken
        if (rnd(2)) emit(XL(rnd(2) ? 13 : 5, rnd(32), 0, 16));
        emit(XL(20, 0, 0, 16));                          // blr: a different return each call
        code[at] = B(int32_t(here() - (base + 4 * uint32_t(at))));
        for (int c = 0, nc = 2 + rnd(3); c < nc; c++) {
          if (rnd(3) == 0) { li32(12, sub); emit(MTSPR(9, 12)); emit(XL(20, 0, 0, 528, 1)); }   // bctrl
          else emit(B(int32_t(sub - here()), 0, 1));                                            // bl
          body(rnd(2));
        }
        break;
      }
      case 3: {                                          // a call inside a counted loop
        size_t at = code.size(); emit(0);
        uint32_t sub = here();
        body(1 + rnd(2));
        emit(XL(20, 0, 0, 16));
        code[at] = B(int32_t(here() - (base + 4 * uint32_t(at))));
        emit(D(14, 12, 0, 2 + rnd(3))); emit(MTSPR(9, 12));
        uint32_t top = here();
        body(rnd(3));
        emit(B(int32_t(sub - here()), 0, 1));
        body(rnd(2));
        emit(BC(16, 0, int32_t(top - here())));
        break;
      }
      default: {                                         // real mode: a branch in a loop is rewritten
        int skip = 1 + rnd(2);
        int kind = rnd(3);                               // 0 b, 1 bc always, 2 blr (LR set before the loop)
        size_t lr_at = code.size();                      // (blr: LR jumps over the filler, as b would)
        if (kind == 2) { li32(12, 0); emit(MTSPR(8, 12)); }
        emit(D(14, 12, 0, 2 + rnd(3))); emit(MTSPR(9, 12));
        uint32_t top = here();
        body(rnd(3));
        uint32_t x = here();
        if (kind == 2) {                                 // LR = x + 4 (skip + 1), fixed up now x is known
          uint32_t t = x + 4 * uint32_t(skip + 1);
          code[lr_at] = D(15, 12, 0, t >> 16); code[lr_at + 1] = D(24, 12, 12, t & 0xFFFF);
        }
        emit(kind == 0 ? B(4 * (skip + 1)) : kind == 1 ? BC(20, 0, 4 * (skip + 1)) : XL(20, 0, 0, 16));
        filler(skip);
        body(rnd(2));
        uint32_t w;
        switch (rnd(4)) {
          case 0:  w = D(24, 0, 0, 0); break;                     // a nop
          case 1:  w = B(4); break;                               // another target
          case 2:  w = BC(4, rnd(32), 4 * (skip + 1)); break;     // a bc the rule predicts not taken
          default: w = BC(20, 0, 4 * skip); break;                // b... one word short
        }
        li32(12, x); li32(11, w); emit(D(36, 11, 12, 0));          // stw: the next pass runs w
        emit(X(0, 0, 12, 54)); emit(SYNC); emit(X(0, 0, 12, 982)); emit(ISYNC);   // dcbst sync icbi isync
        emit(BC(16, 0, int32_t(top - here())));
        break;
      }
    }
  }
  void exc() {
    switch (rnd(5)) {
      case 4: {                                    // rfi straight after its SRR writes
        if (xl || irq) { rfi_to(pick_msr(), rnd(2)); break; }
        uint32_t tgt = CODE + 4 * uint32_t(code.size() + 7), srr1 = (r32() & ~(M_IR | M_DR)) | 0x40;
        bool srr1_last = rnd(2);
        if (srr1_last) { li32(12, tgt); emit(SPR(12, 26, 467)); li32(12, srr1); emit(SPR(12, 27, 467)); }
        else           { li32(12, srr1); emit(SPR(12, 27, 467)); li32(12, tgt); emit(SPR(12, 26, 467)); }
        emit(XL(0, 0, 0, 50));
        break;
      }
      case 0: emit(17u << 26 | 2); cur_msr &= ~M_PR; break;             // sc (back to supervisor)
      case 1: emit(D(3, rnd(32), src(), r32())); break;                 // twi
      case 2: emit(X(rnd(32), src(), src(), 4)); break;                 // tw
      default: {                                                         // illegal: reserved, fsqrt, fsqrts
        static const uint32_t ill[] = {1u << 26, FA(63, 1, 0, 2, 0, 22), FA(59, 3, 0, 4, 0, 22), FX(5, 0, 6, 1000)};
        if (rnd(2)) emit(ill[rnd(4)]);
        else        fp_illegal();
        break;
      }
    }
  }

  // ------------------------------------------------------- floating point
  int fr() { return rnd(32); }
  // an unassigned opcode 59 / 63 encoding (illegal whatever MSR[FP] is): half
  // of them an A-form op on the other opcode (fsqrt(s), fsels, fres on 63,
  // frsqrtes), the rest any other field values; every draw the reference
  // model decodes as an FP op is drawn again
  void fp_illegal() {
    static const int near[] = {22, 23, 24, 26};
    uint32_t w;
    do {
      w = (rnd(2) ? 59u : 63u) << 26 | (r32() & 0x03FFFFFFu);
      if (rnd(2)) w = (w & ~0x3Eu) | uint32_t(near[rnd(4)]) << 1;
    } while (PpcIss::fp_op(w) >= 0);
    emit(w);
  }
  // a source FPR: often the last multiply-add result, so a consumer meets
  // its producer in flight (the scoreboard, the forward into ID's capture
  // and the fast port's route from forwarded values)
  int last_fd = 0;
  int fs() { return rnd(4) == 0 ? last_fd : fr(); }
  void fp_madd(int rc) {
    static const int xo[] = {20, 21, 25, 28, 29, 30, 31};
    int x = xo[rnd(7)], op = rnd(2) ? 59 : 63, d = fr();
    emit(FA(op, d, fs(), x == 25 ? 0 : fs(), (x == 20 || x == 21) ? 0 : fs(), x, rc));
    last_fd = d;
  }
  void fp_arith() {
    int rc = rnd(8) == 0;
    int k = rnd(44);
    if (k < 22) {                                        // multiply-add family, both precisions
      fp_madd(rc);
    } else if (k >= 40) {                                // in flight while an FP access and a move pass
      fp_madd(rc);
      fp_mem();
      static const int xo[] = {72, 40, 264, 136};
      emit(FX(fr(), 0, fs(), xo[rnd(4)], rnd(8) == 0));
    } else if (k < 25) {                                 // frsp fctiw fctiwz
      static const int xo[] = {12, 14, 15};
      emit(FX(fr(), 0, fr(), xo[rnd(3)], rc));
    } else if (k < 31) {                                 // fmr fneg fabs fnabs fsel
      static const int xo[] = {72, 40, 264, 136};
      if (rnd(5)) emit(FX(fr(), 0, fr(), xo[rnd(4)], rc));
      else emit(FA(63, fr(), fr(), fr(), fr(), 23, rc));
    } else if (k < 35) {                                 // fcmpu fcmpo, then often a branch on it
      int bf = rnd(8);
      emit(FX(bf << 2, fr(), fr(), rnd(2) ? 32 : 0));
      if (rnd(2)) { int skip = 1 + rnd(2); emit(BC(rnd(2) ? 12 : 4, bf * 4 + rnd(4), 4 * (skip + 1))); filler(skip); }
    } else if (k < 37) {                                 // fdiv(s), fres: the divider
      switch (rnd(3)) {
        case 0: emit(FA(63, fr(), fr(), fr(), 0, 18, rc)); break;
        case 1: emit(FA(59, fr(), fr(), fr(), 0, 18, rc)); break;
        default: emit(FA(59, fr(), 0, fr(), 0, 24, rc)); break;
      }
    } else if (k < 38) {
      emit(FA(63, fr(), 0, fr(), 0, 26, rc));            // frsqrte
    } else {                                             // an op, then FPRF through mcrfs (R30)
      static const int xo[] = {21, 25, 29};
      int x = xo[rnd(3)];
      emit(FA(rnd(2) ? 59 : 63, fr(), fr(), x == 25 ? 0 : fr(), x == 21 ? 0 : fr(), x, 0));
      emit(FX(rnd(8) << 2, (rnd(2) ? 4 : 3) << 2, 0, 64));          // mcrfs crD, cr4 / cr3
    }
  }
  void fp_fpscr() {
    int rc = rnd(6) == 0;
    switch (rnd(8)) {
      case 0: emit(FX(fr(), 0, 0, 583, rc)); break;                                     // mffs
      case 1: {                                                                         // mtfsf
        emit(MTFSF(rnd(4) ? 0xFF : rnd(256), fr(), rc));
        if (rnd(3)) emit(MTFSFI(6, rnd(4) ? 0 : rnd(16)));                              // mostly no enables
        break;
      }
      case 2: emit(MTFSFI(rnd(8), rnd(16), rc)); break;
      case 3: emit(MTFSFI(7, rnd(16), rc)); break;                                      // XE NI RN
      case 4: emit(FX(rnd(32), 0, 0, 70, rc)); break;                                   // mtfsb0
      case 5: { int b = rnd(32); if (b >= 24 && b <= 28 && rnd(2)) b = 31; emit(FX(b, 0, 0, 38, rc)); break; }   // mtfsb1
      default: emit(FX(rnd(8) << 2, rnd(8) << 2, 0, 64)); break;                       // mcrfs
    }
  }
  void fp_mem() {
    int base_r;
    uint32_t bv = data_base(base_r);
    if (g_mis && rnd(8) == 0) {                          // not word-aligned: alignment exception
      static const int ops[] = {48, 50, 52, 54};
      emit(D(ops[rnd(4)], fr(), base_r, ((r32() % (g_span / 2 - 0x100)) & ~7u) + 1 + uint32_t(rnd(7))));
      if (rnd(2)) return;
    }
    if (g_alias && rnd(3) == 0) {                        // update forms, rA = rB: EA = 2 x rA
      static const int xo[] = {567, 631, 695, 759};      // lfsux lfdux stfsux stfdux
      uint32_t ea = bv + ((r32() % (g_span / 2 - 0x800)) & ~7u);
      li32(6, ea >> 1); emit(X(fr(), 6, 6, xo[rnd(4)]));
      if (rnd(2)) emit(XO(dst(), 6, 6, 266, 0, 0));       // r6 straight after: the EA
      return;
    }
    bool pool = !xl && rnd(2);
    int k = rnd(10);
    if (xl && rnd(16) == 0) {
      // a double across a page boundary, so its second word replays (or
      // faults) alone. The 603e takes an alignment exception for this in
      // page-translated memory (603e 4.5.6.1); the core has none yet, and
      // both models make two word accesses
      li32(6, (rnd(2) ? L.vd1 : L.vd1b) + 0x1000 * uint32_t(rnd(7)) + 0xFFC);
      emit(D(rnd(2) ? 50 : 54, fr(), 6, 0));
      return;
    }
    if (k < 5) {                                         // D form
      static const int ops[] = {48, 50, 52, 54};
      int op = ops[rnd(4)];
      bool dbl = op & 2;
      uint32_t off = pool && op < 52 ? (FPOOL - DATA) + 8 * uint32_t(rnd(FPOOL_N)) + (dbl ? 0 : 4 * uint32_t(rnd(2)))
                                     : (r32() % (g_span / 2 - 0x100)) & ~uint32_t(dbl ? 7 : 3);
      if (pool) { base_r = 1; }
      emit(D(op, fr(), base_r, off));
    } else if (k < 7) {                                  // update forms through r6
      static const int ops[] = {49, 51, 53, 55};
      int op = ops[rnd(4)];
      li32(6, bv + ((r32() % (g_span / 2 - 0x800)) & ~7u));
      emit(D(op, fr(), 6, (r32() % 0x400) & ~7u));
    } else {                                             // indexed through r3, plain and update
      static const int xo[] = {535, 599, 663, 727, 983, 567, 631, 695, 759};
      int x = xo[rnd(9)];
      bool upd = x & 32 && x != 983;
      if (upd) { li32(6, bv); emit(D(14, 3, 0, (r32() % (g_span / 4 - 0x100)) & ~7u)); emit(X(fr(), 6, 3, x)); }
      else { emit(D(14, 3, 0, (r32() % (g_span / 4 - 0x100)) & ~7u)); emit(X(fr(), base_r, 3, x)); }
    }
  }
  // --fpburst (cpi112): multiply-add ops, then FP loads behind them, so loads reach WB while results come
  // back on the FPR write port and the load write slot must wait (ppc_core FLW), and consumers right after
  void fp_burst() {
    int n = 1 + rnd(4);
    for (int i = 0; i < n; i++) fp_madd(0);
    int m = 2 + rnd(4);
    for (int j = 0; j < m; j++) fp_mem();
    if (rnd(2)) fp_madd(0);
    if (rnd(2)) fp_arith();
  }
  // --fpchain (fp117): single-precision multiply-adds reading the last results (one, two or three back) and often
  // writing one of their own sources or the FPR still being written (WAW), with now and then a move, a store, a
  // compare, an frsqrte or a double op among them: the early forward (ppc_core FEF) at every distance and the
  // relaxed WAW rule (FWW), on single values (the pool's), so most of them take the fast port
  int fd_hist[3] = {0, 0, 0};
  int fcs() { int k = rnd(8); return k < 3 ? fd_hist[0] : k < 5 ? fd_hist[1] : k < 6 ? fd_hist[2] : fr(); }
  void fp_chain() {
    int n = 2 + rnd(7);
    for (int i = 0; i < n; i++) {
      static const int xo[] = {20, 21, 25, 28, 29, 30, 31};
      int x = xo[rnd(7)], k = rnd(16);
      int a = fcs(), b = (x == 25) ? 0 : fcs(), c = (x == 20 || x == 21) ? 0 : fcs();
      int d = k < 5 ? a : k < 8 ? (x == 25 ? a : b) : k < 10 ? fd_hist[0] : fr();
      if (k == 10) { emit(FX(fr(), 0, fd_hist[0], 72, 0)); continue; }                        // fmr of the last result
      if (k == 11) { fp_mem(); continue; }
      if (k == 12 && rnd(2)) { emit(FX(fr(), 0, fd_hist[0], 26, 0)); continue; }           // frsqrte
      emit(FA(rnd(8) ? 59 : 63, d, a, b, c, x, rnd(16) == 0));
      fd_hist[2] = fd_hist[1]; fd_hist[1] = fd_hist[0]; fd_hist[0] = d; last_fd = d;
    }
  }
  void fp_insn() {
    if (g_fpchain && rnd(2) == 0) { fp_chain(); return; }
    if (g_fpburst && rnd(3) == 0) { fp_burst(); return; }
    int k = rnd(20);
    if (k < 10) fp_arith();
    else if (k < 17) fp_mem();
    else fp_fpscr();
  }
  // the pool: classes the census found (R30) and the edges the units have
  void make_fpool() {
    static const uint64_t sp[] = {
      0, 0x8000000000000000ull, 0x7FF0000000000000ull, 0xFFF0000000000000ull,         // zeros, infinities
      0x7FF8000000000000ull, 0xFFF8000000000123ull, 0x7FF0000000000001ull, 0x7FF4000000000000ull,   // QNaN, SNaN
      0x0000000000000001ull, 0x800FFFFFFFFFFFFFull, 0x0010000000000000ull, 0x7FEFFFFFFFFFFFFFull,   // denormal, min, max
      0x3FF0000000000000ull, 0xBFF0000000000000ull, 0x3FE0000000000000ull, 0x4004000000000000ull,   // 1 -1 .5 2.5
      0x41DFFFFFFFE00000ull, 0x41E0000000000000ull, 0xC1E0000000000000ull, 0x41DFFFFFFFF00000ull,   // fctiw edges
      0xC1E0000000100000ull, 0x36A0000000000000ull, 0x3810000000000000ull, 0x47EFFFFFE0000000ull,   // single edges
      0x3800000000000000ull, 0x380FFFFFE0000000ull, 0x47F0000000000000ull, 0x3FF8000000000000ull,
    };
    int n = 0;
    for (uint64_t v : sp) fpool[n++] = v;
    while (n < int(FPOOL_N)) {
      switch (rnd(3)) {
        case 0: fpool[n++] = ppcfpu::lfs(r32()); break;                                  // single values
        case 1: fpool[n++] = (uint64_t(r32()) << 32) | r32(); break;
        default: fpool[n++] = ppcfpu::lfs((r32() & 0x807FFFFF) | ((0x60 + rnd(0x40)) << 23)); break;   // near 1
      }
    }
    fpool[FPOOL_N - 1] = uint32_t(rnd(4) ? rnd(4) : r32() & 0xFFFFF0FF);                 // FPSCR seed: RN, or random
  }
  // FPRs from the pool and the FPSCR from its last entry; MSR[FP] on (the
  // seed code runs in real mode with physical addresses)
  void init_fp() {
    if (!fp) return;
    make_fpool();
    emit(MFMSR(12)); emit(D(24, 12, 12, M_FP)); emit(MTMSR(12));
    li32(12, FPOOL);
    for (int f = 0; f < 32; f++) emit(D(50, f, 12, 8 * uint32_t(rnd(FPOOL_N - 1))));
    emit(D(50, 0, 12, 8 * (FPOOL_N - 1))); emit(MTFSF(0xFF, 0));
    emit(D(50, 0, 12, 8 * uint32_t(rnd(FPOOL_N - 1))));
    if (!xl && !irq && rnd(4) == 0) { emit(MFMSR(12)); emit(M(21, 12, 12, 0, 19, 17, 0)); emit(MTMSR(12)); }   // FP off: 0x800 first
  }

  // ---------------------------------------------- translation-only traffic
  void mmu() {
    switch (rnd(18)) {
      case 0: emit(MFSR(dst(), rnd(16))); break;
      case 1: { li32(12, r32()); int d = dst(); emit(MFSRIN(d == 12 ? 13 : d, 12)); break; }
      case 2: { int k = rnd(16); if (k == 6) k = 9; li32(12, L.sr[k]);           // rewrite, same value
                if (rnd(2)) emit(MTSR(k, 12)); else { li32(13, uint32_t(k) << 28 | (r32() & 0x0FFFFFFF)); emit(MTSRIN(12, 13)); }
                break; }
      case 3: li32(12, (L.sr[6] & ~0xFFFFFFu) | L.vsid6[rnd(2)]); emit(MTSR(6, 12)); break;   // remap seg 6
      case 4: {                                    // tlbie: data pages, code pages, anything
        uint32_t ea = rnd(3) == 0 ? r32() : rnd(2) ? (rnd(2) ? L.vd1 : L.vd1b) + (uint32_t(rnd(8)) << 12)
                                                   : L.vmain + (uint32_t(rnd(4)) << 12);
        li32(12, ea); emit(TLBIE(12)); if (rnd(2)) emit(SYNC);
        break;
      }
      case 5: case 6: {                            // direct-store or entry-less segment: DSI
        li32(12, (rnd(2) ? 0x80000000u : 0x90000000u) | (r32() & 0x0FFFFFFC));
        emit(rnd(2) ? D(32, 13, 12, 0) : D(36, src(), 12, 0));
        break;
      }
      case 7: case 8: {                            // a fetch that faults: ISI, then back through LR
        static const uint32_t tgt[] = {0x80001000, 0xA0001000, 0xB0001000, 0xB0002000, 0x30001000, 0x90004000};
        li32(12, tgt[rnd(6)] + 4 * uint32_t(rnd(64)));
        emit(SPR(12, 9, 467)); emit(XL(20, 0, 0, 528, 1));                  // mtctr; bcctrl
        break;
      }
      case 9: if (!(cur_msr & M_PR)) rfi_to(cur_msr | M_PR, rnd(2)); break; // into user mode
      case 10: emit(rnd(2) ? SYNC : ISYNC); break;
      case 11: {                                   // HASH1/HASH2 straight after a write to their inputs
        static const int n[] = {DMISS_, DCMP_, IMISS_, ICMP_};
        li32(12, r32()); emit(MTSPR(n[rnd(4)], 12)); emit(MFSPR(dst(), rnd(2) ? HASH1_ : HASH2_));
        break;
      }
      case 12: { static const int n[] = {DMISS_, DCMP_, HASH1_, HASH2_, IMISS_, ICMP_, RPA_, DSISR_, DAR_};
                 emit(MFSPR(dst(), n[rnd(9)])); break; }
      case 13: emit(MFSPR(dst(), rnd(2) ? DEC_ : (rnd(2) ? 268 : 269))); break;
      case 14: {                                   // a BAT rewrite with its own value, or unused BATs
        int k = rnd(3);
        if (k == 0) { li32(12, L.bat[8]); emit(MTSPR(536, 12)); }
        else { li32(12, r32() & ~3u); emit(MTSPR(k == 1 ? 534 : 542, 12)); li32(12, r32()); emit(MTSPR(k == 1 ? 535 : 543, 12)); }
        break;
      }
      case 15: if (irq) { li32(12, 20 + rnd(400)); emit(MTSPR(DEC_, 12)); } break;
      case 16:                                     // code in pages: to the other alias (EA bit 17
        if (!L.code_bat) {                         // flipped: the same ITLB set, the other EA[4-14])
          emit(BC(20, 31, 4, 1)); emit(SPR(12, 8, 339)); emit(D(27, 12, 12, 0x0002));
          emit(D(14, 12, 12, 20)); emit(SPR(12, 9, 467)); emit(XL(20, 0, 0, 528));
          break;
        }
        emit(SPR(dst(), SDR1_, 339));
        break;
      default: emit(SPR(dst(), SDR1_, 339)); break;
    }
  }

  // interrupts in real mode: DEC and timebase reads, mtdec
  void timers() {
    switch (rnd(3)) {
      case 0: emit(MFSPR(dst(), DEC_)); break;
      case 1: emit(SPR(dst(), rnd(2) ? 268 : 269, 371)); break;            // mftb / mftbu
      default: li32(12, 20 + rnd(400)); emit(MTSPR(DEC_, 12)); break;
    }
  }

  // ------------------------------------------------------------ page table
  uint32_t pteg_of(uint32_t vsid, uint32_t ea, bool sec) const {
    uint32_t h = (vsid & 0x7FFFF) ^ ((ea >> 12) & 0xFFFF);
    if (sec) h = ~h & 0x7FFFF;
    uint32_t s = L.sdr1;
    return (s & 0xFE000000) | ((((s >> 16) & 0x1FF) | ((h >> 10) & s & 0x1FF)) << 16) | ((h & 0x3FF) << 6);
  }
  // one PTE (PEM 7.6.1): word 0 V | VSID | H | API, word 1 RPN | R | C | WIMG | PP
  void pte(uint32_t vsid, uint32_t ea, uint32_t pa, uint32_t wimg, uint32_t pp, bool r, bool c) {
    bool sec = rnd(4) == 0;
    for (int tries = 0; tries < 2; tries++, sec = !sec) {
      uint32_t g = pteg_of(vsid, ea, sec);
      uint32_t &used = pteg_used[g];
      if (used == 0xFF) continue;
      int slot; do slot = rnd(8); while (used & (1u << slot));
      used |= 1u << slot;
      image[g + 8 * slot]     = 0x80000000 | (vsid << 7) | (uint32_t(sec) << 6) | ((ea >> 22) & 0x3F);
      image[g + 8 * slot + 4] = (pa & 0xFFFFF000) | (uint32_t(r) << 8) | (uint32_t(c) << 7) | (wimg << 3) | pp;
      return;
    }
  }
  void junk_ptes(int n) {                         // entries that never match: wrong VSID, or V = 0
    for (int i = 0; i < n; i++) {
      uint32_t g = pteg_of(r32() & 0xFFFFFF, r32(), rnd(2));
      uint32_t &used = pteg_used[g];
      if (used == 0xFF) continue;
      int slot; do slot = rnd(8); while (used & (1u << slot));
      used |= 1u << slot;
      image[g + 8 * slot]     = (rnd(4) ? 0x80000000 : 0) | (r32() & 0x7FFFFFFF);
      image[g + 8 * slot + 4] = r32();
    }
  }

  // the translated layout: segment registers, BATs, page table (see Layout)
  void layout() {
    L.code_bat = rnd(2);
    L.vmain = L.code_bat ? 0x40020000 : 0x50020000;
    base = L.vmain;
    for (int k = 0; k < 16; k++) L.sr[k] = (r32() & 0x60FFFFFF);            // Ks Kp at random, T = N = 0
    L.sr[5] = 0x20000000 | (r32() & 0xFFFFFF);                               // Ks = 0, Kp = 1
    L.vsid6[0] = r32() & 0xFFFFFF; L.vsid6[1] = r32() & 0xFFFFFF;
    L.sr[6] = (r32() & 0x60000000) | L.vsid6[0];
    L.sr[8] = 0x80000000 | (r32() & 0x7FFFFFFF);                            // T = 1
    L.sr[10] = 0x10000000 | (r32() & 0x60FFFFFF);                           // N = 1
    bool big = rnd(2);                                                       // 64 or 128 KiB table
    L.sdr1 = big ? 0x001C0001 : 0x001F0000;
    // BATs: BEPI | BL | Vs | Vp, BRPN | WIMG | PP
    if (L.code_bat) {
      L.bat[0] = 0x40000000 | (1u << 2) | 3;
      L.bat[1] = 0x00000000 | ((r32() & 6) << 3) | (rnd(2) ? 2 : rnd(2) ? 1 : 3);
    }
    L.bat[2] = 0x30000000 | 2 | 1;                                           // IBAT1 probe: no access or guarded
    L.bat[3] = 0x00060000 | (rnd(2) ? 0 : (0x8 | 2));
    static const uint32_t dpp[] = {2, 2, 2, 2, 1, 3};
    L.bat[8] = L.vd2 | 2 | uint32_t(rnd(2));                                 // DBAT0: Vs, Vp at random
    L.bat[9] = 0x00140000 | ((r32() & 7) << 3) | dpp[rnd(6)];
    // page table: main code (segment 5), data (segment 6, both VSIDs), probes (segment 10, 11)
    for (int i = 0; i < 32; i++) L.code_frame[i] = 0x20000 + 0x1000 * uint32_t(i);
    for (int i = 31; i > 0; i--) std::swap(L.code_frame[i], L.code_frame[rnd(i + 1)]);
    if (!L.code_bat)
      for (int p = 0; p < 32; p++) {
        static const uint32_t cpp[] = {2, 2, 1, 3};
        for (uint32_t alias : {0u, 0x20000u})     // the program at 0x50020000 and at 0x50000000
          pte(L.sr[5] & 0xFFFFFF, (L.vmain ^ alias) + 0x1000 * uint32_t(p), L.code_frame[p], (r32() & 6), cpp[rnd(4)], rnd(2), rnd(2));
      }
    std::vector<uint32_t> frames;
    for (uint32_t f = 0x100000; f < 0x140000; f += 0x1000) frames.push_back(f);
    for (size_t i = frames.size() - 1; i > 0; i--) std::swap(frames[i], frames[rnd(int(i) + 1)]);
    size_t fi = 0;
    for (int v = 0; v < 2; v++)
      for (uint32_t win : {L.vd1, L.vd1b})
        for (int p = 0; p < 8; p++) {
          if (rnd(10) == 0) continue;                                        // no entry: a page fault
          static const uint32_t pps[] = {2, 2, 2, 0, 1, 3};
          pte(L.vsid6[v], win + 0x1000 * uint32_t(p), frames[fi++ % frames.size()], (r32() & 6), pps[rnd(6)], rnd(2), rnd(2));
        }
    if (g_alias)                                 // frames 40-47 of the 64: the data pages take at most 32
      for (int v = 0; v < 2; v++)
        for (int p = 0; p < 8; p++) pte(L.vsid6[v], L.vrep + 0x1000 * uint32_t(p), frames[40 + p], (r32() & 6), 2, 1, 1);
    if (g_xpage) {                               // --xpage: a page below DBAT0's block, and (half the time) one above
      static const uint32_t xpp[] = {2, 2, 1, 3};
      pte(L.sr[7] & 0xFFFFFF, L.vd2 - 0x1000, frames[50], (r32() & 6), xpp[rnd(4)], rnd(2), rnd(2));
      if (rnd(2)) pte(L.sr[7] & 0xFFFFFF, L.vd2 + 0x20000, frames[51], (r32() & 6), xpp[rnd(4)], rnd(2), rnd(2));
      for (int v = 0; v < 2; v++) {              // seg 6's 0x607FF000 and 0x60800000, either VSID: a crossing whose
        pte(L.vsid6[v], 0x607FF000u, frames[52 + 2 * v], (r32() & 6), xpp[rnd(4)], rnd(2), rnd(2));   // pages differ
        pte(L.vsid6[v], 0x60800000u, frames[53 + 2 * v], (r32() & 6), xpp[rnd(4)], rnd(2), rnd(2));   // in EA[4-9]
      }
    }
    for (int p = 0; p < 64; p++) pte(L.sr[10] & 0xFFFFFF, 0xA0000000 + 0x1000 * uint32_t(p), 0x50000, 0, 2, 1, 0);
    pte(L.sr[11] & 0xFFFFFF, 0xB0001000, 0x51000, 1, 2, 1, 0);              // guarded: the handler's ISI
    junk_ptes(300);
  }

  // translated: real-mode boot at CODE, then rfi into the program
  void boot_code() {
    std::swap(code, boot);
    uint32_t save = base; base = CODE;
    emit(D(14, 12, 0, 0));
    for (int n = 528; n < 544; n++) emit(MTSPR(n, 12));                     // every BAT invalid (5.3)
    for (int s = 0; s < 32; s++) { emit(TLBIE(12)); emit(D(14, 12, 12, 0x1000)); }   // every TLB set (5.4.3.1)
    for (int k = 0; k < 16; k++) {
      li32(12, L.sr[k]);
      if (rnd(2)) emit(MTSR(k, 12)); else { li32(13, uint32_t(k) << 28); emit(MTSRIN(12, 13)); }
    }
    li32(12, L.sdr1); emit(MTSPR(SDR1_, 12));
    for (int n = 0; n < 16; n++) if (L.bat[n]) { li32(12, L.bat[n]); emit(MTSPR(528 + n, 12)); }
    std::swap(code, boot);
    base = save;
  }

  void init_regs() {
    uint32_t d1 = xl ? L.vd1 : DATA, d2 = xl ? L.vd1b : DATA + g_span / 2;
    for (int r = 0; r < 32; r++) if (r != 1 && r != 2 && r != 3) li32(r, edge());
    li32(1, d1); li32(2, d2); emit(D(14, 3, 0, 0x100));
    li32(12, r32()); emit(SPR(12, 9, 467)); emit(SPR(12, 8, 467));
    li32(12, r32() & 0xE000007F); emit(SPR(12, 1, 467));
    // SPRs other than LR/CTR/XER are undefined at reset, on the 603e and in the
    // core; write every one a program may read, so a run cannot depend on the
    // previous one (the board test repeats the program without a power cycle)
    for (int n : {272, 273, 274, 275, 18, 19, 25, 1008, 976, 982, 528, 535, 543, 26, 27}) {
      if (xl && (n == 25 || n == 528)) continue;
      li32(12, n == 1008 ? 0x0000C000u : r32()); emit(SPR(12, n, 467));   // HID0: both caches on
    }
    if (irq) { li32(12, 100 + rnd(2000)); emit(MTSPR(DEC_, 12)); }
    init_fp();
  }

  void build(int len) {
    if (xl) {
      layout();
      boot_code();
      std::swap(code, boot); uint32_t save = base; base = CODE;
      init_regs();
      uint32_t m0 = pick_msr() & ~M_PR;
      li32(12, L.vmain); emit(MTSPR(SRR0_, 12)); li32(12, m0); emit(MTSPR(SRR1_, 12)); emit(RFI);
      std::swap(code, boot); base = save;
      cur_msr = m0;
    } else {
      init_regs();
      if (irq) set_msr(pick_msr());
    }
    if (irq) dec_reload = (g_mis ? 600 : 50) + rnd(1500);   // (--mis: no period shorter than the handler at wait 80)
    while (int(code.size()) < len) {
      if (fp && rnd(100) < 25) { fp_insn(); continue; }
      int k = rnd(100);
      if      (k < 20) alu_imm();
      else if (k < 34) alu_xo();
      else if (k < 46) alu_x();
      else if (k < 54) rotate();
      else if (k < 60) compare();
      else if (k < 66) { if (xl && rnd(2)) mmu(); else if (irq && !xl && rnd(3) == 0) timers(); else crops(); }
      else if (k < 70) spr();
      else if (k < 72) msr();
      else if (k < 86) mem();
      else if (k < 89) muldiv();
      else if (k < 97) { if (g_loops && rnd(3) == 0) loop_form(); else branch(); }
      else exc();
    }
    if (g_alias) repro();
    if (fp) emit(FX(fr(), 0, 0, 583));                    // mffs: the FP state settles before the end
    emit(END_INSN);
  }

  // where each word of the main program lives
  uint32_t phys(uint32_t i) const {
    uint32_t va = base + 4 * i;
    if (!xl) return va;
    if (L.code_bat) return va - 0x40000000;
    return L.code_frame[(va - L.vmain) >> 12] | (va & 0xFFF);
  }
};

const uint32_t PROG_HANDLER[9] = {
  SPR(9, 27, 339), M(21, 9, 9, 0, 24, 19, 0), SPR(9, 27, 467),   // mfsrr1; clear FE0 SE BE FE1; mtsrr1
  D(29, 9, 9, 0x0001), SPR(9, 26, 339), BC(4, 2, 8),             // andis. r9,r9,1 (SRR1[15]); mfsrr0; bne +8
  D(14, 9, 9, 4), SPR(9, 26, 467), XL(0, 0, 0, 50)};             // addi; mtsrr0; rfi
const uint32_t FPUN_HANDLER[4] = {SPR(9, 27, 339), D(24, 9, 9, M_FP), SPR(9, 27, 467), XL(0, 0, 0, 50)};

void install(Mem &m, const Gen &g, std::mt19937 &rng) {
  for (uint32_t a = DATA; a < DATA + g_span; a++) m.wr8(a, uint8_t(rng()));
  if (g.xl) for (uint32_t a = DATA + g_span; a < 0x148000; a++) m.wr8(a, uint8_t(rng()));
  m.wr32(0xFFF00100, B(CODE, 1));                                 // ba CODE
  if (g.fp) for (uint32_t i = 0; i < FPOOL_N; i++) { m.wr32(FPOOL + 8 * i, uint32_t(g.fpool[i] >> 32)); m.wr32(FPOOL + 8 * i + 4, uint32_t(g.fpool[i])); }
  for (uint32_t base : {0x00000000u, 0xFFF00000u}) {
    // program exception: return with FE0 = FE1 = 0, past the instruction
    // unless SRR0 is already past it (SRR1[15]: an FP exception left by an
    // MSR write); clobbers CR0
    for (uint32_t i = 0; i < 9; i++) m.wr32(base + 0x700 + 4 * i, PROG_HANDLER[i]);
    // FP unavailable: back to the instruction with MSR[FP] = 1
    for (uint32_t i = 0; i < 4; i++) m.wr32(base + 0x800 + 4 * i, FPUN_HANDLER[i]);
    m.wr32(base + 0xC00, XL(0, 0, 0, 50));                        // sc: rfi (SRR0 is pc+4)
  }
  if (g.xl || g.irq || g_mis || g_alias) {
    Asm a;
    const uint32_t hb = 0xFFF00000;
    // DSI: read DSISR and DAR (compared through r9), skip the instruction
    a.org(hb + 0x300); a.e(MFSPR(9, DSISR_)); a.e(MFSPR(9, DAR_)); a.e(MFSPR(9, SRR0_)); a.e(D(14, 9, 9, 4));
    a.e(MTSPR(SRR0_, 9)); a.e(RFI);
    // alignment (dcbz to a W or I page): read DSISR and DAR, skip it
    a.org(hb + 0x600); a.e(MFSPR(9, DSISR_)); a.e(MFSPR(9, DAR_)); a.e(MFSPR(9, SRR0_)); a.e(D(14, 9, 9, 4));
    a.e(MTSPR(SRR0_, 9)); a.e(RFI);
    // ISI: the faulting fetch came from a bcctrl; return through LR
    a.org(hb + 0x400); a.e(MFSPR(9, SRR1_)); a.e(MFSPR(9, 8)); a.e(MTSPR(SRR0_, 9)); a.e(RFI);
    // external: acknowledge (the bench drops INT), return
    a.org(hb + 0x500); a.e(D(15, 9, 0, ACK >> 16)); a.e(D(36, 9, 9, 0)); a.e(RFI);
    // decrementer: read it, reload it, return
    a.org(hb + 0x900); a.e(MFSPR(9, DEC_)); a.li32(9, g.dec_reload); a.e(MTSPR(DEC_, 9)); a.e(RFI);
    // system call: back in supervisor mode
    a.org(hb + 0xC00); a.e(MFSPR(9, SRR1_)); a.e(M(21, 9, 9, 0, 18, 16, 0)); a.e(MTSPR(SRR1_, 9)); a.e(RFI);
    if (g.xl) tlb_handlers(a, hb);
    a.done();
    for (auto &w : a.w) m.wr32(w.first, w.second);
  }
  if (g.xl) {
    for (size_t i = 0; i < g.boot.size(); i++) m.wr32(CODE + 4 * uint32_t(i), g.boot[i]);
    for (auto &w : g.image) m.wr32(w.first, w.second);
  }
  for (size_t i = 0; i < g.code.size(); i++) m.wr32(g.phys(uint32_t(i)), g.code[i]);
}

// ------------------------------------------------------------- coverage
struct Cov {
  uint64_t vec[0x20] = {};                    // exceptions taken, by vector / 0x100
  uint64_t ext = 0, dec = 0, sh_writes = 0, seeds[4] = {}, replays = 0, fslow = 0;
  uint64_t btb[4] = {};                       // ev_btb: predicted targets, failed checks, MEM refetches, writes
  uint64_t insns[4] = {}, cycles[4] = {};
  uint64_t fpc = 0, fprec = 0, fpdel = 0, fpsupp = 0, fpld = 0, fpst = 0, fpops[ppcfpu::NUM_OPS] = {};
  uint64_t mis[18] = {};                      // misaligned accesses done (PpcIss::mis_n)
  uint64_t xrf = 0;                           // ev_xrf: a split's first word made again after its second replayed
  // update forms completed with overlapping registers: integer load rD = rB,
  // load rA = rB (FP too), integer store rS = rB, rS = rA, store rA = rB (FP
  // too); and the reproducer's values checked (--alias)
  uint64_t alias[5] = {}, alias_chk = 0;
  uint64_t sn_n[2] = {}, sn_b2b = 0, sn_eager = 0, sn_bytes = 0, sn_unchk = 0, sn_dmaw_new = 0, sn_dmaw_same = 0;
  uint64_t sn_lat = 0, sn_lat_max = 0, sn_rsv_lost = 0;
  void add(const Cov &c) {
    for (int i = 0; i < 2; i++) sn_n[i] += c.sn_n[i];
    sn_b2b += c.sn_b2b; sn_eager += c.sn_eager; sn_bytes += c.sn_bytes; sn_unchk += c.sn_unchk;
    sn_dmaw_new += c.sn_dmaw_new; sn_dmaw_same += c.sn_dmaw_same; sn_lat += c.sn_lat; sn_rsv_lost += c.sn_rsv_lost;
    sn_lat_max = std::max(sn_lat_max, c.sn_lat_max);
    fpc += c.fpc; fprec += c.fprec; fpdel += c.fpdel; fpsupp += c.fpsupp; fpld += c.fpld; fpst += c.fpst;
    for (int i = 0; i < ppcfpu::NUM_OPS; i++) fpops[i] += c.fpops[i];
    for (int i = 0; i < 0x20; i++) vec[i] += c.vec[i];
    for (int i = 0; i < 18; i++) mis[i] += c.mis[i];
    xrf += c.xrf;
    for (int i = 0; i < 5; i++) alias[i] += c.alias[i];
    alias_chk += c.alias_chk;
    ext += c.ext; dec += c.dec; sh_writes += c.sh_writes; replays += c.replays; fslow += c.fslow;
    for (int i = 0; i < 4; i++) btb[i] += c.btb[i];
    for (int i = 0; i < 4; i++) { insns[i] += c.insns[i]; cycles[i] += c.cycles[i]; }
    for (int i = 0; i < 4; i++) seeds[i] += c.seeds[i];
  }
};
Cov g_cov;

// ------------------------------------------------------------- one seed
bool run(uint32_t seed, int len, int max_wait, int mode, uint64_t &insns, uint64_t &cycles) {
  Gen g(seed, mode);
  g.build(len);
  std::mt19937 drng(seed ^ 0x9E3779B9u);
  Mem rtl_mem, iss_mem;
  {
    std::mt19937 a(seed * 7 + 1), b(seed * 7 + 1);
    install(rtl_mem, g, a);
    install(iss_mem, g, b);
  }

  // Every data access, in order, on both sides: a load's word address, a
  // store's bytes. An access the RTL makes and the model does not (a uop that
  // was killed, replaced or replayed and still reached memory) shows here even
  // when it changed nothing, as it would matter on I/O.
  std::vector<uint64_t> acc_rtl, acc_iss;
  size_t acc_last_n = 0;                               // stream entries of the last data request (ev_xrf)
  auto acc_ld = [](uint32_t a) { return (uint64_t(1) << 40) | a; };
  auto acc_st = [](uint32_t a, uint8_t v) { return (uint64_t(2) << 40) | (uint64_t(v) << 32) | a; };
  PpcIss iss;
  // FPR writes: the RTL's in the order its write port makes them (FPU
  // results complete after younger instructions retire), the model's in
  // program order; matched per register (no two writes of one FPR are ever
  // outstanding together: the scoreboard's WAW stall)
  std::vector<uint64_t> fq_rtl[32], fq_iss[32];
  uint64_t rtl_fpr[32] = {};
  uint64_t fp_checks = 0;
  static const bool fptrace = getenv("FPTRACE");
  iss.on_fpw = [&](int r, uint64_t v) { fq_iss[r].push_back(v); if (fptrace) printf("    model f%d = %016llX (pc %08X)\n", r, (unsigned long long)v, iss.pc); };
  auto fp_match = [&](int r) -> bool {
    while (!fq_rtl[r].empty() && !fq_iss[r].empty()) {
      if (fq_rtl[r].front() != fq_iss[r].front()) {
        printf("seed %u: f%d written %016llX by the RTL, %016llX by the model\n", seed, r,
               (unsigned long long)fq_rtl[r].front(), (unsigned long long)fq_iss[r].front());
        return false;
      }
      fq_rtl[r].erase(fq_rtl[r].begin()); fq_iss[r].erase(fq_iss[r].begin());
      fp_checks++;
    }
    return true;
  };
  iss.rd8 = [&](uint32_t a) { return iss_mem.rd8(a); };
  iss.wr8 = [&](uint32_t a, uint8_t v) { iss_mem.wr8(a, v); acc_iss.push_back(acc_st(a, v)); };
  iss.on_load = [&](uint32_t a) { acc_iss.push_back(acc_ld(a)); };

  // The decrementer, timebase and INT as the RTL sees them, cycle by cycle.
  // The RTL reads DEC and TB in EX, three cycles before the record (plus the
  // cycles its uop then waited in MEM, rt_dly), from the value after the edge
  // before that; it decides an interrupt there too, from INT registered one
  // cycle earlier still. History is kept per edge: dec_h, tb_h and pend_h are
  // written a cycle late (the timers lag, below), int_h as INT is driven.
  const int H = 4096;
  uint32_t dec = 0xFFFFFFFF, dec_prev = 0xFFFFFFFF; uint64_t tb = 0; bool pend = false, int_drv = false;
  static uint32_t dec_h[H]; static uint64_t tb_h[H]; static bool pend_h[H], int_h[H];
  for (int i = 0; i < H; i++) { dec_h[i] = 0xFFFFFFFF; tb_h[i] = 0; pend_h[i] = false; int_h[i] = false; }
  uint64_t cyc_now = 0;
  int cur_dly = 0;
  auto hist = [&](uint64_t back) { return int((cyc_now + H - back) % H); };
  iss.dec_read  = [&]() { return dec_h[hist(2 + cur_dly)]; };
  iss.dec_write = [&](uint32_t v) { dec = v; };
  iss.tb_read   = [&](bool hi) { return uint32_t(hi ? tb_h[hist(2 + cur_dly)] >> 32 : tb_h[hist(2 + cur_dly)]); };
  iss.tb_write  = [&](bool hi, uint32_t v) { tb = hi ? (tb & 0xFFFFFFFFull) | (uint64_t(v) << 32) : (tb & ~0xFFFFFFFFull) | v; };
  bool irq_mode = g.irq;
  std::mt19937 irng(seed ^ 0x2545F491u);
  int tick_div = 1 + int(irng() % 6);
  uint32_t int_rate = g_mis ? 150 + irng() % 600 : 1500 + irng() % 6000;   // (--mis: often, to meet a split's second cycle)
  int pend_run = 0;
  uint64_t insns_here = 0;
  Cov cov; cov.seeds[mode]++;

  Top top;
  uint32_t gpr[36] = {};
  std::mt19937 lat(seed ^ 0x51ED270Bu);
  auto wait_for = [&]() { return max_wait ? 1 + int(lat() % max_wait) : 1; };
  std::mt19937 rrng(seed ^ 0x7F4A7C15u);

  bool i_pend = false;
  int i_cnt = 0;
  uint32_t i_word = 0, i_word2 = 0;
  bool i_two = false;                                  // two-word fetch offered this cycle (random)
  struct DAns { uint64_t t; uint32_t word; };
  std::deque<DAns> dq;                                 // data answers outstanding, in order
  uint64_t d_last = 0;                                 // cycle of the latest answer scheduled
#ifdef TB_LU2
  uint32_t e_prev = 0;                                 // last cycle's d_eword (LU2, below)
  std::mt19937 e_rng(seed ^ 0x5A5A1717u);              // its own stream: the others' are as without LU2
#endif
  bool d_rdy = true;

#ifdef PPC_CPU
  Memory cmem;
  for (uint32_t a = 0; a < rtl_mem.lo.size(); a += 4) cmem.w[a >> 2] = rtl_mem.rd32(a);
  for (uint32_t a = 0; a < rtl_mem.hi.size(); a += 4) cmem.w[(0xFFF00000u + a) >> 2] = rtl_mem.rd32(0xFFF00000u + a);
  std::mt19937 mrng(seed ^ 0x6A09E667u);
  MemAgent ma; ma.rng = &mrng; ma.mem = &cmem; ma.lat_lo = 1; ma.lat_hi = max_wait > 1 ? max_wait : 1;
  top.clk = 0; top.rst = 1; top.m_ack = 0; top.m_rvalid = 0; top.m_wnext = 0; top.m_rdata = 0; top.tb_tick = 0; top.ext_int = 0;
  top.sn_req = 0; top.sn_addr = 0; top.sn_inv = 0;
  // ---- the DMA master (--snoop)
  Mem smem = rtl_mem;                                  // memory as the core's requests have written it
  struct BLog { uint64_t seq; uint32_t a; uint8_t old, nw; };
  std::deque<BLog> blog;                               // the latest bytes written, by request number
  uint64_t n_seq = 0, n_ans = 0;
  std::map<uint32_t, uint64_t> line_req;               // by line: the cycle of the latest request
  std::set<uint32_t> line_inh;                         // lines accessed cache-inhibited
  std::deque<uint32_t> rq_lines;
  uint32_t last_st_line = 0; uint64_t last_st_cyc = 0;
  std::mt19937 srng(seed ^ 0x3C6EF372u);
  auto srnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(srng); };
  struct Snoop { bool act = false, inv = false, chk = true; uint32_t line = 0; uint64_t t0 = 0; uint8_t v0[32];
                 std::vector<std::pair<int, uint8_t>> maybe; } sn;
  int sn_gap = 40;
  uint64_t last_ack = 0;
  bool m_rsvq = false, m_rvkill = false; uint32_t m_rvpa = 0, m_rsvline = 0;
  std::vector<bool> lw_kill;                           // by lwarx request: its reservation was lost since
  uint64_t n_lw_ret = 0;
  auto view_at = [&](uint32_t line, uint64_t k, uint8_t *v) {   // the line after the first k requests
    for (int b = 0; b < 32; b++) v[b] = smem.rd8(line + b);
    for (auto it = blog.rbegin(); it != blog.rend() && it->seq >= k; ++it)
      if ((it->a & ~31u) == line) v[it->a & 31] = it->old;
  };
#else
  top.clk = 0; top.rst = 1; top.i_valid = 0; top.d_valid = 0; top.tb_tick = 0; top.ext_int = 0;
  top.i_way = 0; top.i_arr = 0; top.i_pa = 0;          // no I-cache: the BTB's pre-read is not used
#endif
  for (int i = 0; i < 4; i++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;

  // the last retirements, printed when a seed fails
  struct Rec { uint32_t pc, insn, msr, srr0, srr1; int irq; uint64_t cyc; };
  Rec ring[48] = {}; unsigned nring = 0;
  auto dump = [&]() {
    for (unsigned i = nring > 48 ? nring - 48 : 0; i < nring; i++) {
      const Rec &r = ring[i % 48];
      printf("  %8llu  %08X  %08X  msr %08X srr0 %08X srr1 %08X%s\n", (unsigned long long)r.cyc, r.pc, r.insn,
             r.msr, r.srr0, r.srr1, r.irq ? (r.irq == 1 ? "  external" : "  decrementer") : "");
    }
  };
  struct Dumper { std::function<void()> f; bool armed = true; ~Dumper() { if (armed) f(); } } dumper{dump};

  uint64_t idle = 0;
  for (cycles = 0; cycles < 200ull * len + 200000; cycles++) {
    cyc_now = cycles;
    // inputs for this cycle
    bool tick = irq_mode && (cycles % uint64_t(tick_div)) == 0;
    if (irq_mode && !int_drv && irng() % int_rate == 0) int_drv = true;
    else if (int_drv && irng() % g_intdrop == 0) int_drv = false;          // dropped before it is taken (--intdrop N: 1 in N clocks)
    top.tb_tick = tick;
    top.ext_int = int_drv;
    int_h[hist(0)] = int_drv;
    // responses for this cycle
#ifdef PPC_CPU
    bool sn_inw = false;                                 // a write snoop is in progress in the cache this cycle
    if (g_snoop) {
      bool sack = top.sn_ack;
      sn_inw = sn.act && sn.inv && cycles > sn.t0 && !sack;
      if (sack && !sn.act) { printf("seed %u: sn_ack with no snoop at cycle %llu\n", seed, (unsigned long long)cycles); return false; }
      if (sn.act && sack) {
        if (sn.chk) {
          for (int b = 0; b < 32; b++) {
            uint8_t m = uint8_t(cmem.rd((sn.line + b) & ~3u) >> (24 - 8 * (b & 3)));
            bool ok = m == sn.v0[b];
            for (auto &mv : sn.maybe) ok = ok || (mv.first == b && mv.second == m);
            cov.sn_bytes++;
            if (!ok) {
              printf("seed %u: DMA %s of line %08X byte %d reads %02X, expected %02X (%zu later values), snoop %llu-%llu\n", seed,
                     sn.inv ? "write snoop" : "read", sn.line, b, m, sn.v0[b], sn.maybe.size(), (unsigned long long)sn.t0,
                     (unsigned long long)cycles);
              return false;
            }
          }
        } else cov.sn_unchk++;
        if (sn.inv) {                                    // the master's write
          auto it = line_req.find(sn.line);
          bool quiet_line = !g.xl && sn.line >= DATA && sn.line + 32 <= DATA + g_span &&
                            (it == line_req.end() || cycles - it->second > 400 + (cycles - sn.t0));
          uint8_t v[32];
          view_at(sn.line, n_ans, v);
          if (quiet_line) {
            int mask = srnd(1, 255);
            for (int w = 0; w < 8; w++) if (mask & (1 << w))
              for (int b = 0; b < 4; b++) v[4 * w + b] = uint8_t(srng());
            for (int b = 0; b < 32; b++) { smem.wr8(sn.line + b, v[b]); iss_mem.wr8(sn.line + b, v[b]); }
            cov.sn_dmaw_new++;
          } else cov.sn_dmaw_same++;
          for (int w = 0; w < 8; w++)
            cmem.wr(sn.line + 4 * w, (v[4 * w] << 24) | (v[4 * w + 1] << 16) | (v[4 * w + 2] << 8) | v[4 * w + 3], 0xF);
        }
        cov.sn_lat += cycles - sn.t0; cov.sn_lat_max = std::max<uint64_t>(cov.sn_lat_max, cycles - sn.t0);
        sn.act = false; last_ack = cycles;
        int gg = srnd(0, 9);
        sn_gap = gg < 4 ? 0 : gg < 8 ? srnd(1, 30) : srnd(30, 300);
      } else if (sn.act && cycles - sn.t0 > 30000) {
        printf("seed %u: no sn_ack for line %08X since cycle %llu\n", seed, sn.line, (unsigned long long)sn.t0); return false;
      } else if (!sn.act) {
        bool eager = cycles - last_st_cyc <= 2 && srnd(0, 3) == 0;
        if (sn_gap > 0 && !eager) sn_gap--;
        else {
          int r = srnd(0, 99);
          uint32_t line;
          if (eager) line = last_st_line;
          else if (r < 50 && !rq_lines.empty()) line = rq_lines[srnd(0, int(rq_lines.size()) - 1)];
          else if (r < 75 && !lw_kill.empty()) line = m_rsvq ? m_rsvline : m_rvpa;     // the reservation's line
          else if (r < 90) line = (DATA + uint32_t(srnd(0, int(g_span) - 32))) & ~31u;
          else line = uint32_t(srnd(0, 0x1FFFFF)) & ~31u;
          sn.act = true; sn.line = line; sn.t0 = cycles; sn.maybe.clear();
          sn.inv = g_snoop >= 2 && srnd(0, 99) < (r >= 50 && r < 75 ? 80 : 30);
          sn.chk = !line_inh.count(line);
          view_at(line, n_ans, sn.v0);
          for (auto &e : blog) if (e.seq >= n_ans && (e.a & ~31u) == line) sn.maybe.push_back({int(e.a & 31), e.nw});
          cov.sn_n[sn.inv]++; cov.sn_b2b += cycles == last_ack + 1; cov.sn_eager += eager;
        }
      }
      top.sn_req = sn.act; top.sn_addr = sn.line >> 5; top.sn_inv = sn.inv;
    }
    ma.step(top.m_req, top.m_cmd, top.m_addr, top.m_be, top.m_wdata);
    top.m_ack = ma.ack; top.m_rvalid = ma.rvalid; top.m_wnext = ma.wnext; top.m_rdata = ma.rdata;
    if (ma.err) { printf("seed %u: memory agent protocol error\n", seed); return false; }
    top.clk = 0; top.eval();
#else
    top.i_valid = i_pend && i_cnt == 0;
    top.i_data  = i_word;
    top.i_data2 = i_word2;
    top.i_two   = i_two;
    top.d_valid = !dq.empty() && dq.front().t == cycles;
    top.d_rdata = top.d_valid ? dq.front().word : uint32_t(rrng());   // junk outside an answer
    top.d_rdy   = d_rdy;
    // ppc_core LU2's early word, as ppc_dcache gives it: the word of the answer due next cycle three times in
    // four, else junk (a miss, the other way); d_eok when this cycle's answer is last cycle's early word, seven
    // times in eight (else as for a store-buffer byte). The core may only rely on d_eok with d_valid.
#ifdef TB_LU2
    {
      size_t const k = top.d_valid ? 1 : 0;
      bool const nx = dq.size() > k && dq[k].t == cycles + 1;
      top.d_eok = top.d_valid && top.d_rdata == e_prev && (e_rng() & 7) != 0;
      top.d_eword = (nx && (e_rng() & 3) != 0) ? dq[k].word : uint32_t(e_rng());
      e_prev = top.d_eword;
    }
#endif
    top.d_idle  = dq.empty();
    top.icbi_ack = top.icbi_req;                       // no I-cache: taken at once
    top.clk = 0; top.eval();
#endif

    if (top.rt_fw_en) {                                                    // written at this edge
      rtl_fpr[top.rt_fw_idx] = top.rt_fw_val;
      if (fptrace) printf("    rtl   f%d = %016llX (cycle %llu)\n", top.rt_fw_idx, (unsigned long long)top.rt_fw_val, (unsigned long long)cycles);
      fq_rtl[top.rt_fw_idx].push_back(top.rt_fw_val);
      if (fq_rtl[top.rt_fw_idx].size() > 4) { printf("seed %u: f%d written by the RTL with no model write\n", seed, top.rt_fw_idx); return false; }
    }
    for (int i = 0; i < 4; i++) cov.btb[i] += (top.ev_btb >> i) & 1;
    static uint32_t last_fs = 0;
    if (fptrace && top.rt_fpscr != last_fs) { printf("    rtl   fpscr %08X (cycle %llu)\n", top.rt_fpscr, (unsigned long long)cycles); last_fs = top.rt_fpscr; }
#ifdef PPC_CPU
    if (top.t_dreq && top.t_dwe && top.t_daddr == ACK) int_drv = false;  // the handler's acknowledge
    if (g_snoop) {
      // the reservation rule this cycle (ppc_dcache.sv, "the snoop")
      bool exp_kill = m_rvkill || (m_rsvq && sn_inw && m_rsvline == sn.line);
      if (exp_kill && !lw_kill.empty()) lw_kill.back() = true;
      if (top.t_dreq && top.t_dcnd && exp_kill) {
        printf("seed %u: stwcx. stored at cycle %llu with its reservation lost to a snooped write\n", seed, (unsigned long long)cycles);
        return false;
      }
      bool kn;
      if (m_rsvq) { kn = sn_inw && m_rsvline == sn.line; m_rvpa = m_rsvline; }
      else kn = m_rvkill || (sn_inw && m_rvpa == sn.line);
      m_rvkill = kn;
      m_rsvq = top.t_dreq && top.t_drsv;
      if (m_rsvq) { m_rsvline = top.t_daddr & ~31u; lw_kill.push_back(false); }
      if (getenv("SNTRACE") && (m_rsvq || exp_kill != m_rvkill || (top.t_dreq && top.t_dcnd)))
        printf("    cyc %llu: lwarx %d stwcx %d addr %08X kill %d->%d n_lw %zu ret %llu sn %d inv %d line %08X\n", (unsigned long long)cycles,
               int(m_rsvq), int(top.t_dreq && top.t_dcnd), top.t_daddr, int(exp_kill), int(m_rvkill), lw_kill.size(),
               (unsigned long long)n_lw_ret, int(sn.act), int(sn.inv), sn.line);
      // answers, then this cycle's request into the shadow
      if (top.t_dvalid) n_ans++;
      if (top.t_dreq) {
        uint32_t a = top.t_daddr & ~3u, line = a & ~31u;
        auto put = [&](uint32_t x, uint8_t v) {
          blog.push_back({n_seq, x, smem.rd8(x), v}); if (blog.size() > 1024) blog.pop_front();
          smem.wr8(x, v);
          if (sn.act && (x & ~31u) == sn.line) sn.maybe.push_back({int(x & 31), v});
        };
        if (top.t_dop == 0 && top.t_dwe) {
          for (int b = 0; b < 4; b++) if (top.t_dbe & (8 >> b)) put(a + b, uint8_t(top.t_dwdata >> (24 - 8 * b)));
          last_st_line = line; last_st_cyc = cycles;
        } else if (top.t_dop == 2 && !(top.t_dwimg & 0xC))
          for (int b = 0; b < 32; b++) put(line + b, 0);
        if (top.t_dwimg & 4) line_inh.insert(line);
        line_req[line] = cycles;
        rq_lines.push_back(line); if (rq_lines.size() > 16) rq_lines.pop_front();
        n_seq++;
      }
    }
    if (dtrace && (top.t_dreq || top.t_dvalid))
      printf("    cyc %llu: %s%s op %d %08X wimg %X | ans %d %08X\n", (unsigned long long)cycles, top.t_dreq ? "req " : "",
             top.t_dreq ? (top.t_dwe ? "st" : "ld") : "", int(top.t_dop), top.t_daddr, int(top.t_dwimg), int(top.t_dvalid), top.t_drdata);
    struct Edge { Top &t; ~Edge() { t.clk = 1; t.eval(); } } edge{top};
    if (dtrace && top.rt_valid)
      printf("    cyc %llu: retire pc %08X insn %08X last %d gpr_we %d r%d%s = %08X dly %d\n", (unsigned long long)cycles, top.rt_pc, top.rt_insn, int(top.rt_last), int(top.rt_gpr_we), top.rt_gpr, top.rt_gpr_sh ? "(sh)" : "", top.rt_gpr_val, int(top.rt_dly));
#else
    bool ireq = top.i_req, dreq = top.d_req, dwe = top.d_we;
    uint32_t ia = top.i_addr, da = top.d_addr, dwd = top.d_wdata;
    int dbe = top.d_be;
    bool i_took = top.i_valid, d_took = top.d_valid, dtook_ok = !dq.empty();

    if (dtrace && (dreq || top.d_valid))
      printf("    cyc %llu: %s%s %08X be %X wd %08X | ans %d %08X\n", (unsigned long long)cycles, dreq ? "req " : "", dreq ? (dwe ? "st" : "ld") : "", da, dbe, dwd, int(top.d_valid), top.d_rdata);
    if (dtrace && top.rt_valid)
      printf("    cyc %llu: retire pc %08X insn %08X last %d gpr_we %d r%d%s = %08X dly %d\n", (unsigned long long)cycles, top.rt_pc, top.rt_insn, int(top.rt_last), int(top.rt_gpr_we), top.rt_gpr, top.rt_gpr_sh ? "(sh)" : "", top.rt_gpr_val, int(top.rt_dly));
    if (dreq && !d_rdy) { printf("seed %u: data request with d_rdy low at cycle %llu\n", seed, (unsigned long long)cycles); return false; }
    // ev_xrf: a page crossing's second word replayed (micro-TLB miss) after its first word was made, and the
    // instruction is fetched again, so that first word will be made again: its earlier access, the last
    // request made (the replay holds EX from the cycle after it), leaves the stream the model compares
#ifndef TB_NO_XRF                                      // (an RTL without the port: the reproducer on the old core)
    if (top.ev_xrf) {
      if (acc_last_n == 0 || acc_last_n > acc_rtl.size()) { printf("seed %u: ev_xrf with no request to drop at cycle %llu\n", seed, (unsigned long long)cycles); return false; }
      acc_rtl.resize(acc_rtl.size() - acc_last_n); acc_last_n = 0; cov.xrf++;
    }
#endif
    cov.replays += (top.ev_mmu >> 4) & 1;
    cov.fslow   += (top.ev_mmu >> 5) & 1;
    // The retire record is taken before the edge, as its uop leaves WB (the
    // values of a load answered in that cycle are not registers); the edge
    // itself is made at the end of the iteration, whichever way it ends.
    struct Edge { Top &t; ~Edge() { t.clk = 1; t.eval(); } } edge{top};

    // memory side of the edge
    if (i_took) i_pend = false;
    if (d_took) dq.pop_front();
    if (i_pend) i_cnt--;
    if (ireq) { i_pend = true; i_cnt = (g_ilat ? g_ilat : wait_for()) - 1; i_word = rtl_mem.rd32(ia);
                if (top.i_w2) { if (!i_two) { printf("seed %u: i_w2 without i_two\n", seed); return false; }
                                i_word2 = rtl_mem.rd32(ia + 4); }
                else i_word2 = uint32_t(rrng()); }
    i_two = (rrng() & 3) != 0;                          // the memory side's offer for the next cycle
    d_rdy = !(g_rdy && int(rrng() % 100) < g_rdy);     // next cycle's, registered
    uint32_t d_word = 0;
    if (dreq) {
      size_t acc_n0 = acc_rtl.size();
      struct Mark { std::vector<uint64_t> &v; size_t n0; size_t &n; ~Mark() { n = v.size() - n0; } } mark{acc_rtl, acc_n0, acc_last_n};
      uint64_t t = cycles + uint64_t(g_dlat ? g_dlat : wait_for());
      if (t <= d_last) t = d_last + 1;
      d_last = t;
      if (top.d_op == 2 && (top.d_wimg & 0xC)) {                           // dcbz to a W or I page: the core takes
                                                                           // the alignment exception, no effect here
      } else if (top.d_op == 2) {                                          // dcbz: no cache, so the block in memory
        for (uint32_t b = 0; b < 32; b++) { rtl_mem.wr8((da & ~31u) + b, 0); acc_rtl.push_back(acc_st((da & ~31u) + b, 0)); }
      } else if (top.d_op != 0) {                                          // other cache operations: nothing here
      } else if (dwe) {
        for (int b = 0; b < 4; b++) if (dbe & (8 >> b)) { rtl_mem.wr8(da + b, uint8_t(dwd >> (24 - 8 * b))); acc_rtl.push_back(acc_st(da + b, uint8_t(dwd >> (24 - 8 * b)))); }
        if (da == ACK) int_drv = false;                                    // the handler's acknowledge
      } else { d_word = rtl_mem.rd32(da); acc_rtl.push_back(acc_ld(da)); }
      dq.push_back({t, d_word});
    }
    if (d_took && !dtook_ok) { printf("seed %u: d_valid with nothing outstanding\n", seed); return false; }
#endif

    // The timers lag one edge: a record is taken as its uop leaves WB, one
    // edge after it committed in MEM, so the record seen now belongs to the
    // previous edge. That edge's tick was applied last iteration; a commit of
    // mtdec / mttb, which the model makes as it steps below, replaces it.
    // Then the previous edge is finished and this edge's tick applied.
    bool dec_taken = false;

    auto finish_edge = [&]() {
      if (!(dec_prev >> 31) && (dec >> 31)) pend = true;                   // DEC[0] 0 -> 1
      if (dec_taken) pend = false;                                         // taking it cancels the request
      dec_h[hist(0)] = dec; tb_h[hist(0)] = tb; pend_h[hist(0)] = pend;
      dec_prev = dec;
      if (tick) { dec--; tb++; }
    };

    // FP state at a quiet point: nothing older outstanding, so every FPR and
    // the FPSCR must be the model's
    auto fp_quiet = [&]() -> bool {
      for (int r = 0; r < 32; r++) {
        if (!fp_match(r)) return false;
        if (!fq_rtl[r].empty() || !fq_iss[r].empty()) {
          printf("seed %u: f%d: %zu RTL / %zu model writes unmatched at a quiet point\n", seed, r, fq_rtl[r].size(), fq_iss[r].size());
          return false;
        }
        if (rtl_fpr[r] != iss.fpr[r]) { printf("seed %u: f%d rtl %016llX model %016llX\n", seed, r, (unsigned long long)rtl_fpr[r], (unsigned long long)iss.fpr[r]); return false; }
      }
      if (top.rt_fpscr != iss.fpscr) { printf("seed %u: fpscr rtl %08X model %08X\n", seed, top.rt_fpscr, iss.fpscr); return false; }
      fp_checks++;
      return true;
    };

    // retire
    if (!top.rt_valid) {
      finish_edge();
      if (++idle > 2000) { printf("seed %u: no retire for 2000 cycles at cycle %llu\n", seed, (unsigned long long)cycles); return false; }
      continue;
    }
    idle = 0;
    cur_dly = top.rt_dly;
    if (top.rt_gpr_we) { gpr[top.rt_gpr + (top.rt_gpr_sh ? 32 : 0)] = top.rt_gpr_val; cov.sh_writes += top.rt_gpr_sh; }
    if (!top.rt_last) { finish_edge(); continue; }

    uint32_t rpc = top.rt_pc;
    ring[nring++ % 48] = {rpc, top.rt_insn, top.rt_msr, top.rt_srr0, top.rt_srr1, int(top.rt_irq), cycles};
    if (rpc != iss.pc) {
      printf("seed %u: retired pc %08X, model pc %08X (insn %08X)\n", seed, rpc, iss.pc, top.rt_insn);
      return false;
    }
    bool want = (iss.msr & M_EE) && (int_h[hist(3 + cur_dly)] || pend_h[hist(2 + cur_dly)]);
    if (top.rt_irq) {
      // an interrupt in place of the instruction at rpc: legal here?
      bool ext = top.rt_irq & 1;
      const char *why = nullptr;
      if (!(iss.msr & M_EE))                         why = "MSR[EE] = 0";
      else if (ext && !int_h[hist(3 + cur_dly)])     why = "INT was not asserted";
      else if (!ext && !pend_h[hist(2 + cur_dly)])   why = "no decrementer request";
      else if (!ext && int_h[hist(3 + cur_dly)])     why = "decrementer taken over a pending external interrupt";
      if (why) { printf("seed %u: interrupt %s at pc %08X is not legal: %s\n", seed, ext ? "external" : "decrementer", rpc, why); return false; }
      iss.interrupt(ext ? 0x500 : 0x900);
      if (!ext) dec_taken = true;
      (ext ? cov.ext : cov.dec)++;
      pend_run = 0;
    } else {
      if (top.rt_insn == END_INSN) {
        if (!top.rt_fpq) { printf("seed %u: FP state not settled at the end\n", seed); return false; }
        if (!fp_quiet()) return false;
        for (auto &e : g.expect)
          if (gpr[e.first] != e.second || iss.gpr[e.first] != e.second) {
            printf("seed %u: reproducer r%d rtl %08X model %08X, expected %08X\n", seed, e.first, gpr[e.first],
                   iss.gpr[e.first], e.second);
            return false;
          }
        cov.alias_chk += g.expect.size();
#ifndef PPC_CPU
        for (uint32_t a = 0; a < rtl_mem.lo.size(); a++)
          if (rtl_mem.lo[a] != iss_mem.lo[a]) { printf("seed %u: memory %08X rtl %02X model %02X\n", seed, a, rtl_mem.lo[a], iss_mem.lo[a]); return false; }
        for (uint32_t a = 0; a < rtl_mem.hi.size(); a++)
          if (rtl_mem.hi[a] != iss_mem.hi[a]) { printf("seed %u: memory %08X rtl %02X model %02X\n", seed, 0xFFF00000u + a, rtl_mem.hi[a], iss_mem.hi[a]); return false; }
        if (acc_rtl != acc_iss) {
          size_t i = 0;
          while (i < acc_rtl.size() && i < acc_iss.size() && acc_rtl[i] == acc_iss[i]) i++;
          auto show = [](const std::vector<uint64_t> &v, size_t i) {
            if (i >= v.size()) return std::string("(none)");
            char b[64]; uint64_t e = v[i];
            snprintf(b, sizeof b, "%s %08X", (e >> 40) == 1 ? "load" : "store", uint32_t(e));
            if ((e >> 40) == 2) snprintf(b + strlen(b), sizeof b - strlen(b), " = %02X", unsigned(e >> 32) & 0xFF);
            return std::string(b);
          };
          printf("seed %u: data access %zu of %zu (rtl) / %zu (model) differs: rtl %s, model %s\n", seed, i,
                 acc_rtl.size(), acc_iss.size(), show(acc_rtl, i).c_str(), show(acc_iss, i).c_str());
          return false;
        }
#endif
        for (int i = 0; i < 0x20; i++) cov.vec[i] = iss.exc_count[i];
        cov.fprec = iss.fp_prec; cov.fpdel = iss.fp_delayed; cov.fpsupp = iss.fp_supp;
        cov.fpld = iss.fp_ls[0]; cov.fpst = iss.fp_ls[1];
        for (int i = 0; i < 18; i++) cov.mis[i] = iss.mis_n[i];
        for (int i = 0; i < ppcfpu::NUM_OPS; i++) cov.fpops[i] = iss.fp_ops[i];
        cov.insns[mode] = insns_here; cov.cycles[mode] = cycles;
        cov.fpc = fp_checks;
        g_cov.add(cov);
        dumper.armed = false;
        return true;
      }
      // an interrupt that stays requested with EE = 1 must be taken soon
      if (want) { if (++pend_run > 4) { printf("seed %u: interrupt requested with EE = 1 and not taken by pc %08X\n", seed, rpc); return false; } }
      else pend_run = 0;
#ifdef PPC_CPU
      bool is_lwarx = (top.rt_insn >> 26) == 31 && ((top.rt_insn >> 1) & 0x3FF) == 20;
      bool st_done = false;
      if (g_snoop && (top.rt_insn >> 26) == 31 && ((top.rt_insn >> 1) & 0x3FF) == 150 && iss.res && !(top.rt_cr & 0x20000000)) {
        // completed, not an exception in its place (a DTLB miss after a replay)?
        PpcIss probe = iss; probe.res = false; probe.wr8 = [](uint32_t, uint8_t) {}; probe.on_load = [](uint32_t) {};
        probe.step();
        st_done = probe.pc == rpc + 4;
      }
      if (st_done) {
        // the RTL's stwcx. failed where the model's would store: only if the
        // snoop rule cleared the reservation since its lwarx
        if (n_lw_ret == 0 || n_lw_ret > lw_kill.size() || !lw_kill[n_lw_ret - 1]) {
          printf("seed %u: stwcx. at %08X failed with its reservation not cleared (lwarx %llu of %zu made)\n", seed, rpc, (unsigned long long)n_lw_ret, lw_kill.size()); return false;
        }
        iss.res = false; cov.sn_rsv_lost++;
      }
#endif
      { uint32_t f0 = iss.fpscr, in0 = top.rt_insn;
      iss.step();
      if (iss.pc == rpc + 4) {                            // completed: the update forms with overlapping registers
        int op = in0 >> 26, xo = (in0 >> 1) & 0x3FF, rd = (in0 >> 21) & 31, ra = (in0 >> 16) & 31, rb = (in0 >> 11) & 31;
        bool xl_ = op == 31 && (xo == 55 || xo == 119 || xo == 311 || xo == 375), xf = op == 31 && (xo == 567 || xo == 631);
        bool xs_ = op == 31 && (xo == 183 || xo == 247 || xo == 439), xfs = op == 31 && (xo == 695 || xo == 759);
        bool ds_ = op == 37 || op == 39 || op == 45;
        if (xl_ && rd == rb) cov.alias[0]++;
        if ((xl_ || xf) && ra == rb) cov.alias[1]++;
        if (xs_ && rd == rb) cov.alias[2]++;
        if ((xs_ || ds_) && rd == ra) cov.alias[3]++;
        if ((xs_ || xfs) && ra == rb) cov.alias[4]++;
      }
#ifdef PPC_CPU
      if (is_lwarx && iss.res && iss.pc == rpc + 4) n_lw_ret++;          // (completed: not an exception in its place)
#endif
      if (fptrace && iss.fpscr != f0) printf("    model fpscr %08X (pc %08X insn %08X)\n", iss.fpscr, rpc, in0); }
      insns++; insns_here++;
    }
    finish_edge();

    bool bad = false;
    for (int r = 0; r < 32; r++) if (gpr[r] != iss.gpr[r]) { printf("  r%d rtl %08X model %08X\n", r, gpr[r], iss.gpr[r]); bad = true; }
    for (int r = 0; r < 4; r++) if (gpr[32 + r] != iss.tgpr[r]) { printf("  tgpr%d rtl %08X model %08X\n", r, gpr[32 + r], iss.tgpr[r]); bad = true; }
    auto chk = [&](const char *n, uint32_t a, uint32_t b) { if (a != b) { printf("  %s rtl %08X model %08X\n", n, a, b); bad = true; } };
    chk("cr", top.rt_cr, iss.cr);
    chk("xer", top.rt_xer, iss.xer);
    chk("lr", top.rt_lr, iss.lr);
    chk("ctr", top.rt_ctr, iss.ctr);
    chk("msr", top.rt_msr, iss.msr);
    chk("srr0", top.rt_srr0, iss.srr0);
    chk("srr1", top.rt_srr1, iss.srr1);
    for (int r = 0; r < 32; r++) if (!fp_match(r)) bad = true;
    if (!bad && top.rt_fpq && !fp_quiet()) bad = true;
    if (bad) { printf("seed %u (mode %d): state differs after pc %08X insn %08X (instruction %llu)\n", seed, mode, rpc, top.rt_insn, (unsigned long long)insns); return false; }
  }
  printf("seed %u: did not reach the end in %llu cycles\n", seed, (unsigned long long)cycles);
  return false;
}

} // namespace

// --emit <seed> <length> <dir>: the board test's program (rtl/dbg/m3_cputest.sv).
// Writes cputest_code.hex (32 KiB: code and vectors) and cputest_data.hex
// (16 KiB), after checking the program against the model like any other seed.
// Always the integer kind: the board's memory map has no room for translation.
int emit(uint32_t seed, int len, const char *dir) {
  g_board = true;
  g_span = 0x4000;
  uint64_t insns = 0, cyc = 0;
  if (!run(seed, len, 1, 0, insns, cyc)) { printf("emit: seed %u fails the model check\n", seed); return 1; }
  Gen g(seed, 0);
  g.build(len);
  Mem m;
  std::mt19937 a(seed * 7 + 1);
  install(m, g, a);
  for (uint32_t off = 0x100; off < 0x1000; off += 4)               // high vectors share the ROM
    if (m.rd32(0xFFF00000u + off)) m.wr32(off, m.rd32(0xFFF00000u + off));
  char path[512];
  snprintf(path, sizeof(path), "%s/cputest_code.hex", dir);
  FILE *f = fopen(path, "w");
  for (uint32_t w = 0; w < 8192; w++) fprintf(f, "%08X\n", m.rd32(4 * w));
  fclose(f);
  snprintf(path, sizeof(path), "%s/cputest_data.hex", dir);
  f = fopen(path, "w");
  for (uint32_t w = 0; w < 4096; w++) fprintf(f, "%08X\n", m.rd32(DATA + 4 * w));
  fclose(f);
  printf("emit: seed %u, %zu words of code, %llu instructions checked, files in %s\n", seed, g.code.size(), (unsigned long long)insns, dir);
  return 0;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  if (argc > 1 && !strcmp(argv[1], "--emit"))
    return emit(argc > 2 ? strtoul(argv[2], nullptr, 0) : 1, argc > 3 ? atoi(argv[3]) : 1500, argc > 4 ? argv[4] : ".");
  int force_mode = -1;
  std::vector<char *> pos;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--mode") && i + 1 < argc) force_mode = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--dlat") && i + 1 < argc) g_dlat = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--ilat") && i + 1 < argc) g_ilat = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--rdy") && i + 1 < argc) g_rdy = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--loops")) g_loops = true;
    else if (!strcmp(argv[i], "--intdrop") && i + 1 < argc) g_intdrop = uint32_t(atoi(argv[++i]));   // study R104
    else if (!strcmp(argv[i], "--mis")) g_mis = true;
    else if (!strcmp(argv[i], "--xpage")) g_xpage = true;
    else if (!strcmp(argv[i], "--alias")) g_alias = true;
    else if (!strcmp(argv[i], "--fpburst")) g_fpburst = true;
    else if (!strcmp(argv[i], "--fpchain")) g_fpchain = true;
    else if (!strcmp(argv[i], "--snoop") && i + 1 < argc) g_snoop = atoi(argv[++i]);
    else pos.push_back(argv[i]);
  }
  int seeds = pos.size() > 0 ? atoi(pos[0]) : 200;
  uint32_t first = pos.size() > 1 ? strtoul(pos[1], nullptr, 0) : 1;
  int len = pos.size() > 2 ? atoi(pos[2]) : 1500;
  int max_wait = pos.size() > 3 ? atoi(pos[3]) : 4;
  uint64_t insns = 0, cycles_all = 0;
  int pass = 0;
  for (int s = 0; s < seeds; s++) {
    uint64_t cyc = 0;
    uint32_t seed = first + s;
    int mode = force_mode >= 0 ? force_mode : int(seed % 4);
    if (run(seed, len, max_wait, mode, insns, cyc)) pass++;
    else { printf("FAIL\n"); return 1; }
    cycles_all += cyc;
  }
  // --loops runs the BTB's paths (ev_btb): with 100 seeds or more, each must have
  // been taken, rewritten code (real mode) checked, and a register target refetched,
  // so a BTB that has stopped predicting fails here although nothing else would
  if (g_loops && seeds >= 100 && (g_cov.btb[0] == 0 || g_cov.btb[3] == 0 || g_cov.btb[2] == 0 ||
                                  (force_mode < 0 && g_cov.btb[1] == 0))) {
    printf("FAIL: --loops coverage: BTB targets predicted %llu, failed checks %llu, register targets refetched %llu, writes %llu\n",
           (unsigned long long)g_cov.btb[0], (unsigned long long)g_cov.btb[1], (unsigned long long)g_cov.btb[2], (unsigned long long)g_cov.btb[3]);
    return 1;
  }
  printf("PASS: %d seeds, %llu instructions compared against the model, %llu cycles (max wait %d)\n",
         pass, (unsigned long long)insns, (unsigned long long)cycles_all, max_wait);
  const Cov &c = g_cov;
  printf("  seeds by kind: integer %llu, translated %llu, translated+interrupts %llu, interrupts %llu\n",
         (unsigned long long)c.seeds[0], (unsigned long long)c.seeds[1], (unsigned long long)c.seeds[2], (unsigned long long)c.seeds[3]);
  printf("  taken: DSI %llu  ISI %llu  ext %llu  program %llu  dec %llu  sc %llu  ITLB miss %llu  DTLB load %llu  DTLB store %llu  alignment %llu\n",
         (unsigned long long)c.vec[3], (unsigned long long)c.vec[4], (unsigned long long)c.ext, (unsigned long long)c.vec[7],
         (unsigned long long)c.dec, (unsigned long long)c.vec[0xC], (unsigned long long)c.vec[0x10],
         (unsigned long long)c.vec[0x11], (unsigned long long)c.vec[0x12], (unsigned long long)c.vec[6]);
  if (g_mis)
    printf("  misaligned done: in a word %llu, across a word %llu, a doubleword %llu, a line %llu, a page %llu (with DR = 1, in a DBAT block: %llu; stores %llu)\n",
           (unsigned long long)c.mis[0], (unsigned long long)c.mis[1], (unsigned long long)c.mis[2],
           (unsigned long long)c.mis[3], (unsigned long long)c.mis[4], (unsigned long long)c.mis[6], (unsigned long long)c.mis[5]);
  if (g_mis || g_xpage)
    printf("  page crossings with DR = 1: done %llu (both pages page-translated %llu, one word in a DBAT block %llu), "
           "second page DTLB miss %llu, second page DSI %llu; first words made again after a replay (ev_xrf) %llu\n",
           (unsigned long long)c.mis[6], (unsigned long long)c.mis[7], (unsigned long long)c.mis[8],
           (unsigned long long)c.mis[9], (unsigned long long)c.mis[10], (unsigned long long)c.xrf);
  if (g_mis || g_xpage)
    printf("  page crossings with DR = 1 met, by page offset: word at FFD %llu, FFE %llu, FFF %llu; halfword at FFF %llu\n",
           (unsigned long long)c.mis[11], (unsigned long long)c.mis[12], (unsigned long long)c.mis[13], (unsigned long long)c.mis[14]);
  if (g_xpage)
    printf("  FP doubles across a page with DR = 1: alignment exception (no DBAT hit) %llu, two words inside a DBAT block %llu, two words leaving it %llu\n",
           (unsigned long long)c.mis[15], (unsigned long long)c.mis[16], (unsigned long long)c.mis[17]);
  if (g_alias)
    printf("  aliased update forms done: load rD = rB %llu, load rA = rB %llu, store rS = rB %llu, store rS = rA %llu,"
           " store rA = rB %llu; reproducer values checked %llu\n",
           (unsigned long long)c.alias[0], (unsigned long long)c.alias[1], (unsigned long long)c.alias[2],
           (unsigned long long)c.alias[3], (unsigned long long)c.alias[4], (unsigned long long)c.alias_chk);
  if (g_snoop)
    printf("  snoops: reads %llu, writes %llu (back to back %llu, straight after a store %llu), mean %.1f, max %llu cycles; bytes checked %llu, lines not checked %llu;"
           " DMA writes new %llu, as left %llu; stwcx. failed by a snooped write %llu\n",
           (unsigned long long)c.sn_n[0], (unsigned long long)c.sn_n[1], (unsigned long long)c.sn_b2b, (unsigned long long)c.sn_eager,
           (c.sn_n[0] + c.sn_n[1]) ? double(c.sn_lat) / (c.sn_n[0] + c.sn_n[1]) : 0.0, (unsigned long long)c.sn_lat_max,
           (unsigned long long)c.sn_bytes, (unsigned long long)c.sn_unchk, (unsigned long long)c.sn_dmaw_new,
           (unsigned long long)c.sn_dmaw_same, (unsigned long long)c.sn_rsv_lost);
  printf("  FP unavailable %llu, FP program precise %llu / after an MSR write %llu, suppressed results %llu\n",
         (unsigned long long)c.vec[8], (unsigned long long)c.fprec, (unsigned long long)c.fpdel, (unsigned long long)c.fpsupp);
  printf("  FP loads %llu, stores %llu, FPR writes and FP quiet points checked %llu\n  FP ops:",
         (unsigned long long)c.fpld, (unsigned long long)c.fpst, (unsigned long long)c.fpc);
  for (int i = 0; i < ppcfpu::NUM_OPS; i++) printf(" %s %llu", ppcfpu::op_name[i], (unsigned long long)c.fpops[i]);
  printf("\n");
  printf("  TGPR shadow writes %llu, data micro-TLB replays %llu, fetch translation slow cycles %llu\n",
         (unsigned long long)c.sh_writes, (unsigned long long)c.replays, (unsigned long long)c.fslow);
  printf("  BTB: targets predicted %llu, failed word checks %llu, register targets refetched %llu, entries written %llu\n",
         (unsigned long long)c.btb[0], (unsigned long long)c.btb[1], (unsigned long long)c.btb[2], (unsigned long long)c.btb[3]);
  static const char *kn[] = {"integer", "translated", "translated+interrupts", "interrupts"};
  for (int i = 0; i < 4; i++)
    if (c.insns[i]) printf("  %s: %llu instructions, %llu cycles, CPI %.3f\n", kn[i], (unsigned long long)c.insns[i],
                           (unsigned long long)c.cycles[i], double(c.cycles[i]) / double(c.insns[i]));
  return 0;
}
