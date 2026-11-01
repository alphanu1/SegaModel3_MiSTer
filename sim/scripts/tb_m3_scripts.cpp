// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// tb_m3_scripts [options] <file.vec>...
//
// m3_scripts against the HW-form reference (m3s_ref.h), on the capture vectors
// and on synthetic programs. Every CPU access goes to both; the RTL is let run
// until it is idle before the next access (the emulators run SCRIPTS at once),
// then every register read, the interrupt line, and the memory traffic since
// the last access are compared: the sequence of word addresses read and of
// (address, data) words written.
//
// The memory speaks ppc_cache_pkg's protocol and can be hostile:
//   --ack MIN MAX    acknowledge delay in cycles (counted on cycles it is not busy)
//   --busy PCT       a cycle is busy (no acknowledge, no beat) with this chance
//   --gap PCT        a beat cycle carries no beat with this chance
//   --seed N
// and it checks the protocol: fields held until the acknowledge, the request
// dropped after it, nothing asked while beats are due, line addresses aligned.
//
//   --every K        replay every K-th chain of each capture (a chain: a DSP
//                    write up to the next one), for the slow profiles
//   --poll           while the RTL runs, read ISTAT at random and require DIP 0
//   --synth N        N random programs: driver-style single-step chains, free
//                    runs started by a DSP write (MAN 0) ending in INT, illegal
//                    instructions, zero counts, misaligned moves, a software
//                    reset in mid-move, IRQD and DIEN masking, RD_ALL reads
//   --rd_all         the build has RD_ALL = 1

#include "Vm3_scripts.h"
#include "verilated.h"
#include "m3s_ref.h"
#include "m3s_vec.h"
#include <algorithm>
#include <cstring>
#include <random>

static Vm3_scripts *top;
static uint64_t cyc = 0, errors = 0;
static std::mt19937_64 rng(1);
static int ack_min = 0, ack_max = 2, busy_pct = 0, gap_pct = 0;
static uint64_t timeout = 400000000ull;
static bool poll = false, rd_all = false, no_int = false;

static int rnd(int n) { return n <= 0 ? 0 : int(rng() % uint64_t(n)); }
static bool chance(int pct) { return pct > 0 && rnd(100) < pct; }

// the first 20 errors are printed; the run stops at the 21st (a hang would otherwise time out at every access)
#define FAIL(...) do { errors++; if (errors <= 20) { std::printf("  ERROR @%llu: ", (unsigned long long)cyc); std::printf(__VA_ARGS__); std::printf("\n"); } \
                       else { std::printf("FAIL: stopped after 20 errors\n"); std::fflush(stdout); std::exit(1); } } while (0)

// ------------------------------------------------------------------ memory model
struct Port {
  BenchMem mem;
  std::vector<uint32_t> rd_log;                         // word addresses read
  std::vector<std::pair<uint32_t, uint32_t>> wr_log;    // words written
  enum { IDLE, WAIT, BEATS, AFTER } st = IDLE;
  int delay = 0, beats = 0, beat_i = 0;
  uint32_t cmd = 0, addr = 0, be = 0, wdata = 0;
  uint64_t trans = 0, lines = 0;

  // called after each rising edge: decide the inputs for the next one
  void step() {
    top->m_ack = 0; top->m_rvalid = 0; top->m_wnext = 0;
    switch (st) {
      case AFTER:
        if (top->m_req) FAIL("m_req still high the cycle after its acknowledge");
        st = beats ? BEATS : IDLE;
        if (st == IDLE) break;
        [[fallthrough]];
      case BEATS:
        if (top->m_req) FAIL("new request while %d beats are due", beats);
        if (st == BEATS && !chance(busy_pct) && !chance(gap_pct)) {
          uint32_t a = addr + 4 * beat_i;
          if (cmd == 1) { top->m_wnext = 1; wr_log.push_back({a, top->m_wdata}); mem.wr(a, top->m_wdata); }
          else { top->m_rvalid = 1; top->m_rdata = mem.rd(a); rd_log.push_back(a); }
          beat_i++;
          if (--beats == 0) st = IDLE;
        }
        break;
      case IDLE:
        if (!top->m_req) break;
        cmd = top->m_cmd; addr = top->m_addr; be = top->m_be;
        if ((cmd == 0 || cmd == 1) && (addr & 31)) FAIL("line %u at unaligned %08X", cmd, addr);
        if (be != 0xF) FAIL("byte enables %X", be);
        delay = ack_min + rnd(ack_max - ack_min + 1);
        st = WAIT; trans++; if (cmd < 2) lines++;
        [[fallthrough]];
      case WAIT:
        if (!top->m_req || top->m_cmd != cmd || top->m_addr != addr) { FAIL("request changed before its acknowledge"); }
        if (chance(busy_pct)) break;
        if (delay > 0) { delay--; break; }
        top->m_ack = 1;
        beat_i = 0;
        beats = cmd == 3 ? 0 : cmd == 2 ? 1 : 8;
        if (cmd == 3) { wr_log.push_back({addr, top->m_wdata}); mem.wr(addr, top->m_wdata); }
        st = AFTER;
        break;
    }
  }
} port;

static void tick() {
  top->clk = 1; top->eval();
  top->clk = 0; top->eval();
  cyc++;
  port.step();
  top->eval();
}

// ------------------------------------------------------------------ CPU side
static uint32_t cpu_access(bool we, unsigned off, unsigned be, uint32_t data) {
  top->s_req = 1; top->s_we = we; top->s_addr = (off >> 2) & 31; top->s_be = be; top->s_wdata = data;
  int n = 0;
  do { tick(); if (++n > 100) { FAIL("no s_ack"); break; } } while (!top->s_ack);
  uint32_t r = top->s_rdata, mask = 0;
  for (int i = 0; i < 4; i++) if (be & (1u << i)) mask |= 0xFFu << (8 * i);
  top->s_req = 0; tick();
  return r & mask;                                       // only the lanes read matter on the bus
}
static uint32_t cfg_access(bool we, unsigned idx, uint32_t data) {
  top->c_req = 1; top->c_we = we; top->c_idx = idx; top->c_be = 0xF; top->c_wdata = data;
  int n = 0;
  do { tick(); if (++n > 100) { FAIL("no c_ack"); break; } } while (!top->c_ack);
  uint32_t r = top->c_rdata;
  top->c_req = 0; tick();
  return r;
}

// run until idle; with --poll, read ISTAT on the way: DIP must stay 0 unless an
// interrupt was already pending when the run started (the CPU may start one unread)
static uint64_t busy_cycles = 0;
static void settle(bool pending = false) {
  uint64_t t0 = cyc;
  while (top->busy) {
    if (poll && chance(5)) {
      uint32_t v = cpu_access(false, 0x14, 8, 0);
      if (top->busy && !pending && (v >> 24) & 1) FAIL("ISTAT.DIP set while still running");
    } else tick();
    if (cyc - t0 > timeout) { FAIL("still busy after %llu cycles", (unsigned long long)timeout); break; }
  }
  busy_cycles += cyc - t0;
}

// ------------------------------------------------------------------ the pair
struct Pair {
  M3sRef ref;
  BenchMem rmem;
  std::vector<uint32_t> r_rd;
  std::vector<std::pair<uint32_t, uint32_t>> r_wr;
  uint64_t reads = 0, writes = 0, cap_diff = 0, cap_diff_other = 0;

  Pair() {
    ref.form = M3sRef::HW;
    ref.rd_all = rd_all;
    ref.int_op = !no_int;
    ref.rd = [this](uint32_t a) { r_rd.push_back(a & ~3u); return rmem.rd(a); };
    ref.wr = [this](uint32_t a, uint32_t d) { r_wr.push_back({a & ~3u, d}); rmem.wr(a, d); };
    ref.reset();
  }
  void load(uint32_t a, uint32_t w) { rmem.wr(a, w); port.mem.wr(a, w); }
  void compare_traffic() {
    if (port.rd_log != r_rd) {
      size_t i = 0;
      while (i < port.rd_log.size() && i < r_rd.size() && port.rd_log[i] == r_rd[i]) i++;
      FAIL("reads differ at %zu of %zu/%zu: rtl %08X ref %08X", i, port.rd_log.size(), r_rd.size(),
           i < port.rd_log.size() ? port.rd_log[i] : 0, i < r_rd.size() ? r_rd[i] : 0);
    }
    if (port.wr_log != r_wr) {
      size_t i = 0;
      while (i < port.wr_log.size() && i < r_wr.size() && port.wr_log[i] == r_wr[i]) i++;
      FAIL("writes differ at %zu of %zu/%zu: rtl %08X=%08X ref %08X=%08X", i, port.wr_log.size(), r_wr.size(),
           i < port.wr_log.size() ? port.wr_log[i].first : 0, i < port.wr_log.size() ? port.wr_log[i].second : 0,
           i < r_wr.size() ? r_wr[i].first : 0, i < r_wr.size() ? r_wr[i].second : 0);
    }
    port.rd_log.clear(); port.wr_log.clear(); r_rd.clear(); r_wr.clear();
  }
  void write(unsigned off, unsigned be, uint32_t v) {
    writes++;
    bool pending = ref.dstat != 0;
    cpu_access(true, off, be, v);
    ref.write(off, be, v);
    settle(pending);
    compare_traffic();
  }
  uint32_t read(unsigned off, unsigned be) {
    reads++;
    uint32_t g = cpu_access(false, off, be, 0);
    uint32_t e = ref.read(off, be);
    if (g != e) FAIL("read %02X/%X: rtl %08X ref %08X", off, be, g, e);
    return g;
  }
  void irq() { if (bool(top->irq) != ref.irq()) FAIL("irq rtl %d ref %d", top->irq, ref.irq()); }
};

static void reset() {
  top->rst = 1; top->s_req = 0; top->c_req = 0; top->m_ack = 0; top->m_rvalid = 0; top->m_wnext = 0;
  for (int i = 0; i < 4; i++) tick();
  top->rst = 0; tick();
  port.st = Port::IDLE;
  port.mem.m.clear(); port.rd_log.clear(); port.wr_log.clear();   // each file starts from a clean memory
}

// ------------------------------------------------------------------ capture replay
static void replay(const std::string &path, int every) {
  auto v = load_vec(path);
  reset();
  Pair p;
  // chains: a DSP write (W 2C) starts one; chain -1 (the set-up) is always replayed
  long chain = -1;
  uint64_t chains = 0, t0 = port.trans, l0 = port.lines;
  uint64_t b0 = busy_cycles, c0 = cyc;
  bool on = true;
  std::vector<uint64_t> per_frame;          // busy cycles in each frame with a move
  uint64_t fb = busy_cycles, bytes = 0, fbytes = 0, max_fbytes = 0;
  auto frame_end = [&]() {
    if (busy_cycles > fb && fbytes) per_frame.push_back(busy_cycles - fb);
    max_fbytes = std::max(max_fbytes, fbytes);
    fb = busy_cycles; fbytes = 0;
  };
  for (auto &r : v) {
    if (r.k == 'F') frame_end();
    if (r.k == 'M' && on) { bytes += r.c; fbytes += r.c; }
    if (r.k == 'W' && r.a == 0x2C) { chain++; on = (chain % every) == 0; if (on) chains++; }
    if (r.k == 'L') { p.load(r.a, r.b); continue; }
    if (!on) continue;
    switch (r.k) {
      case 'P':
        cfg_access(true, r.a, r.b); p.ref.cfg_write(r.a, r.b);
        if (top->win_base != p.ref.bar1 || bool(top->win_en) != bool(p.ref.cmd & 2))
          FAIL("window: rtl %06X/%d ref %06X/%d", top->win_base, top->win_en, p.ref.bar1, (p.ref.cmd >> 1) & 1);
        for (unsigned idx : {0u, 1u, 3u, 5u})
          if (cfg_access(false, idx, 0) != p.ref.cfg_read(idx)) FAIL("config read %u", idx);
        break;
      case 'p': { uint32_t g = cfg_access(false, r.a, 0); if (g != r.b) FAIL("config read %u: %08X", r.a, g); break; }
      case 'W': p.write(r.a, r.b, r.c); break;
      case 'R': {
        uint32_t g = p.read(r.a, r.b);
        if (g != r.c) { if (r.a == 0x38 && (g ^ r.c) == 0x04) p.cap_diff++; else p.cap_diff_other++; }
        break;
      }
      case 'Q': p.irq(); break;
      default: break;
    }
  }
  frame_end();
  std::sort(per_frame.begin(), per_frame.end());
  if (!per_frame.empty()) {
    uint64_t bc = busy_cycles - b0;
    std::printf("  moved %llu bytes in %llu busy cycles: %.2f cycles a word, %.1f MB/s at 100 MHz; per frame with moves "
                "(%zu): median %.2f ms, p99 %.2f ms, max %.2f ms (largest frame %llu bytes)\n",
                (unsigned long long)bytes, (unsigned long long)bc, bytes ? 4.0 * bc / bytes : 0.0,
                bc ? bytes * 100.0 / bc : 0.0, per_frame.size(), per_frame[per_frame.size() / 2] * 1e-5,
                per_frame[per_frame.size() * 99 / 100] * 1e-5, per_frame.back() * 1e-5, (unsigned long long)max_fbytes);
  }
  std::printf("%s: %llu chains of %ld, %llu writes, %llu reads (differ from the capture: %llu DCNTL.STD, %llu other), "
              "%llu cycles, %llu busy, %llu port transactions (%llu lines), errors %llu\n",
              path.c_str(), (unsigned long long)chains, chain + 1, (unsigned long long)p.writes,
              (unsigned long long)p.reads, (unsigned long long)p.cap_diff, (unsigned long long)p.cap_diff_other,
              (unsigned long long)(cyc - c0),
              (unsigned long long)(busy_cycles - b0), (unsigned long long)(port.trans - t0), (unsigned long long)(port.lines - l0),
              (unsigned long long)errors);
}

// ------------------------------------------------------------------ synthetic programs
static uint32_t mmov(uint32_t n) { return 0xC0000000u | n; }
static void put_insn(Pair &p, uint32_t a, uint32_t w0, uint32_t w1, int n = 2, uint32_t w2 = 0) {
  p.load(a, M3sRef::swap(w0)); p.load(a + 4, M3sRef::swap(w1));
  if (n == 3) p.load(a + 8, M3sRef::swap(w2));
}
static uint32_t rnd_len() {
  if (rnd(40) == 0) return 4 * ((1u << (14 + rnd(5))) + rnd(64));   // 64 KiB - 1 MiB: the count's high bits
  switch (rnd(4)) {
    case 0: return 4 * (1 + rnd(4));
    case 1: return 4 * (1 + rnd(40));
    case 2: return 4 * (1 + rnd(600));
    default: return 32 * (1 + rnd(64));
  }
}
static uint32_t rnd_addr(uint32_t base) { return base + 4 * rnd(0x4000); }

static void synth(int n) {
  reset();
  Pair p;
  uint64_t kinds[8] = {0};
  for (int t = 0; t < n; t++) {
    int kind = rnd(8);
    kinds[kind]++;
    uint32_t prog = 0x00100000 + 0x100 * rnd(64);
    // set-up as the driver does it, with some variation
    p.write(0x38, 8, uint32_t(0xC1) << 24);                       // DMODE: BL 16, MAN
    p.write(0x38, 4, uint32_t(kind == 6 ? 0x00 : 0x0D) << 16);    // DIEN: SSI SIR IID (none for kind 6)
    if (kind == 0 || kind == 6 || kind == 7) {
      // the driver: k moves, then INT 1, single-stepped from the "ISR"
      int k = 1 + rnd(6);
      for (int i = 0; i < k; i++) put_insn(p, prog + 12 * i, mmov(rnd_len()), rnd_addr(0x00200000), 3, rnd_addr(0x98000000));
      put_insn(p, prog + 12 * k, 0x98080000, 1);
      p.write(0x2C, 0xF, M3sRef::swap(prog));
      bool irqd = kind == 7;
      for (int i = 0; i < k; i++) {
        p.read(0x38, 1);
        p.write(0x38, 1, irqd ? 0x16 : 0x14);
        p.irq();
        p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF);
        if (rd_all) { p.read(0x1C, 0xF); p.read(0x24, 0xF); p.read(0x30, 0xF); }
        p.irq();
      }
    } else if (kind == 1) {
      // a free run started by the DSP write (MAN clear), ending in INT
      p.write(0x38, 8, 0);
      p.write(0x38, 1, rnd(2) ? 0x10 : 0x00);                    // sometimes SSM: the run stops after one
      int k = 1 + rnd(5);
      for (int i = 0; i < k; i++) put_insn(p, prog + 12 * i, mmov(rnd_len()), rnd_addr(0x00200000), 3, rnd_addr(0x8E000000));
      put_insn(p, prog + 12 * k, 0x98080000, 0x1234 + t);
      p.write(0x2C, 0xF, M3sRef::swap(prog));
      p.irq(); p.read(0x14, 8); p.read(0x38, 0xF);
      if (rd_all) p.read(0x30, 0xF);
      p.read(0x0C, 8); p.irq(); p.read(0x2C, 0xF);
      // halted after INT: STD alone must not run until DSP is written
      p.write(0x38, 1, 0x04); p.read(0x0C, 8);
    } else if (kind == 2) {
      // illegal: near misses of the two built opcodes, every other instruction class, or random
      static const uint32_t tmpl[] = {
        0x00000000, 0x08000000, 0x40000000, 0x48000000, 0x50000000, 0x58000000, 0x60000000,   // block move, I/O
        0x68000000, 0x70000000, 0x78000000,                                                    // read/write register
        0x80080000, 0x88080000, 0x90080000, 0xA0080000, 0xA8080000, 0xB0080000, 0xB8080000,   // JUMP CALL RETURN, reserved
        0x98280000, 0x98180000, 0x980C0000, 0x980A0000,                                        // INT with CT, IF, CD, CP
        0xC2000000, 0xC4000000, 0xC8000000, 0xD0000000,                                        // MMOV with a reserved bit
        0xE0000000, 0xE1000000, 0xE4000000, 0xE9000000, 0xF0000000 };                         // load/store, top codes
      uint32_t w0;
      do {
        w0 = rnd(4) ? tmpl[rnd(int(sizeof tmpl / sizeof tmpl[0]))] | (uint32_t(rng()) & 0x0000FFFF) | (no_int ? 0 : 0) : uint32_t(rng());
      } while (((w0 >> 29) == 6 && ((w0 >> 25) & 15) == 0) || (!no_int && (w0 >> 27) == 0x13 && !(w0 & 0x00360000)));
      put_insn(p, prog, w0, uint32_t(rng()), 3, uint32_t(rng()));
      p.write(0x2C, 0xF, M3sRef::swap(prog));
      p.write(0x38, 1, 0x14);
      p.irq(); p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF); p.irq();
    } else if (kind == 3) {
      // zero count or misaligned
      int c = rnd(4);
      uint32_t len = c == 0 ? 0 : c == 1 ? 4 * (1 + rnd(8)) + 2 : 16;
      uint32_t src = rnd_addr(0x00200000) + (c == 2 ? 1 + rnd(3) : 0);
      uint32_t dst = rnd_addr(0x98000000) + (c == 3 ? 1 + rnd(3) : 0);
      put_insn(p, prog, mmov(len), src, 3, dst);
      p.write(0x2C, 0xF, M3sRef::swap(prog));
      p.write(0x38, 1, 0x14);
      p.irq(); p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF); p.irq();
      // halted: a DSP write releases it
      put_insn(p, prog + 0x40, mmov(8), 0x00200000, 3, 0x98000000);
      p.write(0x2C, 0xF, M3sRef::swap(prog + 0x40));
      p.write(0x38, 1, 0x14); p.read(0x0C, 8); p.irq();
    } else if (kind == 4) {
      // a software reset in mid-move (RTL only: the reference has already finished),
      // then the registers must read their defaults and the next move must be clean
      put_insn(p, prog, mmov(4096), rnd_addr(0x00200000), 3, rnd_addr(0x98000000));
      p.write(0x2C, 0xF, M3sRef::swap(prog));
      cpu_access(true, 0x38, 1, 0x14);
      for (int i = rnd(300); i > 0 && top->busy; i--) tick();
      cpu_access(true, 0x14, 8, 0x40u << 24);
      cpu_access(true, 0x14, 8, 0);
      settle();
      p.ref.reset_ops(); p.ref.istat = 0;
      port.rd_log.clear(); port.wr_log.clear(); p.r_rd.clear(); p.r_wr.clear();
      p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF); p.read(0x38, 0xF); p.irq();
      p.write(0x38, 8, uint32_t(0xC1) << 24);
      p.write(0x38, 4, uint32_t(0x08) << 16);
      put_insn(p, prog + 0x40, mmov(rnd_len()), rnd_addr(0x00200000), 3, rnd_addr(0x98000000));
      p.write(0x2C, 0xF, M3sRef::swap(prog + 0x40));
      p.write(0x38, 1, 0x14); p.irq(); p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF);
    } else {
      // kind 5: byte-wide DSP writes in a random order, then a step; with MAN clear
      // half the time, so the run starts at whichever write carries byte 2F
      put_insn(p, prog, mmov(rnd_len()), rnd_addr(0x00200000), 3, rnd_addr(0x94000000));
      if (rnd(2)) { p.write(0x38, 8, 0); p.write(0x38, 1, 0x10); }
      uint32_t w = M3sRef::swap(prog);
      int ord[4] = {0, 1, 2, 3};
      for (int b = 3; b > 0; b--) std::swap(ord[b], ord[rnd(b + 1)]);
      for (int b = 0; b < 4; b++) p.write(0x2C, 1u << ord[b], w & (0xFFu << (8 * ord[b])));
      p.read(0x2C, 0xF);
      p.write(0x38, 1, 0x14); p.irq(); p.read(0x14, 8); p.read(0x0C, 8); p.read(0x2C, 0xF); p.irq();
    }
  }
  std::printf("synth: %d programs (driver %llu, free run %llu, illegal %llu, zero/misaligned %llu, reset %llu, bytes %llu, "
              "masked %llu, IRQD %llu), %llu cycles, %llu port transactions, errors %llu\n",
              n, (unsigned long long)kinds[0], (unsigned long long)kinds[1], (unsigned long long)kinds[2],
              (unsigned long long)kinds[3], (unsigned long long)kinds[4], (unsigned long long)kinds[5],
              (unsigned long long)kinds[6], (unsigned long long)kinds[7], (unsigned long long)cyc,
              (unsigned long long)port.trans, (unsigned long long)errors);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  top = new Vm3_scripts;
  int every = 1, nsynth = 0;
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--ack" && i + 2 < argc) { ack_min = std::atoi(argv[++i]); ack_max = std::atoi(argv[++i]); }
    else if (a == "--busy" && i + 1 < argc) busy_pct = std::atoi(argv[++i]);
    else if (a == "--gap" && i + 1 < argc) gap_pct = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) rng.seed(std::atoll(argv[++i]));
    else if (a == "--every" && i + 1 < argc) every = std::atoi(argv[++i]);
    else if (a == "--synth" && i + 1 < argc) nsynth = std::atoi(argv[++i]);
    else if (a == "--poll") poll = true;
    else if (a == "--rd_all") rd_all = true;
    else if (a == "--timeout" && i + 1 < argc) timeout = std::strtoull(argv[++i], nullptr, 0);
    else if (a == "--no_int") no_int = true;
    else if (a == "--off") {
      // ENABLE = 0: the window reads 0, configuration reads all ones, nothing is asked of memory
      reset();
      cfg_access(true, 5, 0xC0000000u);
      if (cfg_access(false, 0, 0) != 0xFFFFFFFFu) FAIL("off: config ID");
      cpu_access(true, 0x2C, 0xF, 0x80131000u);
      cpu_access(true, 0x38, 0x9, 0xC1000014u);
      for (int k = 0; k < 1000; k++) { tick(); if (top->m_req || top->irq || top->busy || top->win_en) FAIL("off: active"); }
      if (cpu_access(false, 0x2C, 0xF, 0) || cpu_access(false, 0x14, 8, 0)) FAIL("off: reads not 0");
      std::printf("off: %llu errors\n", (unsigned long long)errors);
    }
    else if (a[0] != '+') files.push_back(a);
  }
  std::printf("profile: ack %d-%d, busy %d%%, gap %d%%%s\n", ack_min, ack_max, busy_pct, gap_pct, poll ? ", ISTAT polled" : "");
  for (auto &f : files) replay(f, every);
  if (nsynth) synth(nsynth);
  std::printf("%s: %llu errors\n", errors ? "FAIL" : "PASS", (unsigned long long)errors);
  delete top;
  return errors ? 1 : 0;
}
