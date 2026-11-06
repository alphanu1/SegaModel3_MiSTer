// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_bus with its SDRAM target (m3_sdr_port, m2_sdram, the SDRAM at its pins):
// two masters in the caches' protocol, each holding its request to the
// acknowledge and presenting the next one the cycle after (as ppc_cpu can),
// against a reference of what each access must do:
//   - RAM (word form) and tile VRAM (byte form): reads, writes, line fills,
//     cast-outs, against a byte image;
//   - the IRQ enable register, CONFIG_ADDR and the tile register shadow: read
//     back what was written;
//   - tile registers and palette: the m3_tilegen strobes, data byte-reversed;
//   - the Real3D windows: counted, line writes word by word; status FFFFFFFD/F;
//   - the 53C810 window and the 315-5649: one target access each, the data
//     returned; the JTAG write waits while the chain is busy;
//   - an unmapped read: FFFFFFFF, counted.
// Every transaction must be acknowledged exactly once, with its beats after
// the acknowledge; a spurious acknowledge to an idle master fails the run.
//
// The snoop (D-41) and posted writes: master 0 also keeps a small copy-back
// data cache of lines in 00012000-000123FF (stores stay in it; misses fill
// it; a dirty victim is cast out), and answers the bus's snoop as ppc_dcache
// does: once master 0 is quiet, a dirty line is pushed (a line write), then
// sn_ack; a write snoop invalidates. The 53C810 reads and writes the same
// lines. Checked: a 53C810 write never lands while the line is still held
// (the CPU refills it at once after the invalidate, to race it); a 53C810
// read returns the line as it was at the snoop (the push landed first); every
// CPU load hit returns the coherent value. Reads often follow a write to the
// same address (a posted write must be seen).
//
//   Vm3_bus_tb TRANSACTIONS SEED
#include "Vm3_bus_tb.h"
#include "verilated.h"
#include "sdram_dev.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <map>
#include <random>
#include <vector>

static Vm3_bus_tb *t;
static sdram_dev sd;
static uint64_t cyc;
static std::mt19937 rng;
static long bad = 0, proto = 0;

static std::vector<uint8_t> ref;      // the SDRAM byte image as the bus should leave it
static uint32_t sdr_of(uint32_t a, bool &swap) {
  swap = (a >> 24) == 0xF1;
  return swap ? 0x6C00000 + (a & 0xFFFFF) : 0x6400000 + (a & 0x7FFFFF);
}
static void ref_write(uint32_t a, uint32_t v, uint32_t be) {
  bool swap; uint32_t const s = sdr_of(a & ~3u, swap);
  for (int k = 0; k < 4; k++) if ((be >> (3 - k)) & 1) ref[s + (swap ? k : 3 - k)] = uint8_t(v >> (24 - 8 * k));
}
static uint32_t ref_word(uint32_t a) {
  bool swap; uint32_t const b = sdr_of(a & ~3u, swap);
  return swap ? (uint32_t(ref[b]) << 24) | (uint32_t(ref[b + 1]) << 16) | (uint32_t(ref[b + 2]) << 8) | ref[b + 3]
              : (uint32_t(ref[b + 3]) << 24) | (uint32_t(ref[b + 2]) << 16) | (uint32_t(ref[b + 1]) << 8) | ref[b];
}
static uint32_t bswap(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24); }

// one transaction
struct Txn {
  int cmd;                 // 0 LINE_RD, 1 LINE_WR, 2 RD, 3 WR
  uint32_t addr, be;
  uint32_t wd[8];
  std::vector<uint32_t> exp;   // read beats expected (empty: not checked here)
  bool check = true;
  int kind = 0;            // what it touches, for the counters
};
enum { K_MEM, K_IRQEN, K_PCIA, K_TGREG, K_PAL, K_R3D88, K_R3D8C, K_R3D98, K_R3DST, K_SCSI, K_IO, K_JTAG, K_UNM, K_FILL };

// the coherent region and master 0's cache of it
static uint32_t const C_LO = 0x00012000, C_HI = 0x00012400;
static bool in_c(uint32_t a) { return a >= C_LO && a < C_HI; }
struct Line { uint32_t d[8]; bool dirty; };
static std::map<uint32_t, Line> dc;            // by line address
static std::map<uint32_t, uint32_t> tru;       // the coherent value of every word in C
static long n_hit = 0, n_fill = 0, n_co = 0, n_push = 0, n_snoop = 0, n_inv = 0, n_dmaw_c = 0, n_dmar_c = 0, n_raw = 0;
static long stale = 0, unpushed = 0;
static uint32_t snap[8];                       // the line at the last read snoop's sn_ack
static uint32_t snap_line = ~0u, hot_line = ~0u;
static uint32_t last_w = 0; static int last_w_ok = 0;
static uint32_t fill_line = ~0u;                // master 0's fill, from its acknowledge to its install

// the model of the internal registers
static uint8_t irq_en = 0;
static uint32_t pci_addr = 0, tg_shadow[64] = {};
static long n88 = 0, n8c = 0, n98 = 0, nunm = 0, n_io = 0, n_jt = 0, n_sc = 0;
static std::deque<std::pair<int, uint32_t>> tg_regs_exp;   // (word, LE value)
static std::deque<std::pair<int, uint16_t>> pal_exp;

// the Real3D host port's writes, in bus order (R112: with R3P they are posted; each strobe must be the
// next of these, and a status read's strobe must find none still owed)
static std::deque<std::pair<uint32_t, uint32_t>> r3_exp;
static long r3_bad = 0, r3_n = 0;
static long r3_order = 0;           // an access to another bus target taken with host-port writes still owed (MPC105 8.1.2.2: one PCI queue)
static int r3_ready = 10;          // the host port ready this many clocks in ten
static uint8_t io_byte(int off) { return uint8_t(0x5A ^ (off * 17)); }
static uint32_t sc_data(int w) { return 0xC0DE0000u | uint32_t(w); }

static Txn gen(int m) {
  Txn x;
  int r = int(rng() % 100);
  x.be = 0xF;
  for (auto &w : x.wd) w = rng();
  auto mem_addr = [&]() {
    uint32_t a = (rng() & 1) ? (0x00010000u + (rng() & 0x7FFC)) : (0xF1000000u + 0x8000 + (rng() & 0x7FFC));
    return a;
  };
  auto c_addr = [&]() { return C_LO + (rng() % (C_HI - C_LO)) & ~3u; };
  if (m == 1) {        // the DMA: memory reads and writes, Real3D writes
    if (r < 35) { x.cmd = (rng() & 1) ? 0 : 2; x.addr = (rng() & 1) ? c_addr() : mem_addr(); if (x.cmd == 0) x.addr &= ~31u; }
    else if (r < 55) {
      x.cmd = (rng() & 1) ? 1 : 3; x.addr = (rng() % 4) ? c_addr() : (0x00014000u + (rng() & 0x3FFC));
      if (x.cmd == 1) x.addr &= ~31u; else x.be = 1 + rng() % 15;
    }
    else if (r < 80) { x.cmd = 1; x.addr = ((rng() & 1) ? 0x8C000000u : 0x98000000u) + (rng() & 0xFFE0); x.kind = (x.addr >> 24) == 0x8C ? K_R3D8C : K_R3D98; }
    else { x.cmd = 3; x.addr = (rng() & 1) ? 0x88000000u : 0x98000000u + (rng() & 0xFFFC); x.kind = x.addr == 0x88000000u ? K_R3D88 : K_R3D98; }
    if (x.cmd >= 2 || x.cmd == 0) {}
    if (x.kind == 0) { x.kind = K_MEM; }
    return x;
  }
  if (r < 55) {
    x.kind = K_MEM; x.addr = mem_addr();
    if (in_c(x.addr)) x.addr ^= 0x4000;        // not the cached lines: uncached access to them is not coherent
    int c = int(rng() % 4);
    x.cmd = c;
    if (c == 3) x.be = rng() & 15;
    if (c == 1) x.addr &= ~31u;
    if ((c == 0 || c == 2) && last_w_ok && (rng() & 1)) { x.addr = c == 0 ? (last_w & ~31u) : last_w; n_raw++; }
  } else if (r < 60) { x.kind = K_IRQEN; x.cmd = (rng() & 1) ? 3 : 2; x.addr = 0xF0100014; x.be = 8; }
  else if (r < 64) { x.kind = K_PCIA; x.cmd = (rng() & 1) ? 3 : 2; x.addr = 0xF0800CF8; }
  else if (r < 70) { x.kind = K_TGREG; x.cmd = (rng() & 1) ? 3 : 2; x.addr = 0xF1180000 + 4 * (rng() % 64); if (x.addr == 0xF1180010 || x.addr == 0xF1180008) x.addr += 0x20; }
  else if (r < 74) { x.kind = K_PAL; x.cmd = 3; x.addr = 0xF1100000 + 4 * (rng() % 32768); }
  else if (r < 78) { x.kind = K_R3DST; x.cmd = 2; x.addr = 0x84000000; }
  else if (r < 84) { x.kind = K_SCSI; x.cmd = (rng() & 1) ? 3 : 2; x.addr = 0xC0000000 + 4 * (rng() % 32); }
  else if (r < 90) { x.kind = K_IO; x.cmd = 2; x.addr = 0xF0040000 + 4 * (rng() % 16); x.be = 8; }
  else if (r < 95) { x.kind = K_JTAG; x.cmd = 3; x.addr = 0xF010000C; x.be = 8; }
  else if (r < 98) { x.kind = K_UNM; x.cmd = 2; x.addr = 0xA0000000 + (rng() & 0xFFFC); }
  else { x.kind = K_R3D88; x.cmd = 3; x.addr = 0x88000000; }
  return x;
}

// the effect a transaction must have once it is acknowledged (writes), and
// what its reads must return (computed at acknowledge: in order with the rest)
static void at_ack(Txn &x, int m) {
  // the 53C810 in the coherent region
  if (m == 1 && x.kind == K_MEM && in_c(x.addr)) {
    uint32_t const ln = x.addr >> 5;
    if (x.cmd == 1 || x.cmd == 3) {
      n_dmaw_c++;
      if (dc.count(ln) || fill_line == ln) { if (stale++ < 5) std::printf("cycle %lu: 53C810 write to %08X lands with the line still in the CPU's cache\n", (unsigned long)cyc, x.addr); }
      if (x.cmd == 3) {
        uint32_t &w = tru[x.addr & ~3u];
        for (int k = 0; k < 4; k++) if ((x.be >> (3 - k)) & 1) w = (w & ~(0xFFu << (24 - 8 * k))) | (x.wd[0] & (0xFFu << (24 - 8 * k)));
      } else for (int k = 0; k < 8; k++) tru[x.addr + 4 * k] = x.wd[k];
    } else {
      n_dmar_c++;
      int const nw = x.cmd == 0 ? 8 : 1;
      for (int k = 0; k < nw; k++) {
        uint32_t const a = x.cmd == 0 ? (x.addr & ~31u) + 4 * k : x.addr;
        if (snap_line != ln || ref_word(a) != snap[(a >> 2) & 7]) { if (unpushed++ < 5) std::printf("cycle %lu: 53C810 read of %08X: memory %08X, the line at the snoop %08X\n", (unsigned long)cyc, a, ref_word(a), snap_line == ln ? snap[(a >> 2) & 7] : 0xDEADDEAD); break; }
      }
    }
  }
  if (x.kind == K_MEM && (x.cmd == 1 || x.cmd == 3) && !in_c(x.addr)) { last_w = x.addr; last_w_ok = 1; }
  // R114: every access to a bus target other than the SDRAM is taken after every host-port write before it
  // has gone to the Real3D (a posted write of the host port is the one exception)
  if (x.kind != K_MEM && x.kind != K_FILL && !((x.kind == K_R3D88 || x.kind == K_R3D8C || x.kind == K_R3D98) && (x.cmd == 1 || x.cmd == 3)) &&
      !r3_exp.empty()) {
    if (r3_order++ < 5) std::printf("cycle %lu: master %d kind %d %08X taken with %zu host-port writes owed\n", (unsigned long)cyc, m, x.kind, x.addr, r3_exp.size());
  }
  switch (x.kind) {
    case K_FILL: { fill_line = x.addr >> 5; uint32_t c = (x.addr >> 2) & 7; for (int k = 0; k < 8; k++) x.exp.push_back(tru[(x.addr & ~31u) + 4 * ((c + k) & 7)]); break; }
    case K_MEM:
      if (x.cmd == 3) ref_write(x.addr, x.wd[0], x.be);
      else if (x.cmd == 1) for (int k = 0; k < 8; k++) ref_write(x.addr + 4 * k, x.wd[k], 15);
      else if (x.cmd == 2) x.exp = {ref_word(x.addr)};
      else { uint32_t c = (x.addr >> 2) & 7; for (int k = 0; k < 8; k++) x.exp.push_back(ref_word((x.addr & ~31u) + 4 * ((c + k) & 7))); }
      break;
    case K_IRQEN:
      if (x.cmd == 3) irq_en = uint8_t(x.wd[0] >> 24); else x.exp = {(uint32_t(irq_en) << 24) | 0xFFFFFF};
      break;
    case K_PCIA:
      if (x.cmd == 3) pci_addr = bswap(x.wd[0]); else x.exp = {bswap(pci_addr)};
      break;
    case K_TGREG: {
      int w = int((x.addr >> 2) & 63);
      if (x.cmd == 3) { tg_shadow[w] = bswap(x.wd[0]); tg_regs_exp.push_back({w, bswap(x.wd[0])}); }
      else x.exp = {bswap(tg_shadow[w])};
      break;
    }
    case K_PAL: pal_exp.push_back({int((x.addr >> 2) & 0x7FFF), uint16_t(((x.wd[0] >> 16) & 0xFF) << 8 | (x.wd[0] >> 24))}); break;
    case K_R3D88: n88++; r3_exp.push_back({x.addr, bswap(x.wd[0])}); break;
    case K_R3D8C: case K_R3D98:
      (x.kind == K_R3D8C ? n8c : n98) += x.cmd == 1 ? 8 : 1;
      if (x.cmd == 1) for (int k = 0; k < 8; k++) r3_exp.push_back({x.addr + 4 * k, bswap(x.wd[k])});
      else r3_exp.push_back({x.addr, bswap(x.wd[0])});
      break;
    case K_R3DST: x.check = false; break;
    case K_SCSI: n_sc++; if (x.cmd == 2) x.exp = {sc_data(int((x.addr >> 2) & 31))}; break;
    case K_IO: n_io++; x.exp = {uint32_t(io_byte(int((x.addr >> 2) & 15))) << 24}; break;
    case K_JTAG: n_jt++; break;
    case K_UNM: nunm++; x.exp = {0xFFFFFFFF}; break;
  }
}

struct Master {
  int id;
  bool busy = false, acked = false;
  Txn x;
  int beats = 0, wbeat = 0;
  long done = 0;
  uint32_t got[8];
  bool cop = false, cop_st = false; uint32_t cop_a = 0, cop_v = 0;   // master 0: the cached access waiting on its fill
  int push = 0;                                                     // master 0: 1 the transaction is a snoop push
};

// a cached access by master 0 to a line it holds
static void c_access(bool st, uint32_t a, uint32_t v) {
  Line &l = dc[a >> 5];
  if (st) { l.d[(a >> 2) & 7] = v; l.dirty = true; tru[a] = v; }
  else if (l.d[(a >> 2) & 7] != tru[a]) { if (stale++ < 5) std::printf("cycle %lu: CPU load hit %08X: %08X, coherent %08X\n", (unsigned long)cyc, a, l.d[(a >> 2) & 7], tru[a]); }
  n_hit++;
}

int main(int argc, char **argv) {
  long n = argc > 1 ? std::atol(argv[1]) : 20000;
  rng.seed(argc > 2 ? unsigned(std::atoi(argv[2])) : 1);
  for (int k = 3; k < argc; k++) if (!std::strcmp(argv[k], "--bxor")) sd.bxor = true;   // the build's BXOR (study R107)
  for (int k = 3; k < argc; k++) if (!std::strcmp(argv[k], "--r3stall")) r3_ready = 6;   // the host port not ready 40% of clocks (R112)
  for (int k = 3; k < argc; k++) if (!std::strcmp(argv[k], "--r3slow")) r3_ready = 1;    // ready one clock in ten: the post buffer fills (R112)
  t = new Vm3_bus_tb;
  ref.assign(size_t(1) << 27, 0);
  for (uint32_t a = 0; a < 0x10000; a += 2) {
    for (uint32_t base : {0x6410000u, 0x6C08000u}) {
      uint16_t v = uint16_t(rng()); sd.m[(base + a) >> 1] = v; ref[base + a] = uint8_t(v); ref[base + a + 1] = uint8_t(v >> 8);
    }
  }
  t->rst_n = 0; t->rst = 1; t->m_req = 0; t->sc_ack = 0; t->cf_ack = 0; t->jt_busy = 0; t->ln_start = 0;
  t->sc_win_en = 1; t->sc_win_base = 0xC00000; t->scsi_irq = 0; t->jt_tdo = 0;
  t->h_rdy_i = 1;
  auto tick = [&]() {
    t->sd_dq_i = sd.dq(cyc);
    t->io_rbyte = io_byte(t->io_off);
    t->h_rdy_i = r3_ready >= 10 ? 1 : int(rng() % 10) < r3_ready;
    t->clk = 0; t->eval();
    // the host port's strobe at this edge
    if (t->h_go_o && t->h_rdy_i) {
      if (t->h_we_o) {
        r3_n++;
        if (r3_exp.empty() || r3_exp.front().first != t->h_addr_o || r3_exp.front().second != t->h_wdata_o) {
          if (r3_bad++ < 5) std::printf("cycle %lu: host port write %08X %08X, expected %08X %08X\n", (unsigned long)cyc, t->h_addr_o, t->h_wdata_o,
                                        r3_exp.empty() ? 0 : r3_exp.front().first, r3_exp.empty() ? 0 : r3_exp.front().second);
        }
        if (!r3_exp.empty()) r3_exp.pop_front();
      } else if (!r3_exp.empty()) {
        if (r3_bad++ < 5) std::printf("cycle %lu: host port read with %zu earlier writes not yet given\n", (unsigned long)cyc, r3_exp.size());
      }
    }
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
    t->clk = 1; t->eval();
    cyc++;
  };
  for (int i = 0; i < 4; i++) tick();
  t->rst_n = 1; t->rst = 0;
  while (!t->ready) tick();
  for (uint32_t a = C_LO; a < C_HI; a += 4) tru[a] = ref_word(a);
  int sn_st = 0, sn_dly = 0;       // the snoop: 0 idle, 1 waiting for master 0 quiet, 2 push out, 3 acknowledge

  Master M[2] = {{0}, {1}};
  long issued = 0, sc_wait = 0, jt_left = 0;
  bool sc_pend = false;
  long tg_bad = 0, pal_bad = 0, io_sel_n = 0, jt_wr_n = 0, jt_early = 0;
  for (long guard = 0; guard < 400 * n && (issued < n || M[0].busy || M[1].busy); guard++) {
    // masters present their requests
    // the snoop (above): taken when master 0 is quiet
    t->sn_ack = 0;
    if (sn_st == 0 && t->sn_req) { sn_st = 1; n_snoop++; }
    if (sn_st == 1 && !M[0].busy) {
      uint32_t const ln = t->sn_addr;
      auto it = dc.find(ln);
      if (it != dc.end() && it->second.dirty) {
        Txn &x = M[0].x; x = Txn(); x.cmd = 1; x.addr = ln << 5; x.be = 15; x.kind = K_MEM;
        for (int k = 0; k < 8; k++) x.wd[k] = it->second.d[k];
        M[0].busy = true; M[0].acked = false; M[0].beats = 0; M[0].wbeat = 0; M[0].push = 1; M[0].cop = false;
        sn_st = 2; n_push++;
      } else { sn_st = 3; sn_dly = int(rng() % 3); }
    }
    if (sn_st == 2 && !M[0].busy) { sn_st = 3; sn_dly = 0; }
    if (sn_st == 3 && sn_dly-- <= 0) {
      uint32_t const ln = t->sn_addr;
      if (t->sn_inv) { if (dc.erase(ln)) n_inv++; if (in_c(ln << 5)) hot_line = ln; }
      else { auto it = dc.find(ln); if (it != dc.end()) it->second.dirty = false; snap_line = ln; for (int k = 0; k < 8; k++) snap[k] = tru[(ln << 5) + 4 * k]; }
      t->sn_ack = 1; sn_st = 0;
    }
    for (auto &m : M) {
      // master 0's cached accesses (not while a snoop is being answered)
      if (m.id == 0 && !m.busy && issued < n && sn_st == 0 && ((rng() % 3) == 0 || (t->sn_ack && hot_line != ~0u))) {
        uint32_t a = (C_LO + (rng() % (C_HI - C_LO))) & ~3u;
        if (hot_line != ~0u && (t->sn_ack || (rng() & 1))) { a = (hot_line << 5) + 4 * (rng() & 7); hot_line = ~0u; }
        bool const st = rng() & 1; uint32_t const v = rng();
        issued++;
        if (dc.count(a >> 5)) { c_access(st, a, v); m.done++; }
        else {
          if (dc.size() >= 6) {
            auto vi = dc.begin(); std::advance(vi, rng() % dc.size());
            if (vi->second.dirty) {       // the cast-out first, then the access is tried again later
              m.x = Txn(); m.x.cmd = 1; m.x.addr = vi->first << 5; m.x.be = 15; m.x.kind = K_MEM;
              for (int k = 0; k < 8; k++) m.x.wd[k] = vi->second.d[k];
              dc.erase(vi); n_co++;
              m.busy = true; m.acked = false; m.beats = 0; m.wbeat = 0; m.cop = false; m.push = 0;
              goto present;
            }
            dc.erase(vi);
          }
          m.x = Txn(); m.x.cmd = 0; m.x.addr = a; m.x.be = 15; m.x.kind = K_FILL;
          m.busy = true; m.acked = false; m.beats = 0; m.wbeat = 0; m.push = 0;
          m.cop = true; m.cop_st = st; m.cop_a = a; m.cop_v = v; n_fill++;
        }
      }
      if (!m.busy && issued < n && (rng() % 4) != 0 && !(m.id == 0 && sn_st != 0)) { m.x = gen(m.id); m.busy = true; m.acked = false; m.beats = 0; m.wbeat = 0; m.cop = false; m.push = 0; issued++; }
      present:
      int idx = m.id;
      uint32_t req = t->m_req;
      if (m.busy && !m.acked) req |= (1u << idx); else req &= ~(1u << idx);
      t->m_req = req;
      t->m_cmd = uint8_t((t->m_cmd & ~(3u << (2 * idx))) | (uint32_t(m.x.cmd) << (2 * idx)));
      t->m_addr = (t->m_addr & ~(0xFFFFFFFFull << (32 * idx))) | (uint64_t(m.x.addr) << (32 * idx));
      t->m_be = uint8_t((t->m_be & ~(15u << (4 * idx))) | (m.x.be << (4 * idx)));
      t->m_wdata = (t->m_wdata & ~(0xFFFFFFFFull << (32 * idx))) | (uint64_t(m.x.wd[m.wbeat < 8 ? m.wbeat : 7]) << (32 * idx));
    }
    // the 53C810 window: acknowledge after a random delay
    if (t->sc_req && !sc_pend) { sc_pend = true; sc_wait = long(rng() % 6); }
    t->sc_ack = 0;
    if (sc_pend && sc_wait-- <= 0) { t->sc_ack = 1; t->sc_rdata = sc_data(t->sc_addr); sc_pend = false; }
    t->cf_ack = t->cf_req;
    // the JTAG chain: busy for a while after each write
    t->jt_busy = jt_left > 0;
    // a line event now and then (ping-pong, IRQs)
    t->ln_start = (rng() % 200) == 0; t->ln = uint16_t(rng() % 424);
    // outputs seen this cycle (registered: they describe the last edge)
    uint32_t const ack = t->m_ack, rv = t->m_rvalid, wn = t->m_wnext, rd = t->m_rdata;
    if (t->tg_reg_we) {
      if (tg_regs_exp.empty() || tg_regs_exp.front().first != t->tg_reg_addr || tg_regs_exp.front().second != t->tg_reg_wdata) tg_bad++;
      if (!tg_regs_exp.empty()) tg_regs_exp.pop_front();
    }
    if (t->tg_pal_we) {
      if (pal_exp.empty() || pal_exp.front().first != t->tg_pal_addr || pal_exp.front().second != t->tg_pal_wdata) pal_bad++;
      if (!pal_exp.empty()) pal_exp.pop_front();
    }
    if (t->io_sel) io_sel_n++;
    if (t->jt_wr) { jt_wr_n++; if (t->jt_busy) jt_early++; jt_left = long(rng() % 12); }
    else if (jt_left > 0) jt_left--;
    for (auto &m : M) {
      uint32_t const bit = 1u << m.id;
      if (ack & bit) {
        if (!m.busy || m.acked) { proto++; if (proto < 5) std::printf("cycle %lu: master %d acknowledged with nothing pending\n", (unsigned long)cyc, m.id); }
        else { m.acked = true; at_ack(m.x, m.id); }
      }
      if (rv & bit) {
        if (!m.busy || !m.acked) { proto++; if (proto < 5) std::printf("cycle %lu: master %d read beat before acknowledge\n", (unsigned long)cyc, m.id); }
        else {
          if (m.x.check && m.beats < int(m.x.exp.size()) && rd != m.x.exp[m.beats]) {
            if (bad++ < 10) std::printf("cycle %lu: master %d %s %08X beat %d: %08X, expected %08X (kind %d)\n", (unsigned long)cyc, m.id,
                                        m.x.cmd == 0 ? "LINE_RD" : "RD", m.x.addr, m.beats, rd, m.x.exp[m.beats], m.x.kind);
          }
          if (m.x.kind == K_R3DST && (rd | 2) != 0xFFFFFFFF) { if (bad++ < 10) std::printf("status %08X\n", rd); }
          if (m.beats < 8) m.got[m.beats] = rd;
          m.beats++;
        }
      }
      if (wn & bit) { m.wbeat++; }
      // done?
      if (m.busy && m.acked) {
        bool fin = (m.x.cmd == 3) || (m.x.cmd == 2 && m.beats == 1) || (m.x.cmd == 0 && m.beats == 8) || (m.x.cmd == 1 && m.wbeat == 8);
        if (fin) {
          m.busy = false;
          if (m.push) m.push = 0;                      // the snoop's push: not a counted transaction
          else m.done++;
          if (m.cop) {                                 // the fill: install, then the access
            Line &l = dc[m.x.addr >> 5]; uint32_t const c = (m.x.addr >> 2) & 7;
            for (int k = 0; k < 8; k++) l.d[(c + k) & 7] = m.got[k];
            l.dirty = false; m.cop = false; fill_line = ~0u;
            c_access(m.cop_st, m.cop_a, m.cop_v);
          }
        }
      }
    }
    tick();
  }
  for (int i = 0; i < 300; i++) {
    t->m_req = 0; t->sn_ack = 0; t->jt_busy = 0;
    if (t->io_sel) io_sel_n++;
    if (t->jt_wr) jt_wr_n++;
    // strobes of the last writes, a few clocks after their acknowledge
    if (t->tg_reg_we) {
      if (tg_regs_exp.empty() || tg_regs_exp.front().first != t->tg_reg_addr || tg_regs_exp.front().second != t->tg_reg_wdata) tg_bad++;
      if (!tg_regs_exp.empty()) tg_regs_exp.pop_front();
    }
    if (t->tg_pal_we) {
      if (pal_exp.empty() || pal_exp.front().first != t->tg_pal_addr || pal_exp.front().second != t->tg_pal_wdata) pal_bad++;
      if (!pal_exp.empty()) pal_exp.pop_front();
    }
    tick();
  }
  // the memory image
  long img_bad = 0;
  for (uint32_t base : {0x6410000u, 0x6C08000u})
    for (uint32_t a = 0; a < 0x8000; a++) {
      uint8_t got = uint8_t(sd.m[(base + a) >> 1] >> (((base + a) & 1) * 8));
      if (got != ref[base + a]) img_bad++;
    }
  long cnt_bad = 0;
  if (long(t->n_submit) != n88) cnt_bad++;
  if (long(t->n_cull_lo) != n8c) cnt_bad++;
  if (long(t->n_poly) != n98) cnt_bad++;
  if (long(t->n_unmapped) != nunm) cnt_bad++;
  if (io_sel_n != n_io) cnt_bad++;
  if (jt_wr_n != n_jt) cnt_bad++;
  std::printf("m3_bus: %ld + %ld transactions; 88 %ld/%u 8C %ld/%u 98 %ld/%u unmapped %ld/%u io %ld/%ld jtag %ld/%ld (%ld while busy) scsi %ld\n",
              M[0].done, M[1].done, n88, unsigned(t->n_submit), n8c, unsigned(t->n_cull_lo), n98, unsigned(t->n_poly),
              nunm, unsigned(t->n_unmapped), io_sel_n, n_io, jt_wr_n, n_jt, jt_early, n_sc);
  std::printf("  coherent lines: %ld load/store hits, %ld fills, %ld cast-outs, %ld snoops (%ld pushes, %ld invalidates), 53C810 %ld writes / %ld reads; %ld reads after a write\n",
              n_hit, n_fill, n_co, n_snoop, n_push, n_inv, n_dmaw_c, n_dmar_c, n_raw);
  std::printf("  %ld stale (a write landing on a held line, or a stale hit), %ld 53C810 reads before the push\n", stale, unpushed);
  std::printf("  %ld read mismatches, %ld protocol errors, %ld image bytes wrong, %ld tile register / %ld palette strobes wrong, %ld counters wrong, %zu/%zu strobes missing\n",
              bad, proto, img_bad, tg_bad, pal_bad, cnt_bad, tg_regs_exp.size(), pal_exp.size());
  std::printf("  host port: %ld writes in order, %ld wrong or out of order, %zu not given, %ld accesses ahead of them\n", r3_n, r3_bad, r3_exp.size(), r3_order);
  bool ok = !r3_bad && !r3_order && r3_exp.empty() && !stale && !unpushed && n_push > 0 && n_inv > 0 && !bad && !proto && !img_bad && !tg_bad && !pal_bad && !cnt_bad && !jt_early && tg_regs_exp.empty() && pal_exp.empty() && M[0].done + M[1].done == n;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
