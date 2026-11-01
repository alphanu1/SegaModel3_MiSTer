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
//   Vm3_bus_tb TRANSACTIONS SEED
#include "Vm3_bus_tb.h"
#include "verilated.h"
#include "sdram_dev.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
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
enum { K_MEM, K_IRQEN, K_PCIA, K_TGREG, K_PAL, K_R3D88, K_R3D8C, K_R3D98, K_R3DST, K_SCSI, K_IO, K_JTAG, K_UNM };

// the model of the internal registers
static uint8_t irq_en = 0;
static uint32_t pci_addr = 0, tg_shadow[64] = {};
static long n88 = 0, n8c = 0, n98 = 0, nunm = 0, n_io = 0, n_jt = 0, n_sc = 0;
static std::deque<std::pair<int, uint32_t>> tg_regs_exp;   // (word, LE value)
static std::deque<std::pair<int, uint16_t>> pal_exp;

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
  if (m == 1) {        // the DMA: memory reads and Real3D writes
    if (r < 50) { x.cmd = (rng() & 1) ? 0 : 2; x.addr = mem_addr(); if (x.cmd == 0) x.addr &= ~31u; }
    else if (r < 80) { x.cmd = 1; x.addr = ((rng() & 1) ? 0x8C000000u : 0x98000000u) + (rng() & 0xFFE0); x.kind = (x.addr >> 24) == 0x8C ? K_R3D8C : K_R3D98; }
    else { x.cmd = 3; x.addr = (rng() & 1) ? 0x88000000u : 0x98000000u + (rng() & 0xFFFC); x.kind = x.addr == 0x88000000u ? K_R3D88 : K_R3D98; }
    if (x.cmd >= 2 || x.cmd == 0) {}
    if (x.kind == 0) { x.kind = K_MEM; }
    return x;
  }
  if (r < 55) {
    x.kind = K_MEM; x.addr = mem_addr();
    int c = int(rng() % 4);
    x.cmd = c;
    if (c == 3) x.be = rng() & 15;
    if (c == 1) x.addr &= ~31u;
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
static void at_ack(Txn &x) {
  switch (x.kind) {
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
    case K_R3D88: n88++; break;
    case K_R3D8C: n8c += x.cmd == 1 ? 8 : 1; break;
    case K_R3D98: n98 += x.cmd == 1 ? 8 : 1; break;
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
};

int main(int argc, char **argv) {
  long n = argc > 1 ? std::atol(argv[1]) : 20000;
  rng.seed(argc > 2 ? unsigned(std::atoi(argv[2])) : 1);
  t = new Vm3_bus_tb;
  ref.assign(size_t(1) << 27, 0);
  for (uint32_t a = 0; a < 0x10000; a += 2) {
    for (uint32_t base : {0x6410000u, 0x6C08000u}) {
      uint16_t v = uint16_t(rng()); sd.m[(base + a) >> 1] = v; ref[base + a] = uint8_t(v); ref[base + a + 1] = uint8_t(v >> 8);
    }
  }
  t->rst_n = 0; t->rst = 1; t->m_req = 0; t->sc_ack = 0; t->cf_ack = 0; t->jt_busy = 0; t->ln_start = 0;
  t->sc_win_en = 1; t->sc_win_base = 0xC00000; t->scsi_irq = 0; t->jt_tdo = 0;
  auto tick = [&]() {
    t->sd_dq_i = sd.dq(cyc);
    t->io_rbyte = io_byte(t->io_off);
    t->clk = 0; t->eval();
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
    t->clk = 1; t->eval();
    cyc++;
  };
  for (int i = 0; i < 4; i++) tick();
  t->rst_n = 1; t->rst = 0;
  while (!t->ready) tick();

  Master M[2] = {{0}, {1}};
  long issued = 0, sc_wait = 0, jt_left = 0;
  bool sc_pend = false;
  long tg_bad = 0, pal_bad = 0, io_sel_n = 0, jt_wr_n = 0, jt_early = 0;
  for (long guard = 0; guard < 400 * n && (issued < n || M[0].busy || M[1].busy); guard++) {
    // masters present their requests
    for (auto &m : M) {
      if (!m.busy && issued < n && (rng() % 4) != 0) { m.x = gen(m.id); m.busy = true; m.acked = false; m.beats = 0; m.wbeat = 0; issued++; }
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
        else { m.acked = true; at_ack(m.x); }
      }
      if (rv & bit) {
        if (!m.busy || !m.acked) { proto++; if (proto < 5) std::printf("cycle %lu: master %d read beat before acknowledge\n", (unsigned long)cyc, m.id); }
        else {
          if (m.x.check && m.beats < int(m.x.exp.size()) && rd != m.x.exp[m.beats]) {
            if (bad++ < 10) std::printf("cycle %lu: master %d %s %08X beat %d: %08X, expected %08X (kind %d)\n", (unsigned long)cyc, m.id,
                                        m.x.cmd == 0 ? "LINE_RD" : "RD", m.x.addr, m.beats, rd, m.x.exp[m.beats], m.x.kind);
          }
          if (m.x.kind == K_R3DST && (rd | 2) != 0xFFFFFFFF) { if (bad++ < 10) std::printf("status %08X\n", rd); }
          m.beats++;
        }
      }
      if (wn & bit) { m.wbeat++; }
      // done?
      if (m.busy && m.acked) {
        bool fin = (m.x.cmd == 3) || (m.x.cmd == 2 && m.beats == 1) || (m.x.cmd == 0 && m.beats == 8) || (m.x.cmd == 1 && m.wbeat == 8);
        if (fin) { m.busy = false; m.done++; }
      }
    }
    tick();
  }
  for (int i = 0; i < 300; i++) { t->m_req = 0; tick(); }
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
  std::printf("  %ld read mismatches, %ld protocol errors, %ld image bytes wrong, %ld tile register / %ld palette strobes wrong, %ld counters wrong, %zu/%zu strobes missing\n",
              bad, proto, img_bad, tg_bad, pal_bad, cnt_bad, tg_regs_exp.size(), pal_exp.size());
  bool ok = !bad && !proto && !img_bad && !tg_bad && !pal_bad && !cnt_bad && !jt_early && tg_regs_exp.empty() && pal_exp.empty() && M[0].done + M[1].done == n;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
