// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// r3d_texup's Step 2.x texture port (STEP2_PORT, study R100) against its
// FIFO path: each batch of random textures (types 0-3, 32-128 texels a side,
// 8 and 16 bits, either byte lane, any page and position) is uploaded twice,
// by two instances, and the texture sheets must come out the same:
//   A  the FIFO (94): for each texture a size word, the header and its texel
//      words, then words of garbage, so a decode that reads past the words the
//      port would copy sees different data in A and B
//   B  the Step 2 port (90): the VROM word address and the header; the texels
//      are in VROM (64 MiB, the MRA's four 16 MiB blocks)
// The size word in A is the one the port makes, plus the garbage; the batch's
// sheets are compared texel for texel after every upload.
//   Vr3d_texup [batches] [seed]
#include "Vr3d_texup.h"
#include "Vr3d_texup___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <random>
#include <vector>

struct Mem {   // the DDR3 at r3d_texup's d_* port, and VROM at its s_* port
  std::map<uint32_t, uint64_t> ddr;
  std::vector<uint32_t> *vrom;
  struct rd { uint32_t a, n; uint64_t at; };
  std::deque<rd> q;
  bool wr = false, busy = false; uint32_t wa = 0; int ack_wait = 0;
  int s_wait = -1;
};

static uint64_t cyc;

static void tick(Vr3d_texup *t, Mem &m, std::mt19937 &rng) {
  t->d_ack = 0; t->d_rvalid = 0; t->d_wacc = 0; t->s_ack = 0;
  // DDR3: one transaction at a time; a 1-beat write is taken and acknowledged, a read's beats follow
  if (t->d_req && !m.busy) {
    m.busy = true;
    if (t->d_we) { m.wr = true; m.wa = t->d_addr; m.ack_wait = 1 + int(rng() % 4); }
    else { m.wr = false; m.q.push_back({uint32_t(t->d_addr), uint32_t(t->d_blen ? t->d_blen : 256), cyc + 6 + rng() % 20}); }
  }
  if (m.busy && m.wr) {
    if (--m.ack_wait == 0) {
      uint64_t &w = m.ddr[m.wa];
      for (int b = 0; b < 8; b++) if ((t->d_be >> b) & 1) w = (w & ~(0xFFull << (8 * b))) | (t->d_din & (0xFFull << (8 * b)));
      t->d_wacc = 1; t->d_ack = 1; m.busy = false;
    }
  } else if (m.busy && !m.q.empty() && m.q.front().at <= cyc) {
    auto &r = m.q.front();
    auto it = m.ddr.find(r.a);
    t->d_rvalid = 1; t->d_dout = it == m.ddr.end() ? 0 : it->second;
    r.a++;
    if (--r.n == 0) { t->d_ack = 1; m.busy = false; m.q.pop_front(); }
  }
  // VROM (m2_sdram's p_ contract, a 64-bit read a request)
  if (t->s_req && m.s_wait < 0) m.s_wait = 2 + int(rng() % 10);
  if (m.s_wait == 0) {
    uint32_t const off = uint32_t(t->s_addr) - 0x2000000u, beat = off >> 2, b = beat >> 21, g = beat & 0x1FFFFF;
    uint32_t const i0 = (g << 3) | (b << 1);
    t->s_dout = (uint64_t((*m.vrom)[i0 + 1]) << 32) | (*m.vrom)[i0];
    t->s_ack = 1; m.s_wait = -1;
  } else if (m.s_wait > 0) m.s_wait--;
  t->clk = 0; t->eval(); t->clk = 1; t->eval();
}

// texel words of a texture, as the decode consumes them
static uint32_t words(uint32_t h) {
  unsigned const type = h >> 24, b16 = (h >> 23) & 1, lw0 = 5 + ((h >> 14) & 7), lh0 = 5 + ((h >> 17) & 7);
  if (type > 2) return 0;
  unsigned const mn = lw0 < lh0 ? lw0 : lh0, first = type == 2 ? 1 : 0, last = type == 1 ? 0 : mn - 1;
  uint32_t n = 0;
  for (unsigned lv = first; lv <= last; lv++) n += 1u << (lw0 + lh0 - 2 * lv - 1 - (b16 ? 0 : 1));
  return n;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long nb = argc > 1 ? std::atol(argv[1]) : 200;
  std::mt19937 rng(argc > 2 ? unsigned(std::atoi(argv[2])) : 1);
  std::vector<uint32_t> vrom(1u << 24);
  for (auto &w : vrom) w = rng();
  Vr3d_texup *a = new Vr3d_texup, *b = new Vr3d_texup;
  Mem ma, mb; ma.vrom = mb.vrom = &vrom;
  auto both = [&]() { tick(a, ma, rng); tick(b, mb, rng); cyc++; };
  a->rst = b->rst = 1; a->step2 = 0; b->step2 = 1;
  a->t_valid = b->t_valid = 0; a->t_flush = b->t_flush = 0; a->mark = b->mark = 0; a->go = b->go = 0;
  for (int i = 0; i < 4; i++) both();
  a->rst = b->rst = 0;
  long bad = 0, ntex = 0, texels = 0;
  auto send = [&](Vr3d_texup *t, Mem &m, bool vrom_port, uint32_t d) {
    t->t_valid = 1; t->t_vrom = vrom_port; t->t_data = d;
    long g = 0;
    while (true) { bool r = t->t_ready; tick(t, m, rng); cyc++; if (r) break; if (++g > 20000000) { std::printf("stuck: %s, word %08X, d_req %d busy %d, a_v %d v_on %d v_n %u s_req %d p2_st %d\n", t == a ? "A" : "B", d, int(t->d_req), int(t->busy), int(t->rootp->r3d_texup__DOT__a_v), int(t->rootp->r3d_texup__DOT__v_on), unsigned(t->rootp->r3d_texup__DOT__v_n), int(t->s_req), int(t->rootp->r3d_texup__DOT__p2_st)); std::printf("s_wait %d v_sgap %d s_ack %d v_a %06X\n", m.s_wait, int(t->rootp->r3d_texup__DOT__v_sgap), int(t->s_ack), unsigned(t->rootp->r3d_texup__DOT__v_a)); std::exit(1); } }
    t->t_valid = 0;
  };
  auto upload = [&](Vr3d_texup *t, Mem &m) {
    long g = 0;
    while (!t->t_ready && g++ < 4000000) { tick(t, m, rng); cyc++; }
    t->t_flush = 1; t->mark = 1; tick(t, m, rng); cyc++; t->t_flush = 0; t->mark = 0;
    t->go = 1; tick(t, m, rng); cyc++; t->go = 0;
    g = 0;
    while ((t->busy || t->t_mpend) && g++ < 40000000) { tick(t, m, rng); cyc++; }
    for (int k = 0; k < 8; k++) { tick(t, m, rng); cyc++; }
  };
  for (long n = 0; n < nb; n++) {
    int const k = 1 + int(rng() % 5);
    for (int j = 0; j < k; j++) {
      uint32_t type = (rng() % 16 == 0) ? 3 + rng() % 4 : rng() % 3;
      uint32_t const wc = rng() % 3, hc = rng() % 3;
      uint32_t h = (type << 24) | ((rng() & 1) << 23) | ((rng() & 3) << 21) | ((rng() & 1) << 20) | (hc << 17) | (wc << 14) |
                   ((rng() % 16) << 7) | (rng() % 32);
      uint32_t const nw = words(h);
      uint32_t const addr = uint32_t(rng() % ((1u << 24) - nw - 8));
      int const pad = int(rng() % 8);
      // A: the FIFO
      send(a, ma, false, (8 * (nw + 2 + pad)) - 4);
      send(a, ma, false, h);
      for (uint32_t i = 0; i < nw; i++) send(a, ma, false, vrom[addr + i]);
      for (int i = 0; i < pad; i++) send(a, ma, false, rng());
      // B: the Step 2 port
      send(b, mb, true, addr);
      send(b, mb, true, h);
      ntex++; texels += nw;
    }
    send(a, ma, false, 0); send(a, ma, false, 0); send(a, ma, false, 0);   // a size of 0 ends the list
    upload(a, ma);
    // B's list ends at the mark (fewer than three words left)
    upload(b, mb);
    // the sheets
    long d = 0;
    for (auto &e : ma.ddr) if (e.first >= 0xC00000) { auto it = mb.ddr.find(e.first); if (it == mb.ddr.end() || it->second != e.second) d++; }
    for (auto &e : mb.ddr) if (e.first >= 0xC00000 && !ma.ddr.count(e.first)) d++;
    if (d && bad++ < 10) std::printf("batch %ld: %ld sheet beats differ\n", n, d);
  }
  std::printf("%ld batches, %ld textures, %ld texel words, %zu sheet beats written; %ld batches differ -> %s\n", nb, ntex, texels,
              size_t(ma.ddr.size()), bad, bad ? "FAIL" : "PASS");
  delete a; delete b;
  return bad ? 1 : 0;
}
