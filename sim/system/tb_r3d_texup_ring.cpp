// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// r3d_texup's ring when uploads are held back (study R100, hand-off 5):
// Daytona 2's boot sends its texture sheets through the FIFO (94), one
// 256 x 256 16-bit texture with its mip levels (43,692 words) and a command
// port write (the flush) at a time, while r3d_cmd starts no upload (frames in
// flight). The ring holds 2^RW words. Two instances get the same textures:
//   A  uploads are held back: no go until HOLD textures have been sent (or the
//      sender has waited on t_ready for 2,000 cycles), then a mark and a go
//      whenever the last upload has finished and words are flushed (r3d_cmd's
//      between-frames rule)
//   B  a mark and a go after every texture (the ring never holds more than one)
// The sheets must come out the same, and A must report every texture (n_tex).
//   Vr3d_texup [textures] [hold] [seed] [log2 side]
#include "Vr3d_texup.h"
#include "Vr3d_texup___024root.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <map>
#include <random>
#include <vector>
#ifndef RING_RW
#define RING_RW 18
#endif

struct Mem {   // the DDR3 at r3d_texup's d_* port (VROM unused: FIFO only)
  std::map<uint32_t, uint64_t> ddr;
  struct rd { uint32_t a, n; uint64_t at; };
  std::deque<rd> q;
  bool wr = false, busy = false; uint32_t wa = 0; int ack_wait = 0;
};

static uint64_t cyc;

static void tick(Vr3d_texup *t, Mem &m, std::mt19937 &rng) {
  t->d_ack = 0; t->d_rvalid = 0; t->d_wacc = 0; t->s_ack = 0;
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
  t->clk = 0; t->eval(); t->clk = 1; t->eval();
}

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
  int const ntex = argc > 1 ? std::atoi(argv[1]) : 12;
  int const hold = argc > 2 ? std::atoi(argv[2]) : 11;
  std::mt19937 rng(argc > 3 ? unsigned(std::atoi(argv[3])) : 1);
  int const ls = argc > 4 ? std::atoi(argv[4]) : 8;          // log2 side: 8 = 256 x 256, Daytona 2's
  Vr3d_texup *a = new Vr3d_texup, *b = new Vr3d_texup;
  Mem ma, mb;
  a->rst = b->rst = 1; a->step2 = b->step2 = 1;
  a->t_valid = b->t_valid = 0; a->t_flush = b->t_flush = 0; a->mark = b->mark = 0; a->go = b->go = 0;
  for (int i = 0; i < 4; i++) { tick(a, ma, rng); tick(b, mb, rng); cyc++; }
  a->rst = b->rst = 0;
  // the textures: Daytona 2's boot headers (type 0, 16-bit, both bytes), tiles of the two pages
  std::vector<std::vector<uint32_t>> tex;
  uint32_t const sc = uint32_t(ls - 5);
  for (int k = 0; k < ntex; k++) {
    uint32_t const tiles = 2048u >> ls;
    uint32_t const x32 = (uint32_t(k) % tiles) << (ls - 5), y32 = ((uint32_t(k) / tiles) % (1024u >> ls)) << (ls - 5);
    uint32_t const h = (0u << 24) | (1u << 23) | (3u << 21) | ((uint32_t(k / 16) & 1) << 20) | (sc << 17) | (sc << 14) | (y32 << 7) | x32;
    uint32_t const nw = words(h);
    std::vector<uint32_t> w;
    w.push_back(8 * (nw + 2) - 4); w.push_back(h);
    for (uint32_t i = 0; i < nw; i++) w.push_back(rng());
    tex.push_back(w);
  }
  // B: one texture at a time
  auto run_b = [&]() {
    for (auto const &w : tex) {
      for (uint32_t d : w) {
        b->t_valid = 1; b->t_vrom = 0; b->t_data = d;
        while (true) { bool r = b->t_ready; tick(b, mb, rng); cyc++; if (r) break; }
        b->t_valid = 0;
      }
      while (!b->t_ready) { tick(b, mb, rng); cyc++; }
      b->t_flush = 1; b->mark = 1; tick(b, mb, rng); cyc++; b->t_flush = 0; b->mark = 0;
      b->go = 1; tick(b, mb, rng); cyc++; b->go = 0;
      long g = 0;
      while ((b->busy || b->t_mpend) && g++ < 400000000) { tick(b, mb, rng); cyc++; }
    }
  };
  run_b();
  // A: held back, then r3d_cmd's between-frames rule
  bool released = false;
  int sent = 0;
  long waited = 0, max_wait = 0;
  int mstate = 0;   // 0 idle, 1 marked (go next)
  auto step_a = [&]() {
    a->mark = 0; a->go = 0;
    if (released) {
      if (mstate == 1) { a->go = 1; mstate = 0; }
      else if (!a->busy && a->t_pend) { a->mark = 1; mstate = 1; }
    }
    tick(a, ma, rng); cyc++;
  };
  for (auto const &w : tex) {
    for (uint32_t d : w) {
      a->t_valid = 1; a->t_vrom = 0; a->t_data = d;
      long g = 0;
      while (true) {
        bool r = a->t_ready;
        step_a();
        if (r) break;
        if (++g > 2000 && !released) released = true;
        if (g > 400000000) { std::printf("A stuck\n"); return 2; }
      }
      waited += g; if (g > max_wait) max_wait = g;
      a->t_valid = 0;
    }
    while (!a->t_ready) step_a();
    a->t_flush = 1; step_a(); a->t_flush = 0;
    if (++sent >= hold) released = true;
  }
  released = true;
  for (long g = 0; (a->busy || a->t_pend || mstate) && g < 400000000; g++) step_a();
  for (int k = 0; k < 16; k++) step_a();
  long d = 0;
  for (auto &e : mb.ddr) if (e.first >= 0xC00000) { auto it = ma.ddr.find(e.first); if (it == ma.ddr.end() || it->second != e.second) d++; }
  for (auto &e : ma.ddr) if (e.first >= 0xC00000 && !mb.ddr.count(e.first)) d++;
  unsigned const na = a->n_tex, nb = b->n_tex;
  std::printf("%d textures of %u words (ring 2^%d words), uploads held for %d: A uploaded %u, B %u; sender waited %ld cycles (longest %ld); "
              "%ld sheet beats differ -> %s\n", ntex, uint32_t(tex[0].size()), RING_RW, hold, na, nb, waited,
              max_wait, d, (d == 0 && na == nb && nb == unsigned(ntex)) ? "PASS" : "FAIL");
  delete a; delete b;
  return (d == 0 && na == nb) ? 0 : 1;
}
