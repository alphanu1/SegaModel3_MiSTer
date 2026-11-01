// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_compose against Supermodel's composite (D-37 stage B). The tile pixel is
// Supermodel's: its two tile surfaces from the stage B oracle run (.surf,
// bottom and top, RGBA; R77: the tile RTL equals them), played as the tile
// generator plays its line buffer (drawn, above, colour, three cycles after
// the read). The 3D layer is preloaded in the DDR3 model in m2_fb_read's form.
//
// Checked, every visible pixel of two whole frames after the first:
//   exact against Supermodel's order (bottom surface, then the 3D, then the
//   top surface), stated here independently of the RTL;
//   with --grab, against Supermodel's finished frame: counts of pixels equal
//   and within 9 a channel (its GL path converts 5-bit colour its own way,
//   and its grab has a black edge), reported, not failed.
//
//   Vm3_compose_tb --surf F [--layer F | --synth SEED] [--grab F] [--show 0|1]
//       [--buf K] [--busy PCT] [--lat LO HI] [--ppm OUT]
// --layer: 496 x 384 little-endian words {painted << 24 | RGB}; --grab: raw RGB.
#include "Vm3_compose_tb.h"
#include "verilated.h"
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>
#include <vector>

static constexpr int W = 496, H = 384;

static std::vector<uint8_t> rd(std::string const &fn) {
  std::vector<uint8_t> d;
  if (std::FILE *f = std::fopen(fn.c_str(), "rb")) {
    std::fseek(f, 0, SEEK_END); long n = std::ftell(f); std::fseek(f, 0, SEEK_SET);
    d.resize(size_t(n)); d.resize(std::fread(d.data(), 1, size_t(n), f)); std::fclose(f);
  }
  return d;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string surf_fn, layer_fn, grab_fn, ppm_fn;
  int synth = -1, show = 1, buf = 1, busy = 20, lat_lo = 8, lat_hi = 60;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return std::string(argv[++i]); };
    if (a == "--surf") surf_fn = nx();
    else if (a == "--layer") layer_fn = nx();
    else if (a == "--synth") synth = std::atoi(nx().c_str());
    else if (a == "--grab") grab_fn = nx();
    else if (a == "--show") show = std::atoi(nx().c_str());
    else if (a == "--buf") buf = std::atoi(nx().c_str());
    else if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--ppm") ppm_fn = nx();
  }
  std::vector<uint8_t> sf = rd(surf_fn);
  if (sf.size() != 2u * 512 * 384 * 4) { std::printf("no surfaces\n"); return 2; }
  auto surf = [&](int k, int x, int y) { uint8_t const *p = &sf[(size_t(k) * 512 * 384 + size_t(y) * 512 + size_t(x)) * 4]; return uint32_t(p[0]) << 16 | uint32_t(p[1]) << 8 | p[2] | uint32_t(p[3]) << 24; };
  std::vector<uint32_t> layer(size_t(W) * H, 0);
  std::mt19937 rng(unsigned(synth < 0 ? 1 : synth));
  if (!layer_fn.empty()) {
    std::vector<uint8_t> l = rd(layer_fn);
    if (l.size() != layer.size() * 4) { std::printf("bad layer\n"); return 2; }
    std::memcpy(layer.data(), l.data(), l.size());
  } else if (synth >= 0) {
    // blobs of painted pixels, random colours, so every case meets every other
    for (auto &p : layer) p = (rng() & 0xffffff) | ((rng() % 3 == 0) ? 0x1000000u : 0u);
  }
  std::vector<uint8_t> grab = grab_fn.empty() ? std::vector<uint8_t>() : rd(grab_fn);

  // DDR3: the layer in buffer `buf`, m2_fb_read's form
  std::vector<uint64_t> mem(0x100000, 0);
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      uint64_t &w = mem[size_t(buf) * 0x20000 + size_t(y) * 256 + size_t(x >> 1)];
      uint64_t const v = layer[size_t(y) * W + size_t(x)];
      w |= v << (32 * (x & 1));
    }
  // and garbage in the other buffers, painted, so a wrong buffer shows
  for (int k = 0; k < 4; k++) if (k != buf) for (size_t i = 0; i < 0x20000; i++) mem[size_t(k) * 0x20000 + i] = 0x01FF00FF01FF00FFull;

  auto *t = new Vm3_compose_tb;
  uint64_t cyc = 0;
  struct burst { uint64_t at, addr; uint32_t n; };
  std::deque<burst> rq;
  struct pend { uint64_t at; int x, y; };
  std::deque<pend> tq, cq;
  int frame = 0; unsigned prev_v = 0, late0 = 0;
  uint64_t checked = 0, bad = 0, g_eq = 0, g_near = 0, g_n = 0;
  std::vector<uint32_t> out(size_t(W) * H, 0);
  bool rdy = false; uint64_t dout = 0;
  t->rst_n = 0; t->ddr_ok = 0; t->show_buf = uint8_t(buf); t->show_3d = uint8_t(show);
  for (; frame < 4 && cyc < 40000000; cyc++) {
    if (cyc == 10) t->rst_n = 1;
    if (cyc == 2000) t->ddr_ok = 1;
    t->DDRAM_BUSY = int(rng() % 100) < busy;
    rdy = false;
    if (!rq.empty() && rq.front().at <= cyc && (rng() % 100) >= 10) {
      auto &b = rq.front(); rdy = true; dout = mem[b.addr & 0xFFFFF]; b.addr++;
      if (--b.n == 0) rq.pop_front();
    }
    t->DDRAM_DOUT_READY = rdy; t->DDRAM_DOUT = dout;
    // the tile generator: o_* three cycles after its read
    t->o_v = 0;
    if (!tq.empty() && tq.front().at == cyc) {
      pend const p = tq.front(); tq.pop_front();
      uint32_t const top = surf(1, p.x, p.y), bot = surf(0, p.x, p.y);
      uint32_t const c = (top >> 24) ? top : bot;
      t->o_v = 1; t->o_drawn = ((top | bot) >> 24) != 0; t->o_above = (top >> 24) != 0;
      t->o_r = uint8_t(c >> 16); t->o_g = uint8_t(c >> 8); t->o_b = uint8_t(c);
      cq.push_back({cyc + 1, p.x, p.y});
    }
    t->clk = 0; t->eval();
    if (t->rst_n && !t->DDRAM_BUSY && t->DDRAM_RD) {
      uint64_t at = cyc + uint64_t(lat_lo + int(rng() % unsigned(lat_hi - lat_lo + 1)));
      if (!rq.empty()) at = std::max(at, rq.back().at);
      rq.push_back({at, t->DDRAM_ADDR, t->DDRAM_BURSTCNT ? uint32_t(t->DDRAM_BURSTCNT) : 256u});
    }
    // the composed pixel, the cycle after o_v
    if (!cq.empty() && cq.front().at == cyc) {
      pend const p = cq.front(); cq.pop_front();
      if (frame >= 2) {
        uint32_t const top = surf(1, p.x, p.y), bot = surf(0, p.x, p.y), l = layer[size_t(p.y) * W + size_t(p.x)];
        // Supermodel: the bottom surface, then the 3D, then the top surface
        uint32_t exp = 0;
        if (bot >> 24) exp = bot & 0xffffff;
        if (show && (l >> 24)) exp = l & 0xffffff;
        if (top >> 24) exp = top & 0xffffff;
        uint32_t const got = t->px;
        checked++;
        if (got != exp) { if (bad < 5) std::printf("frame %d (%d, %d): %06X, expected %06X\n", frame, p.x, p.y, got, exp); bad++; }
        out[size_t(p.y) * W + size_t(p.x)] = got;
        if (!grab.empty()) {
          uint8_t const *g = &grab[(size_t(p.y) * W + size_t(p.x)) * 3];
          int const dr = std::abs(int(got >> 16 & 255) - g[0]), dg = std::abs(int(got >> 8 & 255) - g[1]), db = std::abs(int(got & 255) - g[2]);
          g_n++;
          if (!dr && !dg && !db) g_eq++;
          if (dr <= 9 && dg <= 9 && db <= 9) g_near++;
        }
      }
    }
    unsigned const sc = t->sc_rd, hx = t->hcnt, vy = t->vcnt;
    t->clk = 1; t->eval();
    if (sc) tq.push_back({cyc + 3, int(hx), int(vy)});
    if (t->vcnt == 0 && prev_v != 0) { frame++; if (frame == 2) late0 = t->n_late; }
    prev_v = t->vcnt;
  }
  std::printf("%" PRIu64 " pixels checked, %" PRIu64 " differ from Supermodel's order; %u lines late (%u in the warm-up)", checked, bad, unsigned(t->n_late) - late0, late0);
  if (g_n) std::printf("; against the grab: %" PRIu64 " equal (%.2f%%), %" PRIu64 " within 9 (%.2f%%)", g_eq, 100.0 * double(g_eq) / double(g_n), g_near, 100.0 * double(g_near) / double(g_n));
  std::printf("\n");
  if (!ppm_fn.empty()) if (std::FILE *f = std::fopen(ppm_fn.c_str(), "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (uint32_t v : out) { uint8_t c[3] = {uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)}; std::fwrite(c, 1, 3, f); }
    std::fclose(f);
  }
  bool const fail = bad || checked != 2u * W * H || t->n_late != late0;
  std::printf("%s\n", fail ? "FAIL" : "PASS");
  delete t;
  return fail ? 1 : 0;
}
