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
// r3d_rast_band: every pixel render_d25 writes (translucency 0, D-29), its
// colour after fog and its blend factor, in the RTL's order (8-row bands, the
// passes in turn, draw order), with an end-of-band mark after each band.
// Behind it a memory with m2_ddr3's contract (a request held to its ack, a
// beat taken when not busy) and random BUSY. The layer it writes is compared
// with render_d25's image on every pixel. --random: pixels piled on a few
// words of each band, half of them blending, with a random scroll-fog
// rectangle, against the same rules in C++.
//
//   Vr3d_rast_band --tris DIR --dumps DIR [--frames N] | --random N  [--gap PCT] [--busy PCT]

#include "Vr3d_rast_band.h"
#include "Vr3d_rast_band___024root.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <filesystem>
#include <random>
#include <string>
#include <vector>
#include <zlib.h>

using namespace r3d;
namespace fs = std::filesystem;

struct pix { uint16_t x, y; uint32_t rgb; bool bl; uint8_t f; };

struct hook : d25_hook {
  rframe const *f; std::vector<pix> bucket[4][48];
  void px(d25_px const &p) override {
    if (!p.ok || p.layer) return;
    rtri const &t = f->tris[size_t(p.tri)];
    bucket[t.pass][p.y / 8].push_back({uint16_t(p.x), uint16_t(p.y), p.col_fog & 0xffffff, p.fb >= 0, uint8_t(p.fb & 255)});
  }
};

int gap = 10, busy = 30;
uint64_t haz_cyc = 0;   // cycles a blend waited for a write in flight
std::mt19937 rng(7);
static bool pct(int p) { return int(rng() % 100) < p; }

struct cfg { int x0 = 0, x1 = 0, y0 = 0, y1 = 0; uint32_t col = 0, shown = 0; };
static constexpr uint32_t BASE = 0x10000, STRIDE = 256;

// one frame through the block: the layer it wrote (0x01000000 | rgb when drawn)
static void run(Vr3d_rast_band &t, cfg const &c, std::vector<std::vector<pix>> const &bands, std::vector<uint32_t> &layer,
                uint64_t &cyc, uint64_t &eob_wait) {
  layer.assign(size_t(SCR_W) * SCR_H, 0xdeadbeefu);
  t.fb_base = BASE; t.sf_x0 = uint32_t(c.x0); t.sf_x1 = uint32_t(c.x1); t.sf_y0 = uint32_t(c.y0); t.sf_y1 = uint32_t(c.y1);
  t.sf_col = c.col; t.sf_out = c.shown;
  auto tick = [&]() { t.clk = 0; t.eval(); t.clk = 1; t.eval(); cyc++; };
  // the memory
  enum { M_IDLE, M_ISSUE } ms = M_IDLE;
  uint32_t ma = 0, beat = 0, blen = 0; bool ack = false;
  auto mem = [&]() {
    bool const wacc = ms == M_ISSUE && !pct(busy);
    t.m_wacc = wacc; t.m_ack = ack;
    t.clk = 0; t.eval();
    bool ack_n = false;
    if (ms == M_ISSUE && wacc) {
      uint32_t const y = (ma - BASE) / STRIDE, x = 2 * beat;
      uint64_t const d = t.m_din;
      if (y < uint32_t(SCR_H) && x + 1 < uint32_t(SCR_W) + 1) {
        if (x < uint32_t(SCR_W)) layer[size_t(y) * SCR_W + x] = uint32_t(d) & 0x01ffffffu;
        if (x + 1 < uint32_t(SCR_W)) layer[size_t(y) * SCR_W + x + 1] = uint32_t(d >> 32) & 0x01ffffffu;
      }
      if (++beat == blen) { ack_n = true; ms = M_IDLE; }
    } else if (ms == M_IDLE && t.m_req && !ack) { ma = t.m_addr; blen = t.m_blen; beat = 0; ms = M_ISSUE; }
    ack = ack_n;
  };
  auto step = [&]() { mem(); t.clk = 1; t.eval(); cyc++; };
  // the frame's clear
  t.p_valid = 0; t.f_start = 1; mem(); t.clk = 1; t.eval(); cyc++; t.f_start = 0;
  for (int k = 0; k < 4; k++) step();
  for (int b = 0; b < 48; b++) {
    std::vector<pix> const &v = bands[size_t(b)];
    for (size_t i = 0; i <= v.size();) {
      bool const offer = !pct(gap);
      t.p_valid = offer;
      t.p_eob = i == v.size();
      t.p_band = uint32_t(b);
      if (i < v.size()) { t.p_x = v[i].x; t.p_row = v[i].y & 7; t.p_rgb = v[i].rgb; t.p_bl = v[i].bl; t.p_f = v[i].f; }
      mem();
      haz_cyc += t.rootp->r3d_rast_band__DOT__haz_v;
      bool const took = t.p_valid && t.p_ready;
      if (offer && !t.p_ready && i == v.size()) eob_wait++;
      t.clk = 1; t.eval(); cyc++;
      if (took) i++;
    }
  }
  t.p_valid = 0;
  for (int k = 0; k < 200000; k++) { step(); if (!t.busy && !t.m_req && ms == M_IDLE) break; }
  (void)tick;
}

// the rules in C++: the clear, each pixel (MAME's blend), the undrawn shown
static void model(cfg const &c, std::vector<std::vector<pix>> const &bands, std::vector<uint32_t> &img) {
  img.assign(size_t(SCR_W) * SCR_H, 0);
  for (int y = c.y0; y < c.y1; y++) for (int x = c.x0; x < c.x1; x++) img[size_t(y) * SCR_W + size_t(x)] = c.col;
  for (auto const &v : bands)
    for (auto const &p : v) {
      uint32_t &d = img[size_t(p.y) * SCR_W + p.x];
      uint32_t o = p.rgb;
      if (p.bl) {
        o = 0;
        for (int sh = 0; sh < 24; sh += 8) {
          uint32_t const cc = (p.rgb >> sh) & 255, dd = (d >> sh) & 255;
          o |= (((cc * p.f + dd * (256 - p.f)) >> 8) & 255) << sh;
        }
      }
      d = 0x01000000u | o;
    }
  for (auto &p : img) if (!(p >> 24)) p = (p & 0xffffff) ? (0x01000000u | c.shown) : 0;
}

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, nrand = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--gap") gap = std::atoi(nx().c_str()); else if (a == "--busy") busy = std::atoi(nx().c_str());
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_band t;
  t.rst = 1; t.p_valid = 0; t.f_start = 0; t.m_wacc = 0; t.m_ack = 0;
  for (int k = 0; k < 4; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
  t.rst = 0;
  uint64_t bad = 0, npx = 0, nfr = 0, cyc = 0, eobw = 0, nbl = 0;
  std::vector<uint64_t> fc;
  auto check = [&](std::vector<uint32_t> const &layer, std::vector<uint32_t> const &want, char const *nm) {
    uint64_t b = 0;
    for (size_t i = 0; i < want.size(); i++)
      if (layer[i] != want[i]) { if (b++ < 4) std::printf("  %s (%zu,%zu): %08x, want %08x\n", nm, i % SCR_W, i / SCR_W, layer[i], want[i]); }
    if (b) std::printf("  %s: %" PRIu64 " pixels wrong\n", nm, b);
    bad += b;
  };
  if (nrand) {
    std::mt19937 g(11);
    for (int fr = 0; fr < nrand; fr++) {
      cfg c;
      if (g() % 4) {
        c.x0 = int(g() % 496); c.x1 = c.x0 + int(g() % uint32_t(497 - c.x0));
        c.y0 = int(g() % 384); c.y1 = c.y0 + int(g() % uint32_t(385 - c.y0));
        c.col = g() & 0xffffff; if (g() % 8 == 0) c.col = 0;
        c.shown = g() & 0xffffff;
      }
      std::vector<std::vector<pix>> bands(48);
      for (int b = 0; b < 48; b++) {
        int const n = (g() % 4 == 0) ? 0 : int(g() % 400);
        int const wx = int(g() % 490), wr = int(g() % 8);
        for (int k = 0; k < n; k++) {
          pix p;
          if (g() % 3) { p.x = uint16_t(wx + int(g() % 6)); p.y = uint16_t(b * 8 + (wr + int(g() % 2)) % 8); }   // piled up
          else { p.x = uint16_t(g() % 496); p.y = uint16_t(b * 8 + int(g() % 8)); }
          p.rgb = g() & 0xffffff; p.bl = g() % 2; p.f = uint8_t(g());
          nbl += p.bl;
          bands[size_t(b)].push_back(p); npx++;
        }
      }
      std::vector<uint32_t> layer, want;
      run(t, c, bands, layer, cyc, eobw);
      model(c, bands, want);
      check(layer, want, "random");
    }
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    texram tex; std::vector<uint32_t> dl(0x40000);
    for (auto const &nm : names) {
      rframe f;
      if (!read_rframe(tris + "/" + nm + ".tri", f)) continue;
      gzFile gz = gzopen((dumps + "/" + nm + ".bin.gz").c_str(), "rb");
      bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
                gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
      if (gz) gzclose(gz);
      if (!ok) { bad++; continue; }
      for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
      hook h; h.f = &f; d.hook = &h;
      rimage img; d25_stats st;
      render_d25(f, tex, o, d, img, st);
      cfg c; d25_sfog sg;
      if (d25_scroll_fog(f, sg)) { c.x0 = sg.x0; c.x1 = sg.x1; c.y0 = sg.y0; c.y1 = sg.y1; c.col = sg.col; c.shown = sg.shown; }
      std::vector<std::vector<pix>> bands(48);
      for (int b = 0; b < 48; b++) for (int p = 0; p < 4; p++) {
        bands[size_t(b)].insert(bands[size_t(b)].end(), h.bucket[p][b].begin(), h.bucket[p][b].end());
        for (auto const &q : h.bucket[p][b]) { npx++; nbl += q.bl; }
      }
      std::vector<uint32_t> layer, want(img.px.size());
      for (size_t i = 0; i < img.px.size(); i++) want[i] = (img.px[i] >> 24) ? (0x01000000u | (img.px[i] & 0xffffff)) : 0;
      uint64_t const c0 = cyc;
      run(t, c, bands, layer, cyc, eobw);
      fc.push_back(cyc - c0);
      check(layer, want, nm.c_str());
      // render_d25's colour rules again in C++: the bench's own reading of them
      std::vector<uint32_t> m;
      model(c, bands, m);
      check(m, want, "model");
      nfr++;
    }
  }
  std::sort(fc.begin(), fc.end());
  if (!fc.empty()) std::printf("  cycles a frame p50 / p99: %" PRIu64 " / %" PRIu64 "\n", fc[fc.size() / 2], fc[std::min(fc.size() - 1, fc.size() * 99 / 100)]);
  std::printf("r3d_rast_band: %" PRIu64 " frames, %" PRIu64 " pixels (%" PRIu64 " blended, %d random frames), %" PRIu64 " wrong; "
              "%" PRIu64 " cycles, end-of-band waits %" PRIu64 ", blend waits %" PRIu64 " (gap %d%%, busy %d%%)\n",
              nfr, npx, nbl, nrand, bad, cyc, eobw, haz_cyc, gap, busy);
  return bad ? 1 : 0;
}
