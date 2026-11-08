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
// r3d_rast_colour against render_d25: every Z-passing pixel, its discard, Z
// write, blend factor and colour after fog against the model's. --random:
// every kind, any texel, colour and transparency, the shade over the whole
// int32 range (MAME's wrap included), fog over its range, through d25_colour.
//
//   Vr3d_rast_colour --tris DIR --dumps DIR [--frames N] | --random N

#include "Vr3d_rast_colour.h"
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

struct cin { uint32_t kind, texel, color, trans, shade, fogint, fc, fs, fp, am; bool ok, zw; int fb; uint32_t rgb; };
static bool g_sm = true;   // --sm 0: MAME's kinds, for an RTL built with -GSMTX=0 (study R135)

struct hook : d25_hook {
  rframe const *f; std::vector<cin> *v;
  void px(d25_px const &p) override {
    if (!p.zpass) return;
    rtri const &t = f->tris[size_t(p.tri)];
    cin c{};
    c.kind = p.k; c.texel = p.texel; c.color = t.color & 0xffffff; c.trans = t.trans; c.shade = uint32_t(p.shade);
    c.am = d25_am(t);
    if (p.fog) { c.fogint = p.fogint; c.fc = uint32_t(p.fc[0] << 16 | p.fc[1] << 8 | p.fc[2]); c.fs = uint32_t(p.S) & 0x3ffff; c.fp = uint32_t(p.P); }
    c.ok = p.ok; c.zw = p.zw; c.fb = p.fb; c.rgb = p.col_fog & 0xffffff;
    // the model's own colour stage, standalone, must agree with the frame's
    if (p.ok) {
      uint32_t rgb; int fb; bool zw;
      d25_colour(p.k, p.texel, t.color, t.trans, p.shade, c.fogint, p.fc, p.S, p.P, rgb, fb, zw, g_sm, c.am);
      if (rgb != c.rgb || fb != c.fb) std::printf("  bench: d25_colour %06x %d against the frame's %06x %d\n", rgb, fb, c.rgb, c.fb);
    }
    v->push_back(c);
  }
};

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, nrand = 0;
  unsigned step = 0;   // --step HEX: the record's texture alpha from the polygon headers (study R135)
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--sm") g_sm = std::atoi(nx().c_str()) != 0;
    else if (a == "--step") step = unsigned(std::strtoul(nx().c_str(), nullptr, 16));
  }
  std::vector<cin> v;
  size_t nfr = 0;
  if (nrand) {
    std::mt19937 g(4);
    for (int i = 0; i < nrand; i++) {
      cin c{};
      c.kind = 1 + g() % 6; c.texel = g(); c.color = g() & 0xffffff; c.trans = g() % 33;
      if (g() % 8 == 0) c.texel = (c.texel & 0x00ffffff) | (0xf0u + g() % 16) << 24;
      // study R135: the alpha mode, and alphas about the contour test (32) and the 1/32 discard
      c.am = g() % 16;
      if (g() % 8 == 0) c.texel = (c.texel & 0x00ffffff) | (g() % 48) << 24;
      if (g() % 16 == 0) c.texel |= 0xff000000u;
      switch (g() % 4) {
      case 0: c.shade = g() % 300; break;
      case 1: c.shade = uint32_t(int32_t(g() % 200000) - 100000); break;
      case 2: c.shade = g(); break;
      default: c.shade = uint32_t(int32_t(g() >> (g() % 32)) * ((g() & 1) ? -1 : 1)); break;
      }
      if (g() & 1) { c.fogint = g() % 32; c.fc = g() & 0xffffff; c.fs = uint32_t(int32_t(g() % 6000) - 3000) & 0x3ffff; c.fp = g() % 4097; }
      int32_t fc[3] = {int32_t(c.fc >> 16 & 255), int32_t(c.fc >> 8 & 255), int32_t(c.fc & 255)};
      int32_t const S = int32_t(c.fs << 14) >> 14;
      c.ok = d25_colour(rkind(c.kind), c.texel, c.color, c.trans, int32_t(c.shade), c.fogint, fc, S, int32_t(c.fp), c.rgb, c.fb, c.zw, g_sm, c.am);
      v.push_back(c);
    }
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    nfr = names.size();
    texram tex; std::vector<uint32_t> dl(0x40000);
    tex.sm_tex = g_sm;
    for (auto const &nm : names) {
      rframe f;
      if (!read_rframe(tris + "/" + nm + ".tri", f)) continue;
      if (step) set_talpha(f, step);
      gzFile gz = gzopen((dumps + "/" + nm + ".bin.gz").c_str(), "rb");
      bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
                gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
      if (gz) gzclose(gz);
      if (!ok) continue;
      for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
      hook h; h.f = &f; h.v = &v; d.hook = &h;
      rimage img; d25_stats st;
      render_d25(f, tex, o, d, img, st);
    }
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_colour t;
  std::mt19937 rng(3);
  t.rst = 1; t.en = 1; t.i_valid = 0;
  for (int k = 0; k < 2; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
  t.rst = 0;
  uint64_t bad = 0, got = 0;
  size_t si = 0;
  while (got < v.size()) {
    bool const en = rng() % 5 != 0;
    t.en = en;
    bool const feed = si < v.size() && (rng() % 5 != 0);
    t.i_valid = feed;
    if (feed) {
      cin const &c = v[si];
      t.i_tag = uint32_t(si & 255); t.i_kind = c.kind; t.i_texel = c.texel; t.i_color = c.color; t.i_trans = c.trans; t.i_shade = c.shade;
      t.i_fogint = c.fogint; t.i_fc = c.fc; t.i_fs = c.fs; t.i_fp = c.fp; t.i_am = c.am;
    }
    t.clk = 0; t.eval();
    bool const out = t.en && t.o_valid;
    uint32_t const ot = t.o_tag, ok = t.o_ok, zw = t.o_zw, fbv = t.o_fbv, fb = t.o_fb, rgb = t.o_rgb;
    t.clk = 1; t.eval();
    if (en && feed) si++;
    if (!out) continue;
    cin const &c = v[got];
    bool good = ot == (got & 255) && bool(ok) == c.ok;
    if (c.ok) good = good && bool(zw) == c.zw && (fbv ? int(fb) : -1) == c.fb && rgb == c.rgb;
    if (!good && bad++ < 10)
      std::printf("  %" PRIu64 ": kind %u am %u texel %08x color %06x trans %u shade %08x fog %u/%06x/%u/%u: rtl ok %u zw %u fb %d rgb %06x, model ok %d zw %d fb %d rgb %06x\n",
                  got, c.kind, c.am, c.texel, c.color, c.trans, c.shade, c.fogint, c.fc, c.fs, c.fp, ok, zw, fbv ? int(fb) : -1, rgb, c.ok, c.zw, c.fb, c.rgb);
    got++;
  }
  std::printf("r3d_rast_colour: %zu frames, %zu pixels (%d random), %" PRIu64 " wrong\n", nfr, v.size(), nrand, bad);
  return bad ? 1 : 0;
}
