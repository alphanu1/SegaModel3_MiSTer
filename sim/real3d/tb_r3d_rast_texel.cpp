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
// r3d_rast_texel against render_d25: every textured Z-passing pixel's filtered
// texel against the model's, the texture RAM of the frame behind an ideal
// four-bank port (bank {A[11], A[0]}, word {A[21:12], A[10:1]}, one cycle,
// held while en is low). --random: random u, v, sheet positions, sizes,
// mirrors and formats over random texture RAM, against fetch_texel. Random
// stalls and gaps. --mip (a MIP = 1 build): render_d25 with mip = 1, lod = 1,
// each pixel at its level (d25_px::lv0); the random set at random levels up to
// the texture's highest, against fetch_texel_lv.
//
//   Vr3d_rast_texel --tris DIR --dumps DIR [--frames N] | --random N  [--mip]

#include "Vr3d_rast_texel.h"
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

struct tin { uint32_t u, v, page, tx, ty, tw, th, mu, mv, fmt, texel, lv, ctr; };
static tin mk(rtri const &t, uint32_t u, uint32_t v, uint32_t texel, uint32_t lv = 0) {
  return {u, v, t.page & 1, t.tex_x & 63, t.tex_y & 31, t.tex_w & 7, t.tex_h & 7, (t.param & TP_MIRROR_U) ? 1u : 0u,
          (t.param & TP_MIRROR_V) ? 1u : 0u, t.fmt & 7, texel, lv, (t.param & TP_CONTOUR) ? 1u : 0u};
}

struct hook : d25_hook {
  rframe const *f; std::vector<tin> *v;
  void px(d25_px const &p) override {
    if (!p.zpass || p.k < K_TEX) return;
    v->push_back(mk(f->tris[size_t(p.tri)], p.u, p.v, p.texel, uint32_t(p.lv0)));
  }
};

static uint64_t run(Vr3d_rast_texel &t, texram const &tex, std::vector<tin> const &v, std::mt19937 &rng) {
  uint64_t bad = 0, got = 0;
  size_t si = 0;
  uint16_t q[4] = {};
  while (got < v.size()) {
    bool const en = rng() % 5 != 0;
    bool const feed = si < v.size() && rng() % 6 != 0;
    t.en = en; t.i_valid = feed;
    if (feed) {
      tin const &c = v[si];
      t.i_tag = uint32_t(si & 255); t.i_u = c.u; t.i_v = c.v; t.i_page = c.page; t.i_tx = c.tx; t.i_ty = c.ty;
      t.i_tw = c.tw; t.i_th = c.th; t.i_mu = c.mu; t.i_mv = c.mv; t.i_fmt = c.fmt; t.i_lv = c.lv; t.i_ctr = c.ctr;
    }
    t.m_q0 = q[0]; t.m_q1 = q[1]; t.m_q2 = q[2]; t.m_q3 = q[3];
    t.clk = 0; t.eval();
    bool const out = en && t.o_valid;
    uint32_t const ot = t.o_tag, ox = t.o_texel;
    uint32_t const a[4] = {t.m_a0, t.m_a1, t.m_a2, t.m_a3};
    t.clk = 1; t.eval();
    if (en) {
      for (int b = 0; b < 4; b++) {
        uint32_t const w = ((a[b] >> 10) << 12) | (uint32_t(b >> 1) << 11) | ((a[b] & 1023) << 1) | uint32_t(b & 1);
        q[b] = tex.t[w];
      }
      if (feed) si++;
    }
    if (!out) continue;
    tin const &c = v[got];
    if ((ot != (got & 255) || ox != c.texel) && bad++ < 10)
      std::printf("  %" PRIu64 ": u %08x v %08x page %u tx %u ty %u w %u h %u mir %u%u fmt %u ctr %u lv %u: rtl %08x model %08x\n",
                  got, c.u, c.v, c.page, c.tx, c.ty, c.tw, c.th, c.mu, c.mv, c.fmt, c.ctr, c.lv, ox, c.texel);
    got++;
  }
  return bad;
}

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, nrand = 0;
  bool mip = false, sm = true;
  int rkey = 0;   // --rkey N: random texture RAM with one word in N of all-ones nibbles / bytes (study R135)
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--mip") mip = true;
    else if (a == "--sm") sm = std::atoi(nx().c_str()) != 0;
    else if (a == "--rkey") rkey = std::atoi(nx().c_str());
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_texel t;
  std::mt19937 rng(3);
  t.rst = 1; t.en = 1; t.i_valid = 0;
  for (int k = 0; k < 2; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
  t.rst = 0;
  uint64_t bad = 0, n = 0;
  size_t nfr = 0;
  texram tex;
  tex.sm_tex = sm;
  if (nrand) {
    std::mt19937 g(6);
    for (auto &w : tex.t) w = uint16_t(g());
    // keys (luminance 15, L8 0xff, bit 15) dense enough that most contour quads have both kinds of corner
    if (rkey) for (auto &w : tex.t) if (g() % uint32_t(rkey) == 0) w = uint16_t(w | (0xfu << (4 * (g() % 4))) | ((g() & 1) ? 0x00ffu : 0) | ((g() & 1) ? 0xff00u : 0));
    std::vector<tin> v;
    for (int i = 0; i < nrand; i++) {
      rtri r{};
      r.page = g() & 1; r.tex_x = g() & 63; r.tex_y = g() & 31; r.tex_w = g() % 8; r.tex_h = g() % 8; r.fmt = g() & 7;
      r.param = TP_TEXTURE | ((g() & 1) ? TP_MIRROR_U : 0) | ((g() & 1) ? TP_MIRROR_V : 0) | ((g() & 1) ? TP_CONTOUR : 0);
      uint32_t const u = (g() % 4) ? (g() & 0x3fffff) : g(), vv = (g() % 4) ? (g() & 0x3fffff) : g();
      bool outside;
      if (mip) {
        // what the geometry sends: size codes up to 5 (it drops 6 and 7), so levels up to 9
        r.tex_w %= 6; r.tex_h %= 6;
        uint32_t const lv = g() % uint32_t(std::min(mip_levels(r), 9) + 1);
        v.push_back(mk(r, u, vv, fetch_texel_lv(r, tex, u, vv, int(lv)), lv));
      } else v.push_back(mk(r, u, vv, fetch_texel(r, tex, u, vv, outside)));
    }
    bad += run(t, tex, v, rng); n += v.size();
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    nfr = names.size();
    std::vector<uint32_t> dl(0x40000);
    for (auto const &nm : names) {
      rframe f;
      if (!read_rframe(tris + "/" + nm + ".tri", f)) continue;
      gzFile gz = gzopen((dumps + "/" + nm + ".bin.gz").c_str(), "rb");
      bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
                gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
      if (gz) gzclose(gz);
      if (!ok) { bad++; continue; }
      for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      std::vector<tin> v;
      rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
      if (mip) { d.mip = 1; d.lod = 1; }
      hook h; h.f = &f; h.v = &v; d.hook = &h;
      rimage img; d25_stats st;
      render_d25(f, tex, o, d, img, st);
      uint64_t const b = run(t, tex, v, rng);
      if (b) std::printf("  %s: %" PRIu64 " wrong\n", nm.c_str(), b);
      bad += b; n += v.size();
    }
  }
  std::printf("r3d_rast_texel%s: %zu frames, %" PRIu64 " texels (%d random), %" PRIu64 " wrong\n", mip ? " (mip 1, lod 1)" : "", nfr, n, nrand, bad);
  return bad ? 1 : 0;
}
