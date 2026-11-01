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
// r3d_rast_tcache: the four bank words of every textured Z-passing pixel of
// render_d25, in the RTL's order (8-row bands, passes, draw order), against
// the texture RAM, and the pixels' order. Behind it a memory with DDR3-like
// behaviour: requests taken with random gaps, each line four 64-bit beats
// after a random latency, in order, with gaps between beats. --random: quads
// clustered so lines collide in sets, and scattered. Cycles a frame reported.
// Every fill address is also checked, in order, against a model of the
// allocation rule (a pixel's distinct missing lines lowest bank first, the
// victim a global round-robin way never one the pixel uses): a duplicate or a
// different victim shows as a wrong fill even when every texel is right.
//
//   Vr3d_rast_tcache --tris DIR --dumps DIR [--frames N] | --random N
//   [--lat MIN MAX] [--gap PCT] [--stall PCT] [--mip]
//
// --mip: render_d25 with mip = 1, lod = 1, each pixel's quad at its level
// (texel_addr4 at d25_px::lv0, the addresses r3d_rast_texel MIP = 1 forms).

#include "Vr3d_rast_tcache.h"
#include "verilated.h"
#include "r3d_rast.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <zlib.h>

using namespace r3d;
namespace fs = std::filesystem;

struct req { uint32_t a[4]; uint32_t use; };   // bank in-word addresses

static uint32_t mir(uint32_t c, uint32_t p) { return c < p ? c : 2 * p - 1 - c; }
static req mkreq(uint32_t const A[4]) {
  req r{}; 
  for (int b = 0; b < 4; b++)
    for (int k = 0; k < 4; k++) {
      uint32_t const bk = ((A[k] >> 11) & 1) << 1 | (A[k] & 1);
      if (bk == uint32_t(b) && !(r.use >> b & 1)) { r.use |= 1u << b; r.a[b] = ((A[k] >> 12) << 10) | ((A[k] >> 1) & 1023); }
    }
  return r;
}
static req quad(rtri const &t, uint32_t u, uint32_t v) {
  uint32_t const um = (((t.param & TP_MIRROR_U) ? 64u : 32u) << t.tex_w) - 1, vm = (((t.param & TP_MIRROR_V) ? 64u : 32u) << t.tex_h) - 1;
  uint32_t const pw = 32u << t.tex_w, ph = 32u << t.tex_h;
  uint32_t const u1 = (u >> 8) & um, v1 = (v >> 8) & vm, u2 = (u1 + 1) & um, v2 = (v1 + 1) & vm;
  uint32_t const c1 = t.tex_x * 32 + mir(u1, pw), c2 = t.tex_x * 32 + mir(u2, pw);
  uint32_t const r1 = (t.page & 1) * 1024 + t.tex_y * 32 + mir(v1, ph), r2 = (t.page & 1) * 1024 + t.tex_y * 32 + mir(v2, ph);
  uint32_t const A[4] = {((r1 << 11) + c1) & 0x3fffff, ((r1 << 11) + c2) & 0x3fffff, ((r2 << 11) + c1) & 0x3fffff, ((r2 << 11) + c2) & 0x3fffff};
  return mkreq(A);
}
static uint32_t word(uint32_t b, uint32_t a) { return ((a >> 10) << 12) | ((b >> 1) << 11) | ((a & 1023) << 1) | (b & 1); }

static bool g_mip = false;
struct hook : d25_hook {
  rframe const *f; std::vector<req> bucket[4][48];
  void px(d25_px const &p) override {
    if (!p.zpass || p.k < K_TEX) return;
    rtri const &t = f->tris[size_t(p.tri)];
    if (g_mip) { uint32_t A[4]; texel_addr4(t, p.u, p.v, p.lv0, A); bucket[t.pass][p.y / 8].push_back(mkreq(A)); }
    else bucket[t.pass][p.y / 8].push_back(quad(t, p.u, p.v));
  }
};

int lat0 = 20, lat1 = 60, gap = 10, stall = 10;

#ifndef TC_SB
#define TC_SB 8
#endif
#ifndef TC_WAYS
#define TC_WAYS 4
#endif
// The RTL's allocation rule, line for line
struct cmodel {
  static constexpr int SB = TC_SB, W = TC_WAYS, XB = (SB + 1) / 2, YB = SB / 2;
  std::vector<uint32_t> ln = std::vector<uint32_t>(size_t(W) << SB, 0);
  std::vector<uint8_t> val = std::vector<uint8_t>(size_t(W) << SB, 0);
  uint32_t rr = 0;
  static uint32_t line(uint32_t a) { return ((a >> 11) << 9) | ((a >> 1) & 511); }
  static uint32_t set(uint32_t l) { return (((l >> 9) & ((1u << YB) - 1)) << XB) | (l & ((1u << XB) - 1)); }
  void inval() { std::fill(val.begin(), val.end(), 0); }
  void pixel(req const &q, std::vector<uint32_t> &out) {
    bool ok[4], hit[4]; uint32_t hw[4] = {}, wy[4] = {}, l[4], st[4];
    for (int b = 0; b < 4; b++) {
      ok[b] = !(q.use >> b & 1); hit[b] = false;
      l[b] = line(q.a[b]); st[b] = set(l[b]);
      if (ok[b]) continue;
      for (uint32_t w = 0; w < W; w++)
        if (val[st[b] * W + w] && ln[st[b] * W + w] == l[b]) { hit[b] = true; hw[b] = w; }
    }
    for (;;) {
      int mb = -1;
      for (int b = 3; b >= 0; b--) if ((q.use >> b & 1) && !ok[b] && !hit[b]) mb = b;
      if (mb < 0) break;
      uint32_t busy = 0;
      for (int b = 0; b < 4; b++)
        if ((q.use >> b & 1) && (ok[b] || hit[b]) && st[b] == st[mb]) busy |= 1u << (ok[b] ? wy[b] : hw[b]);
      uint32_t vic = rr;
      for (int k = W - 1; k >= 0; k--) if (!(busy >> ((rr + uint32_t(k)) % W) & 1)) vic = (rr + uint32_t(k)) % W;
      out.push_back(l[mb]);
      ln[st[mb] * W + vic] = l[mb]; val[st[mb] * W + vic] = 1;
      rr = (vic + 1) % W;
      uint32_t const lm = l[mb];
      for (int b = 0; b < 4; b++) if ((q.use >> b & 1) && !ok[b] && l[b] == lm) { ok[b] = true; wy[b] = vic; }
    }
  }
};
cmodel cm;

static uint64_t run(Vr3d_rast_tcache &t, texram const &tex, std::vector<req> const &v, std::mt19937 &rng, uint64_t &cyc, uint64_t &fills) {
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  struct inflight { uint64_t due; uint32_t line; };
  std::deque<inflight> mq;
  int beat = 0;
  size_t si = 0, oi = 0;
  uint64_t bad = 0, idle = 0;
  std::vector<uint32_t> want, got;
  for (auto const &q : v) cm.pixel(q, want);
  while (oi < v.size() && idle < 100000) {
    bool const offer = si < v.size() && !pct(gap);
    if (offer) { req const &q = v[si]; t.i_a0 = q.a[0]; t.i_a1 = q.a[1]; t.i_a2 = q.a[2]; t.i_a3 = q.a[3]; t.i_use = q.use; t.i_pay = uint32_t(si); }
    t.i_valid = offer; t.o_ready = !pct(stall); t.inval = 0;
    t.f_ack = !pct(30);
    bool const rv = !mq.empty() && mq.front().due <= cyc && !pct(15);
    t.f_rvalid = rv;
    if (rv) {
      uint32_t const ln = mq.front().line, yt = ln >> 9, xt = ln & 511;
      uint64_t d = 0;
      for (int x = 0; x < 4; x++) d |= uint64_t(tex.t[(yt * 4 + uint32_t(beat)) * 2048 + xt * 4 + uint32_t(x)]) << (16 * x);
      t.f_rdata = d;
    }
    t.clk = 0; t.eval();
    bool const took = t.i_valid && t.i_ready, out = t.o_valid && t.o_ready, fr = t.f_req && t.f_ack;
    uint32_t const fa = t.f_addr, op = t.o_pay, oq[4] = {t.o_q0, t.o_q1, t.o_q2, t.o_q3};
    t.clk = 1; t.eval(); cyc++;
    if (took) si++;
    if (fr) { mq.push_back({cyc + uint64_t(lat0 + int(rng() % uint32_t(lat1 - lat0 + 1))), fa}); fills++; got.push_back(fa); }
    if (rv && ++beat == 4) { beat = 0; mq.pop_front(); }
    if (!out) { idle++; continue; }
    idle = 0;
    req const &q = v[oi];
    bool ok = op == uint32_t(oi);
    for (int b = 0; b < 4; b++) if (q.use >> b & 1) ok = ok && oq[b] == tex.t[word(uint32_t(b), q.a[b])];
    if (!ok && bad++ < 10) std::printf("  pixel %zu: pay %u, words %04x %04x %04x %04x\n", oi, op, oq[0], oq[1], oq[2], oq[3]);
    oi++;
  }
  if (oi < v.size()) { bad += v.size() - oi; std::printf("  stuck: %zu of %zu out\n", oi, v.size()); }
  size_t fb = 0;
  for (size_t k = 0; k < std::max(want.size(), got.size()); k++)
    if (k >= want.size() || k >= got.size() || want[k] != got[k]) {
      if (fb++ < 4) std::printf("  fill %zu: line %05x, the rule's %05x\n", k, k < got.size() ? got[k] : 0u, k < want.size() ? want[k] : 0u);
    }
  if (fb) std::printf("  fills: %zu issued, %zu by the rule, %zu differ\n", got.size(), want.size(), fb);
  return bad + fb;
}

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, nrand = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--lat") { lat0 = std::atoi(nx().c_str()); lat1 = std::atoi(nx().c_str()); }
    else if (a == "--gap") gap = std::atoi(nx().c_str()); else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--mip") g_mip = true;
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_tcache t;
  std::mt19937 rng(5);
  t.rst = 1; t.i_valid = 0; t.o_ready = 0; t.f_rvalid = 0; t.f_ack = 0; t.inval = 0;
  for (int k = 0; k < 2; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
  t.rst = 0;
  uint64_t bad = 0, n = 0, cyc = 0, fills = 0;
  std::vector<uint64_t> fc;
  size_t nfr = 0;
  texram tex;
  if (nrand) {
    std::mt19937 g(8);
    for (auto &w : tex.t) w = uint16_t(g());
    std::vector<req> v;
    uint32_t base = 0;
    for (int i = 0; i < nrand; i++) {
      if (g() % 64 == 0) base = g() & 0x3fffff;
      uint32_t a0 = (g() % 3) ? (base + (g() % 64) + ((g() % 64) << 11)) & 0x3fffff : g() & 0x3fffff;
      if (g() % 8 == 0) a0 = (base + ((g() % 4) << (11 + 4 * 1 + 2)) + ((g() % 4) << 6)) & 0x3fffff;   // same set, other tags
      uint32_t const du = (g() % 8) ? 1 : 0, dv = (g() % 8) ? 2048 : 0;
      uint32_t A[4] = {a0, (a0 + du) & 0x3fffff, (a0 + dv) & 0x3fffff, (a0 + du + dv) & 0x3fffff};
      if (g() % 8 == 0) {
        // corners in one set with other tags (16 tiles apart in x and y): the pixel's own ways must survive
        uint32_t const s0 = a0 & ~uint32_t(0x801);
        A[0] = s0; A[1] = (s0 + 1 + 64 * (1 + g() % 3)) & 0x3fffff;
        A[2] = (s0 + 2048 + (64u << 11) * (1 + g() % 3)) & 0x3fffff; A[3] = (A[2] + 1 + 64 * (1 + g() % 3)) & 0x3fffff;
      }
      v.push_back(mkreq(A));
    }
    // a texture upload between halves: invalidate, then the same addresses over new texels
    bad += run(t, tex, v, rng, cyc, fills); n += v.size();
    for (auto &w : tex.t) w = uint16_t(g());
    cm.inval();
    t.inval = 1; t.clk = 0; t.eval(); t.clk = 1; t.eval(); t.inval = 0;
    for (int k = 0; k < 300; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
    bad += run(t, tex, v, rng, cyc, fills); n += v.size();
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
      rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
      if (g_mip) { d.mip = 1; d.lod = 1; }
      hook h; h.f = &f; d.hook = &h;
      rimage img; d25_stats st;
      render_d25(f, tex, o, d, img, st);
      std::vector<req> v;
      for (int b = 0; b < 48; b++) for (int p = 0; p < 4; p++) v.insert(v.end(), h.bucket[p][b].begin(), h.bucket[p][b].end());
      // each frame after a texture upload
      cm.inval();
      t.inval = 1; t.clk = 0; t.eval(); t.clk = 1; t.eval(); t.inval = 0;
      for (int k = 0; k < 300; k++) { t.clk = 0; t.eval(); t.clk = 1; t.eval(); }
      uint64_t const c0 = cyc;
      uint64_t const b = run(t, tex, v, rng, cyc, fills);
      fc.push_back(cyc - c0);
      if (b) std::printf("  %s: %" PRIu64 " wrong\n", nm.c_str(), b);
      bad += b; n += v.size();
    }
  }
  std::sort(fc.begin(), fc.end());
  if (!fc.empty())
    std::printf("  cycles a frame p50 / p99: %" PRIu64 " / %" PRIu64 " (%.1f%% of a 100 MHz frame at p99)\n", fc[fc.size() / 2],
                fc[std::min(fc.size() - 1, fc.size() * 99 / 100)], 100.0 * double(fc[std::min(fc.size() - 1, fc.size() * 99 / 100)]) / 1738530.0);
  std::printf("r3d_rast_tcache: %zu frames, %" PRIu64 " pixels (%d random), %" PRIu64 " line fills, %" PRIu64 " wrong; %.3f cycles a pixel (latency %d-%d, gap %d%%, stall %d%%)\n",
              nfr, n, nrand, fills, bad, double(cyc) / double(std::max<uint64_t>(n, 1)), lat0, lat1, gap, stall);
  return bad ? 1 : 0;
}
