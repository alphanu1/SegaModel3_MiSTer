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
// r3d_rast_interp against render_d25: every Z-passing pixel of the frames
// (depth on 1/w), its shade, u and v against the model's, and r, n against
// the reciprocal of the 1/w plane value. --random: planes at make_plane's
// bounds, any exponent, 1/w of either sign or zero, through rtl_tri::eval.
// Random input gaps and output stalls. --mip (a MIP = 1 build): render_d25
// with mip = 1, lod = 1, and each textured pixel's level o_lv against the
// model's (d25_px::lv0), the triangle's i_lk = lb8 + 65152 (0 for no B) and
// i_lvm its highest level; the random set with random terms against render_d25's
// formula on the pixel's Z word.
//
//   Vr3d_rast_interp --tris DIR --dumps DIR [--frames N] | --random N  [--gap PCT] [--stall PCT] [--mip]

#include "Vr3d_rast_interp.h"
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
using i128 = __int128;

struct pin { int x, y; rtl_tri::plane o, s, u, v; bool tex; uint32_t eu, ev, es; int64_t o2; uint32_t lk = 0, lvm = 0; };
struct pexp { uint32_t u, v, shade, r; int n; bool pos; int lv = 0; };
static bool g_mip = false;
// the triangle's LOD constant: lb8 + 65152 (render_d25's +128 rounding and 2 x 127 x 256 folded in), 0 for none
static uint32_t lk_of(uint32_t bw) { int32_t const lb = d25_lod_lb8(bw); return lb == INT32_MIN ? 0u : uint32_t(lb + 65152); }
// render_d25's nearest level (lod = 1) from the Z word and lb8
static int lv_of(uint32_t z, uint32_t bw, int levels) {
  int32_t const lb = d25_lod_lb8(bw);
  int32_t const lz = (int32_t(z >> 16) - 127) * 256 + int32_t(d25_log2t(int((z >> 10) & 63)));
  int32_t l8 = lb == INT32_MIN ? 0 : lb - 2 * lz;
  l8 = std::clamp(l8, 0, levels * 256);
  return ((l8 + 128) >> 8);
}

static i128 at2(rtl_tri::plane const &p, int x, int y) { return i128(p.c) * 2 + i128(p.dx) * (2 * x + 1) + i128(p.dy) * (2 * y + 1); }
static int msb128(unsigned __int128 v) { int n = -1; while (v) { v >>= 1; n++; } return n; }

static void expect_r(i128 o2, pexp &e) {
  e.pos = o2 > 0; e.r = 0; e.n = 0;
  if (o2 == 0) return;
  unsigned __int128 const a = (unsigned __int128)(o2 < 0 ? -o2 : o2);
  int const n = msb128(a);
  uint64_t const m = uint64_t(n >= 23 ? (a >> (n - 23)) : (a << (23 - n)));
  e.n = n; e.r = uint32_t(((uint64_t(1) << 47) + m / 2) / m);
}

struct hook : d25_hook {
  std::vector<rtl_tri> tr;
  std::vector<pin> *in; std::vector<pexp> *ex;
  void tri(int32_t idx, int, rtl_tri const &r, rkind, bool) override { tr[size_t(idx)] = r; }
  void px(d25_px const &p) override {
    if (!p.zpass) return;
    rtl_tri const &r = tr[size_t(p.tri)];
    pin q{};
    q.x = p.x; q.y = p.y; q.tex = r.tex; q.o = r.pl[0];
    if (r.tex) { q.u = r.pl[2]; q.v = r.pl[3]; q.s = r.pl[4]; } else q.s = r.pl[1];
    q.lk = lk_of(p.lodb); q.lvm = uint32_t(p.lvmax);
    i128 const o2 = at2(r.pl[0], p.x, p.y);
    q.o2 = int64_t(o2);
    pexp e{};
    e.u = r.tex ? p.u : 0; e.v = r.tex ? p.v : 0; e.shade = uint32_t(p.shade);
    expect_r(o2, e);
    e.lv = p.lv0;
    in->push_back(q); ex->push_back(e);
  }
};

int main(int argc, char **argv) {
  std::string tris, dumps;
  int frames = 1 << 30, gap = 20, stall = 20, nrand = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--dumps") dumps = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--gap") gap = std::atoi(nx().c_str());
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--random") nrand = std::atoi(nx().c_str());
    else if (a == "--mip") g_mip = true;
  }
  Verilated::commandArgs(argc, argv);
  Vr3d_rast_interp top;
  std::mt19937 rng(1);
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  top.rst = 1; top.i_valid = 0; top.o_ready = 0;
  for (int k = 0; k < 2; k++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;
  uint64_t bad = 0, npx = 0, cyc = 0;
  uint64_t const M40 = (uint64_t(1) << 40) - 1, M42 = (uint64_t(1) << 42) - 1;
  auto run = [&](std::vector<pin> const &in, std::vector<pexp> const &ex) {
    size_t si = 0, oi = 0; uint64_t idle = 0;
    while (oi < ex.size() && idle < 10000) {
      bool const offer = si < in.size() && !pct(gap);
      if (offer) {
        pin const &q = in[si];
        top.i_tag = uint32_t(si & 255); top.i_x = uint32_t(q.x); top.i_y = uint32_t(q.y);
        top.i_o2 = uint64_t(q.o2) & M42; top.i_eo = uint32_t(q.o.e) & 0x1ff; top.i_tex = q.tex;
        top.i_sc = uint64_t(q.s.c) & M40; top.i_sdx = uint64_t(q.s.dx) & M40; top.i_sdy = uint64_t(q.s.dy) & M40; top.i_se = uint32_t(q.s.e) & 0x1ff;
        top.i_uc = uint64_t(q.u.c) & M40; top.i_udx = uint64_t(q.u.dx) & M40; top.i_udy = uint64_t(q.u.dy) & M40; top.i_ue = uint32_t(q.u.e) & 0x1ff;
        top.i_vc = uint64_t(q.v.c) & M40; top.i_vdx = uint64_t(q.v.dx) & M40; top.i_vdy = uint64_t(q.v.dy) & M40; top.i_ve = uint32_t(q.v.e) & 0x1ff;
        top.i_lk = q.lk; top.i_lvm = q.lvm;
      }
      top.i_valid = offer;
      top.o_ready = !pct(stall);
      top.clk = 0; top.eval();
      bool const took = top.i_valid && top.i_ready, out = top.o_valid && top.o_ready;
      uint32_t const ou = top.o_u, ov = top.o_v, os = top.o_shade, orr = top.o_r, on = top.o_n, op = top.o_pos, ot = top.o_tag, olv = top.o_lv;
      top.clk = 1; top.eval(); cyc++;
      if (took) si++;
      if (!out) { idle++; continue; }
      idle = 0;
      pexp const &e = ex[oi];
      pin const &q = in[oi];
      bool ok = ot == (oi & 255) && os == e.shade && op == e.pos;
      if (q.tex) ok = ok && ou == e.u && ov == e.v;
      if (q.tex && g_mip) ok = ok && int(olv) == e.lv;
      if (q.o2 != 0) ok = ok && orr == e.r && int(on) == e.n;
      if (!ok && bad++ < 10)
        std::printf("  px %zu (%d,%d) tex %d: rtl u %08x v %08x s %08x r %07x n %u pos %u tag %u lv %u; model u %08x v %08x s %08x r %07x n %d pos %d lv %d\n",
                    oi, q.x, q.y, q.tex, ou, ov, os, orr, on, op, ot, olv, e.u, e.v, e.shade, e.r, e.n, e.pos, e.lv);
      oi++; npx++;
    }
    if (oi < ex.size()) { bad += ex.size() - oi; std::printf("  %zu pixels missing\n", ex.size() - oi); }
  };
  size_t nfr = 0;
  if (nrand) {
    std::mt19937 g(5);
    std::vector<pin> in; std::vector<pexp> ex;
    auto r = [&](int bits) { int64_t m = (int64_t(1) << bits) - 1; int64_t v = int64_t(uint64_t(g()) << 32 | g()) & m; return (g() & 1) ? -v : v; };
    for (int i = 0; i < nrand; i++) {
      rtl_tri t; t.tex = true; t.np = 5; t.recip = 24; t.zbits = 24;
      for (int k = 1; k < 5; k++) {
        auto &p = t.pl[k];
        p.c = r(38 - int(g() % 30)); p.dx = r(29 - int(g() % 29)); p.dy = r(29 - int(g() % 29));
        p.e = int(g() % 120) - 40;
        if (g() % 8 == 0) p.e = int(g() % 500) - 250;
        if (g() % 32 == 0) { p.dx = 0; p.dy = 0; p.c = int64_t(g() % 3) - 1; }
      }
      if (g() % 4) { t.pl[1].c = std::abs(t.pl[1].c); }   // mostly positive 1/w
      t.pl[0] = t.pl[1];
      int const x = int(g() % 496), y = int(g() % 384);
      rtl_px const p = t.eval(x, y);
      pin q{}; q.x = x; q.y = y; q.tex = true; q.o = t.pl[1]; q.u = t.pl[2]; q.v = t.pl[3]; q.s = t.pl[4];
      i128 const o2 = at2(t.pl[1], x, y); q.o2 = int64_t(o2);
      pexp e{}; e.u = p.u; e.v = p.v; e.shade = uint32_t(p.shade); expect_r(o2, e);
      if (g_mip) {
        // B: none, not positive, or any positive finite word; the highest level 4..9 (and past it)
        uint32_t bw = g() % 8 == 0 ? (g() % 2 ? 0x80000000u | (g() & 0x7fffffff) : (g() % 3 ? 0x7f800000u | (g() & 7) : 0u)) : (g() & 0x7f7fffff);
        if (bw == 0x7f800000u) bw = 0;
        q.lk = lk_of(bw); q.lvm = 4 + g() % 12;
        e.lv = lv_of(p.z, bw, int(q.lvm));
      }
      in.push_back(q); ex.push_back(e);
    }
    run(in, ex);
  } else {
    std::vector<std::string> names;
    for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
    std::sort(names.begin(), names.end());
    if (int(names.size()) > frames) names.resize(size_t(frames));
    nfr = names.size();
    std::vector<std::vector<pin>> ins(names.size()); std::vector<std::vector<pexp>> exs(names.size());
    std::vector<std::atomic<int>> ready(names.size());
    for (auto &x : ready) x = 0;
    std::atomic<size_t> nxt{0}, used{0};
    auto wk = [&]() {
      texram tex; std::vector<uint32_t> dl(0x40000);
      for (size_t i; (i = nxt++) < names.size();) {
        while (i > used + 16) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        rframe f;
        bool ok = read_rframe(tris + "/" + names[i] + ".tri", f);
        gzFile gz = ok ? gzopen((dumps + "/" + names[i] + ".bin.gz").c_str(), "rb") : nullptr;
        ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
             gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
        if (gz) gzclose(gz);
        if (ok) {
          for (auto &v : f.vps) for (int k = 0; k < 0x30; k++) v.node[k] = dl[((v.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
          rtl_opts o; o.oow = true; d25_opts d; d.band = 8;
          if (g_mip) { d.mip = 1; d.lod = 1; }
          hook h; h.tr.resize(f.tris.size()); h.in = &ins[i]; h.ex = &exs[i]; d.hook = &h;
          rimage img; d25_stats st;
          render_d25(f, tex, o, d, img, st);
        }
        ready[i] = ok ? 1 : 2;
      }
    };
    std::vector<std::thread> th;
    for (int k = 0; k < 12; k++) th.emplace_back(wk);
    for (size_t i = 0; i < names.size(); i++) {
      while (!ready[i]) std::this_thread::yield();
      if (ready[i] == 2) { bad++; continue; }
      run(ins[i], exs[i]);
      ins[i] = {}; exs[i] = {};
      used = i + 1;
    }
    for (auto &t : th) t.join();
  }
  std::printf("r3d_rast_interp%s: %zu frames, %" PRIu64 " pixels (%d random), %" PRIu64 " wrong; %.3f cycles a pixel (gap %d%%, stall %d%%)\n",
              g_mip ? " (mip 1, lod 1: the level too)" : "", nfr, npx, nrand, bad, double(cyc) / double(std::max<uint64_t>(npx, 1)), gap, stall);
  return bad ? 1 : 0;
}
