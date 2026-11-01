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
// The M2.5 viewer (rtl/viewer/m3v_core.sv) whole, against the C++ models.
//
//   Vm3v_core --vrom FILE.v3v --dump F.bin.gz [--dump ...] --out DIR
//             [--busy PCT] [--lat LO HI] [--seed S] [--sdlat L] [--frames N]
//             [--no-loader] [--video] [--maxrd K] [--no-probe] [--pause]
//             [--idle CYCLES] [--busyrun N]
//
// Memories: the SDRAM is modelled at its pins (m2_sdram drives it as on the
// board: two 64 MB chips on one select, the byte mask on A12:A11), with flat
// storage, read data L cycles after the READ (L matched to rd_lat_sel = 2,
// the board's capture depth); the DDR3 port is hostile: BUSY a random PCT of
// cycles, read bursts after LO..HI cycles, and with --maxrd at most K reads
// held at once (BUSY while K are outstanding), as a bridge with a short
// command queue would. --busyrun N makes the acknowledges late (study R73):
// each time BUSY rises it stays up for 1..N cycles, so a request can wait
// hundreds of cycles to be taken.
//
// The DDR3 probe (m3v_probe) runs throughout unless --no-probe; its results
// are printed and checked at the end (no pattern errors; with --maxrd, the
// most reads held is K). --idle runs that many cycles after the last frame,
// so the probe's idle case gets batches; --pause asks for its case (a).
//
// 1. Loader: windows of the real files through ioctl (both indexes, both
//    region boundaries, each frame's last word), then every byte written is
//    checked in the memory models.
// 2. The rest of the files are put straight into the models (a 49 MiB ioctl
//    stream is hours of simulation), the loader's flags set, and the viewer
//    draws dump 0 (and, with --frames, steps through the rest).
// 3. Checks, each frame:
//    - every word the geometry sends (r3d_geo SETUP_OUT, LOD_WORD: one set-up
//      record a triangle, header words 0-4 with the LOD word B, then x1 y1 x2
//      y2 x3 y3, det, and pdx, pdy, ps a parameter), in order, against the
//      geometry model (r3d_geo, the RTL's default rows, lod_word) and
//      rtl_tri::setup on its triangles; the viewport records' count and length;
//    - every set-up record in the DDR3 store against the same, field for
//      field, list for list, in order, and the store's word 0 (nw, the rows);
//    - the 3D layer in DDR3 against render_d25 with textures at the nearest
//      mip level (mip = 1, lod = 1, B from the geometry model's LOD word; fog
//      off, no culling, translucency mode 0), pixel for pixel;
//    - with --video, one frame of the video output against the layer.
//    PNGs: the RTL's layer, the reference, and the full RTL-form model
//    (render_d25: textures, fog) for what the finished rasteriser will show.

#include "Vm3v_core.h"
#include "Vm3v_core___024root.h"
#include "verilated.h"
#include "r3d_geo.h"
#include "r3d_rast.h"
#include <zlib.h>
#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
static uint32_t fb_(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

// ---- PNG -------------------------------------------------------------------------
static void png_chunk(std::FILE *f, char const *t, std::vector<uint8_t> const &d) {
  uint8_t l[4] = {uint8_t(d.size() >> 24), uint8_t(d.size() >> 16), uint8_t(d.size() >> 8), uint8_t(d.size())};
  std::fwrite(l, 1, 4, f);
  std::fwrite(t, 1, 4, f);
  if (!d.empty()) std::fwrite(d.data(), 1, d.size(), f);
  uLong c = crc32(0, reinterpret_cast<Bytef const *>(t), 4);
  if (!d.empty()) c = crc32(c, d.data(), uInt(d.size()));
  uint8_t cb[4] = {uint8_t(c >> 24), uint8_t(c >> 16), uint8_t(c >> 8), uint8_t(c)};
  std::fwrite(cb, 1, 4, f);
}
static void write_png(std::string const &fn, std::vector<uint32_t> const &px, int w, int h) {
  std::vector<uint8_t> raw;
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    for (int x = 0; x < w; x++) {
      uint32_t const p = px[size_t(y) * size_t(w) + size_t(x)];
      raw.push_back(uint8_t(p >> 16)); raw.push_back(uint8_t(p >> 8)); raw.push_back(uint8_t(p));
    }
  }
  uLongf zl = compressBound(uLong(raw.size()));
  std::vector<uint8_t> z(zl);
  compress2(z.data(), &zl, raw.data(), uLong(raw.size()), 6);
  z.resize(zl);
  std::FILE *f = std::fopen(fn.c_str(), "wb");
  if (!f) return;
  static uint8_t const sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  std::fwrite(sig, 1, 8, f);
  std::vector<uint8_t> ih = {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w),
                             uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h), 8, 2, 0, 0, 0};
  png_chunk(f, "IHDR", ih);
  png_chunk(f, "IDAT", z);
  png_chunk(f, "IEND", {});
  std::fclose(f);
}

// ---- the white-texel render (render_d25 with texel 0xffffffff, fog off,
// translucency 0, back faces already dropped by the geometry) -------------------------
struct c4 { int32_t a, r, g, b; };
static c4 un4(uint32_t c) { return {int32_t((c >> 24) & 0xff), int32_t((c >> 16) & 0xff), int32_t((c >> 8) & 0xff), int32_t(c & 0xff)}; }
static int32_t c8(int32_t x) { return x < 0 ? 0 : x > 255 ? 255 : x; }
static uint32_t pk4(c4 const &c) { return (uint32_t(c8(c.a)) << 24) | (uint32_t(c8(c.r)) << 16) | (uint32_t(c8(c.g)) << 8) | uint32_t(c8(c.b)); }
static int32_t m32(int32_t a, int32_t b) { return int32_t(uint32_t(a) * uint32_t(b)); }
static void sc_imm(c4 &c, int32_t s) { c.a = c8(m32(c.a, s) >> 8); c.r = c8(m32(c.r, s) >> 8); c.g = c8(m32(c.g, s) >> 8); c.b = c8(m32(c.b, s) >> 8); }
static void sc_rgba(c4 &c, c4 const &s) { c.a = c8((c.a * s.a) >> 8); c.r = c8((c.r * s.r) >> 8); c.g = c8((c.g * s.g) >> 8); c.b = c8((c.b * s.b) >> 8); }
static void bl(c4 &c, c4 const &o, uint8_t f) {
  int32_t const s1 = f, s2 = 256 - s1;
  c.a = (c.a * s1 + o.a * s2) >> 8; c.r = (c.r * s1 + o.r * s2) >> 8; c.g = (c.g * s1 + o.g * s2) >> 8; c.b = (c.b * s1 + o.b * s2) >> 8;
}
// render_d25's frag() with the texel given
static bool frag(rtri const &t, rkind k, uint32_t texel, int32_t ii, c4 &col, int &f, bool &zw) {
  f = -1;
  switch (k) {
  case K_SOLID: col = un4(t.color); sc_imm(col, ii); zw = true; return true;
  case K_SOLID_TRANS: {
    col = un4(t.color); sc_imm(col, ii);
    int32_t const tr = int32_t((t.trans << 3) | (t.trans >> 2));
    if (tr != 0xff) f = uint8_t(tr);
    zw = false; return true;
  }
  case K_TEX: col = un4(texel); sc_imm(col, ii); zw = true; return true;
  case K_TEX_COLORMOD: col = un4(texel); sc_rgba(col, un4(t.color)); sc_imm(col, ii); zw = true; return true;
  case K_CONTOUR: {
    uint32_t const fa = texel >> 24;
    if (fa < 0xf8) return false;
    col = un4(texel); sc_rgba(col, un4(t.color)); sc_imm(col, ii); f = int(fa); zw = true; return true;
  }
  case K_TEX_ALPHA: {
    uint32_t const fa = texel >> 24;
    if (fa == 0) return false;
    col = un4(texel); sc_imm(col, ii); f = int(fa); zw = false; return true;
  }
  default: return false;
  }
}
static int g_dx = -1, g_dy = -1;
static void render_white(rframe const &f, rtl_opts const &o, std::vector<uint32_t> &img, uint64_t &shaded) {
  size_t const NP = size_t(SCR_W) * SCR_H;
  img.assign(NP, 0);
  std::vector<uint32_t> zb(NP);
  shaded = 0;
  for (uint32_t pass = 0; pass < 4; pass++) {
    std::vector<int32_t> op, al;
    for (size_t i = 0; i < f.tris.size(); i++)
      if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int32_t(i));
    if (op.empty() && al.empty()) continue;
    std::sort(op.begin(), op.end(), [&](int32_t x, int32_t y) { return f.tris[size_t(x)].buf < f.tris[size_t(y)].buf; });
    std::sort(al.begin(), al.end(), [&](int32_t x, int32_t y) { return f.tris[size_t(x)].buf > f.tris[size_t(y)].buf; });
    std::fill(zb.begin(), zb.end(), 0u);
    auto draw = [&](int32_t idx) {
      rtri const &t = f.tris[size_t(idx)];
      rkind const k = kind_of(t);
      rtl_tri r;
      if (!r.setup(t, k, o)) return;
      for (int32_t y = r.y0; y < r.y1; y++) {
        int32_t is, ie;
        if (!r.span(y, is, ie)) continue;
        for (int x = is; x < ie; x++) {
          size_t const i = size_t(y) * SCR_W + size_t(x);
          rtl_px const p = r.eval(x, y);
          if (x == g_dx && y == g_dy) {
            auto const &pl = r.pl[r.tex ? 4 : 1];
            __int128 v2 = __int128(pl.c) * 2 + __int128(pl.dx) * (2 * x + 1) + __int128(pl.dy) * (2 * y + 1);
            std::printf("model px (%d,%d): tri %d kind %d z %06X ref %06X pass %d v2 %011llX e %d shade %d c %llX dx %llX dy %llX\n", x, y, idx, int(k), p.z, zb[i],
                        p.z >= zb[i], (unsigned long long)(uint64_t(v2) & 0x3ffffffffffull), pl.e, p.shade,
                        (unsigned long long)pl.c, (unsigned long long)pl.dx, (unsigned long long)pl.dy);
          }
          if (!(p.z >= zb[i])) continue;
          c4 col; int fbv; bool zw = false;
          if (!frag(t, k, 0xffffffffu, p.shade, col, fbv, zw)) continue;
          if (fbv >= 0) bl(col, un4(img[i]), uint8_t(fbv));
          img[i] = 0xff000000u | (pk4(col) & 0xffffff);
          if (zw) zb[i] = p.z;
          shaded++;
        }
      }
    };
    for (int32_t i : op) draw(i);
    for (int32_t i : al) draw(i);
  }
}

// ---- memory models ----------------------------------------------------------------
struct sdram_dev {
  std::vector<uint16_t> m = std::vector<uint16_t>(size_t(1) << 26);   // 2 chips x 4 banks x 8192 rows x 1024 columns
  uint32_t row[2][4] = {};
  int lat = 2;                                    // READ on the pins in cycle c -> data on dq in cycle c + lat
  std::deque<std::pair<uint64_t, uint16_t>> rq;   // (cycle, data)
  uint64_t reads = 0, writes = 0;
  static uint32_t idx(uint32_t chip, uint32_t bank, uint32_t r, uint32_t c) { return (chip << 25) | (bank << 23) | (r << 10) | c; }
};
struct ddr_dev {
  static constexpr uint64_t BASE = 0x06000000, SIZE = 0x1300000;   // to the probe's region's end
  unsigned maxrd = 0;
  std::vector<uint64_t> m = std::vector<uint64_t>(SIZE);
  int busy_pct = 20, lat_lo = 8, lat_hi = 40;
  unsigned run_max = 0, run_left = 0;   // --busyrun
  uint32_t wr_left = 0; uint64_t wr_addr = 0;
  struct burst { uint64_t at, addr; uint32_t n; };
  std::deque<burst> rq;
  uint64_t rd_beats = 0, wr_beats = 0, bad_addr = 0;
  uint64_t &at(uint64_t a) {
    static uint64_t sink;
    if (a < BASE || a >= BASE + SIZE) { bad_addr++; return sink; }
    return m[a - BASE];
  }
};

// texture RAM in DDR3 (m3v_loader): texel A = y 2048 + x of slot k at
// TEX + 0x100000 k + {y / 4, x / 4, y % 4}, texel x % 4 a 16-bit lane
static uint64_t tex_word(int k, uint32_t a) {
  uint32_t const x = a & 2047, y = a >> 11;
  return 0x06000000ull + 0xC00000 + 0x100000ull * uint64_t(k) + ((uint64_t(y >> 2) << 11) | (uint64_t(x >> 2) << 2) | (y & 3));
}

static std::vector<uint8_t> read_file(std::string const &fn) {
  std::vector<uint8_t> d;
  std::FILE *f = std::fopen(fn.c_str(), "rb");
  if (!f) return d;
  std::fseek(f, 0, SEEK_END); long n = std::ftell(f); std::fseek(f, 0, SEEK_SET);
  d.resize(size_t(n));
  if (std::fread(d.data(), 1, d.size(), f) != d.size()) d.clear();
  std::fclose(f);
  return d;
}
static std::vector<uint8_t> read_gz(std::string const &fn, size_t n) {
  std::vector<uint8_t> d(n);
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return {};
  if (gzread(g, d.data(), unsigned(n)) != int(n)) d.clear();
  gzclose(g);
  return d;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string vrom_fn, out_dir = ".";
  std::vector<std::string> dumps;
  unsigned seed = 1;
  int busy = 20, lat_lo = 8, lat_hi = 40, sdlat = -1, nframes = 1;
  bool loader = true, video = false, overlay = false, probe = true, pause = false;
  unsigned maxrd = 0, busyrun = 0;
  uint64_t idle_cyc = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--vrom") vrom_fn = nx();
    else if (a == "--dump") dumps.push_back(nx());
    else if (a == "--out") out_dir = nx();
    else if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--sdlat") sdlat = std::atoi(nx().c_str());
    else if (a == "--frames") nframes = std::atoi(nx().c_str());
    else if (a == "--no-loader") loader = false;
    else if (a == "--video") video = true;
    else if (a == "--overlay") overlay = true;
    else if (a == "--maxrd") maxrd = unsigned(std::atoi(nx().c_str()));
    else if (a == "--no-probe") probe = false;
    else if (a == "--pause") pause = true;
    else if (a == "--idle") idle_cyc = uint64_t(std::atoll(nx().c_str()));
    else if (a == "--busyrun") busyrun = unsigned(std::atoi(nx().c_str()));
  }
  if (vrom_fn.empty() || dumps.empty()) { std::fprintf(stderr, "need --vrom and --dump\n"); return 2; }
  fs::create_directories(out_dir);
  nframes = std::min<int>(nframes, int(dumps.size()));

  // the files
  std::vector<uint8_t> vrom = read_file(vrom_fn);
  if (vrom.size() != 0x2000000) { std::fprintf(stderr, "VROM file is not 32 MiB\n"); return 2; }
  std::vector<std::vector<uint8_t>> frm;
  for (auto const &d : dumps) {
    frm.push_back(read_gz(d, 0x1100000));
    if (frm.back().empty()) { std::fprintf(stderr, "bad dump %s\n", d.c_str()); return 2; }
  }
  int const nslots = std::min<int>(int(frm.size()), 6);

  std::mt19937 rng(seed);
  sdram_dev sd; ddr_dev dd;
  dd.busy_pct = busy; dd.lat_lo = lat_lo; dd.lat_hi = lat_hi; dd.maxrd = maxrd; dd.run_max = busyrun;
  sd.lat = sdlat >= 0 ? sdlat : 2;   // matches rd_lat_sel = 2 (CL + 2), the board's capture depth

  auto *t = new Vm3v_core;
  uint64_t cyc = 0;
  bool ddr_rd_ready = false; uint64_t ddr_rd_data = 0;

  // the board's video, captured from the outputs
  int dbg_x = -1, dbg_y = -1;
  if (char const *e = std::getenv("M3V_PIX")) { dbg_x = std::atoi(e); if (char const *c = std::strchr(e, ',')) dbg_y = std::atoi(c + 1); }
  g_dx = dbg_x; g_dy = dbg_y;
  uint64_t pram_bad = 0, sdr_bad = 0, geo_tris = 0;
  std::vector<std::vector<uint32_t>> geo_recs;   // the geometry's records this frame, as sent
  uint64_t tr_cull = 0, tr_vrom = 0, tr_pram = 0, tr_drd[5] = {}, tr_dwr[5] = {}, tr_geo_busy = 0, tr_geo_stall = 0;
  long trace_n = std::getenv("M3V_TRACE") ? std::atol(std::getenv("M3V_TRACE")) : 0;
  bool cap_on = false, cap_done = false;
  std::vector<uint32_t> cap(size_t(SCR_W) * SCR_H, 0);
  int cap_x = 0, cap_y = 0; bool cap_prev_hb = true, cap_prev_vb = true, cap_seen_vb = false;

  auto step = [&]() {
    // inputs for this cycle
    {
      bool b;
      if (dd.run_left) { dd.run_left--; b = true; }
      else {
        b = int(rng() % 100) < dd.busy_pct;
        if (b && dd.run_max > 1) dd.run_left = unsigned(rng() % dd.run_max);
      }
      t->DDRAM_BUSY = (b || (dd.maxrd && dd.rq.size() >= dd.maxrd)) ? 1 : 0;
    }
    ddr_rd_ready = false;
    if (!dd.rq.empty() && dd.rq.front().at <= cyc && (rng() % 100) >= 10) {
      auto &b = dd.rq.front();
      ddr_rd_ready = true; ddr_rd_data = dd.at(b.addr);
      dd.rd_beats++;
      b.addr++;
      if (--b.n == 0) dd.rq.pop_front();
    }
    t->DDRAM_DOUT_READY = ddr_rd_ready; t->DDRAM_DOUT = ddr_rd_data;
    uint16_t dq = 0;
    while (!sd.rq.empty() && sd.rq.front().first < cyc) sd.rq.pop_front();
    if (!sd.rq.empty() && sd.rq.front().first == cyc) dq = sd.rq.front().second;
    t->sd_dq_i = dq;
    t->clk = 0; t->eval();
    // the SDRAM command on the pins this cycle
    if (t->sd_cke) {
      uint32_t const chip = t->sd_cs_n ? 1u : 0u;
      uint32_t const cmd = (uint32_t(t->sd_ras_n) << 2) | (uint32_t(t->sd_cas_n) << 1) | uint32_t(t->sd_we_n);
      uint32_t const bank = t->sd_ba, a = t->sd_a;
      if (cmd == 3) sd.row[chip][bank] = a & 0x1fff;                     // ACTIVATE
      else if (cmd == 5) {                                              // READ
        sd.rq.push_back({cyc + uint64_t(sd.lat), sd.m[sdram_dev::idx(chip, bank, sd.row[chip][bank], a & 0x3ff)]});
        sd.reads++;
      } else if (cmd == 4) {                                            // WRITE, mask on A12:A11
        uint32_t const i = sdram_dev::idx(chip, bank, sd.row[chip][bank], a & 0x3ff);
        uint16_t const d = t->sd_dq_o;
        uint16_t &w = sd.m[i];
        if (!((a >> 11) & 1)) w = uint16_t((w & 0xff00) | (d & 0x00ff));
        if (!((a >> 12) & 1)) w = uint16_t((w & 0x00ff) | (d & 0xff00));
        sd.writes++;
      }
    }
    // DDR3 requests taken this cycle
    if (!t->DDRAM_BUSY) {
      if (t->DDRAM_WE) {
        if (dd.wr_left == 0) { dd.wr_addr = t->DDRAM_ADDR; dd.wr_left = t->DDRAM_BURSTCNT ? t->DDRAM_BURSTCNT : 256; }
        uint64_t &w = dd.at(dd.wr_addr);
        uint64_t const din = t->DDRAM_DIN;
        for (int b = 0; b < 8; b++) if ((t->DDRAM_BE >> b) & 1) w = (w & ~(0xffull << (8 * b))) | (din & (0xffull << (8 * b)));
        dd.wr_addr++; dd.wr_left--; dd.wr_beats++;
      } else if (t->DDRAM_RD) {
        uint64_t const l = uint64_t(dd.lat_lo + int(rng() % unsigned(dd.lat_hi - dd.lat_lo + 1)));
        uint64_t at = cyc + l;
        if (!dd.rq.empty()) at = std::max(at, dd.rq.back().at);
        dd.rq.push_back({at, t->DDRAM_ADDR, t->DDRAM_BURSTCNT ? uint32_t(t->DDRAM_BURSTCNT) : 256u});
      }
    }
    // video capture
    if (cap_on && t->ce_pix) {
      bool const hb = t->v_hblank, vb = t->v_vblank;
      if (!cap_seen_vb) { if (vb) cap_seen_vb = true; }
      else {
        if (!vb && !hb) {
          if (cap_x < SCR_W && cap_y < SCR_H) cap[size_t(cap_y) * SCR_W + size_t(cap_x)] = (uint32_t(t->v_r) << 16) | (uint32_t(t->v_g) << 8) | t->v_b;
          cap_x++;
        }
        if (hb && !cap_prev_hb && !vb) { cap_x = 0; cap_y++; }
        if (vb && !cap_prev_vb && cap_y > 0) { cap_on = false; cap_done = true; }
      }
      cap_prev_hb = hb; cap_prev_vb = vb;
    }
    {
      // traffic by client: geometry words per port, DDR3 beats per master
      auto *rp = t->rootp;
      tr_cull += rp->m3v_core__DOT__cr_rv; tr_vrom += rp->m3v_core__DOT__vr_rv; tr_pram += rp->m3v_core__DOT__pr_rv;
      for (int k = 0; k < 5; k++) { tr_drd[k] += (rp->m3v_core__DOT__d_rvalid >> k) & 1; tr_dwr[k] += (rp->m3v_core__DOT__d_wacc >> k) & 1; }
      tr_geo_busy += rp->m3v_core__DOT__geo_busy;
      tr_geo_stall += rp->m3v_core__DOT__geo_busy && rp->m3v_core__DOT__g_valid && !rp->m3v_core__DOT__g_ready;
    }
    {
      // every polygon RAM word the geometry gets, against the dump
      auto *rp = t->rootp;
      static std::deque<uint32_t> pexp;
      if (rp->m3v_core__DOT__pr_ack) {
        uint32_t const a = rp->m3v_core__DOT__pr_addr, n = uint32_t(rp->m3v_core__DOT__pr_len) + 1;
        auto const &f = frm[size_t(rp->m3v_core__DOT__slot)];
        for (uint32_t i = 0; i < n; i++) {
          uint32_t const w = a + i;
          uint32_t v = 0;
          if (w < 0x100000) { uint32_t const o = 0x500000 + 4 * w; v = uint32_t(f[o]) | (uint32_t(f[o + 1]) << 8) | (uint32_t(f[o + 2]) << 16) | (uint32_t(f[o + 3]) << 24); }
          pexp.push_back(v);
        }
      }
      if (rp->m3v_core__DOT__pr_rv) {
        if (pexp.empty()) { if (pram_bad++ < 10) std::printf("%" PRIu64 ": polygon RAM word with none due\n", cyc); }
        else {
          if (rp->m3v_core__DOT__pr_rd != pexp.front() && pram_bad++ < 10)
            std::printf("%" PRIu64 ": polygon RAM word %08X, dump %08X\n", cyc, rp->m3v_core__DOT__pr_rd, pexp.front());
          pexp.pop_front();
        }
      }
    }
    {
      // culling RAM and VROM words, against the files
      auto *rp = t->rootp;
      static std::deque<uint32_t> cexp, vexp;
      auto const &f = frm[size_t(rp->m3v_core__DOT__slot)];
      auto rd32 = [](std::vector<uint8_t> const &m, size_t o) { return uint32_t(m[o]) | (uint32_t(m[o + 1]) << 8) | (uint32_t(m[o + 2]) << 16) | (uint32_t(m[o + 3]) << 24); };
      if (rp->m3v_core__DOT__cr_ack) {
        uint32_t const a = rp->m3v_core__DOT__cr_addr, n = uint32_t(rp->m3v_core__DOT__cr_len) + 1;
        for (uint32_t i = 0; i < n; i++) {
          uint32_t const w = a + i;
          cexp.push_back(w < 0x100000 ? rd32(f, 4 * size_t(w)) : (w >= 0x800000 && w < 0x840000) ? rd32(f, 0x400000 + 4 * size_t(w - 0x800000)) : 0u);
        }
      }
      if (rp->m3v_core__DOT__cr_rv) {
        if (cexp.empty()) { if (sdr_bad++ < 10) std::printf("%" PRIu64 ": culling word with none due\n", cyc); }
        else { if (rp->m3v_core__DOT__cr_rd != cexp.front() && sdr_bad++ < 10) std::printf("%" PRIu64 ": culling word %08X, dump %08X\n", cyc, rp->m3v_core__DOT__cr_rd, cexp.front()); cexp.pop_front(); }
      }
      if (rp->m3v_core__DOT__vr_ack) {
        uint32_t const a = rp->m3v_core__DOT__vr_addr, n = uint32_t(rp->m3v_core__DOT__vr_len) + 1;
        for (uint32_t i = 0; i < n; i++) { uint32_t const w = a + i; vexp.push_back((w & 0x800000) ? rd32(vrom, 4 * size_t(w & 0x7fffff)) : 0u); }
      }
      if (rp->m3v_core__DOT__vr_rv) {
        if (vexp.empty()) { if (sdr_bad++ < 10) std::printf("%" PRIu64 ": VROM word with none due\n", cyc); }
        else { if (rp->m3v_core__DOT__vr_rd != vexp.front() && sdr_bad++ < 10) std::printf("%" PRIu64 ": VROM word %08X, file %08X\n", cyc, rp->m3v_core__DOT__vr_rd, vexp.front()); vexp.pop_front(); }
      }
    }
    {
      // triangles leaving the geometry: each record's vertex count, less two;
      // and every record, word for word
      auto *rp = t->rootp;
      static bool first = true;
      static std::vector<uint32_t> grec;
      if (rp->m3v_core__DOT__g_valid && rp->m3v_core__DOT__g_ready) {
        if (first) { uint32_t const nv = rp->m3v_core__DOT__g_data >> 27; if (nv >= 3) geo_tris += nv - 2; }
        first = rp->m3v_core__DOT__g_last;
        grec.push_back(rp->m3v_core__DOT__g_data);
        if (rp->m3v_core__DOT__g_last) { geo_recs.push_back(grec); grec.clear(); }
      }
    }
    if (trace_n > 0) {
      auto *rp = t->rootp;
      if (rp->m3v_core__DOT__cr_ack) { std::printf("%" PRIu64 " cull req %06X len %u\n", cyc, rp->m3v_core__DOT__cr_addr, 0u); trace_n--; }
      if (rp->m3v_core__DOT__cr_rv) { std::printf("%" PRIu64 "   cull word %08X\n", cyc, rp->m3v_core__DOT__cr_rd); trace_n--; }
      if (rp->m3v_core__DOT__vr_ack) { std::printf("%" PRIu64 " vrom req %06X len %u\n", cyc, rp->m3v_core__DOT__vr_addr, rp->m3v_core__DOT__vr_len); trace_n--; }
      if (rp->m3v_core__DOT__vr_rv) { std::printf("%" PRIu64 "   vrom word %08X\n", cyc, rp->m3v_core__DOT__vr_rd); trace_n--; }
      if (rp->m3v_core__DOT__pr_ack) { std::printf("%" PRIu64 " pram req %06X len %u\n", cyc, rp->m3v_core__DOT__pr_addr, rp->m3v_core__DOT__pr_len); trace_n--; }
      if (rp->m3v_core__DOT__pr_rv) { std::printf("%" PRIu64 "   pram word %08X\n", cyc, rp->m3v_core__DOT__pr_rd); trace_n--; }
    }
    t->clk = 1; t->eval();
    cyc++;
  };

  // ---- reset: memory out of reset at once, the viewer held -----------------------
  t->mem_rst_n = 0; t->rst = 1; t->ioctl_download = 0; t->ioctl_wr = 0; t->ioctl_index = 0;
  t->rd_lat_sel = 2; t->next_frame = 0; t->redraw = 0; t->loop = 0; t->overlay_en = 0;
  t->probe_en = probe; t->probe_pause = pause; t->ddr_ok = 1;
  for (int i = 0; i < 8; i++) step();
  t->mem_rst_n = 1;
  for (int i = 0; i < 12000 && !t->rootp->m3v_core__DOT__mem_ready; i++) step();
  if (!t->rootp->m3v_core__DOT__mem_ready) { std::printf("FAIL: SDRAM never ready\n"); return 1; }
  std::printf("SDRAM ready at cycle %" PRIu64 "\n", cyc);

  int fails = 0;
  // ---- 1. the loader ------------------------------------------------------------------
  if (loader) {
    struct win { uint32_t lo, hi; };
    std::vector<win> vw = {{0, 0x2000}, {0x0ffe000, 0x1002000}, {0x1ffe000, 0x2000000}};
    uint32_t const F = 0x1100000;
    std::vector<win> fw;
    for (int k = 0; k < nslots; k++) {
      uint32_t const b = uint32_t(k) * F;
      // the region boundaries, and in each region (8C, 8E, polygon RAM,
      // texture RAM) 8 KiB from its first non-zero byte, so that a misplaced
      // byte cannot hide in zeros
      std::vector<win> ws = {{0, 0x1000}, {0x3ff000, 0x401000}, {0x4ff000, 0x501000}, {0x8ff000, 0x901000}, {0x10ff000, F}};
      for (win r : {win{0, 0x400000}, win{0x400000, 0x500000}, win{0x500000, 0x900000}, win{0x900000, F}}) {
        uint32_t o = r.lo;
        while (o < r.hi && frm[size_t(k)][o] == 0) o++;
        // texture RAM: four whole texel rows (16 KiB), from a row a multiple of
        // four, so every beat of a tiled line is written and checked
        uint32_t const len = (r.lo == 0x900000) ? 0x4000u : 0x2000u;
        // (past the region's boundary window, which holds row 0: rows 1-8 at least)
        o = (r.lo == 0x900000) ? std::max<uint32_t>(r.lo + 0x1000, r.lo + ((o - r.lo) & ~0x3fffu)) : (o & ~0xfffu);
        if (o + len <= r.hi && o >= 0x1000 + r.lo) ws.push_back({o, o + len});
      }
      std::sort(ws.begin(), ws.end(), [](win const &x, win const &y) { return x.lo < y.lo; });
      for (win w : ws) if (fw.empty() || b + w.lo >= fw.back().hi) fw.push_back({b + w.lo, b + w.hi});
    }
    auto send = [&](uint16_t index, std::vector<win> const &ws, auto get) {
      t->ioctl_index = index; t->ioctl_download = 1;
      for (int i = 0; i < 4; i++) step();
      for (win w : ws)
        for (uint32_t a = w.lo; a < w.hi; a += 2) {
          while (t->ioctl_wait) step();
          t->ioctl_addr = a; t->ioctl_dout = get(a); t->ioctl_wr = 1; step(); t->ioctl_wr = 0;
          for (int k = 0; k < 3; k++) step();
        }
      for (int i = 0; i < 4; i++) step();
      t->ioctl_download = 0;
      for (int i = 0; i < 20000; i++) step();
    };
    send(1, vw, [&](uint32_t a) -> uint16_t { return uint16_t(vrom[a] | (vrom[a + 1] << 8)); });
    send(2, fw, [&](uint32_t a) -> uint16_t { auto const &f = frm[a / F]; uint32_t const o = a % F; return uint16_t(f[o] | (f[o + 1] << 8)); });
    auto *rp = t->rootp;
    uint64_t bad = 0, n = 0;
    for (win w : vw)
      for (uint32_t a = w.lo; a < w.hi; a += 2, n++) if (sd.m[a >> 1] != uint16_t(vrom[a] | (vrom[a + 1] << 8))) bad++;
    for (win w : fw)
      for (uint32_t a = w.lo; a < w.hi; a += 2, n++) {
        uint32_t const k = a / F, o = a % F;
        uint16_t const v = uint16_t(frm[k][o] | (frm[k][o + 1] << 8));
        uint32_t const slot = 0x2000000 + k * 0x1000000;
        uint16_t got;
        if (o < 0x500000) got = sd.m[(slot + o) >> 1];
        else if (o < 0x900000) { uint64_t const q = dd.at(ddr_dev::BASE + 0x100000 + uint64_t(k) * 0x80000 + ((o - 0x500000) >> 3)); got = uint16_t(q >> (16 * ((o >> 1) & 3))); }
        else got = uint16_t(dd.at(tex_word(k, (o - 0x900000) >> 1)) >> (16 * (((o - 0x900000) >> 1) & 3)));
        if (got != v) bad++;
      }
    bool const flags = rp->m3v_core__DOT__u_ld__DOT__vrom_loaded && rp->m3v_core__DOT__u_ld__DOT__frames_loaded && rp->m3v_core__DOT__u_ld__DOT__n_frames == nslots;
    std::printf("loader: %" PRIu64 " words in windows, %" PRIu64 " wrong; flags vrom %d frames %d n %d (want %d)%s\n", n, bad,
                rp->m3v_core__DOT__u_ld__DOT__vrom_loaded, rp->m3v_core__DOT__u_ld__DOT__frames_loaded, rp->m3v_core__DOT__u_ld__DOT__n_frames, nslots,
                (bad || !flags) ? "  FAIL" : "");
    if (bad || !flags) fails++;
  }

  // ---- 2. the rest straight into the models --------------------------------------------
  for (uint32_t a = 0; a < 0x2000000; a += 2) sd.m[a >> 1] = uint16_t(vrom[a] | (vrom[a + 1] << 8));
  for (int k = 0; k < nslots; k++) {
    auto const &f = frm[size_t(k)];
    uint32_t const slot = 0x2000000 + uint32_t(k) * 0x1000000;
    for (uint32_t o = 0; o < 0x500000; o += 2) sd.m[(slot + o) >> 1] = uint16_t(f[o] | (f[o + 1] << 8));
    for (uint32_t o = 0x900000; o < 0x1100000; o += 8) {
      uint64_t q = 0;
      for (int b = 7; b >= 0; b--) q = (q << 8) | f[o + uint32_t(b)];
      dd.at(tex_word(k, (o - 0x900000) >> 1)) = q;
    }
    for (uint32_t o = 0x500000; o < 0x900000; o += 8) {
      uint64_t q = 0;
      for (int b = 7; b >= 0; b--) q = (q << 8) | f[o + uint32_t(b)];
      dd.at(ddr_dev::BASE + 0x100000 + uint64_t(k) * 0x80000 + ((o - 0x500000) >> 3)) = q;
    }
  }
  {
    auto *rp = t->rootp;
    rp->m3v_core__DOT__u_ld__DOT__vrom_loaded = 1;
    rp->m3v_core__DOT__u_ld__DOT__frames_loaded = 1;
    rp->m3v_core__DOT__u_ld__DOT__n_frames = uint8_t(nslots);
  }
  t->rst = 0;

  // ---- 3. frames ------------------------------------------------------------------------
  uint64_t pix_prev = 0;
  rtl_opts ro; ro.sub = 16; ro.zbits = 24; ro.recip_bits = 24; ro.ftz = true; ro.oow = true;
  for (int fi = 0; fi < nframes; fi++) {
    uint32_t const done0 = t->frames_done;
    tr_cull = tr_vrom = tr_pram = tr_geo_busy = tr_geo_stall = 0;
    for (int k = 0; k < 5; k++) tr_drd[k] = tr_dwr[k] = 0;
    geo_recs.clear();
    if (fi > 0) { t->next_frame = 1; step(); t->next_frame = 0; }
    uint64_t const c0 = cyc;
    uint64_t geo_end = 0;
    bool timeout = false;
    while (t->frames_done == done0) {
      step();
      if (!geo_end && t->rootp->m3v_core__DOT__q == 5) geo_end = cyc - c0;   // Q_BAND
      if (cyc - c0 > 400000000ull) { timeout = true; break; }
    }
    if (timeout) { std::printf("frame %d: TIMEOUT, sequencer state %d band %d\n", fi, t->rootp->m3v_core__DOT__q, t->rootp->m3v_core__DOT__band); return 1; }
    uint64_t const frame_cyc = cyc - c0;
    uint64_t const f_cull = tr_cull, f_vrom = tr_vrom, f_pram = tr_pram, f_busy = tr_geo_busy, f_stall = tr_geo_stall;
    uint64_t f_drd[5], f_dwr[5];
    for (int k = 0; k < 5; k++) { f_drd[k] = tr_drd[k]; f_dwr[k] = tr_dwr[k]; }
    // the swap, then one video frame
    bool const show0 = t->fb_show_o;
    for (int i = 0; i < 2000000 && t->fb_show_o == show0; i++) step();
    if (video) {
      t->overlay_en = overlay;
      cap_on = true; cap_done = false; cap_x = 0; cap_y = 0; cap_seen_vb = false; cap_prev_hb = true; cap_prev_vb = true;
      for (int i = 0; i < 4000000 && !cap_done; i++) step();
    }

    // ---- the reference: the geometry model on the dump ----
    r3d_mem mem;
    std::memcpy(mem.cull_lo.data(), frm[size_t(fi)].data(), 0x400000);
    std::memcpy(mem.cull_hi.data(), frm[size_t(fi)].data() + 0x400000, 0x100000);
    std::memcpy(mem.poly.data(), frm[size_t(fi)].data() + 0x500000, 0x400000);
    std::fill(mem.vrom.begin(), mem.vrom.begin() + 0x800000, 0u);
    std::memcpy(mem.vrom.data() + 0x800000, vrom.data(), 0x2000000);
    r3d_geo geo(mem, arith::rtl, false);
    geo.frustum_sm = true; geo.cull = true; geo.rows(true); geo.step = 0x10;
    geo.lod_word = true;   // LOD_WORD = 1: each 'T' record ends with d and B
    geo.frame(uint32_t(fi));
    std::vector<record> recs;
    geo.out.records(recs);
    rframe fr;
    uint32_t cnt[2] = {0, 0};
    uint64_t recip_diff = 0, tex_flag_diff = 0, vp_model = 0;
    fp_unit rq;
    // per triangle: the model's B (render_d25's lodb, by tr.poly, which is
    // made the triangle's index so that B is the triangle's own record's), and
    // the header fields the geometry sends: {pass, alpha, polygon, the ten
    // texture and colour fields, inverted copy, fog intensity, layer}
    std::vector<uint32_t> lodb;
    std::vector<std::array<uint32_t, 16>> hexp;
    std::vector<record const *> precs;
    for (auto const &r : recs) if (r.tag == 'P') precs.push_back(&r);
    for (auto const &r : recs) {
      if (r.tag == 'V') vp_model++;
      if (r.tag != 'T') continue;
      if (r.w.size() >= 37 && (r.w[35] & 2)) continue;   // faces away: FACE_DROP
      rtri tr;
      tr.pass = r.w[0]; tr.alpha = r.w[1]; tr.buf = cnt[tr.alpha & 1]++; tr.poly = uint32_t(fr.tris.size());
      lodb.push_back(r.w.size() >= 39 ? r.w.back() : 0xffffffffu);
      {
        std::array<uint32_t, 16> h{};
        h[0] = r.w[0] & 3; h[1] = r.w[1] & 1; h[2] = r.w[3] & 0xfffff;
        for (int k = 0; k < 10; k++) h[size_t(3 + k)] = r.w[size_t(25 + k)];
        h[13] = r.w.size() >= 37 ? (r.w[35] & 1) : 0;
        uint32_t const h6 = r.w[3] < precs.size() ? precs[r.w[3]]->w[9] : 0;
        h[14] = (h6 >> 11) & 31; h[15] = (h6 >> 17) & 1;
        hexp.push_back(h);
      }
      for (int v = 0; v < 3; v++) {
        size_t const b = 4 + 7 * size_t(v);
        tr.x[v] = bf(r.w[b]); tr.y[v] = bf(r.w[b + 1]); tr.z[v] = bf(r.w[b + 2]); tr.w[v] = bf(r.w[b + 3]);
        tr.p0[v] = bf(r.w[b + 4]); tr.p1[v] = bf(r.w[b + 5]); tr.p2[v] = bf(r.w[b + 6]);
        if (fb_(rq.recip(tr.w[v])) != fb_(1.0f / tr.w[v])) recip_diff++;
      }
      tr.tex_en = r.w[25]; tr.page = r.w[26]; tr.tex_x = r.w[27]; tr.tex_y = r.w[28];
      tr.tex_w = r.w[29]; tr.tex_h = r.w[30]; tr.fmt = r.w[31]; tr.param = r.w[32];
      tr.trans = r.w[33]; tr.color = r.w[34];
      if (bool(tr.tex_en) != bool(tr.param & TP_TEXTURE)) tex_flag_diff++;
      fr.tris.push_back(tr);
    }
    // ---- set-up records against rtl_tri::setup ----
    uint64_t rec_n = 0, rec_bad = 0, rec_count_bad = 0, rec_flat = 0;
    {
      std::vector<std::vector<size_t>> want(8);
      for (size_t i = 0; i < fr.tris.size(); i++) {
        rtri const &tr = fr.tris[i];
        bool fin = true;
        for (int v = 0; v < 3; v++) fin = fin && std::isfinite(tr.x[v]) && std::isfinite(tr.y[v]) && std::isfinite(1.0f / tr.w[v]);
        if (fin) want[(tr.pass & 3) * 2 + (tr.alpha & 1)].push_back(i);
      }
      int shown = 0;
      for (int l = 0; l < 8; l++) {
        uint32_t const got_n = (t->rootp->m3v_core__DOT__st_cnt[l / 2] >> (16 * (l % 2))) & 0xffff;
        if (got_n != want[size_t(l)].size()) { rec_count_bad++; std::printf("list %d: %u records, model %zu\n", l, got_n, want[size_t(l)].size()); }
        for (size_t j = 0; j < std::min<size_t>(got_n, want[size_t(l)].size()); j++) {
          rtri const &tr = fr.tris[want[size_t(l)][j]];
          rtl_tri r;
          r.setup(tr, kind_of(tr), ro);
          uint32_t w[32];
          uint64_t const base = ddr_dev::BASE + 0x800000 + uint64_t(l) * 0x80000 + j * 16;
          for (int k = 0; k < 16; k++) { uint64_t const q = dd.at(base + uint64_t(k)); w[2 * k] = uint32_t(q); w[2 * k + 1] = uint32_t(q >> 32); }
          // SETUP4 (R65): the model's planes 0 (1/w), 2, 3, 4 textured (its plane
          // 1 is the second copy of 1/w, which the record no longer carries);
          // 0, 1 untextured
          int const pmap_t[4] = {0, 2, 3, 4}, pmap_u[2] = {0, 1};
          int const np = r.tex ? 4 : 2;
          int const *pmap = r.tex ? pmap_t : pmap_u;
          std::vector<uint32_t> e = {w[1] & 0xffffffffu};
          (void)e;
          bool ok = true;
          auto ck = [&](int wi, uint32_t v, char const *nm) {
            if (w[wi] != v) { ok = false; if (shown < 12) { shown++; std::printf("list %d rec %zu word %d (%s): rtl %08X model %08X\n", l, j, wi, nm, w[wi], v); } }
          };
          // the stored record: word 0 the store's, 1-5 the header (5 the LOD
          // word), 6-11 x1 y1 x2 y2 x3 y3, 12 det, 13.. the planes
          ck(5, lodb[want[size_t(l)][j]], "B");
          ck(6, fb_(r.fx[0]), "x1"); ck(7, fb_(r.fy[0]), "y1"); ck(8, fb_(r.fx[1]), "x2");
          ck(9, fb_(r.fy[1]), "y2"); ck(10, fb_(r.fx[2]), "x3"); ck(11, fb_(r.fy[2]), "y3");
          ck(12, fb_(r.det), "det");
          for (int p = 0; p < np; p++) {
            int const m = pmap[p];
            ck(13 + 3 * p, fb_(r.pdx[m]), "pdx"); ck(14 + 3 * p, fb_(r.pdy[m]), "pdy"); ck(15 + 3 * p, fb_(r.ps[m]), "ps");
          }
          if (((w[0] >> 18) & 31) != uint32_t(12 + 3 * np)) { ok = false; if (shown < 12) { shown++; std::printf("list %d rec %zu: nw %u\n", l, j, (w[0] >> 18) & 31); } }
          {
            // the rows: floor(y1), floor(y3) clamped to 0..511
            auto fl = [](float y) { return y < 0.0f || !(y == y) ? 0u : y >= 512.0f ? 511u : uint32_t(std::floor(y)); };
            uint32_t const y0 = fl(r.fy[0]), y1 = fl(r.fy[2]);
            if ((w[0] & 0x1ff) != y0 || ((w[0] >> 9) & 0x1ff) != y1) { ok = false; if (shown < 12) { shown++; std::printf("list %d rec %zu: rows %u..%u, model %u..%u\n", l, j, w[0] & 0x1ff, (w[0] >> 9) & 0x1ff, y0, y1); } }
          }
          rec_n++;
          if (r.flat) rec_flat++;
          if (!ok) rec_bad++;
        }
      }
    }
    // ---- the geometry's words, in the order sent ----
    uint64_t gw_bad = 0, gw_recs = 0, gw_vp = 0;
    {
      std::vector<size_t> fin;   // the triangles the geometry sends (x, y, 1/w finite), in order
      for (size_t i = 0; i < fr.tris.size(); i++) {
        rtri const &tr = fr.tris[i];
        bool ok = true;
        for (int v = 0; v < 3; v++) ok = ok && std::isfinite(tr.x[v]) && std::isfinite(tr.y[v]) && std::isfinite(1.0f / tr.w[v]);
        if (ok) fin.push_back(i);
      }
      size_t ti = 0;
      int shown = 0;
      auto bad = [&](char const *what, size_t k, uint32_t got, uint32_t want) {
        gw_bad++;
        if (shown < 12) { shown++; std::printf("geometry record %" PRIu64 " (triangle %zu) %s word %zu: rtl %08X model %08X\n", gw_recs, ti, what, k, got, want); }
      };
      for (auto const &g : geo_recs) {
        if ((g[0] >> 27) == 0) {   // a viewport record
          gw_vp++;
          if (g.size() != 6) bad("viewport record length", 0, uint32_t(g.size()), 6);
          continue;
        }
        gw_recs++;
        if (ti >= fin.size()) { bad("record past the model's last", 0, 0, 0); continue; }
        rtri const &tr = fr.tris[fin[ti]];
        auto const &h = hexp[fin[ti]];
        rtl_tri r;
        r.setup(tr, kind_of(tr), ro);
        int const np = r.tex ? 4 : 2;
        size_t const len = 5 + 7 + 3 * size_t(np);
        if ((g[0] >> 27) != 3 || g.size() != len) { bad("n / length", 0, uint32_t((g[0] >> 27) * 1000 + g.size()), uint32_t(3000 + len)); ti++; continue; }
        uint32_t const w0 = g[0], w1 = g[1], w3 = g[3];
        uint32_t const f[16] = {w0 & 3, (w0 >> 2) & 1, w1 & 0xfffff, (w0 >> 4) & 1, (w0 >> 5) & 1, (w1 >> 20) & 0x3f, (w1 >> 26) & 0x1f,
                                (w0 >> 6) & 7, (w0 >> 9) & 7, (w0 >> 12) & 7, (w0 >> 15) & 0x3f, (w0 >> 21) & 0x3f, g[2], (w0 >> 3) & 1, w3 & 31, (w3 >> 5) & 1};
        for (size_t k = 0; k < 16; k++) if (f[k] != h[k]) bad("header field", k, f[k], h[k]);
        if (w3 >> 6) bad("header word 3 high bits", 3, w3, w3 & 63);
        if (g[4] != lodb[fin[ti]]) bad("LOD word", 4, g[4], lodb[fin[ti]]);
        int const pm_t[4] = {0, 2, 3, 4}, pm_u[2] = {0, 1};
        int const *pm = r.tex ? pm_t : pm_u;
        std::vector<uint32_t> e;
        for (int v = 0; v < 3; v++) { e.push_back(fb_(r.fx[v])); e.push_back(fb_(r.fy[v])); }
        e.push_back(fb_(r.det));
        for (int p = 0; p < np; p++) { e.push_back(fb_(r.pdx[pm[p]])); e.push_back(fb_(r.pdy[pm[p]])); e.push_back(fb_(r.ps[pm[p]])); }
        for (size_t k = 0; k < e.size(); k++)
          if (g[5 + k] != e[k] && !(std::isnan(bf(g[5 + k])) && std::isnan(bf(e[k])))) bad("set-up", 5 + k, g[5 + k], e[k]);
        ti++;
      }
      if (ti != fin.size()) { std::printf("geometry: %zu set-up records, model %zu\n", ti, fin.size()); gw_bad++; }
      if (gw_vp != vp_model) { std::printf("geometry: %" PRIu64 " viewport records, model %" PRIu64 "\n", gw_vp, vp_model); gw_bad++; }
    }
    // ---- images ----
    // the reference: render_d25 with this frame's textures at the nearest mip
    // level, B the geometry model's LOD word (mip = 1, lod = 1), fog off
    // (render_d25's fog needs the viewport records the bench's frame lacks)
    std::vector<uint32_t> ref;
    uint64_t ref_shaded = 0;
    uint64_t const self_bad = 0;
    {
      texram tex;
      std::memcpy(tex.t.data(), frm[size_t(fi)].data() + 0x900000, 0x800000);
      rimage img; d25_stats st; d25_opts dop; dop.fog = false; dop.cull = 0; dop.trans = 0; dop.band = 8;
      dop.mip = 1; dop.lod = 1; dop.lodb = &lodb;
      render_d25(fr, tex, ro, dop, img, st);
      ref = img.px;
      ref_shaded = st.shaded;
    }
    bool const show = t->fb_show_o;
    std::vector<uint32_t> got(size_t(SCR_W) * SCR_H);
    for (int y = 0; y < SCR_H; y++)
      for (int x = 0; x < SCR_W; x++) {
        uint64_t const q = dd.at(ddr_dev::BASE + (show ? 0x20000u : 0u) + uint64_t(y) * 256 + uint64_t(x >> 1));
        uint32_t const p = uint32_t(x & 1 ? q >> 32 : q) & 0x1ffffff;
        got[size_t(y) * SCR_W + size_t(x)] = ((p >> 24) & 1 ? 0xff000000u : 0u) | (p & 0xffffff);
      }
    uint64_t px_bad = 0, drawn = 0;
    int shown = 0;
    for (size_t i = 0; i < got.size(); i++) {
      uint32_t const a = got[i], b = ref[i];
      bool const da = a >> 24, db = b >> 24;
      drawn += db;
      if (da != db || (db && (a & 0xffffff) != (b & 0xffffff))) {
        px_bad++;
        if (shown < 10) { shown++; std::printf("pixel (%zu, %zu): rtl %08X model %08X\n", i % SCR_W, i / SCR_W, a, b); }
      }
    }
    uint64_t vid_bad = 0;
    if (video) {
      // (with the overlay on, its box, 8..296 x 8..44, is left out)
      for (size_t i = 0; i < got.size(); i++) {
        size_t const x = i % SCR_W, y = i / SCR_W;
        if (overlay && x >= 6 && x < 300 && y >= 6 && y < (probe ? 120u : 48u)) continue;
        uint32_t const want = (got[i] >> 24) ? (got[i] & 0xffffff) : 0;
        if (cap[i] != want) vid_bad++;
      }
    }
    std::string const stem = out_dir + "/" + fs::path(dumps[size_t(fi)]).stem().stem().string();
    write_png(stem + "_rtl.png", got, SCR_W, SCR_H);
    write_png(stem + "_ref.png", ref, SCR_W, SCR_H);
    if (video) write_png(stem + "_video.png", cap, SCR_W, SCR_H);
    write_png(stem + "_d25tex.png", ref, SCR_W, SCR_H);
    auto *rp = t->rootp;
    std::printf("frame %d: geometry %" PRIu64 " set-up records and %" PRIu64 " viewport records, %" PRIu64 " words or counts wrong\n", fi, gw_recs, gw_vp, gw_bad);
    std::printf("frame %d (%s): %zu triangles, %" PRIu64 " set-up records checked, %" PRIu64 " wrong, %" PRIu64 " lists miscounted; "
                "1/w differs from 1.0f/w on %" PRIu64 " vertices, texture flag on %" PRIu64 "; %" PRIu64 " flat (|det| < 1e-5)\n",
                fi, fs::path(dumps[size_t(fi)]).filename().c_str(), fr.tris.size(), rec_n, rec_bad, rec_count_bad, recip_diff, tex_flag_diff, rec_flat);
    std::printf("  image: %" PRIu64 " pixels drawn (model), %" PRIu64 " differ%s; %" PRIu64 " pixels shaded by the RTL, %" PRIu64 " by the model\n",
                drawn, px_bad, video ? (vid_bad ? ", video output DIFFERS" : ", video output matches") : "",
                uint64_t(rp->m3v_core__DOT__n_pix) - pix_prev, ref_shaded);
    pix_prev = rp->m3v_core__DOT__n_pix;
    std::printf("  cycles: geometry phase %" PRIu64 " (%.2f ms at 100 MHz), frame %" PRIu64 " (%.1f ms); geometry err %02X, store overflow %d\n",
                geo_end, geo_end / 1e5, frame_cyc, frame_cyc / 1e5, rp->m3v_core__DOT__geo_err, rp->m3v_core__DOT__st_ovf);
    std::printf("  memory: SDRAM %" PRIu64 " reads %" PRIu64 " writes (16-bit), DDR3 %" PRIu64 " read beats %" PRIu64 " write beats, %" PRIu64 " outside the model\n",
                sd.reads, sd.writes, dd.rd_beats, dd.wr_beats, dd.bad_addr);
    std::printf("  traffic: geometry words culling %" PRIu64 ", VROM %" PRIu64 ", polygon RAM %" PRIu64 " (%.1f / %.1f / %.1f MB/s at 57.52 Hz); "
                "geometry busy %" PRIu64 " cycles, held by the store %" PRIu64 "\n",
                f_cull, f_vrom, f_pram, f_cull * 4 * 57.52 / 1e6, f_vrom * 4 * 57.52 / 1e6, f_pram * 4 * 57.52 / 1e6, f_busy, f_stall);
    std::printf("  DDR3 beats (read / write): scan-out %" PRIu64 " / -, loader - / %" PRIu64 ", polygon RAM %" PRIu64 " / -, store %" PRIu64 " / %" PRIu64 ", layer - / %" PRIu64 "\n",
                f_drd[0], f_dwr[1], f_drd[2], f_drd[3], f_dwr[3], f_dwr[4]);
    std::printf("  geometry: %llu triangles out (all frames so far)\n", (unsigned long long)geo_tris);
    std::printf("  set-up (geometry): %u triangles stored; store counts %u %u %u %u %u %u %u %u\n", unsigned(rp->m3v_core__DOT__su_tris),
                unsigned(rp->m3v_core__DOT__st_cnt[0] & 0xffff), unsigned(rp->m3v_core__DOT__st_cnt[0] >> 16), unsigned(rp->m3v_core__DOT__st_cnt[1] & 0xffff), unsigned(rp->m3v_core__DOT__st_cnt[1] >> 16),
                unsigned(rp->m3v_core__DOT__st_cnt[2] & 0xffff), unsigned(rp->m3v_core__DOT__st_cnt[2] >> 16), unsigned(rp->m3v_core__DOT__st_cnt[3] & 0xffff), unsigned(rp->m3v_core__DOT__st_cnt[3] >> 16));
    if (sdr_bad) std::printf("  culling RAM / VROM: %" PRIu64 " words differ\n", sdr_bad);
    if (pram_bad) std::printf("  polygon RAM: %" PRIu64 " words differ from the dump\n", pram_bad);
    if (sdr_bad || pram_bad || rec_bad || rec_count_bad || px_bad || vid_bad || self_bad || dd.bad_addr || rp->m3v_core__DOT__geo_err || gw_bad) fails++;
  }
  if (probe) {
    for (uint64_t i = 0; i < idle_cyc; i++) step();
    auto *rp = t->rootp;
    auto st = [&](int k) { return uint32_t(rp->m3v_core__DOT__g_probe__DOT__u_probe__DOT__st[k]); };
    char const *cn[3] = {"(a) idle, scan-out paused", "(b) idle, scan-out", "(c) drawing"};
    for (int c = 0; c < 3; c++)
      std::printf("probe %-26s %4u batches: latency min %u median %u p99 %u max %u; MB/s at 1/4/8/16 in flight %.1f %.1f %.1f %.1f\n",
                  cn[c], st(28 + c), st(8 * c), st(8 * c + 1), st(8 * c + 2), st(8 * c + 3),
                  st(8 * c + 4) / 1000.0, st(8 * c + 5) / 1000.0, st(8 * c + 6) / 1000.0, st(8 * c + 7) / 1000.0);
    std::printf("probe: %u loops; taken before the first beat %u, held at once %u, pattern errors %u, slowest first beat %u\n",
                st(31), st(24), st(25), st(26), st(27));
    bool bad = st(26) != 0 || st(31) == 0;
    if (maxrd && st(25) != std::min(maxrd, 32u)) bad = true;
    if (st(25) > 32 || (st(28 + 2) && st(8 * 2) < unsigned(lat_lo))) bad = true;
    if (bad) { std::printf("probe: FAIL\n"); fails++; }
  }
  std::printf("%s\n", fails ? "FAIL" : "PASS");
  delete t;
  return fails ? 1 : 0;
}
