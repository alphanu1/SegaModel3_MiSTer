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
// r3d_rast, the whole rasteriser, against render_d25 (depth on 1/w, the
// normal's cull flag, fog, translucency 0, textures at level 0; --mip: the
// nearest level, lod = 1, with each triangle's B in its record; --geo-lod
// VROM: that B is the geometry's LOD word, the record the geometry sends
// (r3d_geo_lod.h, study R75), in the record and in the model alike).
//
// Input: the records the geometry sends, built from the model: every triangle
// render_d25 draws (back faces are not sent, as with FACE_DROP), in the list's
// order, as its SETUP4 record (rtl_tri::setup's FP32 results), each
// viewport's record (R62) before its first triangle. Frames go in back to
// back: frame N + 1 is binned while frame N is replayed. Scroll fog and the
// layer's base come in at f_end.
//
// Memory: one flat DDR3 (layer buffers, the tiled texture RAM of m3v_loader,
// the store's banks), three ports with their contracts enforced:
//   store, layer  m2_ddr3: req held to ack with addr, blen, we unchanged; a
//                 request is taken (after ACK cycles) only when no ack is out;
//                 a write beat taken when not BUSY (wacc), a read beat per
//                 rvalid after LAT cycles with GAP holes; ack after the last
//   texture       pipelined reads: t_addr held to t_ack (after TACK cycles,
//                 refused while MAXRD lines are out), four beats later in order
// ACK and TACK draw from LO..HI on PCT percent of requests, 1..3 otherwise.
// Every layer pixel of every frame is compared with render_d25's image.
//
//   Vr3d_rast --tris DIR --dumps DIR [--frames N] [--first K] [--seed S]
//             [--ack LO HI PCT] [--tack LO HI PCT] [--busy PCT] [--lat LO HI]
//             [--gap PCT] [--maxrd K] [--igap PCT] [--mip] [--geo-lod VROM] [--out DIR]

#include "Vr3d_rast.h"
#include "Vr3d_rast___024root.h"
#include "verilated.h"
#include "r3d_rast.h"
#include "r3d_geo_lod.h"
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <zlib.h>

using namespace r3d;
namespace fs = std::filesystem;

static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

static constexpr uint64_t FB0 = 0x000000, FB1 = 0x020000, TEX = 0x100000, STORE = 0x200000, MEMW = 0x600000;

static int g_px = -1, g_py = -1;       // R3D_PIX=x,y: trace that pixel, model and RTL
struct hook : d25_hook {
  std::vector<int> drawn;              // index -> set-up slot, -1 not drawn
  std::vector<rtl_tri> su;
  void px(d25_px const &p) override {
    if (p.x != g_px || p.y != g_py || p.layer) return;
    std::printf("    model tri %d kind %d z %06X ref %06X pass %d ok %d zw %d texel %08X shade %d fog %d P %d raw %06X fog %06X dst %08X out %08X fb %d\n",
                p.tri, int(p.k), p.z, p.zref, p.zpass, p.ok, p.zw, p.texel, p.shade, p.fog, p.P, p.col_raw & 0xffffff, p.col_fog & 0xffffff, p.dst, p.out, p.fb);
  }
  void tri(int32_t idx, int layer, rtl_tri const &r, rkind, bool culled) override {
    if (layer || culled) return;
    if (size_t(idx) >= drawn.size()) drawn.resize(size_t(idx) + 1, -1);
    drawn[size_t(idx)] = int(su.size());
    su.push_back(r);
  }
};

struct frame_in {
  std::string name;
  std::vector<std::pair<uint32_t, bool>> words;   // (word, last)
  std::vector<uint32_t> want;                    // layer words, {drawn, rgb}
  d25_sfog sg; bool sf = false;
  uint64_t tris = 0, shaded = 0;
  std::vector<uint16_t> tex;                     // its texture RAM, loaded when its replay is next
};

int main(int argc, char **argv) {
  std::string tris, dumps, out, geo_vrom;
  int frames = 1 << 30, first = 0;
  unsigned seed = 1;
  int ack_lo = 1, ack_hi = 3, ack_p = 0, tack_lo = 1, tack_hi = 3, tack_p = 0, busy = 20, lat_lo = 13, lat_hi = 40, gap = 10, igap = 10;
  unsigned maxrd = 16;
  bool mip = false;
  if (char const *e = std::getenv("R3D_PIX")) { g_px = std::atoi(e); if (char const *c = std::strchr(e, ',')) g_py = std::atoi(c + 1); }
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : "0"); };
    if (a == "--tris") tris = nx(); else if (a == "--dumps") dumps = nx(); else if (a == "--out") out = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str()); else if (a == "--first") first = std::atoi(nx().c_str());
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--ack") { ack_lo = std::atoi(nx().c_str()); ack_hi = std::atoi(nx().c_str()); ack_p = std::atoi(nx().c_str()); }
    else if (a == "--tack") { tack_lo = std::atoi(nx().c_str()); tack_hi = std::atoi(nx().c_str()); tack_p = std::atoi(nx().c_str()); }
    else if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--gap") gap = std::atoi(nx().c_str()); else if (a == "--igap") igap = std::atoi(nx().c_str());
    else if (a == "--maxrd") maxrd = unsigned(std::atoi(nx().c_str()));
    else if (a == "--mip") mip = true;
    else if (a == "--geo-lod") geo_vrom = nx();
  }
  Verilated::commandArgs(argc, argv);
  std::vector<std::string> names;
  for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
  std::sort(names.begin(), names.end());
  if (first > 0) names.erase(names.begin(), names.begin() + std::min<size_t>(names.size(), size_t(first)));
  if (int(names.size()) > frames) names.resize(size_t(frames));

  std::unique_ptr<r3d_mem> gm;
  if (!geo_vrom.empty()) {
    gm = std::make_unique<r3d_mem>(); gm->vrom.assign(0x1000000, 0);
    std::FILE *fv = std::fopen(geo_vrom.c_str(), "rb");
    if (!fv) { std::fprintf(stderr, "no VROM %s\n", geo_vrom.c_str()); return 2; }
    size_t const n = std::fread(gm->vrom.data(), 4, gm->vrom.size(), fv); (void)n; std::fclose(fv);
  }
  uint64_t geo_given = 0, geo_tex = 0;
  std::mt19937 rng(seed), hr(seed * 2654435761u + 7u);
  auto pct = [&](int p) { return int(rng() % 100) < p; };
  std::vector<uint64_t> mem(MEMW, 0x5a5a5a5a5a5a5a5aull);

  // ---- a frame's records and its reference ---------------------------------------
  uint64_t order_bad = 0;
  auto build = [&](std::string const &nm, frame_in &fi) -> bool {
    rframe f;
    if (!read_rframe(tris + "/" + nm + ".tri", f)) return false;
    texram tex; std::vector<uint32_t> dl(0x40000);
    gzFile gz = gzopen((dumps + "/" + nm + ".bin.gz").c_str(), "rb");
    bool ok = gz && gzseek(gz, 0x400000, SEEK_SET) == 0x400000 && gzread(gz, dl.data(), 0x100000) == 0x100000 &&
              gzseek(gz, 0x900000, SEEK_SET) == 0x900000 && gzread(gz, tex.t.data(), 0x800000) == 0x800000;
    if (gz) gzclose(gz);
    if (!ok) return false;
    for (auto &vp : f.vps) for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
    fi.tex = tex.t;
    rtl_opts o; o.oow = true;
    d25_opts d; d.band = 8; d.fog = true; d.cull = 2; d.trans = 0;
    if (mip) { d.mip = 1; d.lod = 1; }
    std::vector<uint32_t> lodb;
    if (mip && gm) {
      if (!geo_lod_load(dumps + "/" + nm + ".bin.gz", *gm)) return false;
      lodb = geo_lod_words(*gm);
      d.lodb = &lodb;
    }
    hook h; d.hook = &h;
    rimage img; d25_stats st;
    render_d25(f, tex, o, d, img, st);
    fi.name = nm; fi.tris = h.su.size(); fi.shaded = st.shaded;
    fi.want.resize(img.px.size());
    for (size_t i = 0; i < img.px.size(); i++) fi.want[i] = (img.px[i] >> 24) ? (0x01000000u | (img.px[i] & 0xffffff)) : 0;
    fi.sf = d25_scroll_fog(f, fi.sg);
    // the records, in the list's order; buf must rise within each (pass, alpha) list
    int64_t lastbuf[8]; for (auto &x : lastbuf) x = -1;
    int last_vp = -2;
    auto put = [&](uint32_t w, bool last) { fi.words.push_back({w, last}); };
    for (size_t i = 0; i < f.tris.size(); i++) {
      if (i >= h.drawn.size() || h.drawn[i] < 0) continue;
      rtri const &t = f.tris[i];
      rtl_tri const &r = h.su[size_t(h.drawn[i])];
      int const l = int(t.pass & 3) * 2 + int(t.alpha & 1);
      if (int64_t(t.buf) <= lastbuf[l]) order_bad++;
      lastbuf[l] = int64_t(t.buf);
      if (t.vp >= 0 && t.vp != last_vp) {
        rvp const &v = f.vps[size_t(t.vp)];
        put(v.pass & 3, false); put(v.node[0x20], false); put(v.node[0x22], false); put(v.node[0x23], false);
        put(v.node[0x25], false); put(fb(-v.proj[11]), true);
        last_vp = t.vp;
      }
      uint32_t const tex_on = (t.param & TP_TEXTURE) ? 1u : 0u;
      uint32_t const w0 = (t.pass & 3) | ((t.alpha & 1) << 2) | (tex_on << 4) | ((t.page & 1) << 5) | ((t.tex_w & 7) << 6) |
                          ((t.tex_h & 7) << 9) | ((t.fmt & 7) << 12) | ((t.param & 63) << 15) | ((t.trans & 63) << 21) | (3u << 27);
      uint32_t const w1 = (t.poly & 0xfffff) | ((t.tex_x & 63) << 20) | ((t.tex_y & 31) << 26);
      uint32_t const w3 = (t.vp >= 0 ? ((t.hdr[6] >> 11) & 31) : 0u) | (((t.hdr[6] >> 17) & 1) << 5);
      put(w0, false); put(w1, false); put(t.color & 0xffffff, false); put(w3, false);
      if (mip) {
        double dv; uint32_t bw = 0;
        if (!d25_lod_terms(f, t, dv, bw)) bw = 0;
        if (gm && tex_on) {                // render_d25 takes lodb's word where one is given
          geo_tex++;
          if (t.poly < lodb.size() && lodb[t.poly] != 0xffffffffu) { bw = lodb[t.poly]; geo_given++; }
        }
        put(bw, false);
      }
      for (int k = 0; k < 3; k++) { put(fb(r.fx[k]), false); put(fb(r.fy[k]), false); }
      put(fb(r.det), false);
      std::vector<int> pls;
      if (r.tex) pls = {0, 2, 3, 4}; else pls = {0, 1};
      for (size_t p = 0; p < pls.size(); p++) {
        int const m = pls[p];
        put(fb(r.pdx[m]), false); put(fb(r.pdy[m]), false); put(fb(r.ps[m]), p + 1 == pls.size());
      }
    }
    return true;
  };

  // texture RAM into DDR3, tiled (texel x % 4 a 16-bit lane of word TEX + {y / 4, x / 4, y % 4}),
  // between replays (the core uploads textures between frames)
  auto load_tex = [&](frame_in &fi) {
    for (uint32_t y = 0; y < 2048; y++)
      for (uint32_t x = 0; x < 2048; x += 4) {
        uint64_t w = 0;
        for (uint32_t i = 0; i < 4; i++) w |= uint64_t(fi.tex[size_t(y) * 2048 + x + i]) << (16 * i);
        mem[TEX + ((y >> 2) << 11) + ((x >> 2) << 2) + (y & 3)] = w;
      }
    fi.tex.clear(); fi.tex.shrink_to_fit();
  };

  // ---- the RTL and the ports ---------------------------------------------------------
  auto *t = new Vr3d_rast;
  t->rst = 1; t->i_valid = 0; t->f_begin = 0; t->f_end = 0;
  t->s_wacc = 0; t->s_rvalid = 0; t->s_ack = 0; t->t_ack = 0; t->t_rvalid = 0; t->l_wacc = 0; t->l_ack = 0;
  t->store_base = STORE; t->tex_base = TEX;
  for (int i = 0; i < 8; i++) { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }
  t->rst = 0;
  uint64_t cyc = 0, viol = 0, bad_addr = 0;
  auto violation = [&](char const *what) { if (viol++ < 20) std::printf("  %" PRIu64 ": port rule: %s\n", cyc, what); };
  auto ackd = [&](int lo, int hi, int p) { return (int(hr() % 100) < p) ? lo + int(hr() % unsigned(hi - lo + 1)) : 1 + int(hr() % 3); };

  // an m2_ddr3 port
  struct mport {
    char const *nm; uint64_t lo, hi;
    int st = 0;                    // 0 idle, 1 waiting the take, 2 write beats, 3 read beats due, 9 ack out
    bool we = false; uint64_t addr = 0; uint32_t blen = 0, beat = 0; int wait = 0;
    std::deque<uint64_t> due;      // read beats: the cycle each may come
    uint64_t rbytes = 0, wbytes = 0, reqs = 0, rbytes_f = 0, wbytes_f = 0, reqs_f = 0;
  };
  mport sp{"store", STORE, STORE + 0x400000}, lp{"layer", FB0, FB1 + 0x20000};
  // the texture port
  struct tline { uint64_t at, addr; };
  std::deque<tline> tq;            // lines acknowledged, in order
  int t_wait = -1; uint64_t t_addr0 = 0; uint32_t t_beat = 0; uint64_t tex_lines = 0, tex_lines_f = 0;

  uint64_t st_cache = 0, st_back = 0, st_store = 0, st_zb = 0, st_px = 0, st_run = 0;
  // one cycle: drive inputs, evaluate, account
  auto step = [&]() {
    // ---- store and layer ports: this cycle's outputs (decided from the state)
    auto drive = [&](mport &p, bool req, bool we, uint64_t addr, uint32_t blen, uint8_t &wacc, uint8_t &rvalid, uint8_t &ackv, bool rd_ok) {
      wacc = 0; rvalid = 0; ackv = 0;
      (void)req; (void)we; (void)addr; (void)blen; (void)rd_ok;
      if (p.st == 9) { ackv = 1; }
      if (p.st == 2 && !pct(busy)) wacc = 1;
      if (p.st == 3 && !p.due.empty() && p.due.front() <= cyc && !pct(gap)) rvalid = 1;
    };
    uint8_t s_wacc, s_rv, s_ackv, l_wacc, l_rv, l_ackv;
    drive(sp, t->s_req, t->s_we, t->s_addr, t->s_blen, s_wacc, s_rv, s_ackv, true);
    drive(lp, t->l_req, true, t->l_addr, t->l_blen, l_wacc, l_rv, l_ackv, false);
    t->s_wacc = s_wacc; t->s_ack = s_ackv; t->s_rvalid = s_rv;
    t->l_wacc = l_wacc; t->l_ack = l_ackv;
    if (s_rv) t->s_dout = mem[sp.addr + sp.beat];
    // ---- texture port
    bool tack = false, trv = false;
    if (t->t_req) {
      if (t_wait < 0) { t_wait = ackd(tack_lo, tack_hi, tack_p); t_addr0 = t->t_addr; }
      if (t->t_addr != t_addr0) violation("texture address changed before t_ack");
      if (tq.size() < maxrd && --t_wait <= 0) tack = true;
    }
    if (!tq.empty() && tq.front().at <= cyc && !pct(gap)) {
      trv = true;
      uint64_t const a = tq.front().addr + t_beat;
      if (a < TEX || a >= TEX + 0x100000) bad_addr++;
      t->t_dout = mem[a < MEMW ? a : 0];
    }
    t->t_ack = tack; t->t_rvalid = trv;
    t->clk = 0; t->eval();
    // ---- what the ports see this cycle
    auto see = [&](mport &p, bool req, bool we, uint64_t addr, uint32_t blen, uint64_t din, uint8_t be, bool wacc, bool rv, bool ackv) {
      if (p.st == 9) { p.st = 0; if (ackv) {} }
      else if (p.st == 0) {
        if (req && !ackv) {
          p.st = 1; p.we = we; p.addr = addr; p.blen = blen ? blen : 256; p.beat = 0; p.wait = ackd(ack_lo, ack_hi, ack_p);
          p.reqs++; p.reqs_f++;
          if (addr < p.lo || addr + p.blen > p.hi) { bad_addr++; if (bad_addr < 10) std::printf("  %s: address %06" PRIx64 " +%u outside\n", p.nm, addr, p.blen); }
        }
      } else {
        if (!req) violation("request dropped before ack");
        else if (p.we != we || p.addr != addr || p.blen != (blen ? blen : 256)) violation("request changed before ack");
      }
      if (p.st == 1 && --p.wait <= 0) {
        if (p.we) p.st = 2;
        else {
          uint64_t const l = uint64_t(lat_lo + int(hr() % unsigned(lat_hi - lat_lo + 1)));
          for (uint32_t k = 0; k < p.blen; k++) p.due.push_back(cyc + l + k);
          p.st = 3;
        }
      } else if (p.st == 2 && wacc) {
        uint64_t &w = mem[std::min<uint64_t>(p.addr + p.beat, MEMW - 1)];
        for (int b = 0; b < 8; b++) if ((be >> b) & 1) w = (w & ~(0xffull << (8 * b))) | (din & (0xffull << (8 * b)));
        p.wbytes += 8; p.wbytes_f += 8;
        if (++p.beat == p.blen) p.st = 9;
      } else if (p.st == 3 && rv) {
        p.due.pop_front(); p.rbytes += 8; p.rbytes_f += 8;
        if (++p.beat == p.blen) p.st = 9;
      }
    };
    see(sp, t->s_req, t->s_we, t->s_addr, t->s_blen, t->s_din, t->s_be, s_wacc, s_rv, s_ackv);
    see(lp, t->l_req, true, t->l_addr, t->l_blen, t->l_din, 0xff, l_wacc, false, l_ackv);
    if (tack) {
      uint64_t const l = uint64_t(lat_lo + int(hr() % unsigned(lat_hi - lat_lo + 1)));
      uint64_t at = cyc + l;
      if (!tq.empty()) at = std::max(at, tq.back().at);
      tq.push_back({at, t->t_addr}); tex_lines++; tex_lines_f++; t_wait = -1;
    }
    if (trv) { if (++t_beat == 4) { t_beat = 0; tq.pop_front(); } }
    if (g_px >= 0) {
      auto *rp = t->rootp;
      uint32_t const rc = rp->r3d_rast__DOT__rq_c;
      static long tlo = std::getenv("R3D_TLO") ? std::atol(std::getenv("R3D_TLO")) : -1, thi = std::getenv("R3D_THI") ? std::atol(std::getenv("R3D_THI")) : -1;
      if (long(cyc) >= tlo && long(cyc) <= thi)
        std::printf("    %" PRIu64 ": g%d tf%d fb%d fs%d zp%d tq%d zr%d zo%d h%d p%d r%d f1%d f2%d n%d tcr%d tco%d bk%d bi%d tb%d cb%d co%d cc%d q%d\n", cyc,
          rp->r3d_rast__DOT__g_valid, rp->r3d_rast__DOT__t_full, rp->r3d_rast__DOT__f_busy, rp->r3d_rast__DOT__f_svalid, rp->r3d_rast__DOT__za_pvalid,
          rp->r3d_rast__DOT__tq_n, rp->r3d_rast__DOT__zs_ready, rp->r3d_rast__DOT__zo_valid, rp->r3d_rast__DOT__h_v, rp->r3d_rast__DOT__p_v,
          rp->r3d_rast__DOT__r_v, rp->r3d_rast__DOT__f1_v, rp->r3d_rast__DOT__f2_v, rp->r3d_rast__DOT__n_valid, rp->r3d_rast__DOT__tc_iready,
          rp->r3d_rast__DOT__tc_ovalid, rp->r3d_rast__DOT__bk_n, rp->r3d_rast__DOT__b_in_v, rp->r3d_rast__DOT__tb_valid, rp->r3d_rast__DOT__cb_v,
          rp->r3d_rast__DOT__co_v, rp->r3d_rast__DOT__cc_v, rp->r3d_rast__DOT__quiet);
      if (std::getenv("R3D_BAND")) {
        unsigned const wa = unsigned(((g_py & 7) << 8) | (g_px >> 1)), wk = unsigned(g_px & 1);
        if (rp->r3d_rast__DOT__u_band__DOT__p0_v && rp->r3d_rast__DOT__u_band__DOT__p0_go)
          if (rp->r3d_rast__DOT__u_band__DOT__p0_eob || (rp->r3d_rast__DOT__u_band__DOT__p0_x == unsigned(g_px) && rp->r3d_rast__DOT__u_band__DOT__p0_row == unsigned(g_py & 7)))
            std::printf("    band %" PRIu64 ": p0 eob %d band %d x %d cur %d fbuf %d cst %d\n", cyc, rp->r3d_rast__DOT__u_band__DOT__p0_eob, rp->r3d_rast__DOT__u_band__DOT__p0_band,
                        rp->r3d_rast__DOT__u_band__DOT__p0_x, rp->r3d_rast__DOT__u_band__DOT__cur, rp->r3d_rast__DOT__u_band__DOT__fbuf, rp->r3d_rast__DOT__u_band__DOT__cst);
        for (int bi = 0; bi < 2; bi++)
          if (rp->r3d_rast__DOT__u_band__DOT__bwe[bi][wk] && rp->r3d_rast__DOT__u_band__DOT__bwa[bi][wk] == wa)
            std::printf("    band %" PRIu64 ": write buffer %d word %08X cur %d fbuf %d cst %d\n", cyc, bi, unsigned(rp->r3d_rast__DOT__u_band__DOT__bwd[bi][wk]),
                        rp->r3d_rast__DOT__u_band__DOT__cur, rp->r3d_rast__DOT__u_band__DOT__fbuf, rp->r3d_rast__DOT__u_band__DOT__cst);
      }
      if (rp->r3d_rast__DOT__eob) std::printf("    rtl %" PRIu64 ": eob band %d\n", cyc, rp->r3d_rast__DOT__eob_band);
      if (rp->r3d_rast__DOT__cc_v && int((rc >> 10) & 511) == g_px && int((rc >> 1) & 511) == g_py)
        std::printf("    rtl %" PRIu64 ": ok %d zw %d bl %d f %d rgb %06X\n", cyc, rp->r3d_rast__DOT__cc_ok, rp->r3d_rast__DOT__cc_zw,
                    rp->r3d_rast__DOT__cc_fbv, rp->r3d_rast__DOT__cc_fb, rp->r3d_rast__DOT__cc_rgb);
    }
    {
      auto *rp = t->rootp;
      st_cache += rp->r3d_rast__DOT__dbg_cache; st_back += rp->r3d_rast__DOT__dbg_back; st_store += rp->r3d_rast__DOT__dbg_store;
      st_zb += rp->r3d_rast__DOT__dbg_zb; st_px += rp->r3d_rast__DOT__dbg_px; st_run += rp->r3d_rast__DOT__dbg_run;
    }
    t->clk = 1; t->eval();
    cyc++;
  };

  // ---- frames ---------------------------------------------------------------------------
  uint64_t px_bad = 0, nfr = 0, npx = 0, nfail = 0;
  std::vector<uint64_t> fcyc, bcyc, sbytes, eobw, rd_f;
  std::deque<frame_in> pending;          // given to the RTL, waiting their done
  size_t next = 0;
  frame_in cur;
  size_t wi = 0;
  int fst = 0;                           // 0 none, 1 records going in, 2 f_end given, binning
  uint64_t bin_c0 = 0, w0 = 0, r0 = 0, l0 = 0, done_seen = 0;
  std::vector<uint64_t> wr_of;            // store bytes written binning each frame
  uint32_t fbk = 0;
  uint64_t tex0 = 0;
  uint64_t const limit = 400000000ull * std::max<size_t>(names.size(), 1);
  while ((next < names.size() || fst || !pending.empty()) && cyc < limit) {
    t->f_begin = 0; t->f_end = 0; t->i_valid = 0;
    if (fst == 0 && next < names.size() && pending.size() < 2 && t->f_ready) {
      bool ok = false;
      while (next < names.size() && !(ok = build(names[next], cur))) next++;
      if (ok) { t->f_begin = 1; fst = 1; wi = 0; bin_c0 = cyc; w0 = sp.wbytes; next++; }
    } else if (fst == 1) {
      if (wi < cur.words.size()) {
        bool const offer = !pct(igap);
        t->i_valid = offer; t->i_data = cur.words[wi].first; t->i_last = cur.words[wi].second;
      } else {
        t->fb_base = (fbk & 1) ? FB1 : FB0;
        t->sf_x0 = cur.sf ? uint32_t(cur.sg.x0) : 0; t->sf_x1 = cur.sf ? uint32_t(cur.sg.x1) : 0;
        t->sf_y0 = cur.sf ? uint32_t(cur.sg.y0) : 0; t->sf_y1 = cur.sf ? uint32_t(cur.sg.y1) : 0;
        t->sf_col = cur.sf ? cur.sg.col : 0; t->sf_out = cur.sf ? cur.sg.shown : 0;
        t->f_end = 1; fst = 2; fbk++;
        if (pending.empty()) load_tex(cur);
        pending.push_back(std::move(cur)); cur = frame_in();
      }
    } else if (fst == 2 && !t->f_binning) {
      bcyc.push_back(cyc - bin_c0); wr_of.push_back(sp.wbytes - w0); fst = 0;
    }
    t->clk = 0; t->eval();
    bool const took = t->i_valid && t->i_ready;
    step();
    if (took) wi++;
    if (t->done) {
      // the oldest pending frame is finished: its layer against the model
      frame_in &fi = pending.front();
      uint64_t const base = (done_seen & 1) ? FB1 : FB0;
      uint64_t b = 0;
      for (int y = 0; y < SCR_H; y++)
        for (int x = 0; x < SCR_W; x++) {
          uint64_t const q = mem[base + uint64_t(y) * 256 + uint64_t(x >> 1)];
          uint32_t const p = uint32_t((x & 1) ? q >> 32 : q) & 0x01ffffffu;
          uint32_t const w = fi.want[size_t(y) * SCR_W + size_t(x)];
          if (p != w) { if (b++ < 5) std::printf("  %s (%d,%d): rtl %08X model %08X\n", fi.name.c_str(), x, y, p, w); }
        }
      if (b) { std::printf("  %s: %" PRIu64 " pixels wrong\n", fi.name.c_str(), b); nfail++; }
      px_bad += b; npx += fi.shaded;
      fcyc.push_back(t->f_cycles);
      eobw.push_back(t->f_eobw);
      rd_f.push_back(sp.rbytes - r0); r0 = sp.rbytes;
      if (nfr < wr_of.size()) sbytes.push_back(wr_of[nfr] + rd_f.back());
      std::printf("FRAME %s tris %" PRIu64 " shaded %" PRIu64 " replay %u bin %" PRIu64 " swr %" PRIu64 " srd %" PRIu64 " eobw %u tex %" PRIu64 " lwr %" PRIu64 " wrong %" PRIu64 "\n",
                  fi.name.c_str(), fi.tris, fi.shaded, t->f_cycles, nfr < bcyc.size() ? bcyc[nfr] : 0, nfr < wr_of.size() ? wr_of[nfr] : 0,
                  rd_f.back(), t->f_eobw, tex_lines - tex0, lp.wbytes - l0, b);
      l0 = lp.wbytes;
      std::printf("STALL %s px %" PRIu64 " cache %" PRIu64 " back %" PRIu64 " store %" PRIu64 " zband %" PRIu64 " run %" PRIu64 "\n", fi.name.c_str(), st_px, st_cache, st_back, st_store, st_zb, st_run);
      st_cache = st_back = st_store = st_zb = st_px = st_run = 0;
      tex0 = tex_lines;
      nfr++; done_seen++;
      pending.pop_front();
      if (!pending.empty()) load_tex(pending.front());
    }
  }
  if (cyc >= limit) std::printf("  TIMEOUT: %zu frames pending, store write state %d\n", pending.size(), int(t->f_binning));
  auto pc = [](std::vector<uint64_t> v, int p) -> uint64_t {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return p >= 100 ? v.back() : v[std::min(v.size() - 1, v.size() * size_t(p) / 100)];
  };
  double const F = 1738530.0;   // cycles a 57.52 Hz frame at 100 MHz
  std::printf("r3d_rast: %" PRIu64 " frames, %" PRIu64 " shaded pixels, %" PRIu64 " pixels wrong in %" PRIu64 " frames; "
              "%" PRIu64 " port-rule violations, %" PRIu64 " bad addresses, %" PRIu64 " out of order; %" PRIu64 " cycles\n",
              nfr, npx, px_bad, nfail, viol, bad_addr, order_bad, cyc);
  std::printf("  replay cycles p50 %" PRIu64 " p99 %" PRIu64 " max %" PRIu64 " (p99 %.1f%% of a frame); binning p50 %" PRIu64 " p99 %" PRIu64
              "; end-of-band drain p99 %" PRIu64 "\n",
              pc(fcyc, 50), pc(fcyc, 99), pc(fcyc, 100), 100.0 * double(pc(fcyc, 99)) / F, pc(bcyc, 50), pc(bcyc, 99), pc(eobw, 99));
  std::printf("  store bytes a frame (binning + replay of the frame before) p50 %" PRIu64 " p99 %" PRIu64 " = %.1f MB/s at 57.52 Hz; "
              "store %" PRIu64 " requests; texture lines %" PRIu64 "\n",
              pc(sbytes, 50), pc(sbytes, 99), double(pc(sbytes, 99)) * 57.52 / 1e6, sp.reqs, tex_lines);
  std::printf("  totals: store read %" PRIu64 " written %" PRIu64 " bytes, layer written %" PRIu64 " (profile ack %d-%d@%d%% tack %d-%d@%d%% busy %d lat %d-%d gap %d maxrd %u)\n",
              sp.rbytes, sp.wbytes, lp.wbytes, ack_lo, ack_hi, ack_p, tack_lo, tack_hi, tack_p, busy, lat_lo, lat_hi, gap, maxrd);
  if (gm) std::printf("  geometry's B: %" PRIu64 " of %" PRIu64 " textured records given by the geometry model\n", geo_given, geo_tex);
  bool const fail = px_bad || viol || bad_addr || nfr != names.size();
  (void)out;
  return fail ? 1 : 0;
}
