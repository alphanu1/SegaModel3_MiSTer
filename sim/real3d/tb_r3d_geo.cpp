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
// r3d_geo against the reference model, triangle by triangle, every field. The
// RTL's word stream (polygon and viewport records, r3d_geo.sv's emitter,
// m2-notes section 15) is cut into the fan's triangles here, with MAME's buffer
// index counted per alpha buffer. The model's triangles that face away (its
// back-face test, R62) are left out unless --face-keep (FACE_DROP = 0); 1/w is
// the model's reciprocal of its w; the viewport records' words are read from
// the dump at each 'V' record's address.
//
//   Vr3d_geo --dumps DIR --vrom FILE [--frames N] [--first N] [--lat LO HI]
//            [--stall P] [--seed S] [--setup sm|cr|alg] [--cull] [--rows sm|mame|LIST]
//            [--step HEX] [--face-keep] [--lod]
//            [--hostile MASK] [--ack LO HI] [--busy PCT] [--gap PCT] [--maxrd K]
//            [--replay TRACE]
//
// --lod: the build has LOD_WORD = 1; each polygon record is 5 + 6 n words and
// its header word 4, the LOD word B, is checked against the model's.
//
// --setup-out: the build has SETUP_OUT = 1 (the set-up on the pool, m2-notes
// section 15): the RTL sends one set-up record a triangle, the polygon header
// with n = 3 (and the LOD word with --lod), then x1 y1 x2 y2 x3 y3, det and
// pdx, pdy, ps of 1/w, u/w, v/w, shade (textured, 23 words) or 1/w, shade (17).
// Each is checked word by word against rtl_tri::setup (r3d_rast_rtl.cpp, depth
// on 1/w, flush to zero) on the model's triangle, the header fields as above.
// A triangle whose x, y or 1/w is not finite is left out of the model's list,
// as the set-up port (and m3v_setup) drops it.
//
// --rows gives the model's D-19 rows (m2-notes section 3) and must match the
// RTL build's SM_* parameters: sm (default, every row), mame, or a list of
// flat, fixed, double, light, trans, ctab, cull (flat: row 18 as built, the (2b + 1) / 255
// normals; face adds the face normals of flat polygons, a model switch only;
// cull the back-face test, SM_CULL).
//
// Each memory port answers a request after 1-3 cycles and returns its words
// LO..HI cycles later (random per request, in order, one word a cycle); up to
// four requests may be in flight on the polygon RAM and VROM ports. P is the
// chance, in percent, that the triangle consumer is not ready. Reports frames
// that match exactly, mismatches per field, cycles per frame, and words and
// requests per port.
//
// --hostile MASK (1 culling, 2 polygon RAM, 4 VROM) gives those ports a slow,
// irregular memory instead, with its own random stream (study R70): a request
// is acknowledged ACK LO..HI cycles after it is raised (default 1-3), and on
// any cycle of that wait with chance BUSY (percent) it is held one cycle
// more; with --maxrd at most K requests are held at once (acknowledged and not
// all returned), so a new one waits for the oldest to drain, as the viewer's
// one-request DDR3 reader does. Words come back in order LO..HI cycles after
// the acknowledge, and each cycle a due word is held back with chance GAP.
// The other ports keep the default model.
//
// --replay TRACE drives every port's acknowledge and valid, out_ready and
// start from a trace of one frame, a byte a cycle (sim/viewer/tb_m3v.cpp,
// M3V_GTRACE), so a timing seen in a whole-core bench runs here alone. Use it
// with --frames 1 and --first for that frame.

#include "Vr3d_geo.h"
#include "Vr3d_geo___024root.h"
#include "verilated.h"
#include "r3d_geo.h"
#include "r3d_ucode.h"
#ifdef GEO_SETUP_OUT
#include "r3d_rast.h"   // rtl_tri::setup, the set-up records' reference
#endif
#include <zlib.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

static bool load_dump(std::string const &fn, r3d_mem &m) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  bool ok = gzread(g, m.cull_lo.data(), 0x400000) == 0x400000 && gzread(g, m.cull_hi.data(), 0x100000) == 0x100000 &&
            gzread(g, m.poly.data(), 0x400000) == 0x400000;
  gzclose(g);
  return ok;
}

struct port {
  struct beatq { uint64_t at; uint32_t d; bool last; };
  std::deque<beatq> q;
  int ack_wait = -1;
  uint64_t last_at = 0;
  uint64_t reqs = 0, words = 0;
  unsigned held = 0;    // --hostile: requests acknowledged and not all returned
};

#ifdef MS_MON
static uint64_t n_ready_pend = 0, n_ready_pend_w = 0, n_wait = 0;
#endif
static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
// a 'T' record of the model with the LOD words (d, B) after its other words
static bool geo_lod_ok(record const &r) { return r.w.size() >= 39; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string dumps, vrom;
  long max_frames = -1, first = 0;
  int lat_lo = 12, lat_hi = 20, stall = 20;
  unsigned seed = 1;
  bool setup_cr = false, cull = false, frustum_sm = false, face_keep = false, lod = false, setup_out = false;
  uint32_t step = 0x10;
  std::string rows = "sm";
  unsigned hostile = 0, h_maxrd = 0;
  std::vector<uint8_t> replay;
  bool replaying = false;
  int h_ack_lo = 1, h_ack_hi = 3, h_busy = 0, h_gap = 0;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--dumps") dumps = nx();
    else if (a == "--vrom") vrom = nx();
    else if (a == "--frames") max_frames = std::atol(nx().c_str());
    else if (a == "--first") first = std::atol(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--setup") { std::string const v = nx(); setup_cr = v == "cr"; frustum_sm = v == "sm"; }
    else if (a == "--cull") cull = true;
    else if (a == "--rows") rows = nx();
    else if (a == "--step") step = uint32_t(std::strtoul(nx().c_str(), nullptr, 16));
    else if (a == "--face-keep") face_keep = true;
    else if (a == "--lod") lod = true;
    else if (a == "--setup-out") {
#ifdef GEO_SETUP_OUT
      setup_out = true;
#else
      std::fprintf(stderr, "--setup-out needs the bench built with -DGEO_SETUP_OUT (r3d_rast.cpp, r3d_rast_rtl.cpp)\n"); return 2;
#endif
    }
    else if (a == "--hostile") hostile = unsigned(std::strtoul(nx().c_str(), nullptr, 0));
    else if (a == "--ack") { h_ack_lo = std::max(1, std::atoi(nx().c_str())); h_ack_hi = std::max(h_ack_lo, std::atoi(nx().c_str())); }
    else if (a == "--busy") h_busy = std::atoi(nx().c_str());
    else if (a == "--gap") h_gap = std::atoi(nx().c_str());
    else if (a == "--maxrd") h_maxrd = unsigned(std::atoi(nx().c_str()));
    else if (a == "--replay") {
      std::FILE *f = std::fopen(nx().c_str(), "rb");
      if (!f) { std::fprintf(stderr, "no trace\n"); return 2; }
      int c; while ((c = std::fgetc(f)) != EOF) replay.push_back(uint8_t(c));
      std::fclose(f); replaying = true;
    }
  }

  r3d_mem mem;
  {
    std::FILE *f = std::fopen(vrom.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "need --vrom\n"); return 2; }
    size_t n = std::fread(mem.vrom.data(), 4, mem.vrom.size(), f); (void)n; std::fclose(f);
  }
  std::vector<std::string> files;
  for (auto const &e : fs::directory_iterator(dumps)) if (e.path().extension() == ".gz") files.push_back(e.path().string());
  std::sort(files.begin(), files.end());
  if (first > 0) files.erase(files.begin(), files.begin() + std::min<long>(first, long(files.size())));
  if (max_frames >= 0 && size_t(max_frames) < files.size()) files.resize(size_t(max_frames));

  std::mt19937 rng(seed);
  std::mt19937 hrng(seed * 2654435761u + 12345u);   // the hostile ports' own stream
  Vr3d_geo *t = new Vr3d_geo;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; t->start = 0; t->cull_ack = 0; t->pram_ack = 0; t->vrom_ack = 0; t->step15 = step >= 0x15;
  t->cull_rvalid = 0; t->pram_rvalid = 0; t->vrom_rvalid = 0; t->out_ready = 0;
  tick(); tick(); t->rst = 0;

  static char const *fname[] = {"pass", "alpha", "index", "poly", "x", "y", "1/w", "u/w", "v/w", "shade",
                                "tex_en", "page", "tex_x", "tex_y", "tex_w", "tex_h", "tex_fmt", "param", "transparency", "colour",
                                "inverted copy", "faces away", "fog", "translucency layer", "viewport record", "LOD word",
                                "set-up x", "set-up y", "det", "pdx", "pdy", "ps", "set-up record length"};
  uint64_t su_tris = 0, su_tex = 0, su_flat = 0, su_nonfin = 0, su_nonfin_w = 0, su_rows0 = 0;
  uint64_t face_single = 0, face_back = 0, face_dbl = 0, face_zero = 0, vrecs = 0, vrec_bad = 0, tris_culled = 0;
  uint32_t fsubn_last = 0; uint64_t fsubn_total = 0;
  std::map<std::string, uint64_t> fbad;
  uint64_t frames_ok = 0, tris_total = 0, count_bad = 0;
  uint64_t port_open = 0, port_open_f = 0;   // frames whose busy fell with a port open
  std::vector<uint64_t> cycles;
  std::vector<uint64_t> pw[3], pr[3];
  int shown = 0;
  std::map<int, uint64_t> st_hist;
  bool prof = std::getenv("R3D_PROF") != nullptr;
  uint64_t const prof_min = std::getenv("R3D_PROF_MIN") ? std::strtoull(std::getenv("R3D_PROF_MIN"), nullptr, 10) : 0;
  std::map<int, uint64_t> fh;
  uint32_t subn_last = 0, iss_last = 0, lop_last = 0, hi_last = 0;
  std::vector<uint64_t> iss_f, lop_f, hi_f;
  uint64_t subn_frames = 0, subn_total = 0;

  for (size_t fi = 0; fi < files.size(); fi++) {
    if (!load_dump(files[fi], mem)) { std::fprintf(stderr, "bad dump\n"); return 1; }
    // reference
    std::vector<record> ref_all, ref;
    {
      r3d_geo geo(mem, arith::rtl, false);
      geo.setup_cr = setup_cr;
      geo.frustum_sm = frustum_sm;
      geo.cull = cull;
      geo.rows(rows == "sm");
      if (rows != "sm" && rows != "mame") {
        auto has = [&](char const *k) { return rows.find(k) != std::string::npos; };
        geo.sm_n255 = has("flat"); geo.sm_flat = has("face"); geo.sm_fixed = has("fixed"); geo.sm_double = has("double");
        geo.sm_light = has("light"); geo.sm_trans = has("trans"); geo.sm_ctab = has("ctab"); geo.sm_cull = has("cull");
      }
      geo.step = step;
      geo.lod_word = lod;
      geo.frame(uint32_t(fi));
      geo.out.records(ref_all);
      face_single += geo.st.face_single; face_back += geo.st.face_back; face_dbl += geo.st.face_vs_double; face_zero += geo.st.face_zero;
    }
    // the model's triangles as the RTL sends them: without those facing away
    // (FACE_DROP), 1/w for z and w, the header's fog and layer bits
    std::vector<record> precs;
    for (auto &r : ref_all) if (r.tag == 'P') precs.push_back(r);
    {
      fp_unit rq;
      uint32_t ro = 0, ra = 0;
      for (auto &r : ref_all) if (r.tag == 'T') {
        bool const culled = r.w.size() >= 37 && (r.w[35] & 2);
        if (culled && !face_keep) { tris_culled++; continue; }
        record e; e.tag = 'T';
        e.w.assign(40, 0);   // 37-39: the model's w (the set-up's reference, --setup-out)
        e.w[0] = r.w[0]; e.w[1] = r.w[1]; e.w[2] = r.w[1] ? ra++ : ro++; e.w[3] = r.w[3];
        for (int v = 0; v < 3; v++) {
          uint32_t const *q = &r.w[size_t(4 + 7 * v)];
          uint32_t *o = &e.w[size_t(4 + 6 * v)];
          o[0] = q[0]; o[1] = q[1]; o[2] = fbits(rq.recip(bitsf(q[3]))); o[3] = q[4]; o[4] = q[5]; o[5] = q[6];
          e.w[size_t(37 + v)] = q[3];
        }
        for (int k = 0; k < 10; k++) e.w[size_t(22 + k)] = r.w[size_t(25 + k)];
        e.w[32] = r.w.size() >= 37 ? (r.w[35] & 1) : 0;
        e.w[33] = r.w.size() >= 37 ? ((r.w[35] >> 1) & 1) : 0;
        uint32_t const h6 = r.w[3] < precs.size() ? precs[r.w[3]].w[9] : 0;
        e.w[34] = (h6 >> 11) & 31; e.w[35] = (h6 >> 17) & 1;
        e.w[36] = (lod && geo_lod_ok(r)) ? r.w.back() : 0;
        ref.push_back(e);
      }
    }
    // SETUP_OUT: the set-up records the RTL sends, from rtl_tri::setup on each
    // triangle; a triangle with a non-finite x, y or 1/w word is not sent
    std::vector<std::vector<uint32_t>> sref, sgot;
#ifdef GEO_SETUP_OUT
    if (setup_out) {
      std::vector<record> keep;
      uint32_t ro = 0, ra = 0;
      rtl_opts o; o.oow = true; o.ftz = true;
      for (auto &e : ref) {
        bool nf = false;
        for (int v = 0; v < 3; v++) for (int k = 0; k < 3; k++) if (((e.w[size_t(4 + 6 * v + k)] >> 23) & 0xff) == 0xff) nf = true;
        if (nf) { su_nonfin++; continue; }
        e.w[2] = e.w[1] ? ra++ : ro++;
        rtri t{};
        for (int v = 0; v < 3; v++) {
          uint32_t const *q = &e.w[size_t(4 + 6 * v)];
          // the model's w: rtl_tri::setup forms 1/w as 1.0f / w
          t.x[v] = bf(q[0]); t.y[v] = bf(q[1]); t.w[v] = bf(e.w[size_t(37 + v)]);
          { float const iw = 1.0f / t.w[v]; uint32_t u; std::memcpy(&u, &iw, 4); if (u != q[2]) su_nonfin_w++; }
          t.p0[v] = bf(q[3]); t.p1[v] = bf(q[4]); t.p2[v] = bf(q[5]);
        }
        t.param = e.w[29]; t.alpha = e.w[1];
        rtl_tri r;
        bool const rows = r.setup(t, kind_of(t), o);
        if (!rows) su_rows0++;
        std::vector<uint32_t> w;
        auto fb = [](float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; };
        for (int v = 0; v < 3; v++) { w.push_back(fb(r.fx[v])); w.push_back(fb(r.fy[v])); }
        w.push_back(fb(r.det));
        std::vector<int> pls = r.tex ? std::vector<int>{0, 2, 3, 4} : std::vector<int>{0, 1};
        for (int pp : pls) { w.push_back(fb(r.pdx[pp])); w.push_back(fb(r.pdy[pp])); w.push_back(fb(r.ps[pp])); }
        su_tris++; if (r.tex) su_tex++; if (r.flat) su_flat++;
        sref.push_back(w);
        keep.push_back(e);
      }
      ref.swap(keep);
    }
#endif
    // the viewport records the RTL sends: one a 'V' record, words from the dump
    std::vector<std::array<uint32_t, 6>> vref, vgot;
    for (auto &r : ref_all) if (r.tag == 'V') {
      uint32_t const a = r.w[1];
      auto W = [&](uint32_t k) -> uint32_t {
        uint32_t const x = a + k;
        if (x & 0x800000) { uint32_t o = x - 0x800000; return (x < 0x840000 && o < 0x40000) ? mem.cull_hi[o] : 0; }
        return x < 0x100000 ? mem.cull_lo[x] : 0;
      };
      vref.push_back({r.w[0] & 3, W(0x20), W(0x22), W(0x23), W(0x25), fbits(-bitsf(r.w[9 + 11]))});
    }

    port P[3];  // cull, pram, vrom
    auto mem_word = [&](int pi, uint32_t a) -> uint32_t {
      if (pi == 0) {
        if (a & 0x800000) { uint32_t o = a - 0x800000; return o < 0x40000 ? mem.cull_hi[o] : 0xDEADBEEFu; }
        return a < 0x100000 ? mem.cull_lo[a] : 0xDEADBEEFu;
      }
      if (pi == 1) return a < 0x100000 ? mem.poly[a] : replaying ? 0u : 0xDEADBEEFu;
      return a < 0x1000000 ? mem.vrom[a] : 0xDEADBEEFu;
    };
    std::vector<std::array<uint32_t, 37>> got;
    std::vector<uint32_t> rec;
    uint32_t cnt_o = 0, cnt_a = 0;
    uint64_t rec_bad = 0;
    // one polygon record into its fan's triangles
    auto take = [&]() {
      uint32_t const w0 = rec.size() > 0 ? rec[0] : 0, w1 = rec.size() > 1 ? rec[1] : 0;
      uint32_t const nv = w0 >> 27;
      if (nv == 0) {   // a viewport record
        if (rec.size() != 6) { rec_bad++; rec.clear(); return; }
        vgot.push_back({rec[0], rec[1], rec[2], rec[3], rec[4], rec[5]});
        rec.clear(); return;
      }
      size_t const hw = lod ? 5 : 4;
      if (setup_out) {
        // one set-up record: the header, then 19 (textured) or 13 words
        size_t const nw = ((w0 >> 4) & 1) ? 19 : 13;
        if (nv != 3 || rec.size() != hw + nw) {
          rec_bad++; fbad[fname[32]]++;
          if (shown < 12) { shown++; std::printf("set-up record: n %u, %zu words\n", nv, rec.size()); }
          rec.clear(); return;
        }
        std::array<uint32_t, 37> r{};
        uint32_t const w3 = rec[3];
        uint32_t const alpha = (w0 >> 2) & 1;
        r[0] = w0 & 3; r[1] = alpha; r[2] = alpha ? cnt_a++ : cnt_o++; r[3] = w1 & 0xfffff;
        r[22] = (w0 >> 4) & 1; r[23] = (w0 >> 5) & 1; r[24] = (w1 >> 20) & 0x3f; r[25] = (w1 >> 26) & 0x1f;
        r[26] = (w0 >> 6) & 7; r[27] = (w0 >> 9) & 7; r[28] = (w0 >> 12) & 7; r[29] = (w0 >> 15) & 0x3f;
        r[30] = (w0 >> 21) & 0x3f; r[31] = rec[2]; r[32] = (w0 >> 3) & 1; r[33] = (w3 >> 6) & 1;
        r[34] = w3 & 31; r[35] = (w3 >> 5) & 1;
        r[36] = lod ? rec[4] : 0;
        if (w3 >> 7) rec_bad++;
        got.push_back(r);
        sgot.push_back(std::vector<uint32_t>(rec.begin() + long(hw), rec.end()));
        rec.clear(); return;
      }
      if (rec.size() != hw + 6 * size_t(nv) || nv < 3) { rec_bad++; rec.clear(); return; }
      uint32_t const w3 = rec[3];
      for (uint32_t i = 2; i < nv; i++) {
        std::array<uint32_t, 37> r{};
        uint32_t const alpha = (w0 >> 2) & 1;
        r[0] = w0 & 3; r[1] = alpha; r[2] = alpha ? cnt_a++ : cnt_o++; r[3] = w1 & 0xfffff;
        uint32_t const vs[3] = {0, i - 1, i};
        for (int v = 0; v < 3; v++) for (int k = 0; k < 6; k++) r[size_t(4 + 6 * v + k)] = rec[hw + 6 * vs[v] + size_t(k)];
        r[22] = (w0 >> 4) & 1; r[23] = (w0 >> 5) & 1; r[24] = (w1 >> 20) & 0x3f; r[25] = (w1 >> 26) & 0x1f;
        r[26] = (w0 >> 6) & 7; r[27] = (w0 >> 9) & 7; r[28] = (w0 >> 12) & 7; r[29] = (w0 >> 15) & 0x3f;
        r[30] = (w0 >> 21) & 0x3f; r[31] = rec[2]; r[32] = (w0 >> 3) & 1; r[33] = (w3 >> 6) & 1;
        r[34] = w3 & 31; r[35] = (w3 >> 5) & 1;
        r[36] = lod ? rec[4] : 0;
        if (w3 >> 7) rec_bad++;
        got.push_back(r);
      }
      rec.clear();
    };
    uint64_t cyc = 0;
    uint64_t replay_bad = 0;
    if (!replaying) { t->start = 1; tick(); t->start = 0; }
    static uint64_t const maxcyc = std::getenv("R3D_MAXCYC") ? std::strtoull(std::getenv("R3D_MAXCYC"), nullptr, 10) : 200000000ull;
    while ((t->busy || cyc < 4 || (replaying && cyc < replay.size())) && cyc < maxcyc) {
      // drive memory
      uint8_t req[3] = {t->cull_req, t->pram_req, t->vrom_req};
      uint32_t addr[3] = {t->cull_addr, t->pram_addr, t->vrom_addr};
      uint32_t len[3] = {t->cull_len, t->pram_len, t->vrom_len};
      uint8_t ack[3] = {0, 0, 0}, rv[3] = {0, 0, 0};
      uint32_t rd[3] = {0, 0, 0};
      if (replaying) t->start = cyc < replay.size() ? (replay[cyc] >> 7) & 1 : 0;
      for (int pi = 0; pi < 3; pi++) {
        port &p = P[pi];
        if (replaying) {
          uint8_t const b = cyc < replay.size() ? replay[cyc] : 0;
          if ((b >> (2 * pi)) & 1) {
            if (!req[pi] && replay_bad++ < 10) std::printf("replay: port %d acknowledged at %" PRIu64 " with no request\n", pi, cyc);
            ack[pi] = 1; p.reqs++;
            for (uint32_t k = 0; k <= len[pi]; k++) { p.q.push_back({0, mem_word(pi, addr[pi] + k), k == len[pi]}); p.words++; }
          }
          if ((b >> (2 * pi + 1)) & 1) {
            if (p.q.empty()) { if (replay_bad++ < 10) std::printf("replay: port %d word at %" PRIu64 " with none due\n", pi, cyc); }
            else { rv[pi] = 1; rd[pi] = p.q.front().d; p.q.pop_front(); }
          }
          continue;
        }
        if ((hostile >> pi) & 1) {
          if (req[pi]) {
            if (p.ack_wait < 0) p.ack_wait = h_ack_lo + int(hrng() % unsigned(h_ack_hi - h_ack_lo + 1));
            bool const hold = (int(hrng() % 100) < h_busy) || (h_maxrd && p.held >= h_maxrd);
            if (!hold && --p.ack_wait == 0) {
              ack[pi] = 1; p.ack_wait = -1; p.reqs++; p.held++;
              uint64_t start_at = std::max<uint64_t>(cyc + uint64_t(lat_lo + int(hrng() % unsigned(lat_hi - lat_lo + 1))), p.last_at + 1);
              for (uint32_t k = 0; k <= len[pi]; k++) { p.q.push_back({start_at + k, mem_word(pi, addr[pi] + k), k == len[pi]}); p.words++; }
              p.last_at = start_at + len[pi];
            }
          }
          if (!p.q.empty() && p.q.front().at <= cyc && int(hrng() % 100) >= h_gap) {
            rv[pi] = 1; rd[pi] = p.q.front().d;
            if (p.q.front().last) p.held--;
            p.q.pop_front();
          }
          continue;
        }
        if (req[pi]) {
          if (p.ack_wait < 0) p.ack_wait = int(rng() % 3) + 1;
          if (--p.ack_wait == 0) {
            ack[pi] = 1; p.ack_wait = -1; p.reqs++;
            uint64_t start_at = std::max<uint64_t>(cyc + uint64_t(lat_lo + int(rng() % unsigned(lat_hi - lat_lo + 1))), p.last_at + 1);
            for (uint32_t k = 0; k <= len[pi]; k++) { p.q.push_back({start_at + k, mem_word(pi, addr[pi] + k), k == len[pi]}); p.words++; }
            p.last_at = start_at + len[pi];
          }
        }
        if (!p.q.empty() && p.q.front().at <= cyc) { rv[pi] = 1; rd[pi] = p.q.front().d; p.q.pop_front(); }
      }
      t->cull_ack = ack[0]; t->pram_ack = ack[1]; t->vrom_ack = ack[2];
      t->cull_rvalid = rv[0]; t->cull_rdata = rd[0];
      t->pram_rvalid = rv[1]; t->pram_rdata = rd[1];
      t->vrom_rvalid = rv[2]; t->vrom_rdata = rd[2];
      t->out_ready = replaying ? ((cyc < replay.size() ? replay[cyc] >> 6 : 1) & 1) : (int(rng() % 100) >= stall) ? 1 : 0;
      t->eval();
      if (t->out_valid && t->out_ready) {
        rec.push_back(t->out_data);
        if (t->out_last) take();
      }
#ifdef FACE_MON
      // debug (R3D_FACEDBG): the facing test's inputs, matrix writes, starts and results
      static bool const facedbg = std::getenv("R3D_FACEDBG") != nullptr;
      if (facedbg) {
        static uint64_t nv = 0;
        if (t->rootp->r3d_geo__DOT__f_vv) std::printf("RF %lu %d%d\n", (unsigned long)nv++, t->rootp->r3d_geo__DOT__f_gt, t->rootp->r3d_geo__DOT__f_lt);
        static uint32_t fw[6]; static uint64_t nc = 0;
        if (t->rootp->r3d_geo__DOT__f_twe && !t->rootp->r3d_geo__DOT__f_thold)
          std::printf("RT %d %d %d %08X\n", t->rootp->r3d_geo__DOT__tbuf_m, t->rootp->r3d_geo__DOT__ft_row, t->rootp->r3d_geo__DOT__ft_col,
                      t->rootp->r3d_geo__DOT__x_rdata[t->rootp->r3d_geo__DOT__ft_col]);
        if (t->rootp->r3d_geo__DOT__f_commit) std::printf("RC mb %d\n", t->rootp->r3d_geo__DOT__tbuf_d);
        {
          auto *rp = t->rootp;
          bool const bnd = !rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__run || (rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__r == 3 && rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__s == 6);
          int const cpp = rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__cp;
          if (bnd && rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__pend && ((rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__tv >> ((rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__mbs >> cpp) & 1)) & 1))
            std::printf("RS slot %d mb %d tv %d\n", cpp, (rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__mbs >> cpp) & 1, rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__tv);
          auto *fu = rp;
          if (fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__wv || fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__wn || fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__wd)
            std::printf("RW %c sum %08X\n", fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__wv ? 'V' : fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__wn ? 'N' : 'D',
                        fu->r3d_geo__DOT__g_face__DOT__u_face__DOT__sum);
          static int ltv = 0;
          if (rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__tv != ltv) { ltv = rp->r3d_geo__DOT__g_face__DOT__u_face__DOT__tv; std::printf("RV tv %d\n", ltv); }
        }
        if (t->rootp->r3d_geo__DOT__f_we && !t->rootp->r3d_geo__DOT__f_hold) {
          fw[t->rootp->r3d_geo__DOT__f_word % 6] = t->rootp->r3d_geo__DOT__f_d;
          std::printf("RA slot %d word %d %06X full %X dst %d\n", t->rootp->r3d_geo__DOT__g_face__DOT__u_face__DOT__fp, t->rootp->r3d_geo__DOT__f_word,
                      t->rootp->r3d_geo__DOT__f_d, t->rootp->r3d_geo__DOT__g_face__DOT__u_face__DOT__full, int(t->rootp->r3d_geo__DOT__dst));
        }
        if (t->rootp->r3d_geo__DOT__f_commit) std::printf("RI %lu %06X %06X %06X n %06X %06X %06X\n", (unsigned long)nc++, fw[0], fw[1], fw[2], fw[3], fw[4], fw[5]);
      }
#endif
#ifdef MS_MON
      {
        // debug: the model stream started with a request of the last model still waiting
        auto *rp = t->rootp;
        static uint64_t n_ss = 0, n_sd = 0;
        if (rp->r3d_geo__DOT__ms_start && rp->r3d_geo__DOT__ms_ready && (t->pram_req || t->vrom_req) && n_ss++ < 20)
          std::printf("MS start at %" PRIu64 " with a request waiting (pram %d vrom %d)\n", cyc, t->pram_req, t->vrom_req);
        if (rp->r3d_geo__DOT__ms_stop && (t->pram_req || t->vrom_req) && n_sd++ < 20)
          std::printf("MS stop at %" PRIu64 " with a request waiting, outst %d\n", cyc, rp->r3d_geo__DOT__u_ms__DOT__outst);
        bool const rdy = rp->r3d_geo__DOT__ms_ready, pend = (t->pram_req && !rp->r3d_geo__DOT__u_ms__DOT__c_issued) || t->vrom_req;
        bool const dwait = rp->r3d_geo__DOT__dst == 0 && rp->r3d_geo__DOT__d_req != rp->r3d_geo__DOT__d_ack;
        if (rdy && pend) { n_ready_pend++; if (dwait) n_ready_pend_w++; }
        if (dwait && !rdy) n_wait++;
        static long const w0 = std::getenv("R3D_MSWIN") ? std::atol(std::getenv("R3D_MSWIN")) : -1;
        if (w0 >= 0 && cyc + 3000 >= uint64_t(w0) && cyc <= uint64_t(w0) + 60)
          std::printf("  %" PRIu64 ": req %d addr %06X ack %d rv %d | start %d stop %d active %d draining %d outst %d count %d next %06X dst %d dwait %d mst %d\n", cyc,
                      t->pram_req, t->pram_addr, t->pram_ack, t->pram_rvalid, rp->r3d_geo__DOT__ms_start, rp->r3d_geo__DOT__ms_stop,
                      rp->r3d_geo__DOT__u_ms__DOT__active, rp->r3d_geo__DOT__u_ms__DOT__draining, rp->r3d_geo__DOT__u_ms__DOT__outst,
                      rp->r3d_geo__DOT__u_ms__DOT__count, rp->r3d_geo__DOT__u_ms__DOT__next, int(rp->r3d_geo__DOT__dst), int(dwait), int(rp->r3d_geo__DOT__mst));
      }
#endif
      static bool const dumpvp = std::getenv("R3D_DUMPVP") != nullptr;
      if (dumpvp) {
        // debug: the first model set-up's broadcast matrices and lane registers
        static int st_ = 0;
        auto *rp = t->rootp;
        if (st_ == 0 && rp->r3d_geo__DOT__vk_busy && rp->r3d_geo__DOT__vk_entry == r3d_uc::K_MODEL) st_ = 1;
        if (st_ == 1 && !rp->r3d_geo__DOT__vk_busy) {
          st_ = 2;
          auto bank = [&](int b, int e) -> uint32_t {
            switch (b) {
              case 0: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__0__KET____DOT__u_m0__DOT__g_m10k__DOT__m[e];
              case 1: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__1__KET____DOT__u_m0__DOT__g_m10k__DOT__m[e];
              case 2: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__2__KET____DOT__u_m0__DOT__g_m10k__DOT__m[e];
              default: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__3__KET____DOT__u_m0__DOT__g_m10k__DOT__m[e];
            }
          };
          auto reg = [&](int l, int r) -> uint32_t {
            switch (l) {
              case 0: return rp->r3d_geo__DOT__u_vpu__DOT__g_lane__BRA__0__KET____DOT__u_rfa__DOT__g_m10k__DOT__m[r];
              case 1: return rp->r3d_geo__DOT__u_vpu__DOT__g_lane__BRA__1__KET____DOT__u_rfa__DOT__g_m10k__DOT__m[r];
              case 2: return rp->r3d_geo__DOT__u_vpu__DOT__g_lane__BRA__2__KET____DOT__u_rfa__DOT__g_m10k__DOT__m[r];
              default: return rp->r3d_geo__DOT__u_vpu__DOT__g_lane__BRA__3__KET____DOT__u_rfa__DOT__g_m10k__DOT__m[r];
            }
          };
          for (int i = 0; i < 4; i++) std::printf("BC_TOP row %d: %08X %08X %08X %08X\n", i, bank(0, r3d_uc::R_BC_TOP + i), bank(1, r3d_uc::R_BC_TOP + i), bank(2, r3d_uc::R_BC_TOP + i), bank(3, r3d_uc::R_BC_TOP + i));
          for (int i = 0; i < 4; i++) std::printf("BC_VP row %d: %08X %08X %08X %08X\n", i, bank(0, r3d_uc::R_BC_VP + i), bank(1, r3d_uc::R_BC_VP + i), bank(2, r3d_uc::R_BC_VP + i), bank(3, r3d_uc::R_BC_VP + i));
          for (int i = 0; i < 4; i++) std::printf("CO %d: %08X %08X %08X %08X\n", i, reg(0, r3d_uc::R_CO + i), reg(1, r3d_uc::R_CO + i), reg(2, r3d_uc::R_CO + i), reg(3, r3d_uc::R_CO + i));
          for (int i = 0; i < 4; i++) std::printf("PR %d: %08X %08X %08X %08X\n", i, reg(0, r3d_uc::R_PR + i), reg(1, r3d_uc::R_PR + i), reg(2, r3d_uc::R_PR + i), reg(3, r3d_uc::R_PR + i));
        }
      }
      if (prof) {
        auto *rp = t->rootp;
        int ms = int(rp->r3d_geo__DOT__mst);
        // main waiting for a chunk: 900 + 1 (decoder has none) + 2 (vertex store full)
        if (ms == 50) ms = 900 + (rp->r3d_geo__DOT__dq_cnt == 0 ? 1 : 0) + (rp->r3d_geo__DOT__cq_cnt >= 3 ? 2 : 0);
        fh[ms]++;
        fh[1000 + int(rp->r3d_geo__DOT__pst)]++;
        if (rp->r3d_geo__DOT__vk_busy) fh[2000 + int(rp->r3d_geo__DOT__vk_entry)]++;
        else fh[3000]++;
        fh[4000 + int(rp->r3d_geo__DOT__dst)]++;
      }
      tick();
      cyc++;
    }
    {
      // busy low must leave every port quiet (no request waiting, no word due): the
      // next frame's ports start empty here, as an owner may hand them on
      unsigned due = 0; for (int pi = 0; pi < 3; pi++) due += unsigned(P[pi].q.size());
      if (!replaying && (t->cull_req || t->pram_req || t->vrom_req || due)) {
        port_open++;
        if (port_open <= 10) std::printf("%s: busy fell with requests %d%d%d waiting and %u words due\n", fs::path(files[fi]).filename().c_str(),
                                         t->cull_req, t->pram_req, t->vrom_req, due);
      }
    }
    if (cyc >= maxcyc) {
      // the frame did not finish: where each machine stands
      auto *rp = t->rootp;
      std::printf("%s: no end after %" PRIu64 " cycles: main %d decoder %d polygon %d emitter %d walker %d; culling owner %d acked %d, "
                  "requests %d%d%d; stream active %d draining %d outst %d count %d; chunks %d %d\n", fs::path(files[fi]).filename().c_str(), cyc,
                  int(rp->r3d_geo__DOT__mst), int(rp->r3d_geo__DOT__dst), int(rp->r3d_geo__DOT__pst), int(rp->r3d_geo__DOT__est),
                  int(rp->r3d_geo__DOT__u_walker__DOT__st), int(rp->r3d_geo__DOT__own), int(rp->r3d_geo__DOT__own_acked),
                  t->cull_req, t->pram_req, t->vrom_req, rp->r3d_geo__DOT__u_ms__DOT__active, rp->r3d_geo__DOT__u_ms__DOT__draining,
                  rp->r3d_geo__DOT__u_ms__DOT__outst, rp->r3d_geo__DOT__u_ms__DOT__count, int(rp->r3d_geo__DOT__dq_cnt), int(rp->r3d_geo__DOT__cq_cnt));
    }
    cycles.push_back(cyc);
    {
      uint32_t const is = t->rootp->r3d_geo__DOT__u_vpu__DOT__iss_cnt, lo = t->rootp->r3d_geo__DOT__u_vpu__DOT__lop_cnt;
      iss_f.push_back(is - iss_last); lop_f.push_back(lo - lop_last); iss_last = is; lop_last = lo;
      uint32_t const hi = t->rootp->r3d_geo__DOT__u_vpu__DOT__hi_cnt;
      hi_f.push_back(hi - hi_last); hi_last = hi;
    }
    if (std::getenv("R3D_FRAME_LOG")) std::printf("frame %s cycles %" PRIu64 "\n", fs::path(files[fi]).filename().c_str(), cyc);
    if (cyc >= prof_min) for (auto const &kv : fh) st_hist[kv.first] += kv.second;
    fh.clear();
    {
      uint32_t const sn = t->rootp->r3d_geo__DOT__u_vpu__DOT__subn_cnt;
      if (sn != subn_last) { subn_frames++; subn_total += sn - subn_last; }
      subn_last = sn;
#ifdef FACE_MON
      uint32_t const fsn = t->rootp->r3d_geo__DOT__g_face__DOT__u_face__DOT__subn_cnt;
      fsubn_total += fsn - fsubn_last; fsubn_last = fsn;
#endif
    }
    for (int pi = 0; pi < 3; pi++) { pw[pi].push_back(P[pi].words); pr[pi].push_back(P[pi].reqs); }
    // compare
    if (replay_bad) std::printf("replay: %" PRIu64 " port events off the trace\n", replay_bad);
    if (!rec.empty() || rec_bad) { std::printf("%s: %zu words left over, %" PRIu64 " malformed records\n", fs::path(files[fi]).filename().c_str(), rec.size(), rec_bad); }
    bool ok = got.size() == ref.size() && !t->err && rec.empty() && !rec_bad && port_open == port_open_f;
    port_open_f = port_open;
    if (got.size() != ref.size()) count_bad++;
    size_t n = std::min(got.size(), ref.size());
    for (size_t i = 0; i < n; i++) {
      auto const &a = ref[i].w;
      auto const &b = got[i];
      for (int k = 0; k < 37; k++) {
        if (setup_out && k >= 4 && k < 22) continue;   // the vertex words: the set-up words below
        bool same = a[k] == b[k] || ((k >= 4 && k < 22) || k == 36) && std::isnan(bf(a[k])) && std::isnan(bf(b[k]));
        if (!same) {
          ok = false;
          int fk = k < 4 ? k : k < 22 ? 4 + (k - 4) % 6 : k == 36 ? 25 : 10 + (k - 22);
          fbad[fname[fk]]++;
          if (shown < 12) { shown++; std::printf("%s tri %zu field %s (word %d): model %08X rtl %08X\n", fs::path(files[fi]).filename().c_str(), i, fname[fk], k, a[k], b[k]); }
        }
      }
    }
    if (setup_out) {
      for (size_t i = 0; i < n; i++) {
        auto const &a = sref[i];
        auto const &b = sgot[i];
        if (a.size() != b.size()) { ok = false; fbad[fname[32]]++; continue; }
        for (size_t k = 0; k < a.size(); k++) {
          bool const same = a[k] == b[k] || (std::isnan(bf(a[k])) && std::isnan(bf(b[k])));
          if (same) continue;
          ok = false;
          int const fk = k < 6 ? 26 + int(k & 1) : k == 6 ? 28 : 29 + int((k - 7) % 3);
          fbad[fname[fk]]++;
          if (shown < 12) { shown++; std::printf("%s tri %zu set-up word %zu (%s): model %08X rtl %08X\n", fs::path(files[fi]).filename().c_str(), i, k, fname[fk], a[k], b[k]); }
        }
      }
    }
    vrecs += vref.size();
    if (vref.size() != vgot.size() || vref != vgot) {
      ok = false; vrec_bad++; fbad[fname[24]]++;
      if (shown < 12) { shown++; std::printf("%s: viewport records model %zu rtl %zu differ\n", fs::path(files[fi]).filename().c_str(), vref.size(), vgot.size()); }
    }
    if (char const *sq = std::getenv("R3D_SEQ")) {
      // debug: the triangle sequence (poly, flags) of both, from index sq
      size_t const k0 = size_t(std::atol(sq));
      for (size_t k = k0; k < k0 + 40 && k < std::max(ref.size(), got.size()); k++)
        std::printf("seq %zu model %08X/%u rtl %08X/%u\n", k, k < ref.size() ? ref[k].w[3] : 0, k < ref.size() ? ref[k].w[32] : 9,
                    k < got.size() ? got[k][3] : 0, k < got.size() ? got[k][32] : 9);
    }
    tris_total += ref.size();
    if (ok) frames_ok++;
    else if (got.size() != ref.size() && shown < 12) {
      shown++;
      size_t k = 0;
      while (k < n && got[k][3] == ref[k].w[3]) k++;
      std::printf("%s: model %zu triangles, rtl %zu, err %02X; first different triangle %zu (model polygon %u, rtl %u)\n", fs::path(files[fi]).filename().c_str(),
                  ref.size(), got.size(), t->err, k, k < ref.size() ? ref[k].w[3] : 0u, k < got.size() ? got[k][3] : 0u);
    }
  }
  auto pct = [](std::vector<uint64_t> v, double p) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return double(v[std::min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))]); };
  std::printf("r3d_geo: %zu frames, %" PRIu64 " match the model on every triangle field (%" PRIu64 " triangles), %" PRIu64 " with a different count; latency %d-%d, stall %d%%\n",
              files.size(), frames_ok, tris_total, count_bad, lat_lo, lat_hi, stall);
  for (auto const &kv : fbad) std::printf("  field %-12s %" PRIu64 " mismatches\n", kv.first.c_str(), kv.second);
  if (port_open) std::printf("  %" PRIu64 " frames ended with a port still open\n", port_open);
  if (setup_out)
    std::printf("set-up records (model): %" PRIu64 " triangles, %" PRIu64 " textured, %" PRIu64 " flat; %" PRIu64 " left out (x, y or 1/w not finite), "
                "%" PRIu64 " vertices whose 1/w word is not 1.0f / w, %" PRIu64 " with no row on screen (sent)\n",
                su_tris, su_tex, su_flat, su_nonfin, su_nonfin_w, su_rows0);
  std::printf("cycles per frame p50 %.0f p99 %.0f max %.0f (p99 = %.1f%% of a 50 MHz frame, %.1f%% at 100 MHz)\n",
              pct(cycles, .5), pct(cycles, .99), pct(cycles, 1), 100.0 * pct(cycles, .99) / 869265, 100.0 * pct(cycles, .99) / 1738530);
  std::printf("subnormal or flushed values at an FP unit input or output: %" PRIu64 " (in %" PRIu64 " frames); at the facing test's units %" PRIu64 "%s\n",
              subn_total, subn_frames, fsubn_total, std::getenv("R3D_NOFACEMON") ? "" : "");
  std::printf("facing test (model): %" PRIu64 " single-sided polygons, %" PRIu64 " facing away, %" PRIu64 " with d exactly 0, %" PRIu64
              " where FP32 and double differ; %" PRIu64 " triangles left out; %" PRIu64 " viewport records, %" PRIu64 " frames with one wrong\n",
              face_single, face_back, face_zero, face_dbl, tris_culled, vrecs, vrec_bad);
  {
    // occupancy of the p99 frame: issue slots used, and lane operations of four a cycle
    std::vector<size_t> ix(cycles.size());
    for (size_t i = 0; i < ix.size(); i++) ix[i] = i;
    std::sort(ix.begin(), ix.end(), [&](size_t a, size_t b) { return cycles[a] < cycles[b]; });
    size_t const k = ix[std::min(ix.size() - 1, size_t(0.99 * (ix.size() - 1) + 0.5))];
    std::printf("pool, p99 frame (%" PRIu64 " cycles): %" PRIu64 " instructions issued (%.1f%% of its cycles, %.1f%% of a 100 MHz frame), "
                "lane operations %.1f%% of 4 a cycle\n", cycles[k], iss_f[k], 100.0 * iss_f[k] / cycles[k], 100.0 * iss_f[k] / 1738530,
                100.0 * lop_f[k] / (4.0 * cycles[k]));
  }
  {
    // study R99: a two-lane pool, each MAC word on lanes 2-3 a second cycle, nothing rescheduled
    std::vector<uint64_t> c2(cycles.size());
    for (size_t i = 0; i < c2.size(); i++) c2[i] = cycles[i] + hi_f[i];
    std::printf("two-lane estimate: MAC words on lanes 2-3 p99 %.0f; frame cycles p50 %.0f p99 %.0f max %.0f "
                "(p99 = %.1f%% of a 57.52 Hz frame at 120 MHz; four lanes %.1f%%)\n", pct(hi_f, .99), pct(c2, .5), pct(c2, .99), pct(c2, 1),
                100.0 * pct(c2, .99) / (120e6 / 57.52), 100.0 * pct(cycles, .99) / (120e6 / 57.52));
  }
  static char const *pn[] = {"culling", "polygon RAM", "VROM"};
  for (int pi = 0; pi < 3; pi++)
    std::printf("  %-12s words p50 %.0f p99 %.0f (%.1f MB/s at 57.52 Hz), requests p50 %.0f p99 %.0f\n", pn[pi],
                pct(pw[pi], .5), pct(pw[pi], .99), pct(pw[pi], .99) * 4 * 57.52 / 1e6, pct(pr[pi], .5), pct(pr[pi], .99));
  if (prof) {
    std::vector<std::pair<uint64_t, int>> v;
    for (auto const &kv : st_hist) v.push_back({kv.second, kv.first});
    std::sort(v.rbegin(), v.rend());
    uint64_t tot = 0; for (auto &x : v) tot += x.first;
    uint64_t const fr = tot / 4;   // four histogram entries a cycle
    for (size_t i = 0; i < v.size() && i < 40; i++)
      std::printf("  %s %3d: %5.1f%%\n", v[i].second >= 4000 ? "decoder" : v[i].second >= 3000 ? "engine idle" : v[i].second >= 2000 ? "kernel" : v[i].second >= 1000 ? "poly state" : "main state",
                  v[i].second % 1000, 100.0 * v[i].first / fr);
  }
#ifdef MS_MON
  std::printf("MS ready with a request waiting %" PRIu64 " cycles, %" PRIu64 " with the decoder waiting; decoder waiting for the stream %" PRIu64 " cycles\n", n_ready_pend, n_ready_pend_w, n_wait);
#endif
  delete t;
  return (frames_ok == files.size() && subn_total == 0 && fsubn_total == 0) ? 0 : 1;
}
