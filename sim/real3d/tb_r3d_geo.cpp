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
// RTL's word stream (polygon records, r3d_geo.sv's emitter) is cut into the
// fan's triangles here, with MAME's buffer index counted per alpha buffer.
//
//   Vr3d_geo --dumps DIR --vrom FILE [--frames N] [--first N] [--lat LO HI]
//            [--stall P] [--seed S] [--setup sm|cr|alg] [--cull] [--rows sm|mame|LIST]
//
// --rows gives the model's D-19 rows (m2-notes section 3) and must match the
// RTL build's SM_* parameters: sm (default, every row), mame, or a list of
// flat, fixed, double, light, trans, ctab (flat: row 18 as built, the (2b + 1) / 255
// normals; face adds the face normals of flat polygons, a model switch only).
//
// Each memory port answers a request after 1-3 cycles and returns its words
// LO..HI cycles later (random per request, in order, one word a cycle); up to
// four requests may be in flight on the polygon RAM and VROM ports. P is the
// chance, in percent, that the triangle consumer is not ready. Reports frames
// that match exactly, mismatches per field, cycles per frame, and words and
// requests per port.

#include "Vr3d_geo.h"
#include "Vr3d_geo___024root.h"
#include "verilated.h"
#include "r3d_geo.h"
#include "r3d_ucode.h"
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
  struct beatq { uint64_t at; uint32_t d; };
  std::deque<beatq> q;
  int ack_wait = -1;
  uint64_t last_at = 0;
  uint64_t reqs = 0, words = 0;
};

static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string dumps, vrom;
  long max_frames = -1, first = 0;
  int lat_lo = 12, lat_hi = 20, stall = 20;
  unsigned seed = 1;
  bool setup_cr = false, cull = false, frustum_sm = false;
  std::string rows = "sm";
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
  Vr3d_geo *t = new Vr3d_geo;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; t->start = 0; t->cull_ack = 0; t->pram_ack = 0; t->vrom_ack = 0;
  t->cull_rvalid = 0; t->pram_rvalid = 0; t->vrom_rvalid = 0; t->out_ready = 0;
  tick(); tick(); t->rst = 0;

  static char const *fname[] = {"pass", "alpha", "index", "poly", "x", "y", "z", "w", "u/w", "v/w", "shade",
                                "tex_en", "page", "tex_x", "tex_y", "tex_w", "tex_h", "tex_fmt", "param", "transparency", "colour",
                                "flags", "model alpha"};
  std::map<std::string, uint64_t> fbad;
  uint64_t frames_ok = 0, tris_total = 0, count_bad = 0;
  std::vector<uint64_t> cycles;
  std::vector<uint64_t> pw[3], pr[3];
  int shown = 0;
  std::map<int, uint64_t> st_hist;
  bool prof = std::getenv("R3D_PROF") != nullptr;
  uint64_t const prof_min = std::getenv("R3D_PROF_MIN") ? std::strtoull(std::getenv("R3D_PROF_MIN"), nullptr, 10) : 0;
  std::map<int, uint64_t> fh;
  uint32_t subn_last = 0, iss_last = 0, lop_last = 0;
  std::vector<uint64_t> iss_f, lop_f;
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
        geo.sm_light = has("light"); geo.sm_trans = has("trans"); geo.sm_ctab = has("ctab");
      }
#ifdef SETUP_FACE
      std::vector<uint8_t> bfv;
      geo.backface = &bfv;
#endif
      geo.frame(uint32_t(fi));
      geo.out.records(ref_all);
#ifdef SETUP_FACE
      // the model's back-face verdicts for the RTL's set-up measurement (SETUP_FACE)
      for (size_t i = 0; i < 32768; i++) t->rootp->r3d_geo__DOT__face_mem[i] = i < bfv.size() ? bfv[i] : 0;
      { static uint64_t nb = 0, np = 0; for (uint8_t b : bfv) nb += b; np += bfv.size();
        if (fi + 1 == files.size()) std::printf("back faces (model, Supermodel's test): %" PRIu64 " of %" PRIu64 " polygons\n", nb, np); }
#endif
    }
    for (auto &r : ref_all) if (r.tag == 'T') ref.push_back(r);

    port P[3];  // cull, pram, vrom
    auto mem_word = [&](int pi, uint32_t a) -> uint32_t {
      if (pi == 0) {
        if (a & 0x800000) { uint32_t o = a - 0x800000; return o < 0x40000 ? mem.cull_hi[o] : 0xDEADBEEFu; }
        return a < 0x100000 ? mem.cull_lo[a] : 0xDEADBEEFu;
      }
      if (pi == 1) return a < 0x100000 ? mem.poly[a] : 0xDEADBEEFu;
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
      if (rec.size() != 3 + 7 * size_t(nv) || nv < 3) { rec_bad++; rec.clear(); return; }
      for (uint32_t i = 2; i < nv; i++) {
        std::array<uint32_t, 37> r{};
        uint32_t const alpha = (w0 >> 2) & 1;
        r[0] = w0 & 3; r[1] = alpha; r[2] = alpha ? cnt_a++ : cnt_o++; r[3] = w1 & 0xfffff;
        uint32_t const vs[3] = {0, i - 1, i};
        for (int v = 0; v < 3; v++) for (int k = 0; k < 7; k++) r[size_t(4 + 7 * v + k)] = rec[3 + 7 * vs[v] + size_t(k)];
        r[25] = (w0 >> 4) & 1; r[26] = (w0 >> 5) & 1; r[27] = (w1 >> 20) & 0x3f; r[28] = (w1 >> 26) & 0x1f;
        r[29] = (w0 >> 6) & 7; r[30] = (w0 >> 9) & 7; r[31] = (w0 >> 12) & 7; r[32] = (w0 >> 15) & 0x3f;
        r[33] = (w0 >> 21) & 0x3f; r[34] = rec[2]; r[35] = (w0 >> 3) & 1; r[36] = 0x3f800000u;   // model alpha 1.0 (row 6 not built)
        got.push_back(r);
      }
      rec.clear();
    };
    uint64_t cyc = 0;
    t->start = 1; tick(); t->start = 0;
    while ((t->busy || cyc < 4) && cyc < 200000000ull) {
      // drive memory
      uint8_t req[3] = {t->cull_req, t->pram_req, t->vrom_req};
      uint32_t addr[3] = {t->cull_addr, t->pram_addr, t->vrom_addr};
      uint32_t len[3] = {t->cull_len, t->pram_len, t->vrom_len};
      uint8_t ack[3] = {0, 0, 0}, rv[3] = {0, 0, 0};
      uint32_t rd[3] = {0, 0, 0};
      for (int pi = 0; pi < 3; pi++) {
        port &p = P[pi];
        if (req[pi]) {
          if (p.ack_wait < 0) p.ack_wait = int(rng() % 3) + 1;
          if (--p.ack_wait == 0) {
            ack[pi] = 1; p.ack_wait = -1; p.reqs++;
            uint64_t start_at = std::max<uint64_t>(cyc + uint64_t(lat_lo + int(rng() % unsigned(lat_hi - lat_lo + 1))), p.last_at + 1);
            for (uint32_t k = 0; k <= len[pi]; k++) { p.q.push_back({start_at + k, mem_word(pi, addr[pi] + k)}); p.words++; }
            p.last_at = start_at + len[pi];
          }
        }
        if (!p.q.empty() && p.q.front().at <= cyc) { rv[pi] = 1; rd[pi] = p.q.front().d; p.q.pop_front(); }
      }
      t->cull_ack = ack[0]; t->pram_ack = ack[1]; t->vrom_ack = ack[2];
      t->cull_rvalid = rv[0]; t->cull_rdata = rd[0];
      t->pram_rvalid = rv[1]; t->pram_rdata = rd[1];
      t->vrom_rvalid = rv[2]; t->vrom_rdata = rd[2];
      t->out_ready = (int(rng() % 100) >= stall) ? 1 : 0;
      t->eval();
      if (t->out_valid && t->out_ready) {
        rec.push_back(t->out_data);
        if (t->out_last) take();
      }
      if (std::getenv("R3D_DUMPVP")) {
        // debug: the first model set-up's broadcast matrices and lane registers
        static int st_ = 0;
        auto *rp = t->rootp;
        if (st_ == 0 && rp->r3d_geo__DOT__vk_busy && rp->r3d_geo__DOT__vk_entry == r3d_uc::K_MODEL) st_ = 1;
        if (st_ == 1 && !rp->r3d_geo__DOT__vk_busy) {
          st_ = 2;
          auto bank = [&](int b, int e) -> uint32_t {
            switch (b) {
              case 0: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__0__KET____DOT__u_m0__DOT__g_mlab_sync__DOT__m[e];
              case 1: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__1__KET____DOT__u_m0__DOT__g_mlab_sync__DOT__m[e];
              case 2: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__2__KET____DOT__u_m0__DOT__g_mlab_sync__DOT__m[e];
              default: return rp->r3d_geo__DOT__u_vpu__DOT__g_bank__BRA__3__KET____DOT__u_m0__DOT__g_mlab_sync__DOT__m[e];
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
    cycles.push_back(cyc);
    {
      uint32_t const is = t->rootp->r3d_geo__DOT__u_vpu__DOT__iss_cnt, lo = t->rootp->r3d_geo__DOT__u_vpu__DOT__lop_cnt;
      iss_f.push_back(is - iss_last); lop_f.push_back(lo - lop_last); iss_last = is; lop_last = lo;
    }
    if (std::getenv("R3D_FRAME_LOG")) std::printf("frame %s cycles %" PRIu64 "\n", fs::path(files[fi]).filename().c_str(), cyc);
    if (cyc >= prof_min) for (auto const &kv : fh) st_hist[kv.first] += kv.second;
    fh.clear();
    {
      uint32_t const sn = t->rootp->r3d_geo__DOT__u_vpu__DOT__subn_cnt;
      if (sn != subn_last) { subn_frames++; subn_total += sn - subn_last; }
      subn_last = sn;
    }
    for (int pi = 0; pi < 3; pi++) { pw[pi].push_back(P[pi].words); pr[pi].push_back(P[pi].reqs); }
    // compare
    if (!rec.empty() || rec_bad) { std::printf("%s: %zu words left over, %" PRIu64 " malformed records\n", fs::path(files[fi]).filename().c_str(), rec.size(), rec_bad); }
    bool ok = got.size() == ref.size() && !t->err && rec.empty() && !rec_bad;
    if (got.size() != ref.size()) count_bad++;
    size_t n = std::min(got.size(), ref.size());
    for (size_t i = 0; i < n; i++) {
      auto const &a = ref[i].w;
      auto const &b = got[i];
      int const nw = a.size() >= 37 ? 37 : 35;   // flags and model alpha when the model writes them
      for (int k = 0; k < nw; k++) {
        bool same = a[k] == b[k] || (k >= 4 && k < 25 && std::isnan(bf(a[k])) && std::isnan(bf(b[k])));
        if (!same) {
          ok = false;
          int fk = k < 4 ? k : k < 25 ? 4 + (k - 4) % 7 : 11 + (k - 25);
          fbad[fname[fk]]++;
          if (shown < 12) { shown++; std::printf("%s tri %zu field %s (word %d): model %08X rtl %08X\n", fs::path(files[fi]).filename().c_str(), i, fname[fk], k, a[k], b[k]); }
        }
      }
    }
    if (char const *sq = std::getenv("R3D_SEQ")) {
      // debug: the triangle sequence (poly, flags) of both, from index sq
      size_t const k0 = size_t(std::atol(sq));
      for (size_t k = k0; k < k0 + 40 && k < std::max(ref.size(), got.size()); k++)
        std::printf("seq %zu model %08X/%u rtl %08X/%u\n", k, k < ref.size() ? ref[k].w[3] : 0, k < ref.size() && ref[k].w.size() > 35 ? ref[k].w[35] : 9,
                    k < got.size() ? got[k][3] : 0, k < got.size() ? got[k][35] : 9);
    }
    tris_total += ref.size();
    if (ok) frames_ok++;
    else if (got.size() != ref.size() && shown < 12) { shown++; std::printf("%s: model %zu triangles, rtl %zu, err %02X\n", fs::path(files[fi]).filename().c_str(), ref.size(), got.size(), t->err); }
  }
  auto pct = [](std::vector<uint64_t> v, double p) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return double(v[std::min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))]); };
  std::printf("r3d_geo: %zu frames, %" PRIu64 " match the model on every triangle field (%" PRIu64 " triangles), %" PRIu64 " with a different count; latency %d-%d, stall %d%%\n",
              files.size(), frames_ok, tris_total, count_bad, lat_lo, lat_hi, stall);
  for (auto const &kv : fbad) std::printf("  field %-12s %" PRIu64 " mismatches\n", kv.first.c_str(), kv.second);
  std::printf("cycles per frame p50 %.0f p99 %.0f max %.0f (p99 = %.1f%% of a 50 MHz frame, %.1f%% at 100 MHz)\n",
              pct(cycles, .5), pct(cycles, .99), pct(cycles, 1), 100.0 * pct(cycles, .99) / 869265, 100.0 * pct(cycles, .99) / 1738530);
  std::printf("subnormal or flushed values at an FP unit input or output: %" PRIu64 " (in %" PRIu64 " frames)\n", subn_total, subn_frames);
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
  delete t;
  return (frames_ok == files.size() && subn_total == 0) ? 0 : 1;
}
