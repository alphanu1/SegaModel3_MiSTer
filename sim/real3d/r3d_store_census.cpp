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
// Census for the D-20 polygon store (r3d_rast_store.sv): the set-up records
// render_d25 draws (the geometry's records: back faces dropped by the normal
// test), binned by 8-row band as the store bins them, and the DDR3 traffic of
// two store layouts, per frame, p50 / p99 / max over the frames.
//
//   bins   a record's bands: floor(y) of its first and last sorted vertex (the
//          store's rule, conservative) against the rows the front walks
//   dup    every record written into each band list it touches, each band's
//          lists read back in bursts: (pairs) x record, twice
//   ptr    every record written once; per band list 16-bit pointers; each
//          band reads its pointers, then each record: tris x record +
//          pairs x (record + 4)
//
// Records: textured 23 words, untextured 17, plus the store's word (the
// viewport's number and the bands), in 64-bit beats: 12 and 9 beats.
//
//   r3d_store_census --tris DIR [--frames N] [--threads N]

#include "r3d_rast.h"
#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

struct fr_stat {
  uint64_t tris = 0, tex = 0, pairs = 0, pairs_x = 0, band_max = 0, list_max = 0, list_tot_max = 0, vps = 0;
  uint64_t bytes_rec = 0, bytes_pairs = 0;   // records once; records x bands
  uint64_t tex_pairs = 0, alpha_pairs = 0;
  uint64_t passes_used = 0;                  // (band, pass) with any record: Z generations
  uint64_t runs[3] = {};                     // ptr: read bursts when consecutive records coalesce, cap 32 / 64 / 128 beats
};

struct hook : d25_hook {
  rframe const *f = nullptr;
  fr_stat s;
  uint64_t band_n[48] = {}, list_n[48][8] = {}, tot_list[8] = {};
  bool pass_used[48][4] = {};
  uint32_t lidx[8] = {};                      // records so far in each list (the list's record index)
  int64_t last[3][48][8]; uint32_t run_b[3][48][8] = {};
  hook() { for (auto &a : last) for (auto &b : a) for (auto &c : b) c = -2; }
  void tri(int32_t idx, int layer, rtl_tri const &r, rkind, bool culled) override {
    if (layer || culled) return;
    rtri const &t = f->tris[size_t(idx)];
    s.tris++;
    s.tex += r.tex;
    int const lo = std::max(0, int(std::floor(r.fy[0]))), hi = std::min(SCR_H - 1, int(std::floor(r.fy[2])));
    if (hi < lo) return;
    uint64_t const rb = r.tex ? 96 : 72;
    s.bytes_rec += rb;
    int const l = int(t.pass) * 2 + int(t.alpha);
    tot_list[l]++;
    uint32_t const ri = lidx[l]++;
    uint32_t const nb = r.tex ? 11 : 8;
    for (int c = 0; c < 3; c++) {
      uint32_t const cap = 32u << c;
      for (int b = lo / 8; b <= hi / 8; b++) {
        if (last[c][b][l] + 1 == int64_t(ri) && run_b[c][b][l] + nb <= cap) run_b[c][b][l] += nb;
        else { s.runs[c]++; run_b[c][b][l] = nb; }
        last[c][b][l] = ri;
      }
    }
    for (int b = lo / 8; b <= hi / 8; b++) {
      s.pairs++; s.tex_pairs += r.tex; s.alpha_pairs += t.alpha;
      band_n[b]++; list_n[b][l]++;
      pass_used[b][t.pass] = true;
      s.bytes_pairs += rb;
    }
    for (int b = r.y0 / 8; b <= (r.y1 - 1) / 8; b++) s.pairs_x++;
  }
};

static uint64_t pct(std::vector<uint64_t> v, int p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  if (p >= 100) return v.back();
  return v[std::min(v.size() - 1, v.size() * size_t(p) / 100)];
}

int main(int argc, char **argv) {
  std::string tris;
  int frames = 1 << 30, threads = 12;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--threads") threads = std::atoi(nx().c_str());
  }
  std::vector<std::string> names;
  for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().string());
  std::sort(names.begin(), names.end());
  if (int(names.size()) > frames) names.resize(size_t(frames));
  std::vector<fr_stat> st(names.size());
  std::atomic<size_t> next{0};
  auto wk = [&]() {
    texram tex;
    for (size_t i; (i = next++) < names.size();) {
      rframe f;
      if (!read_rframe(names[i], f)) continue;
      rtl_opts o; o.oow = true;
      d25_opts d; d.band = 8; d.fog = false;
      hook h; h.f = &f; d.hook = &h;
      rimage img; d25_stats ds;
      render_d25(f, tex, o, d, img, ds);
      h.s.vps = f.vps.size();
      for (int b = 0; b < 48; b++) {
        h.s.band_max = std::max(h.s.band_max, h.band_n[b]);
        for (int l = 0; l < 8; l++) h.s.list_max = std::max(h.s.list_max, h.list_n[b][l]);
        for (int p = 0; p < 4; p++) h.s.passes_used += h.pass_used[b][p];
      }
      for (int l = 0; l < 8; l++) h.s.list_tot_max = std::max(h.s.list_tot_max, h.tot_list[l]);
      st[i] = h.s;
    }
  };
  std::vector<std::thread> th;
  for (int k = 0; k < threads; k++) th.emplace_back(wk);
  for (auto &t : th) t.join();

  auto col = [&](char const *nm, auto get) {
    std::vector<uint64_t> v;
    for (auto const &s : st) v.push_back(get(s));
    std::printf("  %-34s p50 %9" PRIu64 "  p99 %9" PRIu64 "  max %9" PRIu64 "\n", nm, pct(v, 50), pct(v, 99), pct(v, 100));
  };
  std::printf("store census, %zu frames, %s\n", st.size(), tris.c_str());
  col("triangles (records)", [](fr_stat const &s) { return s.tris; });
  col("textured", [](fr_stat const &s) { return s.tex; });
  col("(record, band) pairs, store rule", [](fr_stat const &s) { return s.pairs; });
  col("(record, band) pairs, front rows", [](fr_stat const &s) { return s.pairs_x; });
  col("alpha-list pairs", [](fr_stat const &s) { return s.alpha_pairs; });
  col("records in one band", [](fr_stat const &s) { return s.band_max; });
  col("records in one (band, list)", [](fr_stat const &s) { return s.list_max; });
  col("records in one list (frame)", [](fr_stat const &s) { return s.list_tot_max; });
  col("(band, pass) Z generations", [](fr_stat const &s) { return s.passes_used; });
  col("ptr read bursts, runs to 32 beats", [](fr_stat const &s) { return s.runs[0]; });
  col("ptr read bursts, runs to 64 beats", [](fr_stat const &s) { return s.runs[1]; });
  col("ptr read bursts, runs to 128 beats", [](fr_stat const &s) { return s.runs[2]; });
  col("viewports", [](fr_stat const &s) { return s.vps; });
  double const hz = 57.52;
  col("dup: bytes a frame", [](fr_stat const &s) { return 2 * s.bytes_pairs; });
  col("ptr: bytes a frame", [](fr_stat const &s) { return s.bytes_rec + s.bytes_pairs + 4 * s.pairs; });
  std::vector<uint64_t> d, p;
  for (auto const &s : st) { d.push_back(2 * s.bytes_pairs); p.push_back(s.bytes_rec + s.bytes_pairs + 4 * s.pairs); }
  std::printf("  MB/s at %.2f Hz: dup p50 %.1f p99 %.1f; ptr p50 %.1f p99 %.1f\n", hz,
              double(pct(d, 50)) * hz / 1e6, double(pct(d, 99)) * hz / 1e6, double(pct(p, 50)) * hz / 1e6, double(pct(p, 99)) * hz / 1e6);
  std::vector<uint64_t> w, r;
  for (auto const &s : st) { w.push_back(s.bytes_rec); r.push_back(s.bytes_pairs); }
  std::printf("  records once p99 %.1f MB/s; records x bands p99 %.1f MB/s\n", double(pct(w, 99)) * hz / 1e6, double(pct(r, 99)) * hz / 1e6);
  return 0;
}
