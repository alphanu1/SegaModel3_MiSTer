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
// The geometry's LOD word against the rasteriser model's (study R67, D-33):
// polygon by polygon, r3d_face's facing dot d and B = d NP (-cota) kz^2 from
// the geometry model (lod_word, MAME's correctly rounded frustum so kz is
// MAME's, every node walked so the polygon numbers are MAME's) against
// render_d25's dot(v0, n) and B, formed from MAME's triangle list (the view
// position rebuilt from vertex 0's screen position and w, the header normal
// through the 'M' record's matrix and the coordinate matrix).
//
//   r3d_lod_cmp --tris DIR --dumps DIR --vrom FILE [--frames N]
//
// Reports d / dv as log2 bins (a factor of 2 is one bin of 1.0), the sign
// agreement, and the hardware LOD's -256 log2 B against render_d25's.

#include "r3d_geo.h"
#include "r3d_rast.h"
#include "r3d_trilist.h"
#include <zlib.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
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
static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

int main(int argc, char **argv) {
  std::string tris, dumps, vrom;
  long frames = 1 << 30;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--dumps") dumps = nx();
    else if (a == "--vrom") vrom = nx();
    else if (a == "--frames") frames = std::atol(nx().c_str());
  }
  r3d_mem mem;
  {
    std::FILE *f = std::fopen(vrom.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "need --vrom\n"); return 2; }
    size_t n = std::fread(mem.vrom.data(), 4, mem.vrom.size(), f); (void)n; std::fclose(f);
  }
  std::vector<std::string> names;
  for (auto const &e : fs::directory_iterator(tris)) if (e.path().extension() == ".tri") names.push_back(e.path().stem().string());
  std::sort(names.begin(), names.end());
  if (long(names.size()) > frames) names.resize(size_t(frames));

  // log2(d / dv) in bins of 1/16 around 0; signs; lb8 differences
  std::map<int, uint64_t> rbin, lbin;
  uint64_t polys = 0, matched = 0, sign_same = 0, sign_diff = 0, zero_either = 0, both_lod = 0, one_lod = 0;
  uint64_t tex_polys = 0, near_level_diff = 0;
  double maxrel = 0;
  uint64_t nclip = 0, nclip_off = 0, nunclip_off = 0;
  std::vector<double> rel;
  for (auto const &nm : names) {
    rframe f;
    if (!read_rframe(tris + "/" + nm + ".tri", f)) continue;
    if (!load_dump(dumps + "/" + nm + ".bin.gz", mem)) continue;
    for (auto &v : f.vps) for (int k = 0; k < 0x30; k++) v.node[k] = mem.cull_hi[((v.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
    r3d_geo geo(mem, arith::rtl, false);
    geo.setup_cr = true; geo.frustum_sm = false; geo.cull = false;
    geo.rows(true);
    geo.lod_word = true;
    geo.frame(0);
    std::vector<record> recs;
    geo.out.records(recs);
    // the polygon's own record (not the inverted copy): d, B
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> gd;
    for (auto const &r : recs)
      if (r.tag == 'T' && r.w.size() >= 39 && !(r.w[35] & 1) && !gd.count(r.w[3])) gd[r.w[3]] = {r.w[r.w.size() - 2], r.w.back()};
    // MAME's 'P' records: vertex 0's w before the clip, to tell a clipped vertex 0
    std::vector<record> mr;
    read_trilist(tris + "/" + nm + ".tri", mr);
    std::vector<uint32_t> w0pre;
    for (auto const &r : mr) if (r.tag == 'P' && r.w.size() >= 18) w0pre.push_back(r.w[17]);
    std::vector<bool> seen;
    for (auto const &t : f.tris) {
      if (t.poly >= seen.size()) seen.resize(t.poly + 1, false);
      if (seen[t.poly]) continue;
      seen[t.poly] = true;
      polys++;
      double dv; uint32_t bw;
      if (!d25_lod_terms(f, t, dv, bw)) continue;
      auto it = gd.find(t.poly);
      if (it == gd.end()) continue;
      matched++;
      double const d = double(bf(it->second.first));
      if (d == 0 || dv == 0) { zero_either++; continue; }
      if ((d > 0) == (dv > 0)) sign_same++; else sign_diff++;
      double const lr = std::log2(std::fabs(d / dv));
      uint32_t w0; std::memcpy(&w0, &t.w[0], 4);
      bool const clipped = t.poly >= w0pre.size() || w0pre[t.poly] != w0;
      rbin[int(std::floor(lr * 16.0 + 0.5))]++;
      if (clipped) { nclip++; if (std::fabs(lr) >= 1.0 / 32) nclip_off++; }
      else if (std::fabs(lr) >= 1.0 / 32) nunclip_off++;
      rel.push_back(std::fabs(d / dv - 1.0));
      maxrel = std::max(maxrel, std::fabs(lr));
      if (t.tex_en) {
        tex_polys++;
        int32_t const lg = d25_lod_lb8(it->second.second), lr8 = d25_lod_lb8(bw);
        if (lg == INT32_MIN && lr8 == INT32_MIN) continue;
        if (lg == INT32_MIN || lr8 == INT32_MIN) { one_lod++; continue; }
        both_lod++;
        lbin[lg - lr8]++;
        // the nearest level differs somewhere when the two lb8 round apart for some pixel:
        // counted as |difference| >= 128 (half a level)
        if (std::abs(lg - lr8) >= 128) near_level_diff++;
      }
    }
  }
  std::printf("r3d_lod_cmp: %zu frames, %" PRIu64 " polygons in MAME's lists, %" PRIu64 " matched to the geometry model\n", names.size(), polys, matched);
  std::printf("  d against render_d25's dot(v0, n): same sign %" PRIu64 ", opposite %" PRIu64 ", either zero %" PRIu64 "\n", sign_same, sign_diff, zero_either);
  std::sort(rel.begin(), rel.end());
  auto q = [&](double p) { return rel.empty() ? 0.0 : rel[std::min(rel.size() - 1, size_t(p * double(rel.size() - 1)))]; };
  std::printf("  |d / dv - 1|: p50 %.3g p90 %.3g p99 %.3g p99.9 %.3g max %.3g; max |log2(d / dv)| %.3f\n", q(.5), q(.9), q(.99), q(.999),
              rel.empty() ? 0.0 : rel.back(), maxrel);
  std::printf("  off by 1/32 level or more: %" PRIu64 " with vertex 0 clipped (of %" PRIu64 " clipped), %" PRIu64 " unclipped\n", nclip_off, nclip, nunclip_off);
  std::printf("  log2 |d / dv| in 1/16 bins (bin: polygons):");
  int k = 0;
  for (auto const &kv : rbin) { if (k++ % 8 == 0) std::printf("\n   "); std::printf(" %+5.2f: %" PRIu64, kv.first / 16.0, kv.second); }
  std::printf("\n  textured polygons %" PRIu64 ": both B positive %" PRIu64 ", one only %" PRIu64 "; -256 log2 B differs by >= 1/2 level on %" PRIu64 "\n",
              tex_polys, both_lod, one_lod, near_level_diff);
  std::printf("  lb8 (geometry) - lb8 (render_d25), 1/256 level:");
  k = 0;
  uint64_t small = 0;
  for (auto const &kv : lbin) {
    if (std::abs(kv.first) <= 4) { small += kv.second; if (k++ % 8 == 0) std::printf("\n   "); std::printf(" %+d: %" PRIu64, kv.first, kv.second); }
  }
  std::printf("\n   within 4/256: %" PRIu64 " of %" PRIu64 "\n", small, both_lod);
  return 0;
}
