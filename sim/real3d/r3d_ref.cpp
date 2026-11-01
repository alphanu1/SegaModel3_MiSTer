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
// Runs the Real3D geometry model over Supermodel memory dumps and diffs its
// triangle lists against MAME's (tools/mame-census, M3TRIS).
//
//   r3d_ref --dumps DIR --vrom FILE [--golden DIR] [--out DIR]
//           [--arith host|rtl] [--setup cr|alg|sm] [--ftz] [--frames N] [--step HEX]
//
// --vrom is MAME's 64 MiB interleaved VROM image, written by the replay run
// (M3TRIS_VROM); it is ROM-derived and stays under sm3-builds/.
// Prints match rates per field, operation counts per stage and memory reads.

#include "r3d_geo.h"
#include <zlib.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

namespace {

bool load_dump(std::string const &fn, r3d_mem &m) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  auto rdw = [&](std::vector<uint32_t> &v, size_t bytes) {
    return gzread(g, v.data(), unsigned(bytes)) == int(bytes);
  };
  bool ok = rdw(m.cull_lo, 0x400000) && rdw(m.cull_hi, 0x100000) && rdw(m.poly, 0x400000);
  gzclose(g);
  return ok;
}

// ---- field statistics -------------------------------------------------------

struct fstat {
  uint64_t n = 0, exact = 0, ulp1 = 0, nan2 = 0;
  uint64_t px16 = 0, px2 = 0;   // within 1/16 and 1/2 of a unit (pixels, texels)
  double maxabs = 0;
  void add_f(uint32_t a, uint32_t b, bool pixel = false) {
    n++;
    float const fa = bitsf(a), fb = bitsf(b);
    if (a == b) { exact++; ulp1++; px16++; px2++; return; }
    if (std::isnan(fa) && std::isnan(fb)) { nan2++; exact++; ulp1++; px16++; px2++; return; }
    int64_t const ia = (a & 0x80000000u) ? -int64_t(a & 0x7fffffff) : int64_t(a);
    int64_t const ib = (b & 0x80000000u) ? -int64_t(b & 0x7fffffff) : int64_t(b);
    if (std::llabs(ia - ib) <= 1) ulp1++;
    double const d = std::fabs(double(fa) - double(fb));
    if (pixel) { if (d <= 1.0 / 16) px16++; if (d <= 0.5) px2++; }
    if (std::isfinite(d)) maxabs = std::max(maxabs, d);
    else maxabs = INFINITY;
  }
  void add_i(uint32_t a, uint32_t b) { n++; if (a == b) { exact++; ulp1++; px16++; px2++; } }
};

struct diff_state {
  std::map<std::string, fstat> f;
  uint64_t frames = 0, frames_exact = 0, count_mismatch_frames = 0;
  std::map<char, uint64_t> recs_gold, recs_model;
  int examples = 0;
};


// MAME's draw_model zeroes prev_vertex[] per model but not vertex[]: a slot of
// vertex[] that no polygon of the model has written holds stale stack data,
// and it reaches prev_vertex[] (and later polygons) through the copy. Track
// which slots are defined, copying them exactly as MAME copies vertices.
std::vector<char> mame_undefined(std::vector<record const *> const &P) {
  std::vector<char> ub(P.size(), 0);
  uint32_t cur = ~0u;
  bool vdef[4] = {}, pdef[4] = {};
  for (size_t i = 0; i < P.size(); i++) {
    auto const &a = P[i]->w;
    if (a[2] == 0 || a[1] != cur) {
      cur = a[1];
      for (int k = 0; k < 4; k++) { vdef[k] = false; pdef[k] = true; }
    }
    uint32_t const h0 = a[3];
    int const nv = (h0 & 0x40) ? 4 : 3;
    int vi = 0;
    for (int v = 0; v < 4; v++) if (h0 & (1u << v)) { vdef[vi] = pdef[v]; vi++; }
    for (; vi < nv; vi++) vdef[vi] = true;
    for (int k = 0; k < nv; k++) if (!vdef[k]) ub[i] = 1;
    for (int k = 0; k < 4; k++) pdef[k] = vdef[k];
  }
  return ub;
}

void diff_frame(std::string const &name, std::vector<record> const &g, std::vector<record> const &m, diff_state &ds) {
  ds.frames++;
  auto miss = [&]() { uint64_t t = 0; for (auto const &kv : ds.f) t += kv.second.n - kv.second.exact; return t; };
  uint64_t const miss0 = miss();
  std::map<char, std::vector<record const *>> G, M;
  for (auto const &r : g) { G[r.tag].push_back(&r); ds.recs_gold[r.tag]++; }
  for (auto const &r : m) { M[r.tag].push_back(&r); ds.recs_model[r.tag]++; }
  bool all_exact = true, counts_ok = true;
  for (char t : {'F', 'V', 'M', 'P', 'T', 'X', 'E'})
    if (G[t].size() != M[t].size()) {
      counts_ok = false;
      if (ds.examples < 20) { ds.examples++; std::printf("  %s: %c records gold %zu model %zu\n", name.c_str(), t, G[t].size(), M[t].size()); }
    }
  if (!counts_ok) ds.count_mismatch_frames++;

  auto cmp = [&](char t, auto fn) {
    size_t const n = std::min(G[t].size(), M[t].size());
    for (size_t i = 0; i < n; i++) {
      if (G[t][i]->w.size() != M[t][i]->w.size()) { all_exact = false; continue; }
      size_t cmpn = G[t][i]->w.size();
      if (t == 'P') cmpn = 14 + 7 * std::min<size_t>(G[t][i]->w[10], 4);  // unused vertex slots hold stale data
      if (!std::equal(G[t][i]->w.begin(), G[t][i]->w.begin() + long(std::min(cmpn, G[t][i]->w.size())), M[t][i]->w.begin())) {
        all_exact = false;
        if (ds.examples < 20 && t != 'T' && t != 'P') {
          ds.examples++;
          std::printf("  %s: %c[%zu] differs:", name.c_str(), t, i);
          for (size_t k = 0; k < G[t][i]->w.size(); k++)
            if (G[t][i]->w[k] != M[t][i]->w[k]) std::printf(" w%zu %08X/%08X", k, G[t][i]->w[k], M[t][i]->w[k]);
          std::printf("\n");
        }
      }
      fn(G[t][i]->w, M[t][i]->w);
    }
  };
  cmp('V', [&](auto const &a, auto const &b) {
    for (int k = 0; k < 3; k++) ds.f["V.pass/addr/node0"].add_i(a[k], b[k]);
    for (int k = 3; k < 9; k++) ds.f["V.viewport x y w h near far"].add_f(a[k], b[k]);
    for (int k = 9; k < 25; k++) ds.f["V.projection matrix"].add_f(a[k], b[k]);
    for (int k = 25; k < 41; k++) ds.f["V.coordinate matrix"].add_f(a[k], b[k]);
    for (int k = 41; k < 46; k++) ds.f["V.light, intensity, ambient"].add_f(a[k], b[k]);
  });
  cmp('M', [&](auto const &a, auto const &b) {
    for (int k = 0; k < 4; k++) ds.f["M.pass/addr/stack/listdepth"].add_i(a[k], b[k]);
    for (int k = 4; k < 20; k++) ds.f["M.matrix stack top"].add_f(a[k], b[k]);
    for (int k = 20; k < 36; k++) ds.f["M.view-projection matrix"].add_f(a[k], b[k]);
  });
  size_t p_idx = 0;
  std::vector<char> const ubp = mame_undefined(G['P']);
  cmp('P', [&](auto const &a, auto const &b) {
    if (ubp[p_idx++]) return;
    for (int k = 0; k < 3; k++) ds.f["P.pass/model/offset"].add_i(a[k], b[k]);
    for (int k = 3; k < 10; k++) ds.f["P.header words"].add_i(a[k], b[k]);
    for (int k = 10; k < 14; k++) ds.f["P.vertex counts, clip count, status"].add_i(a[k], b[k]);
    int const nv = int(std::min<uint32_t>(a[10], 4));
    static char const *nm[7] = {"x", "y", "z", "w", "p0", "p1", "p2"};
    for (int v = 0; v < nv; v++)
      for (int k = 0; k < 7; k++) ds.f[std::string("P.clip-space ") + nm[k]].add_f(a[14 + v * 7 + k], b[14 + v * 7 + k]);
  });
  // Per polygon: undefined in MAME, and the triangles grouped by polygon so a
  // missing triangle does not shift every later comparison.
  std::vector<char> const ub = mame_undefined(G['P']);
  std::vector<std::vector<record const *>> gt(G['P'].size() + 1), mt(M['P'].size() + 1);
  for (auto r : G['T']) if (r->w[3] < gt.size()) gt[r->w[3]].push_back(r);
  for (auto r : M['T']) if (r->w[3] < mt.size()) mt[r->w[3]].push_back(r);
  for (size_t i = 0; i < G['P'].size(); i++) if (ub[i]) ds.f["~ polygons undefined in MAME"].add_i(0, 0);
  size_t const np = std::min(gt.size(), mt.size());
  for (size_t i = 0; i < np; i++) {
    if (i < ub.size() && ub[i]) continue;
    if (gt[i].size() != mt[i].size()) { ds.f["T.triangles per polygon"].add_i(0, 1); all_exact = false; continue; }
    ds.f["T.triangles per polygon"].add_i(0, 0);
    for (size_t k = 0; k < gt[i].size(); k++) {
      auto const &a = gt[i][k]->w; auto const &b = mt[i][k]->w;
      if (a != b) all_exact = false;
      for (int q = 0; q < 2; q++) ds.f["T.pass/alpha"].add_i(a[q], b[q]);
      static char const *nm[7] = {"screen x", "screen y", "z", "w", "u/w", "v/w", "shade"};
      for (int v = 0; v < 3; v++)
        for (int q = 0; q < 7; q++) ds.f[std::string("T.") + nm[q]].add_f(a[4 + v * 7 + q], b[4 + v * 7 + q], q < 2 || q == 4 || q == 5);
      for (int q = 25; q < 32; q++) ds.f["T.texture fields"].add_i(a[q], b[q]);
      ds.f["T.param flags"].add_i(a[32], b[32]);
      ds.f["T.transparency"].add_i(a[33], b[33]);
      ds.f["T.colour"].add_i(a[34], b[34]);
    }
  }
  if (G['T'].size() != M['T'].size()) all_exact = false;
  if (false) cmp('T', [&](auto const &a, auto const &b) {
    for (int k = 0; k < 4; k++) ds.f["T.pass/alpha/index/poly"].add_i(a[k], b[k]);
    static char const *nm[7] = {"screen x", "screen y", "z", "w", "u/w", "v/w", "shade"};
    for (int v = 0; v < 3; v++)
      for (int k = 0; k < 7; k++) ds.f[std::string("T.") + nm[k]].add_f(a[4 + v * 7 + k], b[4 + v * 7 + k], k < 2 || k == 4 || k == 5);
    for (int k = 25; k < 32; k++) ds.f["T.texture fields"].add_i(a[k], b[k]);
    ds.f["T.param flags"].add_i(a[32], b[32]);
    ds.f["T.transparency"].add_i(a[33], b[33]);
    ds.f["T.colour"].add_i(a[34], b[34]);
  });
  cmp('X', [&](auto const &a, auto const &b) { for (size_t k = 0; k < a.size(); k++) ds.f["X.early exits"].add_i(a[k], b[k]); });
  cmp('E', [&](auto const &a, auto const &b) { for (size_t k = 0; k < a.size(); k++) ds.f["E.frame counts"].add_i(a[k], b[k]); });
  (void)all_exact;
  if (miss() == miss0) ds.frames_exact++;  // every compared field equal; MAME-undefined polygons excluded
}

void print_counts(char const *nm, fp_counts const &c, double frames) {
  std::printf("  %-8s mul %10.0f  add %10.0f  recip %8.0f  div %6.0f  sqrt %4.0f  half %8.0f\n", nm,
              c.mul / frames, c.add / frames, c.recip / frames, c.div / frames, c.sqrt / frames, c.half / frames);
}

} // namespace

int main(int argc, char **argv) {
  std::string dumps, vrom, golden, outdir;
  arith mode = arith::rtl;
  bool ftz = false, setup_cr = false, cull = false, frustum_sm = false;
  long max_frames = -1;
  uint32_t step = 0x10;
  // D-19 rows: Supermodel unless the output is diffed against MAME's lists
  std::string rows;
  bool no_sun_clamp = false, emit_sf = false, lod = false;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--dumps") dumps = nx();
    else if (a == "--vrom") vrom = nx();
    else if (a == "--golden") golden = nx();
    else if (a == "--out") outdir = nx();
    else if (a == "--arith") mode = nx() == "host" ? arith::host : arith::rtl;
    else if (a == "--ftz") ftz = true;
    else if (a == "--setup") { std::string const v = nx(); setup_cr = v == "cr"; frustum_sm = v == "sm"; }
    else if (a == "--cull") cull = true;
    else if (a == "--rows") rows = nx();          // mame | sm | a list: flat,fixed,double,light,trans,ctab,cull
    else if (a == "--no-sun-clamp") no_sun_clamp = true;
    else if (a == "--emit-sf") emit_sf = true;
    else if (a == "--lod") lod = true;
    else if (a == "--frames") max_frames = std::atol(nx().c_str());
    else if (a == "--step") step = uint32_t(std::strtoul(nx().c_str(), nullptr, 16));
    else if (a == "--selftest") {
      uint64_t w;
      uint64_t const bad = r3d_fp_selftest(&w);
      std::printf("reciprocal: %" PRIu64 " mismatches against 1.0f / x; Newton estimate within %" PRIu64 " unit\n", bad, w);
      return bad ? 1 : 0;
    } else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  if (dumps.empty() || vrom.empty()) { std::fprintf(stderr, "need --dumps and --vrom\n"); return 2; }

  r3d_mem mem;
  {
    std::FILE *f = std::fopen(vrom.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "cannot open %s\n", vrom.c_str()); return 1; }
    size_t const n = std::fread(mem.vrom.data(), 4, mem.vrom.size(), f);
    std::fclose(f);
    if (n != mem.vrom.size()) { std::fprintf(stderr, "short VROM image\n"); return 1; }
  }
  std::vector<std::string> files;
  for (auto const &e : fs::directory_iterator(dumps))
    if (e.path().extension() == ".gz") files.push_back(e.path().string());
  std::sort(files.begin(), files.end());
  if (max_frames >= 0 && size_t(max_frames) < files.size()) files.resize(size_t(max_frames));

  r3d_geo geo(mem, mode, ftz);
  geo.setup_cr = setup_cr;
  geo.frustum_sm = frustum_sm;
  geo.cull = cull;
  geo.step = step;
  if (rows.empty()) rows = golden.empty() ? "sm" : "mame";
  geo.rows(rows == "sm", lod);
  if (rows != "sm" && rows != "mame") {
    auto has = [&](char const *k) { return rows.find(k) != std::string::npos; };
    geo.sm_n255 = has("flat"); geo.sm_flat = has("face"); geo.sm_fixed = has("fixed"); geo.sm_double = has("double");
    geo.sm_light = has("light"); geo.sm_trans = has("trans"); geo.sm_ctab = has("ctab"); geo.sm_cull = has("cull");
  }
  geo.sun_clamp = !no_sun_clamp;
  geo.emit_sf = emit_sf;
  diff_state ds;
  std::vector<uint64_t> tris_per_frame, newv_per_frame, polys_per_frame, mul_per_frame;
  // per frame, with the vertex cache: multiplies, adds, reciprocals, divides,
  // words read from culling RAM, polygon RAM and VROM, and walker events
  struct pf { uint64_t mul, add, recip, div, lo, hi, poly, vrom, nodes, models, polys, outv; };
  std::vector<pf> per;
  pf last{};
  uint64_t prev_mul = 0, prev_tris = 0, prev_newv = 0, prev_polys = 0;
  for (size_t i = 0; i < files.size(); i++) {
    if (!load_dump(files[i], mem)) { std::fprintf(stderr, "bad dump %s\n", files[i].c_str()); return 1; }
    std::string const name = fs::path(files[i]).stem().stem().string();
    geo.frame(uint32_t(i));
    stage_counts const &s = geo.st;
    uint64_t const mul = s.stack.mul + s.model.mul + s.vertex.mul + s.clip.mul + s.project.mul + s.setup.mul;
    tris_per_frame.push_back(s.tris - prev_tris);
    newv_per_frame.push_back(s.verts_new - prev_newv);
    polys_per_frame.push_back(s.polys - prev_polys);
    mul_per_frame.push_back(mul - prev_mul);
    prev_tris = s.tris; prev_newv = s.verts_new; prev_polys = s.polys; prev_mul = mul;
    {
      auto sum = [&](uint64_t fp_counts::*m) { return s.setup.*m + s.stack.*m + s.model.*m + s.vertex.*m + s.clip.*m + s.project.*m - s.cached.*m; };
      auto rw = [&](int r) { uint64_t t = 0; for (int c = 0; c < read_counts::NC; c++) t += geo.rc.w[r][c]; return t; };
      pf cur{sum(&fp_counts::mul), sum(&fp_counts::add), sum(&fp_counts::recip), sum(&fp_counts::div),
             rw(read_counts::R_LO), rw(read_counts::R_HI), rw(read_counts::R_POLY), rw(read_counts::R_VROM),
             s.nodes, s.models, s.polys, s.verts_out};
      pf d{cur.mul - last.mul, cur.add - last.add, cur.recip - last.recip, cur.div - last.div, cur.lo - last.lo, cur.hi - last.hi,
           cur.poly - last.poly, cur.vrom - last.vrom, cur.nodes - last.nodes, cur.models - last.models, cur.polys - last.polys, cur.outv - last.outv};
      per.push_back(d);
      last = cur;
    }
    if (!outdir.empty()) geo.out.save(outdir + "/" + name + ".tri");
    if (!golden.empty()) {
      std::vector<record> g, m;
      if (!read_trilist(golden + "/" + name + ".tri", g)) { std::fprintf(stderr, "no golden list for %s\n", name.c_str()); continue; }
      geo.out.records(m);
      diff_frame(name, g, m, ds);
    }
  }
  // collect fp counters
  // (the stage units are private; counts are reported through stage_counts)
  double const nf = double(files.size());
  stage_counts const &s = geo.st;
  std::printf("frames %zu  arith %s%s%s\n", files.size(), mode == arith::host ? "host" : "rtl", ftz ? " ftz" : "", setup_cr ? " setup-cr" : "");
  std::printf("per frame: viewports %.1f  nodes %.0f  lists %.1f (entries %.0f)  models %.0f  polys %.0f  new verts %.0f  reused %.0f  out verts %.0f  tris %.0f\n",
              s.viewports / nf, s.nodes / nf, s.lists / nf, s.list_entries / nf, s.models / nf, s.polys / nf, s.verts_new / nf,
              s.verts_reused / nf, s.verts_out / nf, s.tris / nf);
  if (cull) std::printf("node culling: %.1f of %.1f nodes tested a frame have their child culled\n", s.culled / nf, s.cull_tests / nf);
  std::printf("polygons clipped %.1f / frame; lit %.1f%%; max matrix stack %" PRIu64 "; max list depth %" PRIu64 "; clip buffer over 10: %" PRIu64 "; cut recursion %" PRIu64 "; bad addresses %" PRIu64 "\n",
              s.polys_clipped / nf, 100.0 * s.polys_lit / std::max<uint64_t>(1, s.polys), s.max_stack, s.max_list_depth, s.clip_overflow, s.recursion_cut, s.bad_addr);
  std::printf("Supermodel would differ (per frame): vp disabled %.2f, node discard %.2f, node reset-matrix %.2f, colour-table nodes %.2f, LOD-list nodes %.1f,\n"
              "  other link types %.2f, list entries with null bit %.2f, polys: transparency 0 %.2f, transparency bit 5 %.2f, one discard bit %.2f,\n"
              "  both discard bits %.2f, flat (vertex normals ignored) %.0f, fixed shading %.1f, double-sided %.1f, texture size >= 6 %.2f, first shares %.2f, indexed colour %.1f\n",
              s.sm_vp_disabled / nf, s.sm_node_discard / nf, s.sm_node_reset / nf, s.sm_node_colour_table / nf, s.sm_lod_list / nf,
              s.sm_link_other_type / nf, s.sm_list_null_entry / nf, s.sm_poly_trans0 / nf, s.sm_poly_trans_bit5 / nf, s.sm_poly_discard_one / nf,
              s.sm_poly_discard_both / nf, s.sm_poly_flat / nf, s.sm_poly_fixed_shade / nf, s.sm_poly_double / nf, s.sm_poly_texsize6 / nf,
              s.sm_poly_first_shared / nf, s.sm_poly_indexed_colour / nf);
  std::printf("D-19 rows (%s): flat-normal vertex reused by a smooth polygon %" PRIu64 ", fixed polygon reusing a non-fixed vertex %" PRIu64
              ", fixed + specular %" PRIu64 ", dropped (transparency 0) %" PRIu64 ", double-sided copies %" PRIu64
              ", indexed colour with a non-default base %" PRIu64 " (totals)\n",
              rows.c_str(), s.d19_flat_reuse, s.d19_fixed_reuse_nf, s.d19_fixed_spec, s.d19_dropped, s.d19_copies, s.d19_ctab_models);
  if (geo.sm_lod)
    std::printf("row 6 (LOD), per frame: nodes with a table %.1f, culled by distance %.2f, LOD lists entry 0/1/2/3 %.2f/%.2f/%.2f/%.2f, blends %.2f,"
                " entries walked as nodes %.2f, models with alpha < 1 %.2f (alpha 0 %.2f)\n",
                s.lod_tests / nf, s.lod_far / nf, s.lod_pick[0] / nf, s.lod_pick[1] / nf, s.lod_pick[2] / nf, s.lod_pick[3] / nf,
                s.lod_blend / nf, s.lod_node_entries / nf, s.lod_alpha_models / nf, s.lod_alpha0 / nf);
  std::printf("FP operations per frame (mean):\n");
  print_counts("setup", s.setup, nf); print_counts("stack", s.stack, nf); print_counts("model", s.model, nf);
  print_counts("vertex", s.vertex, nf); print_counts("clip", s.clip, nf); print_counts("project", s.project, nf);
  auto pct = [](std::vector<uint64_t> v, double p) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return double(v[std::min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))]); };
  std::printf("per frame p50/p99/max: polys %.0f/%.0f/%.0f  new verts %.0f/%.0f/%.0f  tris %.0f/%.0f/%.0f  multiplies %.0f/%.0f/%.0f\n",
              pct(polys_per_frame, .5), pct(polys_per_frame, .99), pct(polys_per_frame, 1),
              pct(newv_per_frame, .5), pct(newv_per_frame, .99), pct(newv_per_frame, 1),
              pct(tris_per_frame, .5), pct(tris_per_frame, .99), pct(tris_per_frame, 1),
              pct(mul_per_frame, .5), pct(mul_per_frame, .99), pct(mul_per_frame, 1));
  {
    // Cycle estimate at 50 MHz, 57.52 frames/s = 869,265 cycles a frame.
    // Pipelined units, one issue each per cycle; a frame costs the busiest
    // resource's issue count divided by its unit count, plus the walker's
    // dependent reads (a node, list entry or matrix: latency L each) and
    // the model fetch (one word a cycle once streaming).
    std::printf("with shared vertices cached (RTL), per frame p50 / p99 / max:\n");
    auto col = [&](char const *nm, uint64_t pf::*m) {
      std::vector<uint64_t> v; for (auto const &x : per) v.push_back(x.*m);
      std::printf("  %-26s %10.0f %10.0f %10.0f\n", nm, pct(v, .5), pct(v, .99), pct(v, 1));
    };
    col("FP multiplies", &pf::mul); col("FP adds", &pf::add); col("reciprocals", &pf::recip); col("divides (clip, setup)", &pf::div);
    col("words: culling 8C", &pf::lo); col("words: culling 8E", &pf::hi); col("words: polygon RAM", &pf::poly); col("words: VROM", &pf::vrom);
    col("nodes", &pf::nodes); col("models", &pf::models); col("polygons", &pf::polys); col("output vertices", &pf::outv);
    std::printf("cycle estimate, share of a 50 MHz frame (869,265 cycles) at p50 / p99 / max:\n");
    for (int nm : {2, 3, 4}) {
      std::vector<uint64_t> cyc;
      for (auto const &x : per) {
        uint64_t const arith = std::max({(x.mul + nm - 1) / nm, (x.add + nm - 1) / nm, x.recip});
        uint64_t const walk = (x.nodes + x.models) * 16;      // 16-cycle dependent read per node and model start
        uint64_t const fetch = x.poly + x.vrom;               // one word a cycle, overlapped with arithmetic
        cyc.push_back(std::max(arith, fetch) + walk + x.div * 30);
      }
      std::printf("  %d multipliers + %d adders: %6.1f%% %6.1f%% %6.1f%%\n", nm, nm, 100.0 * pct(cyc, .5) / 869265, 100.0 * pct(cyc, .99) / 869265, 100.0 * pct(cyc, 1) / 869265);
    }
  }
  static char const *rn[] = {"culling lo (8C)", "culling hi (8E)", "polygon RAM", "VROM", "bad address"};
  static char const *cn[] = {"viewport", "node", "list", "matrix", "poly header", "vertex", "colour"};
  std::printf("words read per frame:\n");
  for (int r = 0; r < read_counts::NR; r++) {
    uint64_t t = 0;
    for (int c = 0; c < read_counts::NC; c++) t += geo.rc.w[r][c];
    if (!t) continue;
    std::printf("  %-16s %10.0f :", rn[r], t / nf);
    for (int c = 0; c < read_counts::NC; c++) if (geo.rc.w[r][c]) std::printf(" %s %.0f", cn[c], geo.rc.w[r][c] / nf);
    std::printf("\n");
  }
  if (!golden.empty()) {
    std::printf("diff against %s: %" PRIu64 " frames, %" PRIu64 " bit-exact (MAME-undefined polygons excluded), %" PRIu64 " with record-count differences\n",
                golden.c_str(), ds.frames, ds.frames_exact, ds.count_mismatch_frames);
    std::printf("  %-38s %12s %9s %9s %9s %9s %12s\n", "field", "values", "exact%", "1ulp%", "1/16%", "1/2%", "max |diff|");
    for (auto const &[k, v] : ds.f)
      std::printf("  %-38s %12" PRIu64 " %9.5f %9.5f %9.5f %9.5f %12.6g\n", k.c_str(), v.n, 100.0 * v.exact / std::max<uint64_t>(1, v.n),
                  100.0 * v.ulp1 / std::max<uint64_t>(1, v.n), 100.0 * v.px16 / std::max<uint64_t>(1, v.n), 100.0 * v.px2 / std::max<uint64_t>(1, v.n), v.maxabs);
  }
  return 0;
}
