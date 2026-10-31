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
// The geometry model's D-19 lighting rows, measured through the rasteriser's
// Supermodel-form model (render_sm, every switch on) against Supermodel's
// 3D-only grabs. One frame at a time, in-process: dump -> r3d_geo -> render_sm.
//
//   r3d_light_cmp --dumps DIR --vrom FILE --sm DIR [--first N] [--frames N] [--png DIR]
//                 [--variants mame,vertex,fragment,-flat,+face,-fixed,-double,-light,-trans,-ctab]
//
// Variants (all with Supermodel's frustum and node culling):
//   mame     MAME's lighting rows (18-22 off); transparency and colour table on
//   vertex   every D-19 row: Supermodel's formula per vertex, Gouraud
//   fragment the same, but the formula evaluated per fragment: lit triangles
//            are cut into pieces of at most 4 px a side and the unclamped
//            sunFactor interpolated perspective-correct to their corners
//            (GL's varyings), so the clamps act per piece, not per vertex
//   -ROW     vertex with that one row back to MAME's (flat, fixed, double,
//            light, trans, ctab)
//   lod      vertex plus row 6; the rasteriser model takes no model alpha, so
//            it goes in as the polygon's translucency (header[6] bits 18-23,
//            1/32 steps), and alpha 0 drops the triangle
// For double-sided polygons the two copies are culled as Supermodel's shader
// culls them: header bit 0x10 cleared, the inverted copy's face normal negated.
// Prints one line per frame and variant: PSNR's squared error, pixel count,
// coverage disagreements. sim/real3d/light_cmp.py sums them.

#include "r3d_geo.h"
#include "r3d_rast.h"
#include <zlib.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

namespace {

bool load_all(std::string const &fn, r3d_mem &m, texram &t, std::vector<uint32_t> &dl) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  bool ok = gzread(g, m.cull_lo.data(), 0x400000) == 0x400000 && gzread(g, m.cull_hi.data(), 0x100000) == 0x100000 &&
            gzread(g, m.poly.data(), 0x400000) == 0x400000 && gzread(g, t.t.data(), 0x800000) == 0x800000;
  gzclose(g);
  dl = m.cull_hi;
  return ok;
}

bool load_bmp(std::string const &fn, std::vector<uint32_t> &px) {
  std::FILE *f = std::fopen(fn.c_str(), "rb");
  if (!f) return false;
  std::vector<uint8_t> b;
  uint8_t buf[65536];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) b.insert(b.end(), buf, buf + n);
  std::fclose(f);
  if (b.size() < 54 || b[0] != 'B' || b[1] != 'M') return false;
  auto u32 = [&](size_t o) { return uint32_t(b[o]) | uint32_t(b[o + 1]) << 8 | uint32_t(b[o + 2]) << 16 | uint32_t(b[o + 3]) << 24; };
  uint32_t const off = u32(10);
  int32_t const w = int32_t(u32(18)), h = int32_t(u32(22));
  int const bpp = b[28] | b[29] << 8;
  if (w != SCR_W || std::abs(h) != SCR_H || (bpp != 32 && bpp != 24)) return false;
  size_t const stride = ((size_t(w) * size_t(bpp) / 8) + 3) & ~size_t(3);
  px.assign(SCR_W * SCR_H, 0);
  for (int y = 0; y < SCR_H; y++) {
    int const sy = h > 0 ? SCR_H - 1 - y : y;
    size_t const o = off + size_t(sy) * stride;
    if (o + stride > b.size()) return false;
    for (int x = 0; x < SCR_W; x++) {
      uint8_t const *p = &b[o + size_t(x) * size_t(bpp / 8)];
      px[size_t(y) * SCR_W + size_t(x)] = 0xff000000u | uint32_t(p[2]) << 16 | uint32_t(p[1]) << 8 | p[0];
    }
  }
  return true;
}

void save_ppm(std::string const &fn, std::vector<uint32_t> const &px) {
  if (std::FILE *f = std::fopen(fn.c_str(), "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", SCR_W, SCR_H);
    for (uint32_t p : px) { uint8_t c[3] = {uint8_t(p >> 16), uint8_t(p >> 8), uint8_t(p)}; std::fwrite(c, 1, 3, f); }
    std::fclose(f);
  }
}

float uf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

// extra per-triangle data the model appends to 'T' (r3d_geo.h)
struct text { float malpha = 1.0f; bool inv = false; float sf[3] = {}; float d = 0, a = 0; bool floor = true; bool has_sf = false; };

// read_rframe, from records in memory
void to_rframe(std::vector<record> const &recs, rframe &out, std::vector<text> &ext) {
  out.tris.clear(); out.vps.clear(); out.models.clear(); ext.clear();
  std::vector<std::array<uint32_t, 7>> polyhdr;
  for (auto const &r : recs) {
    auto f = [&](size_t i) { return uf(r.w[i]); };
    if (r.tag == 'V' && r.w.size() >= 25) {
      rvp v;
      v.pass = r.w[0]; v.addr = r.w[1];
      v.x = f(3); v.y = f(4); v.w = f(5); v.h = f(6); v.near = f(7); v.far = f(8);
      for (int k = 0; k < 16; k++) v.proj[k] = f(size_t(9 + k));
      for (int k = 0; k < 16; k++) v.coord[k] = f(size_t(25 + k));
      out.vps.push_back(v);
    } else if (r.tag == 'M' && r.w.size() >= 20) {
      rmodel m;
      for (int k = 0; k < 16; k++) m.top[k] = f(size_t(4 + k));
      out.models.push_back(m);
    } else if (r.tag == 'P' && r.w.size() >= 10) {
      std::array<uint32_t, 7> h;
      for (int k = 0; k < 7; k++) h[size_t(k)] = r.w[size_t(3 + k)];
      polyhdr.push_back(h);
    } else if (r.tag == 'T' && r.w.size() >= 35) {
      rtri t;
      t.pass = r.w[0]; t.alpha = r.w[1]; t.buf = r.w[2]; t.poly = r.w[3];
      for (int k = 0; k < 3; k++) {
        size_t const b = 4 + 7 * size_t(k);
        t.x[k] = f(b); t.y[k] = f(b + 1); t.z[k] = f(b + 2); t.w[k] = f(b + 3);
        t.p0[k] = f(b + 4); t.p1[k] = f(b + 5); t.p2[k] = f(b + 6);
      }
      t.tex_en = r.w[25]; t.page = r.w[26]; t.tex_x = r.w[27]; t.tex_y = r.w[28];
      t.tex_w = r.w[29]; t.tex_h = r.w[30]; t.fmt = r.w[31]; t.param = r.w[32];
      t.trans = r.w[33]; t.color = r.w[34];
      if (t.poly < polyhdr.size()) for (int k = 0; k < 7; k++) t.hdr[k] = polyhdr[t.poly][size_t(k)];
      t.vp = int32_t(out.vps.size()) - 1;
      t.model = int32_t(out.models.size()) - 1;
      text e;
      if (r.w.size() >= 37) { e.inv = (r.w[35] & 1) != 0; e.malpha = f(36); }
      if (r.w.size() >= 43) {
        e.has_sf = true;
        for (int k = 0; k < 3; k++) e.sf[k] = f(size_t(37 + k));
        e.d = f(40); e.a = f(41); e.floor = r.w[42] != 0;
      }
      out.tris.push_back(t);
      ext.push_back(e);
    }
  }
}

// Supermodel's shader on one interpolated sunFactor: clamp, floor, diffuse
// plus ambient, the Step 1.0 intensity limit; as a shade 0..255
float shade_of(float sf, text const &e) {
  float x = std::clamp(sf, -1.0f, 1.0f);
  if (e.floor) x = std::max(x, 0.0f);
  return std::min(x * e.d + e.a, 1.0f) * 255.0f;
}

struct pv { float x, y, z, w, p0, p1, sf; };

void split(rtri const &t0, text const &e, pv const v[3], int depth, std::vector<rtri> &out) {
  float m = 0;
  for (int k = 0; k < 3; k++) {
    float const dx = v[k].x - v[(k + 1) % 3].x, dy = v[k].y - v[(k + 1) % 3].y;
    m = std::max(m, dx * dx + dy * dy);
  }
  if (m <= 16.0f || depth >= 9) {
    rtri t = t0;
    for (int k = 0; k < 3; k++) {
      t.x[k] = v[k].x; t.y[k] = v[k].y; t.z[k] = v[k].z; t.w[k] = v[k].w;
      t.p0[k] = v[k].p0; t.p1[k] = v[k].p1; t.p2[k] = shade_of(v[k].sf, e);
    }
    out.push_back(t);
    return;
  }
  auto mid = [](pv const &a, pv const &b) {
    pv c;
    c.x = 0.5f * (a.x + b.x); c.y = 0.5f * (a.y + b.y); c.z = 0.5f * (a.z + b.z);
    c.p0 = 0.5f * (a.p0 + b.p0); c.p1 = 0.5f * (a.p1 + b.p1);
    double const ia = 1.0 / double(a.w), ib = 1.0 / double(b.w);
    c.w = float(2.0 / (ia + ib));
    c.sf = float((double(a.sf) * ia + double(b.sf) * ib) / (ia + ib));
    return c;
  };
  pv const m01 = mid(v[0], v[1]), m12 = mid(v[1], v[2]), m20 = mid(v[2], v[0]);
  pv const a[3] = {v[0], m01, m20}, b[3] = {m01, v[1], m12}, c[3] = {m20, m12, v[2]}, d[3] = {m01, m12, m20};
  split(t0, e, a, depth + 1, out); split(t0, e, b, depth + 1, out);
  split(t0, e, c, depth + 1, out); split(t0, e, d, depth + 1, out);
}

} // namespace

int main(int argc, char **argv) {
  std::string dumps, vrom, smdir, png, vlist = "mame,vertex,fragment";
  long first = 0, frames = -1;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--dumps") dumps = nx();
    else if (a == "--vrom") vrom = nx();
    else if (a == "--sm") smdir = nx();
    else if (a == "--first") first = std::atol(nx().c_str());
    else if (a == "--frames") frames = std::atol(nx().c_str());
    else if (a == "--png") png = nx();
    else if (a == "--variants") vlist = nx();
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  r3d_mem mem;
  {
    std::FILE *f = std::fopen(vrom.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "need --vrom\n"); return 2; }
    size_t const n = std::fread(mem.vrom.data(), 4, mem.vrom.size(), f); (void)n; std::fclose(f);
  }
  std::vector<std::string> files;
  for (auto const &e : fs::directory_iterator(dumps)) if (e.path().extension() == ".gz") files.push_back(e.path().string());
  std::sort(files.begin(), files.end());
  if (first > 0) files.erase(files.begin(), files.begin() + std::min<long>(first, long(files.size())));
  if (frames >= 0 && size_t(frames) < files.size()) files.resize(size_t(frames));
  texram tex;
  std::vector<uint32_t> dl;
  std::vector<std::string> vn;
  for (size_t p = 0; p <= vlist.size();) {
    size_t const q = std::min(vlist.find(',', p), vlist.size());
    if (q > p) vn.push_back(vlist.substr(p, q - p));
    p = q + 1;
  }
  for (auto const &fn : files) {
    if (!load_all(fn, mem, tex, dl)) { std::fprintf(stderr, "bad dump %s\n", fn.c_str()); return 1; }
    std::string const name = fs::path(fn).stem().stem().string();
    char smn[64];
    std::snprintf(smn, sizeof(smn), "/full_%06d.bmp", std::atoi(name.c_str() + 1));
    std::vector<uint32_t> grab;
    if (!load_bmp(smdir + smn, grab)) { std::fprintf(stderr, "no grab for %s\n", name.c_str()); continue; }
    for (size_t vi = 0; vi < vn.size(); vi++) {
      std::string const &vs = vn[vi];
      int const v = vs == "mame" ? 0 : vs == "fragment" ? 2 : 1;
      r3d_geo geo(mem, arith::rtl, true);
      geo.frustum_sm = true; geo.cull = true;
      geo.rows(true, vs == "lod");
      if (v == 0) geo.sm_flat = geo.sm_n255 = geo.sm_fixed = geo.sm_double = geo.sm_light = false;
      if (vs == "-flat") geo.sm_flat = geo.sm_n255 = false;   // row 18 off: MAME's b / 127 normals
      if (vs == "+face") geo.sm_flat = true;                   // row 18 in full: face normals (model only)
      if (vs == "-fixed") geo.sm_fixed = false;
      if (vs == "-double") geo.sm_double = false;
      if (vs == "-light") geo.sm_light = false;
      if (vs == "-trans") geo.sm_trans = false;
      if (vs == "-ctab") geo.sm_ctab = false;
      geo.emit_sf = v == 2;
      geo.frame(0);
      std::vector<record> recs;
      geo.out.records(recs);
      rframe f;
      std::vector<text> ext;
      to_rframe(recs, f, ext);
      f.name = name;
      for (auto &vp : f.vps)
        for (int k = 0; k < 0x30; k++) vp.node[k] = dl[((vp.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
      uint64_t pieces = 0;
      if (v > 0) {
        std::vector<rtri> nt;
        nt.reserve(f.tris.size() * (v == 2 ? 4 : 1));
        for (size_t i = 0; i < f.tris.size(); i++) {
          rtri t = f.tris[i];
          text const &e = ext[i];
          if ((t.hdr[1] & 0x10) && geo.sm_double) {
            // both copies culled by facing; the inverted one with -n
            t.hdr[1] &= ~0x10u;
            if (e.inv)
              for (int k = 1; k <= 3; k++) {
                uint32_t const n24 = uint32_t(-(int32_t(t.hdr[k]) >> 8)) & 0xffffff;
                t.hdr[k] = (n24 << 8) | (t.hdr[k] & 0xff);
              }
          }
          if (e.malpha < 1.0f) {
            double const a0 = (t.hdr[6] & 0x800000) ? 1.0 : double((((t.hdr[6] >> 18) & 0x3f) * 255) / 32) / 255.0;
            double const a = a0 * double(e.malpha);
            if (a <= 0.0) continue;
            uint32_t const t6 = uint32_t(std::clamp(std::lround(a * 32.0), 1L, 31L));
            t.hdr[6] = (t.hdr[6] & ~0xfc0000u) | (t6 << 18);
            t.alpha = 1; t.trans = t6;
          }
          bool const lit = !(t.hdr[6] & 0x10000);
          if (v == 2 && lit && e.has_sf && !(e.sf[0] == e.sf[1] && e.sf[1] == e.sf[2])) {
            pv const p[3] = {{t.x[0], t.y[0], t.z[0], t.w[0], t.p0[0], t.p1[0], e.sf[0]},
                             {t.x[1], t.y[1], t.z[1], t.w[1], t.p0[1], t.p1[1], e.sf[1]},
                             {t.x[2], t.y[2], t.z[2], t.w[2], t.p0[2], t.p1[2], e.sf[2]}};
            size_t const n0 = nt.size();
            split(t, e, p, 0, nt);
            pieces += nt.size() - n0;
          } else nt.push_back(t);
        }
        f.tris.swap(nt);
      }
      rimage img;
      rstats st;
      render_sm(f, tex, SM_ALL, img, st);
      double se = 0;
      uint64_t n = 0, cov = 0, n8 = 0;
      for (size_t p = 0; p < grab.size(); p++) {
        uint32_t const a = img.px[p] & 0xffffff, b = grab[p] & 0xffffff;
        int mx = 0;
        for (int sh = 0; sh < 24; sh += 8) {
          int const d = int((a >> sh) & 0xff) - int((b >> sh) & 0xff);
          se += double(d) * d;
          mx = std::max(mx, std::abs(d));
        }
        n++;
        n8 += mx > 8;
        cov += ((img.px[p] >> 24) != 0) != (b != 0);
      }
      std::printf("%s %s se %.0f n %" PRIu64 " n8 %" PRIu64 " cov %" PRIu64 " tris %zu pieces %" PRIu64 "\n", name.c_str(), vs.c_str(), se, n, n8, cov,
                  f.tris.size(), pieces);
      if (!png.empty()) {
        save_ppm(png + "/" + name + "_" + vs + ".ppm", img.px);
        if (vi == 0) save_ppm(png + "/" + name + "_sm.ppm", grab);
      }
    }
    std::fflush(stdout);
  }
  return 0;
}
