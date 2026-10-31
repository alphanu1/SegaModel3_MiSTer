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
// Real3D rasteriser model, Supermodel form (study D-19): where MAME and
// Supermodel differ, the rasteriser follows Supermodel's New3D renderer
// (Supermodel 036af1c, Src/Graphics/New3D: R3DShaderCommon.h,
// R3DShaderTriangles.h, R3DFrameBuffers.cpp, New3D.cpp, PolyHeader.cpp, GPL-3).
// This is a behavioural model in floating point, as the GPU computes it: it
// measures which behaviours close the gap to Supermodel's images. The RTL's
// own arithmetic for them comes after, in the RTL form.
//
// Each behaviour is a switch (sm_feat); off gives MAME's behaviour for that
// aspect:
//   COLOUR  textures always modulated by the polygon colour (MAME: only with
//           header[3] bit 7, which Supermodel reads as a wrap-mode bit)
//   DECODE  Supermodel's texel decode: L8 white and contour A4L4 luminance 15
//           are transparent, A4L4 nibble order, format 7 without alpha test
//   SAMPLE  bilinear around texel centres (half-texel offset), repeat / clamp /
//           mirror by header[3] bits 7, 6 and header[2] bits 1, 0, the sheet
//           coordinate wrapped at 2048 x 1024
//   ALPHA   contour: alpha below 32/255 discarded, colour dilated from the
//           opaque neighbours; texture alpha: the opaque part in the opaque
//           pass; any pixel with alpha below 1/32 discarded
//   TRANS   two translucency layers (header[6] bit 17), the nearest fragment
//           of each kept, composited after the pass (Supermodel's
//           R3DFrameBuffers); MAME blends every translucent polygon back to front
//   DEPTH   Z on 1/w, larger nearer (the GPU's perspective-correct depth);
//           MAME compares w interpolated linearly in screen space
//   LAYER   layered polygons (header[6] bit 3) drawn after the rest of their
//           viewport, depth x (1 + 1/512), only the first one per pixel
//   CULL    back faces discarded: dot(view vertex, view face normal) > 0,
//           unless double-sided
//   FOG     per-pixel fog from the viewport's fog words and header[6] bits
//           11-15, and the scroll-fog layer behind the scene
//   MIP     mip level from Supermodel's LOD (w, cota, header[5] NP and the
//           view-space dot product), trilinear between levels
//   PRIO    high-priority polygons (header[6] bit 4) in a second sub-pass with
//           Z cleared
// Lighting stays MAME's per-vertex shade (geometry, M2's row list).

#include "r3d_rast.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace r3d {

uint32_t mame_decode(int fmt, uint32_t p);

namespace {

constexpr int MIPX[] = {0, 1024, 1536, 1792, 1920, 1984, 2016, 2032, 2040, 2044, 2046, 2047};
constexpr int MIPY[] = {0, 512, 768, 896, 960, 992, 1008, 1016, 1020, 1022, 1023};

struct col4 { double r = 0, g = 0, b = 0, a = 0; };
inline col4 mix(col4 const &x, col4 const &y, double t) {
  return {x.r + (y.r - x.r) * t, x.g + (y.g - x.g) * t, x.b + (y.b - x.b) * t, x.a + (y.a - x.a) * t};
}
inline float uf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

// Supermodel's ExtractColour, formats 0-11
col4 sm_decode(int type, uint32_t v) {
  col4 c;
  auto lum = [&](double l, bool key) { c.r = c.g = c.b = l; c.a = key ? (l == 1.0 ? 0.0 : 1.0) : l; };
  switch (type) {
  case 0: c.r = ((v >> 10) & 31) / 31.0; c.g = ((v >> 5) & 31) / 31.0; c.b = (v & 31) / 31.0; c.a = 1.0 - ((v >> 15) & 1); break;
  case 1: c.r = c.g = c.b = (v & 15) / 15.0; c.a = ((v >> 4) & 15) / 15.0; break;
  case 2: c.a = (v & 15) / 15.0; c.r = c.g = c.b = ((v >> 4) & 15) / 15.0; break;
  case 3: c.r = c.g = c.b = ((v >> 8) & 15) / 15.0; c.a = ((v >> 12) & 15) / 15.0; break;
  case 4: c.a = ((v >> 8) & 15) / 15.0; c.r = c.g = c.b = ((v >> 12) & 15) / 15.0; break;
  case 5: lum((v & 255) / 255.0, true); break;
  case 6: lum(((v >> 8) & 255) / 255.0, true); break;
  case 7: c.r = ((v >> 12) & 15) / 15.0; c.g = ((v >> 8) & 15) / 15.0; c.b = ((v >> 4) & 15) / 15.0; c.a = (v & 15) / 15.0; break;
  case 8: lum((v & 15) / 15.0, true); break;
  case 9: lum(((v >> 4) & 15) / 15.0, true); break;
  case 10: lum(((v >> 8) & 15) / 15.0, true); break;
  case 11: lum(((v >> 12) & 15) / 15.0, true); break;
  }
  return c;
}
col4 from_argb(uint32_t c) { return {((c >> 16) & 255) / 255.0, ((c >> 8) & 255) / 255.0, (c & 255) / 255.0, ((c >> 24) & 255) / 255.0}; }

// a plane through three screen points, in double
struct dplane {
  double a = 0, b = 0, c = 0;
  double at(double x, double y) const { return a * x + b * y + c; }
};
dplane fit(double const x[3], double const y[3], double const p[3]) {
  dplane q;
  double const d = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
  if (d == 0 || !std::isfinite(d)) { q.c = p[0]; return q; }
  double const d1 = p[1] - p[0], d2 = p[2] - p[0];
  q.a = (d1 * (y[2] - y[0]) - d2 * (y[1] - y[0])) / d;
  q.b = (d2 * (x[1] - x[0]) - d1 * (x[2] - x[0])) / d;
  q.c = p[0] - q.a * x[0] - q.b * y[0];
  return q;
}

struct fbuf {
  std::vector<col4> c, t1, t2;
  std::vector<double> z, zsave;
  std::vector<uint8_t> st;
  fbuf() : c(SCR_W * SCR_H), t1(SCR_W * SCR_H), t2(SCR_W * SCR_H), z(SCR_W * SCR_H), zsave(SCR_W * SCR_H), st(SCR_W * SCR_H) {}
};

enum class layer { colour, trans1, trans2, mame_alpha };

struct ctx {
  rframe const &f;
  texram const &tex;
  uint32_t feat;
  fbuf &fb;
  rstats &st;
  bool on(uint32_t b) const { return (feat & b) != 0; }
};

// One texel of a level of a page; the sheet coordinate wraps (SAMPLE) or runs
// on linearly as MAME's pointer does
uint32_t raw_texel(ctx const &c, int page, int level, int x, int y) {
  if (c.on(SM_SAMPLE)) {
    x &= (2047 >> level);
    y &= (1023 >> level);
    return c.tex.t[size_t(page) * 2048 * 1024 + size_t(MIPY[level] + y) * 2048 + size_t(MIPX[level] + x)];
  }
  size_t const a = size_t(y) * 2048 + size_t(x);
  return c.tex.t[(size_t(page) * 2048 * 1024 + a) & (2048u * 2048u - 1)];
}

// Supermodel's LinearTexLocations: texel indices and the weight
double locations(int mode, int size, double u, int &i0, int &i1) {
  if (mode == 0) {                 // repeat
    double const s = u * size - 0.5;
    double const f = std::floor(s);
    i0 = int(((long long)f % size + size) % size);
    i1 = (i0 + 1) % size;
    return s - f;
  }
  double uu;
  if (mode == 1) uu = u - std::floor(u);   // repeat + clamp
  else {                                   // mirror (+ clamp)
    double const odd = std::floor(std::fmod(std::floor(u), 2.0) + 2.0);
    uu = (std::fmod(odd, 2.0) > 0) ? 1.0 - (u - std::floor(u)) : u - std::floor(u);
  }
  double const s = uu * size - 0.5;
  double const f = std::floor(s);
  i0 = int(f);
  i1 = i0 + 1;
  if (i0 < 0) i0 = 0;
  if (i1 >= size) i1 = size - 1;
  return s - f;
}

struct texsamp { col4 c; bool discard = false; };

// Supermodel's GetTextureValue (micro-textures not modelled)
texsamp sm_texture(ctx const &c, rtri const &t, double u_tex, double v_tex, double lod, bool opaque_pass) {
  texsamp out;
  uint32_t const *h = t.hdr;
  int const w = 32 << t.tex_w, hh = 32 << t.tex_h;
  bool alpha_test = (h[6] & 0x80000000u) != 0;
  bool const tex_alpha = (h[6] & 7) != 0;
  int fmt = int(t.fmt & 7);
  if (c.on(SM_DECODE) && alpha_test && fmt >= 1 && fmt <= 4) fmt += 7;
  if (c.on(SM_DECODE) && fmt == 7) alpha_test = false;
  auto decode = [&](uint32_t v) -> col4 {
    if (c.on(SM_DECODE)) return sm_decode(fmt, v);
    return from_argb(mame_decode(fmt & 7, v));
  };
  bool const smooth_u = (h[3] & 0x80) && !alpha_test, smooth_v = (h[3] & 0x40) && !alpha_test;
  int const mode_u = (h[2] & 2) ? (smooth_u ? 2 : 3) : (smooth_u ? 0 : 1);
  int const mode_v = (h[2] & 1) ? (smooth_v ? 2 : 3) : (smooth_v ? 0 : 1);
  int const px = t.tex_x * 32, py = t.tex_y * 32;

  auto bilinear = [&](int level) -> col4 {
    int const sw = std::max(w >> level, 1), sh = std::max(hh >> level, 1);
    int const lx = px >> level, ly = py >> level;
    col4 q[4];
    double a, b;
    int x0, x1, y0, y1;
    if (c.on(SM_SAMPLE)) {
      a = locations(mode_u, sw, u_tex / double(w), x0, x1);
      b = locations(mode_v, sh, v_tex / double(hh), y0, y1);
      q[0] = decode(raw_texel(c, int(t.page), level, lx + x0, ly + y0));
      q[1] = decode(raw_texel(c, int(t.page), level, lx + x1, ly + y0));
      q[2] = decode(raw_texel(c, int(t.page), level, lx + x0, ly + y1));
      q[3] = decode(raw_texel(c, int(t.page), level, lx + x1, ly + y1));
    } else {
      // MAME: no half-texel offset, its doubled mirror image, level 0
      uint32_t const umask = (((t.param & TP_MIRROR_U) ? 64u : 32u) << t.tex_w) - 1;
      uint32_t const vmask = (((t.param & TP_MIRROR_V) ? 64u : 32u) << t.tex_h) - 1;
      double const us = std::floor(u_tex * 256.0), vs = std::floor(v_tex * 256.0);
      uint32_t const ui = uint32_t(int64_t(us)), vi = uint32_t(int64_t(vs));
      uint32_t const u1 = (ui >> 8) & umask, v1 = (vi >> 8) & vmask, u2 = (u1 + 1) & umask, v2 = (v1 + 1) & vmask;
      a = (ui & 255) / 256.0; b = (vi & 255) / 256.0;
      auto tx = [&](uint32_t uu, uint32_t vv) {
        uint32_t const pw = uint32_t(w), ph = uint32_t(hh);
        uint32_t const ux = uu < pw ? uu : 2 * pw - 1 - uu, vy = vv < ph ? vv : 2 * ph - 1 - vv;
        return decode(raw_texel(c, int(t.page), 0, int(t.tex_x * 32 + ux), int(t.tex_y * 32 + vy)));
      };
      q[0] = tx(u1, v1); q[1] = tx(u2, v1); q[2] = tx(u1, v2); q[3] = tx(u2, v2);
    }
    if (c.on(SM_ALPHA) && alpha_test) {   // colour dilation from opaque neighbours
      auto take = [&](col4 const &s, col4 &d) { if (s.a > d.a) { d.r = s.r; d.g = s.g; d.b = s.b; } };
      take(q[0], q[1]); take(q[0], q[2]);
      take(q[1], q[0]); take(q[1], q[3]);
      take(q[2], q[0]); take(q[2], q[3]);
      take(q[3], q[2]); take(q[3], q[1]);
    }
    return mix(mix(q[0], q[1], a), mix(q[2], q[3], a), b);
  };

  col4 tc;
  if (c.on(SM_MIP)) {
    double const levels = std::floor(std::log2(double(std::min(w, hh)))) - 1.0;
    double const fl = std::clamp(lod, 0.0, levels);
    int const il = int(fl);
    tc = bilinear(il);
    double const fr = fl - il;
    if (fr > 0) tc = mix(tc, bilinear(il + 1), fr);
  } else tc = bilinear(0);
  c.st.texels += c.on(SM_MIP) ? 8 : 4;

  if (c.on(SM_DECODE) && ((h[6] >> 24) & 0x7f) == 2) { tc.r = 1 - tc.r; tc.g = 1 - tc.g; tc.b = 1 - tc.b; }
  if (c.on(SM_ALPHA)) {
    if (alpha_test && tc.a < 32.0 / 255.0) { out.discard = true; return out; }
    if (tex_alpha && opaque_pass && tc.a < 1.0) { out.discard = true; return out; }
    if (!tex_alpha && !alpha_test) tc.a = 1.0;
  }
  out.c = tc;
  return out;
}

} // namespace

void render_sm(rframe const &f, texram const &tex, uint32_t feat, rimage &img, rstats &st) {
  img = rimage();
  fbuf fb;
  ctx c{f, tex, feat, fb, st};
  // MAME form for the parts switched off: alpha flag 0 = not drawn
  for (auto &p : fb.c) p = col4{};

  // scroll fog behind the scene (New3D DrawScrollFog), the lowest priority's
  // highest scroll-fog value
  if (c.on(SM_FOG)) {
    int best = -1;
    for (int pri = 0; pri < 4 && best < 0; pri++)
      for (size_t i = 0; i < f.vps.size(); i++) {
        auto const &v = f.vps[i];
        if (int((v.node[0] >> 3) & 3) != pri) continue;
        double const sf = (v.node[0x20] & 0xff) / 255.0;
        if (sf > 0 && (best < 0 || (f.vps[size_t(best)].node[0x20] & 0xff) < (v.node[0x20] & 0xff))) best = int(i);
      }
    if (best >= 0) {
      auto const &v = f.vps[size_t(best)];
      bool on = false;
      for (auto const &n : f.vps)
        if ((n.node[0x22] & 0xffffff) == (v.node[0x22] & 0xffffff)) {
          double const dens = std::fabs(double(uf(n.node[0x23])));
          double const start = double(int16_t(n.node[0x25] & 0xffff)) / 255.0;
          if ((dens > 0 && dens < 99999.0) || start > 0 || (n.node[0x24] & 0xff)) { on = true; break; }
        }
      if (on) {
        double const amb = ((v.node[0x25] >> 16) & 255) / 255.0;
        col4 const fc{((v.node[0x22] >> 16) & 255) / 255.0 * amb, ((v.node[0x22] >> 8) & 255) / 255.0 * amb,
                      (v.node[0x22] & 255) / 255.0 * amb, (v.node[0x20] & 255) / 255.0};
        int const x0 = std::max(0, int(v.x)), x1 = std::min(SCR_W, int(v.x + v.w + 0.5f));
        int const y0 = std::max(0, int(v.y)), y1 = std::min(SCR_H, int(v.y + v.h + 0.5f));
        for (int y = y0; y < y1; y++)
          for (int x = x0; x < x1; x++) fb.c[size_t(y) * SCR_W + size_t(x)] = fc;
      }
    }
  }

  rtl_opts const o;
  for (uint32_t pass = 0; pass < 4; pass++) {
    std::vector<int32_t> op, al;
    for (size_t i = 0; i < f.tris.size(); i++)
      if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int32_t(i));
    if (op.empty() && al.empty()) continue;
    std::sort(op.begin(), op.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf < f.tris[y].buf; });
    std::sort(al.begin(), al.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf > f.tris[y].buf; });
    // Supermodel: layered polygons after the rest of their viewport
    auto layered = [&](int32_t i) { return c.on(SM_LAYER) && (f.tris[size_t(i)].hdr[6] & 8); };
    auto order = [&](std::vector<int32_t> &v) {
      if (!c.on(SM_LAYER)) return;
      std::stable_sort(v.begin(), v.end(), [&](int32_t a, int32_t b) {
        auto const &ta = f.tris[size_t(a)], &tb = f.tris[size_t(b)];
        if (ta.vp != tb.vp) return false;
        return !layered(a) && layered(b);
      });
    };
    order(op);
    order(al);

    for (int sub = 0; sub < (c.on(SM_PRIO) ? 2 : 1); sub++) {
      bool const high = sub == 1;
      std::fill(fb.z.begin(), fb.z.end(), c.on(SM_DEPTH) ? 0.0 : 1e10);
      std::fill(fb.st.begin(), fb.st.end(), 0);
      // the triangles of a layer, MAME's draw order within it
      auto draw = [&](std::vector<int32_t> const &lst, layer ly) {
        for (int32_t idx : lst) {
          rtri const &t = f.tris[size_t(idx)];
          if (c.on(SM_PRIO) && (((t.hdr[6] & 0x10) != 0) != high)) continue;
          bool const tx = (t.param & TP_TEXTURE) != 0;
          bool const tex_alpha = tx && (t.hdr[6] & 7);
          bool const poly_alpha = (t.hdr[6] & 0x800000) == 0;
          bool const lsel = (t.hdr[6] & 0x20000) != 0;
          if (c.on(SM_TRANS)) {
            if (ly == layer::colour && poly_alpha) continue;
            if (ly == layer::trans1 && ((!tex_alpha && !poly_alpha) || lsel)) continue;
            if (ly == layer::trans2 && ((!tex_alpha && !poly_alpha) || !lsel)) continue;
          }
          rtl_tri r;
          if (!r.setup(t, kind_of(t), o)) continue;
          // double planes: 1/w, u/w, v/w (texels x 256), shade, w
          double xs[3], ys[3], oz[3], uz[3], vz[3], sh[3], ww[3];
          for (int k = 0; k < 3; k++) {
            xs[k] = t.x[k]; ys[k] = t.y[k];
            oz[k] = 1.0 / double(t.w[k]); uz[k] = t.p0[k]; vz[k] = t.p1[k]; sh[k] = t.p2[k]; ww[k] = t.w[k];
          }
          dplane const pO = fit(xs, ys, oz), pU = fit(xs, ys, uz), pV = fit(xs, ys, vz), pS = fit(xs, ys, sh), pW = fit(xs, ys, ww);
          rvp const *vp = t.vp >= 0 ? &f.vps[size_t(t.vp)] : nullptr;
          double const kz = vp ? -double(vp->proj[11]) : 2.0;   // w = kz * view depth
          // view-space face normal, and the view position of a pixel, for the
          // back-face test and the LOD (Supermodel's CalcBackFace)
          double n[3] = {0, 0, 0};
          {
            float const nm[3] = {float(int32_t(t.hdr[1]) >> 8) / 4194304.0f, float(int32_t(t.hdr[2]) >> 8) / 4194304.0f,
                                 float(int32_t(t.hdr[3]) >> 8) / 4194304.0f};
            if (t.model >= 0) {
              float const *m = f.models[size_t(t.model)].top;
              static int const cv = std::getenv("R3D_CULLV") ? std::atoi(std::getenv("R3D_CULLV")) : 0;
              for (int j = 0; j < 3; j++)
                n[j] = (cv & 1) ? nm[0] * m[j * 4 + 0] + nm[1] * m[j * 4 + 1] + nm[2] * m[j * 4 + 2]
                                : nm[0] * m[0 * 4 + j] + nm[1] * m[1 * 4 + j] + nm[2] * m[2 * 4 + j];
              if (cv & 2) for (double &q : n) q = -q;
              // MAME's view space is the model matrix times its coordinate matrix
              if (vp && !(cv & 4)) { n[0] *= vp->coord[1]; n[1] *= vp->coord[6]; n[2] *= -vp->coord[8]; }
            }
          }
          auto view_of = [&](double x, double y, double w, double v3[3]) {
            double const xn = ((x - vp->x) / vp->w - 0.5) * 2.0, yn = ((y - vp->y) / vp->h - 0.5) * 2.0;
            v3[0] = xn * w / double(vp->proj[0]);
            v3[1] = yn * w / double(vp->proj[5]);
            v3[2] = -w / kz;
          };
          if (c.on(SM_CULL) && vp && !(t.hdr[1] & 0x10)) {
            double v3[3];
            view_of(t.x[0], t.y[0], t.w[0], v3);
            if (v3[0] * n[0] + v3[1] * n[1] + v3[2] * n[2] > 0) { st.kind_tris[0]++; continue; }
          }
          col4 const face = {((t.color >> 16) & 255) / 255.0, ((t.color >> 8) & 255) / 255.0, (t.color & 255) / 255.0, 1.0};
          double alpha = 1.0;
          if (c.on(SM_TRANS)) {
            alpha = (t.hdr[6] & 0x800000) ? 1.0 : double((((t.hdr[6] >> 18) & 0x3f) * 255) / 32) / 255.0;
            if ((t.hdr[0] & 0x200) && !(t.hdr[0] & 0x100)) alpha = double(int(alpha * 255.0 + 0.5) / 2) / 255.0;
          }
          bool const lay = layered(idx);
          double const fogint = ((t.hdr[6] >> 11) & 31) / 16.0;
          double fog_r = 0, fog_g = 0, fog_b = 0, dens = 0, start = 0;
          if (vp && c.on(SM_FOG)) {
            double const amb = ((vp->node[0x25] >> 16) & 255) / 255.0;
            fog_r = ((vp->node[0x22] >> 16) & 255) / 255.0 * amb;
            fog_g = ((vp->node[0x22] >> 8) & 255) / 255.0 * amb;
            fog_b = (vp->node[0x22] & 255) / 255.0 * amb;
            dens = std::fabs(double(uf(vp->node[0x23])));
            if (!(dens < 99999.0)) dens = 0;
            start = double(int16_t(vp->node[0x25] & 0xffff)) / 255.0;
          }
          double const cota = vp ? double(uf(vp->node[3])) : 0.0;
          double const np = double(t.hdr[5] >> 8) / 4096.0;
          rkind const k = kind_of(t);
          for (int32_t y = r.y0; y < r.y1; y++) {
            int32_t x0, x1;
            if (!r.span(y, x0, x1)) continue;
            st.spans++;
            for (int x = x0; x < x1; x++) {
              size_t const i = size_t(y) * SCR_W + size_t(x);
              double const px = x + 0.5, py = y + 0.5;
              double const ooz = pO.at(px, py);
              st.pixels++;
              // depth
              double z;
              bool pass_z;
              if (c.on(SM_DEPTH)) {
                z = ooz * (lay ? 1.0 + 1.0 / 512.0 : 1.0);
                pass_z = (ly == layer::colour) ? z >= fb.z[i] : z > fb.z[i];
              } else {
                z = pW.at(px, py);
                pass_z = z <= fb.z[i];
              }
              if (!pass_z) continue;
              if (lay && fb.st[i] != 0) continue;
              // colour
              col4 tc{1, 1, 1, 1};
              double const w_pix = 1.0 / ooz;
              if (tx) {
                double const u = pU.at(px, py) * w_pix / 256.0, v = pV.at(px, py) * w_pix / 256.0;
                double lod = 0;
                if (c.on(SM_MIP) && vp) {
                  double v3[3];
                  view_of(px, py, w_pix, v3);
                  double const dotvn = v3[0] * n[0] + v3[1] * n[1] + v3[2] * n[2];
                  double const fragw = kz * ooz;   // 1 / view depth
                  double const lb = dotvn * -cota * np;
                  lod = (lb > 0) ? -std::log2(fragw * fragw * lb) : 0.0;
                }
                texsamp const ts = sm_texture(c, t, u, v, lod, ly == layer::colour);
                if (ts.discard) continue;
                tc = ts.c;
                if (c.on(SM_TRANS) && tex_alpha && ly != layer::colour && tc.a * alpha >= 1.0) continue;
              }
              col4 out;
              bool const modulate = c.on(SM_COLOUR) || !tx || (t.param & (TP_COLORMOD | TP_CONTOUR));
              out.r = tc.r * (modulate ? face.r : 1.0);
              out.g = tc.g * (modulate ? face.g : 1.0);
              out.b = tc.b * (modulate ? face.b : 1.0);
              out.a = tc.a * alpha;
              if (c.on(SM_ALPHA) && out.a < 1.0 / 32.0) continue;
              if (!c.on(SM_ALPHA)) {
                // MAME's alpha rules
                if (k == K_CONTOUR && tc.a < 248.0 / 255.0) continue;
                if (k == K_TEX_ALPHA && tc.a == 0) continue;
              }
              static bool const lightmax = std::getenv("R3D_LIGHTMAX") != nullptr;   // experiment: shade forced to full
              double const li = lightmax ? 1.0 : std::clamp(double(int32_t(pS.at(px, py))), 0.0, 255.0) / 255.0;
              out.r = std::min(out.r * li, 1.0); out.g = std::min(out.g * li, 1.0); out.b = std::min(out.b * li, 1.0);
              if (c.on(SM_FOG) && vp) {
                double const depth = w_pix / kz;
                double const fg = fogint * std::clamp(start + depth * dens, 0.0, 1.0);
                out.r += (fog_r - out.r) * fg; out.g += (fog_g - out.g) * fg; out.b += (fog_b - out.b) * fg;
              }
              st.zpass++;
              if (lay) fb.st[i] = 1;
              if (ly == layer::colour) {
                fb.z[i] = z;
                if (c.on(SM_TRANS)) {
                  out.a = 1.0;
                  fb.c[i] = out; fb.t1[i] = col4{}; fb.t2[i] = col4{};
                } else if (k == K_CONTOUR) {
                  col4 const d = fb.c[i];
                  double const fa = tc.a;
                  fb.c[i] = {out.r * fa + d.r * (1 - fa), out.g * fa + d.g * (1 - fa), out.b * fa + d.b * (1 - fa), 1.0};
                } else {
                  out.a = 1.0;
                  fb.c[i] = out;
                }
              } else if (ly == layer::trans1) { fb.z[i] = z; fb.t1[i] = out; }
              else if (ly == layer::trans2) { fb.z[i] = z; fb.t2[i] = out; }
              else {
                // MAME: blend into the colour buffer, no Z write
                double fa;
                if (k == K_TEX_ALPHA) fa = tc.a;
                else { int const tr = int((t.trans << 3) | (t.trans >> 2)); fa = (tr == 0xff) ? 1.0 : double(uint8_t(tr)) / 256.0; }
                col4 const d = fb.c[i];
                fb.c[i] = {out.r * fa + d.r * (1 - fa), out.g * fa + d.g * (1 - fa), out.b * fa + d.b * (1 - fa), 1.0};
              }
              img.kind[i] = uint8_t(k);
              img.tri[i] = idx;
              st.written++;
            }
          }
        }
      };
      draw(op, layer::colour);
      if (c.on(SM_TRANS)) {
        std::vector<int32_t> all = op;
        all.insert(all.end(), al.begin(), al.end());
        fb.zsave = fb.z;
        draw(all, layer::trans1);
        fb.z = fb.zsave;
        draw(all, layer::trans2);
      } else {
        draw(al, layer::mame_alpha);
      }
    }
  }
  // composite: the colour layer, then the translucency layers over it
  // (R3DFrameBuffers::Draw), as Supermodel's back buffer holds them over black
  for (size_t i = 0; i < fb.c.size(); i++) {
    col4 base = fb.c[i];
    double r = base.r * base.a, g = base.g * base.a, b = base.b * base.a, a = base.a;
    if (c.on(SM_TRANS)) {
      col4 const &p = fb.t1[i], &q = fb.t2[i];
      col4 tl;
      if (p.a * q.a > 0) { tl = mix(p, q, (q.a + (1.0 - p.a)) / 2.0); tl.a = 1.0; }
      else if (p.a > 0) tl = p;
      else tl = q;
      r = tl.r * tl.a + r * (1 - tl.a); g = tl.g * tl.a + g * (1 - tl.a); b = tl.b * tl.a + b * (1 - tl.a);
      a = tl.a + a * (1 - tl.a);
    }
    auto q8 = [](double v) { return uint32_t(std::clamp(std::lround(v * 255.0), 0L, 255L)); };
    img.px[i] = (q8(a) << 24) | (q8(r) << 16) | (q8(g) << 8) | q8(b);
  }
}

} // namespace r3d
