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
// Real3D rasteriser model. See r3d_rast.h for the two arithmetic forms.
// Build with -ffp-contract=off: MAME's x86-64 build has no FMA, and a fused
// multiply-add would round differently from it.

#include "r3d_rast.h"
#include "r3d_trilist.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace r3d {

char const *kind_name(int k) {
  static char const *const n[K_COUNT] = {"none", "solid", "solid_trans", "tex", "tex_colormod", "contour", "tex_alpha"};
  return k >= 0 && k < K_COUNT ? n[k] : "?";
}

bool read_rframe(std::string const &fn, rframe &out) {
  std::vector<record> recs;
  if (!read_trilist(fn, recs)) return false;
  out.tris.clear();
  out.vps.clear();
  out.models.clear();
  std::vector<std::array<uint32_t, 7>> polyhdr;
  for (auto const &r : recs) {
    auto f = [&](size_t i) { float x; std::memcpy(&x, &r.w[i], 4); return x; };
    if (r.tag == 'V' && r.w.size() >= 25) {
      rvp v;
      v.pass = r.w[0]; v.addr = r.w[1];
      v.x = f(3); v.y = f(4); v.w = f(5); v.h = f(6); v.near = f(7); v.far = f(8);
      for (int k = 0; k < 16; k++) v.proj[k] = f(size_t(9 + k));
      for (int k = 0; k < 16; k++) v.coord[k] = f(size_t(25 + k));
      out.vps.push_back(v);
      continue;
    }
    if (r.tag == 'M' && r.w.size() >= 20) {
      rmodel m;
      for (int k = 0; k < 16; k++) m.top[k] = f(size_t(4 + k));
      out.models.push_back(m);
      continue;
    }
    if (r.tag == 'P' && r.w.size() >= 10) {
      std::array<uint32_t, 7> h;
      for (int k = 0; k < 7; k++) h[size_t(k)] = r.w[size_t(3 + k)];
      polyhdr.push_back(h);
      continue;
    }
    if (r.tag != 'T' || r.w.size() < 35) continue;
    rtri t;
    t.pass = r.w[0]; t.alpha = r.w[1]; t.buf = r.w[2]; t.poly = r.w[3];
    for (int k = 0; k < 3; k++) {
      size_t const b = 4 + 7 * k;
      t.x[k] = f(b); t.y[k] = f(b + 1); t.z[k] = f(b + 2); t.w[k] = f(b + 3);
      t.p0[k] = f(b + 4); t.p1[k] = f(b + 5); t.p2[k] = f(b + 6);
    }
    t.tex_en = r.w[25]; t.page = r.w[26]; t.tex_x = r.w[27]; t.tex_y = r.w[28];
    t.tex_w = r.w[29]; t.tex_h = r.w[30]; t.fmt = r.w[31]; t.param = r.w[32];
    t.trans = r.w[33]; t.color = r.w[34];
    if (t.poly < polyhdr.size()) for (int k = 0; k < 7; k++) t.hdr[k] = polyhdr[t.poly][size_t(k)];
    t.vp = int32_t(out.vps.size()) - 1;
    t.model = int32_t(out.models.size()) - 1;
    out.tris.push_back(t);
  }
  return true;
}

// ---- texels ------------------------------------------------------------------

namespace {
inline uint32_t pal1(uint32_t b) { return (b & 1) ? 0xff : 0; }
inline uint32_t pal4(uint32_t b) { b &= 0xf; return (b << 4) | b; }
inline uint32_t pal5(uint32_t b) { b &= 0x1f; return (b << 3) | (b >> 2); }
inline uint32_t argb(uint32_t a, uint32_t r, uint32_t g, uint32_t b) {
  return (a << 24) | (r << 16) | (g << 8) | b;
}
} // namespace

// MAME get_texture: the texture is decoded with its mirror images, so (u, v)
// in the second half of either axis reads the first half reflected. The
// source address is linear in the page, as MAME's pointer arithmetic is: a
// texture that runs past x = 2047 continues on the next row, and one that
// runs past row 1023 reads outside the page (MAME reads past its array;
// counted, and read from the dump's next page here).
uint32_t texram::texel(uint32_t page, uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th, uint32_t fmt,
                       uint32_t u, uint32_t v, bool *outside) const {
  uint32_t const pw = 32u << tw, ph = 32u << th;
  uint32_t const uu = u < pw ? u : 2 * pw - 1 - u;
  uint32_t const vv = v < ph ? v : 2 * ph - 1 - v;
  uint32_t const a = (ty * 32 + vv) * 2048 + tx * 32 + uu;
  if (outside) *outside = a >= 2048u * 1024u;
  return mame_decode(int(fmt & 7), t[(page * 2048u * 1024u + a) & (2048u * 2048u - 1)]);
}

// MAME get_texture's decode of one 16-bit texel, ARGB8888
uint32_t mame_decode(int fmt, uint32_t p) {
  switch (fmt & 7) {
  case 0: return argb(pal1(~p >> 15), pal5(p >> 10), pal5(p >> 5), pal5(p));
  case 1:
  case 2: { uint32_t const g = pal4(p); return argb(pal4(p >> 4), g, g, g); }
  case 3: { uint32_t const g = pal4(p >> 8); return argb(pal4(p >> 12), g, g, g); }
  case 4: { uint32_t const d = (p >> 8) & 0xff; uint32_t const g = pal4(d >> 4); return argb(pal4(d), g, g, g); }
  case 5: { uint32_t const g = p & 0xff; return argb(0xff, g, g, g); }
  case 6: { uint32_t const g = (p >> 8) & 0xff; return argb(0xff, g, g, g); }
  default: return argb(pal4(p), pal4(p >> 12), pal4(p >> 8), pal4(p >> 4));
  }
}

int span_group(rkind k) { return (k == K_SOLID || k == K_TEX || k == K_TEX_COLORMOD) ? 0 : k == K_CONTOUR ? 1 : 2; }

void count_span(rkind k, int len, rstats &st) {
  if (len <= 0) return;
  int const g = span_group(k);
  st.cyc[g][0] += uint64_t(len);
  st.cyc[g][1] += uint64_t((len + 1) / 2);
  st.cyc[g][2] += uint64_t((len + 3) / 4);
}

namespace {

// ---- MAME's integer helpers (rgbutil.h generic, rgbutil.cpp SSE2) -------------

struct rgba { int32_t a, r, g, b; };
inline rgba unpack(uint32_t c) { return {int32_t((c >> 24) & 0xff), int32_t((c >> 16) & 0xff), int32_t((c >> 8) & 0xff), int32_t(c & 0xff)}; }
inline int32_t clamp8(int32_t x) { return x < 0 ? 0 : x > 255 ? 255 : x; }
inline int32_t mul32(int32_t a, int32_t b) { return int32_t(uint32_t(a) * uint32_t(b)); }  // s32 wrap, as the x86 multiply
inline void scale_imm(rgba &c, int32_t s) {
  c.a = clamp8(mul32(c.a, s) >> 8); c.r = clamp8(mul32(c.r, s) >> 8);
  c.g = clamp8(mul32(c.g, s) >> 8); c.b = clamp8(mul32(c.b, s) >> 8);
}
inline void scale_rgba(rgba &c, rgba const &s) {
  c.a = clamp8((c.a * s.a) >> 8); c.r = clamp8((c.r * s.r) >> 8);
  c.g = clamp8((c.g * s.g) >> 8); c.b = clamp8((c.b * s.b) >> 8);
}
inline void blend(rgba &c, rgba const &o, uint8_t f) {
  int32_t const s1 = f, s2 = 256 - s1;
  c.a = (c.a * s1 + o.a * s2) >> 8; c.r = (c.r * s1 + o.r * s2) >> 8;
  c.g = (c.g * s1 + o.g * s2) >> 8; c.b = (c.b * s1 + o.b * s2) >> 8;
}
inline uint32_t pack_clamp(rgba const &c) {
  return (uint32_t(clamp8(c.a)) << 24) | (uint32_t(clamp8(c.r)) << 16) | (uint32_t(clamp8(c.g)) << 8) | uint32_t(clamp8(c.b));
}
// rgbaint_t::bilinear_filter, SSE2: the u lerp at full width, halved, then the
// v lerp and >> 15
inline uint32_t bilinear(uint32_t c00, uint32_t c01, uint32_t c10, uint32_t c11, uint32_t u, uint32_t v) {
  uint32_t r = 0;
  for (int sh = 0; sh < 32; sh += 8) {
    uint32_t const a = (c00 >> sh) & 0xff, b = (c01 >> sh) & 0xff, c = (c10 >> sh) & 0xff, d = (c11 >> sh) & 0xff;
    uint32_t const top = a * (256 - u) + b * u, bot = c * (256 - u) + d * u;
    uint32_t const o = ((top >> 1) * (256 - v) + (bot >> 1) * v) >> 15;
    r |= (o > 255 ? 255 : o) << sh;
  }
  return r;
}

// x86-64 conversions as MAME's build emits them: (int) is cvttss2si into 32 bits
// (out of range or NaN gives INT_MIN); (uint32_t) is cvttss2si into 64 bits and
// the low half (out of range or NaN gives 0)
inline int32_t f2i32(float f) {
  if (!(f >= -2147483648.0f && f < 2147483648.0f)) return std::numeric_limits<int32_t>::min();
  return int32_t(f);
}
inline uint32_t f2u32(float f) {
  if (!(f >= -9223372036854775808.0f && f < 9223372036854775808.0f)) return 0;
  return uint32_t(uint64_t(int64_t(f)));
}
// poly.h round_coordinate
inline int32_t round_coord(float v) {
  if (v >= float(std::numeric_limits<int32_t>::max())) return std::numeric_limits<int32_t>::max();
  float const ip = std::floor(v);
  if (ip < float(std::numeric_limits<int32_t>::min())) return std::numeric_limits<int32_t>::min();
  float const fp = v - ip;
  return f2i32(ip) + ((fp > 0.5f) ? 1 : 0);
}

struct pvert { float x, y, p[5]; };

// One span of one triangle as the pixel loop receives it
struct span {
  int y, x0, x1;
  float start[5], dpdx[5];
};

// ---- MAME: poly.h render_triangle -------------------------------------------

template <int N, class F>
void tri_mame(pvert const &a, pvert const &b, pvert const &c, F &&emit) {
  pvert const *v1 = &a, *v2 = &b, *v3 = &c;
  if (v2->y < v1->y) std::swap(v1, v2);
  if (v3->y < v2->y) {
    std::swap(v2, v3);
    if (v2->y < v1->y) std::swap(v1, v2);
  }
  int32_t const v1y = round_coord(v1->y), v3y = round_coord(v3->y);
  int32_t const y0 = std::max(v1y, 0), y1 = std::min(v3y, SCR_H);
  if (y1 - y0 <= 0) return;

  float const dxdy_v1v2 = (v2->y == v1->y) ? 0.0f : (v2->x - v1->x) / (v2->y - v1->y);
  float const dxdy_v1v3 = (v3->y == v1->y) ? 0.0f : (v3->x - v1->x) / (v3->y - v1->y);
  float const dxdy_v2v3 = (v3->y == v2->y) ? 0.0f : (v3->x - v2->x) / (v3->y - v2->y);

  float ps[N], pdx[N], pdy[N];
  float const a00 = v2->y - v3->y, a01 = v3->x - v2->x, a02 = v2->x * v3->y - v3->x * v2->y;
  float const a10 = v3->y - v1->y, a11 = v1->x - v3->x, a12 = v3->x * v1->y - v1->x * v3->y;
  float const a20 = v1->y - v2->y, a21 = v2->x - v1->x, a22 = v1->x * v2->y - v2->x * v1->y;
  float const det = a02 + a12 + a22;
  if (std::abs(det) < 0.00001f) {
    for (int i = 0; i < N; i++) { pdx[i] = 0.0f; pdy[i] = 0.0f; ps[i] = v1->p[i]; }
  } else {
    float const idet = 1.0f / det;
    for (int i = 0; i < N; i++) {
      pdx[i] = idet * (v1->p[i] * a00 + v2->p[i] * a10 + v3->p[i] * a20);
      pdy[i] = idet * (v1->p[i] * a01 + v2->p[i] * a11 + v3->p[i] * a21);
      ps[i] = idet * (v1->p[i] * a02 + v2->p[i] * a12 + v3->p[i] * a22);
    }
  }
  for (int32_t y = y0; y < y1; y++) {
    float const fully = float(y) + 0.5f;
    float const sx = v1->x + (fully - v1->y) * dxdy_v1v3;
    float const ex = (fully < v2->y) ? v1->x + (fully - v1->y) * dxdy_v1v2 : v2->x + (fully - v2->y) * dxdy_v2v3;
    int32_t is = round_coord(sx), ie = round_coord(ex);
    if (is > ie) std::swap(is, ie);
    is = std::max(is, 0);
    ie = std::min(ie, SCR_W);
    if (is >= ie) is = ie = 0;
    span s;
    s.y = y; s.x0 = is; s.x1 = ie;
    float const fsx = float(is) + 0.5f;
    for (int i = 0; i < N; i++) {
      s.start[i] = ps[i] + fsx * pdx[i] + fully * pdy[i];
      s.dpdx[i] = pdx[i];
    }
    emit(s);
  }
}

// ---- the pixel pipe ----------------------------------------------------------

struct target {
  rimage &img;
  std::vector<float> &zb;
  rstats &st;
};

struct tri_ctx {
  rtri const *t;
  int32_t idx;
  rkind kind;
  texram const *tex;
};

// Interpolant source for one span: MAME accumulates each parameter with a
// float add per pixel, from the span's start.
struct acc_mame {
  float v[5], d[5];
  int n;
  acc_mame(span const &s, int np) : n(np) { for (int i = 0; i < n; i++) { v[i] = s.start[i]; d[i] = s.dpdx[i]; } }
  void step() { for (int i = 0; i < n; i++) v[i] += d[i]; }
};

// Texel fetch, MAME's TEX_FETCH_BILINEAR
inline uint32_t fetch_mame(tri_ctx const &c, float ooz, float uoz, float voz, rstats &st, bool &outside) {
  rtri const &t = *c.t;
  float const intz = 1.0f / ooz;
  uint32_t const u = f2u32(uoz * intz), v = f2u32(voz * intz);
  uint32_t const umask = (((t.param & TP_MIRROR_U) ? 64u : 32u) << t.tex_w) - 1;
  uint32_t const vmask = (((t.param & TP_MIRROR_V) ? 64u : 32u) << t.tex_h) - 1;
  uint32_t const u1 = (u >> 8) & umask, v1 = (v >> 8) & vmask;
  uint32_t const u2 = (u1 + 1) & umask, v2 = (v1 + 1) & vmask;
  bool o0, o1, o2, o3;
  uint32_t const p00 = c.tex->texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u1, v1, &o0);
  uint32_t const p01 = c.tex->texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u2, v1, &o1);
  uint32_t const p10 = c.tex->texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u1, v2, &o2);
  uint32_t const p11 = c.tex->texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u2, v2, &o3);
  st.texels += 4;
  st.tex_outside += o0 + o1 + o2 + o3;
  outside = o0 || o1 || o2 || o3;
  return bilinear(p00, p01, p10, p11, u & 0xff, v & 0xff);
}

// Colour and blend for a pixel that passed Z, per draw kind. Returns false if
// nothing is written (alpha test). zw says whether Z is written.
inline bool shade_px(tri_ctx const &c, uint32_t texel, float in, uint32_t dst, uint32_t &out, bool &zw) {
  rtri const &t = *c.t;
  int32_t const ii = f2i32(in);
  rgba col;
  switch (c.kind) {
  case K_SOLID:
    col = unpack(t.color); scale_imm(col, ii); zw = true; break;
  case K_SOLID_TRANS: {
    col = unpack(t.color); scale_imm(col, ii);
    int32_t const tr = int32_t((t.trans << 3) | (t.trans >> 2));
    if (tr != 0xff) blend(col, unpack(dst), uint8_t(tr));
    zw = false; break;
  }
  case K_TEX:
    col = unpack(texel); scale_imm(col, ii); zw = true; break;
  case K_TEX_COLORMOD:
    col = unpack(texel); scale_rgba(col, unpack(t.color)); scale_imm(col, ii); zw = true; break;
  case K_CONTOUR: {
    uint32_t const fa = texel >> 24;
    if (fa < 0xf8) return false;
    col = unpack(texel); scale_rgba(col, unpack(t.color)); scale_imm(col, ii);
    blend(col, unpack(dst), uint8_t(fa));
    zw = true; break;
  }
  case K_TEX_ALPHA: {
    uint32_t const fa = texel >> 24;
    if (fa == 0) return false;
    col = unpack(texel); scale_imm(col, ii);
    blend(col, unpack(dst), uint8_t(fa));
    zw = false; break;
  }
  default: return false;
  }
  out = 0xff000000u | pack_clamp(col);
  return true;
}

// R3D_PIX=x,y traces every Z-passing write to that pixel in the MAME form
int g_dbg_x = std::getenv("R3D_PIX") ? std::atoi(std::getenv("R3D_PIX")) : -1;
int g_dbg_y = std::getenv("R3D_PIX") && std::strchr(std::getenv("R3D_PIX"), ',') ? std::atoi(std::strchr(std::getenv("R3D_PIX"), ',') + 1) : -1;

void draw_span_mame(tri_ctx const &c, span const &s, target &tg) {
  bool const tex = c.kind >= K_TEX;
  count_span(c.kind, s.x1 - s.x0, tg.st);
  acc_mame p(s, tex ? 5 : 2);
  float *const zb = &tg.zb[size_t(s.y) * SCR_W];
  uint32_t *const fb = &tg.img.px[size_t(s.y) * SCR_W];
  tg.st.spans++;
  for (int x = s.x0; x < s.x1; x++) {
    float const z = p.v[0];
    tg.st.pixels++;
    tg.st.px_kind[c.kind]++;
    size_t const i = size_t(s.y) * SCR_W + x;
    uint8_t &tn = tg.img.taint[i];
    bool const zt = tn & 2;   // the Z compared against is uncertain
    if (z <= zb[x]) {
      tg.st.zpass++;
      tg.st.zpass_g[span_group(c.kind)]++;
      uint32_t texel = 0;
      bool outside = false;
      if (tex) { texel = fetch_mame(c, p.v[1], p.v[2], p.v[3], tg.st, outside); tg.st.fmt_px[c.t->fmt & 7]++; }
      uint32_t out;
      bool zw = false;
      bool const blends = c.kind == K_SOLID_TRANS || c.kind == K_CONTOUR || c.kind == K_TEX_ALPHA;
      bool const wr = shade_px(c, texel, tex ? p.v[4] : p.v[1], fb[x], out, zw);
      bool const unsure = zt || outside;
      bool const may_zw = c.kind == K_SOLID || c.kind == K_TEX || c.kind == K_TEX_COLORMOD || c.kind == K_CONTOUR;
      if (g_dbg_x == x && g_dbg_y == s.y)
        std::printf("pix (%d,%d) tri %d kind %s z %.9g zb %.9g in %.9g texel %08x dst %08x -> %s %08x\n", x, s.y, c.idx, kind_name(c.kind),
                    double(z), double(zb[x]), double(tex ? p.v[4] : p.v[1]), texel, fb[x], wr ? "write" : "skip", wr ? out : 0);
      if (wr) {
        fb[x] = out;
        if (zw) zb[x] = z;
        tg.img.kind[i] = c.kind;
        tg.img.fmt[i] = tex ? uint8_t(c.t->fmt & 7) : 0xff;
        tg.img.tri[i] = c.idx;
        tg.st.written++;
        // colour: certain only if this write is certain and does not blend a tainted pixel
        tn = uint8_t((tn & ~1) | ((unsure || (blends && (tn & 1))) ? 1 : 0));
        if (zw) tn = uint8_t((tn & ~2) | (unsure ? 2 : 0));
      } else if (unsure) {
        tn |= 1;
        if (may_zw) tn |= 2;
      }
    } else if (zt) {
      tn |= 1;   // MAME may have passed this test
      tn |= 2;
    }
    p.step();
  }
}

// The vertex parameters as MAME's draw_*_triangles builds them
void make_verts(rtri const &t, bool tex, pvert v[3]) {
  for (int i = 0; i < 3; i++) {
    v[i].x = t.x[i];
    v[i].y = t.y[i];
    v[i].p[0] = t.w[i];
    if (tex) {
      v[i].p[1] = 1.0f / t.w[i];
      v[i].p[2] = t.p0[i];
      v[i].p[3] = t.p1[i];
      v[i].p[4] = t.p2[i];
    } else {
      v[i].p[1] = t.p2[i];
      v[i].p[2] = v[i].p[3] = v[i].p[4] = 0.0f;
    }
  }
}

} // namespace

rkind kind_of(rtri const &t) {
  if (t.param & TP_TEXTURE) {
    if (t.alpha) return K_TEX_ALPHA;
    if (t.param & TP_CONTOUR) return K_CONTOUR;
    return (t.param & TP_COLORMOD) ? K_TEX_COLORMOD : K_TEX;
  }
  return t.alpha ? K_SOLID_TRANS : K_SOLID;
}

// Defined in r3d_rast_rtl.cpp
void draw_tri_rtl(rtri const &t, int32_t idx, rkind kind, texram const &tex, rtl_opts const &o,
                  rimage &img, std::vector<uint32_t> &zb, rstats &st);
void clear_z_rtl(std::vector<uint32_t> &zb, rtl_opts const &o);

void render(rframe const &f, texram const &tex, rarith a, rtl_opts const &o, rimage &img, rstats &st) {
  img = rimage();
  {
    // polygons: triangles of one polygon are consecutive and share its poly number
    size_t i = 0;
    while (i < f.tris.size()) {
      size_t j = i;
      float ymin = 1e30f, ymax = -1e30f;
      while (j < f.tris.size() && f.tris[j].poly == f.tris[i].poly && f.tris[j].pass == f.tris[i].pass) {
        for (int k = 0; k < 3; k++) if (std::isfinite(f.tris[j].y[k])) { ymin = std::min(ymin, f.tris[j].y[k]); ymax = std::max(ymax, f.tris[j].y[k]); }
        j++;
      }
      st.polys++;
      st.poly_verts += (j - i) + 2;
      int32_t r0 = std::max(round_coord(ymin), 0), r1 = std::min(round_coord(ymax), SCR_H);
      if (r1 > r0) {
        int const hs[3] = {8, 16, 32};
        for (int b = 0; b < 3; b++) st.poly_bands[b] += uint64_t((r1 - 1) / hs[b] - r0 / hs[b] + 1);
      }
      i = j;
    }
  }
  std::vector<float> zb(size_t(SCR_W) * SCR_H);
  std::vector<uint32_t> zr(size_t(SCR_W) * SCR_H);
  target tg{img, zb, st};
  for (uint32_t pass = 0; pass < 4; pass++) {
    std::vector<int32_t> op, al;
    for (size_t i = 0; i < f.tris.size(); i++)
      if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int32_t(i));
    if (op.empty() && al.empty()) continue;
    std::sort(op.begin(), op.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf < f.tris[y].buf; });
    std::sort(al.begin(), al.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf > f.tris[y].buf; });
    std::fill(zb.begin(), zb.end(), 10000000000.0f);
    clear_z_rtl(zr, o);
    std::vector<int32_t> const tri_before = img.tri;
    for (auto const *lst : {&op, &al})
      for (int32_t idx : *lst) {
        rtri const &t = f.tris[idx];
        rkind const k = kind_of(t);
        st.tris++;
        st.kind_tris[k]++;
        // band statistics: rows the triangle covers, from its rounded y extent
        {
          float ymin = std::min({t.y[0], t.y[1], t.y[2]}), ymax = std::max({t.y[0], t.y[1], t.y[2]});
          int32_t r0 = std::max(round_coord(ymin), 0), r1 = std::min(round_coord(ymax), SCR_H);
          if (r1 > r0) {
            int const hs[3] = {8, 16, 32};
            for (int b = 0; b < 3; b++) st.tri_bands[b] += uint64_t((r1 - 1) / hs[b] - r0 / hs[b] + 1);
          }
        }
        bool const nonfinite = !(std::isfinite(t.x[0]) && std::isfinite(t.x[1]) && std::isfinite(t.x[2]) && std::isfinite(t.y[0]) &&
                                 std::isfinite(t.y[1]) && std::isfinite(t.y[2]) && std::isfinite(t.w[0]) && std::isfinite(t.w[1]) && std::isfinite(t.w[2]));
        st.nonfinite_tris += nonfinite;
        uint64_t const written0 = st.written;
        if (a == rarith::mame) {
          pvert v[3];
          bool const tx = k >= K_TEX;
          make_verts(t, tx, v);
          tri_ctx c{&t, idx, k, &tex};
          bool any = false;
          auto emit = [&](span const &s) { any = true; draw_span_mame(c, s, tg); };
          if (tx) tri_mame<5>(v[0], v[1], v[2], emit);
          else tri_mame<2>(v[0], v[1], v[2], emit);
          st.tris_drawn += any;
        } else {
          draw_tri_rtl(t, idx, k, tex, o, img, zr, st);
        }
        if (nonfinite) st.nonfinite_px += st.written - written0;
      }
    for (size_t p = 0; p < img.tri.size(); p++)
      if (img.tri[p] != tri_before[p] && img.tri[p] >= 0) {
        rkind const k = rkind(img.kind[p]);
        if (k == K_SOLID || k == K_TEX || k == K_TEX_COLORMOD) { st.winners++; st.tex_winners += k != K_SOLID; }
      }
  }
}

// Shared with r3d_rast_rtl.cpp
bool shade_pixel(rtri const &t, rkind k, uint32_t texel, int32_t shade_int, uint32_t dst, uint32_t &out, bool &zw) {
  tri_ctx c{&t, 0, k, nullptr};
  // shade_px takes the float shade; the RTL passes an integer, so go through an exact float
  return shade_px(c, texel, float(shade_int), dst, out, zw);
}
uint32_t bilinear_filter(uint32_t c00, uint32_t c01, uint32_t c10, uint32_t c11, uint32_t u, uint32_t v) {
  return bilinear(c00, c01, c10, c11, u, v);
}


void mame_spans(rtri const &t, rkind k, std::vector<mame_span> &out) {
  out.clear();
  pvert v[3];
  bool const tx = k >= K_TEX;
  make_verts(t, tx, v);
  auto emit = [&](span const &sp) {
    mame_span m;
    m.y = sp.y; m.x0 = sp.x0; m.x1 = sp.x1;
    std::memcpy(m.start, sp.start, sizeof(m.start));
    std::memcpy(m.dpdx, sp.dpdx, sizeof(m.dpdx));
    out.push_back(m);
  };
  if (tx) tri_mame<5>(v[0], v[1], v[2], emit);
  else tri_mame<2>(v[0], v[1], v[2], emit);
}

uint32_t fetch_texel(rtri const &t, texram const &tex, uint32_t u, uint32_t v, bool &outside);

void lockstep::merge(lockstep const &o) {
  spans += o.spans; span_diff += o.span_diff; px_both += o.px_both; px_only_mame += o.px_only_mame; px_only_rtl += o.px_only_rtl;
  ztests += o.ztests; zdiff += o.zdiff; texfetch += o.texfetch; tex_addr_diff += o.tex_addr_diff;
  frac1 += o.frac1; frac4 += o.frac4; fracbig += o.fracbig; texel_diff += o.texel_diff;
  shade_n += o.shade_n; shade_diff += o.shade_diff; colour_n += o.colour_n; colour_diff += o.colour_diff;
  colour_maxd = std::max(colour_maxd, o.colour_maxd);
  for (int k = 0; k < K_COUNT; k++) { k_px[k] += o.k_px[k]; k_diff[k] += o.k_diff[k]; }
  for (int k = 0; k < 5; k++) { uv_mame[k] += o.uv_mame[k]; uv_rtl[k] += o.uv_rtl[k]; }
  sh_mame_bad += o.sh_mame_bad; sh_rtl_bad += o.sh_rtl_bad;
}

// MAME's frame state advances exactly as MAME's; the RTL's Z buffer holds the
// RTL's Z for every value MAME wrote, so the RTL's Z tests see MAME's history.
void render_lockstep(rframe const &f, texram const &tex, rtl_opts const &o, lockstep &ls) {
  std::vector<uint32_t> fb(size_t(SCR_W) * SCR_H, 0);
  std::vector<float> zb(size_t(SCR_W) * SCR_H);
  std::vector<uint32_t> zr(size_t(SCR_W) * SCR_H);
  std::vector<mame_span> ms;
  uint32_t const zmax = o.zbits >= 32 ? 0xffffffffu : (1u << o.zbits) - 1;
  auto fz = [&](float z) { uint32_t b; std::memcpy(&b, &z, 4); return (b >> 31) ? 0u : (b >> (31 - o.zbits)) & zmax; };
  rstats dummy;
  for (uint32_t pass = 0; pass < 4; pass++) {
    std::vector<int32_t> op, al;
    for (size_t i = 0; i < f.tris.size(); i++)
      if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int32_t(i));
    if (op.empty() && al.empty()) continue;
    std::sort(op.begin(), op.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf < f.tris[y].buf; });
    std::sort(al.begin(), al.end(), [&](int32_t x, int32_t y) { return f.tris[x].buf > f.tris[y].buf; });
    std::fill(zb.begin(), zb.end(), 10000000000.0f);
    std::fill(zr.begin(), zr.end(), zmax);
    for (auto const *lst : {&op, &al})
      for (int32_t idx : *lst) {
        rtri const &t = f.tris[idx];
        rkind const k = kind_of(t);
        bool const tx = k >= K_TEX;
        mame_spans(t, k, ms);
        rtl_tri r;
        bool const rok = r.setup(t, k, o);
        tri_ctx c{&t, idx, k, &tex};
        for (auto const &s : ms) {
          int32_t rs = 0, re = 0;
          bool const rsp = rok && s.y >= r.y0 && s.y < r.y1 && r.span(s.y, rs, re);
          if (!rsp) rs = re = 0;
          bool const mempty = s.x0 >= s.x1;
          if (mempty && !rsp) continue;
          ls.spans++;
          if (!((mempty && !rsp) || (s.x0 == rs && s.x1 == re))) ls.span_diff++;
          span sp;
          sp.y = s.y; sp.x0 = s.x0; sp.x1 = s.x1;
          std::memcpy(sp.start, s.start, sizeof(sp.start));
          std::memcpy(sp.dpdx, s.dpdx, sizeof(sp.dpdx));
          acc_mame p(sp, tx ? 5 : 2);
          int const lo = std::min(s.x0, rsp ? rs : s.x0), hi = std::max(s.x1, rsp ? re : s.x1);
          for (int x = s.x0; x < s.x1; x++, p.step()) {
            size_t const i = size_t(s.y) * SCR_W + size_t(x);
            bool const inr = rsp && x >= rs && x < re;
            float const z = p.v[0];
            bool const mpass = z <= zb[i];
            uint32_t texel = 0, mu = 0, mv = 0;
            bool outside = false;
            if (mpass && tx) {
              float const intz = 1.0f / p.v[1];
              mu = f2u32(p.v[2] * intz); mv = f2u32(p.v[3] * intz);
              texel = fetch_texel(t, tex, mu, mv, outside);
            }
            uint32_t mout = 0;
            bool mzw = false;
            bool const mwr = mpass && shade_px(c, texel, tx ? p.v[4] : p.v[1], fb[i], mout, mzw);
            if (!inr) { ls.px_only_mame++; }
            else {
              ls.px_both++;
              ls.k_px[k]++;
              rtl_px const rp = r.eval(x, s.y);
              bool const rpass = rp.z <= zr[i];
              bool any = false;
              ls.ztests++;
              if (rpass != mpass) { ls.zdiff++; any = true; }
              if (mpass && rpass) {
                uint32_t rtexel = 0;
                if (tx) {
                  ls.texfetch++;
                  uint32_t const umask = (((t.param & TP_MIRROR_U) ? 64u : 32u) << t.tex_w) - 1;
                  uint32_t const vmask = (((t.param & TP_MIRROR_V) ? 64u : 32u) << t.tex_h) - 1;
                  bool const adiff = ((mu >> 8) & umask) != ((rp.u >> 8) & umask) || ((mv >> 8) & vmask) != ((rp.v >> 8) & vmask);
                  if (adiff) { ls.tex_addr_diff++; any = true; }
                  else {
                    int const du = std::abs(int(mu & 0xff) - int(rp.u & 0xff)), dv = std::abs(int(mv & 0xff) - int(rp.v & 0xff));
                    int const dm = std::max(du, dv);
                    if (dm == 1) ls.frac1++; else if (dm >= 2 && dm <= 4) ls.frac4++; else if (dm > 4) ls.fracbig++;
                  }
                  {
                    // the set-up planes (FP32, MAME's) at the pixel centre, exactly
                    auto ideal = [&](int pi) {
                      auto const &q = r.pl[pi];
                      long double const v2 = (long double)q.c * 2 + (long double)q.dx * (2 * x + 1) + (long double)q.dy * (2 * s.y + 1);
                      return std::ldexp(v2, q.e - 41);
                    };
                    long double const io = ideal(1);
                    auto bin = [](long double d) { d = d < 0 ? -d : d; return d < 0.5L ? 0 : d < 1.5L ? 1 : d < 4.5L ? 2 : d < 16.5L ? 3 : 4; };
                    long double const iu = io != 0 ? ideal(2) / io : 0, iv = io != 0 ? ideal(3) / io : 0;
                    auto tr = [](long double q) { return (long double)(int64_t)q; };
                    ls.uv_mame[std::max(bin((long double)int32_t(mu) - tr(iu)), bin((long double)int32_t(mv) - tr(iv)))]++;
                    ls.uv_rtl[std::max(bin((long double)int32_t(rp.u) - tr(iu)), bin((long double)int32_t(rp.v) - tr(iv)))]++;
                    long double const ish = ideal(4);
                    int32_t const ishi = int32_t(ish);
                    ls.sh_mame_bad += f2i32(p.v[4]) != ishi;
                    ls.sh_rtl_bad += rp.shade != ishi;
                  }
                  bool oo;
                  rtexel = fetch_texel(t, tex, rp.u, rp.v, oo);
                  if (rtexel != texel) { ls.texel_diff++; any = true; }
                }
                int32_t const mshade = f2i32(tx ? p.v[4] : p.v[1]);
                ls.shade_n++;
                if (mshade != rp.shade) { ls.shade_diff++; any = true; }
                uint32_t rout = 0;
                bool rzw = false;
                bool const rwr = shade_pixel(t, k, rtexel, rp.shade, fb[i], rout, rzw);
                if (rwr || mwr) {
                  ls.colour_n++;
                  if (rwr != mwr || rout != mout) {
                    ls.colour_diff++; any = true;
                    for (int sh = 0; sh < 24; sh += 8)
                      ls.colour_maxd = std::max(ls.colour_maxd, std::abs(int((rout >> sh) & 0xff) - int((mout >> sh) & 0xff)));
                  }
                }
              }
              if (any) ls.k_diff[k]++;
              if (mwr && mzw) zr[i] = rp.z;
            }
            if (mwr) {
              fb[i] = mout;
              if (mzw) { zb[i] = z; if (!inr) zr[i] = fz(z); }
            }
          }
          for (int x = lo; x < hi; x++)
            if (rsp && x >= rs && x < re && !(x >= s.x0 && x < s.x1)) ls.px_only_rtl++;
        }
        // rows only the RTL covers
        if (rok)
          for (int32_t y = r.y0; y < r.y1; y++) {
            bool have = false;
            for (auto const &s : ms) if (s.y == y) { have = true; break; }
            if (have) continue;
            int32_t rs, re;
            if (r.span(y, rs, re)) { ls.spans++; ls.span_diff++; ls.px_only_rtl += uint64_t(re - rs); }
          }
      }
  }
  (void)dummy;
}

} // namespace r3d
