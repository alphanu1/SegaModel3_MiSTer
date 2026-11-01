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
// Real3D rasteriser model, RTL arithmetic: what the rasteriser RTL computes,
// bit for bit, so its bench can diff against this file.
//
//   Triangle set-up    FP32 in MAME's order (poly.h render_triangle): the plane
//                      coefficients start, d/dx, d/dy of each parameter are the
//                      floats MAME computes, on a shared FP unit, once per
//                      triangle.
//   Edges (coverage)   fixed point: vertices rounded to 1/2^sub px, each edge
//                      x(y) = xa + (y + 1/2 - ya) * slope with a 2^-20 px slope,
//                      rounded by poly.h's rule (floor, up if the fraction > 1/2).
//   Interpolants       block fixed point: each parameter's three coefficients at
//                      one exponent per triangle, 40-bit mantissas; a pixel's
//                      value is evaluated exactly, p = start + n * d/dx, not
//                      accumulated as MAME's float adds are.
//   Perspective        u = 256u/w / (1/w), v likewise; exact rational quotient,
//                      truncated as MAME's (uint32_t) conversion truncates.
//   Z                  w, linear in screen space as MAME interpolates it, stored
//                      as an unsigned float-like word (8-bit exponent, zbits - 8
//                      mantissa bits), compared as an integer.
//   Shade              truncated to an integer as MAME's (int).
//
// Texel formats, mirroring, the bilinear filter, colour modulation and the
// blends are the integer operations of r3d_rast.cpp, shared with the MAME form.

#include "r3d_rast.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace r3d {

bool shade_pixel(rtri const &t, rkind k, uint32_t texel, int32_t shade_int, uint32_t dst, uint32_t &out, bool &zw);
void count_span(rkind k, int len, rstats &st);
uint32_t bilinear_filter(uint32_t c00, uint32_t c01, uint32_t c10, uint32_t c11, uint32_t u, uint32_t v);

namespace {

using i128 = __int128;

constexpr int WB = 40;   // plane mantissa bits
constexpr int SF = 20;   // edge slope fraction bits

int msb(unsigned __int128 v) {
  int n = -1;
  while (v) { v >>= 1; n++; }
  return n;
}

// float -> fixed with s fraction bits, round to nearest (ties away), saturated
// to the 32-bit signed vertex word the edge walker takes (16.16 at s = 16)
int64_t to_fix(float f, int s) {
  double const d = std::ldexp(double(f), s);
  if (!(d > -2147483648.0 && d < 2147483647.0)) return d > 0 ? 2147483647 : -2147483647 - 1;
  return std::llround(d);
}

// poly.h round_coordinate on a fixed-point value with q fraction bits
int32_t round_fix(i128 v, int q) {
  i128 const one = i128(1) << q;
  i128 ip = v >> q;  // floor (arithmetic shift)
  i128 const fr = v - (ip << q);
  if (fr > one / 2) ip++;
  if (ip > std::numeric_limits<int32_t>::max()) return std::numeric_limits<int32_t>::max();
  if (ip < std::numeric_limits<int32_t>::min()) return std::numeric_limits<int32_t>::min();
  return int32_t(ip);
}

using plane = rtl_tri::plane;

// One parameter's plane in block fixed point: twice its value at pixel centre
// (x + 1/2, y + 1/2) is (2 c + dx (2x + 1) + dy (2y + 1)) * 2^(e - WB - 1)
i128 at2(plane const &p, int x, int y) { return i128(p.c) * 2 + i128(p.dx) * (2 * x + 1) + i128(p.dy) * (2 * y + 1); }

// The exponent covers the start and 512 px of either gradient, plus two guard
// bits; each FP32 coefficient is rounded to a WB-bit mantissa at it.
plane make_plane(long double ps, long double pdx, long double pdy) {
  plane p;
  long double const m = std::max({std::fabs(ps), std::fabs(pdx) * 512.0L, std::fabs(pdy) * 512.0L});
  int ex = 0;
  if (m > 0 && std::isfinite(m)) std::frexp(m, &ex);
  p.e = ex + 2;
  auto q = [&](long double f) -> int64_t {
    if (!std::isfinite(f)) return 0;
    return std::llround(std::ldexp(f, WB - p.e));
  };
  p.c = q(ps); p.dx = q(pdx); p.dy = q(pdy);
  return p;
}

// The integer part, truncated toward zero, of a plane value v2 * 2^(e - WB - 1)
int64_t plane_int(i128 v2, int e) {
  int const sh = e - WB - 1;
  i128 r;
  if (sh >= 0) {
    if (sh > 60) return v2 > 0 ? std::numeric_limits<int64_t>::max() : v2 < 0 ? std::numeric_limits<int64_t>::min() : 0;
    r = v2 << sh;
  } else {
    int const s = -sh;
    if (s > 120) return 0;
    r = v2 >= 0 ? v2 >> s : -((-v2) >> s);
  }
  if (r > std::numeric_limits<int64_t>::max()) return std::numeric_limits<int64_t>::max();
  if (r < std::numeric_limits<int64_t>::min()) return std::numeric_limits<int64_t>::min();
  return int64_t(r);
}

// Z word: unsigned float-like, 8-bit exponent and zbits - 8 mantissa bits
// (hidden one dropped), truncated. w <= 0 maps to 0.
uint32_t z_word(i128 v2, int e, int zbits) {
  if (v2 <= 0) return 0;
  int const n = msb((unsigned __int128)v2);
  int const mb = zbits - 8;
  int const ex = n + (e - WB - 1) + 127;
  if (ex <= 0) return 0;
  if (ex >= 255) return zbits >= 32 ? 0xffffffffu : (1u << zbits) - 1;
  unsigned __int128 const u = (unsigned __int128)v2;
  unsigned __int128 const mm = (((unsigned __int128)1) << mb) - 1;
  uint32_t const man = uint32_t((n >= mb ? (u >> (n - mb)) : (u << (mb - n))) & mm);
  return (uint32_t(ex) << mb) | man;
}

// u = (256u/w) / (1/w), truncated toward zero; the low 32 bits, as MAME's
// (uint32_t) conversion leaves them. recip > 0: 1/w through a reciprocal of
// that many bits (round to nearest) instead of an exact quotient.
uint32_t persp(i128 u2, int eu, i128 o2, int eo, int recip) {
  if (o2 == 0) return 0;
  if (recip > 0) {
    bool const neg = o2 < 0;
    unsigned __int128 const a = (unsigned __int128)(neg ? -o2 : o2);
    int const n = msb(a);
    // m: the top recip bits of |o2|; r = round(2^(2 recip - 1) / m), recip + 1 bits
    uint64_t const m = uint64_t(n >= recip - 1 ? (a >> (n - (recip - 1))) : (a << ((recip - 1) - n)));
    i128 const r = ((i128(1) << (2 * recip - 1)) + m / 2) / m;
    // 1/o2 = r * 2^-(2 recip - 1) * 2^-(n - recip + 1) = r * 2^-(n + recip)
    i128 prod = u2 * r;
    if (neg) prod = -prod;
    int const sh = n + recip - (eu - eo);
    i128 q;
    if (sh >= 0) { if (sh > 126) return 0; q = prod >= 0 ? prod >> sh : -((-prod) >> sh); }
    else { if (-sh > 60) return 0; q = prod << -sh; }
    return uint32_t(uint64_t(int64_t(q)));
  }
  i128 num = u2, den = o2;
  int const d = eu - eo;
  if (d > 0) { if (d > 60) return 0; num <<= d; }
  else if (d < 0) { if (-d > 60) return 0; den <<= -d; }
  i128 const q = num / den;  // truncates toward zero
  return uint32_t(uint64_t(int64_t(q)));
}

int32_t sat32(int64_t s) {
  return int32_t(std::clamp<int64_t>(s, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
}

} // namespace

bool rtl_tri::setup(rtri const &t, rkind kind, rtl_opts const &o) {
  tex = kind >= K_TEX;
  np = tex ? 5 : 2;
  // A vertex the geometry left non-finite (MAME's clip can divide 0 by 0): the
  // triangle is dropped. MAME's float compares draw no pixel from one in 800
  // frames (r3d_rast_ref reports any).
  for (int i = 0; i < 3; i++)
    if (!std::isfinite(t.x[i]) || !std::isfinite(t.y[i]) || !std::isfinite(t.w[i])) return false;
  sub = o.sub; zbits = o.zbits; recip = o.recip_bits;
  struct fv { int64_t x, y; float fx, fy, p[5]; } vv[3];
  for (int i = 0; i < 3; i++) {
    vv[i].fx = t.x[i]; vv[i].fy = t.y[i];
    vv[i].x = to_fix(t.x[i], sub); vv[i].y = to_fix(t.y[i], sub);
    vv[i].p[0] = o.oow ? 1.0f / t.w[i] : t.w[i];
    if (tex) { vv[i].p[1] = 1.0f / t.w[i]; vv[i].p[2] = t.p0[i]; vv[i].p[3] = t.p1[i]; vv[i].p[4] = t.p2[i]; }
    else { vv[i].p[1] = t.p2[i]; vv[i].p[2] = vv[i].p[3] = vv[i].p[4] = 0.0f; }
  }
  // order by y as MAME does, on the float y
  fv const *v1 = &vv[0], *v2 = &vv[1], *v3 = &vv[2];
  odd = false;
  if (v2->fy < v1->fy) { std::swap(v1, v2); odd = !odd; }
  if (v3->fy < v2->fy) {
    std::swap(v2, v3); odd = !odd;
    if (v2->fy < v1->fy) { std::swap(v1, v2); odd = !odd; }
  }
  // edges first: the edge walker takes them whether or not a row is on screen
  vx[0] = v1->x; vy[0] = v1->y; vx[1] = v2->x; vy[1] = v2->y; vx[2] = v3->x; vy[2] = v3->y;
  fx[0] = v1->fx; fy[0] = v1->fy; fx[1] = v2->fx; fy[1] = v2->fy; fx[2] = v3->fx; fy[2] = v3->fy;
  auto slope = [&](int a, int b) -> i128 {
    if (vy[b] == vy[a]) return 0;
    return (i128(vx[b] - vx[a]) << SF) / i128(vy[b] - vy[a]);
  };
  s12 = slope(0, 1); s13 = slope(0, 2); s23 = slope(1, 2);
  // set-up, FP32 in MAME's order, as the set-up unit's dot operations:
  // dot(a1, b1, a2, b2, a3, b3, m) = m * ((a1 * b1 + a2 * b2) + a3 * b3), each
  // operation rounded to FP32 on its own. A difference is a plus a minus product
  // by 1; an unused term is (-0) * 1, which IEEE addition leaves unchanged; a
  // plain result is multiplied by 1. With ftz, every operand and result that is
  // subnormal becomes a zero of its sign (the flush-to-zero FP units).
  bool const ftz = o.ftz;
  auto fl = [ftz](float x) { return (ftz && std::fpclassify(x) == FP_SUBNORMAL) ? std::copysign(0.0f, x) : x; };
  auto mul = [&](float a, float b) { return fl(fl(a) * fl(b)); };
  auto add = [&](float a, float b) { return fl(fl(a) + fl(b)); };
  auto dot = [&](float a1, float b1, float a2, float b2, float a3, float b3, float m) {
    return mul(m, add(add(mul(a1, b1), mul(a2, b2)), mul(a3, b3)));
  };
  float const N0 = -0.0f;
  float const X1 = v1->fx, Y1 = v1->fy, X2 = v2->fx, Y2 = v2->fy, X3 = v3->fx, Y3 = v3->fy;
  float const a00 = dot(Y2, 1, Y3, -1, N0, 1, 1), a01 = dot(X3, 1, X2, -1, N0, 1, 1), a02 = dot(X2, Y3, X3, -Y2, N0, 1, 1);
  float const a10 = dot(Y3, 1, Y1, -1, N0, 1, 1), a11 = dot(X1, 1, X3, -1, N0, 1, 1), a12 = dot(X3, Y1, X1, -Y3, N0, 1, 1);
  float const a20 = dot(Y1, 1, Y2, -1, N0, 1, 1), a21 = dot(X2, 1, X1, -1, N0, 1, 1), a22 = dot(X1, Y2, X2, -Y1, N0, 1, 1);
  det = dot(a02, 1, a12, 1, a22, 1, 1);
  flat = std::abs(det) < 0.00001f;
  if (flat) {
    for (int i = 0; i < np; i++) { pdx[i] = 0.0f; pdy[i] = 0.0f; ps[i] = v1->p[i]; }
  } else {
    idet = fl(1.0f / fl(det));
    for (int i = 0; i < np; i++) {
      float const p1 = v1->p[i], p2 = v2->p[i], p3 = v3->p[i];
      pdx[i] = dot(p1, a00, p2, a10, p3, a20, idet);
      pdy[i] = dot(p1, a01, p2, a11, p3, a21, idet);
      ps[i] = dot(p1, a02, p2, a12, p3, a22, idet);
    }
  }
  int const q = sub + SF;
  y0 = std::max(round_fix(i128(v1->y) << SF, q), 0);
  y1 = std::min(round_fix(i128(v3->y) << SF, q), SCR_H);
  if (y1 - y0 <= 0) return false;

  if (!o.fixed_setup) {
    for (int i = 0; i < np; i++) pl[i] = make_plane(ps[i], pdx[i], pdy[i]);
  } else {
    // exact planes through the fixed-point vertices; the FP32 parameters
    long double const sc = std::ldexp(1.0L, -sub);
    long double const X1 = v1->x * sc, Y1 = v1->y * sc, X2 = v2->x * sc, Y2 = v2->y * sc, X3 = v3->x * sc, Y3 = v3->y * sc;
    long double const d = (X2 - X1) * (Y3 - Y1) - (X3 - X1) * (Y2 - Y1);
    for (int i = 0; i < np; i++) {
      long double const p1 = v1->p[i], d2 = (long double)v2->p[i] - p1, d3 = (long double)v3->p[i] - p1;
      if (d == 0) { pl[i] = make_plane(p1, 0, 0); continue; }
      long double const gx = (d2 * (Y3 - Y1) - d3 * (Y2 - Y1)) / d;
      long double const gy = (d3 * (X2 - X1) - d2 * (X3 - X1)) / d;
      pl[i] = make_plane(p1 - gx * X1 - gy * Y1, gx, gy);
    }
  }

  return true;
}

bool rtl_tri::span(int32_t y, int32_t &x0, int32_t &x1) const {
  int const q = sub + SF;
  int64_t const fy = (int64_t(2 * y + 1) << sub) >> 1;   // y + 1/2, 2^-sub px
  auto ex = [&](int a, i128 s) { return (i128(vx[a]) << SF) + i128(fy - vy[a]) * s; };
  i128 const sx = ex(0, s13);
  i128 const tx = (fy < vy[1]) ? ex(0, s12) : ex(1, s23);
  int32_t is = round_fix(sx, q), ie = round_fix(tx, q);
  if (is > ie) std::swap(is, ie);
  is = std::max(is, 0);
  ie = std::min(ie, SCR_W);
  x0 = is; x1 = ie;
  return is < ie;
}

rtl_px rtl_tri::eval(int x, int y) const {
  rtl_px r{};
  r.z = z_word(at2(pl[0], x, y), pl[0].e, zbits);
  if (tex) {
    i128 const o2 = at2(pl[1], x, y);
    r.u = persp(at2(pl[2], x, y), pl[2].e, o2, pl[1].e, recip);
    r.v = persp(at2(pl[3], x, y), pl[3].e, o2, pl[1].e, recip);
    r.shade = sat32(plane_int(at2(pl[4], x, y), pl[4].e));
  } else {
    r.shade = sat32(plane_int(at2(pl[1], x, y), pl[1].e));
  }
  return r;
}

void clear_z_rtl(std::vector<uint32_t> &zb, rtl_opts const &o) {
  std::fill(zb.begin(), zb.end(), (o.zbits >= 32) ? 0xffffffffu : ((1u << o.zbits) - 1));
}

// The bilinear texel at (u, v), MAME's TEX_FETCH_BILINEAR addressing
uint32_t fetch_texel(rtri const &t, texram const &tex, uint32_t u, uint32_t v, bool &outside) {
  uint32_t const umask = (((t.param & TP_MIRROR_U) ? 64u : 32u) << t.tex_w) - 1;
  uint32_t const vmask = (((t.param & TP_MIRROR_V) ? 64u : 32u) << t.tex_h) - 1;
  uint32_t const u1 = (u >> 8) & umask, v1 = (v >> 8) & vmask;
  uint32_t const u2 = (u1 + 1) & umask, v2 = (v1 + 1) & vmask;
  bool oo[4];
  uint32_t const c00 = tex.texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u1, v1, &oo[0]);
  uint32_t const c01 = tex.texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u2, v1, &oo[1]);
  uint32_t const c10 = tex.texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u1, v2, &oo[2]);
  uint32_t const c11 = tex.texel(t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, u2, v2, &oo[3]);
  outside = oo[0] || oo[1] || oo[2] || oo[3];
  return bilinear_filter(c00, c01, c10, c11, u & 0xff, v & 0xff);
}

// Mip levels at Supermodel's sheet bases (r3d_rast_sm.cpp, Real3D.cpp's
// mipXBase / mipYBase): level L of the page is the 2048 >> L by 1024 >> L
// region at (MIPX[L], MIPY[L]).
static constexpr uint32_t MIPX[] = {0, 1024, 1536, 1792, 1920, 1984, 2016, 2032, 2040, 2044, 2046, 2047};
static constexpr uint32_t MIPY[] = {0, 512, 768, 896, 960, 992, 1008, 1016, 1020, 1022, 1023};

int mip_levels(rtri const &t) {
  uint32_t const m = std::min(32u << t.tex_w, 32u << t.tex_h);
  int l = 0;
  while ((2u << l) <= m) l++;
  return l - 1;
}

void texel_addr4(rtri const &t, uint32_t u, uint32_t v, int L, uint32_t a[4]) {
  uint32_t const pw = std::max(1u, (32u << t.tex_w) >> L), ph = std::max(1u, (32u << t.tex_h) >> L);
  uint32_t const um = ((t.param & TP_MIRROR_U) ? 2 * pw : pw) - 1, vm = ((t.param & TP_MIRROR_V) ? 2 * ph : ph) - 1;
  uint32_t const uL = u >> L, vL = v >> L;
  uint32_t const u1 = (uL >> 8) & um, v1 = (vL >> 8) & vm, u2 = (u1 + 1) & um, v2 = (v1 + 1) & vm;
  auto mr = [](uint32_t c, uint32_t p) { return c < p ? c : 2 * p - 1 - c; };
  uint32_t const x0 = MIPX[L] + ((t.tex_x * 32u) >> L), y0 = (t.page & 1) * 1024u + MIPY[L] + ((t.tex_y * 32u) >> L);
  uint32_t const c1 = x0 + mr(u1, pw), c2 = x0 + mr(u2, pw), r1 = y0 + mr(v1, ph), r2 = y0 + mr(v2, ph);
  a[0] = ((r1 << 11) + c1) & 0x3fffff; a[1] = ((r1 << 11) + c2) & 0x3fffff;
  a[2] = ((r2 << 11) + c1) & 0x3fffff; a[3] = ((r2 << 11) + c2) & 0x3fffff;
}

uint32_t fetch_texel_lv(rtri const &t, texram const &tex, uint32_t u, uint32_t v, int L) {
  uint32_t a[4];
  texel_addr4(t, u, v, L, a);
  int const fm = int(t.fmt & 7);
  return bilinear_filter(mame_decode(fm, tex.t[a[0]]), mame_decode(fm, tex.t[a[1]]), mame_decode(fm, tex.t[a[2]]),
                         mame_decode(fm, tex.t[a[3]]), (u >> L) & 0xff, (v >> L) & 0xff);
}

void draw_tri_rtl(rtri const &t, int32_t idx, rkind kind, texram const &tex, rtl_opts const &o,
                  rimage &img, std::vector<uint32_t> &zb, rstats &st) {
  rtl_tri r;
  if (!r.setup(t, kind, o)) return;
  bool const blends = kind == K_SOLID_TRANS || kind == K_CONTOUR || kind == K_TEX_ALPHA;
  bool drew = false;
  for (int32_t y = r.y0; y < r.y1; y++) {
    int32_t is, ie;
    if (!r.span(y, is, ie)) continue;
    drew = true;
    st.spans++;
    count_span(kind, ie - is, st);
    for (int x = is; x < ie; x++) {
      st.pixels++;
      st.px_kind[kind]++;
      size_t const i = size_t(y) * SCR_W + size_t(x);
      rtl_px const p = r.eval(x, y);
      if (!(p.z <= zb[i])) continue;
      st.zpass++;
      uint32_t texel = 0;
      if (r.tex) {
        bool outside;
        texel = fetch_texel(t, tex, p.u, p.v, outside);
        st.texels += 4;
        st.tex_outside += outside;
        st.fmt_px[t.fmt & 7]++;
      }
      uint32_t out;
      bool zw = false;
      if (shade_pixel(t, kind, texel, p.shade, img.px[i], out, zw)) {
        (void)blends;
        img.px[i] = out;
        if (zw) zb[i] = p.z;
        img.kind[i] = kind;
        img.fmt[i] = r.tex ? uint8_t(t.fmt & 7) : 0xff;
        img.tri[i] = idx;
        st.written++;
      }
    }
  }
  st.tris_drawn += drew;
}


// ---- D-25 RTL form -----------------------------------------------------------------
//
// The integer forms the RTL builds for the D-25 rows. Everything not named here is
// render(rtl): the set-up, edges, planes, Z word, perspective and MAME's kinds.
//
//   Cull     facing = sign(det) xor the y sort's parity; a back face is dropped
//            unless the polygon is double-sided (header[1] bit 4). The geometry's
//            inverted copy of a double-sided polygon flips the verdict.
//   Fog      per viewport: fc = (c a + (c a >> 8) + 128) >> 8 for each fog colour
//            channel c and the fog ambient a; S = (start16 16448 + 2048) >> 12
//            (start / 255 in 1/1024); K = fl(density / kz), kz = -proj[11].
//            Per pixel: P = floor(w K 1024) with w the perspective reciprocal of
//            the 1/w plane (the same r the texture path uses), or for an
//            untextured triangle with w depth the w plane itself (its top 32
//            bits), saturated at 4096;
//            lin = clamp(S + P, 0, 1024); g = (fogint lin) >> 4, fogint header[6]
//            bits 11-15; each channel o = clamp(o + (((fc - o) g) >> 10), 0, 255),
//            after the shade and before any blend. Scroll fog fills its viewport
//            at the start of the frame with fc, not drawn: a blend reads fc, and a
//            pixel nothing draws shows (fc sf + 128) >> 8, sf = node[0x20].
//   Trans    0: MAME's blends. 1: a translucent polygon (transparency < 32) is
//            drawn opaque, Z written, on the pixels where its 4 x 4 ordered
//            dither B(x, y) (inverted by header[6] bit 17) is below
//            transparency / 2. 2: Supermodel's two layers: translucent polygons
//            go to layer header[6] bit 17 after the opaque walk, nearest fragment
//            kept against the opaque Z, composited over the colour at the end.
//   Depth    rtl_opts::oow: Z from the 1/w plane, larger nearer, cleared to 0.

namespace {

struct rgb4 { int32_t a, r, g, b; };
inline rgb4 un4(uint32_t c) { return {int32_t((c >> 24) & 0xff), int32_t((c >> 16) & 0xff), int32_t((c >> 8) & 0xff), int32_t(c & 0xff)}; }
inline int32_t c8(int32_t x) { return x < 0 ? 0 : x > 255 ? 255 : x; }
inline uint32_t pk4(rgb4 const &c) { return (uint32_t(c8(c.a)) << 24) | (uint32_t(c8(c.r)) << 16) | (uint32_t(c8(c.g)) << 8) | uint32_t(c8(c.b)); }
inline int32_t m32(int32_t a, int32_t b) { return int32_t(uint32_t(a) * uint32_t(b)); }
inline void sc_imm(rgb4 &c, int32_t s) {
  c.a = c8(m32(c.a, s) >> 8); c.r = c8(m32(c.r, s) >> 8); c.g = c8(m32(c.g, s) >> 8); c.b = c8(m32(c.b, s) >> 8);
}
inline void sc_rgba(rgb4 &c, rgb4 const &s) {
  c.a = c8((c.a * s.a) >> 8); c.r = c8((c.r * s.r) >> 8); c.g = c8((c.g * s.g) >> 8); c.b = c8((c.b * s.b) >> 8);
}
inline void bl(rgb4 &c, rgb4 const &o, uint8_t f) {
  int32_t const s1 = f, s2 = 256 - s1;
  c.a = (c.a * s1 + o.a * s2) >> 8; c.r = (c.r * s1 + o.r * s2) >> 8; c.g = (c.g * s1 + o.g * s2) >> 8; c.b = (c.b * s1 + o.b * s2) >> 8;
}

// MAME's shading split at the point fog goes in: the unblended colour, then the
// blend factor (-1: none). False: the alpha test discards the pixel.
bool frag(rtri const &t, rkind k, uint32_t texel, int32_t ii, rgb4 &col, int &f, bool &zw) {
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
    col = un4(texel); sc_rgba(col, un4(t.color)); sc_imm(col, ii);
    f = int(fa); zw = true; return true;
  }
  case K_TEX_ALPHA: {
    uint32_t const fa = texel >> 24;
    if (fa == 0) return false;
    col = un4(texel); sc_imm(col, ii);
    f = int(fa); zw = false; return true;
  }
  default: return false;
  }
}

struct fogvp {
  bool on = false;
  int32_t fc[3] = {}, S = 0;
  float K = 0;
};
inline float u2f(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
fogvp make_fog(rvp const &v) {
  fogvp q;
  uint32_t const amb = (v.node[0x25] >> 16) & 255;
  for (int c = 0; c < 3; c++) {
    uint32_t const ch = (v.node[0x22] >> (16 - 8 * c)) & 255, p = ch * amb;
    q.fc[c] = int32_t((p + (p >> 8) + 128) >> 8);
  }
  float dens = std::fabs(u2f(v.node[0x23]));
  if (!(dens < 99999.0f)) dens = 0.0f;
  int32_t const st16 = int16_t(v.node[0x25] & 0xffff);
  q.S = (st16 * 16448 + 2048) >> 12;
  float const kz = -v.proj[11];
  q.K = (kz > 0.0f && std::isfinite(kz)) ? dens / kz : 0.0f;
  if (!std::isfinite(q.K)) q.K = 0.0f;
  q.on = true;
  return q;
}

// floor(w K 1024), w = 1 / (o2 2^(eo - WB - 1)) through the R-bit reciprocal
int64_t fog_p(i128 o2, int eo, float K, int R) {
  if (o2 <= 0) return 4096;
  if (!(K > 0.0f)) return 0;
  unsigned __int128 const a = (unsigned __int128)o2;
  int const n = msb(a);
  uint64_t const m = uint64_t(n >= R - 1 ? (a >> (n - (R - 1))) : (a << ((R - 1) - n)));
  uint64_t const r = ((uint64_t(1) << (2 * R - 1)) + m / 2) / m;
  uint32_t kb; std::memcpy(&kb, &K, 4);
  int const ke = int((kb >> 23) & 255);
  if (ke == 0) return 0;
  uint64_t const km = (kb & 0x7fffff) | 0x800000;
  // w K 1024 = r km 2^(WB + 1 - eo - n - R) 2^(ke - 150) 2^10
  int const sh = WB + 1 - eo - n - R + ke - 150 + 10;
  unsigned __int128 const pr = (unsigned __int128)r * km;
  if (sh >= 0) { if (sh > 40 || (pr >> (60 - sh)) != 0) return 4096; return std::min<int64_t>(int64_t(pr << sh), 4096); }
  if (-sh > 120) return 0;
  return std::min<int64_t>(int64_t(pr >> -sh), 4096);
}

// floor(w K 1024) for a linear w plane value w = v2 2^(e - WB - 1) (untextured, w depth)
int64_t fog_lin(i128 v2, int e, float K) {
  if (v2 <= 0 || !(K > 0.0f)) return v2 <= 0 ? 0 : 0;
  uint32_t kb; std::memcpy(&kb, &K, 4);
  int const ke = int((kb >> 23) & 255);
  if (ke == 0) return 0;
  uint64_t const km = (kb & 0x7fffff) | 0x800000;
  int const n = msb((unsigned __int128)v2);
  // keep the top 32 bits of v2: the product is then 56 bits
  int const drop = std::max(0, n - 31);
  uint64_t const vm = uint64_t((unsigned __int128)v2 >> drop);
  int const sh = drop + e - WB - 1 + ke - 150 + 10;
  unsigned __int128 const pr = (unsigned __int128)vm * km;
  if (sh >= 0) { if (sh > 40 || (pr >> (60 - sh)) != 0) return 4096; return std::min<int64_t>(int64_t(pr << sh), 4096); }
  if (-sh > 120) return 0;
  return std::min<int64_t>(int64_t(pr >> -sh), 4096);
}

inline void fog_apply(rgb4 &c, fogvp const &q, int32_t P, uint32_t fogint) {
  int32_t const lin = std::clamp(q.S + P, 0, 1024);
  int32_t const g = int32_t((fogint * uint32_t(lin)) >> 4);
  int32_t *ch[3] = {&c.r, &c.g, &c.b};
  for (int i = 0; i < 3; i++) *ch[i] = c8(*ch[i] + (((q.fc[i] - *ch[i]) * g) >> 10));
}

constexpr uint8_t BAYER[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

// Supermodel's view-space back-face verdict (r3d_rast_sm.cpp), for the cull table
int sm_back(rframe const &f, rtri const &t) {
  if (t.vp < 0 || t.model < 0) return -1;
  rvp const &vp = f.vps[size_t(t.vp)];
  float const nm[3] = {float(int32_t(t.hdr[1]) >> 8) / 4194304.0f, float(int32_t(t.hdr[2]) >> 8) / 4194304.0f,
                       float(int32_t(t.hdr[3]) >> 8) / 4194304.0f};
  float const *m = f.models[size_t(t.model)].top;
  double n[3];
  for (int j = 0; j < 3; j++) n[j] = nm[0] * m[0 * 4 + j] + nm[1] * m[1 * 4 + j] + nm[2] * m[2 * 4 + j];
  n[0] *= vp.coord[1]; n[1] *= vp.coord[6]; n[2] *= -vp.coord[8];
  double const kz = -double(vp.proj[11]);
  double const xn = ((t.x[0] - vp.x) / vp.w - 0.5) * 2.0, yn = ((t.y[0] - vp.y) / vp.h - 0.5) * 2.0;
  double const v3[3] = {xn * t.w[0] / double(vp.proj[0]), yn * t.w[0] / double(vp.proj[5]), -t.w[0] / kz};
  double const d = v3[0] * n[0] + v3[1] * n[1] + v3[2] * n[2];
  if (!std::isfinite(d)) return -1;
  return d > 0;
}

// The mip terms of one textured triangle (d25_opts::mip)
struct mip_tri {
  bool on = false;
  rvp const *vp = nullptr;
  double n[3] = {0, 0, 0}, kz = 2.0, cnp = 0;   // view-space face normal, w = kz x view depth, -cota x NP
  int levels = 0;
  int32_t lb8 = INT32_MIN;                       // hardware form: -256 log2(B), B = kz^2 dot(v0, n) (-cota) NP as FP32
  double dv = 0;                                 // dot(v0, n), the facing dot at vertex 0
  uint32_t bw = 0;                               // B's FP32 word (0 without a viewport)
};
// 256 log2(1 + (i + 1/2) / 64): the 6-bit mantissa table of the hardware form
struct log2t { uint8_t v[64]; log2t() { for (int i = 0; i < 64; i++) v[i] = uint8_t(std::lround(256.0 * std::log2(1.0 + (i + 0.5) / 64.0))); } };
log2t const LOG2T_ {};
uint8_t const *const LOG2T = LOG2T_.v;

mip_tri make_mip(rframe const &f, rtri const &t) {
  mip_tri m;
  m.on = true;
  m.levels = mip_levels(t);
  if (t.vp < 0) return m;
  m.vp = &f.vps[size_t(t.vp)];
  rvp const &vp = *m.vp;
  m.kz = -double(vp.proj[11]);
  if (t.model >= 0) {
    float const nm[3] = {float(int32_t(t.hdr[1]) >> 8) / 4194304.0f, float(int32_t(t.hdr[2]) >> 8) / 4194304.0f,
                         float(int32_t(t.hdr[3]) >> 8) / 4194304.0f};
    float const *mm = f.models[size_t(t.model)].top;
    for (int j = 0; j < 3; j++) m.n[j] = nm[0] * mm[0 * 4 + j] + nm[1] * mm[1 * 4 + j] + nm[2] * mm[2 * 4 + j];
    m.n[0] *= vp.coord[1]; m.n[1] *= vp.coord[6]; m.n[2] *= -vp.coord[8];
  }
  double const cota = double(u2f(vp.node[3])), np = double(t.hdr[5] >> 8) / 4096.0;
  m.cnp = -cota * np;
  // the hardware form's per-triangle word: dot(v, n) at vertex 0 (the geometry's facing dot)
  double const xn = ((t.x[0] - vp.x) / vp.w - 0.5) * 2.0, yn = ((t.y[0] - vp.y) / vp.h - 0.5) * 2.0, w = t.w[0];
  double const dv = xn * w / double(vp.proj[0]) * m.n[0] + yn * w / double(vp.proj[5]) * m.n[1] - w / m.kz * m.n[2];
  float const B = float(m.kz * m.kz * dv * m.cnp);
  m.dv = dv;
  std::memcpy(&m.bw, &B, 4);
  m.lb8 = d25_lod_lb8(m.bw);
  return m;
}

} // namespace

int32_t d25_lod_lb8(uint32_t bb) {
  float B; std::memcpy(&B, &bb, 4);
  if (!(B > 0.0f && std::isfinite(B))) return INT32_MIN;
  return -((int32_t((bb >> 23) & 255) - 127) * 256 + int32_t(LOG2T[(bb >> 17) & 63]));
}
uint8_t d25_log2t(int i) { return LOG2T[i & 63]; }
bool d25_lod_terms(rframe const &f, rtri const &t, double &dv, uint32_t &bw) {
  mip_tri const m = make_mip(f, t);
  dv = m.dv; bw = m.bw;
  return m.vp != nullptr;
}

int64_t d25_fog_p(__int128 o2, int eo, float K) { return fog_p(o2, eo, K, 24); }

bool d25_colour(rkind k, uint32_t texel, uint32_t color, uint32_t trans, int32_t shade, uint32_t fogint,
                int32_t const fc[3], int32_t S, int32_t P, uint32_t &rgb, int &fb, bool &zw) {
  rtri t{};
  t.color = color; t.trans = trans;
  rgb4 col;
  zw = false;
  if (!frag(t, k, texel, shade, col, fb, zw)) return false;
  if (fogint) {
    fogvp q;
    q.S = S;
    for (int c = 0; c < 3; c++) q.fc[c] = fc[c];
    fog_apply(col, q, P, fogint);
  }
  rgb = pk4(col) & 0xffffff;
  return true;
}

bool d25_scroll_fog(rframe const &f, d25_sfog &out) {
  out = d25_sfog{};
  int best = -1;
  for (int pri = 0; pri < 4 && best < 0; pri++)
    for (size_t i = 0; i < f.vps.size(); i++) {
      auto const &v = f.vps[i];
      if (int((v.node[0] >> 3) & 3) != pri) continue;
      uint32_t const sf = v.node[0x20] & 0xff;
      if (sf > 0 && (best < 0 || (f.vps[size_t(best)].node[0x20] & 0xff) < sf)) best = int(i);
    }
  if (best < 0) return false;
  auto const &v = f.vps[size_t(best)];
  bool on = false;
  for (auto const &n : f.vps)
    if ((n.node[0x22] & 0xffffff) == (v.node[0x22] & 0xffffff)) {
      float const dens = std::fabs(u2f(n.node[0x23]));
      if ((dens > 0 && dens < 99999.0f) || int16_t(n.node[0x25] & 0xffff) > 0 || (n.node[0x24] & 0xff)) { on = true; break; }
    }
  if (!on) return false;
  fogvp const q = make_fog(v);
  out.alpha = v.node[0x20] & 0xff;
  for (int c = 0; c < 3; c++) {
    out.col |= uint32_t(q.fc[c]) << (16 - 8 * c);
    out.shown |= (((uint32_t(q.fc[c]) * out.alpha + 128) >> 8) & 255) << (16 - 8 * c);
  }
  out.x0 = std::max(0, int(v.x)); out.x1 = std::min(SCR_W, int(v.x + v.w + 0.5f));
  out.y0 = std::max(0, int(v.y)); out.y1 = std::min(SCR_H, int(v.y + v.h + 0.5f));
  return true;
}

void render_d25(rframe const &f, texram const &tex, rtl_opts const &o, d25_opts const &d, rimage &img, d25_stats &st) {
  img = rimage();
  size_t const NP = size_t(SCR_W) * SCR_H;
  uint32_t const zfar = o.oow ? 0u : ((o.zbits >= 32) ? 0xffffffffu : ((1u << o.zbits) - 1));
  std::vector<uint32_t> zb(NP), zt(NP), t1(NP, 0), t2(NP, 0);
  std::vector<uint32_t> ztg(NP, 0);   // generation of zt: stale reads as the opaque Z
  uint32_t gen = 0;
  std::vector<fogvp> fv(f.vps.size());
  if (d.fog) for (size_t i = 0; i < f.vps.size(); i++) fv[i] = make_fog(f.vps[i]);

  // scroll fog: the lowest priority's highest scroll-fog viewport (Supermodel's rule)
  uint32_t sfog = 0;
  {
    d25_sfog sg;
    if (d.fog && d25_scroll_fog(f, sg)) {
      sfog = sg.alpha;
      for (int y = sg.y0; y < sg.y1; y++)
        for (int x = sg.x0; x < sg.x1; x++) img.px[size_t(y) * SCR_W + size_t(x)] = sg.col;   // alpha 0: not drawn; blends read the fog colour itself
    }
  }

  int const nb = (SCR_H + d.band - 1) / d.band;
  std::vector<uint64_t> bw(size_t(nb), 0), bz(size_t(nb), 0), bs(size_t(nb), 0);
  auto nearer = [&](uint32_t z, uint32_t ref, bool strict) {
    if (o.oow) return strict ? z > ref : z >= ref;
    return strict ? z < ref : z <= ref;
  };

  // layer: 0 the colour walk, 1 / 2 translucency layer walks (trans = 2)
  auto draw = [&](int32_t idx, int layer) {
    rtri const &t = f.tris[size_t(idx)];
    bool const transl = t.trans < 32;
    rkind k = kind_of(t);
    bool stip = false;
    if (d.trans >= 1 && transl) {
      rtri u = t;
      u.alpha = t.hdr[6] & 1;   // MAME's texture-alpha bit alone
      k = kind_of(u);
      stip = d.trans == 1;
      if (d.trans == 2 && layer == 0) return;
      if (d.trans == 2 && layer != 1 + int((t.hdr[6] >> 17) & 1)) return;
    } else if (layer != 0) return;
    st.tris++;
    rtl_tri r;
    if (!r.setup(t, k, o)) return;
    // cull: det sign against the sort parity
    bool const back = (r.det < 0.0f) != r.odd;
    if (d.cull) {
      int const smv = sm_back(f, t);
      if (smv >= 0 && !(t.hdr[1] & 0x10)) st.cull_tab[back][smv]++;
      bool const drop = d.cull == 1 ? back : smv == 1;
      if (drop && !(t.hdr[1] & 0x10)) { st.culled++; st.setup += 6 + 4; if (d.hook) d.hook->tri(idx, layer, r, k, true); return; }
    }
    if (d.hook) d.hook->tri(idx, layer, r, k, false);
    st.setup += uint64_t(r.tex ? (o.oow ? 54 : 66) : 30) + 4;
    st.store_bytes += uint64_t(r.tex ? (o.oow ? 128 : 144) : 80);
    fogvp const *q = (d.fog && t.vp >= 0) ? &fv[size_t(t.vp)] : nullptr;
    uint32_t const fogint = (t.hdr[6] >> 11) & 31;
    // mip: Supermodel's LOD = -log2((kz / w)^2 dot(v, n) (-cota) NP); dot(v, n)
    // is constant on the triangle's plane, so the per-pixel part is -2 log2(1/w)
    mip_tri mt = (d.mip && r.tex) ? make_mip(f, t) : mip_tri{};
    if (mt.on && d.lodb && t.poly < d.lodb->size() && (*d.lodb)[t.poly] != 0xffffffffu) {
      mt.bw = (*d.lodb)[t.poly];
      mt.lb8 = d25_lod_lb8(mt.bw);
    }
    if (q && fogint == 0) q = nullptr;
    // cycle model, per band touched
    for (int b = r.y0 / d.band; b <= (r.y1 - 1) / d.band; b++) {
      int const b0 = std::max(r.y0, b * d.band), b1 = std::min(r.y1, (b + 1) * d.band);
      uint64_t pz = 0, ps = 0;
      for (int32_t y = b0; y < b1; y++) {
        int32_t is, ie;
        if (!r.span(y, is, ie)) continue;
        st.spans++;
        pz += uint64_t(std::max(1, (ie - is + 1) / 2));
        for (int x = is; x < ie; x++) {
          size_t const i = size_t(y) * SCR_W + size_t(x);
          st.ztest++;
          rtl_px const p = r.eval(x, y);
          d25_px ev;
          ev.tri = idx; ev.layer = layer; ev.x = x; ev.y = y; ev.k = k; ev.z = p.z; ev.u = p.u; ev.v = p.v; ev.shade = p.shade;
          struct emit_t { d25_hook *h; d25_px &e; ~emit_t() { if (h) h->px(e); } } emit{d.hook, ev};
          if (stip) {
            uint32_t bb = BAYER[y & 3][x & 3];
            if (t.hdr[6] & 0x20000) bb = 15 - bb;
            if (!(bb < t.trans / 2)) { ev.stip_out = true; continue; }
          }
          uint32_t zref = zb[i];
          if (layer) zref = (ztg[i] == gen) ? zt[i] : zb[i];
          ev.zref = zref;
          if (!nearer(p.z, zref, layer != 0)) continue;
          ev.zpass = true;
          st.zpass++;
          ps++;
          uint32_t texel = 0;
          if (r.tex && !mt.on) { bool outside; texel = fetch_texel(t, tex, p.u, p.v, outside); }
          else if (r.tex) {
            int32_t l8;   // LOD x 256, clamped to [0, levels]
            if (d.lod == 0) {
              double const ooz = std::ldexp(double(at2(r.pl[1], x, y)), r.pl[1].e - WB - 1);
              double lod = 0;
              if (mt.vp && ooz != 0) {
                double const w = 1.0 / ooz, xn = ((x + 0.5 - mt.vp->x) / mt.vp->w - 0.5) * 2.0, yn = ((y + 0.5 - mt.vp->y) / mt.vp->h - 0.5) * 2.0;
                double const dv = xn * w / double(mt.vp->proj[0]) * mt.n[0] + yn * w / double(mt.vp->proj[5]) * mt.n[1] - w / mt.kz * mt.n[2];
                double const fw = mt.kz * ooz, lb = dv * mt.cnp;
                lod = (lb > 0) ? -std::log2(fw * fw * lb) : 0.0;
              }
              if (!(lod > 0)) lod = 0;
              lod = std::min(lod, double(mt.levels));
              l8 = int32_t(lod * 256.0);
              if (d.mip == 1) l8 = int32_t(std::lround(lod)) * 256;
            } else {
              uint32_t const zw1 = o.oow ? p.z : z_word(at2(r.pl[1], x, y), r.pl[1].e, 24);
              int32_t lz = (int32_t(zw1 >> 16) - 127) * 256 + int32_t(LOG2T[(zw1 >> 10) & 63]);   // 256 log2(1/w)
              l8 = mt.lb8 == INT32_MIN ? 0 : mt.lb8 - 2 * lz;
              l8 = std::clamp(l8, 0, mt.levels * 256);
              if (d.mip == 1) l8 = ((l8 + 128) >> 8) << 8;
            }
            int const il = l8 >> 8;
            uint32_t const fr = (il >= mt.levels) ? 0u : uint32_t(l8 & 255);
            ev.lv0 = il; ev.lodb = mt.bw; ev.lvmax = mt.levels;
            texel = fetch_texel_lv(t, tex, p.u, p.v, il);
            if (d.mip == 2 && fr) {
              ev.lv1 = il + 1; ev.lfr = fr;
              uint32_t const t1 = fetch_texel_lv(t, tex, p.u, p.v, il + 1);
              uint32_t m = 0;
              for (int sh = 0; sh < 32; sh += 8)
                m |= ((((texel >> sh) & 255) * (256 - fr) + ((t1 >> sh) & 255) * fr) >> 8) << sh;
              texel = m;
            }
          }
          ev.texel = texel;
          rgb4 col; int fb; bool zw = false;
          if (!frag(t, k, texel, p.shade, col, fb, zw)) continue;
          ev.ok = true; ev.zw = zw; ev.fb = fb; ev.col_raw = pk4(col);
          if (q) {
            int32_t P;
            if (o.oow) P = int32_t(fog_p(at2(r.pl[0], x, y), r.pl[0].e, q->K, o.recip_bits > 0 ? o.recip_bits : 24));
            else if (r.tex) P = int32_t(fog_p(at2(r.pl[1], x, y), r.pl[1].e, q->K, o.recip_bits > 0 ? o.recip_bits : 24));
            else P = int32_t(fog_lin(at2(r.pl[0], x, y), r.pl[0].e, q->K));
            fog_apply(col, *q, P, fogint);
            st.fog_px++;
            ev.fog = true; ev.P = P;
            ev.K = q->K; ev.S = q->S; ev.fogint = fogint;
            for (int c = 0; c < 3; c++) ev.fc[c] = q->fc[c];
          }
          ev.col_fog = pk4(col);
          ev.dst = img.px[i];
          st.shaded++;
          if (layer) {
            // layer fragment: unblended colour, alpha = transparency (x texel alpha)
            int32_t la = int32_t((t.trans * 255) / 32);
            if (k == K_TEX_ALPHA) la = (la * (fb + (fb >> 7))) >> 8;
            col.a = la;
            (layer == 1 ? t1 : t2)[i] = pk4(col);
            zt[i] = p.z; ztg[i] = gen;
            continue;
          }
          if (fb >= 0) bl(col, un4(img.px[i]), uint8_t(fb));
          img.px[i] = 0xff000000u | (pk4(col) & 0xffffff);
          ev.out = img.px[i];
          if (zw) zb[i] = p.z;
          if (d.trans == 2) { t1[i] = 0; t2[i] = 0; }
          img.kind[i] = k;
          img.tri[i] = idx;
        }
      }
      st.pairs++;
      st.rows += uint64_t(b1 - b0);
      uint64_t const pw = uint64_t(std::max(b1 - b0, d.prod));
      bw[size_t(b)] += pw; bz[size_t(b)] += pz; bs[size_t(b)] += ps;
      st.frame_serial += std::max({pw, pz, ps});
    }
  };

  for (uint32_t pass = 0; pass < 4; pass++) {
    std::vector<int32_t> op, al;
    for (size_t i = 0; i < f.tris.size(); i++)
      if (f.tris[i].pass == pass) (f.tris[i].alpha ? al : op).push_back(int32_t(i));
    if (op.empty() && al.empty()) continue;
    std::sort(op.begin(), op.end(), [&](int32_t x, int32_t y) { return f.tris[size_t(x)].buf < f.tris[size_t(y)].buf; });
    std::sort(al.begin(), al.end(), [&](int32_t x, int32_t y) { return f.tris[size_t(x)].buf > f.tris[size_t(y)].buf; });
    std::fill(zb.begin(), zb.end(), zfar);
    for (int32_t i : op) draw(i, 0);
    for (int32_t i : al) draw(i, 0);
    if (d.trans == 2) {
      for (int ly = 1; ly <= 2; ly++) {
        gen++;
        for (int32_t i : op) draw(i, ly);
        for (int32_t i : al) draw(i, ly);
      }
    }
  }
  if (d.trans == 2) {
    // composite: both layers present mix to opaque, else the one present, over the colour
    for (size_t i = 0; i < NP; i++) {
      rgb4 const p = un4(t1[i]), qq = un4(t2[i]);
      if (!p.a && !qq.a) continue;
      rgb4 tl;
      if (p.a && qq.a) {
        int32_t const m = (qq.a + 255 - p.a) >> 1, mm = m + (m >> 7);
        tl = {255, p.r + (((qq.r - p.r) * mm) >> 8), p.g + (((qq.g - p.g) * mm) >> 8), p.b + (((qq.b - p.b) * mm) >> 8)};
      } else tl = p.a ? p : qq;
      rgb4 base = un4(img.px[i]);
      int32_t const a = tl.a + (tl.a >> 7);
      base.r = (tl.r * a + base.r * (256 - a)) >> 8;
      base.g = (tl.g * a + base.g * (256 - a)) >> 8;
      base.b = (tl.b * a + base.b * (256 - a)) >> 8;
      img.px[i] = 0xff000000u | (pk4(base) & 0xffffff);
      if (img.kind[i] == K_NONE) img.kind[i] = K_SOLID_TRANS;
    }
  }
  // pixels nothing drew: the scroll fog's colour at its alpha
  if (sfog)
    for (auto &p : img.px)
      if (!(p >> 24) && (p & 0xffffff)) {
        uint32_t o2 = 0xff000000u;
        for (int c = 0; c < 3; c++) o2 |= ((((p >> (16 - 8 * c)) & 255) * sfog + 128) >> 8) << (16 - 8 * c);
        p = o2;
      }
  for (int b = 0; b < nb; b++) {
    st.walker += bw[size_t(b)]; st.zwalk += bz[size_t(b)]; st.shade += bs[size_t(b)];
    st.frame_fifo += std::max({bw[size_t(b)], bz[size_t(b)], bs[size_t(b)]});
  }
}

} // namespace r3d
