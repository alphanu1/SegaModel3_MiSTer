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
// Real3D rasteriser model: triangle lists (r3d_trilist.h 'T' records) and
// texture RAM in, the 496 x 384 3D layer out.
//
// Two arithmetic forms of one pipeline:
//
//   MAME   MAME 0.289's software renderer, model3_v.cpp draw_*_triangles and
//          draw_scanline_*, poly.h render_triangle, rgbutil bilinear_filter (the
//          SSE2 form an x86-64 build runs). FP32 in MAME's order, MAME's
//          float-to-integer conversions. This is the bit-level reference for the
//          stage; behaviour notes from those files (BSD-3-Clause, R. Belmont,
//          Ville Linde, Aaron Giles, Vas Crabb, Ryan Holtz) are in
//          THIRD_PARTY.md.
//   RTL    the datapath the rasteriser RTL builds: fixed-point edges and planes,
//          interpolants evaluated (not accumulated) per pixel, a table and
//          Newton reciprocal, Z held in 24 bits. See r3d_rast.cpp and
//          sim/real3d/m3-notes.md.
//
// Everything else - draw order, texel formats, mirroring, filter, blends - is
// shared, so a difference between the forms is the arithmetic and nothing else.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace r3d {

constexpr int SCR_W = 496, SCR_H = 384;

// MAME's TRI_PARAM bits, as the 'T' record carries them
enum : uint32_t {
  TP_PAGE = 0x1, TP_MIRROR_U = 0x2, TP_MIRROR_V = 0x4, TP_TEXTURE = 0x8,
  TP_CONTOUR = 0x10, TP_COLORMOD = 0x20,
};

struct rtri {
  float x[3], y[3], z[3], w[3], p0[3], p1[3], p2[3];  // p0, p1 = 256 u / w, 256 v / w; p2 = shade
  uint32_t pass, alpha, buf, poly;
  uint32_t tex_en, page, tex_x, tex_y, tex_w, tex_h, fmt, param, trans, color;
  // from the records around it: the polygon's header words, its viewport and model
  uint32_t hdr[7] = {};
  int32_t vp = -1, model = -1;
};

// A viewport ('V' record) and a model's matrix ('M' record)
struct rvp {
  uint32_t pass = 0, addr = 0;
  float x = 0, y = 0, w = 0, h = 0, near = 1, far = 1;
  float proj[16] = {}, coord[16] = {};
  uint32_t node[0x30] = {};   // the viewport node's words, filled from the dump (fog, cota)
};
struct rmodel { float top[16] = {}; };

// One frame: triangles in file order (MAME's push order).
struct rframe {
  std::string name;
  std::vector<rtri> tris;
  std::vector<rvp> vps;
  std::vector<rmodel> models;
};

bool read_rframe(std::string const &tri_file, rframe &out);

// Texture RAM as the Supermodel dump holds it: 2048 x 2048 16-bit texels, page 1
// from row 1024 (MAME's m_texture_ram[0] then [1]).
struct texram {
  std::vector<uint16_t> t = std::vector<uint16_t>(2048 * 2048);
  // MAME's texture decode for (sheet position, size, format) at a mirrored
  // coordinate u, v in [0, 2 * (32 << w)) x [0, 2 * (32 << h)), ARGB8888
  uint32_t texel(uint32_t page, uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th, uint32_t fmt,
                 uint32_t u, uint32_t v, bool *outside = nullptr) const;
};

// Draw kinds, MAME's scanline functions
enum rkind : uint8_t {
  K_NONE = 0, K_SOLID, K_SOLID_TRANS, K_TEX, K_TEX_COLORMOD, K_CONTOUR, K_TEX_ALPHA, K_COUNT
};
char const *kind_name(int k);

enum class rarith { mame, rtl };

// RTL arithmetic options (see r3d_rast.cpp)
struct rtl_opts {
  int sub = 16;       // vertex x, y fraction bits
  int zbits = 24;     // Z word: top bits of the FP32 w
  int recip_bits = 24; // 0: exact quotient; else the reciprocal's precision
  bool fixed_setup = false; // plane set-up exact from the fixed-point vertices, instead of FP32 in MAME's order
  bool ftz = true;           // set-up FP units flush subnormal operands and results to zero
  bool oow = false;          // plane 0 is 1/w (depth on 1/w) instead of w
};

struct rstats {
  uint64_t tris = 0, tris_drawn = 0, spans = 0, pixels = 0;  // pixels Z-tested
  uint64_t zpass = 0, written = 0, texels = 0;
  uint64_t tex_outside = 0;                                  // texel read past a sheet's end
  uint64_t nonfinite_tris = 0, nonfinite_px = 0;             // triangles with a non-finite x, y or w; pixels MAME writes from them
  uint64_t px_kind[K_COUNT] = {};                            // pixels Z-tested by kind
  uint64_t kind_tris[K_COUNT] = {};
  uint64_t fmt_px[8] = {};
  // per band of 8 / 16 / 32 rows: (triangle, band) pairs, and the most triangles in one band
  uint64_t tri_bands[3] = {}, band_max[3] = {};
  uint64_t polys = 0, poly_verts = 0, poly_bands[3] = {};   // polygons (fans) the triangles come from
  // cycle model: per span max(1, ceil(len / P)) for P = 1, 2, 4 pixels a clock, split
  // into opaque (solid, tex, colour-modulated), contour and translucent (the rest)
  uint64_t cyc[3][3] = {};          // [group][P = 1, 2, 4]
  uint64_t zpass_g[3] = {};         // pixels passing Z per group
  uint64_t winners = 0;             // pixels whose last write in a pass is opaque (Z pre-pass shades these)
  uint64_t tex_winners = 0;         // of which textured
};

struct rimage {
  std::vector<uint32_t> px = std::vector<uint32_t>(SCR_W * SCR_H, 0);   // ARGB, alpha 0 = not drawn
  std::vector<uint8_t> kind = std::vector<uint8_t>(SCR_W * SCR_H, K_NONE);
  std::vector<uint8_t> fmt = std::vector<uint8_t>(SCR_W * SCR_H, 0xff);  // texel format of the last write
  std::vector<int32_t> tri = std::vector<int32_t>(SCR_W * SCR_H, -1);    // index in rframe::tris of the last write
  // MAME reads past a texture sheet's end into whatever follows its array (UB).
  // A pixel is tainted when its colour, or the Z it was tested against, could
  // depend on such a read; tainted pixels are reported apart.
  std::vector<uint8_t> taint = std::vector<uint8_t>(SCR_W * SCR_H, 0);   // bit 0 colour, bit 1 Z
};

void render(rframe const &f, texram const &tex, rarith a, rtl_opts const &o, rimage &img, rstats &st);

// Supermodel form (r3d_rast_sm.cpp): behaviours switched one by one
enum : uint32_t {
  SM_COLOUR = 1u << 0, SM_DECODE = 1u << 1, SM_SAMPLE = 1u << 2, SM_ALPHA = 1u << 3, SM_TRANS = 1u << 4,
  SM_DEPTH = 1u << 5, SM_LAYER = 1u << 6, SM_CULL = 1u << 7, SM_FOG = 1u << 8, SM_MIP = 1u << 9, SM_PRIO = 1u << 10,
  SM_ALL = (1u << 11) - 1,
};
void render_sm(rframe const &f, texram const &tex, uint32_t feat, rimage &img, rstats &st);
uint32_t mame_decode(int fmt, uint32_t p);

// ---- D-25 RTL form (r3d_rast_rtl.cpp render_d25) -------------------------------
// The RTL arithmetic of render(rtl) plus the D-25 rasteriser rows, each in the
// integer form the RTL builds: fog (and scroll fog), back-face culling from the
// set-up determinant, translucency as MAME's blend, a screen-door stipple, or
// Supermodel's two layers; depth on 1/w is rtl_opts::oow. The cycle model is the
// 100 MHz half-width pipeline of D-28.
struct d25_opts {
  bool fog = true;
  int cull = 2;           // 0 none, 1 det sign xor sort parity, 2 the normal test (a flag from the geometry)
  int trans = 0;          // 0 MAME blend, 1 stipple, 2 two layers
  int band = 16;          // band rows (cycle model)
  int prod = 13;          // walker start product, cycles a (triangle, band) pair
};
struct d25_stats {
  uint64_t tris = 0, culled = 0, pairs = 0, rows = 0, spans = 0, ztest = 0, zpass = 0, shaded = 0;
  // the cull test against Supermodel's view-space test: [det verdict][SM verdict]
  uint64_t cull_tab[2][2] = {};
  uint64_t walker = 0, zwalk = 0, shade = 0;   // per stage, summed over bands (cycles)
  uint64_t frame_fifo = 0;                      // sum over bands of the slowest stage
  uint64_t frame_serial = 0;                    // sum over pairs of the slowest stage
  uint64_t setup = 0;                           // set-up unit cycles (multiplier-bound)
  uint64_t store_bytes = 0;                     // set-up triangle records into the store
  uint64_t fog_px = 0;
};
void render_d25(rframe const &f, texram const &tex, rtl_opts const &o, d25_opts const &d, rimage &img, d25_stats &st);

// ---- per-triangle evaluators, for the lockstep comparison ----------------------

rkind kind_of(rtri const &t);

// MAME: one span as poly.h hands it to draw_scanline_*
struct mame_span {
  int y, x0, x1;
  float start[5], dpdx[5];
};
void mame_spans(rtri const &t, rkind k, std::vector<mame_span> &out);

// RTL: set-up once, then spans and pixels on demand
struct rtl_px {
  uint32_t z;          // Z word
  uint32_t u, v;       // 256 u, 256 v, as MAME's uint32 conversion would give them
  int32_t shade;       // MAME's (int) of the shade
};
struct rtl_tri {
  bool setup(rtri const &t, rkind k, rtl_opts const &o);
  bool span(int32_t y, int32_t &x0, int32_t &x1) const;   // false: empty
  rtl_px eval(int x, int y) const;
  int32_t y0 = 0, y1 = 0;
  // internals
  struct plane { int64_t c = 0, dx = 0, dy = 0; int e = 0; } pl[5];
  float ps[5] = {}, pdx[5] = {}, pdy[5] = {}, det = 0, idet = 0;   // the set-up's FP32 results
  bool flat = false;                                               // |det| < 0.00001
  bool odd = false;                                                // the y sort was an odd permutation
  int64_t vx[3] = {}, vy[3] = {};   // y-sorted vertices, 2^-sub px
  float fx[3] = {}, fy[3] = {};     // the same, FP32 as the set-up record carries them
  __int128 s12 = 0, s13 = 0, s23 = 0;
  int sub = 8, zbits = 24, recip = 0, np = 2;
  bool tex = false;
};

// Lockstep: both forms on each triangle against MAME's own frame state, every
// sub-result compared where both forms produce it. Counts per draw kind.
struct lockstep {
  uint64_t spans = 0, span_diff = 0;                // (triangle, row) extents
  uint64_t px_both = 0, px_only_mame = 0, px_only_rtl = 0;
  uint64_t ztests = 0, zdiff = 0;                   // Z test outcome differs
  uint64_t texfetch = 0, tex_addr_diff = 0;         // texel u1, v1 differ
  uint64_t frac1 = 0, frac4 = 0, fracbig = 0;       // fraction differs by 1..1, 2..4, more (either axis)
  uint64_t texel_diff = 0;                          // filtered texel differs
  uint64_t shade_n = 0, shade_diff = 0;             // integer shade differs
  uint64_t colour_n = 0, colour_diff = 0;           // written colour differs (same inputs otherwise)
  int colour_maxd = 0;
  uint64_t k_px[K_COUNT] = {}, k_diff[K_COUNT] = {}; // per kind: pixels both, any sub-result differs
  // distance from the ideal value (MAME's set-up planes evaluated exactly, no
  // accumulation): u, v in 1/256 texel (0, 1, 2-4, 5-16, more), shade integer (same / differs)
  uint64_t uv_mame[5] = {}, uv_rtl[5] = {}, sh_mame_bad = 0, sh_rtl_bad = 0;
  void merge(lockstep const &o);
};
void render_lockstep(rframe const &f, texram const &tex, rtl_opts const &o, lockstep &ls);

} // namespace r3d
