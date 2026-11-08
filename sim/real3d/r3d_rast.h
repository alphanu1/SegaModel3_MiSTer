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
// Bit 0 was MAME's texture page (a copy of tex page, read by nothing here); from study R135 the geometry
// sends texture alpha there (header[6] bits 2:0 from Step 1.5, bit 0 on 1.0). Lists made before carry the
// page: set_talpha() rewrites it from the polygon header for the game's step.
enum : uint32_t {
  TP_TALPHA = 0x1, TP_MIRROR_U = 0x2, TP_MIRROR_V = 0x4, TP_TEXTURE = 0x8,
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
// TP_TALPHA of every triangle from its polygon header for a game of this step (0x10, 0x15, 0x20, 0x21)
void set_talpha(rframe &f, unsigned step);

// Texture RAM as the Supermodel dump holds it: 2048 x 2048 16-bit texels, page 1
// from row 1024 (MAME's m_texture_ram[0] then [1]).
struct texram {
  std::vector<uint16_t> t = std::vector<uint16_t>(2048 * 2048);
  // MAME's texture decode for (sheet position, size, format) at a mirrored
  // coordinate u, v in [0, 2 * (32 << w)) x [0, 2 * (32 << h)), ARGB8888
  uint32_t texel(uint32_t page, uint32_t tx, uint32_t ty, uint32_t tw, uint32_t th, uint32_t fmt,
                 uint32_t u, uint32_t v, bool *outside = nullptr, bool ctr = false) const;
  // The core's decode of one texel: MAME's, except that with l8_key an L8 texel of 0xFF (formats 5
  // and 6) has alpha 0, as Supermodel's ExtractColour (study R134: Lost World's foliage, format 5 under
  // contour). Alpha only matters to the contour and texture-alpha kinds. Off: MAME's get_texture, for
  // the comparisons with MAME's renders.
  bool l8_key = true;
  // Supermodel's texture decode, contour and texture alpha (study R135, New3D's ExtractColour, GetTexFormat and
  // GetTextureValue): format 2 with its alpha in the low nibble; under contour formats 1-4 lose their alpha
  // nibble and luminance 15 is transparent (Supermodel's types 8-11), and the four corners' colours are dilated
  // from the opaque ones before the filter; contour on format 7 is no test; the contour test keeps a filtered
  // alpha of 32 or more and writes opaque; texture alpha and polygon translucency as Supermodel's alpha
  // (r3d_rast_rtl.cpp frag). Off: the form before (MAME's get_texture and draw kinds, with l8_key).
  bool sm_tex = true;
  // ctr: the polygon's contour bit (with sm_tex, formats 1-4 decode as Supermodel's types 8-11)
  uint32_t decode(int fmt, uint32_t p, bool ctr = false) const;
};
// Supermodel's colour dilation for a contour texel's four corners (texBiLinear under alphaTest): in its order,
// a corner whose alpha is below an edge neighbour's takes that neighbour's colour; alpha stays (study R135)
void sm_dilate(uint32_t c[4]);

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
struct rtl_tri;
// One pixel of a span in render_d25's order (Z-tested, or skipped by the
// stipple), with every intermediate the RTL benches compare against.
struct d25_px {
  int32_t tri = -1; int layer = 0, x = 0, y = 0; rkind k = K_NONE;
  uint32_t z = 0, zref = 0;
  bool stip_out = false, zpass = false, ok = false, zw = false, fog = false;
  uint32_t texel = 0, u = 0, v = 0; int32_t shade = 0, P = 0; int fb = -1;
  uint32_t col_raw = 0, col_fog = 0, dst = 0, out = 0;   // before fog, after fog, the colour under it, the colour written
  float K = 0; int32_t S = 0, fc[3] = {}; uint32_t fogint = 0;   // the viewport's fog record, when fog applies
  int lv0 = 0, lv1 = -1; uint32_t lfr = 0;   // mip (d25_opts::mip): the level read, the second level and its weight /256 (trilinear)
  uint32_t lodb = 0; int lvmax = 0;            // mip: the triangle's B word (lod = 1) and its highest level
};
struct d25_hook {
  virtual ~d25_hook() = default;
  virtual void tri(int32_t, int, rtl_tri const &, rkind, bool) {}   // after set-up: index, layer, set-up, kind, culled
  virtual void px(d25_px const &) {}
};
struct d25_opts {
  d25_hook *hook = nullptr;
  bool fog = true;
  int cull = 2;           // 0 none, 1 det sign xor sort parity, 2 the normal test (a flag from the geometry)
  int trans = 0;          // 0 MAME blend, 1 stipple, 2 two layers
  int band = 16;          // band rows (cycle model)
  int prod = 13;          // walker start product, cycles a (triangle, band) pair
  // Mip levels (m3-notes MIP row): 0 level 0 (MAME), 1 the nearest level of
  // Supermodel's LOD, 2 trilinear between the two levels around it. lod: 0 the
  // LOD in double as Supermodel computes it per pixel; 1 the hardware form, one
  // per-triangle term B and log2 of the pixel's 1/w Z word through a table.
  int mip = 0;
  int lod = 0;
  // lod = 1: B words by polygon number in place of the ones formed from the
  // list (the geometry's LOD word, r3d_geo lod_word); 0xffffffff = none given
  std::vector<uint32_t> const *lodb = nullptr;
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
uint32_t fetch_texel(rtri const &t, texram const &tex, uint32_t u, uint32_t v, bool &outside);
// The same at mip level L (Supermodel's sheet bases, MAME's addressing and
// filter on u, v >> L), and the four texel addresses it reads (22 bits, page 1
// from row 1024; corners 00, 01, 10, 11)
uint32_t fetch_texel_lv(rtri const &t, texram const &tex, uint32_t u, uint32_t v, int L);
void texel_addr4(rtri const &t, uint32_t u, uint32_t v, int L, uint32_t a[4]);
int mip_levels(rtri const &t);   // the highest level Supermodel reads: floor(log2(min side)) - 1
// The hardware LOD form (d25_opts::lod = 1): the triangle's B = kz^2 dot(v0, n)
// (-cota) NP as render_d25 forms it (FP32 word), and dot(v0, n); false without
// a viewport. lb8 = -256 log2(B) through the 64-entry table, INT32_MIN when B
// is not positive and finite (level 0).
bool d25_lod_terms(rframe const &f, rtri const &t, double &dv, uint32_t &bw);
int32_t d25_lod_lb8(uint32_t bw);
uint8_t d25_log2t(int i);        // 256 log2(1 + (i + 1/2) / 64), rounded
// fog_p at 24 bits (render_d25's fog depth term), for the RTL benches
int64_t d25_fog_p(__int128 o2, int eo, float K);
// render_d25's colour for one pixel (frag, then fog when fogint != 0): false when
// the alpha test discards it; rgb after fog, fb the blend factor (-1 none)
// sm, am (study R135): Supermodel's contour and texture alpha, am = d25_am(t) = {inv, at, ta, cm}
bool d25_colour(rkind k, uint32_t texel, uint32_t color, uint32_t trans, int32_t shade, uint32_t fogint,
                int32_t const fc[3], int32_t S, int32_t P, uint32_t &rgb, int &fb, bool &zw, bool sm = false, uint32_t am = 0);
uint32_t d25_am(rtri const &t);
// render_d25's scroll fog: the rectangle [x0, x1) x [y0, y1) holds col (what a
// blend reads) until drawn; a pixel left undrawn shows shown = col x alpha
struct d25_sfog { int x0 = 0, x1 = 0, y0 = 0, y1 = 0; uint32_t col = 0, shown = 0, alpha = 0; };
bool d25_scroll_fog(rframe const &f, d25_sfog &out);
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
