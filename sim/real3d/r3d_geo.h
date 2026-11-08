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
// Real3D geometry reference model (M2): display-list traversal, matrix stack,
// polygon decode, transform and light, frustum clip, divide and viewport.
// Reads one Real3D memory dump and emits the triangle list the rasteriser
// receives, in the format of r3d_trilist.h.
//
// Behaviour follows MAME 0.289 src/mame/sega/model3_v.cpp (BSD-3-Clause,
// copyright-holders R. Belmont, Ville Linde) and the frustum clipper of
// src/devices/video/poly.h (BSD-3-Clause, Ville Linde, Aaron Giles), because
// M2's exit test is MAME's triangle list. Where Supermodel differs, the
// difference is listed in sim/real3d/m2-notes.md. Two arithmetic forms:
//
//   HOST  MAME's expressions as written: full 4x4 matrix products, libm
//         atan2/tan for the frustum, a float divide for 1/w. Checks that the
//         dump is read and walked exactly as MAME walks it.
//   RTL   the datapath the fabric will build, on FP32 round-to-nearest units:
//         affine matrices (12 words, the constant column dropped, products
//         with constant zeros skipped and their IEEE sign effect kept),
//         the bit-level Newton reciprocal, an exponent-decrement halver, and
//         the frustum half-angle tangents as y / (sqrt(x^2 + y^2) + x) in
//         place of tan(atan2(y, x) / 2).
//
// The walker keeps every MAME quirk: viewports are recursed through their
// link before being drawn, once per priority pass; pointer lists are walked
// backwards and nest at most three deep; a node's second link is skipped when
// node[0] & 7 == 6; a model list (node[0] & 8) always takes entry 0; bad
// addresses read as zeros (the census patch's behaviour, which the golden
// lists were made with).

#pragma once
#include "r3d_fp.h"
#include "r3d_trilist.h"
#include <array>
#include <algorithm>
#include <vector>

namespace r3d {

enum class arith { host, rtl };

// Memory regions as MAME holds them, in 32-bit words.
struct r3d_mem {
  std::vector<uint32_t> cull_lo = std::vector<uint32_t>(0x100000);  // 8C, culling RAM
  std::vector<uint32_t> cull_hi = std::vector<uint32_t>(0x40000);   // 8E, MAME's "display list RAM"
  std::vector<uint32_t> poly = std::vector<uint32_t>(0x100000);     // polygon RAM
  std::vector<uint32_t> vrom = std::vector<uint32_t>(0x1000000);    // 64 MiB, MAME layout
};

// Words read by the walker and decoder, by memory and by purpose.
struct read_counts {
  enum { R_LO, R_HI, R_POLY, R_VROM, R_ZERO, NR };
  enum { C_VIEWPORT, C_NODE, C_LIST, C_MATRIX, C_HEADER, C_VERTEX, C_COLOR, NC };
  uint64_t w[NR][NC] = {};
  uint64_t bursts[NR][NC] = {};
};

struct stage_counts {
  fp_counts setup, stack, model, vertex, clip, project;
  uint64_t viewports = 0, nodes = 0, lists = 0, list_entries = 0, models = 0, polys = 0;
  uint64_t verts_new = 0, verts_reused = 0, verts_out = 0, tris = 0, polys_clipped = 0;
  uint64_t polys_lit = 0, max_stack = 0, max_list_depth = 0, clip_overflow = 0;
  uint64_t recursion_cut = 0, bad_addr = 0, culled = 0, cull_tests = 0;
  // where Supermodel's walker or decoder would act differently (counts)
  uint64_t sm_vp_disabled = 0, sm_node_discard = 0, sm_node_reset = 0, sm_node_colour_table = 0;
  uint64_t sm_link_other_type = 0, sm_list_null_entry = 0, sm_lod_list = 0, sm_vp_chain_len = 0;
  uint64_t sm_poly_trans0 = 0, sm_poly_trans_bit5 = 0, sm_poly_discard_one = 0, sm_poly_discard_both = 0;
  uint64_t sm_poly_flat = 0, sm_poly_fixed_shade = 0, sm_poly_double = 0, sm_poly_texsize6 = 0;
  uint64_t sm_poly_first_shared = 0, sm_poly_indexed_colour = 0;
  // D-19 rows as followed: cases where a definition had to be chosen
  uint64_t d19_flat_reuse = 0;      // a smooth polygon reuses a vertex last written by a flat one (face normal carried)
  uint64_t d19_fixed_reuse_nf = 0;  // a fixed polygon reuses a vertex whose producer was not fixed (Supermodel: stale fixedShade)
  uint64_t d19_fixed_spec = 0;      // a fixed polygon with the specular bit (fixed shade unsigned)
  uint64_t d19_dropped = 0, d19_copies = 0, d19_ctab_models = 0, d19_ctab_nondefault = 0;
  // row 6: nodes tested, culled by distance, LOD lists by entry chosen, blends,
  // entries walked as nodes, models drawn with alpha < 1 / alpha 0
  uint64_t lod_tests = 0, lod_far = 0, lod_pick[4] = {}, lod_blend = 0, lod_node_entries = 0, lod_alpha_models = 0, lod_alpha0 = 0;
  // the facing test: single-sided polygons tested, facing away, and where the
  // FP32 form differs from the same test in double
  uint64_t face_single = 0, face_back = 0, face_vs_double = 0, face_zero = 0;
  // operations the RTL does not repeat: shared vertices keep their transform
  // and light, and their projection while the polygon is not clipped
  fp_counts cached;
};

class r3d_geo {
public:
  // setup_cr: the frustum tangents from a correctly rounded float atan2 and
  // tan (what MAME's libm calls give, checked exhaustively for tanf), in
  // place of the algebraic half-angle form
  bool setup_cr = false;
  // Supermodel's frustum (study D-19): viewport words 8-11, reversed-Z with
  // near 1e-3 and no far plane, in place of MAME's tan(atan2 / 2) half angles
  bool frustum_sm = false;
  // Supermodel-style node culling (off: MAME walks every node). A node whose
  // bounding sphere (radius: node word 7, Supermodel's 16-bit float) lies
  // outside one of the viewport's four side planes has its child skipped.
  bool cull = false;
  // Study D-19 rows of the difference list (m2-notes.md section 3); true =
  // Supermodel (New3D.cpp, R3DShaderTriangles.h), false = MAME. Lighting is
  // Supermodel's formula evaluated per vertex (the R3D-100 overview: Gouraud).
  // 18 is two switches: the RTL builds sm_n255 only (SM_FLAT); face normals
  // (sm_flat) are a model switch (study D-25: no measurable image change, 252 ALM)
  bool sm_n255 = true;    // 18: vertex normals (2b + 1) / 255 (MAME: b / 127)
  bool sm_flat = false;   // 18: flat polygons (h1 & 8 clear) light with the header's face normal (model only)
  bool sm_fixed = true;   // 20: fixed shading (h1 & 0x20, h1 & 8 clear): sunFactor from the vertex's x byte
  bool sm_double = true;  // 21: double-sided polygons (h1 & 0x10) emitted twice, the first with the normal inverted
  bool sm_light = true;   // 19, 22: normal matrix, light (n5, -n6, -n4) in GL eye space, sunFactor clamp, ambient / 255, intensity <= 1
  bool sm_trans = true;   // 17: 6-bit transparency; transparency-0 polygons dropped
  bool sm_ctab = true;    // 12: colour-table base from node words 3, 7, 8 (node[0] & 4), 12-bit index
  bool sm_lod = false;    // 6: LOD by distance and blend radius from viewport word 0x17's table, model alpha (model only)
  // R62: Supermodel's back-face test, dot(view vertex 0, view face normal) > 0,
  // in the FP32 order r3d_face.sv computes it (study R62, m2-notes section 15)
  bool sm_cull = true;
  bool sun_clamp = true;  // Mars mode word bit 18 clear: sunFactor floored at 0 (VF3 and Bass Fishing write 0)
  bool emit_sf = false;   // measurement only: unclamped sunFactor per vertex, d, a, floor after the 'T' words
  // The per-polygon LOD word (study D-33, R67; r3d_geo.sv LOD_WORD): with
  // sm_cull, each 'T' record ends with two more words, the facing dot d and
  // B = d x (NP x C) in FP32 with flush to zero (r3d_face.sv's order), NP =
  // header[5] bits 8-31 / 4096 (/ 16384 from Step 1.5), C = -cota kz^2 a viewport (cota viewport word
  // 3; kz^2 = 1 with Supermodel's frustum, else the product kz kz). The
  // inverted copy of a double-sided polygon carries -d and -B.
  bool lod_word = false;
  void rows(bool sm, bool lod = false) { sm_n255 = sm_fixed = sm_double = sm_light = sm_trans = sm_ctab = sm_cull = sm; sm_flat = false; sm_lod = sm && lod; }
  // 'T' carries words 35 (flags: bit 0 the inverted copy of a double-sided
  // polygon, bit 1 the record faces away by the face normal) and 36 (model
  // alpha) when any row that sets them is on
  bool t_ext() const { return sm_double || sm_lod || emit_sf || sm_cull; }
  r3d_geo(r3d_mem const &m, arith a, bool ftz) : mem(m), mode(a) {
    for (fp_unit *u : {&u_setup, &u_stack, &u_model, &u_vertex, &u_clip, &u_project}) u->ftz = ftz;
    u_face.ftz = true;   // r3d_face's units flush (no subnormal reaches them, monitored)
  }

  // walker events, as rtl/real3d/r3d_walker.sv emits them: {type, a, b, c}
  enum { EV_VP = 1, EV_PUSH = 2, EV_TRANS = 3, EV_MAT = 4, EV_MODEL = 5, EV_POP = 6, EV_END = 7 };
  std::vector<std::array<uint32_t, 4>> *trace = nullptr;
  // measurement only (m2-notes section 14): per 'P' record, 1 if Supermodel's
  // back-face test drops the polygon, dot(view vertex 0, view face normal) > 0
  // (never for a double-sided one), in double from the model matrix
  std::vector<uint8_t> *backface = nullptr;
  void ev(uint32_t t, uint32_t a = 0, uint32_t b = 0, uint32_t c = 0) { if (trace) trace->push_back({t, a, b, c}); }

  stage_counts st;
  read_counts rc;
  trilist_writer out;
  uint32_t step = 0x10;

  void frame(uint32_t seq);

private:
  using mat4 = std::array<std::array<float, 4>, 4>;
  struct ptr { int reg; uint32_t idx; };
  // p[2] shade, p[3] the shade of the inverted copy; sf[] measurement only
  struct cvert { float x, y, z, w, p[4]; float sf[2]; };
  // n: the normal the vertex is lit with; fs its fixed shade (Supermodel keeps
  // both in m_prev, so a reused vertex carries its producer's)
  struct vtx { float x, y, z, u, v, nx, ny, nz, fs; uint8_t flat, fixed; };
  vtx vert_[4]{};
  uint32_t ctab = 0x400;         // Supermodel's colour-table base (row 12)
  uint32_t lod_base = 0;         // row 6: the LOD blend table (viewport word 0x17)
  float malpha = 1.0f;           // row 6: the model alpha, multiplied down the node tree

  r3d_mem const &mem;
  arith mode;
  fp_unit u_setup, u_stack, u_model, u_vertex, u_clip, u_project, u_face;

  int sp = 0;
  int list_depth = 0, call_depth = 0;
  uint32_t pass = 0, poly_seq = 0;
  uint64_t tri_o = 0, tri_a = 0;

  // viewport state, as MAME keeps it between viewports
  float vp_x = 0, vp_y = 0, vp_w = 0, vp_h = 0, vp_near = 1, vp_far = 100000;
  mat4 proj{}, coord{};
  float light[3] = {}, light_i = 0, ambient = 0;
  float sm_d = 0, sm_a = 0;      // Supermodel's diffuse clamp(n7, 0, 1) and ambient byte * (1 / 255)
  uint32_t matrix_base = 0;
  float vpl[8] = {};             // viewport words 12..19 (plane terms)
  float lod_c = 0;               // lod_word: the viewport's C = -cota kz^2

  uint32_t rd(ptr p, uint32_t k, int cat);
  uint32_t pk(ptr p, uint32_t k) const;   // a word the walker already holds (not counted)
  ptr node_ptr(uint32_t address);
  void load_matrix(int n, mat4 &o, int cat);
  void draw_viewport(uint32_t address);
  void set_projection_host(float l, float r, float t, float b);
  void process_link(uint32_t link);
  void draw_block(uint32_t address);
  void traverse_list(uint32_t address);
  void draw_model(uint32_t addr);
  void light_sm(cvert &c, vtx const &v, mat4 const &T, bool fixed, bool floor, fp_unit &u);
  int clip_all(cvert *v, int n);
  int clip_w(cvert *v, int n);
  int clip_plane(cvert *v, int n, int axis, bool sign);
};

} // namespace r3d
