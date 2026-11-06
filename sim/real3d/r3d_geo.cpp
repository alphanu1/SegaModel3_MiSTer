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
// Real3D geometry reference model. See r3d_geo.h. Behaviour transcribed from
// MAME 0.289 model3_v.cpp (BSD-3-Clause, R. Belmont, Ville Linde) and
// poly.h's frustum clipper (BSD-3-Clause, Ville Linde, Aaron Giles).

#include "r3d_geo.h"
#include <cmath>
#include <limits>

namespace r3d {

namespace {

// A matrix with a mask of entries that are structural constants of the
// datapath (bit i*4+j). Products with a constant 0 or +-1 are not issued to a
// multiplier, sums with a constant-zero term not to an adder; their IEEE
// value (the sign of a zero, NaN from 0 * inf) is still computed, which is
// what the RTL's sign logic must reproduce.
struct cmat {
  std::array<std::array<float, 4>, 4> m{};
  uint16_t k = 0;
  bool c(int i, int j) const { return (k >> (i * 4 + j)) & 1; }
};

bool trivial(float x) { return x == 0.0f || x == 1.0f || x == -1.0f; }

void mul_masked(fp_unit &u, cmat const &a, cmat const &b, cmat &o) {
  cmat t;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0;
      bool s_zero = true, all_const = true;
      for (int k = 0; k < 4; k++) {
        float const x = a.m[i][k], y = b.m[k][j];
        bool const xc = a.c(i, k), yc = b.c(k, j);
        bool const zero = (xc && x == 0.0f) || (yc && y == 0.0f);
        bool const free_mul = (xc && trivial(x)) || (yc && trivial(y)) || (xc && yc);
        float const p = free_mul ? u.flush(u.flush(x) * u.flush(y)) : u.mul(x, y);
        if (!(xc && yc) && !zero) all_const = false;
        if (k == 0) { s = p; s_zero = zero; continue; }
        if (zero || s_zero) s = u.flush(s + p);  // sign logic, no adder
        else s = u.add(s, p);
        s_zero = s_zero && zero;
      }
      t.m[i][j] = s;
      if (all_const) t.k |= uint16_t(1u << (i * 4 + j));
    }
  o = t;
}

cmat identity() {
  cmat r;
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) r.m[i][j] = (i == j) ? 1.0f : 0.0f;
  r.k = 0xffff;
  return r;
}

// column 3 of every stack and node matrix is (0, 0, 0, 1)
constexpr uint16_t COL3 = (1u << 3) | (1u << 7) | (1u << 11) | (1u << 15);

// one stack a thread: the model runs in several threads at once (r3d_tcache_sim, r3d_rast_ref --geo-lod)
thread_local std::vector<cmat> g_stack(256);

} // namespace

uint32_t r3d_geo::rd(ptr p, uint32_t k, int cat) {
  uint64_t idx = uint64_t(p.idx) + k;
  switch (p.reg) {
    case read_counts::R_LO:   rc.w[p.reg][cat]++; return idx < mem.cull_lo.size() ? mem.cull_lo[idx] : 0;
    case read_counts::R_HI:   rc.w[p.reg][cat]++; return idx < mem.cull_hi.size() ? mem.cull_hi[idx] : 0;
    case read_counts::R_POLY: rc.w[p.reg][cat]++; return idx < mem.poly.size() ? mem.poly[idx] : 0;
    case read_counts::R_VROM: rc.w[p.reg][cat]++; return idx < mem.vrom.size() ? mem.vrom[idx] : 0;
    default:                  rc.w[read_counts::R_ZERO][cat]++; return 0;
  }
}

uint32_t r3d_geo::pk(ptr p, uint32_t k) const {
  uint64_t const idx = uint64_t(p.idx) + k;
  switch (p.reg) {
    case read_counts::R_LO: return idx < mem.cull_lo.size() ? mem.cull_lo[idx] : 0;
    case read_counts::R_HI: return idx < mem.cull_hi.size() ? mem.cull_hi[idx] : 0;
    default: return 0;
  }
}

// MAME get_memory_pointer, with the census patch's zero page for bad addresses
r3d_geo::ptr r3d_geo::node_ptr(uint32_t address) {
  if (address & 0x800000) {
    if (address >= 0x840000) { st.bad_addr++; return {read_counts::R_ZERO, 0}; }
    return {read_counts::R_HI, address & 0x7fffff};
  }
  if (address >= 0x100000) { st.bad_addr++; return {read_counts::R_ZERO, 0}; }
  return {read_counts::R_LO, address};
}

void r3d_geo::load_matrix(int n, mat4 &o, int cat) {
  ptr const p = node_ptr(matrix_base + uint32_t(n) * 12);
  float w[12];
  for (int i = 0; i < 12; i++) w[i] = bitsf(rd(p, i, cat));
  o[0] = {w[3], w[6], w[9], 0.0f};
  o[1] = {w[4], w[7], w[10], 0.0f};
  o[2] = {w[5], w[8], w[11], 0.0f};
  o[3] = {w[0], w[1], w[2], 1.0f};
}

void r3d_geo::set_projection_host(float l, float r, float t, float b) {
  float const n = vp_near, f = vp_far;
  for (auto &row : proj) row.fill(0.0f);
  proj[0][0] = (2.0f * n) / (l - r);
  proj[0][2] = (r + l) / (r - l);
  proj[1][1] = (2.0f * n) / (t - b);
  proj[1][2] = (t + b) / (t - b);
  proj[2][2] = -(f + n) / (f - n);
  proj[2][3] = -(2.0f * f * n) / (f - n);
  proj[3][2] = -1.0f;
}

void r3d_geo::draw_viewport(uint32_t address) {
  if (++call_depth > 4096) { st.recursion_cut++; call_depth--; return; }
  ptr const node = node_ptr(address);
  uint32_t const node0 = rd(node, 0, read_counts::C_VIEWPORT);
  uint32_t const link_address = rd(node, 1, read_counts::C_VIEWPORT);
  if (link_address != 0x01000000 && link_address != 0) draw_viewport(link_address);
  call_depth--;
  if (pass != ((node0 >> 3) & 3)) return;
  st.viewports++;
  if (node0 & 0x20) st.sm_vp_disabled++;

  auto W = [&](int k) { return rd(node, k, read_counts::C_VIEWPORT); };
  auto F = [&](int k) { return bitsf(W(k)); };
  vp_x = float(W(26) & 0xffff) / 16.0f;
  vp_y = float((W(26) >> 16) & 0xffff) / 16.0f;
  vp_w = float(W(20) & 0xffff) / 4.0f;
  vp_h = float((W(20) >> 16) & 0xffff) / 4.0f;
  vp_near = 1.0f;
  vp_far = 100000.0f;

  float l, r, t, b;
  if (frustum_sm) {
    // Supermodel (New3D.cpp, Mat4::FrustumRZ): the frustum from viewport words
    // 8-11 in CPU float, reversed-Z projection with near 1e-3 and no far plane.
    // Row-vector form (clip = v P): x_c = x X + a Z, y_c = y Y + b Z,
    // z_c = near W, w_c = -Z
    fp_unit &u = u_setup;
    float const cv = F(8), cw = F(9), io = F(10), jo = F(11);
    float const left = u.div(u.sub(0.0f, jo), cv);
    float const right = u.div(u.sub(1.0f, jo), cv);
    float const bottom = u.div(-u.sub(1.0f, io), cw);
    float const top = u.div(-u.sub(0.0f, io), cw);
    vp_near = 1e-3f;
    vp_far = 0.0f;
    for (auto &row : proj) row.fill(0.0f);
    float const rl = u.sub(right, left), tb = u.sub(top, bottom);
    proj[0][0] = u.div(2.0f, rl);
    proj[2][0] = u.div(u.sub(right, -left), rl);   // right + left, as the RTL forms it
    proj[1][1] = u.div(2.0f, tb);
    proj[2][1] = u.div(u.sub(top, -bottom), tb);
    proj[3][2] = vp_near;
    proj[2][3] = -1.0f;
    (void)l; (void)r; (void)t; (void)b;
  } else {
  if (mode == arith::host || setup_cr) {
    // MAME: tan(atan2(y, x) / 2), atan2 in double (the overload MAME's
    // unqualified call resolves to), tanf in float
    float const fl = float(std::atan2(double(F(12)), double(F(13))));
    float const fr = -float(std::atan2(double(F(16)), double(-F(17))));
    float const ft = float(std::atan2(double(F(14)), double(F(15))));
    float const fb = -float(std::atan2(double(F(18)), double(-F(19))));
    l = vp_near * tanf(fl * 0.5f);
    r = vp_near * tanf(fr * 0.5f);
    t = vp_near * tanf(ft * 0.5f);
    b = vp_near * tanf(fb * 0.5f);
  } else {
    // tan(atan2(y, x) / 2) = y / (sqrt(x^2 + y^2) + x)
    fp_unit &u = u_setup;
    auto half_tan = [&](float y, float x) {
      float const rr = u.sqrt(u.add(u.mul(x, x), u.mul(y, y)));
      return u.div(y, u.add(rr, x));
    };
    l = half_tan(F(12), F(13));
    r = -half_tan(F(16), -F(17));
    t = half_tan(F(14), F(15));
    b = -half_tan(F(18), -F(19));
  }
  set_projection_host(l, r, t, b);
  if (mode == arith::rtl && !setup_cr) u_setup.n.div += 4, u_setup.n.add += 6, u_setup.n.mul += 2;
  }

  if (lod_word) {
    // C = -cota kz^2 (r3d_geo.sv M_VPW: word 3 into the facing test; with
    // MAME's frustum kz is a constant and the test forms kz^2 cota once)
    float const kz = -proj[2][3];
    float const ncota = -F(3);
    lod_c = frustum_sm ? ncota : u_face.mul(u_face.mul(kz, kz), ncota);
  }
  matrix_base = W(22);
  lod_base = W(0x17) & 0xffffff;
  malpha = 1.0f;
  for (int k = 0; k < 8; k++) vpl[k] = F(12 + k);
  ev(EV_VP, address, matrix_base, pass);
  light[0] = F(5);
  light[1] = F(6);
  light[2] = F(4);
  light_i = F(7);
  ambient = float(uint8_t(W(36) >> 8)) / 256.0f;
  // Supermodel's sun is (n5, -n6, -n4) in GL eye space. MAME's view space is
  // GL eye space with y negated (coordinate matrix diag(c01, c12, -c20)
  // against Supermodel's fix-up x matrix 0), so in MAME's space it is
  // (n5, n6, -n4): only z changes sign. The 'V' record keeps MAME's.
  sm_d = std::max(0.0f, std::min(F(7), 1.0f));
  sm_a = float(uint8_t(W(36) >> 8)) * float(1.0 / 255.0);
  load_matrix(0, coord, read_counts::C_MATRIX);

  out.begin('V'); out.w(pass); out.w(address); out.w(node0);
  out.f(vp_x); out.f(vp_y); out.f(vp_w); out.f(vp_h); out.f(vp_near); out.f(vp_far);
  for (auto &row : proj) for (float x : row) out.f(x);
  for (auto &row : coord) for (float x : row) out.f(x);
  out.f(light[0]); out.f(light[1]); out.f(light[2]); out.f(light_i); out.f(ambient); out.w(matrix_base);
  for (int k = 12; k < 20; k++) out.w(W(k));
  out.end();

  process_link(W(2));
}

void r3d_geo::process_link(uint32_t link) {
  if (link == 0 || link == 0x0fffffff || link == 0x00800800 || link == 0x01000000) return;
  { uint32_t const t = link >> 24; if (t != 0 && t != 1 && t != 3 && t != 4) st.sm_link_other_type++; }
  switch (link >> 24) {
    case 0x00: draw_block(link & 0xffffff); break;
    case 0x01:
    case 0x03: draw_model(link & 0xffffff); break;
    case 0x04: traverse_list(link & 0xffffff); break;
    default: break;
  }
}

void r3d_geo::draw_block(uint32_t address) {
  if (++call_depth > 4096) { st.recursion_cut++; call_depth--; return; }
  st.nodes++;
  ptr const node = node_ptr(address);
  int const off = (step < 0x15) ? 2 : 0;
  // step 1.0 nodes are 7 words (0, 3..8 of the step 1.5 layout)
  uint32_t const n0 = rd(node, 0, read_counts::C_NODE);
  uint32_t const n3 = rd(node, 3 - off, read_counts::C_NODE);
  uint32_t const link = rd(node, 7 - off, read_counts::C_NODE);
  int const node_matrix = int(n3 & 0xfff);
  // Supermodel's colour-table base: set by any node with node[0] & 4, kept
  // until the next one (a renderer register, not a node attribute)
  if (sm_ctab && (n0 & 0x04))
    ctab = ((n3 >> 19) | ((link >> 28) << 13) | ((pk(node, 8 - off) >> 25) << 17)) & 0xfffff;
  if ((n0 & 0x300) == 0x300) st.sm_node_discard++;
  if (n0 & 0x80) st.sm_node_reset++;
  if (n0 & 0x04) st.sm_node_colour_table++;
  if (n0 & 0x08) st.sm_lod_list++;
  if ((n0 & 0x10) || node_matrix == 0)
    (void)node_ptr(matrix_base + uint32_t(node_matrix) * 12);  // MAME loads it unconditionally

  // push
  sp++;
  if (sp >= 256) { st.recursion_cut++; sp--; call_depth--; return; }
  g_stack[sp] = g_stack[sp - 1];
  st.max_stack = std::max<uint64_t>(st.max_stack, uint64_t(sp));

  ev(EV_PUSH);
  if (n0 & 0x10) {
    uint32_t const tx = rd(node, 4 - off, read_counts::C_NODE);
    uint32_t const ty = rd(node, 5 - off, read_counts::C_NODE);
    uint32_t const tz = rd(node, 6 - off, read_counts::C_NODE);
    ev(EV_TRANS, tx, ty, tz);
    cmat tm = identity();
    tm.m[3][0] = bitsf(tx);
    tm.m[3][1] = bitsf(ty);
    tm.m[3][2] = bitsf(tz);
    tm.k = uint16_t(0xffff & ~((1u << 12) | (1u << 13) | (1u << 14)));
    mul_masked(u_stack, tm, g_stack[sp], g_stack[sp]);
  } else if (node_matrix != 0) {
    ev(EV_MAT, matrix_base + uint32_t(node_matrix) * 12);
    cmat mm;
    load_matrix(node_matrix, mm.m, read_counts::C_MATRIX);
    mm.k = COL3;
    mul_masked(u_stack, mm, g_stack[sp], g_stack[sp]);
  }

  bool outside = false;
  // view-space centre, Supermodel's modelview translation: u = (s, 1) x coord
  // (stack top row 3 times the full coordinate matrix), x = u1, y = -u2, z = -u0
  float u[3] = {};
  if (cull || sm_lod) {
    auto const &S = g_stack[sp].m;
    for (int i = 0; i < 3; i++) {
      float t = S[3][0] * coord[0][i];
      t = (S[3][1] * coord[1][i]) + t;
      t = (S[3][2] * coord[2][i]) + t;
      u[i] = (t * 1.0f) + coord[3][i];
    }
  }
  // Supermodel's 16-bit float (R3DFloat::GetFloat16)
  auto f16 = [](uint32_t r16) {
    uint32_t const a1 = (r16 & 0xffff) << 15;
    uint32_t const e6 = (a1 >> 25) & 0x3f;
    return bitsf(((e6 <= 31 ? e6 + 127 : e6 + 63) << 23) | ((a1 & 0x1ffffff) >> 2));
  };
  if (cull) {
    st.cull_tests++;
    uint32_t const r16 = rd(node, 9 - off, read_counts::C_NODE) & 0xffff;
    float const R = f16(r16);
    float const A[4] = {-vpl[0], -vpl[2], -vpl[4], -vpl[6]};
    float const B[4] = {-vpl[1], 0.0f, -vpl[5], 0.0f};
    float const C[4] = {0.0f, -vpl[3], 0.0f, -vpl[7]};
    for (int p = 0; p < 4; p++) {
      float t = A[p] * u[0];
      t = (B[p] * u[1]) + t;
      t = (C[p] * u[2]) + t;
      if (t > R) outside = true;
      if (std::getenv("R3D_CULLDBG")) std::printf("M %llu t%d %08X R %08X\n", (unsigned long long)st.cull_tests, p, fbits(t), fbits(R));
    }
    if (outside) st.culled++;
    if (std::getenv("R3D_CULLDBG") && (st.cull_tests % 200) == 0) std::printf("model culls %llu of %llu\n", (unsigned long long)st.culled, (unsigned long long)st.cull_tests);
  }
  // Row 6, Supermodel's LOD (DescendCullingNode): LODscale = blend radius /
  // |centre| (FP32: squares summed in order, sqrtf, a divide), the node's
  // entry of the viewport's LOD table (4 x {delete size, blend factor});
  // below the last delete size the child is not drawn; a LOD list takes the
  // first entry in reach and fades into the next; every other node fades by
  // the last entry. The model alpha multiplies down the tree.
  float const saved_alpha = malpha;
  float lscale = std::numeric_limits<float>::max();
  float lds[4] = {}, lbf[4] = {};
  bool far = false;
  uint32_t const lptr = (n3 >> 12) & 0x7f;
  bool const nolod = lptr == 0x7f;
  if (sm_lod && !nolod) {
    st.lod_tests++;
    float const Rb = f16(rd(node, 9 - off, read_counts::C_NODE) >> 16);
    float r2 = u[0] * u[0];
    r2 = (u[1] * u[1]) + r2;
    r2 = (u[2] * u[2]) + r2;
    lscale = Rb / std::sqrt(r2);
    lscale = std::clamp(lscale, 0.0f, std::numeric_limits<float>::max());
    ptr const tp = node_ptr(lod_base + lptr * 8);
    for (int m = 0; m < 4; m++) { lds[m] = bitsf(rd(tp, uint32_t(2 * m), read_counts::C_LIST)); lbf[m] = bitsf(rd(tp, uint32_t(2 * m + 1), read_counts::C_LIST)); }
    far = !(lscale >= lds[3]);
    if (far && !outside) st.lod_far++;
  }
  if (outside || far) {
    // child skipped
  } else if (n0 & 0x08) {
    ptr const list = node_ptr(link & 0xffffff);
    if (!sm_lod) draw_model(rd(list, 0, read_counts::C_LIST) & 0xffffff);
    else {
      uint32_t lp[4];
      for (int m = 0; m < 4; m++) lp[m] = rd(list, uint32_t(m), read_counts::C_LIST);
      int m = 0;
      if (nolod) m = 0;   // no table: LODscale is the largest float, entry 0 if valid
      while (m < 3 && !(lscale >= lds[m] && (lp[m] & 0x1000000))) m++;
      st.lod_pick[m]++;
      if (std::getenv("R3D_LODDBG")) std::printf("LOD list node %06X ptr %u scale %g lp %08X %08X %08X %08X ds %g %g %g %g bf %g %g %g %g -> %d\n", address, lptr, double(lscale),
                                                 lp[0], lp[1], lp[2], lp[3], double(lds[0]), double(lds[1]), double(lds[2]), double(lds[3]),
                                                 double(lbf[0]), double(lbf[1]), double(lbf[2]), double(lbf[3]), m);
      float na = nolod ? 1.0f : std::clamp(lbf[m] * (lscale - lds[m]), 0.0f, 1.0f);
      if (na > float(31.0 / 32.0)) na = 1.0f;
      else if (na < float(1.0 / 32.0)) na = 0.0f;
      float const temp = malpha;
      malpha *= na;
      bool const as_node = !(link & 0x01000000);
      if (as_node) st.lod_node_entries++;
      auto walk = [&](uint32_t e) { if (as_node) draw_block(e & 0xffffff); else draw_model(e & 0xffffff); };
      walk(lp[m]);
      if (na < 1.0f && m != 3) {
        st.lod_blend++;
        malpha = (1.0f - na) * temp;
        walk(lp[m + 1]);
      }
    }
  } else {
    if (sm_lod && !nolod) malpha *= std::clamp(lbf[3] * (lscale - lds[3]), 0.0f, 1.0f);
    process_link(link);
  }
  malpha = saved_alpha;

  sp--;
  ev(EV_POP);
  if ((n0 & 0x7) != 0x6) process_link(rd(node, 8 - off, read_counts::C_NODE));
  call_depth--;
}

void r3d_geo::traverse_list(uint32_t address) {
  ptr const list = node_ptr(address);
  if (list_depth > 2) return;
  list_depth++;
  st.lists++;
  st.max_list_depth = std::max<uint64_t>(st.max_list_depth, uint64_t(list_depth));
  uint32_t n = 0;
  for (;;) {
    uint32_t const a = rd(list, n++, read_counts::C_LIST);
    st.list_entries++;
    if (a & 0x02000000) break;
    if (a & 0x01000000) st.sm_list_null_entry++;
    if (a == 0 || (a >> 24) != 0) { n--; break; }
    if (n > 0x100000) break;
  }
  // walked backwards: entries are already latched, no second read
  std::vector<uint32_t> e(n);
  for (uint32_t i = 0; i < n; i++) {
    uint64_t const idx = uint64_t(list.idx) + i;
    e[i] = list.reg == read_counts::R_LO ? (idx < mem.cull_lo.size() ? mem.cull_lo[idx] : 0)
         : list.reg == read_counts::R_HI ? (idx < mem.cull_hi.size() ? mem.cull_hi[idx] : 0) : 0;
  }
  while (n > 0) {
    uint32_t const a = e[--n] & 0xffffff;
    if (a != 0 && a != 0x800800) draw_block(a);
  }
  list_depth--;
}

// ---- polygon decode, transform, light, clip, project ------------------------

int r3d_geo::clip_w(cvert *v, int n) {
  if (n <= 0) return 0;
  float const W_PLANE = 0.000001f;
  cvert c[16];
  int m = 0, prev = n - 1;
  fp_unit &u = u_clip;
  for (int i = 0; i < n; i++) {
    int const s1 = (v[i].w < W_PLANE) ? -1 : 1;
    int const s2 = (v[prev].w < W_PLANE) ? -1 : 1;
    if (s1 * s2 < 0) {
      float const wdiv = u.sub(v[prev].w, v[i].w);
      if (wdiv == 0.0f) return 0;
      float const t = std::fabs(u.div(u.sub(W_PLANE, v[prev].w), wdiv));
      auto L = [&](float a, float b) { return u.add(a, u.mul(u.sub(b, a), t)); };
      if (m < 16) {
        c[m].x = L(v[prev].x, v[i].x); c[m].y = L(v[prev].y, v[i].y);
        c[m].z = L(v[prev].z, v[i].z); c[m].w = L(v[prev].w, v[i].w);
        for (int k = 0; k < 4; k++) c[m].p[k] = L(v[prev].p[k], v[i].p[k]);
        for (int k = 0; k < 2; k++) c[m].sf[k] = v[prev].sf[k] + (v[i].sf[k] - v[prev].sf[k]) * t;
      }
      m++;
    }
    if (s1 > 0) { if (m < 16) c[m] = v[i]; m++; }
    prev = i;
  }
  if (m > 10) st.clip_overflow++;
  m = std::min(m, 16);
  std::copy(c, c + m, v);
  return m;
}

int r3d_geo::clip_plane(cvert *v, int n, int axis, bool sign) {
  if (n <= 0) return 0;
  cvert c[16];
  int m = 0, prev = n - 1;
  fp_unit &u = u_clip;
  auto ax = [&](cvert const &q) { float const a = axis == 0 ? q.x : axis == 1 ? q.y : q.z; return sign ? a : -a; };
  for (int i = 0; i < n; i++) {
    float const a1 = ax(v[i]), a2 = ax(v[prev]);
    int const s1 = (a1 <= v[i].w) ? 1 : -1;
    int const s2 = (a2 <= v[prev].w) ? 1 : -1;
    if (s1 * s2 < 0) {
      float const wdiv = u.sub(u.sub(v[prev].w, a2), u.sub(v[i].w, a1));
      if (wdiv == 0.0f) return 0;
      float const t = std::fabs(u.div(u.sub(v[prev].w, a2), wdiv));
      auto L = [&](float a, float b) { return u.add(a, u.mul(u.sub(b, a), t)); };
      if (m < 16) {
        c[m].x = L(v[prev].x, v[i].x); c[m].y = L(v[prev].y, v[i].y);
        c[m].z = L(v[prev].z, v[i].z); c[m].w = L(v[prev].w, v[i].w);
        for (int k = 0; k < 4; k++) c[m].p[k] = L(v[prev].p[k], v[i].p[k]);
        for (int k = 0; k < 2; k++) c[m].sf[k] = v[prev].sf[k] + (v[i].sf[k] - v[prev].sf[k]) * t;
      }
      m++;
    }
    if (s1 > 0) { if (m < 16) c[m] = v[i]; m++; }
    prev = i;
  }
  if (m > 10) st.clip_overflow++;
  m = std::min(m, 16);
  std::copy(c, c + m, v);
  return m;
}

int r3d_geo::clip_all(cvert *v, int n) {
  // all inside every plane: MAME's clip returns the polygon unchanged
  bool inside = true;
  for (int i = 0; i < n; i++) {
    cvert const &q = v[i];
    if (q.w < 0.000001f || !(-q.x <= q.w) || !(q.x <= q.w) || !(-q.y <= q.w) || !(q.y <= q.w) ||
        !(-q.z <= q.w) || !(q.z <= q.w)) inside = false;
  }
  if (inside) return n;
  st.polys_clipped++;
  n = clip_w(v, n);
  n = clip_plane(v, n, 0, false);
  n = clip_plane(v, n, 0, true);
  n = clip_plane(v, n, 1, false);
  n = clip_plane(v, n, 1, true);
  n = clip_plane(v, n, 2, false);
  n = clip_plane(v, n, 2, true);
  return n;
}

// Supermodel's lighting (R3DShaderTriangles.h) evaluated once per vertex, in
// the operations the T&L kernel issues. sm_light: n' = n x T (MAME uses the
// transpose), L = (n5, n6, -n4) in MAME's view space, sunFactor clamped to
// [-1, 1] and floored at 0 (sun clamp or translucency), I = min(sf d + a, 1),
// shade = I * 255; otherwise MAME's max(dot, 0), dot Ip + amb, clamp to 255.
// A fixed-shade vertex is staged with a zero normal and its fixed shade as F,
// so the dot product yields F. p[3] is the inverted copy's: -n gives
// sf' = N2 (-L2) + (-Dp), exact.
void r3d_geo::light_sm(cvert &c, vtx const &v, mat4 const &T, bool fixed, bool floor, fp_unit &u) {
  float const L0 = light[0], L1 = light[1], L2 = sm_light ? -light[2] : light[2];
  float const nx = fixed ? 0.0f : v.nx, ny = fixed ? 0.0f : v.ny, nz = fixed ? 0.0f : v.nz;
  float const F = fixed ? v.fs : -0.0f;
  float N[3];
  for (int i = 0; i < 3; i++)
    N[i] = sm_light ? u.add(u.add(u.mul(nx, T[0][i]), u.mul(ny, T[1][i])), u.mul(nz, T[2][i]))
                    : u.add(u.add(u.mul(nx, T[i][0]), u.mul(ny, T[i][1])), u.mul(nz, T[i][2]));
  float Dp = u.add(u.mul(N[0], L0), F);
  Dp = u.add(u.mul(N[1], L1), Dp);
  float const s[2] = {u.add(u.mul(N[2], L2), Dp), u.add(u.mul(N[2], -L2), -Dp)};
  for (int k = 0; k < 2; k++) {
    c.sf[k] = s[k];
    float r;
    if (sm_light) {
      float x = std::clamp(s[k], -1.0f, 1.0f);
      if (floor) x = std::max(x, 0.0f);
      r = u.mul(std::min(u.add(u.mul(x, sm_d), sm_a), 1.0f), 255.0f);
    } else {
      r = std::clamp(u.mul(u.add(u.mul(std::max(s[k], 0.0f), light_i), ambient), 255.0f), 0.0f, 255.0f);
    }
    c.p[2 + k] = r;
  }
}

void r3d_geo::draw_model(uint32_t addr) {
  ev(EV_MODEL, addr, sm_ctab ? ctab : 0x400);
  st.models++;
  if (sm_lod && malpha < 1.0f) { st.lod_alpha_models++; if (malpha == 0.0f) st.lod_alpha0++; }
  ptr const model = addr >= 0x100000 ? ptr{read_counts::R_VROM, addr} : ptr{read_counts::R_POLY, addr};
  uint32_t index = 0;
  bool last = false, first = true;

  // per model: transform = top x coord, vp = transform x projection
  cmat cm;
  for (auto &row : cm.m) row.fill(0.0f);
  cm.m[0][0] = coord[0][1];
  cm.m[1][1] = coord[1][2];
  cm.m[2][2] = -coord[2][0];
  cm.m[3][3] = 1.0f;
  cm.k = uint16_t(0xffff & ~((1u << 0) | (1u << 5) | (1u << 10)));
  cmat pm;
  pm.m = proj;
  // projection constants: the zeros, -1, and the near/far terms
  pm.k = frustum_sm ? uint16_t(0xffff & ~((1u << 0) | (1u << 8) | (1u << 5) | (1u << 9)))
                    : uint16_t(0xffff & ~((1u << 0) | (1u << 2) | (1u << 5) | (1u << 6)));
  cmat tr, vp;
  mul_masked(u_model, g_stack[sp], cm, tr);
  mul_masked(u_model, tr, pm, vp);

  out.begin('M'); out.w(pass); out.w(addr); out.w(uint32_t(sp)); out.w(uint32_t(list_depth));
  for (auto &row : g_stack[sp].m) for (float x : row) out.f(x);
  for (auto &row : vp.m) for (float x : row) out.f(x);
  out.end();

  // MAME zeroes prev[] per model but leaves vertex[] uninitialised, so a
  // reuse of slot 3 before any quad in the model reads stale stack data.
  // Here vert[] persists across models, as Supermodel's m_prev does.
  vtx *const vert = vert_;
  vtx prev[4]{};
  float const fpf = (step < 0x15) ? 1.0f / 128.0f : 1.0f / 2048.0f;

  auto exitrec = [&](uint32_t off, uint32_t why) {
    out.begin('X'); out.w(pass); out.w(addr); out.w(off); out.w(why); out.end();
  };

  while (!last) {
    uint32_t const off = index;
    uint32_t h[7];
    for (int i = 0; i < 7; i++) h[i] = rd(model, index++, read_counts::C_HEADER);
    if (first && (h[0] & 0x0f) != 0) { st.sm_poly_first_shared++; exitrec(off, 1); return; }
    first = false;
    if (h[6] == 0) { exitrec(off, 2); return; }
    if (h[1] & 0x4) last = true;
    if ((h[0] & 0x300) == 0x300) { st.sm_poly_discard_both++; exitrec(off, 3); return; }
    if (h[0] & 0x300) st.sm_poly_discard_one++;
    if (!(h[1] & 0x8)) st.sm_poly_flat++;
    if ((h[1] & 0x20) && !(h[1] & 0x8)) st.sm_poly_fixed_shade++;
    if (h[1] & 0x10) st.sm_poly_double++;
    if (!(h[6] & 0x800000) && ((h[6] >> 18) & 0x3f) == 0) st.sm_poly_trans0++;
    if (!(h[6] & 0x800000) && ((h[6] >> 23) & 1) == 0 && ((h[6] >> 18) & 0x20)) st.sm_poly_trans_bit5++;
    if (!(h[1] & 0x2)) st.sm_poly_indexed_colour++;
    int nv = (h[0] & 0x40) ? 4 : 3;
    float const tscale = (h[1] & 0x40) ? 1.0f : (1.0f / 8.0f);

    bool const smooth = (h[1] & 0x8) != 0;
    bool const fixed = sm_fixed && (h[1] & 0x20) && !smooth;
    bool const spec = (h[0] & 0x80) != 0;
    auto b2f = [](uint32_t b) { return float(2 * int(int8_t(b)) + 1) * float(1.0 / 255.0); };   // Supermodel BYTE_TO_FLOAT

    int vi = 0, reused = 0;
    for (int v = 0; v < 4; v++)
      if (h[0] & (1 << v)) {
        if (fixed && !prev[v].fixed && vi < nv) st.d19_fixed_reuse_nf++;
        if (sm_flat && smooth && prev[v].flat && vi < nv) st.d19_flat_reuse++;
        vert[vi++] = prev[v]; reused++;
      }
    if (fixed && spec) st.d19_fixed_spec++;
    for (; vi < nv; vi++) {
      uint32_t const xw = rd(model, index++, read_counts::C_VERTEX);
      uint32_t const yw = rd(model, index++, read_counts::C_VERTEX);
      uint32_t const zw = rd(model, index++, read_counts::C_VERTEX);
      uint32_t const uv = rd(model, index++, read_counts::C_VERTEX);
      // 17.7 fixed point: a 24-bit integer times 2^-7, exact in FP32
      vert[vi].x = float(int32_t(xw) >> 8) * fpf;
      vert[vi].y = float(int32_t(yw) >> 8) * fpf;
      vert[vi].z = float(int32_t(zw) >> 8) * fpf;
      vert[vi].u = float(uint16_t(uv >> 16));
      vert[vi].v = float(uint16_t(uv));
      if (sm_n255 || sm_flat) {
        // Supermodel: (2b + 1) / 255
        vert[vi].nx = b2f(xw); vert[vi].ny = b2f(yw); vert[vi].nz = b2f(zw);
      } else {
        // s.7 normal / 127: a 256-entry constant table in the RTL
        vert[vi].nx = float(int8_t(xw)) / 127.0f;
        vert[vi].ny = float(int8_t(yw)) / 127.0f;
        vert[vi].nz = float(int8_t(zw)) / 127.0f;
      }
      // the fixed shade of the x byte, signed or (specular) unsigned; every
      // vertex keeps the one its producing polygon gave it
      vert[vi].fs = spec ? float(xw & 0xff) * float(1.0 / 255.0) : b2f(xw);
      vert[vi].flat = !smooth;
      vert[vi].fixed = fixed;
    }
    if (sm_flat && !smooth) {
      // every vertex of a flat polygon takes the face normal, and keeps it in
      // the shared slots (Supermodel's m_prev)
      float fn[3];
      for (int k = 0; k < 3; k++) fn[k] = float(int32_t(h[1 + k]) >> 8) * float(1.0 / 4194304.0);
      for (int i = 0; i < nv; i++) { vert[i].nx = fn[0]; vert[i].ny = fn[1]; vert[i].nz = fn[2]; vert[i].flat = 1; }
    }
    std::copy(vert, vert + 4, prev);

    uint32_t color;
    if (h[1] & 0x2) color = (h[4] >> 8) & 0xffffff;
    else if (sm_ctab) {
      color = rd({read_counts::R_POLY, (ctab + ((h[4] >> 8) & 0xfff)) & 0xfffff}, 0, read_counts::C_COLOR);
      st.d19_ctab_models += ctab != 0x400;
    } else {
      color = rd({read_counts::R_POLY, 0x400}, (h[4] >> 8) & 0x7ff, read_counts::C_COLOR);
    }
    // MAME: 5 bits, 32 opaque. Supermodel: 6 bits (* 255 / 32 as a byte),
    // and a polygon whose byte is 0 is not drawn (CopyVertexData)
    int const transparency = (h[6] & 0x800000) ? 32 : int((h[6] >> 18) & (sm_trans ? 0x3f : 0x1f));
    uint32_t const alpha8 = (h[6] & 0x800000) ? 255u : uint8_t((((h[6] >> 18) & 0x3f) * 255) / 32);
    bool const dropped = sm_trans && alpha8 == 0;
    bool const translucent = sm_trans ? alpha8 < 255 : !(h[6] & 0x800000);
    bool const floor = sun_clamp || translucent;   // the shader floors sunFactor for translucent fragments

    st.polys++;
    st.verts_reused += uint64_t(reused);
    st.verts_new += uint64_t(nv - reused);

    cvert cv[16];
    fp_unit &u = u_vertex;
    bool const lit = (h[6] & 0x10000) == 0;
    if (lit) st.polys_lit++;
    for (int i = 0; i < nv; i++) {
      fp_counts const before = u.n;
      float const vx = vert[i].x, vy = vert[i].y, vz = vert[i].z;
      float p[4];
      for (int j = 0; j < 4; j++)
        p[j] = u.add(u.add(u.add(u.mul(vx, vp.m[0][j]), u.mul(vy, vp.m[1][j])), u.mul(vz, vp.m[2][j])), u.flush(1.0f * vp.m[3][j]));
      cv[i].x = p[0]; cv[i].y = p[1]; cv[i].z = p[2]; cv[i].w = p[3];
      // u * scale * 256: an exponent add on an exact integer
      cv[i].p[0] = vert[i].u * tscale * 256.0f;
      cv[i].p[1] = vert[i].v * tscale * 256.0f;
      cv[i].p[3] = 0.0f;
      cv[i].sf[0] = cv[i].sf[1] = 0.0f;
      float intensity;
      if (lit && (sm_light || sm_flat || sm_n255 || sm_fixed || sm_double)) {
        light_sm(cv[i], vert[i], tr.m, fixed, floor, u);
        intensity = cv[i].p[2];
      } else if (lit) {
        auto const &T = tr.m;
        float const n0 = u.add(u.add(u.mul(vert[i].nx, T[0][0]), u.mul(vert[i].ny, T[0][1])), u.mul(vert[i].nz, T[0][2]));
        float const n1 = u.add(u.add(u.mul(vert[i].nx, T[1][0]), u.mul(vert[i].ny, T[1][1])), u.mul(vert[i].nz, T[1][2]));
        float const n2 = u.add(u.add(u.mul(vert[i].nx, T[2][0]), u.mul(vert[i].ny, T[2][1])), u.mul(vert[i].nz, T[2][2]));
        float dot = u.add(u.add(u.mul(n0, light[0]), u.mul(n1, light[1])), u.mul(n2, light[2]));
        dot = std::max(dot, 0.0f);
        intensity = u.mul(u.add(u.mul(dot, light_i), ambient), 255.0f);
        intensity = std::clamp(intensity, 0.0f, 255.0f);
      } else {
        intensity = 255.0f;
        cv[i].p[3] = 255.0f;
        cv[i].sf[0] = cv[i].sf[1] = 1.0f;
      }
      cv[i].p[2] = intensity;
      if (i < reused) { st.cached.mul += u.n.mul - before.mul; st.cached.add += u.n.add - before.add; }
    }
    if (reused > 0) {
      // the vertex counts above transform shared vertices again, as MAME does;
      // the RTL keeps the four previous results instead (counted separately)
    }

    cvert in4[4];
    std::copy(cv, cv + 4, in4);
    int const nv_in = nv;
    uint64_t const clipped0 = st.polys_clipped;
    nv = clip_all(cv, nv);
    bool const unclipped = st.polys_clipped == clipped0;

    // divide by w, viewport
    fp_unit &q = u_project;
    for (int i = 0; i < nv; i++) {
      if (unclipped && i < reused) { st.cached.mul += 5 + 3; st.cached.add += 6; st.cached.recip++; st.cached.half += 3; }
      float const oow = (mode == arith::rtl) ? q.recip(cv[i].w) : (q.n.recip++, q.flush(1.0f / cv[i].w));
      cv[i].x = q.mul(cv[i].x, oow);
      cv[i].y = q.mul(cv[i].y, oow);
      cv[i].z = q.mul(cv[i].z, oow);
      cv[i].p[0] = q.mul(cv[i].p[0], oow);
      cv[i].p[1] = q.mul(cv[i].p[1], oow);
      cv[i].x = q.add(q.mul(q.add(q.half(cv[i].x), 0.5f), vp_w), vp_x);
      cv[i].y = q.add(q.mul(q.add(q.half(cv[i].y), 0.5f), vp_h), vp_y);
      if (frustum_sm) cv[i].z = q.add(q.half(cv[i].z), 0.5f);   // GL window depth, reversed: near = 1
      else cv[i].z = q.add(q.mul(q.add(q.half(cv[i].z), 0.5f), vp_far - vp_near), vp_near);
    }
    st.verts_out += uint64_t(std::max(nv, 0));

    auto poly_rec = [&](uint32_t status) {
      out.begin('P'); out.w(pass); out.w(addr); out.w(off);
      for (int k = 0; k < 7; k++) out.w(h[k]);
      out.w(uint32_t(nv_in)); out.w(uint32_t(nv_in - reused)); out.w(uint32_t(nv)); out.w(status);
      for (int k = 0; k < 4; k++) {
        out.f(in4[k].x); out.f(in4[k].y); out.f(in4[k].z); out.f(in4[k].w);
        out.f(in4[k].p[0]); out.f(in4[k].p[1]); out.f(in4[k].p[2]);
      }
      out.end();
      poly_seq++;
      if (backface) {
        // view = v x T (rows), Supermodel's normal matrix mat3(modelview) the same way
        double nf[3], dv = 0;
        for (int k = 0; k < 3; k++) nf[k] = double(int32_t(h[1 + k]) >> 8) / 4194304.0;
        for (int j = 0; j < 3; j++) {
          double const vj = double(vert[0].x) * tr.m[0][j] + double(vert[0].y) * tr.m[1][j] + double(vert[0].z) * tr.m[2][j] + tr.m[3][j];
          double const nj = nf[0] * tr.m[0][j] + nf[1] * tr.m[1][j] + nf[2] * tr.m[2][j];
          dv += vj * nj;
        }
        backface->push_back(!(h[1] & 0x10) && dv > 0 ? 1 : 0);
      }
    };
    uint32_t const my_poly = poly_seq;

    bool colormod = false;
    if (h[6] & 0x400) {
      int const tw = (h[3] >> 3) & 7, th = h[3] & 7;
      int const tf = (h[6] >> 7) & 7;
      if (tf != 7) colormod = (h[3] >> 7) & 1;
      if (tw >= 6 || th >= 6) { st.sm_poly_texsize6++; poly_rec(1); exitrec(off, 4); return; }
    }
    poly_rec(0);
    if (dropped) { st.d19_dropped++; continue; }
    // Supermodel's back-face test (R3DShaderTriangles.h CalcBackFace, R62):
    // d = dot(view vertex 0, mat3(modelview) x face normal), facing away when
    // d > 0. r3d_face.sv's FP32 order, one multiply-add a step:
    //   V_j = ((x T0j + y T1j) + z T2j) + T3j, N_j = (n0 T0j + n1 T1j) + n2 T2j,
    //   d = (V0 N0 + V1 N1) + V2 N2,
    // T the model matrix (top x coordinate matrix, MAME's view space: the y
    // flip against GL eye space leaves the dot unchanged). The inverted copy
    // of a double-sided polygon has -d exactly.
    bool face_gt = false, face_lt = false;
    float lod_d = 0, lod_b = 0;
    if (sm_cull) {
      fp_unit &f = u_face;
      auto const &T = tr.m;
      float const vx = vert[0].x, vy = vert[0].y, vz = vert[0].z;
      float nf[3];
      for (int k = 0; k < 3; k++) nf[k] = float(int32_t(h[1 + k]) >> 8) * float(1.0 / 4194304.0);
      float V[3], N[3];
      for (int j = 0; j < 3; j++) {
        float a = f.mul(vx, T[0][j]);
        a = f.add(f.mul(vy, T[1][j]), a);
        a = f.add(f.mul(vz, T[2][j]), a);
        V[j] = f.add(f.mul(1.0f, T[3][j]), a);
        float b = f.mul(nf[0], T[0][j]);
        b = f.add(f.mul(nf[1], T[1][j]), b);
        N[j] = f.add(f.mul(nf[2], T[2][j]), b);
      }
      float d = f.mul(V[0], N[0]);
      d = f.add(f.mul(V[1], N[1]), d);
      d = f.add(f.mul(V[2], N[2]), d);
      face_gt = d > 0.0f;
      face_lt = d < 0.0f;
      if (lod_word) {
        // r3d_face.sv: Q = NP x C in the polygon's own period, B = d x Q two later
        float const np = float(h[5] >> 8) * float(1.0 / 4096.0);
        lod_d = d;
        lod_b = f.mul(d, f.mul(np, lod_c));
      }
      static bool const facedbg = std::getenv("R3D_FACEDBG") != nullptr;
      if (facedbg) {
        std::printf("MF %u %d%d d %08X v0 %08X %08X %08X n %08X %08X %08X\n", my_poly, face_gt, face_lt, fbits(d),
                    fbits(vx), fbits(vy), fbits(vz), fbits(nf[0]), fbits(nf[1]), fbits(nf[2]));
        std::printf("MV %u %08X %08X %08X N %08X %08X %08X\n", my_poly, fbits(V[0]), fbits(V[1]), fbits(V[2]), fbits(N[0]), fbits(N[1]), fbits(N[2]));
        std::printf("MT %u", my_poly);
        for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) std::printf(" %08X", fbits(T[i][j]));
        std::printf("\n");
      }
      if (!(h[1] & 0x10)) {
        // the same test in double, for the record
        double dv = 0;
        for (int j = 0; j < 3; j++) {
          double const vj = double(vx) * T[0][j] + double(vy) * T[1][j] + double(vz) * T[2][j] + T[3][j];
          double const nj = double(nf[0]) * T[0][j] + double(nf[1]) * T[1][j] + double(nf[2]) * T[2][j];
          dv += vj * nj;
        }
        st.face_single++;
        st.face_back += face_gt;
        st.face_vs_double += face_gt != (dv > 0);
        st.face_zero += !face_gt && !face_lt;
      }
    }
    // a double-sided polygon: Supermodel draws a copy with the normals
    // inverted first, then the polygon; the rasteriser culls the one facing
    // away. A fixed or unlit copy keeps the polygon's shade.
    bool const twice = sm_double && (h[1] & 0x10);
    if (twice) st.d19_copies++;
    for (int copy = twice ? 1 : 0; copy >= 0; copy--)
    for (int i = 2; i < nv; i++) {
      // texture alpha: h6 bit 0 on Step 1.0; any of bits 2:0 from Step 1.5 (Supermodel's TextureAlpha; Daytona 2's smoke)
      bool const alpha = ((h[6] & (step >= 0x15 ? 0x7 : 0x1)) != 0) || ((h[6] & 0x800000) == 0);
      uint64_t &cnt = alpha ? tri_a : tri_o;
      if (cnt >= (alpha ? 15000u : 50000u)) { exitrec(off, 5); return; }
      uint32_t const bi = uint32_t(cnt++);
      st.tris++;
      uint32_t param = 0;
      param |= (h[4] & 0x40) ? 0x1 : 0;
      param |= (h[6] & 0x400) ? 0x8 : 0;
      param |= (h[2] & 0x2) ? 0x2 : 0;
      param |= (h[2] & 0x1) ? 0x4 : 0;
      param |= (h[6] & 0x80000000) ? 0x10 : 0;
      param |= colormod ? 0x20 : 0;
      out.begin('T'); out.w(pass); out.w(alpha); out.w(bi); out.w(my_poly);
      bool const inv = copy == 1 && !fixed;
      for (int k : {0, i - 1, i}) {
        cvert const &c = cv[k];
        out.f(c.x); out.f(c.y); out.f(c.z); out.f(c.w); out.f(c.p[0]); out.f(c.p[1]); out.f(inv ? c.p[3] : c.p[2]);
      }
      out.w((h[6] & 0x400) ? 1 : 0); out.w((h[4] & 0x40) ? 1 : 0);
      out.w(((h[4] & 0x1f) << 1) | ((h[5] >> 7) & 0x1)); out.w(h[5] & 0x1f);
      out.w((h[3] >> 3) & 0x7); out.w(h[3] & 0x7); out.w((h[6] >> 7) & 0x7);
      out.w(param); out.w(uint32_t(transparency)); out.w(color);
      // bit 1: this record faces away (the polygon d > 0, its inverted copy d < 0)
      if (t_ext()) { out.w((copy == 1 ? 1u : 0u) | (((copy == 1) ? face_lt : face_gt) ? 2u : 0u)); out.f(sm_lod ? malpha : 1.0f); }
      if (emit_sf) {
        for (int k : {0, i - 1, i}) out.f(inv ? cv[k].sf[1] : cv[k].sf[0]);
        out.f(sm_d); out.f(sm_a); out.w(floor ? 1u : 0u);
      }
      if (lod_word && sm_cull) {
        // the copy's: -d and -B exactly (a sign flip)
        uint32_t const sg = copy == 1 ? 0x80000000u : 0u;
        out.w(fbits(lod_d) ^ sg); out.w(fbits(lod_b) ^ sg);
      }
      out.end();
    }
  }
}

void r3d_geo::frame(uint32_t seq) {
  out.buf.clear();
  g_stack[0] = identity();
  g_stack[0].k = COL3;  // identity's 3x3 is data, not a datapath constant
  sp = 0;
  list_depth = 0;
  call_depth = 0;
  poly_seq = 0;
  tri_o = tri_a = 0;
  ctab = 0x400;   // Supermodel keeps it across frames; a replayed dump starts at its reset value
  uint64_t const bad0 = st.bad_addr;
  uint64_t const p0 = st.polys, n0 = st.verts_new, r0 = st.verts_reused;
  out.begin('F'); out.w(seq); out.w(step); out.end();
  for (pass = 0; pass < 4; pass++) draw_viewport(0x800000);
  ev(EV_END);
  out.begin('E');
  out.w(uint32_t(st.polys - p0)); out.w(uint32_t(st.verts_new - n0)); out.w(uint32_t(st.verts_reused - r0));
  out.w(uint32_t(tri_o)); out.w(uint32_t(tri_a)); out.w(uint32_t(st.bad_addr - bad0));
  out.end();
  st.setup = u_setup.n; st.stack = u_stack.n; st.model = u_model.n;
  st.vertex = u_vertex.n; st.clip = u_clip.n; st.project = u_project.n;
}

} // namespace r3d
