// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Stage B in the system: one draw's 3D layer against the RTL-form model. The
// system bench (tb_m3_system --r3d) writes the Real3D memories a draw read
// (rNNNNNN.bin: 8C, 8E and polygon RAM as at its swap, texture RAM as at its
// frame's end, in Supermodel's dump layout) and the layer the draw wrote
// (lNNNNNN.bin: 512 x 384 words {painted, RGB}). Here the geometry model runs
// on those memories (r3d_geo: the RTL's rows, frustum, cull, set-up rows, the
// LOD word, as m3v_core's bench), its records become render_d25's frame
// (viewport node words read from the dumped culling RAM, for fog), and
// render_d25 draws it as the system's r3d_rast is built: depth on 1/w, fog
// on, the nearest mip level with each triangle's B from the geometry, back
// faces dropped (FACE_DROP), translucency 0, 8-row bands. Scroll fog is off
// (the system ties r3d_rast's sf_* off), so each viewport's scroll-fog alpha
// is cleared. Every pixel is compared: painted and, where painted, colour.
//
//   sb_ref --vrom vf3.v3v --dump rNNNNNN.bin --layer lNNNNNN.bin [--png OUT_PREFIX]
#include "r3d_geo.h"
#include "r3d_rast.h"
#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <zlib.h>

using namespace r3d;

static std::vector<uint8_t> rd(std::string const &fn) {
  std::vector<uint8_t> d;
  if (std::FILE *f = std::fopen(fn.c_str(), "rb")) {
    std::fseek(f, 0, SEEK_END); long n = std::ftell(f); std::fseek(f, 0, SEEK_SET);
    d.resize(size_t(n)); d.resize(std::fread(d.data(), 1, size_t(n), f)); std::fclose(f);
  }
  return d;
}

static void ppm(std::string const &fn, std::vector<uint32_t> const &px) {
  if (std::FILE *f = std::fopen(fn.c_str(), "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", SCR_W, SCR_H);
    for (uint32_t p : px) { uint8_t c[3] = {uint8_t(p >> 16), uint8_t(p >> 8), uint8_t(p)}; std::fwrite(c, 1, 3, f); }
    std::fclose(f);
  }
}

int main(int argc, char **argv) {
  std::string vrom_fn, dump_fn, layer_fn, png;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--vrom") vrom_fn = nx();
    else if (a == "--dump") dump_fn = nx();
    else if (a == "--layer") layer_fn = nx();
    else if (a == "--png") png = nx();
  }
  std::vector<uint8_t> vrom = rd(vrom_fn), dmp = rd(dump_fn), lay = rd(layer_fn);
  if (vrom.size() != 0x2000000 || dmp.size() != 0x1100000 || lay.size() != 512u * 384 * 4) {
    std::fprintf(stderr, "need --vrom (32 MiB), --dump (17 MiB), --layer (512 x 384 words)\n"); return 2;
  }
  r3d_mem mem;
  std::memcpy(mem.cull_lo.data(), dmp.data(), 0x400000);
  std::memcpy(mem.cull_hi.data(), dmp.data() + 0x400000, 0x100000);
  std::memcpy(mem.poly.data(), dmp.data() + 0x500000, 0x400000);
  std::fill(mem.vrom.begin(), mem.vrom.begin() + 0x800000, 0u);
  std::memcpy(mem.vrom.data() + 0x800000, vrom.data(), 0x2000000);
  r3d_geo geo(mem, arith::rtl, false);
  geo.frustum_sm = true; geo.cull = true; geo.rows(true); geo.step = 0x10; geo.lod_word = true;
  geo.frame(0);
  std::vector<record> recs;
  geo.out.records(recs);

  // the records as render_d25's frame (read_rframe's reading), the viewport
  // node words from culling RAM, B by triangle
  auto cull_word = [&](uint32_t a) -> uint32_t {
    if (a < 0x100000) return mem.cull_lo[a];
    if (a >= 0x800000 && a < 0x840000) return mem.cull_hi[a - 0x800000];
    return 0;
  };
  rframe fr;
  std::vector<uint32_t> lodb;
  std::vector<std::array<uint32_t, 7>> polyhdr;
  uint64_t dropped = 0;
  for (auto const &r : recs) {
    auto f = [&](size_t i) { float x; std::memcpy(&x, &r.w[i], 4); return x; };
    if (r.tag == 'V' && r.w.size() >= 25) {
      rvp v;
      v.pass = r.w[0]; v.addr = r.w[1];
      v.x = f(3); v.y = f(4); v.w = f(5); v.h = f(6); v.near = f(7); v.far = f(8);
      for (int k = 0; k < 16; k++) v.proj[k] = f(size_t(9 + k));
      for (int k = 0; k < 16; k++) v.coord[k] = f(size_t(25 + k));
      for (uint32_t k = 0; k < 0x30; k++) v.node[k] = cull_word(v.addr + k);
      v.node[0x20] &= ~0xffu;        // no scroll fog (sf_* tied off in the system)
      fr.vps.push_back(v);
      continue;
    }
    if (r.tag == 'M' && r.w.size() >= 20) {
      rmodel m;
      for (int k = 0; k < 16; k++) m.top[k] = f(size_t(4 + k));
      fr.models.push_back(m);
      continue;
    }
    if (r.tag == 'P' && r.w.size() >= 10) {
      std::array<uint32_t, 7> h;
      for (int k = 0; k < 7; k++) h[size_t(k)] = r.w[size_t(3 + k)];
      polyhdr.push_back(h);
      continue;
    }
    if (r.tag != 'T' || r.w.size() < 35) continue;
    if (r.w.size() >= 37 && (r.w[35] & 2)) { dropped++; continue; }   // faces away: FACE_DROP
    rtri t;
    t.pass = r.w[0]; t.alpha = r.w[1]; t.buf = r.w[2];
    uint32_t const pi = r.w[3];
    for (int k = 0; k < 3; k++) {
      size_t const b = 4 + 7 * size_t(k);
      t.x[k] = f(b); t.y[k] = f(b + 1); t.z[k] = f(b + 2); t.w[k] = f(b + 3);
      t.p0[k] = f(b + 4); t.p1[k] = f(b + 5); t.p2[k] = f(b + 6);
    }
    t.tex_en = r.w[25]; t.page = r.w[26]; t.tex_x = r.w[27]; t.tex_y = r.w[28];
    t.tex_w = r.w[29]; t.tex_h = r.w[30]; t.fmt = r.w[31]; t.param = r.w[32];
    t.trans = r.w[33]; t.color = r.w[34];
    if (pi < polyhdr.size()) for (int k = 0; k < 7; k++) t.hdr[k] = polyhdr[pi][size_t(k)];
    t.vp = int32_t(fr.vps.size()) - 1;
    t.model = int32_t(fr.models.size()) - 1;
    t.poly = uint32_t(fr.tris.size());                                 // B by triangle (lodb)
    lodb.push_back(r.w.size() >= 39 ? r.w.back() : 0xffffffffu);
    fr.tris.push_back(t);
  }

  texram tex;
  std::memcpy(tex.t.data(), dmp.data() + 0x900000, 0x800000);
  rtl_opts ro; ro.sub = 16; ro.zbits = 24; ro.recip_bits = 24; ro.ftz = true; ro.oow = true;
  rimage img; d25_stats st; d25_opts dop;
  dop.fog = true; dop.cull = 0; dop.trans = 0; dop.band = 8; dop.mip = 1; dop.lod = 1; dop.lodb = &lodb;
  render_d25(fr, tex, ro, dop, img, st);

  uint64_t bad = 0, bad_paint = 0, painted_ref = 0, painted_rtl = 0;
  int shown = 0;
  std::vector<uint32_t> got(size_t(SCR_W) * SCR_H), ref(size_t(SCR_W) * SCR_H);
  for (int y = 0; y < SCR_H; y++)
    for (int x = 0; x < SCR_W; x++) {
      uint32_t l; std::memcpy(&l, &lay[4 * (size_t(y) * 512 + size_t(x))], 4);
      uint32_t const g = ((l >> 24) & 1 ? 0xff000000u : 0u) | (l & 0xffffff);
      uint32_t const r = img.px[size_t(y) * SCR_W + size_t(x)];
      got[size_t(y) * SCR_W + size_t(x)] = g; ref[size_t(y) * SCR_W + size_t(x)] = r;
      bool const pg = g >> 24, pr = r >> 24;
      painted_ref += pr; painted_rtl += pg;
      if (pg != pr) { bad_paint++; bad++; }
      else if (pr && (g & 0xffffff) != (r & 0xffffff)) bad++;
      if ((pg != pr || (pr && (g & 0xffffff) != (r & 0xffffff))) && shown < 8) {
        shown++; std::printf("pixel (%d, %d): rtl %08X model %08X\n", x, y, g, r);
      }
    }
  std::printf("%zu viewports, %zu triangles (%" PRIu64 " facing away), %" PRIu64 " pixels shaded by the model\n",
              fr.vps.size(), fr.tris.size(), dropped, st.shaded);
  std::printf("layer: %" PRIu64 " of %d pixels differ (%" PRIu64 " in painted); painted rtl %" PRIu64 ", model %" PRIu64 "\n",
              bad, SCR_W * SCR_H, bad_paint, painted_rtl, painted_ref);
  if (!png.empty()) { ppm(png + "_rtl.ppm", got); ppm(png + "_model.ppm", ref); }
  std::printf("%s\n", bad ? "FAIL" : "PASS");
  return bad ? 1 : 0;
}
