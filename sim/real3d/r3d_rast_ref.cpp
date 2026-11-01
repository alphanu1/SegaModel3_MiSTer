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
// Rasteriser model driver.
//
//   r3d_rast_ref --tris DIR --dumps DIR --mame DIR [--arith mame|rtl] [--frames N]
//                [--png DIR] [--sm DIR] [--smfull DIR] [--threads N] [rtl options]
//
// --tris   MAME's triangle lists (*.tri), the model's input
// --dumps  Supermodel's Real3D dumps (*.bin.gz), for texture RAM
// --mame   MAME's frame images from the same dumps (mame0289-real3d-frames.patch):
//          <dump>.3d (3D layer) and <dump>.full (composite), 496 x 384 ARGB words
// --sm     Supermodel's frame grabs with no tile layer (M3NO2D), full_<frame>.bmp
// --smfull Supermodel's finished frames, full_<frame>.bmp
// --compare-only: no model; MAME against Supermodel only
//
// Prints per-frame and per-kind match rates of the model against MAME's 3D
// layer, and with --sm / --smfull the MAME-against-Supermodel difference.

#include "r3d_rast.h"
#include "r3d_geo_lod.h"
#include <memory>
#include <zlib.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

namespace {

bool load_tex(std::string const &fn, texram &t) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  bool ok = gzseek(g, 0x900000, SEEK_SET) == 0x900000 && gzread(g, t.t.data(), 0x800000) == 0x800000;
  gzclose(g);
  return ok;
}

bool load_tex_dl(std::string const &fn, texram &t, std::vector<uint32_t> &dl) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  dl.resize(0x40000);
  bool ok = gzseek(g, 0x400000, SEEK_SET) == 0x400000 && gzread(g, dl.data(), 0x100000) == 0x100000 &&
            gzseek(g, 0x900000, SEEK_SET) == 0x900000 && gzread(g, t.t.data(), 0x800000) == 0x800000;
  gzclose(g);
  return ok;
}

bool load_raw(std::string const &fn, std::vector<uint32_t> &px) {
  std::FILE *f = std::fopen(fn.c_str(), "rb");
  if (!f) return false;
  px.resize(SCR_W * SCR_H);
  bool ok = std::fread(px.data(), 4, px.size(), f) == px.size();
  std::fclose(f);
  return ok;
}

// Supermodel's BMP (WriteSurfaceToBMP): 32-bit, bottom-up unless the height is
// negative. Its file-size field is wrong and is not read.
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
  size_t const stride = ((size_t(w) * bpp / 8) + 3) & ~size_t(3);
  px.assign(SCR_W * SCR_H, 0);
  for (int y = 0; y < SCR_H; y++) {
    int const sy = h > 0 ? SCR_H - 1 - y : y;
    size_t const o = off + size_t(sy) * stride;
    if (o + stride > b.size()) return false;
    for (int x = 0; x < SCR_W; x++) {
      uint8_t const *p = &b[o + size_t(x) * (bpp / 8)];
      px[size_t(y) * SCR_W + x] = 0xff000000u | uint32_t(p[2]) << 16 | uint32_t(p[1]) << 8 | p[0];  // BGR(A)
    }
  }
  return true;
}

// Minimal PNG writer (RGB, no filter), for looking at images
void crc_png(std::vector<uint8_t> &o, char const *type, std::vector<uint8_t> const &d) {
  uint8_t len[4] = {uint8_t(d.size() >> 24), uint8_t(d.size() >> 16), uint8_t(d.size() >> 8), uint8_t(d.size())};
  o.insert(o.end(), len, len + 4);
  size_t const s = o.size();
  o.insert(o.end(), type, type + 4);
  o.insert(o.end(), d.begin(), d.end());
  uLong c = crc32(0, &o[s], uInt(o.size() - s));
  uint8_t cb[4] = {uint8_t(c >> 24), uint8_t(c >> 16), uint8_t(c >> 8), uint8_t(c)};
  o.insert(o.end(), cb, cb + 4);
}
void save_png(std::string const &fn, std::vector<uint32_t> const &px, int w, int h) {
  std::vector<uint8_t> raw;
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    for (int x = 0; x < w; x++) {
      uint32_t const p = px[size_t(y) * w + x];
      raw.push_back(uint8_t(p >> 16)); raw.push_back(uint8_t(p >> 8)); raw.push_back(uint8_t(p));
    }
  }
  uLongf zl = compressBound(raw.size());
  std::vector<uint8_t> z(zl);
  compress2(z.data(), &zl, raw.data(), raw.size(), 6);
  z.resize(zl);
  std::vector<uint8_t> o = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
  std::vector<uint8_t> ih = {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w),
                             uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h), 8, 2, 0, 0, 0};
  crc_png(o, "IHDR", ih);
  crc_png(o, "IDAT", z);
  crc_png(o, "IEND", {});
  if (std::FILE *f = std::fopen(fn.c_str(), "wb")) { std::fwrite(o.data(), 1, o.size(), f); std::fclose(f); }
}

// Image difference over RGB: pixels with any channel different, pixels more
// than 8 and 32 out on some channel, and PSNR
struct idiff {
  uint64_t n = 0, ne = 0, n8 = 0, n32 = 0;
  double se = 0;
  void add(uint32_t a, uint32_t b) {
    n++;
    int m = 0;
    for (int sh = 0; sh < 24; sh += 8) {
      int const d = int((a >> sh) & 0xff) - int((b >> sh) & 0xff);
      se += double(d) * d;
      m = std::max(m, std::abs(d));
    }
    ne += m > 0; n8 += m > 8; n32 += m > 32;
  }
  double psnr() const { return se == 0 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 * 3.0 * double(n) / se); }
  void merge(idiff const &o) { n += o.n; ne += o.ne; n8 += o.n8; n32 += o.n32; se += o.se; }
};

struct frame_res {
  std::string name;
  bool ok = false;
  rstats st;
  uint64_t drawn = 0, diff = 0, diff_cov = 0, tainted = 0, tainted_diff = 0;  // MAME's drawn pixels; model differs; coverage differs
  uint64_t kd[K_COUNT] = {}, kn[K_COUNT] = {};  // per kind of MAME... the model's last writer: pixels, different
  uint64_t fd[8] = {}, fn[8] = {};
  int maxd = 0;
  idiff d3, dfull, dmm, dmodel;                  // MAME-SM 3D, MAME-SM full, model-MAME, rtl-mame form
  uint64_t model_ne = 0;
  uint64_t cov_n = 0, cov_m = 0, cov_s = 0, cov_x = 0;
  idiff d3both, dfull2d;
  lockstep ls;
};

} // namespace

int main(int argc, char **argv) {
  std::string tris, dumps, mame, png, sm, smfull;
  rarith ar = rarith::mame;
  rtl_opts o;
  int frames = 1 << 30, threads = 8;
  bool compare_only = false, ablate = false, d25 = false;
  d25_opts dd;
  int ref_mip = -1, ref_lod = 0;   // --d25: also render this mip variant and diff against it
  std::string geo_vrom;
  int sm2d_dx = 2, sm2d_dy = 2;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--tris") tris = nx();
    else if (a == "--dumps") dumps = nx();
    else if (a == "--mame") mame = nx();
    else if (a == "--png") png = nx();
    else if (a == "--sm") sm = nx();
    else if (a == "--smfull") smfull = nx();
    else if (a == "--arith") ar = nx() == "rtl" ? rarith::rtl : rarith::mame;
    else if (a == "--frames") frames = std::atoi(nx().c_str());
    else if (a == "--threads") threads = std::atoi(nx().c_str());
    else if (a == "--compare-only") compare_only = true;
    else if (a == "--sm-ablation") ablate = true;
    else if (a == "--sm2d-shift") { std::string const v = nx(); sm2d_dx = std::atoi(v.c_str()); sm2d_dy = std::atoi(v.c_str() + v.find(',') + 1); }
    else if (a == "--sub") o.sub = std::atoi(nx().c_str());
    else if (a == "--zbits") o.zbits = std::atoi(nx().c_str());
    else if (a == "--recip") o.recip_bits = std::atoi(nx().c_str());
    else if (a == "--fixed-setup") o.fixed_setup = true;
    else if (a == "--no-ftz") o.ftz = false;
    else if (a == "--d25") d25 = true;
    else if (a == "--fog") dd.fog = std::atoi(nx().c_str()) != 0;
    else if (a == "--cull") dd.cull = std::atoi(nx().c_str());
    else if (a == "--trans") dd.trans = std::atoi(nx().c_str());
    else if (a == "--band") dd.band = std::atoi(nx().c_str());
    else if (a == "--oow") o.oow = true;
    else if (a == "--mip") dd.mip = std::atoi(nx().c_str());
    else if (a == "--lod") dd.lod = std::atoi(nx().c_str());
    else if (a == "--ref-mip") ref_mip = std::atoi(nx().c_str());
    else if (a == "--ref-lod") ref_lod = std::atoi(nx().c_str());
    else if (a == "--geo-lod") geo_vrom = nx();   // --d25, lod 1: each polygon's B from the geometry model (r3d_geo_lod.h)
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  if (mame.empty() || (!compare_only && (tris.empty() || dumps.empty()))) {
    std::fprintf(stderr, "usage: r3d_rast_ref --tris DIR --dumps DIR --mame DIR [--arith mame|rtl] [--sm DIR] [--smfull DIR]\n");
    return 2;
  }
  std::vector<std::string> names;
  for (auto const &e : fs::directory_iterator(compare_only ? mame : tris)) {
    std::string const ext = compare_only ? ".3d" : ".tri";
    if (e.path().extension() == ext) names.push_back(e.path().stem().string());
  }
  std::sort(names.begin(), names.end());
  if (int(names.size()) > frames) names.resize(size_t(frames));
  if (!png.empty()) fs::create_directories(png);

  if (d25) {
    // D-25 RTL form against Supermodel's 3D-only grabs, and the 100 MHz cycle model
    struct d25r { bool ok = false; idiff d, vs, vg, vm; uint64_t vsk[K_COUNT] = {}; uint64_t cn = 0, cx = 0; d25_stats st; };
    std::vector<d25r> rr(names.size());
    std::atomic<size_t> nx{0};
    std::vector<uint32_t> vrom;
    if (!geo_vrom.empty()) {
      vrom.resize(0x1000000);
      std::FILE *fv = std::fopen(geo_vrom.c_str(), "rb");
      if (!fv) { std::fprintf(stderr, "no VROM %s\n", geo_vrom.c_str()); return 2; }
      size_t n = std::fread(vrom.data(), 4, vrom.size(), fv); (void)n; std::fclose(fv);
    }
    auto wk = [&]() {
      texram tex;
      std::vector<uint32_t> dl;
      std::unique_ptr<r3d_mem> gm;
      if (!vrom.empty()) { gm = std::make_unique<r3d_mem>(); gm->vrom = vrom; }
      for (size_t i; (i = nx++) < names.size();) {
        rframe f;
        if (!read_rframe(tris + "/" + names[i] + ".tri", f)) continue;
        if (!load_tex_dl(dumps + "/" + names[i] + ".bin.gz", tex, dl)) continue;
        for (auto &v : f.vps)
          for (int k = 0; k < 0x30; k++) v.node[k] = dl[((v.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
        rimage img;
        d25_opts d1 = dd;
        std::vector<uint32_t> lodb;
        if (gm && geo_lod_load(dumps + "/" + names[i] + ".bin.gz", *gm)) { lodb = geo_lod_words(*gm); d1.lodb = &lodb; }
        render_d25(f, tex, o, d1, img, rr[i].st);
        if (ref_mip >= 0) {
          d25_opts d2 = d1; d2.mip = ref_mip; d2.lod = ref_lod;
          rimage im2; d25_stats s2;
          render_d25(f, tex, o, d2, im2, s2);
          for (size_t p = 0; p < img.px.size(); p++)
            rr[i].vm.add(((img.px[p] >> 24) ? img.px[p] : 0) & 0xffffff, ((im2.px[p] >> 24) ? im2.px[p] : 0) & 0xffffff);
        }
        char smn[64];
        std::snprintf(smn, sizeof(smn), "/full_%06d.bmp", std::atoi(names[i].c_str() + 1));
        std::vector<uint32_t> s3;
        if (!sm.empty() && load_bmp(sm + smn, s3))
          for (size_t p = 0; p < s3.size(); p++) {
            int const x = int(p % SCR_W), y = int(p / SCR_W), sx = x - sm2d_dx, sy = y - sm2d_dy;
            uint32_t const sv = (sx >= 0 && sy >= 0 && sx < SCR_W && sy < SCR_H) ? s3[size_t(sy) * SCR_W + size_t(sx)] : 0xff000000u;
            uint32_t const mv = (img.px[p] >> 24) ? img.px[p] : 0;
            rr[i].d.add(mv & 0xffffff, sv & 0xffffff);
            bool const md = (img.px[p] >> 24) != 0, sd = (sv & 0xffffff) != 0;
            rr[i].cn++; rr[i].cx += md != sd;
          }
        if (!png.empty() && i % 40 == 0) save_png(png + "/" + names[i] + "_d25.png", img.px, SCR_W, SCR_H);
        if (char const *fe = std::getenv("R3D_D25_SM")) {
          // the float Supermodel form with the given switches, against this frame
          rimage si; rstats ss;
          render_sm(f, tex, uint32_t(std::strtoul(fe, nullptr, 0)), si, ss);
          if (!s3.empty())
            for (size_t p = 0; p < s3.size(); p++) {
              int const x = int(p % SCR_W), y = int(p / SCR_W), sx = x - sm2d_dx, sy = y - sm2d_dy;
              uint32_t const sv = (sx >= 0 && sy >= 0 && sx < SCR_W && sy < SCR_H) ? s3[size_t(sy) * SCR_W + size_t(sx)] : 0xff000000u;
              rr[i].vg.add(((si.px[p] >> 24) ? si.px[p] : 0) & 0xffffff, sv & 0xffffff);
            }
          for (size_t p = 0; p < si.px.size(); p++) {
            uint32_t const a = (img.px[p] >> 24) ? img.px[p] : 0, b = (si.px[p] >> 24) ? si.px[p] : 0;
            rr[i].vs.add(a & 0xffffff, b & 0xffffff);
            int dd2 = 0;
            for (int sh = 0; sh < 24; sh += 8) dd2 = std::max(dd2, std::abs(int((a >> sh) & 255) - int((b >> sh) & 255)));
            if (dd2 > 8) rr[i].vsk[std::min<int>(img.kind[p], K_COUNT - 1)]++;
            static std::atomic<int> shown{0};
            if (dd2 > 8 && std::getenv("R3D_D25_DBG") && img.kind[p] == K_TEX_ALPHA && shown++ < 12) {
              rtri const &t = f.tris[size_t(img.tri[p])];
              std::printf("dbg %s (%zu,%zu) d25 %06x sm %06x tri %d trans %u fogint %u h6 %08x vp %d\n", names[i].c_str(), p % SCR_W, p / SCR_W,
                          a & 0xffffff, b & 0xffffff, img.tri[p], t.trans, (t.hdr[6] >> 11) & 31, t.hdr[6], t.vp);
            }
          }
        }
        rr[i].ok = true;
      }
    };
    std::vector<std::thread> tt;
    for (int t = 0; t < threads; t++) tt.emplace_back(wk);
    for (auto &t : tt) t.join();
    auto pct = [](std::vector<uint64_t> v, double q) -> uint64_t {
      if (v.empty()) return 0;
      std::sort(v.begin(), v.end());
      return v[std::min(v.size() - 1, size_t(q * double(v.size())))];
    };
    idiff t; uint64_t cn = 0, cx = 0; std::vector<double> ps;
    d25_stats tot;
    std::vector<uint64_t> vf, vs, vw, vz, vsh, vsu, vby, vpr;
    size_t nf = 0;
    for (auto const &r : rr) {
      if (!r.ok) continue;
      nf++;
      t.merge(r.d); cn += r.cn; cx += r.cx;
      if (r.d.n) ps.push_back(r.d.psnr());
      auto const &s = r.st;
      tot.tris += s.tris; tot.culled += s.culled; tot.pairs += s.pairs; tot.ztest += s.ztest; tot.zpass += s.zpass; tot.shaded += s.shaded; tot.fog_px += s.fog_px;
      for (int a = 0; a < 2; a++) for (int b = 0; b < 2; b++) tot.cull_tab[a][b] += s.cull_tab[a][b];
      vf.push_back(s.frame_fifo); vs.push_back(s.frame_serial); vw.push_back(s.walker); vz.push_back(s.zwalk); vsh.push_back(s.shade);
      vsu.push_back(s.setup); vby.push_back(s.store_bytes); vpr.push_back(s.pairs);
    }
    std::sort(ps.begin(), ps.end());
    if (std::getenv("R3D_D25_SM")) {
      idiff v; uint64_t k[K_COUNT] = {};
      for (auto const &r : rr) if (r.ok) { v.merge(r.vs); for (int j = 0; j < K_COUNT; j++) k[j] += r.vsk[j]; }
      std::vector<double> pg, pd;
      for (auto const &r : rr) if (r.ok && r.vg.n) { pg.push_back(r.vg.psnr()); pd.push_back(r.d.psnr()); }
      std::sort(pg.begin(), pg.end());
      if (!pg.empty()) std::printf("  the float form against the grabs: p50 frame %.2f dB\n", pg[pg.size() / 2]);
      if (char const *fn = std::getenv("R3D_D25_PF")) {
        std::FILE *o2 = std::fopen(fn, "w");
        size_t j = 0;
        for (size_t q = 0; q < rr.size(); q++) if (rr[q].ok && rr[q].vg.n) std::fprintf(o2, "%s %.2f %.2f\n", names[q].c_str(), rr[q].d.psnr(), rr[q].vg.psnr()), j++;
        std::fclose(o2);
      }
      std::printf("  against the float Supermodel form (switches %s): PSNR %.2f dB, differ >8 %.3f%%; by kind >8:", std::getenv("R3D_D25_SM"), v.psnr(), v.n ? 100.0 * double(v.n8) / double(v.n) : 0.0);
      for (int j = 0; j < K_COUNT; j++) std::printf(" %s %" PRIu64, kind_name(j), k[j]);
      std::printf("\n");
    }
    std::printf("D-25 RTL form: fog %d cull %d trans %d oow %d band %d mip %d lod %d; %zu frames\n", dd.fog, dd.cull, dd.trans, o.oow, dd.band, dd.mip, dd.lod, nf);
    if (ref_mip >= 0) {
      idiff v; std::vector<double> pv;
      for (auto const &r : rr) if (r.ok) { v.merge(r.vm); pv.push_back(r.vm.psnr()); }
      std::sort(pv.begin(), pv.end());
      std::printf("  against mip %d lod %d: pixels different %.3f%%, >8 %.3f%%, >32 %.3f%%; PSNR all %.2f dB, p50 frame %.2f dB, p1 frame %.2f dB\n",
                  ref_mip, ref_lod, v.n ? 100.0 * double(v.ne) / double(v.n) : 0.0, v.n ? 100.0 * double(v.n8) / double(v.n) : 0.0,
                  v.n ? 100.0 * double(v.n32) / double(v.n) : 0.0, v.psnr(), pv.empty() ? 0.0 : pv[pv.size() / 2], pv.empty() ? 0.0 : pv[pv.size() / 100]);
    }
    if (cn) std::printf("  against Supermodel's 3D-only grabs: PSNR all %.2f dB, p50 frame %.2f dB, differ >8 %.3f%%, coverage disagrees %.3f%%\n",
                        t.psnr(), ps.empty() ? 0.0 : ps[ps.size() / 2], t.n ? 100.0 * double(t.n8) / double(t.n) : 0.0, 100.0 * double(cx) / double(cn));
    std::printf("  triangles %" PRIu64 ", culled %" PRIu64 "; pairs %" PRIu64 "; Z tests %" PRIu64 ", passed %" PRIu64 ", shaded %" PRIu64 ", fogged %" PRIu64 "\n",
                tot.tris, tot.culled, tot.pairs, tot.ztest, tot.zpass, tot.shaded, tot.fog_px);
    std::printf("  cull (det verdict back / front) against Supermodel's (back / front): back-back %" PRIu64 ", back-front %" PRIu64 ", front-back %" PRIu64 ", front-front %" PRIu64 "\n",
                tot.cull_tab[1][1], tot.cull_tab[1][0], tot.cull_tab[0][1], tot.cull_tab[0][0]);
    double const FR = 1738530.0;   // 100 MHz cycles a 57.52 Hz frame
    auto pr = [&](char const *nm, std::vector<uint64_t> const &v, bool share) {
      std::printf("  %-44s p50 %9" PRIu64 "  p99 %9" PRIu64 "  max %9" PRIu64, nm, pct(v, 0.5), pct(v, 0.99), pct(v, 1.0));
      if (share) std::printf("   (p99 %.1f%%, max %.1f%% of a 100 MHz frame)", 100.0 * double(pct(v, 0.99)) / FR, 100.0 * double(pct(v, 1.0)) / FR);
      std::printf("\n");
    };
    pr("(triangle, band) pairs", vpr, false);
    pr("walker cycles (rows, start product)", vw, true);
    pr("Z walk cycles, 2 a clock", vz, true);
    pr("shading cycles, 1 a clock", vsh, true);
    pr("frame, slowest stage per band (FIFOs)", vf, true);
    pr("frame, slowest stage per pair (no overlap)", vs, true);
    pr("set-up cycles (one multiplier)", vsu, true);
    pr("set-up records into the store, bytes", vby, false);
    return 0;
  }

  if (ablate) {
    // Supermodel form against Supermodel's 3D-only grabs: MAME form, all
    // behaviours, all but one, and each alone
    struct variant { std::string name; int kind; uint32_t feat; };   // kind 0 MAME form, 1 Supermodel form
    std::vector<variant> vs = {{"MAME form", 0, 0}, {"Supermodel form, all", 1, SM_ALL}, {"Supermodel form, none", 1, 0}};
    char const *fn_[] = {"COLOUR", "DECODE", "SAMPLE", "ALPHA", "TRANS", "DEPTH", "LAYER", "CULL", "FOG", "MIP", "PRIO"};
    for (int b = 0; b < 11; b++) vs.push_back({std::string("all but ") + fn_[b], 1, SM_ALL & ~(1u << b)});
    for (int b = 0; b < 11; b++) vs.push_back({std::string("only ") + fn_[b], 1, 1u << b});
    int const nv = int(vs.size());
    std::vector<std::vector<idiff>> dv(names.size(), std::vector<idiff>(size_t(nv)));
    std::vector<std::vector<std::array<uint64_t, 2>>> cv(names.size(), std::vector<std::array<uint64_t, 2>>(size_t(nv)));
    std::atomic<size_t> nx{0};
    auto wk = [&]() {
      texram tex;
      std::vector<uint32_t> dl;
      for (size_t i; (i = nx++) < names.size();) {
        rframe f;
        if (!read_rframe(tris + "/" + names[i] + ".tri", f)) continue;
        if (!load_tex_dl(dumps + "/" + names[i] + ".bin.gz", tex, dl)) continue;
        for (auto &v : f.vps)
          for (int k = 0; k < 0x30; k++) v.node[k] = dl[((v.addr & 0x7fffff) + uint32_t(k)) & 0x3ffff];
        char smn[64];
        std::snprintf(smn, sizeof(smn), "/full_%06d.bmp", std::atoi(names[i].c_str() + 1));
        std::vector<uint32_t> s3;
        if (!load_bmp(sm + smn, s3)) continue;
        for (int v = 0; v < nv; v++) {
          rimage img;
          rstats st;
          if (vs[size_t(v)].kind == 0) {
            render(f, tex, rarith::rtl, o, img, st);
            for (auto &p : img.px) if (!(p >> 24)) p = 0;
          } else render_sm(f, tex, vs[size_t(v)].feat, img, st);
          for (size_t p = 0; p < s3.size(); p++) {
            int const x = int(p % SCR_W), y = int(p / SCR_W), sx = x - sm2d_dx, sy = y - sm2d_dy;
            uint32_t const sv = (sx >= 0 && sy >= 0 && sx < SCR_W && sy < SCR_H) ? s3[size_t(sy) * SCR_W + size_t(sx)] : 0xff000000u;
            dv[i][size_t(v)].add(img.px[p] & 0xffffff, sv & 0xffffff);
            bool const md = (img.px[p] >> 24) != 0, sd = (sv & 0xffffff) != 0;
            cv[i][size_t(v)][0]++;
            cv[i][size_t(v)][1] += md != sd;
          }
          if (!png.empty() && i % 40 == 0 && (v == 0 || v == 1)) {
            std::vector<uint32_t> o2(img.px);
            save_png(png + "/" + names[i] + (v == 0 ? "_mameform.png" : "_smform.png"), o2, SCR_W, SCR_H);
          }
        }
      }
    };
    std::vector<std::thread> tt;
    for (int t = 0; t < threads; t++) tt.emplace_back(wk);
    for (auto &t : tt) t.join();
    std::printf("against Supermodel's 3D-only grabs, Supermodel moved %d,%d px; %zu frames\n", sm2d_dx, sm2d_dy, names.size());
    std::printf("%-28s %10s %10s %10s %12s %12s\n", "variant", "PSNR dB", "p50 frame", "p1 frame", "differ >8 %", "coverage %");
    for (int v = 0; v < nv; v++) {
      idiff t; uint64_t cn = 0, cx = 0; std::vector<double> ps;
      for (size_t i = 0; i < names.size(); i++) { t.merge(dv[i][size_t(v)]); cn += cv[i][size_t(v)][0]; cx += cv[i][size_t(v)][1]; if (dv[i][size_t(v)].n) ps.push_back(dv[i][size_t(v)].psnr()); }
      std::sort(ps.begin(), ps.end());
      double const p50 = ps.empty() ? 0 : ps[ps.size() / 2], p1 = ps.empty() ? 0 : ps[ps.size() / 100];
      std::printf("%-28s %10.2f %10.2f %10.2f %12.3f %12.3f\n", vs[size_t(v)].name.c_str(), t.psnr(), p50, p1,
                  t.n ? 100.0 * double(t.n8) / double(t.n) : 0.0, cn ? 100.0 * double(cx) / double(cn) : 0.0);
    }
    return 0;
  }

  std::vector<frame_res> res(names.size());
  std::atomic<size_t> next{0};
  auto work = [&]() {
    texram tex;
    for (size_t i; (i = next++) < names.size();) {
      frame_res &r = res[i];
      r.name = names[i];
      std::vector<uint32_t> m3d, mfull;
      if (!load_raw(mame + "/" + r.name + ".3d", m3d)) continue;
      bool const have_full = load_raw(mame + "/" + r.name + ".full", mfull);
      int const fno = std::atoi(r.name.c_str() + 1);
      char smn[64];
      std::snprintf(smn, sizeof(smn), "/full_%06d.bmp", fno);
      std::vector<uint32_t> s3, sf;
      if (!sm.empty() && load_bmp(sm + smn, s3))
        for (size_t p = 0; p < m3d.size(); p++) {
          r.d3.add(m3d[p] & 0xffffff, s3[p]);
          bool const md = (m3d[p] >> 24) != 0, sd = (s3[p] & 0xffffff) != 0;
          r.cov_n++; r.cov_m += md; r.cov_s += sd; r.cov_x += md != sd;
          if (md && sd) r.d3both.add(m3d[p] & 0xffffff, s3[p]);
        }
      if (!smfull.empty() && have_full && load_bmp(smfull + smn, sf))
        for (size_t p = 0; p < m3d.size(); p++) {
          r.dfull.add(mfull[p], sf[p]);
          if (!(m3d[p] >> 24) && !s3.empty() && (s3[p] & 0xffffff) == 0) {
            // Supermodel draws the tile layers sm2d_dx, sm2d_dy px up and left of MAME's
            int const x = int(p % SCR_W), y = int(p / SCR_W), sx = x - sm2d_dx, sy = y - sm2d_dy;
            if (sx >= 0 && sy >= 0 && sx < SCR_W && sy < SCR_H) r.dfull2d.add(mfull[p], sf[size_t(sy) * SCR_W + size_t(sx)]);
          }
        }
      if (!png.empty() && i % 40 == 0) {
        save_png(png + "/" + r.name + "_mame3d.png", m3d, SCR_W, SCR_H);
        if (have_full) save_png(png + "/" + r.name + "_mamefull.png", mfull, SCR_W, SCR_H);
        if (!s3.empty()) save_png(png + "/" + r.name + "_sm3d.png", s3, SCR_W, SCR_H);
        if (!sf.empty()) save_png(png + "/" + r.name + "_smfull.png", sf, SCR_W, SCR_H);
      }
      if (compare_only) { r.ok = true; continue; }
      rframe f;
      if (!read_rframe(tris + "/" + r.name + ".tri", f)) continue;
      if (!load_tex(dumps + "/" + r.name + ".bin.gz", tex)) continue;
      rimage img;
      render(f, tex, ar, o, img, r.st);
      if (ar == rarith::rtl) {
        rimage mi;
        rstats ms;
        render(f, tex, rarith::mame, o, mi, ms);
        for (size_t p = 0; p < img.taint.size(); p++) img.taint[p] = mi.taint[p];   // MAME's undefined reads
        for (size_t p = 0; p < img.px.size(); p++) r.dmodel.add(img.px[p] & 0xffffff, mi.px[p] & 0xffffff), r.model_ne += img.px[p] != mi.px[p];
        render_lockstep(f, tex, o, r.ls);
      }
      for (size_t p = 0; p < m3d.size(); p++) {
        uint32_t const a = img.px[p], b = m3d[p];
        bool const da = (a >> 24) != 0, db = (b >> 24) != 0;
        r.drawn += db;
        if (!da && !db) continue;
        if (img.taint[p] & 1) { r.tainted++; r.tainted_diff += a != b; continue; }
        int const k = img.kind[p];
        r.kn[k]++;
        if (img.fmt[p] < 8) r.fn[img.fmt[p]]++;
        r.dmm.add(a & 0xffffff, b & 0xffffff);
        if (a != b) {
          r.diff++;
          r.diff_cov += da != db;
          r.kd[k]++;
          if (img.fmt[p] < 8) r.fd[img.fmt[p]]++;
          for (int sh = 0; sh < 24; sh += 8)
            r.maxd = std::max(r.maxd, std::abs(int((a >> sh) & 0xff) - int((b >> sh) & 0xff)));
        }
      }
      if (char const *dbg = std::getenv("R3D_DEBUG"); dbg && r.name == dbg) {
        std::map<int32_t, int> h;
        for (size_t p = 0; p < m3d.size(); p++)
          if (img.px[p] != m3d[p]) h[img.tri[p]]++;
        for (auto const &[ti, cnt] : h) {
          if (ti < 0) { std::printf("debug: %d px with no model write\n", cnt); continue; }
          rtri const &t = f.tris[size_t(ti)];
          std::printf("debug: tri %d px %d kind %s pass %u alpha %u tex %u page %u xy %u,%u wh %u,%u fmt %u param %02x trans %u col %06x\n",
                      ti, cnt, kind_name(img.kind[0] * 0 + 0), t.pass, t.alpha, t.tex_en, t.page, t.tex_x, t.tex_y, t.tex_w, t.tex_h, t.fmt, t.param, t.trans, t.color);
        }
        int shown = 0;
        for (size_t p = 0; p < m3d.size() && shown < 12; p++)
          if (img.px[p] != m3d[p]) { std::printf("debug: (%zu,%zu) model %08x mame %08x tri %d\n", p % SCR_W, p / SCR_W, img.px[p], m3d[p], img.tri[p]); shown++; }
      }
      if (!png.empty() && i % 40 == 0) save_png(png + "/" + r.name + "_model.png", img.px, SCR_W, SCR_H);
      r.ok = true;
    }
  };
  std::vector<std::thread> th;
  for (int t = 0; t < threads; t++) th.emplace_back(work);
  for (auto &t : th) t.join();

  // ---- report ----
  uint64_t frames_ok = 0, exact = 0, drawn = 0, diff = 0, cov = 0, taint = 0, taint_d = 0, taint_fr = 0, kn[K_COUNT] = {}, kd[K_COUNT] = {}, fnn[8] = {}, fdd[8] = {};
  rstats tot;
  idiff d3, dfull, dmm;
  std::vector<double> ps3, psf;
  std::vector<uint64_t> pix, tb8, tb16, tb32, cy2, cy4, ppo, ppp, win, spn, trs, zps, dec, pol, pvx, pb8, pb16, pb32, sh1, pp41, pp42;
  int maxd = 0;
  for (auto const &r : res) {
    if (!r.ok) continue;
    frames_ok++;
    if (r.d3.n) { d3.merge(r.d3); ps3.push_back(r.d3.psnr()); }
    if (r.dfull.n) { dfull.merge(r.dfull); psf.push_back(r.dfull.psnr()); }
    if (compare_only) continue;
    exact += r.diff == 0;
    drawn += r.drawn; diff += r.diff; cov += r.diff_cov; taint += r.tainted; taint_d += r.tainted_diff; taint_fr += r.tainted > 0;
    dmm.merge(r.dmm);
    maxd = std::max(maxd, r.maxd);
    for (int k = 0; k < K_COUNT; k++) { kn[k] += r.kn[k]; kd[k] += r.kd[k]; }
    for (int k = 0; k < 8; k++) { fnn[k] += r.fn[k]; fdd[k] += r.fd[k]; }
    tot.tris += r.st.tris; tot.spans += r.st.spans; tot.pixels += r.st.pixels; tot.zpass += r.st.zpass;
    tot.written += r.st.written; tot.texels += r.st.texels; tot.tex_outside += r.st.tex_outside;
    tot.nonfinite_tris += r.st.nonfinite_tris; tot.nonfinite_px += r.st.nonfinite_px;
    for (int k = 0; k < K_COUNT; k++) { tot.px_kind[k] += r.st.px_kind[k]; tot.kind_tris[k] += r.st.kind_tris[k]; }
    pix.push_back(r.st.pixels);
    {
      auto const &c = r.st.cyc;
      cy2.push_back(c[0][1] + c[1][1] + c[2][1]);
      cy4.push_back(c[0][2] + c[1][2] + c[2][2]);
      uint64_t const a = c[0][2] + c[1][1] + c[2][1];                 // pass A: opaque Z-only at 4, the rest at 2
      uint64_t const w2 = (r.st.winners + 1) / 2;
      ppo.push_back(a + std::max(c[0][2], w2));                       // pass B overlapped with its shading
      ppp.push_back(a + c[0][2] + w2);                                // not overlapped
      win.push_back(r.st.winners);
      spn.push_back(r.st.spans);
      trs.push_back(r.st.tris);
      zps.push_back(r.st.zpass);
      {
        auto const &c = r.st.cyc;
        uint64_t const shade_pp = r.st.winners + r.st.zpass_g[1] + r.st.zpass_g[2];   // pre-pass: winners + contour + translucent
        sh1.push_back(shade_pp);
        uint64_t const walk = 2 * c[0][2] + c[1][2] + c[2][2];
        pp41.push_back(std::max(walk, shade_pp));             // Z walk 4 px/clk (opaque twice), shading 1 px/clk
        pp42.push_back(std::max(walk, (shade_pp + 1) / 2));   // shading 2 px/clk
      }
      pol.push_back(r.st.polys); pvx.push_back(r.st.poly_verts); pb8.push_back(r.st.poly_bands[0]); pb16.push_back(r.st.poly_bands[1]); pb32.push_back(r.st.poly_bands[2]);
      dec.push_back(std::max(c[0][2] + c[1][2] + c[2][2], (r.st.zpass + 1) / 2));   // Z walk 4 px/clk, shading 2 px/clk
    }
    tb8.push_back(r.st.tri_bands[0]); tb16.push_back(r.st.tri_bands[1]); tb32.push_back(r.st.tri_bands[2]);
  }
  auto pct = [](std::vector<uint64_t> v, double q) -> uint64_t {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(q * double(v.size())))];
  };
  auto pctd = [](std::vector<double> v, double q) -> double {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, size_t(q * double(v.size())))];
  };
  std::printf("frames %" PRIu64 "\n", frames_ok);
  if (!compare_only) {
    std::printf("model (%s) against MAME's 3D layer:\n", ar == rarith::mame ? "mame" : "rtl");
    std::printf("  frames bit-exact %" PRIu64 " / %" PRIu64 "\n", exact, frames_ok);
    std::printf("  pixels drawn by MAME %" PRIu64 ", different %" PRIu64 " (%.6f%%), of which coverage %" PRIu64 "; max channel diff %d; PSNR %.2f dB\n",
                drawn, diff, drawn ? 100.0 * double(diff) / double(drawn) : 0.0, cov, maxd, dmm.psnr());
    std::printf("  excluded: %" PRIu64 " pixels in %" PRIu64 " frames whose MAME value depends on a texel read past a sheet's end (%" PRIu64 " of them differ)\n", taint, taint_fr, taint_d);
    std::printf("  per kind (model's last write): pixels / different\n");
    for (int k = 0; k < K_COUNT; k++)
      if (kn[k]) std::printf("    %-13s %12" PRIu64 " %10" PRIu64 " (%.6f%%)\n", kind_name(k), kn[k], kd[k], 100.0 * double(kd[k]) / double(kn[k]));
    std::printf("  per texel format: pixels / different\n");
    for (int k = 0; k < 8; k++)
      if (fnn[k]) std::printf("    format %d %12" PRIu64 " %10" PRIu64 " (%.6f%%)\n", k, fnn[k], fdd[k], 100.0 * double(fdd[k]) / double(fnn[k]));
    std::printf("  load: triangles %" PRIu64 ", spans %" PRIu64 ", pixels Z-tested %" PRIu64 ", passed %" PRIu64 ", written %" PRIu64 ", texels %" PRIu64 ", texels past a sheet %" PRIu64 "\n",
                tot.tris, tot.spans, tot.pixels, tot.zpass, tot.written, tot.texels, tot.tex_outside);
    std::printf("  triangles with a non-finite x, y or w: %" PRIu64 ", pixels written from them %" PRIu64 "\n", tot.nonfinite_tris, tot.nonfinite_px);
    std::printf("  triangles by kind:");
    for (int k = 1; k < K_COUNT; k++) std::printf(" %s %" PRIu64 ";", kind_name(k), tot.kind_tris[k]);
    std::printf("\n  pixels Z-tested by kind:");
    for (int k = 1; k < K_COUNT; k++) std::printf(" %s %" PRIu64 ";", kind_name(k), tot.px_kind[k]);
    std::printf("\n  per frame p50 / p99 / max: pixels Z-tested %" PRIu64 " / %" PRIu64 " / %" PRIu64 "\n",
                pct(pix, 0.5), pct(pix, 0.99), pct(pix, 1.0));
    auto pr = [&](char const *nm, std::vector<uint64_t> const &v, bool frame_share) {
      std::printf("  %-52s p50 %9" PRIu64 "  p90 %9" PRIu64 "  p99 %9" PRIu64 "  max %9" PRIu64, nm, pct(v, 0.5), pct(v, 0.9), pct(v, 0.99), pct(v, 1.0));
      if (frame_share) std::printf("   (p99 %.0f%%, max %.0f%% of a 50 MHz frame)", 100.0 * double(pct(v, 0.99)) / 869265.0, 100.0 * double(pct(v, 1.0)) / 869265.0);
      std::printf("\n");
    };
    std::printf("  per frame:\n");
    pr("triangles", trs, false);
    pr("polygons (triangle fans)", pol, false);
    pr("polygon vertices", pvx, false);
    pr("(polygon, band) pairs, 8 rows", pb8, false);
    pr("(polygon, band) pairs, 16 rows", pb16, false);
    pr("(polygon, band) pairs, 32 rows", pb32, false);
    pr("spans", spn, false);
    pr("pixels Z-tested", pix, false);
    pr("pixels passing Z", zps, false);
    pr("opaque pass winners (pixels a Z pre-pass shades)", win, false);
    pr("cycles, one pass, 2 px/clk", cy2, true);
    pr("cycles, one pass, 4 px/clk", cy4, true);
    pr("cycles, Z walk 4 px/clk, shading 2 px/clk (FIFO between)", dec, true);
    pr("pixels shaded with a Z pre-pass", sh1, false);
    pr("cycles, pre-pass, walk 4 px/clk, shading 1 px/clk", pp41, true);
    pr("cycles, pre-pass, walk 4 px/clk, shading 2 px/clk", pp42, true);
    pr("cycles, Z pre-pass (opaque 4 px/clk twice), overlapped", ppo, true);
    pr("cycles, Z pre-pass, shading not overlapped", ppp, true);
    std::printf("  (triangle, band) pairs per frame p50 / p99 / max: 8 rows %" PRIu64 " / %" PRIu64 " / %" PRIu64
                ", 16 rows %" PRIu64 " / %" PRIu64 " / %" PRIu64 ", 32 rows %" PRIu64 " / %" PRIu64 " / %" PRIu64 "\n",
                pct(tb8, 0.5), pct(tb8, 0.99), pct(tb8, 1.0), pct(tb16, 0.5), pct(tb16, 0.99), pct(tb16, 1.0),
                pct(tb32, 0.5), pct(tb32, 0.99), pct(tb32, 1.0));
  }
  if (!compare_only && ar == rarith::rtl) {
    idiff dm; uint64_t mne = 0; lockstep ls;
    for (auto const &r : res) if (r.ok) { dm.merge(r.dmodel); mne += r.model_ne; ls.merge(r.ls); }
    std::printf("RTL form against the MAME form over the whole frame (both read past a sheet the same way): pixels different %" PRIu64 " (%.4f%%), PSNR %.2f dB\n",
                mne, 100.0 * double(mne) / double(dm.n), dm.psnr());
    auto pc = [](uint64_t a, uint64_t b) { return b ? 100.0 * double(a) / double(b) : 0.0; };
    std::printf("lockstep, each triangle against MAME's own frame state:\n");
    std::printf("  spans %" PRIu64 ", extent differs %" PRIu64 " (%.5f%%); pixels both %" PRIu64 ", MAME only %" PRIu64 ", RTL only %" PRIu64 "\n",
                ls.spans, ls.span_diff, pc(ls.span_diff, ls.spans), ls.px_both, ls.px_only_mame, ls.px_only_rtl);
    std::printf("  Z tests %" PRIu64 ", outcome differs %" PRIu64 " (%.5f%%)\n", ls.ztests, ls.zdiff, pc(ls.zdiff, ls.ztests));
    std::printf("  texel fetches %" PRIu64 ": address differs %" PRIu64 " (%.4f%%); same address, fraction off by 1: %.4f%%, by 2-4: %.4f%%, by more: %.4f%%; filtered texel differs %.4f%%\n",
                ls.texfetch, ls.tex_addr_diff, pc(ls.tex_addr_diff, ls.texfetch), pc(ls.frac1, ls.texfetch), pc(ls.frac4, ls.texfetch), pc(ls.fracbig, ls.texfetch), pc(ls.texel_diff, ls.texfetch));
    std::printf("  shade %" PRIu64 ", integer differs %" PRIu64 " (%.4f%%); colour written %" PRIu64 ", differs %" PRIu64 " (%.4f%%), max channel diff %d\n",
                ls.shade_n, ls.shade_diff, pc(ls.shade_diff, ls.shade_n), ls.colour_n, ls.colour_diff, pc(ls.colour_diff, ls.colour_n), ls.colour_maxd);
    auto pr5 = [&](char const *nm, uint64_t const *h) {
      uint64_t t = 0; for (int k = 0; k < 5; k++) t += h[k];
      std::printf("  %s u, v from the exact plane quotient (1/256 texel): 0 %.3f%%, 1 %.3f%%, 2-4 %.3f%%, 5-16 %.3f%%, more %.3f%%\n", nm,
                  pc(h[0], t), pc(h[1], t), pc(h[2], t), pc(h[3], t), pc(h[4], t));
    };
    pr5("MAME", ls.uv_mame);
    pr5("RTL ", ls.uv_rtl);
    std::printf("  textured shade integer differs from the exact plane's: MAME %.4f%%, RTL %.4f%%\n", pc(ls.sh_mame_bad, ls.texfetch), pc(ls.sh_rtl_bad, ls.texfetch));
    std::printf("  per kind, pixels with any sub-result different:\n");
    for (int k = 1; k < K_COUNT; k++)
      if (ls.k_px[k]) std::printf("    %-13s %12" PRIu64 " %10" PRIu64 " (%.4f%%)\n", kind_name(k), ls.k_px[k], ls.k_diff[k], pc(ls.k_diff[k], ls.k_px[k]));
  }
  {
    uint64_t n = 0, m = 0, sx = 0, x = 0; idiff db, d2; std::vector<double> psb;
    for (auto const &r : res) if (r.ok) { n += r.cov_n; m += r.cov_m; sx += r.cov_s; x += r.cov_x; db.merge(r.d3both); d2.merge(r.dfull2d); if (r.d3both.n) psb.push_back(r.d3both.psnr()); }
    if (n) {
      std::printf("3D coverage, MAME (drawn) against Supermodel (not black): MAME %.2f%%, Supermodel %.2f%% of pixels, disagree on %.3f%%\n",
                  100.0 * double(m) / double(n), 100.0 * double(sx) / double(n), 100.0 * double(x) / double(n));
      std::printf("3D colour where both draw: pixels different %.3f%%, >8 %.3f%%, >32 %.3f%%; PSNR %.2f dB, per frame p1 / p50 / p99 %.2f / %.2f / %.2f\n",
                  100.0 * double(db.ne) / double(db.n), 100.0 * double(db.n8) / double(db.n), 100.0 * double(db.n32) / double(db.n), db.psnr(),
                  pctd(psb, 0.01), pctd(psb, 0.5), pctd(psb, 0.99));
    }
    if (d2.n)
      std::printf("composite where neither draws 3D, Supermodel moved %d,%d px: pixels different %.3f%%, >8 %.3f%%, >32 %.3f%%; PSNR %.2f dB\n",
                  sm2d_dx, sm2d_dy, 100.0 * double(d2.ne) / double(d2.n), 100.0 * double(d2.n8) / double(d2.n), 100.0 * double(d2.n32) / double(d2.n), d2.psnr());
  }
  if (d3.n)
    std::printf("MAME 3D layer against Supermodel (no tile layers): pixels different %.3f%%, >8 %.3f%%, >32 %.3f%%; PSNR all %.2f dB, per frame p1 / p50 / p99 %.2f / %.2f / %.2f\n",
                100.0 * double(d3.ne) / double(d3.n), 100.0 * double(d3.n8) / double(d3.n), 100.0 * double(d3.n32) / double(d3.n),
                d3.psnr(), pctd(ps3, 0.01), pctd(ps3, 0.5), pctd(ps3, 0.99));
  if (dfull.n)
    std::printf("MAME composite against Supermodel's frame: pixels different %.3f%%, >8 %.3f%%, >32 %.3f%%; PSNR all %.2f dB, per frame p1 / p50 / p99 %.2f / %.2f / %.2f\n",
                100.0 * double(dfull.ne) / double(dfull.n), 100.0 * double(dfull.n8) / double(dfull.n), 100.0 * double(dfull.n32) / double(dfull.n),
                dfull.psnr(), pctd(psf, 0.01), pctd(psf, 0.5), pctd(psf, 0.99));
  if (std::getenv("R3D_PERFRAME")) {
    std::printf("frame,drawn,diff,cov,pixels,tris,psnr_model,d3_ne,d3_psnr,full_ne,full_psnr\n");
    for (auto const &r : res)
      if (r.ok)
        std::printf("%s,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%.2f,%" PRIu64 ",%.2f,%" PRIu64 ",%.2f\n", r.name.c_str(), r.drawn, r.diff,
                    r.diff_cov, r.st.pixels, r.st.tris, r.dmm.psnr(), r.d3.ne, r.d3.psnr(), r.dfull.ne, r.dfull.psnr());
  }
  return 0;
}
