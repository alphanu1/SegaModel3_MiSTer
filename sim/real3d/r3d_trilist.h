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
// Triangle-list file format, shared by the MAME patch
// (tools/mame-census/mame0289-real3d-tris.patch) and this model.
//
// A file is a sequence of records { tag, nwords, word[nwords] }, 32-bit
// little-endian words; floats are stored as IEEE bit patterns.
//
//   'F' seq, step                                             frame start
//   'V' pass, addr, node0, vx, vy, vw, vh, near, far, proj[16], coord[16],
//       light[3], light_int, ambient, matrix_base, node[12..19]
//   'M' pass, addr, stack_ptr, list_depth, top[16], vp[16]
//   'P' pass, model, word_off, hdr[7], nv_in, nv_new, nv_clip, status,
//       clip_in[4][7]   (x, y, z, w, p0, p1, p2 before clip)
//   'T' pass, alpha, buf_index, poly_seq, v[3][7], tex_en, page, tex_x,
//       tex_y, tex_w, tex_h, tex_fmt, param, transparency, color
//       v = x, y, z, w, p0, p1, p2 after clip, divide and viewport:
//       screen x, y; z in [near, far]; w (the rasteriser takes w and 1/w);
//       p0, p1 = 256 u / w, 256 v / w in texels; p2 = shade 0..255, not / w
//   'X' pass, model, word_off, reason    model ended early: 1 first polygon
//       shares vertices, 2 header word 6 is zero, 3 both discard bits,
//       4 texture size field >= 6, 5 triangle buffer full
//   'E' polys, verts_new, verts_reused, tris_opaque, tris_alpha, bad_addr
//
// MAME draws a pass's opaque triangles in buffer order, then its alpha
// triangles in reverse buffer order.

#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace r3d {

struct record {
  char tag;
  std::vector<uint32_t> w;
};

inline bool read_trilist(std::string const &fn, std::vector<record> &out) {
  std::FILE *f = std::fopen(fn.c_str(), "rb");
  if (!f) return false;
  std::vector<uint32_t> buf;
  uint32_t tmp[4096];
  size_t n;
  while ((n = std::fread(tmp, 4, 4096, f)) > 0) buf.insert(buf.end(), tmp, tmp + n);
  std::fclose(f);
  out.clear();
  for (size_t i = 0; i + 1 < buf.size();) {
    record r;
    r.tag = char(buf[i]);
    uint32_t const len = buf[i + 1];
    if (i + 2 + len > buf.size()) break;
    r.w.assign(buf.begin() + i + 2, buf.begin() + i + 2 + len);
    out.push_back(std::move(r));
    i += 2 + len;
  }
  return true;
}

struct trilist_writer {
  std::vector<uint32_t> buf;
  size_t rec = 0;
  void begin(char t) { rec = buf.size(); buf.push_back(uint32_t(t)); buf.push_back(0); }
  void w(uint32_t x) { buf.push_back(x); }
  void f(float x) { uint32_t u; std::memcpy(&u, &x, 4); buf.push_back(u); }
  void end() { buf[rec + 1] = uint32_t(buf.size() - rec - 2); }
  bool save(std::string const &fn) const {
    std::FILE *o = std::fopen(fn.c_str(), "wb");
    if (!o) return false;
    std::fwrite(buf.data(), 4, buf.size(), o);
    std::fclose(o);
    return true;
  }
  // records in memory, for an in-process diff
  void records(std::vector<record> &out) const {
    out.clear();
    for (size_t i = 0; i + 1 < buf.size();) {
      record r;
      r.tag = char(buf[i]);
      uint32_t const len = buf[i + 1];
      r.w.assign(buf.begin() + i + 2, buf.begin() + i + 2 + len);
      out.push_back(std::move(r));
      i += 2 + len;
    }
  }
};

} // namespace r3d
