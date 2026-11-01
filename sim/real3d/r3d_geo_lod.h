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
// The geometry's LOD word for a rasteriser frame (d25_opts::lodb): the model
// r3d_geo with lod_word on a frame's dump, MAME's correctly rounded frustum
// (so kz is MAME's) and every node walked (so its polygon numbers are MAME's
// list's), and each polygon's B from the record the geometry sends: the
// inverted copy's -B for a double-sided polygon facing away, the polygon's
// own when both are sent. own = true: always the polygon's own (measurement).

#pragma once
#include "r3d_geo.h"
#include "r3d_trilist.h"
#include <zlib.h>
#include <cstdint>
#include <string>
#include <vector>

namespace r3d {

inline bool geo_lod_load(std::string const &fn, r3d_mem &m) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  bool ok = gzread(g, m.cull_lo.data(), 0x400000) == 0x400000 && gzread(g, m.cull_hi.data(), 0x100000) == 0x100000 &&
            gzread(g, m.poly.data(), 0x400000) == 0x400000;
  gzclose(g);
  return ok;
}

inline std::vector<uint32_t> geo_lod_words(r3d_mem const &m, bool own = false) {
  r3d_geo geo(m, arith::rtl, false);
  geo.setup_cr = true; geo.frustum_sm = false; geo.cull = false;
  geo.rows(true);
  geo.lod_word = true;
  geo.frame(0);
  std::vector<record> recs;
  geo.out.records(recs);
  std::vector<uint32_t> lodb;
  for (auto const &rc : recs) {
    if (rc.tag != 'T' || rc.w.size() < 39) continue;
    uint32_t const pn = rc.w[3];
    if (pn >= lodb.size()) lodb.resize(pn + 1, 0xffffffffu);
    bool const away = rc.w[35] & 2, copy = rc.w[35] & 1;
    if (own) { if (!copy) lodb[pn] = rc.w.back(); }
    else if (!away && (lodb[pn] == 0xffffffffu || !copy)) lodb[pn] = rc.w.back();
  }
  return lodb;
}

} // namespace r3d
