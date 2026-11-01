// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Tile generator reference models (M0-E).
//
// A dump is the tile generator's memory, F1000000-F111FFFF (0x120000 bytes,
// bus byte order: the byte the PowerPC sees at each address), and its
// registers, F1180000-F11800FF (0x100 bytes, bus order). The chip reads its RAM
// little-endian, so every field below is a little-endian load at a byte offset.
//
// Three renderers:
//   mame_literal  MAME 0.289 model3_v.cpp draw_layer / screen_update_model3,
//                 statement by statement, on MAME's own host-memory layout
//                 (uint64 arrays on a little-endian host, the gfx_element bit
//                 decode, pens from pal5bit). Output: the below-3D and above-3D
//                 passes on cleared bitmaps, as the instrumented MAME dumps them.
//   sm_literal    Supermodel 036af1c CTileGen::DrawLine and GetColour32,
//                 statement by statement. Output: its two draw surfaces.
//   param         One model with a switch per point where the two differ. With
//                 every switch at MAME it equals mame_literal; at Supermodel it
//                 equals sm_literal. The RTL is built to this model.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace tg {

constexpr int W = 496, H = 384;

struct Dump {
  std::vector<uint8_t> vram;   // 0x120000
  uint8_t regs[256];
  std::string name;
  bool load(const std::string &base) {
    name = base;
    vram.assign(0x120000, 0);
    FILE *f = std::fopen((base + ".vram").c_str(), "rb");
    if (!f) return false;
    size_t n = std::fread(vram.data(), 1, vram.size(), f);
    std::fclose(f);
    f = std::fopen((base + ".regs").c_str(), "rb");
    if (!f) return false;
    size_t m = std::fread(regs, 1, 256, f);
    std::fclose(f);
    return n == vram.size() && m == 256;
  }
  uint32_t le16(uint32_t a) const { return vram[a] | (vram[a + 1] << 8); }
  uint32_t le32(uint32_t a) const { return vram[a] | (vram[a + 1] << 8) | (vram[a + 2] << 16) | ((uint32_t)vram[a + 3] << 24); }
  uint32_t reg(uint32_t off) const { return regs[off] | (regs[off + 1] << 8) | (regs[off + 2] << 16) | ((uint32_t)regs[off + 3] << 24); }
  uint64_t be64(const uint8_t *p) const { uint64_t v = 0; for (int i = 0; i < 8; i++) v = (v << 8) | p[i]; return v; }
};

// One output pixel, as the RTL presents it to the compositor.
struct Pix {
  uint8_t valid = 0;   // a tile layer drew here
  uint8_t above = 0;   // that layer is above the 3D
  uint8_t layer = 0;   // which layer (not compared: the RTL does not output it)
  uint16_t entry = 0;  // palette entry, bits 14:0
  uint32_t rgb = 0;    // 0x00RRGGBB
  bool operator==(const Pix &o) const { return valid == o.valid && above == o.above && rgb == o.rgb; }
  bool operator!=(const Pix &o) const { return !(*this == o); }
};
using Frame = std::vector<Pix>;   // H * W

inline uint8_t pal5bit(uint32_t v) { v &= 31; return (uint8_t)((v << 3) | (v >> 2)); }

// ---------------------------------------------------------------- MAME, literal
// below / above: H x W ARGB, alpha 0 = nothing drawn (pens have alpha 0xFF).
inline void mame_literal(const Dump &d, std::vector<uint32_t> &below, std::vector<uint32_t> &above) {
  // Host memory as MAME holds it: m_m3_char_ram (uint64[0x20000]), m_m3_tile_ram
  // (uint64[0x1000]), m_paletteram64 (uint64[0x4000]), little-endian host.
  std::vector<uint64_t> charram(0x100000 / 8, 0), tileram(0x8000 / 8), palram64(0x20000 / 8);
  for (int i = 0; i < 0xf8000 / 8; i++) charram[i] = d.be64(&d.vram[8 * i]);
  for (int i = 0; i < 0x8000 / 8; i++) tileram[i] = d.be64(&d.vram[0xf8000 + 8 * i]);
  for (int i = 0; i < 0x20000 / 8; i++) palram64[i] = d.be64(&d.vram[0x100000 + 8 * i]);
  auto u16of = [](const std::vector<uint64_t> &a, size_t base64, size_t k) -> uint16_t {
    return (uint16_t)(a[base64 + k / 4] >> (16 * (k % 4)));
  };
  auto u32of = [](const std::vector<uint64_t> &a, size_t base64, size_t k) -> uint32_t {
    return (uint32_t)(a[base64 + k / 2] >> (32 * (k % 2)));
  };
  auto hostbyte = [&](size_t h) -> uint8_t { return (uint8_t)(charram[h / 8] >> (8 * (h % 8))); };
  auto rev16 = [](uint32_t x) -> uint32_t { return ((x >> 8) & 0xff) | ((x << 8) & 0xff00); };
  auto rev32 = [](uint32_t x) -> uint32_t {
    return ((x >> 24) & 0xff) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000) | ((x << 24) & 0xff000000u);
  };
  // pens
  std::vector<uint32_t> pens(32768);
  for (int off = 0; off < 0x4000; off++) {
    uint32_t data1 = rev32((uint32_t)(palram64[off] >> 32));
    uint32_t data2 = rev32((uint32_t)(palram64[off]));
    auto pen = [](uint32_t dd) { return 0xff000000u | (pal5bit(dd) << 16) | (pal5bit(dd >> 5) << 8) | pal5bit(dd >> 10); };
    pens[2 * off] = pen(data1);
    pens[2 * off + 1] = pen(data2);
  }
  // gfx decode: char4_layout / char8_layout through readbit on the host bytes
  static const int x4[8] = {0, 4, 8, 12, 16, 20, 24, 28};
  static const int y4[8] = {32, 0, 96, 64, 160, 128, 224, 192};
  static const int x8[8] = {32, 40, 48, 56, 0, 8, 16, 24};
  auto readbit = [&](size_t bit) { return (hostbyte(bit / 8) & (0x80 >> (bit % 8))) != 0; };
  auto pix4 = [&](uint32_t code, int y, int x) {
    int v = 0;
    for (int p = 0; p < 4; p++) if (readbit((size_t)code * 256 + p + y4[y] + x4[x])) v |= 1 << (3 - p);
    return v;
  };
  auto pix8 = [&](uint32_t code, int y, int x) {
    int v = 0;
    for (int p = 0; p < 8; p++) if (readbit((size_t)code * 512 + p + 64 * y + x8[x])) v |= 1 << (7 - p);
    return v;
  };
  // registers as MAME holds them
  uint16_t layer_priority = (uint16_t)(d.be64(&d.regs[0x20]) >> 48);
  uint64_t layer_scroll[2] = {d.be64(&d.regs[0x60]), d.be64(&d.regs[0x68])};
  uint32_t layer_data[4] = {rev32((uint32_t)(layer_scroll[0] >> 32)), rev32((uint32_t)layer_scroll[0]),
                            rev32((uint32_t)(layer_scroll[1] >> 32)), rev32((uint32_t)layer_scroll[1])};
  int sxv[4], syv[4];
  for (int l = 0; l < 4; l++) { sxv[l] = layer_data[l] & 0x1ff; syv[l] = (layer_data[l] >> 16) & 0x1ff; }

  auto draw_layer = [&](std::vector<uint32_t> &bitmap, int layer, int sx, int sy) {
    int bitdepth = (layer_priority & (0x10 << layer)) ? 1 : 0;   // 1 = 4-bit
    const int x1 = 0, y1 = 0, x2 = W - 1, y2 = H - 1;
    int ix = sx, iy = sy;
    if (ix < 0) ix = 0 - ix;
    if (iy < 0) iy = 0 - iy;
    for (int y = y1; y <= y2; y++) {
      uint32_t *dst = &bitmap[(size_t)y * W];
      int srow = iy & 0x1ff;
      int rowscroll = rev16(u16of(charram, 0x1ec00, ((layer * 0x200) + y) ^ 3)) & 0x7fff;
      if (rowscroll & 0x100) rowscroll |= ~0x1ff;
      const uint8_t shift = (layer & 2) ? 0 : 16;
      uint16_t rowmask = (rev32(u32of(charram, 0x1ee00, (y & 0x1ff) ^ 1)) >> shift) & 0xffff;
      if (layer & 1) rowmask ^= 0xffff;
      int iix = ix & 0x1ff;
      int rx1 = x1 - (rowscroll * 2);
      int rx2 = x2 - (rowscroll * 2);
      if (rx1 < 0) { iix += (0 - rx1); rx1 = 0; }
      if (rx2 > x2) rx2 = x2;
      for (int x = rx1; x <= rx2; x++) {
        uint32_t mask = rowmask & (1 << ((iix & 0x1ff) >> 5));
        if (mask) {
          // the tilemap pixmap at (iix & 0x1ff, srow)
          int mx = iix & 0x1ff;
          int tile_index = (srow >> 3) * 64 + (mx >> 3);
          uint16_t t = (uint16_t)rev16(u16of(tileram, layer * 0x400, ((tile_index & 3) ^ 2) + (tile_index / 4) * 4));
          int tile = ((t << 1) & 0x7ffe) | ((t >> 15) & 0x1);
          uint16_t p0;
          if (bitdepth) {
            int color = (t & 0x7ff0) >> 4;
            p0 = (uint16_t)(16 * (color % 2048) + pix4((uint32_t)(tile % 31744), srow & 7, mx & 7));
          } else {
            int color = (t & 0x7f00) >> 8;
            p0 = (uint16_t)(256 * (color % 128) + pix8((uint32_t)((tile >> 1) % 15872), srow & 7, mx & 7));
          }
          if ((u32of(palram64, 0, p0 ^ 1) & 0x00800000) == 0) dst[x] = pens[p0];
        }
        iix++;
      }
      iy++;
    }
  };
  below.assign((size_t)W * H, 0);
  above.assign((size_t)W * H, 0);
  if ((layer_data[3] & 0x80000000) && (layer_priority & 0x8) == 0) draw_layer(below, 3, sxv[3], syv[3]);
  if ((layer_data[2] & 0x80000000) && (layer_priority & 0x4) == 0) draw_layer(below, 2, sxv[2], syv[2]);
  if ((layer_data[1] & 0x80000000) && (layer_priority & 0x2) == 0) draw_layer(below, 1, sxv[1], syv[1]);
  if ((layer_data[0] & 0x80000000) && (layer_priority & 0x1) == 0) draw_layer(below, 0, sxv[0], syv[0]);
  if ((layer_data[3] & 0x80000000) && (layer_priority & 0x8) != 0) draw_layer(above, 3, sxv[3], syv[3]);
  if ((layer_data[2] & 0x80000000) && (layer_priority & 0x4) != 0) draw_layer(above, 2, sxv[2], syv[2]);
  if ((layer_data[0] & 0x80000000) && (layer_priority & 0x1) != 0) draw_layer(above, 0, sxv[0], syv[0]);
  if ((layer_data[1] & 0x80000000) && (layer_priority & 0x2) != 0) draw_layer(above, 1, sxv[1], syv[1]);
}

// ----------------------------------------------------------- Supermodel, literal
// surf[0] bottom, surf[1] top: 384 x 512 RGBA as CTileGen's TileGenBuffer holds
// them ((a << 24) | (b << 16) | (g << 8) | r, a = 0 or 255).
inline void sm_literal(const Dump &d, std::vector<uint32_t> surf[2]) {
  auto vramP = [&](int i) { return d.le32(4 * (uint32_t)i); };
  auto regs = [&](int i) { return d.reg(4 * (uint32_t)i); };
  struct Off { int r, g, b; } offs[2];
  for (int k = 0; k < 2; k++) {
    uint32_t reg = regs(0x40 / 4 + k);
    offs[k].b = (int32_t)(int8_t)((reg >> 16) & 0xFF) * 2;
    offs[k].g = (int32_t)(int8_t)((reg >> 8) & 0xFF) * 2;
    offs[k].r = (int32_t)(int8_t)((reg >> 0) & 0xFF) * 2;
  }
  std::vector<uint32_t> pal[2];
  for (int k = 0; k < 2; k++) {
    pal[k].resize(32768);
    for (int i = 0; i < 32768; i++) {
      uint32_t data = vramP(0x40000 + i);
      int a = (((data >> 15) + 1) & 1) * 255;
      int b = ((((data >> 10) & 0x1F) * 255) / 31) & a;
      int g = ((((data >> 5) & 0x1F) * 255) / 31) & a;
      int r = (((data & 0x1F) * 255) / 31) & a;
      int rr = offs[k].r + r, gg = offs[k].g + g, bb = offs[k].b + b;
      if (rr > 255) rr = 255; else if (rr < 0) rr = 0;
      if (gg > 255) gg = 255; else if (gg < 0) gg = 0;
      if (bb > 255) bb = 255; else if (bb < 0) bb = 0;
      pal[k][i] = ((uint32_t)a << 24) | (bb << 16) | (gg << 8) | rr;
    }
  }
  auto IsEnabled = [&](int l) { return (regs(0x60 / 4 + l) & 0x80000000) > 0; };
  auto Above3D = [&](int l) { return (int)((regs(0x20 / 4) >> (8 + l)) & 0x1); };
  auto Is4Bit = [&](int l) { return (regs(0x20 / 4) & (1 << (12 + l))) != 0; };
  auto GetYScroll = [&](int l) { return (int)((regs(0x60 / 4 + l) >> 16) & 0x1FF); };
  auto GetXScroll = [&](int l) { return (int)(regs(0x60 / 4 + l) & 0x3FF); };
  auto LineScrollMode = [&](int l) { return (regs(0x60 / 4 + l) & 0x8000) != 0; };
  auto GetLineScroll = [&](int l, int y) {
    int index = ((0xF6000 + (l * 0x400)) / 4) + (y / 2);
    int shift = (1 - (y % 2)) * 16;
    return (int)((vramP(index) >> shift) & 0xFFFFu);
  };
  auto GetTilePairNumber = [](int x, int y, int xs, int ys) {
    int xIndex = ((x + xs) / 16) & 0x1F;
    int yIndex = ((y + ys) / 8) & 0x3F;
    return (yIndex * 32) + xIndex;
  };
  auto GetTileData = [&](int l, int n) { return (int)vramP((0xF8000 + (l * 0x2000)) / 4 + n); };
  auto GetVFine = [](int y, int ys) { return (y + ys) & 7; };
  auto GetHFine = [](int x, int xs) { return (x + xs) & 15; };
  auto GetLineMask = [&](int l, int y) {
    unsigned shift = (l < 2) ? 16u : 0u;
    return (int)((vramP(0xF7000 / 4 + y) >> shift) & 0xFFFFu);
  };
  auto GetPixelMask = [](int lineMask, int x) { return (lineMask & (1 << (15 - (x / 32)))) != 0; };
  auto Draw4 = [&](int tileData, int hStart, int hEnd, int vFine, uint32_t *lb, const uint32_t *p, int &x) {
    int po[2];
    po[0] = ((((tileData >> 16) & 0x3FFF) << 1) | ((tileData >> 31) & 1)) * 32 / 4;
    po[1] = (((tileData & 0x3FFF) << 1) | ((tileData >> 15) & 1)) * 32 / 4;
    int pi[2] = {(tileData >> 16) & 0x7FF0, tileData & 0x7FF0};
    uint32_t pat[2] = {vramP(po[0] + vFine), vramP(po[1] + vFine)};
    for (int i = hStart; i < hEnd; i++, x++) {
      uint32_t pattern = pat[i / 8];
      uint32_t px = (pattern >> ((7 - (i % 8)) * 4)) & 0xFu;
      uint32_t c = p[pi[i / 8] | px];
      if (c >= 0x1000000) lb[x] = c;
    }
  };
  auto Draw8 = [&](int tileData, int hStart, int hEnd, int vFine, uint32_t *lb, const uint32_t *p, int &x) {
    int po[2];
    po[0] = ((tileData >> 16) & 0x3FFF) * 64 / 4;
    po[1] = (tileData & 0x3FFF) * 64 / 4;
    int pi[2] = {(tileData >> 16) & 0x7F00, tileData & 0x7F00};
    uint32_t pat[4] = {vramP(po[0] + vFine * 2), vramP(po[0] + vFine * 2 + 1), vramP(po[1] + vFine * 2),
                       vramP(po[1] + vFine * 2 + 1)};
    for (int i = hStart; i < hEnd; i++, x++) {
      uint32_t pattern = pat[i / 4];
      uint32_t px = (pattern >> ((3 - (i % 4)) * 8)) & 0xFFu;
      uint32_t c = p[pi[i / 8] | px];
      if (c >= 0x1000000) lb[x] = c;
    }
  };
  for (int s = 0; s < 2; s++) surf[s].assign(512 * 384 + 64, 0);   // DrawLine may run past x = 511 by < 16
  for (int line = 0; line < 384; line++) {
    for (int i = 2; i-- > 0;) {
      const int primaryIndex = i * 2, altIndex = i * 2 + 1;
      bool hasLayer[2] = {IsEnabled(primaryIndex), IsEnabled(altIndex)};
      if (!hasLayer[0] && !hasLayer[1]) continue;
      uint32_t *drawLayers[2] = {&surf[Above3D(primaryIndex)][512 * line], &surf[Above3D(altIndex)][512 * line]};
      int lineMask = GetLineMask(primaryIndex, line);
      int scrollX[2] = {LineScrollMode(primaryIndex) ? GetLineScroll(primaryIndex, line) : GetXScroll(primaryIndex),
                        LineScrollMode(altIndex) ? GetLineScroll(altIndex, line) : GetXScroll(altIndex)};
      int scrollY[2] = {GetYScroll(primaryIndex), GetYScroll(altIndex)};
      for (int x = 0; x < 496;) {
        int layer = (GetPixelMask(lineMask, x) + 1) & 1;
        bool maskSwitch = GetPixelMask(lineMask, x) != GetPixelMask(lineMask, x + 15);
        if (hasLayer[layer]) {
          const int index = primaryIndex + layer;
          int tilePairNumber = GetTilePairNumber(maskSwitch ? (x - 16) : x, line, scrollX[layer], scrollY[layer]);
          int hStart = GetHFine(x, scrollX[layer]);
          int hEnd = maskSwitch ? 16 - (x & 15) : 16;
          int vFine = GetVFine(line, scrollY[layer]);
          int tileData = GetTileData(index, tilePairNumber);
          if (Is4Bit(index)) Draw4(tileData, hStart, hEnd, vFine, drawLayers[layer], pal[index / 2].data(), x);
          else Draw8(tileData, hStart, hEnd, vFine, drawLayers[layer], pal[index / 2].data(), x);
        } else {
          int hFine = GetHFine(x, 0);
          x += 32 - hFine;
        }
      }
    }
  }
}

// --------------------------------------------------------------- the param model
struct Cfg {
  bool mask_screen = true;   // stencil in screen space, bit 15 leftmost (SM) / map space, bit 0 leftmost (MAME)
  bool scroll_sm = true;     // line scroll replaces X scroll when register bit 15 is set (SM) / always added x2 (MAME)
  bool rs_swap = true;       // line y's scroll word at 2 * (y ^ 1) (SM) / at 2 * y (MAME)
  bool stale = true;         // SM: pixels before a mask change in a straddling tile pair use the pair 16 pixels left
  bool code_wrap = false;    // MAME: pattern codes past 0xF8000 wrap (code % 31744 or % 15872)
  bool expand_sm = true;     // SM: 5 to 8 bits as x * 255 / 31 / MAME: pal5bit, (x << 3) | (x >> 2)
  bool offset = true;        // SM: the pair's colour offset (F1180040 / F1180044, signed, x2, clamped); MAME has none
  static Cfg mame() { return Cfg{false, false, false, false, true, false, false}; }
  static Cfg sm() { return Cfg{}; }
};

inline uint8_t sm5to8(uint32_t v) { return (uint8_t)(((v & 31) * 255) / 31); }

inline uint32_t colour(const Dump &d, const Cfg &c, int pair, uint16_t e) {
  uint32_t reg = c.offset ? d.reg(0x40 + 4 * pair) : 0;
  int ch[3];   // r, g, b
  for (int k = 0; k < 3; k++) ch[k] = c.expand_sm ? sm5to8(e >> (5 * k)) : pal5bit(e >> (5 * k));
  int of[3] = {(int8_t)(reg & 0xff) * 2, (int8_t)((reg >> 8) & 0xff) * 2, (int8_t)((reg >> 16) & 0xff) * 2};
  uint32_t out = 0;
  for (int k = 0; k < 3; k++) {
    int v = ch[k] + of[k];
    if (v > 255) v = 255;
    if (v < 0) v = 0;
    out |= (uint32_t)v << (16 - 8 * k);
  }
  return out;
}

// Per-line layer set-up shared by the model and the RTL bench's diagnostics.
struct LayerLine {
  bool en;
  int off;          // map x = (x + off) & 511
  int xlo, xhi;     // drawn screen x range (empty when xlo > xhi)
  int S;            // SM: the scroll value whose low 4 bits place tile pairs
  int my;           // map y
  uint16_t pmask;   // the pair's 16 mask bits
};

inline LayerLine layer_line(const Dump &d, const Cfg &c, int l, int y) {
  LayerLine L{};
  uint32_t r = d.reg(0x60 + 4 * l);
  L.en = (r >> 31) & 1;
  L.my = (int)((y + ((r >> 16) & 0x1ff)) & 0x1ff);
  uint32_t mw = d.le32(0xF7000 + 4 * (uint32_t)(y & 0x1ff));
  L.pmask = (uint16_t)((l < 2) ? (mw >> 16) : (mw & 0xffff));
  uint32_t rs16 = d.le16(0xF6000 + 0x400 * (uint32_t)l + 2 * (uint32_t)(c.rs_swap ? (y ^ 1) : y));
  if (c.scroll_sm) {
    L.S = (r & 0x8000) ? (int)rs16 : (int)(r & 0x3ff);
    L.off = L.S & 511;
    L.xlo = 0;
    L.xhi = W - 1;
  } else {
    int rs = (int)(rs16 & 0x7fff);
    if (rs & 0x100) rs |= ~0x1ff;
    L.off = ((int)(r & 0x1ff) + 2 * rs) & 511;
    L.S = L.off;
    L.xlo = -2 * rs > 0 ? -2 * rs : 0;
    L.xhi = (W - 1) - 2 * rs < W - 1 ? (W - 1) - 2 * rs : W - 1;
  }
  return L;
}

// The draw order: MAME's (below 3, 2, 1, 0; above 3, 2, 0, 1). Later passes win.
// Supermodel's (pair B then pair A on each surface, top over bottom) is the same
// order wherever it matters, because its pairs never overlap.
inline int pass_order(const Dump &d, int out[4]) {
  uint8_t pri = d.regs[0x21];
  static const int below[4] = {3, 2, 1, 0}, above[4] = {3, 2, 0, 1};
  int n = 0;
  for (int k = 0; k < 4; k++) if (!((pri >> below[k]) & 1)) out[n++] = below[k];
  for (int k = 0; k < 4; k++) if ((pri >> above[k]) & 1) out[n++] = above[k];
  return n;
}

inline void param(const Dump &d, const Cfg &c, Frame &f) {
  f.assign((size_t)W * H, Pix{});
  uint8_t pri = d.regs[0x21];
  int order[4];
  int np = pass_order(d, order);
  for (int y = 0; y < H; y++) {
    for (int k = 0; k < np; k++) {
      int l = order[k];
      LayerLine L = layer_line(d, c, l, y);
      if (!L.en) continue;
      int odd = l & 1;
      bool is4 = (pri >> (4 + l)) & 1;
      for (int x = L.xlo; x <= L.xhi; x++) {
        int mapx = (x + L.off) & 511;
        int bit = c.mask_screen ? ((L.pmask >> (15 - (x >> 5))) & 1) : ((L.pmask >> (mapx >> 5)) & 1);
        if (!(bit ^ odd)) continue;
        int srcx = mapx;
        if (c.stale) {
          int B = (x | 31) + 1;
          if (B < 512) {
            int mB = (L.pmask >> (15 - (B >> 5))) & 1, mB1 = (L.pmask >> (15 - ((B - 1) >> 5))) & 1;
            int s = (B + L.S) & 15;
            if (mB != mB1 && s != 0 && (x & 31) >= 32 - s) srcx = (mapx - 16) & 511;
          }
        }
        uint32_t t = d.le16(0xF8000 + 0x2000 * (uint32_t)l + 128 * (uint32_t)(L.my >> 3) + 2 * (uint32_t)((srcx >> 3) ^ 1));
        int row = L.my & 7, col = srcx & 7;
        uint32_t idx;
        if (is4) {
          uint32_t code = ((t & 0x3fff) << 1) | (t >> 15);
          if (c.code_wrap && code >= 31744) code -= 31744;
          uint32_t px = (d.le32(code * 32 + 4 * row) >> (28 - 4 * col)) & 15;
          idx = (t & 0x7ff0) | px;
        } else {
          uint32_t code = t & 0x3fff;
          if (c.code_wrap && code >= 15872) code -= 15872;
          uint32_t px = (d.le32(code * 64 + 8 * row + 4 * (col >> 2)) >> (24 - 8 * (col & 3))) & 0xff;
          idx = (t & 0x7f00) | px;
        }
        uint32_t e = d.le16(0x100000 + 4 * idx);
        if (e & 0x8000) continue;
        Pix &p = f[(size_t)y * W + x];
        p.valid = 1;
        p.above = (pri >> l) & 1;
        p.layer = (uint8_t)l;
        p.entry = (uint16_t)(e & 0x7fff);
      }
    }
  }
  for (auto &p : f) if (p.valid) p.rgb = colour(d, c, p.layer >> 1, p.entry);
}

// Unify the literal renderers' outputs.
inline void from_mame(const std::vector<uint32_t> &below, const std::vector<uint32_t> &above, Frame &f) {
  f.assign((size_t)W * H, Pix{});
  for (size_t i = 0; i < f.size(); i++) {
    uint32_t a = above[i], b = below[i];
    if (a >> 24) { f[i].valid = 1; f[i].above = 1; f[i].rgb = a & 0xffffff; }
    else if (b >> 24) { f[i].valid = 1; f[i].above = 0; f[i].rgb = b & 0xffffff; }
  }
}
inline uint32_t sm_rgb(uint32_t v) { return ((v & 0xff) << 16) | (v & 0xff00) | ((v >> 16) & 0xff); }
inline void from_sm(const std::vector<uint32_t> surf[2], Frame &f) {
  f.assign((size_t)W * H, Pix{});
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      uint32_t t = surf[1][512 * y + x], b = surf[0][512 * y + x];
      Pix &p = f[(size_t)y * W + x];
      if (t >> 24) { p.valid = 1; p.above = 1; p.rgb = sm_rgb(t); }
      else if (b >> 24) { p.valid = 1; p.above = 0; p.rgb = sm_rgb(b); }
    }
}

// Colour-blind: drawn, class and palette entry (what the line buffer holds).
inline long diff_entry(const Frame &a, const Frame &b) {
  long n = 0;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i].valid != b[i].valid || a[i].above != b[i].above || (a[i].valid && a[i].entry != b[i].entry)) n++;
  return n;
}

inline long diff(const Frame &a, const Frame &b, int *first = nullptr) {
  long n = 0;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i] != b[i]) { if (!n && first) *first = (int)i; n++; }
  return n;
}

inline void write_ppm(const std::string &path, const Frame &f) {
  FILE *o = std::fopen(path.c_str(), "wb");
  if (!o) return;
  std::fprintf(o, "P6\n%d %d\n255\n", W, H);
  for (auto &p : f) { uint8_t c[3] = {(uint8_t)(p.rgb >> 16), (uint8_t)(p.rgb >> 8), (uint8_t)p.rgb}; std::fwrite(c, 1, 3, o); }
  std::fclose(o);
}

// Synthetic variant of a dump: the game's patterns, names and palette with the
// registers, line-scroll tables and stencil replaced by random ones, so every
// feature the games leave idle (line scroll, colour offsets, 4-bit layers, odd
// stencils, codes past the pattern area) is exercised. Deterministic per seed.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 1) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)(s >> 11); }
  uint32_t below(uint32_t n) { return next() % n; }
  bool chance(int pct) { return below(100) < (uint32_t)pct; }
};

inline void mutate(Dump &d, uint64_t seed) {
  Rng r(seed);
  auto put32 = [&](uint8_t *p, uint32_t v) { for (int k = 0; k < 4; k++) p[k] = (uint8_t)(v >> (8 * k)); };
  auto put16 = [&](uint32_t a, uint32_t v) { d.vram[a] = (uint8_t)v; d.vram[a + 1] = (uint8_t)(v >> 8); };
  std::memset(d.regs, 0, sizeof d.regs);
  put32(&d.regs[0x20], (r.next() & 0xffff00ffu) | ((r.next() & 0xff) << 8));
  for (int k = 0; k < 2; k++) put32(&d.regs[0x40 + 4 * k], r.chance(30) ? 0 : (r.next() & 0xffffff));
  for (int l = 0; l < 4; l++) {
    uint32_t v = r.next() & 0x7e00fc00u;                    // junk in the bits nothing reads
    v |= r.chance(85) ? 0x80000000u : 0;
    v |= (r.next() & 0x1ff) << 16;
    v |= r.next() & 0x3ff;
    v |= r.chance(50) ? 0x8000u : 0;
    put32(&d.regs[0x60 + 4 * l], v);
    int mode = (int)r.below(4);
    uint32_t c = r.next();
    int amp = (int)r.below(80), ph = (int)r.below(64);
    for (int y = 0; y < 512; y++) {
      uint32_t val;
      switch (mode) {
        case 0: val = c & 0xffff; break;
        case 1: val = (uint32_t)((int)(c & 0x1ff) + (int)(amp * ((y + ph) % 64 < 32 ? (y + ph) % 32 : 32 - (y + ph) % 32) / 32)) & 0x1ff; break;
        case 2: val = r.next() & 0xffff; break;
        default: { int sv = (int)r.below(160) - 80; val = ((uint32_t)sv & 0x1ff) | (r.chance(50) ? 0x8000u : 0); break; }
      }
      put16(0xF6000 + 0x400 * (uint32_t)l + 2 * (uint32_t)y, val);
    }
  }
  int mmode = (int)r.below(3);
  uint32_t m0 = r.next();
  for (int y = 0; y < 512; y++) {
    uint32_t m = mmode == 0 ? m0 : mmode == 1 ? r.next() : ((y / 16) % 2 ? m0 : ~m0);
    put32(&d.vram[0xF7000 + 4 * (uint32_t)y], m);
  }
  if (r.chance(30)) {
    int l = (int)r.below(4);
    for (int i = 0; i < 64 * 64; i++) put16(0xF8000 + 0x2000 * (uint32_t)l + 2 * (uint32_t)i, r.next() & 0xffff);
  }
}

}  // namespace tg
