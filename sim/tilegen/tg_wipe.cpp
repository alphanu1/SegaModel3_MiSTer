// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Stencil-form study on per-frame dumps (sim/tilegen/notes.md, "The stencil
// against cabinet footage").
//
//   tg_wipe <outdir> <base>...
//
// For each dump: the MAME and Supermodel literal renderers, and the param model
// with the stencil split into its three choices,
//   bit order    bit 15 leftmost (SM) / bit 0 leftmost (MAME)
//   coordinate   screen x (SM) / the layer's map x after scroll (MAME)
//   pair         exclusive: both layers of a pair test one bit, at the even
//                layer's coordinate / per layer: each layer tests its own
//                (in screen space the two are the same)
// with every other switch at Supermodel's. Writes <outdir>/<frame>_<form>.pgm,
// 255 where a tile pixel above the 3D is drawn and 0 where the 3D shows, and
// prints per form the pixels whose coverage differs from each literal.
// Checks first that the variant matching each emulator equals param().
#include "tg_ref.h"

#include <string>

using namespace tg;

struct Form {
  bool b15;      // bit 15 leftmost
  bool screen;   // screen-space coordinate
  bool excl;     // one bit per pair (at the even layer's coordinate)
  std::string name() const {
    return std::string(b15 ? "b15" : "b0") + (screen ? "_scr" : "_map") + (screen ? "" : (excl ? "_excl" : "_lay"));
  }
};

// param() with the stencil test generalised; everything else as in tg_ref.h.
static void param_form(const Dump &d, const Cfg &c, const Form &fm, Frame &f) {
  f.assign((size_t)W * H, Pix{});
  uint8_t pri = d.regs[0x21];
  int order[4];
  int np = pass_order(d, order);
  for (int y = 0; y < H; y++) {
    for (int k = 0; k < np; k++) {
      int l = order[k];
      LayerLine L = layer_line(d, c, l, y);
      if (!L.en) continue;
      LayerLine E = layer_line(d, c, l & 2, y);   // the even layer of the pair
      int odd = l & 1;
      bool is4 = (pri >> (4 + l)) & 1;
      for (int x = L.xlo; x <= L.xhi; x++) {
        int mapx = (x + L.off) & 511;
        int cx = fm.screen ? x : (fm.excl ? ((x + E.off) & 511) : mapx);
        int span = (cx >> 5) & 15;
        int bit = (L.pmask >> (fm.b15 ? 15 - span : span)) & 1;
        if (!(bit ^ odd)) continue;
        int srcx = mapx;
        if (c.stale && fm.screen && fm.b15) {
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

static void coverage(const Frame &f, std::vector<uint8_t> &m) {
  m.assign((size_t)W * H, 0);
  for (size_t i = 0; i < f.size(); i++) m[i] = (f[i].valid && f[i].above) ? 255 : 0;
}

static void write_pgm(const std::string &path, const std::vector<uint8_t> &m) {
  FILE *o = std::fopen(path.c_str(), "wb");
  if (!o) return;
  std::fprintf(o, "P5\n%d %d\n255\n", W, H);
  std::fwrite(m.data(), 1, m.size(), o);
  std::fclose(o);
}

static long cdiff(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b) {
  long n = 0;
  for (size_t i = 0; i < a.size(); i++) n += a[i] != b[i];
  return n;
}

int main(int argc, char **argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: tg_wipe <outdir> <base>...\n"); return 2; }
  std::string out = argv[1];
  Form forms[6] = {{true, true, true}, {false, true, true}, {true, false, true},
                   {true, false, false}, {false, false, true}, {false, false, false}};
  int bad = 0;
  for (int a = 2; a < argc; a++) {
    Dump d;
    if (!d.load(argv[a])) { std::printf("%s: load failed\n", argv[a]); bad++; continue; }
    std::string fr = d.name.substr(d.name.find_last_of('/') + 1);
    std::string tag = d.name.substr(0, d.name.find_last_of('/'));
    tag = tag.substr(tag.find_last_of('/') + 1) + "_" + fr;
    std::vector<uint32_t> below, above, surf[2];
    mame_literal(d, below, above);
    sm_literal(d, surf);
    Frame lm, ls;
    from_mame(below, above, lm);
    from_sm(surf, ls);
    std::vector<uint8_t> cm, cs;
    coverage(lm, cm);
    coverage(ls, cs);
    write_pgm(out + "/" + tag + "_mame_literal.pgm", cm);
    write_pgm(out + "/" + tag + "_sm_literal.pgm", cs);
    // self-checks: the variants that are the emulators' forms equal param()
    Frame p1, p2, v1, v2;
    param(d, Cfg::sm(), p1);
    param_form(d, Cfg::sm(), forms[0], v1);
    param(d, Cfg::mame(), p2);
    param_form(d, Cfg::mame(), forms[5], v2);
    long e1 = diff(p1, v1), e2 = diff(p2, v2);
    if (e1 || e2) bad++;
    std::printf("%s  self-check %ld/%ld  literal MAME vs SM %ld", tag.c_str(), e1, e2, cdiff(cm, cs));
    for (auto &fm : forms) {
      Frame v;
      param_form(d, Cfg::sm(), fm, v);
      std::vector<uint8_t> cv;
      coverage(v, cv);
      write_pgm(out + "/" + tag + "_" + fm.name() + ".pgm", cv);
      std::printf("  %s %ld/%ld", fm.name().c_str(), cdiff(cv, cm), cdiff(cv, cs));
    }
    std::printf("\n");
  }
  std::printf("%s\n", bad ? "FAIL" : "PASS");
  return bad ? 1 : 0;
}
