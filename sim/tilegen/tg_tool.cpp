// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Tile generator reference checks (M0-E).
//   tg_tool check <dir>...   literal renderers against each emulator's own output,
//                            and the param model against the literal renderers
//   tg_tool xdiff <dir>...   MAME against Supermodel on the same dumps, and which
//                            switch each differing pixel comes from
//   tg_tool info <dir>...    register and table summary per dump
//   tg_tool ppm <base> <mame|sm> <out.ppm>
#include "tg_ref.h"

#include <algorithm>
#include <dirent.h>
#include <map>
#include <set>

using namespace tg;

static std::vector<std::string> bases(const std::string &dir) {
  std::vector<std::string> v;
  if (DIR *dp = opendir(dir.c_str())) {
    while (dirent *e = readdir(dp)) {
      std::string n = e->d_name;
      if (n.size() > 5 && n.substr(n.size() - 5) == ".vram") v.push_back(dir + "/" + n.substr(0, n.size() - 5));
    }
    closedir(dp);
  }
  std::sort(v.begin(), v.end());
  return v;
}

static bool readfile(const std::string &p, std::vector<uint32_t> &v, size_t words) {
  FILE *f = std::fopen(p.c_str(), "rb");
  if (!f) return false;
  v.resize(words);
  size_t n = std::fread(v.data(), 4, words, f);
  std::fclose(f);
  return n == words;
}

static int check(const std::vector<std::string> &dirs) {
  int bad = 0;
  for (auto &dir : dirs)
    for (auto &b : bases(dir)) {
      Dump d;
      if (!d.load(b)) { std::printf("%s: load failed\n", b.c_str()); bad++; continue; }
      std::vector<uint32_t> tile;
      std::vector<uint32_t> surf;
      if (readfile(b + ".tile", tile, 2 * (size_t)W * H)) {
        std::vector<uint32_t> below, above;
        mame_literal(d, below, above);
        long e1 = 0;
        for (size_t i = 0; i < (size_t)W * H; i++) {
          if (below[i] != tile[i]) e1++;
          if (above[i] != tile[(size_t)W * H + i]) e1++;
        }
        Frame lit, par;
        from_mame(below, above, lit);
        param(d, Cfg::mame(), par);
        long e2 = diff(lit, par);
        long drawn = 0;
        for (auto &p : lit) drawn += p.valid;
        std::printf("%s  MAME draw_layer vs literal %ld  literal vs param(MAME) %ld  (%ld px drawn)\n", b.c_str(), e1, e2,
                    drawn);
        if (e1 || e2) bad++;
      }
      if (readfile(b + ".surf", surf, 2 * 512 * 384)) {
        std::vector<uint32_t> s[2];
        sm_literal(d, s);
        long e1 = 0;
        for (int k = 0; k < 2; k++)
          for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
              if (s[k][512 * y + x] != surf[(size_t)k * 512 * 384 + 512 * y + x]) e1++;
        Frame lit, par;
        from_sm(s, lit);
        param(d, Cfg::sm(), par);
        long e2 = diff(lit, par);
        long drawn = 0;
        for (auto &p : lit) drawn += p.valid;
        std::string info;
        if (FILE *f = std::fopen((b + ".info").c_str(), "r")) {
          char line[64];
          while (std::fgets(line, sizeof line, f)) { line[std::strcspn(line, "\n")] = ' '; info += line; }
          std::fclose(f);
        }
        std::printf("%s  SM DrawLine vs literal %ld  literal vs param(SM) %ld  (%ld px drawn) %s\n", b.c_str(), e1, e2,
                    drawn, info.c_str());
        if (e2) bad++;
      }
    }
  std::printf("%s\n", bad ? "FAIL" : "PASS");
  return bad ? 1 : 0;
}

static int xdiff(const std::vector<std::string> &dirs) {
  const int NS = 7;
  const char *names[NS] = {"mask_screen", "scroll_sm", "rs_swap", "stale", "code_wrap", "expand_sm", "offset"};
  long tot = 0, tote = 0, totf[NS] = {0}, totb[NS] = {0};
  for (auto &dir : dirs)
    for (auto &b : bases(dir)) {
      Dump d;
      if (!d.load(b)) continue;
      Frame fm, fs;
      param(d, Cfg::mame(), fm);
      param(d, Cfg::sm(), fs);
      long n = diff(fm, fs), ne = diff_entry(fm, fs);
      tot += n;
      tote += ne;
      std::printf("%s  MAME vs SM %6ld px (%6ld by entry)", b.c_str(), n, ne);
      for (int k = 0; k < NS; k++) {
        // one switch moved from MAME to SM (fwd) and from SM to MAME (back)
        Cfg c = Cfg::mame();
        bool *fl[NS] = {&c.mask_screen, &c.scroll_sm, &c.rs_swap, &c.stale, &c.code_wrap, &c.expand_sm, &c.offset};
        *fl[k] = !*fl[k];
        Frame f1;
        param(d, c, f1);
        long a = diff(fm, f1);
        Cfg c2 = Cfg::sm();
        bool *fl2[NS] = {&c2.mask_screen, &c2.scroll_sm, &c2.rs_swap, &c2.stale, &c2.code_wrap, &c2.expand_sm, &c2.offset};
        *fl2[k] = !*fl2[k];
        Frame f2;
        param(d, c2, f2);
        long bb = diff(fs, f2);
        totf[k] += a;
        totb[k] += bb;
        if (a || bb) std::printf("  %s %ld/%ld", names[k], a, bb);
      }
      std::printf("\n");
    }
  std::printf("total MAME vs SM %ld px (%ld by entry); per switch (moved from MAME / moved from SM):", tot, tote);
  for (int k = 0; k < NS; k++) std::printf(" %s %ld/%ld", names[k], totf[k], totb[k]);
  std::printf("\n");
  return 0;
}

static int info(const std::vector<std::string> &dirs) {
  for (auto &dir : dirs)
    for (auto &b : bases(dir)) {
      Dump d;
      if (!d.load(b)) continue;
      uint8_t pri = d.regs[0x21];
      std::printf("%s pri %02X off %08X %08X", b.c_str(), pri, d.reg(0x40), d.reg(0x44));
      for (int l = 0; l < 4; l++) {
        uint32_t r = d.reg(0x60 + 4 * l);
        std::printf(" L%d:%c%c%c x%03X y%03X%s", l, (r >> 31) ? 'E' : '-', (pri >> (4 + l)) & 1 ? '4' : '8',
                    (pri >> l) & 1 ? 'A' : 'B', r & 0x3ff, (r >> 16) & 0x1ff, (r & 0x8000) ? " LS" : "");
        // the line-scroll table: distinct values over the 384 lines
        std::set<uint32_t> rs;
        for (int y = 0; y < H; y++) rs.insert(d.le16(0xF6000 + 0x400 * l + 2 * y));
        std::printf(" rs%zu", rs.size());
        if (rs.size() == 1) std::printf("=%X", *rs.begin());
      }
      std::set<uint32_t> masks;
      for (int y = 0; y < H; y++) masks.insert(d.le32(0xF7000 + 4 * y));
      std::printf(" masks %zu", masks.size());
      int k = 0;
      for (auto m : masks) { if (k++ < 4) std::printf(" %08X", m); }
      std::printf("\n");
    }
  return 0;
}

int main(int argc, char **argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: tg_tool check|xdiff|info <dir>... | ppm <base> <mame|sm> <out>\n"); return 2; }
  std::string cmd = argv[1];
  std::vector<std::string> dirs(argv + 2, argv + argc);
  if (cmd == "check") return check(dirs);
  if (cmd == "xdiff") return xdiff(dirs);
  if (cmd == "info") return info(dirs);
  if (cmd == "ppm" && argc == 5) {
    Dump d;
    if (!d.load(argv[2])) return 1;
    Frame f;
    param(d, std::string(argv[3]) == "sm" ? Cfg::sm() : Cfg::mame(), f);
    write_ppm(argv[4], f);
    return 0;
  }
  return 2;
}
