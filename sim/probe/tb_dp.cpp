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
// The DDR3 probe core (rtl/probe/dp_core.sv) against a DDR3 model.
//
//   Vdp_core [--busy PCT] [--lat LO HI] [--maxrd K] [--seed S] [--windows N]
//            [--png FILE] [--resets N] [--status HEX] [--status2 HEX AT]
//            [--ratio NUM DEN]
//
// The model: BUSY a random PCT of cycles, and always while K reads are held
// (--maxrd); a read's beats from LO..HI cycles after it is taken, in order,
// one a cycle at most (90% of cycles). Built with short windows (-GWIN0..3),
// so the screen's MB/s are scaled.
//
// The port's rules, enforced (a break is a failure): RD and WE never together;
// a command not taken (BUSY) is held unchanged, address, burst count and, for a
// write, data and byte enables; a write burst's beats keep its address and
// count, with no read until its last; burst counts 1-128. --resets pulses the
// core's reset N times at random (the DDR3 master is not reset), reads in
// flight included: the core must carry on. --status sets the OSD's status word
// (ddr3_probe.sv's bit map); --status2 changes it at cycle AT. The DDR3 clock
// runs at the ratio clk_sel names (1, 5/4, 17/20, 5/8 of the fixed clock), or
// --ratio NUM/DEN; a clock change drops the model PLL's lock for a while.
//
// Checks: no pattern errors; windows keep completing; the most reads held
// under --maxrd; minimum latency at least LO; the UART's start, debug and
// result lines decode, and every result line's numbers equal what the core
// held for that window.

#include "Vdp_core.h"
#include "Vdp_core___024root.h"
#include "verilated.h"
#include <zlib.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

static void png_chunk(std::FILE *f, char const *t, std::vector<uint8_t> const &d) {
  uint8_t l[4] = {uint8_t(d.size() >> 24), uint8_t(d.size() >> 16), uint8_t(d.size() >> 8), uint8_t(d.size())};
  std::fwrite(l, 1, 4, f); std::fwrite(t, 1, 4, f);
  if (!d.empty()) std::fwrite(d.data(), 1, d.size(), f);
  uLong c = crc32(0, reinterpret_cast<Bytef const *>(t), 4);
  if (!d.empty()) c = crc32(c, d.data(), uInt(d.size()));
  uint8_t cb[4] = {uint8_t(c >> 24), uint8_t(c >> 16), uint8_t(c >> 8), uint8_t(c)};
  std::fwrite(cb, 1, 4, f);
}
static void write_png(std::string const &fn, std::vector<uint32_t> const &px, int w, int h) {
  std::vector<uint8_t> raw;
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    for (int x = 0; x < w; x++) { uint32_t p = px[size_t(y) * w + x]; raw.push_back(p >> 16); raw.push_back(p >> 8); raw.push_back(p); }
  }
  uLongf zl = compressBound(uLong(raw.size()));
  std::vector<uint8_t> z(zl);
  compress2(z.data(), &zl, raw.data(), uLong(raw.size()), 6);
  z.resize(zl);
  std::FILE *f = std::fopen(fn.c_str(), "wb");
  if (!f) return;
  static uint8_t const sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  std::fwrite(sig, 1, 8, f);
  std::vector<uint8_t> ih = {uint8_t(w >> 24), uint8_t(w >> 16), uint8_t(w >> 8), uint8_t(w),
                             uint8_t(h >> 24), uint8_t(h >> 16), uint8_t(h >> 8), uint8_t(h), 8, 2, 0, 0, 0};
  png_chunk(f, "IHDR", ih); png_chunk(f, "IDAT", z); png_chunk(f, "IEND", {});
  std::fclose(f);
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int busy = 20, lat_lo = 8, lat_hi = 40, windows = 34, resets = 0, rnum = 0, rden = 1;
  unsigned maxrd = 0, seed = 1, status = 0, status2 = 0;
  uint64_t status2_at = 0;
  std::string png;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--maxrd") maxrd = unsigned(std::atoi(nx().c_str()));
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--windows") windows = std::atoi(nx().c_str());
    else if (a == "--png") png = nx();
    else if (a == "--resets") resets = std::atoi(nx().c_str());
    else if (a == "--status") status = unsigned(std::stoul(nx(), nullptr, 16));
    else if (a == "--status2") { status2 = unsigned(std::stoul(nx(), nullptr, 16)); status2_at = uint64_t(std::atoll(nx().c_str())); }
    else if (a == "--ratio") { rnum = std::atoi(nx().c_str()); rden = std::atoi(nx().c_str()); }
  }
  std::mt19937 rng(seed);
  uint64_t const BASE = 0x06000000, SIZE = 0x400000;
  std::vector<uint64_t> mem(SIZE, 0x0123456789abcdefull);
  uint64_t bad_addr = 0;
  auto at = [&](uint64_t a) -> uint64_t & { static uint64_t sink; if (a < BASE || a >= BASE + SIZE) { bad_addr++; return sink; } return mem[a - BASE]; };
  struct burst { uint64_t due, addr; uint32_t n; };
  std::deque<burst> rq;
  uint32_t wr_left = 0; uint64_t wr_addr = 0, cyc = 0, dcyc = 0;

  auto *t = new Vdp_core;
  auto *rp = t->rootp;
  std::vector<uint32_t> img(496 * 384, 0);
  int cx = 0, cy = 0; bool prev_hb = true, prev_vb = true, seen_vb = false, cap = false, cap_done = false;

  int const UDIV = 8;
  uint64_t proto_bad = 0, last_rst = 0;
  auto proto = [&](char const *m) { if (proto_bad++ < 10) std::printf("%llu: port rule: %s\n", (unsigned long long)dcyc, m); };
  bool held_cmd = false, h_rd = false, h_we = false; uint32_t h_ad = 0, h_bc = 0, wb_ad = 0, wb_bc = 0; uint64_t h_din = 0; uint32_t h_be = 0;
  int u_st = 0, u_cnt = 0, u_bits = 0, u_v = 0; std::string uline; std::vector<std::string> ulines;
  int cur_sel = 0; uint64_t unlock_until = 0; unsigned last_tgl = 0;
  int64_t acc = 0;

  auto dstep_pre = [&]() {
    bool rdy = false; uint64_t d = 0;
    if (!rq.empty() && rq.front().due <= dcyc && (rng() % 100) >= 10) {
      auto &b = rq.front(); rdy = true; d = at(b.addr); b.addr++;
      if (--b.n == 0) rq.pop_front();
    }
    t->DDRAM_BUSY = ((int(rng() % 100) < busy) || (maxrd && rq.size() >= maxrd)) ? 1 : 0;
    t->DDRAM_DOUT_READY = rdy; t->DDRAM_DOUT = d;
  };
  auto dstep_mid = [&]() {
    bool const rd = t->DDRAM_RD, we = t->DDRAM_WE;
    uint32_t const ad = t->DDRAM_ADDR, bc = t->DDRAM_BURSTCNT;
    if (rd && we) proto("RD and WE together");
    if ((rd || we) && (bc == 0 || bc > 128)) proto("burst count out of 1-128");
    if (held_cmd) {
      if (rd != h_rd || we != h_we || ad != h_ad || bc != h_bc) proto("a command changed before it was taken");
      if (we && (t->DDRAM_DIN != h_din || t->DDRAM_BE != h_be) && cyc > last_rst + 64) proto("write data changed before it was taken");
    }
    if (wr_left && (rd || (we && (ad != wb_ad || bc != wb_bc)))) {
      if (proto_bad < 3) std::printf("  rd %d we %d addr %X n %u, burst at %X n %u, %u left; ps %d locked %d\n", rd, we, ad, bc, wb_ad, wb_bc, wr_left,
                                     int(rp->dp_core__DOT__ps), int(t->d_locked));
      proto("a write burst interrupted or its address moved");
    }
    held_cmd = (rd || we) && t->DDRAM_BUSY;
    h_rd = rd; h_we = we; h_ad = ad; h_bc = bc; h_din = t->DDRAM_DIN; h_be = t->DDRAM_BE;
    if (we && !t->DDRAM_BUSY && wr_left == 0) { wb_ad = ad; wb_bc = bc; }
    if (!t->DDRAM_BUSY) {
      if (we) {
        if (wr_left == 0) { wr_addr = ad; wr_left = bc ? bc : 256; }
        uint64_t &w = at(wr_addr);
        for (int b = 0; b < 8; b++) if ((t->DDRAM_BE >> b) & 1) w = (w & ~(0xffull << (8 * b))) | (t->DDRAM_DIN & (0xffull << (8 * b)));
        wr_addr++; wr_left--;
      } else if (rd) {
        uint64_t due = dcyc + uint64_t(lat_lo + int(rng() % unsigned(lat_hi - lat_lo + 1)));
        if (!rq.empty()) due = std::max(due, rq.back().due);
        rq.push_back({due, ad, bc ? bc : 256u});
      }
    }
  };
  // one fixed-clock cycle; the DDR3 clock ticks NUM/DEN as often (at most twice)
  auto step = [&]() {
    if (t->cfg_tgl != last_tgl) { last_tgl = t->cfg_tgl; unlock_until = cyc + 3000; cur_sel = t->clk_sel; }
    t->d_locked = cyc >= unlock_until;
    int num = rnum, den = rden;
    if (rnum == 0) { int const r[4][2] = {{1, 1}, {5, 4}, {17, 20}, {5, 8}}; num = r[cur_sel][0]; den = r[cur_sel][1]; }
    acc += num;
    int ticks = 0;
    while (acc >= den && ticks < 2) { acc -= den; ticks++; }
    for (int k = 0; k < std::max(ticks, 1); k++) {
      bool const dt = k < ticks;
      if (dt) dstep_pre();
      if (k == 0) t->clk = 0;
      t->clk_d = 0; t->eval();
      if (dt) dstep_mid();
      if (k == 0) {
        int const tx = t->uart_tx;
        if (u_st == 0) { if (!tx) { u_st = 1; u_cnt = UDIV + UDIV / 2; u_bits = 0; u_v = 0; } }
        else if (--u_cnt == 0) {
          if (u_bits < 8) { u_v |= tx << u_bits; u_bits++; u_cnt = UDIV; }
          else { u_st = 0; if (u_v == 10) { ulines.push_back(uline); uline.clear(); } else if (u_v != 13) uline.push_back(char(u_v)); }
        }
        if (cap && t->ce_pix) {
          bool hb = t->v_hblank, vb = t->v_vblank;
          if (!seen_vb) { if (vb) seen_vb = true; }
          else {
            if (!vb && !hb) { if (cx < 496 && cy < 384) img[size_t(cy) * 496 + cx] = (uint32_t(t->v_r) << 16) | (uint32_t(t->v_g) << 8) | t->v_b; cx++; }
            if (hb && !prev_hb && !vb) { cx = 0; cy++; }
            if (vb && !prev_vb && cy > 0) { cap = false; cap_done = true; }
          }
          prev_hb = hb; prev_vb = vb;
        }
        t->clk = 1;
      }
      if (dt) { t->clk_d = 1; dcyc++; }
      t->eval();
    }
    cyc++;
  };

  t->por_n = 0; t->rst_n = 0; t->status = status; t->d_locked = 1; t->ddr_ok = 0; for (int i = 0; i < 5; i++) t->dbg_emu[i] = uint32_t(i);
  for (int i = 0; i < 10; i++) step();
  t->por_n = 1; t->rst_n = 1;
  std::vector<uint64_t> rst_at;
  for (int i = 0; i < resets; i++) rst_at.push_back(200000 + rng() % 3000000);
  std::sort(rst_at.begin(), rst_at.end());
  size_t rst_i = 0;
  struct snap { int k; uint64_t bytes, bgb; unsigned lmin, lmed, lmax, ac; };
  std::vector<snap> snaps;
  int fails = 0, nwin = 0, prev_ps = -1;
  while (nwin < windows && cyc < 400000000ull) {
    step();
    if (cyc == 60000) t->ddr_ok = 1;   // the gate opens after a while: debug lines before it
    if (status2_at && cyc == status2_at) { t->status = status2; std::printf("status -> %X at %llu\n", status2, (unsigned long long)cyc); }
    if (rst_i < rst_at.size() && cyc >= rst_at[rst_i]) {
      last_rst = cyc + 40; t->rst_n = 0; for (int i = 0; i < 40; i++) step(); t->rst_n = 1; rst_i++;
      std::printf("core reset at %llu\n", (unsigned long long)cyc);
    }
    int const ps = rp->dp_core__DOT__ps;
    if (ps == 7 /* P_RES */ && prev_ps != 7) {
      snap q; q.k = rp->dp_core__DOT__f_k; q.bytes = rp->dp_core__DOT__f_bytes; q.bgb = rp->dp_core__DOT__f_bgb;
      q.lmin = rp->dp_core__DOT__f_lmin > 999 ? 999 : rp->dp_core__DOT__f_lmin; q.lmed = rp->dp_core__DOT__f_lmed;
      q.lmax = rp->dp_core__DOT__f_lmax; q.ac = rp->dp_core__DOT__f_ac;
      snaps.push_back(q); nwin++;
    }
    prev_ps = ps;
  }
  for (int i = 0; i < 2000000 && rp->dp_core__DOT__rep_busy; i++) step();
  for (int i = 0; i < 20000; i++) step();
  cap = true;
  for (int i = 0; i < 4000000 && !cap_done; i++) step();
  if (!png.empty()) write_png(png, img, 496, 384);

  std::printf("screen:\n");
  for (int r = 0; r < 21; r++) {
    char line[49] = {};
    for (int c = 0; c < 41; c++) line[c] = char(32 + rp->dp_core__DOT__scr[r * 48 + c]);
    std::printf("  %s\n", line);
  }
  // the UART's result lines against the windows' snapshots, in order (the
  // printing lags the snapshot, so a line is matched to the next snapshot)
  int nstart = 0, ndbg = 0, nres = 0, nbad = 0;
  size_t si = 0;
  for (auto const &l : ulines) {
    if (l.rfind("DDR3 PROBE BUILD", 0) == 0) nstart++;
    else if (l.rfind("D T", 0) == 0) ndbg++;
    else if ((l.rfind("C ", 0) == 0 || l.rfind("M ", 0) == 0) && !resets && !status2_at) {
      std::istringstream is(l); std::map<std::string, std::string> kv; std::string c, kk, key, val;
      is >> c;
      if (c == "C") { is >> kk; kv["C"] = kk; }
      while (is >> key >> val) kv[key] = val;
      if (si < snaps.size()) {
        snap const &q = snaps[si++];
        char mbs[32]; std::snprintf(mbs, sizeof mbs, "%03llu.%03llu", (unsigned long long)(q.bytes / 1000000 % 1000), (unsigned long long)(q.bytes / 1000 % 1000));
        char bgm[8]; std::snprintf(bgm, sizeof bgm, "%03llu", (unsigned long long)(q.bgb / 1000000 % 1000));
        bool ok = (c == "M" || std::stoi(kv["C"]) == q.k) && kv["MBS"] == mbs && unsigned(std::stoul(kv["MIN"])) == q.lmin &&
                  unsigned(std::stoul(kv["MED"])) == q.lmed && unsigned(std::stoul(kv["MAX"])) == q.lmax &&
                  unsigned(std::stoul(kv["AC"])) == q.ac && kv["BGM"] == bgm;
        if (!ok) { nbad++; std::printf("UART result line differs: %s (want MBS %s MIN %u MED %u MAX %u AC %u BGM %s)\n", l.c_str(), mbs, q.lmin, q.lmed, q.lmax, q.ac, bgm); }
        nres++;
      }
    }
  }
  std::printf("UART: %zu lines: %d start, %d debug, %d result lines checked, %d differ; drops %u\n", ulines.size(), nstart, ndbg, nres, nbad, unsigned(rp->dp_core__DOT__drops));
  for (size_t i = 0; i < ulines.size() && i < 3; i++) std::printf("  | %s\n", ulines[i].c_str());
  for (size_t i = ulines.size() > 2 ? ulines.size() - 2 : 0; i < ulines.size(); i++) std::printf("  | %s\n", ulines[i].c_str());
  if (nstart < 1 || ndbg == 0 || nbad || (!resets && !status2_at && nres < windows - 1)) { std::printf("FAIL: UART\n"); fails++; }
  if (proto_bad) { std::printf("FAIL: %llu port rule breaks\n", (unsigned long long)proto_bad); fails++; }
  unsigned const errs = rp->dp_core__DOT__f_err;
  if (errs) { std::printf("FAIL: %u pattern errors\n", errs); fails++; }
  if (nwin < windows) { std::printf("FAIL: %d windows of %d\n", nwin, windows); fails++; }
  if (bad_addr) { std::printf("FAIL: %llu accesses outside the model\n", (unsigned long long)bad_addr); fails++; }
  for (auto const &q : snaps) if (q.lmin < unsigned(lat_lo) && q.lmin != 0) { std::printf("FAIL: window %d min latency %u < %d\n", q.k, q.lmin, lat_lo); fails++; break; }
  if (maxrd && !(status & 2) && !(status & 0x8000)) {
    for (auto const &q : snaps) if (q.k == 16 && q.ac != std::min(maxrd, 32u)) { std::printf("FAIL: case 16 held %u, want %u\n", q.ac, std::min(maxrd, 32u)); fails++; break; }
  }
  std::printf("%llu fixed cycles, %llu DDR3 cycles, %d windows, %u errors\n%s\n", (unsigned long long)cyc, (unsigned long long)dcyc, nwin, errs, fails ? "FAIL" : "PASS");
  delete t;
  return fails ? 1 : 0;
}
