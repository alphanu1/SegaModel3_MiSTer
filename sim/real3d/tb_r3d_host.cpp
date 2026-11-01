// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// D-37 stage B, the Real3D host side (r3d_cmd, r3d_mem, r3d_texup) on the
// real m2_sdram and m3_ddr3p, driven by Supermodel's bus trace of VF3
// (sm3-builds/sys/sm_bus.bin, sim/system/bustrace.py's records) and checked
// against Supermodel's Real3D memories at the same frames
// (tools/mame-census/supermodel-stageb-dump.patch: r3d_<frame>.bin, 8C 4 MiB,
// 8E 1 MiB, polygon 4 MiB, texture 8 MiB, as frame <frame> is drawn).
//
// The trace's Real3D accesses (84 reads, 88-9C writes), its tile register 08
// and 0C writes and its events (frame start = line 384, IRQ 0x0D = line 0,
// the flip = its line) go in order. The geometry and the rasteriser are
// stand-ins: busy for a random time. Checked:
//   - every status read against the trace's value;
//   - at each listed frame's draw (its bank swap): bank R of 8E and polygon
//     RAM against the dump (Supermodel draws memory as of its flip);
//   - at that frame's f_end: the texture sheets against the dump;
//   - at the frame's end: 8C against the dump, except words 0-0x1FFFF, where
//     Supermodel keeps its update buffers (this design has none).
//
//   Vr3d_host_tb --trace F --sdram IMG --dump DIR --list A,B,... [--to N]
//       [--busy PCT] [--lat LO HI] [--geo LO HI] [--rast LO HI] [--fmin C]
//       [--load PCT] [--seed S] [--sdlat L] [--quiet]
#include "Vr3d_host_tb.h"
#include "verilated.h"
#include "Vr3d_host_tb___024root.h"
#include "../system/sdram_dev.h"
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <set>
#include <string>
#include <vector>

#pragma pack(push, 1)
struct rec { uint32_t frame, pc, addr; uint8_t kind, size, master, pad; uint64_t data; };
#pragma pack(pop)

static uint32_t bswap(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24); }

struct ddr_dev {
  static constexpr uint64_t SIZE = 0xD00000;
  std::vector<uint64_t> m = std::vector<uint64_t>(SIZE);
  int busy_pct = 20, lat_lo = 8, lat_hi = 40;
  uint32_t wr_left = 0; uint64_t wr_addr = 0;
  struct burst { uint64_t at, addr; uint32_t n; };
  std::deque<burst> rq;
  uint64_t bad = 0, rd_beats = 0, wr_beats = 0;
  uint64_t &at(uint64_t a) { static uint64_t s; if (a >= SIZE) { if (!bad) std::printf("DDR3 address %llx out of range\n", (unsigned long long)a); bad++; return s; } return m[a]; }
};

// Supermodel 036af1c Real3D.cpp StoreTexture and UploadTexture, transcribed
// for the synthetic uploads (textureRAM 2048 x 2048, page 1 from row 1024).
namespace smtex {
static const int mipXBase[] = { 0, 1024, 1536, 1792, 1920, 1984, 2016, 2032, 2040, 2044, 2046, 2047 };
static const int mipYBase[] = { 0, 512, 768, 896, 960, 992, 1008, 1016, 1020, 1022, 1023 };
static const int mipDivisor[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
static const unsigned decode8x8[64] = { 1, 0, 5, 4, 9, 8,13,12, 3, 2, 7, 6,11,10,15,14, 17,16,21,20,25,24,29,28, 19,18,23,22,27,26,31,30,
  33,32,37,36,41,40,45,44, 35,34,39,38,43,42,47,46, 49,48,53,52,57,56,61,60, 51,50,55,54,59,58,63,62 };
static const unsigned decode8x4[32] = { 1, 0, 5, 4, 3, 2, 7, 6, 9, 8,13,12, 11,10,15,14, 17,16,21,20, 19,18,23,22, 25,24,29,28, 27,26,31,30 };
static const unsigned decode8x2[16] = { 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14 };
static void store(uint16_t *ram, unsigned xPos, unsigned yPos, unsigned width, unsigned height, const uint16_t *texData,
                  bool sixteenBit, bool writeLSB, bool writeMSB, uint32_t &texDataOffset) {
  const uint32_t tileX = std::min(8u, width), tileY = std::min(8u, height);
  const unsigned *decode = (tileX == 8) ? decode8x8 : (tileX == 4) ? decode8x4 : decode8x2;
  texDataOffset = 0;
  if (sixteenBit) {
    for (uint32_t y = yPos; y < yPos + height; y += tileY)
      for (uint32_t x = xPos; x < xPos + width; x += tileX) {
        uint32_t destOffset = y * 2048 + x;
        for (uint32_t yy = 0; yy < tileY; yy++) {
          for (uint32_t xx = 0; xx < tileX; xx++) { ram[destOffset++] = texData[decode[yy * tileX + xx]]; texDataOffset++; }
          destOffset += 2048 - tileX;
        }
        texData += tileY * tileX;
      }
  } else {
    const uint8_t byteSelect = uint8_t(writeLSB) | uint8_t(writeMSB << 1);
    static const uint16_t byteMask[4] = { 0xFFFF, 0xFF00, 0x00FF, 0x0000 };
    const uint32_t offset = std::max(1u, (tileY * tileX) / 2);
    for (uint32_t y = yPos; y < yPos + height; y += tileY)
      for (uint32_t x = xPos; x < xPos + width; x += tileX) {
        uint32_t destOffset = y * 2048 + x;
        for (uint32_t yy = 0; yy < tileY; yy++) {
          for (uint32_t xx = 0; xx < tileX; xx++) {
            if (writeLSB | writeMSB) {
              ram[destOffset] &= byteMask[byteSelect];
              const uint8_t shift = uint8_t(8 * ((xx & 1) ^ 1));
              const uint8_t index = uint8_t((yy ^ 1) * tileX + (xx ^ 1) - (tileX & 1));
              uint16_t tempData = uint16_t((texData[decode[index] / 2] >> shift) & 0xFF);
              tempData = uint16_t(tempData | (tempData << 8));
              tempData &= uint16_t(byteMask[byteSelect] ^ 0xFFFF);
              ram[destOffset] |= tempData;
            }
            destOffset++;
          }
          destOffset += 2048 - tileX;
        }
        texData += offset; texDataOffset += offset;
      }
  }
}
// returns the halfwords consumed
static uint32_t upload(uint16_t *ram, uint32_t header, const uint16_t *texData) {
  uint32_t x = 32 * (header & 0x3F), y = 32 * ((header >> 7) & 0x1F), page = (header >> 20) & 1;
  uint32_t width = 32u << ((header >> 14) & 7), height = 32u << ((header >> 17) & 7), type = (header >> 24) & 0xFF;
  bool sixteenBit = (header >> 23) & 1, writeUpperByte = (header >> 22) & 1, writeLowerByte = (header >> 21) & 1;
  uint32_t offset = 0, used = 0;
  switch (type) {
  case 0x00: case 0x01:
    store(ram, x, y + page * 1024, width, height, texData, sixteenBit, writeLowerByte, writeUpperByte, offset);
    texData += offset; used += offset;
    if (type == 0x01) break;
    [[fallthrough]];
  case 0x02:
    for (int i = 1; width >= 4 && height >= 4; i++) {
      int xPos = mipXBase[i] + int(x / unsigned(mipDivisor[i])), yPos = mipYBase[i] + int(y / unsigned(mipDivisor[i]));
      width /= 2; height /= 2;
      store(ram, unsigned(xPos), unsigned(yPos) + page * 1024, width, height, texData, sixteenBit, writeLowerByte, writeUpperByte, offset);
      texData += offset; used += offset;
    }
    break;
  default: break;
  }
  return used;
}
}

static std::vector<uint8_t> read_file(std::string const &fn, size_t off = 0, size_t n = 0) {
  std::vector<uint8_t> d;
  std::FILE *f = std::fopen(fn.c_str(), "rb");
  if (!f) return d;
  if (!n) { std::fseek(f, 0, SEEK_END); n = size_t(std::ftell(f)) - off; }
  std::fseek(f, long(off), SEEK_SET);
  d.resize(n);
  d.resize(std::fread(d.data(), 1, n, f));
  std::fclose(f);
  return d;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string trace_fn, sdram_fn, dump_dir;
  std::set<uint32_t> list;
  uint32_t to = 1500;
  int busy = 20, lat_lo = 8, lat_hi = 40, load = 10, sdlat = 2;
  uint64_t geo_lo = 20000, geo_hi = 400000, rast_lo = 20000, rast_hi = 300000, fmin = 200000;
  unsigned seed = 1;
  bool quiet = false;
  unsigned min_chk = 0;
  int synth_tex = 0;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return std::string(argv[++i]); };
    if (a == "--trace") trace_fn = nx();
    else if (a == "--sdram") sdram_fn = nx();
    else if (a == "--dump") dump_dir = nx();
    else if (a == "--list") { std::string l = nx(); for (char *p = std::strtok(l.data(), ","); p; p = std::strtok(nullptr, ",")) list.insert(uint32_t(std::atoi(p))); }
    else if (a == "--to") to = uint32_t(std::atoi(nx().c_str()));
    else if (a == "--busy") busy = std::atoi(nx().c_str());
    else if (a == "--lat") { lat_lo = std::atoi(nx().c_str()); lat_hi = std::atoi(nx().c_str()); }
    else if (a == "--geo") { geo_lo = std::strtoull(nx().c_str(), nullptr, 0); geo_hi = std::strtoull(nx().c_str(), nullptr, 0); }
    else if (a == "--rast") { rast_lo = std::strtoull(nx().c_str(), nullptr, 0); rast_hi = std::strtoull(nx().c_str(), nullptr, 0); }
    else if (a == "--fmin") fmin = std::strtoull(nx().c_str(), nullptr, 0);
    else if (a == "--load") load = std::atoi(nx().c_str());
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
    else if (a == "--sdlat") sdlat = std::atoi(nx().c_str());
    else if (a == "--quiet") quiet = true;
    else if (a == "--minchk") min_chk = unsigned(std::atoi(nx().c_str()));
    else if (a == "--synth-tex") synth_tex = std::atoi(nx().c_str());
  }
  std::vector<uint8_t> tr = read_file(trace_fn);
  size_t const nrec = tr.size() / sizeof(rec);
  rec const *R = reinterpret_cast<rec const *>(tr.data());
  if (!nrec) { std::printf("no trace\n"); return 2; }

  std::mt19937 rng(seed);
  sdram_dev sd; ddr_dev dd;
  sd.lat = sdlat; dd.busy_pct = busy; dd.lat_lo = lat_lo; dd.lat_hi = lat_hi;
  {   // VROM in the MRA's layout, 0x4000000-0x5FFFFFF
    std::vector<uint8_t> v = read_file(sdram_fn, 0x4000000, 0x2000000);
    if (v.size() != 0x2000000) { std::printf("no VROM in %s\n", sdram_fn.c_str()); return 2; }
    std::memcpy(&sd.m[0x4000000 >> 1], v.data(), v.size());
  }

  auto *t = new Vr3d_host_tb;
  uint64_t cyc = 0;
  bool ddr_rdy = false; uint64_t ddr_d = 0;
  // stand-ins
  uint64_t g_left = 0, r_left = 0;
  int banks = 0;                    // store banks in use
  std::deque<int> rq_frames;        // frames ended, waiting to replay (count)
  bool x_on = false; uint64_t xs = 0;
  // checks
  uint64_t st_reads = 0, st_bad = 0, fails = 0, n_mchk = 0, n_tchk = 0, n_cchk = 0, n_gchk = 0;
  uint32_t cur_frame = 0, swap_frame = 0;
  bool chk_tex_pend = false; uint32_t tex_frame = 0;
  std::set<uint32_t> drawn;
  uint64_t frame_start_cyc = 0;

  uint16_t const *synth_ref = nullptr;
  bool synth_armed = false;         // the bench's own flip has begun a draw
  std::vector<uint8_t> chk_dump; uint32_t chk_frame = 0; int chk_bank = 0; bool chk_geo = false;
  uint64_t n_rchk = 0;
  auto cmp_tex = [&](uint16_t const *ref, char const *what, uint32_t f) {
    uint64_t bad = 0, first = ~0ull, nz = 0;
    for (uint32_t y = 0; y < 2048; y++)
      for (uint32_t x = 0; x < 2048; x++) {
        uint16_t const r = ref[size_t(y) * 2048 + x];
        uint64_t const q = dd.at(0xC00000 + ((uint64_t(y >> 2) << 11) | (uint64_t(x >> 2) << 2) | (y & 3)));
        uint16_t const got = uint16_t(q >> (16 * (x & 3)));
        if (r) nz++;
        if (got != r) { if (!bad) first = (uint64_t(y) << 16) | x; bad++; }
      }
    std::printf("f%u %s: %" PRIu64 " texels differ (%" PRIu64 " non-zero)%s\n", f, what, bad, nz, bad ? " FAIL" : "");
    if (bad) { std::printf("   first at x %u y %u\n", unsigned(first & 0xffff), unsigned(first >> 16)); fails++; }
  };
  auto dump_tex = [&](uint32_t f) {
    char b[16]; std::snprintf(b, 16, "%06u", f);
    std::vector<uint8_t> d = read_file(dump_dir + "/r3d_" + b + ".bin", 0x900000, 0x800000);
    d.resize(0x800000);
    return d;
  };
  std::deque<uint32_t> r_frames;   // the frames ended, in order (0: not listed)
  uint32_t r_cur = 0;
  std::vector<uint32_t> r_snap;     // bank R's polygon RAM at the swap
  auto snap_r = [&]() { r_snap.resize(0x100000); for (uint32_t a = 0; a < 0x100000; a++) r_snap[a] = uint32_t(dd.at((chk_bank ? 0x180000u : 0x100000u) + (a >> 1)) >> (32 * (a & 1))); };
  uint64_t n_writes_geo = 0;        // writes to 8E / 98 during a checked frame's geometry
  auto cmp_bank = [&](char const *when) {
    std::vector<uint8_t> const &d = chk_dump;
    uint64_t bad8e = 0, badp = 0;
    for (uint32_t a = 0; a < 0x40000; a++) {
      uint32_t const ref = d[0x400000 + 4 * a] | (d[0x400001 + 4 * a] << 8) | (d[0x400002 + 4 * a] << 16) | (uint32_t(d[0x400003 + 4 * a]) << 24);
      uint32_t const sb = (chk_bank ? 0x7200000u : 0x7100000u) + 4 * a;
      if ((sd.m[sb >> 1] | (uint32_t(sd.m[(sb >> 1) + 1]) << 16)) != ref) bad8e++;
    }
    for (uint32_t a = 0; a < 0x100000; a++) {
      uint32_t const ref = d[0x500000 + 4 * a] | (d[0x500001 + 4 * a] << 8) | (d[0x500002 + 4 * a] << 16) | (uint32_t(d[0x500003 + 4 * a]) << 24);
      uint64_t const q = dd.at((chk_bank ? 0x180000u : 0x100000u) + (a >> 1));
      if (uint32_t(q >> (32 * (a & 1))) != ref) badp++;
    }
    std::printf("f%u %s (bank %d): 8E %" PRIu64 " words differ, polygon RAM %" PRIu64 "%s\n", chk_frame, when, chk_bank, bad8e, badp, (bad8e || badp) ? " FAIL" : "");
    if (bad8e || badp) fails++;
  };
  auto step = [&]() {
    t->DDRAM_BUSY = int(rng() % 100) < dd.busy_pct;
    ddr_rdy = false;
    if (!dd.rq.empty() && dd.rq.front().at <= cyc && (rng() % 100) >= 10) {
      auto &b = dd.rq.front();
      ddr_rdy = true; ddr_d = dd.at(b.addr); b.addr++; dd.rd_beats++;
      if (--b.n == 0) dd.rq.pop_front();
    }
    t->DDRAM_DOUT_READY = ddr_rdy; t->DDRAM_DOUT = ddr_d;
    t->sd_dq_i = sd.dq(cyc);
    // stand-ins' outputs
    t->g_busy = g_left > 0;
    t->f_ready = banks < 2;
    t->r_busy = r_left > 0;
    t->r_done = 0;
    if (r_left == 1) t->r_done = 1;
    // the load generator
    if (!x_on && int(rng() % 1000) < load) { x_on = true; t->x_addr = rng() % 0x100000; t->x_blen = 1 + rng() % 16; }
    t->x_req = x_on;
    t->clk = 0; t->eval();
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
    if (!t->DDRAM_BUSY && t->mem_rst_n) {
      if ((t->DDRAM_WE || t->DDRAM_RD) && t->DDRAM_ADDR >= ddr_dev::SIZE && !dd.bad)
        std::printf("bad DDR3 command at cycle %llu frame %u: we %d n %u arb owner %u tex own %u tex state %u\n", (unsigned long long)cyc, cur_frame, t->DDRAM_WE, t->DDRAM_BURSTCNT,
                    t->rootp->r3d_host_tb__DOT__u_arb__DOT__owner, t->rootp->r3d_host_tb__DOT__u_tex__DOT__own, unsigned(t->rootp->r3d_host_tb__DOT__u_tex__DOT__us));
      if (t->DDRAM_WE) {
        if (dd.wr_left == 0) { dd.wr_addr = t->DDRAM_ADDR; dd.wr_left = t->DDRAM_BURSTCNT ? t->DDRAM_BURSTCNT : 256; }
        uint64_t &w = dd.at(dd.wr_addr);
        uint64_t const din = t->DDRAM_DIN;
        for (int b = 0; b < 8; b++) if ((t->DDRAM_BE >> b) & 1) w = (w & ~(0xffull << (8 * b))) | (din & (0xffull << (8 * b)));
        dd.wr_addr++; dd.wr_left--; dd.wr_beats++;
      } else if (t->DDRAM_RD) {
        uint64_t at = cyc + uint64_t(dd.lat_lo + int(rng() % unsigned(dd.lat_hi - dd.lat_lo + 1)));
        if (!dd.rq.empty()) at = std::max(at, dd.rq.back().at);
        dd.rq.push_back({at, t->DDRAM_ADDR, t->DDRAM_BURSTCNT ? uint32_t(t->DDRAM_BURSTCNT) : 256u});
      }
    }
    bool const live = !t->rst;
    bool const g_start = live && t->g_start, f_begin = live && t->f_begin, f_end = live && t->f_end, m_swap = live && t->m_swap, x_ack = t->x_ack;
    t->clk = 1; t->eval();
    cyc++;
    // stand-ins' state
    if (x_ack) x_on = false;
    if (g_left) { if (--g_left == 0 && chk_geo) {
      chk_geo = false; cmp_bank("geometry end"); n_gchk++;
      uint64_t ch = 0; for (uint32_t a = 0; a < 0x100000; a++) if (uint32_t(dd.at((chk_bank ? 0x180000u : 0x100000u) + (a >> 1)) >> (32 * (a & 1))) != r_snap[a]) ch++;
      if (std::getenv("SB_RCH")) std::printf("   f%u: bank R polygon words changed during the geometry: %llu (8E/98 writes then %llu)\n", chk_frame, (unsigned long long)ch, (unsigned long long)n_writes_geo);
    } }
    if (g_start) g_left = geo_lo + rng() % (geo_hi - geo_lo + 1);
    if (f_begin) banks++;
    if (f_end) { rq_frames.push_back(1); r_frames.push_back(chk_tex_pend ? tex_frame : 0); }
    if (r_left) {
      if (--r_left == 0) {
        banks--;
        // the replay's end: the sheets must still be its frame's
        if (r_cur) { std::vector<uint8_t> d = dump_tex(r_cur); cmp_tex(reinterpret_cast<uint16_t const *>(d.data()), "textures at the replay's end", r_cur); n_rchk++; }
      }
    }
    if (!r_left && !rq_frames.empty()) { rq_frames.pop_front(); r_cur = r_frames.front(); r_frames.pop_front(); r_left = rast_lo + rng() % (rast_hi - rast_lo + 1); }
    // checks at the swap and the frame's end
    if (m_swap) {
      if (std::getenv("SB_SWAPS")) std::printf("swap at f%u cycle %llu\n", cur_frame, (unsigned long long)cyc);
      swap_frame = cur_frame;
      drawn.insert(cur_frame);
      if (list.count(cur_frame)) {
        std::vector<uint8_t> d = read_file(dump_dir + "/r3d_" + [&] { char b[16]; std::snprintf(b, 16, "%06u", cur_frame); return std::string(b); }() + ".bin");
        if (d.size() != 0x1100000) { std::printf("f%u: no dump\n", cur_frame); fails++; }
        else {
          int const bk = t->bank;        // the swap has just taken effect: R
          uint64_t bad8e = 0, badp = 0, first8e = ~0ull, firstp = ~0ull;
          for (uint32_t a = 0; a < 0x40000; a++) {
            uint32_t const ref = d[0x400000 + 4 * a] | (d[0x400001 + 4 * a] << 8) | (d[0x400002 + 4 * a] << 16) | (uint32_t(d[0x400003 + 4 * a]) << 24);
            uint32_t const sb = (bk ? 0x7200000u : 0x7100000u) + 4 * a;
            uint32_t const got = sd.m[sb >> 1] | (uint32_t(sd.m[(sb >> 1) + 1]) << 16);
            if (got != ref) { if (!bad8e) first8e = a; bad8e++; }
          }
          for (uint32_t a = 0; a < 0x100000; a++) {
            uint32_t const ref = d[0x500000 + 4 * a] | (d[0x500001 + 4 * a] << 8) | (d[0x500002 + 4 * a] << 16) | (uint32_t(d[0x500003 + 4 * a]) << 24);
            uint64_t const q = dd.at((bk ? 0x180000u : 0x100000u) + (a >> 1));
            uint32_t const got = uint32_t(q >> (32 * (a & 1)));
            if (got != ref) { if (!badp) firstp = a; badp++; }
          }
          n_mchk++;
          std::printf("f%u draw (bank %d): 8E %" PRIu64 " words differ, polygon RAM %" PRIu64 "%s\n", cur_frame, bk, bad8e, badp,
                      (bad8e || badp) ? " FAIL" : "");
          if (bad8e) std::printf("   8E first at word %06" PRIx64 "\n", first8e);
          if (badp) std::printf("   polygon first at word %06" PRIx64 "\n", firstp);
          if (bad8e || badp) fails++;
          chk_tex_pend = true; tex_frame = cur_frame;
          chk_dump = d; chk_frame = cur_frame; chk_bank = bk; chk_geo = true;
          snap_r();
        }
      }
    }
    if (f_end && chk_tex_pend && !synth_ref) {
      chk_tex_pend = false;
      std::vector<uint8_t> d = dump_tex(tex_frame);
      n_tchk++;
      cmp_tex(reinterpret_cast<uint16_t const *>(d.data()), "textures", tex_frame);
    }
    if (f_end && synth_ref && synth_armed) { synth_armed = false; cmp_tex(synth_ref, "synthetic textures", cur_frame); n_tchk++; }
  };

  // reset
  t->mem_rst_n = 0; t->rst = 1; t->rd_lat_sel = 2;
  for (int i = 0; i < 8; i++) step();
  t->mem_rst_n = 1;
  for (int i = 0; i < 20000 && !t->sd_ready; i++) step();
  t->rst = 0;
  for (int i = 0; i < 16; i++) step();

  auto wait_rdy = [&]() { uint64_t n = 0; while (!t->h_rdy) { step(); if (++n > 50000000) { std::printf("host stuck at frame %u\n", cur_frame); std::exit(1); } } };
  auto drain = [&]() { uint64_t n = 0; while (!t->mem_idle && n < 20000000) { step(); n++; } };
  // 8C's writes landed: the host empty and r3d_mem not inside a write (its
  // states 1-3). Not the full drain, which waits for the geometry and the
  // copies: that would keep a listed frame's geometry from ever overlapping
  // the next frame's writes, and hide the deferral.
  auto drain8c = [&]() { uint64_t n = 0; while ((!t->h_rdy || (t->rootp->r3d_host_tb__DOT__u_mem__DOT__st >= 1 && t->rootp->r3d_host_tb__DOT__u_mem__DOT__st <= 3)) && n < 20000000) { step(); n++; } };

  for (size_t i = 0; i < nrec; i++) {
    rec const &r = R[i];
    if (r.frame > to) break;
    if (r.kind == 2) {
      if (r.size == 3) {
        // the frame's end: 8C of a listed frame
        if (cur_frame && list.count(cur_frame)) {
          drain8c();
          std::vector<uint8_t> d = read_file(dump_dir + "/r3d_" + [&] { char b[16]; std::snprintf(b, 16, "%06u", cur_frame); return std::string(b); }() + ".bin", 0, 0x400000);
          uint64_t bad = 0, upd = 0, first = ~0ull;
          for (uint32_t a = 0; a < 0x100000 && d.size() == 0x400000; a++) {
            uint32_t const ref = d[4 * a] | (d[4 * a + 1] << 8) | (d[4 * a + 2] << 16) | (uint32_t(d[4 * a + 3]) << 24);
            uint32_t const sb = 0x6D00000u + 4 * a;
            uint32_t const got = sd.m[sb >> 1] | (uint32_t(sd.m[(sb >> 1) + 1]) << 16);
            if (got != ref) { if (a < 0x20000) upd++; else { if (!bad) first = a; bad++; } }
          }
          n_cchk++;
          std::printf("f%u 8C: %" PRIu64 " words differ, %" PRIu64 " in Supermodel's update buffers%s%s\n", cur_frame, bad, upd,
                      drawn.count(cur_frame) ? "" : " (not drawn)", bad ? " FAIL" : "");
          if (bad) { std::printf("   first at word %06" PRIx64 "\n", first); fails++; }
        }
        while (cyc - frame_start_cyc < fmin) step();
        frame_start_cyc = cyc;
        cur_frame = r.frame;
        t->ln_start = 1; t->ln = 384; step(); t->ln_start = 0;
        if (std::getenv("SB_DBG") && cur_frame >= unsigned(std::atoi(std::getenv("SB_DBG"))) && cur_frame < unsigned(std::atoi(std::getenv("SB_DBG"))) + 12) {
          auto *p = t->rootp;
          std::printf("f%u g_run %d g_seen %d e_wait %d x_need %d x_run %d inf_n %d sw_wait %d | tex us %u f_on %d wp %x fp %x rp %x have %d fbp %x dp %x | mem st %u | banks %d r_left %llu g_left %llu\n", cur_frame,
            p->r3d_host_tb__DOT__u_cmd__DOT__g_run, p->r3d_host_tb__DOT__u_cmd__DOT__g_seen, p->r3d_host_tb__DOT__u_cmd__DOT__e_wait, p->r3d_host_tb__DOT__u_cmd__DOT__x_need,
            p->r3d_host_tb__DOT__u_cmd__DOT__x_run, p->r3d_host_tb__DOT__u_cmd__DOT__inf_n, p->r3d_host_tb__DOT__u_cmd__DOT__sw_wait,
            unsigned(p->r3d_host_tb__DOT__u_tex__DOT__us), p->r3d_host_tb__DOT__u_tex__DOT__f_on, p->r3d_host_tb__DOT__u_tex__DOT__wp, p->r3d_host_tb__DOT__u_tex__DOT__fp,
            p->r3d_host_tb__DOT__u_tex__DOT__rp, int(int16_t(p->r3d_host_tb__DOT__u_tex__DOT__have << 5) >> 5), p->r3d_host_tb__DOT__u_tex__DOT__fbp, p->r3d_host_tb__DOT__u_tex__DOT__dp,
            unsigned(p->r3d_host_tb__DOT__u_mem__DOT__st), banks, (unsigned long long)r_left, (unsigned long long)g_left);
        }
        if (!quiet && cur_frame % 100 == 0) std::printf("frame %u, cycle %" PRIu64 ", drawn %u, dropped %u\n", cur_frame, cyc, unsigned(t->n_draw), unsigned(t->n_drop));
      } else if (r.size == 2) {
        t->ln_start = 1; t->ln = uint16_t(r.addr); step(); t->ln_start = 0;
      }
      continue;
    }
    uint32_t const hi = r.addr >> 24;
    if (r.addr == 0xF1180008u || r.addr == 0xF118000Cu) {
      if (r.kind == 1) { t->tg_reg_we = 1; t->tg_reg_addr = (r.addr >> 2) & 63; t->tg_reg_wdata = bswap(uint32_t(r.data)); step(); t->tg_reg_we = 0; }
      continue;
    }
    if (hi == 0x84 && r.kind == 0) {
      wait_rdy();
      t->h_go = 1; t->h_we = 0; t->h_addr = r.addr; step(); t->h_go = 0;
      if (!t->h_rv) { std::printf("no read answer\n"); return 1; }
      st_reads++;
      if (t->h_rdata != uint32_t(r.data)) {
        if (st_bad < 10) std::printf("f%u status %08X: %08X, trace %08X\n", r.frame, r.addr, unsigned(t->h_rdata), unsigned(r.data));
        st_bad++;
      }
      continue;
    }
    if (r.kind == 1 && (hi == 0x88 || hi == 0x8C || hi == 0x8E || hi == 0x90 || hi == 0x94 || hi == 0x98 || hi == 0x9C)) {
      wait_rdy();
      t->h_go = 1; t->h_we = 1; t->h_addr = r.addr; t->h_wdata = bswap(uint32_t(r.data)); step(); t->h_go = 0;
      if (chk_geo && (hi == 0x8E || hi == 0x98)) n_writes_geo++;
    }
  }
  drain();
  if (synth_tex) {
    // the sheets as they are now, then random uploads of every type, both
    // depths and the byte selects, one to four a flush; each flush drawn
    // nothing in flight: no frame drawing or waiting to replay
    for (uint64_t i = 0; i < 50000000 && (g_left || r_left || !rq_frames.empty() || banks || t->t_busy || !t->mem_idle); i++) step();
    for (int i = 0; i < 64; i++) step();
    std::vector<uint16_t> ref(size_t(2048) * 2048);
    for (uint32_t y = 0; y < 2048; y++)
      for (uint32_t x = 0; x < 2048; x++)
        ref[size_t(y) * 2048 + x] = uint16_t(dd.at(0xC00000 + ((uint64_t(y >> 2) << 11) | (uint64_t(x >> 2) << 2) | (y & 3))) >> (16 * (x & 3)));
    synth_ref = ref.data();
    t->tg_reg_we = 1; t->tg_reg_addr = 2; t->tg_reg_wdata = 100; step(); t->tg_reg_we = 0;   // flip at line 100
    auto host_w = [&](uint32_t a, uint32_t v) { wait_rdy(); t->h_go = 1; t->h_we = 1; t->h_addr = a; t->h_wdata = v; step(); t->h_go = 0; };
    int done = 0, flush = 0;
    while (done < synth_tex) {
      int const n = 1 + int(rng() % 4);
      for (int k = 0; k < n && done < synth_tex; k++, done++) {
        uint32_t const type = (rng() % 8 == 0) ? 0x80u : rng() % 3;
        uint32_t const wl = rng() % 4, hl = rng() % 4;         // 32 to 256
        uint32_t const w = 32u << wl, h = 32u << hl;
        uint32_t const x = (rng() % ((2048 - w) / 32 + 1)), y = (rng() % ((1024 - h) / 32 + 1));
        uint32_t const b16 = rng() & 1, sel = rng() % 4;
        uint32_t const hdr = (type << 24) | (b16 << 23) | (sel << 21) | ((rng() & 1) << 20) | (hl << 17) | (wl << 14) | (y << 7) | x;
        // the halfwords the upload consumes, from a dry run
        std::vector<uint16_t> data(size_t(w) * h * 2 + 64), scratch(size_t(2048) * 2048);
        for (auto &d : data) d = uint16_t(rng());
        uint32_t const used = (type == 0x80) ? 2 * (rng() % 64) : smtex::upload(scratch.data(), hdr, data.data());
        uint32_t const words = 2 + (used + 1) / 2;
        smtex::upload(ref.data(), hdr, data.data());
        host_w(0x94000000u, 8 * words - 4);                    // (2 + s / 2) / 4 = words
        host_w(0x94000000u, hdr);
        for (uint32_t i = 0; i + 2 < words; i++) host_w(0x94000000u, uint32_t(data[2 * i]) | (uint32_t(data[2 * i + 1]) << 16));
      }
      host_w(0x88000000u, 0);
      t->tg_reg_we = 1; t->tg_reg_addr = 3; t->tg_reg_wdata = 0; step(); t->tg_reg_we = 0;
      uint64_t const tc = n_tchk;
      wait_rdy();
      synth_armed = true;
      t->ln_start = 1; t->ln = 100; step(); t->ln_start = 0;
      for (uint64_t i = 0; i < 40000000 && n_tchk == tc; i++) step();
      if (n_tchk == tc) {
        auto *q = t->rootp;
        std::printf("synthetic flush %d not drawn: g_run %d e_wait %d x_need %d x_run %d inf_n %d sw_wait %d draws %u | tex us %u busy %d have %d dp %x fp %x fp_m %x rp %x | banks %d\n", flush,
          q->r3d_host_tb__DOT__u_cmd__DOT__g_run, q->r3d_host_tb__DOT__u_cmd__DOT__e_wait, q->r3d_host_tb__DOT__u_cmd__DOT__x_need, q->r3d_host_tb__DOT__u_cmd__DOT__x_run,
          q->r3d_host_tb__DOT__u_cmd__DOT__inf_n, q->r3d_host_tb__DOT__u_cmd__DOT__sw_wait, unsigned(t->n_draw), unsigned(q->r3d_host_tb__DOT__u_tex__DOT__us), t->t_busy,
          int(int16_t(q->r3d_host_tb__DOT__u_tex__DOT__have << 5) >> 5), q->r3d_host_tb__DOT__u_tex__DOT__dp, q->r3d_host_tb__DOT__u_tex__DOT__fp, q->r3d_host_tb__DOT__u_tex__DOT__fp_m, q->r3d_host_tb__DOT__u_tex__DOT__rp, banks);
        fails++; break;
      }
      flush++;
      t->ln_start = 1; t->ln = 384; step(); t->ln_start = 0;
    }
    std::printf("synthetic: %d textures in %d flushes\n", done, flush);
  }
  if (st_bad) fails++;
  std::printf("status reads %" PRIu64 ", %" PRIu64 " differ; drawn %u, dropped %u; DDR3 %" PRIu64 " read beats, %" PRIu64 " written, %" PRIu64 " out of range; SDRAM %" PRIu64 " reads, %" PRIu64 " writes; %" PRIu64 " cycles\n",
              st_reads, st_bad, unsigned(t->n_draw), unsigned(t->n_drop), dd.rd_beats, dd.wr_beats, dd.bad, sd.reads, sd.writes, cyc);
  if (dd.bad) fails++;
  std::printf("r3d_mem: %u runs deferred, %u copy transfers\n", unsigned(t->rootp->r3d_host_tb__DOT__u_mem__DOT__n_runs), unsigned(t->rootp->r3d_host_tb__DOT__u_mem__DOT__n_copy));
  std::printf("checked: %" PRIu64 " draws (8E, polygon RAM), %" PRIu64 " again at the geometry's end, %" PRIu64 " texture sheets, %" PRIu64 " again at the replay's end, %" PRIu64 " 8C\n", n_mchk, n_gchk, n_tchk, n_rchk, n_cchk);
  if (n_mchk < min_chk || n_tchk < min_chk) { std::printf("fewer than %u frames checked\n", min_chk); fails++; }
  std::printf("%s\n", fails ? "FAIL" : "PASS");
  delete t;
  return fails ? 1 : 0;
}
