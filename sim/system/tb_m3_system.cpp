// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// The stage-A system (rtl/sys/m3_system.sv) whole: the 603e, the bus, the
// 53C810, the 315-5649 and EEPROM, the JTAG chain, the tile generator and the
// video, the loader, m2_sdram and the SDRAM at its pins (sdram_dev.h).
//
//   Vm3_system --image IMG [--frames N] [--trace FILE] [--dump DIR] [--dump-at F,F..]
//              [--ppm DIR] [--ppm-at F,F..] [--window BYTES] [--quiet] [--tb-fast]
//              [--save FILE] [--restore FILE] [--r3d DIR --r3d-at F,F..]
//              [--ddr-busy PCT] [--ddr-lat LO HI]
//
// --save writes a checkpoint when the run ends (the model, built --savable,
// and the bench's own state: the SDRAM, the tile state, the cycle count);
// --restore starts from one, so a long boot is run once, in segments of
// minutes, and the screens are reached from the last segment.
//
// IMG is the SDRAM image the MRA describes (sim/system/m3_mra.py pack). The
// first --window bytes of it (default 64 KiB) go through ioctl and the loader
// and are checked in the SDRAM; the rest is put straight into the model (a
// 98 MiB ioctl stream is hours of simulation), and the end of the download
// releases the game, as on the board.
//
// --trace writes every bus access outside RAM and CROM reads in Supermodel's
// trace format (sim/system/bustrace.py). --dump writes the tile generator's
// state at the start of the active display of each --dump-at frame as
// tg_tool reads it (fNNNNNN.vram: VRAM from the SDRAM and the palette as
// written, CPU byte order; fNNNNNN.regs). --ppm saves the video output of each
// --ppm-at frame (496 x 384, the overlay off).
//
// Stage B: the DDR3 port is a model (DDR_BASE on, BUSY a random PCT of
// cycles, read bursts after LO..HI cycles), in the checkpoints with the
// SDRAM. --r3d writes, for the first draw at or after each --r3d-at frame,
// the Real3D memories that draw reads, in Supermodel's dump form
// (rNNNNNN.bin: 8C 4 MiB, 8E 1 MiB and polygon RAM 4 MiB from bank R at the
// swap, texture RAM 8 MiB at the frame's end), and the layer it draws
// (lNNNNNN.bin: 512 x 384 words {painted, RGB}), for sim/system/sb_ref.
#include "Vm3_system.h"
#include "Vm3_system___024root.h"
#include "verilated.h"
#include "verilated_save.h"
#include "sdram_dev.h"
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <deque>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#pragma pack(push, 1)
struct Rec { uint32_t frame, pc, addr; uint8_t kind, size, master, pad; uint64_t data; };
#pragma pack(pop)

static Vm3_system *t;
static sdram_dev sd;
static uint64_t cyc;

// DDR3 at the framework's DDRAM pins (64-bit words from the core's base)
struct ddr_dev {
  static constexpr uint64_t BASE = 0x06000000, SIZE = 0xD00000;
  std::vector<uint64_t> m = std::vector<uint64_t>(SIZE);
  int busy_pct = 10, lat_lo = 12, lat_hi = 40;
  uint32_t wr_left = 0; uint64_t wr_addr = 0;
  struct burst { uint64_t at, addr; uint32_t n; };
  std::deque<burst> rq;
  uint64_t bad = 0, rd_beats = 0, wr_beats = 0;
  std::mt19937 rng{7};
  uint64_t &at(uint64_t a) { static uint64_t s; if (a < BASE || a >= BASE + SIZE) { bad++; return s; } return m[a - BASE]; }
};
static ddr_dev dd;

static std::set<long> parse_list(const char *s) {
  std::set<long> r;
  while (s && *s) { r.insert(std::strtol(s, (char **)&s, 0)); if (*s == ',') s++; }
  return r;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string img_fn, trace_fn, dump_dir, ppm_dir, save_fn, restore_fn;
  long nframes = 10, window = 0x10000;
  std::set<long> dump_at, ppm_at, r3d_at;
  std::string r3d_dir;
  bool quiet = false, tb_fast = false, fatal_stop = true;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--image") img_fn = nx();
    else if (a == "--frames") nframes = std::atol(nx().c_str());
    else if (a == "--trace") trace_fn = nx();
    else if (a == "--dump") dump_dir = nx();
    else if (a == "--dump-at") dump_at = parse_list(nx().c_str());
    else if (a == "--ppm") ppm_dir = nx();
    else if (a == "--ppm-at") ppm_at = parse_list(nx().c_str());
    else if (a == "--window") window = std::strtol(nx().c_str(), nullptr, 0);
    else if (a == "--quiet") quiet = true;
    else if (a == "--tb-fast") tb_fast = true;
    else if (a == "--no-fatal-stop") fatal_stop = false;
    else if (a == "--save") save_fn = nx();
    else if (a == "--restore") restore_fn = nx();
    else if (a == "--r3d") r3d_dir = nx();
    else if (a == "--r3d-at") r3d_at = parse_list(nx().c_str());
    else if (a == "--ddr-busy") dd.busy_pct = std::atoi(nx().c_str());
    else if (a == "--ddr-lat") { dd.lat_lo = std::atoi(nx().c_str()); dd.lat_hi = std::atoi(nx().c_str()); }
  }
  std::vector<uint8_t> img;
  {
    std::FILE *f = std::fopen(img_fn.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "need --image\n"); return 2; }
    std::fseek(f, 0, SEEK_END); img.resize(size_t(std::ftell(f))); std::fseek(f, 0, SEEK_SET);
    if (std::fread(img.data(), 1, img.size(), f) != img.size()) return 2;
    std::fclose(f);
  }
  std::FILE *trf = trace_fn.empty() ? nullptr : std::fopen(trace_fn.c_str(), restore_fn.empty() ? "wb" : "ab");

  t = new Vm3_system;
  auto tick = [&]() {
    t->sd_dq_i = sd.dq(cyc);
    // DDR3: this cycle's BUSY and read beat
    t->DDRAM_BUSY = int(dd.rng() % 100) < dd.busy_pct;
    t->DDRAM_DOUT_READY = 0;
    if (!dd.rq.empty() && dd.rq.front().at <= cyc) {
      auto &b = dd.rq.front();
      t->DDRAM_DOUT_READY = 1; t->DDRAM_DOUT = dd.at(b.addr); b.addr++; dd.rd_beats++;
      if (--b.n == 0) dd.rq.pop_front();
    }
    t->clk = 0; t->eval();
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
    if (!t->DDRAM_BUSY && t->mem_rst_n) {
      if (t->DDRAM_WE) {
        if (dd.wr_left == 0) { dd.wr_addr = t->DDRAM_ADDR; dd.wr_left = t->DDRAM_BURSTCNT ? t->DDRAM_BURSTCNT : 256; }
        uint64_t &w = dd.at(dd.wr_addr);
        uint64_t const din = t->DDRAM_DIN;
        for (int b = 0; b < 8; b++) if ((t->DDRAM_BE >> b) & 1) w = (w & ~(0xffull << (8 * b))) | (din & (0xffull << (8 * b)));
        dd.wr_addr++; dd.wr_left--; dd.wr_beats++;
      } else if (t->DDRAM_RD) {
        uint64_t a = cyc + uint64_t(dd.lat_lo + int(dd.rng() % unsigned(dd.lat_hi - dd.lat_lo + 1)));
        if (!dd.rq.empty()) a = std::max(a, dd.rq.back().at);
        dd.rq.push_back({a, t->DDRAM_ADDR, t->DDRAM_BURSTCNT ? uint32_t(t->DDRAM_BURSTCNT) : 256u});
      }
    }
    t->clk = 1; t->eval();
    cyc++;
  };

  // power-on
  t->mem_rst_n = 0; t->reset = 0; t->ioctl_download = 0; t->ioctl_wr = 0; t->ioctl_index = 0;
  t->rd_lat_sel = 2; t->overlay_en = 0; t->tb_fast = 0; t->ddr_ok = 1;
  for (int i = 0; i < 8; i++) tick();
  t->mem_rst_n = 1;

  std::vector<uint8_t> pal(0x20000, 0), regs(256, 0);
  uint32_t gpr[32] = {};
  if (!restore_fn.empty()) {
    VerilatedRestore is;
    is.open(restore_fn.c_str());
    is >> *t;
    uint64_t n = 0;
    is.read(&cyc, sizeof cyc);
    is.read(sd.m.data(), sd.m.size() * 2);
    is.read(sd.row, sizeof sd.row);
    is.read(&n, sizeof n);
    sd.rq.clear();
    for (uint64_t i = 0; i < n; i++) { std::pair<uint64_t, uint16_t> e; is.read(&e.first, 8); is.read(&e.second, 2); sd.rq.push_back(e); }
    is.read(pal.data(), pal.size()); is.read(regs.data(), regs.size()); is.read(gpr, sizeof gpr);
    is.read(dd.m.data(), dd.m.size() * 8);
    is.read(&dd.wr_left, sizeof dd.wr_left); is.read(&dd.wr_addr, sizeof dd.wr_addr);
    is.read(&n, sizeof n);
    dd.rq.clear();
    for (uint64_t i = 0; i < n; i++) { ddr_dev::burst b; is.read(&b, sizeof b); dd.rq.push_back(b); }
    is.close();
    std::printf("restored %s: frame %u, cycle %" PRIu64 "\n", restore_fn.c_str(), t->frame, cyc);
  } else {
  // 1. the loader: a window of the image through ioctl
  for (size_t a = 0; a < img.size(); a += 2) sd.m[a >> 1] = uint16_t(0xA55A);   // not the image: the window must be written
  t->ioctl_download = 1;
  for (long a = 0; a < window; a += 2) {
    while (t->ioctl_wait) tick();
    t->ioctl_wr = 1; t->ioctl_addr = uint32_t(a); t->ioctl_dout = uint16_t(img[a] | (img[a + 1] << 8));
    tick();
    t->ioctl_wr = 0;
    for (int k = 0; k < 3; k++) tick();        // the HPS's pace, roughly
  }
  t->ioctl_download = 0;
  long guard = 0;
  while (!t->rom_loaded && guard++ < 2000000) tick();
  long ld_bad = 0;
  for (long a = 0; a < window; a += 2)
    if (sd.m[a >> 1] != uint16_t(img[a] | (img[a + 1] << 8))) ld_bad++;
  std::printf("loader: %ld bytes through ioctl, %ld words wrong, rom_loaded %d after %" PRIu64 " cycles\n",
              window, ld_bad, int(t->rom_loaded), cyc);
  if (ld_bad || !t->rom_loaded) { std::printf("FAIL\n"); return 1; }
  // The boot ROM's first act is to wait for timebase bit 24 (about one second,
  // 58 frames, at FFF0018C). With --tb-fast the timebase counts every clock
  // until that loop is left: the bench's only departure from the board, and
  // it shifts frame numbers by about 48 against Supermodel's.
  t->tb_fast = tb_fast;
  if (const char *e = std::getenv("M3_LOGW")) sd.log_writes = std::atol(e);
  // 2. the rest straight into the model (the game is still in reset this cycle)
  for (size_t a = 0; a + 1 < img.size(); a += 2) sd.m[a >> 1] = uint16_t(img[a] | (img[a + 1] << 8));
  }

  // the tile generator's state as written (CPU byte order), for --dump: pal, regs
  long nrec = 0, nunm = 0;
  // stage B dumps
  bool r3_armed = false; long r3_frame = 0; int r3_bank = 0, r3_stage = 0, r3_buf = 0;
  uint64_t r3_ends = 0, r3_dones = 0, r3_target = 0;
  std::vector<uint8_t> r3_mem;
  // the last HN retirements, printed at the fatal handler (M3_HIST=n: a ring of 2^n, default 2^10)
  struct H { uint32_t pc, insn, val; int8_t rd; };
  unsigned const hist_lg = std::getenv("M3_HIST") ? unsigned(std::atoi(std::getenv("M3_HIST"))) : 10u;
  uint64_t const HN = uint64_t(1) << hist_lg;
  std::vector<H> hist(HN, H{0, 0, 0, -1});
  uint64_t hist_n = 0;
  // M3_WATCH=pc,pc,...: print every retirement at those PCs (hex). M3_TRIG=pc:value[:after]: when a
  // retirement at pc writes value, print the history ring, then the next `after` retirements
  // (default 4096), then stop.
  std::set<uint32_t> watch;
  if (const char *e = std::getenv("M3_WATCH")) for (char *q = const_cast<char *>(e); *q;) { watch.insert(uint32_t(std::strtoul(q, &q, 16))); if (*q) q++; }
  bool trig_on = false; uint32_t trig_pc = 0, trig_val = 0; long trig_after = 4096, trig_left = -1;
  if (const char *e = std::getenv("M3_TRIG")) {
    char *q = const_cast<char *>(e);
    trig_pc = uint32_t(std::strtoul(q, &q, 16)); if (*q) q++;
    trig_val = uint32_t(std::strtoul(q, &q, 16)); if (*q) { q++; trig_after = std::atol(q); }
    trig_on = true;
  }
  bool fatal_seen = false;
  uint32_t mis_pc = 0, mis_in = 0;
  long mis_n = 0;
  std::map<std::string, long> mis_kind;
  uint32_t last_pc = 0, frame_seen = 0;
  // video capture
  std::vector<uint32_t> cap(496 * 384, 0);
  bool cap_on = false; int cx = 0, cy = 0; bool phb = true, pvb = true;
  long cap_frame = -1;
  auto t0 = std::chrono::steady_clock::now();
  uint64_t c0 = cyc;

  while (t->frame < uint32_t(nframes)) {
    tick();
    if (t->rt_valid_o) {
      last_pc = t->rt_pc_o;
      if (t->tb_fast && last_pc == 0xFFF00198) { t->tb_fast = 0; std::printf("frame %u: boot wait left at cycle %" PRIu64 "\n", t->frame, cyc); }
      uint32_t const in = t->rt_insn_o;
      // mtspr to the BATs, SDR1 or HID0: the value from the shadow GPRs
      if ((in >> 26) == 31 && ((in >> 1) & 0x3FF) == 467) {
        uint32_t const spr = ((in >> 16) & 31) | (((in >> 11) & 31) << 5), rs = (in >> 21) & 31;
        if ((spr >= 528 && spr <= 543) || spr == 25 || spr == 1008 || spr == 1009)
          std::printf("frame %u pc %08X: mtspr %u <- %08X\n", t->frame, last_pc, spr, gpr[rs]);
      }
      // misaligned loads and stores (the core forces them aligned, ppc_core's
      // header): the effective address from the shadow GPRs before this one
      if (!(last_pc == mis_pc && in == mis_in)) {
        uint32_t const op = in >> 26, xo = (in >> 1) & 0x3FF, ra = (in >> 16) & 31, rb = (in >> 11) & 31;
        int sz = 0; bool x = false;
        switch (op) {
          case 32: case 33: case 36: case 37: case 46: case 47: case 48: case 49: case 52: case 53: sz = 4; break;
          case 40: case 41: case 42: case 43: case 44: case 45: sz = 2; break;
          case 50: case 51: case 54: case 55: sz = 8; break;
          case 31: x = true;
            switch (xo) {
              case 23: case 55: case 151: case 183: case 534: case 662: case 535: case 663: case 567: case 695: case 20: case 150: case 983: sz = 4; break;
              case 279: case 311: case 343: case 375: case 407: case 439: case 790: case 918: sz = 2; break;
              case 599: case 727: case 631: case 759: sz = 8; break;
              default: break;
            }
            break;
          default: break;
        }
        if (sz) {
          uint32_t const base = ra ? gpr[ra] : 0;
          uint32_t const ea = x ? base + gpr[rb] : base + uint32_t(int32_t(int16_t(in & 0xFFFF)));
          if (ea & uint32_t(sz == 8 ? 3 : sz - 1)) {
            if (mis_n++ < 20) std::printf("frame %u pc %08X insn %08X: misaligned %d-byte access at %08X\n", t->frame, last_pc, in, sz, ea);
            {
              // the value against memory (word form: RAM and fixed CROM), CPU byte order
              auto mem_byte = [&](uint32_t a, bool &ok) -> uint8_t {
                uint32_t sa;
                if (a < 0x800000) sa = 0x6400000 + a; else if (a >= 0xFFE00000) sa = 0x6000000 + (a - 0xFFE00000); else { ok = false; return 0; }
                uint32_t const w = sa & ~3u, k = sa & 3;
                uint32_t const v = uint32_t(sd.m[w >> 1]) | (uint32_t(sd.m[(w >> 1) + 1]) << 16);
                return uint8_t(v >> (8 * (3 - k)));
              };
              bool ok = true;
              uint32_t mv = 0;
              for (int b = 0; b < (sz > 4 ? 4 : sz); b++) mv = (mv << 8) | mem_byte(ea + uint32_t(b), ok);
              uint32_t const rs = (in >> 21) & 31;
              bool const is_load = x ? !(xo == 151 || xo == 183 || xo == 662 || xo == 407 || xo == 439 || xo == 918 || xo == 663 || xo == 695 || xo == 727 || xo == 759 || xo == 150 || xo == 983)
                                     : !(op == 36 || op == 37 || op == 38 || op == 39 || op == 44 || op == 45 || op == 47 || (op >= 52 && op <= 55));
              if (ok && sz <= 4 && op != 43 && op != 42)   // (algebraic halfwords sign-extend; not used misaligned)
                std::printf("  %s: register %08X, memory now %08X%s\n", is_load ? "load " : "store", is_load ? t->rt_gpr_val_o : gpr[rs], mv,
                            is_load ? (t->rt_gpr_val_o == mv ? "  (equal)" : "  (DIFFERENT)") : "");
            }
            bool const st = x ? (xo == 151 || xo == 183 || xo == 662 || xo == 663 || xo == 695 || xo == 150 || xo == 983 || xo == 407 || xo == 439 || xo == 918 || xo == 727 || xo == 759)
                              : (op == 36 || op == 37 || op == 44 || op == 45 || op == 47 || (op >= 52 && op <= 55));
            bool const cross = ((ea & 3) + uint32_t(sz > 4 ? 4 : sz)) > 4;
            char key[96];
            std::snprintf(key, sizeof key, "pc %08X op %2u/%3u %s %d-byte, ea mod 4 = %u%s, region %02X", last_pc, op, x ? xo : 0,
                          st ? "store" : "load ", sz, ea & 3, cross ? " (crosses a word)" : "", ea >> 24);
            mis_kind[key]++;
          }
        }
      }
      mis_pc = last_pc; mis_in = in;
      if (t->rt_gpr_we_o && !t->rt_gpr_sh_o) gpr[t->rt_gpr_o] = t->rt_gpr_val_o;
      hist[hist_n++ % HN] = {last_pc, in, t->rt_gpr_we_o ? t->rt_gpr_val_o : 0u,
                             int8_t(t->rt_gpr_we_o ? (t->rt_gpr_sh_o ? 32 + t->rt_gpr_o : t->rt_gpr_o) : -1)};
      if (!watch.empty() && watch.count(last_pc))
        std::printf("watch frame %u cycle %" PRIu64 " pc %08X insn %08X val %08X%s\n", t->frame, cyc, last_pc, in,
                    t->rt_gpr_val_o, t->rt_gpr_we_o ? "" : " (no write)");
      if (trig_left > 0) {
        std::printf("  + %08X %08X %08X%s\n", last_pc, in, t->rt_gpr_we_o ? t->rt_gpr_val_o : 0u,
                    t->rt_gpr_we_o ? (t->rt_gpr_sh_o ? " t" : " r") : "");
        if (--trig_left == 0) { std::printf("trigger window done at cycle %" PRIu64 "\n", cyc); break; }
      } else if (trig_on && trig_left < 0 && last_pc == trig_pc && t->rt_gpr_we_o && t->rt_gpr_val_o == trig_val) {
        std::printf("trigger: frame %u cycle %" PRIu64 " pc %08X wrote %08X; GPRs:", t->frame, cyc, last_pc, trig_val);
        for (int k = 0; k < 32; k++) std::printf(" r%d=%08X", k, gpr[k]);
        std::printf("\n  the history (oldest first), then the next %ld:\n", trig_after);
        for (uint64_t k = std::min<uint64_t>(HN, hist_n); k-- > 0;) {
          auto &h = hist[(hist_n - 1 - k) % HN];
          std::printf("  - %08X %08X %08X", h.pc, h.insn, h.val);
          if (h.rd >= 32) std::printf(" t%d", h.rd - 32); else if (h.rd >= 0) std::printf(" r%d", h.rd);
          std::printf("\n");
        }
        trig_left = trig_after;
      }
      // the boot ROM's fatal-exception handler (every vector branches to FFF00000)
      if (last_pc == 0xFFF00000 && !fatal_seen) {
        fatal_seen = true;
        std::printf("frame %u: the fatal-exception handler at cycle %" PRIu64 "; the last retirements:\n", t->frame, cyc);
        std::printf("  GPRs:"); for (int k = 0; k < 32; k++) std::printf(" r%d=%08X", k, gpr[k]); std::printf("\n");
        for (uint64_t k = std::min<uint64_t>(HN, hist_n); k-- > 0;) {
          auto &h = hist[(hist_n - 1 - k) % HN];
          if (h.rd < 0)       std::printf("  %08X %08X %08X\n", h.pc, h.insn, h.val);
          else if (h.rd < 32) std::printf("  %08X %08X %08X r%d\n", h.pc, h.insn, h.val, h.rd);
          else                std::printf("  %08X %08X %08X t%d\n", h.pc, h.insn, h.val, h.rd - 32);   // a TGPR shadow
        }
        for (uint32_t a = 0xF0; a < 0x120; a += 4) {
          uint32_t const sa = 0x6400000 + a;
          std::printf("  RAM %06X in SDRAM: %08X\n", a, uint32_t(sd.m[sa >> 1]) | (uint32_t(sd.m[(sa >> 1) + 1]) << 16));
        }
        {
          int shown = 0;
          for (uint32_t w = 0x3000000; w < 0x4000000 && shown < 8; w++)   // the upper chip
            if (sd.m[w] == 0xFAA6 && sd.m[w + 1] == 0x7C70) { std::printf("  7C70FAA6 found at SDRAM byte %07X\n", w * 2); shown++; }
        }
        if (fatal_stop) break;
      }
    }
    if (t->tr_v) {
      uint32_t be = t->tr_be, d = t->tr_data;
      int lo = 0; while (lo < 4 && !((be >> lo) & 1)) lo++;
      int hi = 3; while (hi >= 0 && !((be >> hi) & 1)) hi--;
      int size = hi >= lo ? hi - lo + 1 : 0;
      uint32_t addr = t->tr_addr + uint32_t(3 - hi);
      uint64_t v = size == 4 ? d : (d >> (8 * lo)) & ((1u << (8 * size)) - 1);
      // the tile generator's state, CPU byte order
      if (t->tr_we && (t->tr_addr >> 17) == (0xF1100000u >> 17)) {
        for (int k = 0; k < 4; k++) if ((be >> (3 - k)) & 1) pal[(t->tr_addr & 0x1FFFC) + k] = uint8_t(d >> (24 - 8 * k));
      }
      if (t->tr_we && (t->tr_addr & 0xFFFFFF00) == 0xF1180000) {
        for (int k = 0; k < 4; k++) if ((be >> (3 - k)) & 1) regs[(t->tr_addr & 0xFC) + k] = uint8_t(d >> (24 - 8 * k));
      }
      if (trf) {
        Rec r{t->frame, last_pc, addr, uint8_t(t->tr_we), uint8_t(size), uint8_t(t->tr_master), 0, v};
        std::fwrite(&r, sizeof r, 1, trf);
      }
      nrec++;
    }
    // stage B: a draw's memories and its layer (--r3d)
    if (!r3d_dir.empty()) {
      auto *rp = t->rootp;
      if (rp->m3_system__DOT__m3_swap && !r3_armed && !r3d_at.empty() && long(t->frame) >= *r3d_at.begin()) {
        // the swap takes effect at this edge's end: bank R is the one m3_bank becomes
        r3_armed = true; r3_frame = long(t->frame); r3_bank = !rp->m3_system__DOT__m3_bank; r3_stage = 0;
        r3_mem.assign(0x1100000, 0);
        for (uint32_t a = 0; a < 0x100000; a++) {                     // 8C
          uint32_t const sb = 0x6D00000u + 4 * a;
          uint32_t const v = uint32_t(sd.m[sb >> 1]) | (uint32_t(sd.m[(sb >> 1) + 1]) << 16);
          std::memcpy(&r3_mem[4 * a], &v, 4);
        }
        for (uint32_t a = 0; a < 0x40000; a++) {                      // 8E, bank R
          uint32_t const sb = (r3_bank ? 0x7200000u : 0x7100000u) + 4 * a;
          uint32_t const v = uint32_t(sd.m[sb >> 1]) | (uint32_t(sd.m[(sb >> 1) + 1]) << 16);
          std::memcpy(&r3_mem[0x400000 + 4 * a], &v, 4);
        }
        for (uint32_t a = 0; a < 0x80000; a++) {                      // polygon RAM, bank R
          uint64_t const q = dd.at(ddr_dev::BASE + (r3_bank ? 0x180000u : 0x100000u) + a);
          std::memcpy(&r3_mem[0x500000 + 8 * a], &q, 8);
        }
        r3_target = r3_ends + 1;   // frames end, and finish, in order
        std::printf("frame %u: r3d dump armed (bank %d)\n", t->frame, r3_bank);
      }
      if (rp->m3_system__DOT__f_end) {
        r3_ends++;
        if (r3_armed && r3_stage == 0 && r3_ends == r3_target) {
          r3_stage = 1; r3_buf = rp->m3_system__DOT__f_buf;
          for (uint32_t y = 0; y < 2048; y++)                          // the sheets, at the frame's end
            for (uint32_t x = 0; x < 2048; x++) {
              uint64_t const q = dd.at(ddr_dev::BASE + 0xC00000 + ((uint64_t(y >> 2) << 11) | (uint64_t(x >> 2) << 2) | (y & 3)));
              uint16_t const v = uint16_t(q >> (16 * (x & 3)));
              std::memcpy(&r3_mem[0x900000 + 2 * (size_t(y) * 2048 + x)], &v, 2);
            }
        }
      }
      if (rp->m3_system__DOT__r_done) {
        r3_dones++;
        if (r3_armed && r3_stage == 1 && r3_dones == r3_target) {
          char fn[512];
          std::snprintf(fn, sizeof fn, "%s/r%06ld.bin", r3d_dir.c_str(), r3_frame);
          if (std::FILE *f = std::fopen(fn, "wb")) { std::fwrite(r3_mem.data(), 1, r3_mem.size(), f); std::fclose(f); }
          std::vector<uint32_t> lay(512 * 384);
          for (uint32_t y = 0; y < 384; y++)
            for (uint32_t x = 0; x < 512; x++) {
              uint64_t const q = dd.at(ddr_dev::BASE + uint64_t(r3_buf) * 0x20000 + uint64_t(y) * 256 + (x >> 1));
              lay[size_t(y) * 512 + x] = uint32_t(x & 1 ? q >> 32 : q);
            }
          std::snprintf(fn, sizeof fn, "%s/l%06ld.bin", r3d_dir.c_str(), r3_frame);
          if (std::FILE *f = std::fopen(fn, "wb")) { std::fwrite(lay.data(), 4, lay.size(), f); std::fclose(f); }
          std::printf("frame %u: r3d dump of frame %ld written (layer buffer %d)\n", t->frame, r3_frame, r3_buf);
          r3d_at.erase(r3d_at.begin());
          while (!r3d_at.empty() && *r3d_at.begin() <= long(t->frame)) r3d_at.erase(r3d_at.begin());
          r3_armed = false;
        }
      }
    }
    // a new frame: the bench's frame count steps at the start of vertical blanking
    if (t->frame != frame_seen) {
      frame_seen = t->frame;
      if (trf) { Rec r{t->frame, last_pc, 0, 2, 3, 0, 0, 0}; std::fwrite(&r, sizeof r, 1, trf); std::fflush(trf); }
      if (!quiet && (frame_seen % 10 == 0 || frame_seen < 5)) {
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("frame %5u  cycle %11" PRIu64 "  pc %08X  %6.0f kHz  records %ld\n", frame_seen, cyc, last_pc,
                    double(cyc - c0) / s / 1000.0, nrec);
        std::fflush(stdout);
      }
      if (ppm_at.count(long(frame_seen))) { cap_on = true; cap_frame = long(frame_seen); cx = cy = 0; }
    }
    // the state at the start of the active display (line 0)
    if (!dump_dir.empty() && dump_at.count(long(t->frame)) && t->v_vblank == 0 && pvb == 1) {
      char base[512];
      std::snprintf(base, sizeof base, "%s/f%06u", dump_dir.c_str(), t->frame);
      std::vector<uint8_t> v(0x120000, 0);
      for (uint32_t a = 0; a < 0x100000; a += 2) { uint16_t w = sd.m[(0x6C00000 + a) >> 1]; v[a] = uint8_t(w); v[a + 1] = uint8_t(w >> 8); }
      std::memcpy(&v[0x100000], pal.data(), 0x20000);
      if (std::FILE *f = std::fopen((std::string(base) + ".vram").c_str(), "wb")) { std::fwrite(v.data(), 1, v.size(), f); std::fclose(f); }
      // registers: tg_tool's .regs hold each little-endian value (the bus word
      // byte-reversed) as little-endian bytes, which is the CPU's byte order
      std::vector<uint8_t> rr(regs);
      if (std::FILE *f = std::fopen((std::string(base) + ".regs").c_str(), "wb")) { std::fwrite(rr.data(), 1, 256, f); std::fclose(f); }
      dump_at.erase(long(t->frame));
    }
    // video capture: the first full frame after the request
    if (t->ce_pix) {
      bool hb = t->v_hblank, vb = t->v_vblank;
      if (cap_on) {
        if (!vb && !hb) { if (cx < 496 && cy < 384) cap[size_t(cy) * 496 + size_t(cx)] = (uint32_t(t->v_r) << 16) | (uint32_t(t->v_g) << 8) | t->v_b; cx++; }
        if (hb && !phb && !vb) { cx = 0; cy++; }
        if (vb && !pvb && cy > 0) {
          cap_on = false;
          char fn[512];
          std::snprintf(fn, sizeof fn, "%s/f%06ld.ppm", ppm_dir.c_str(), cap_frame);
          if (std::FILE *f = std::fopen(fn, "wb")) {
            std::fprintf(f, "P6\n496 384\n255\n");
            for (uint32_t p : cap) { uint8_t c[3] = {uint8_t(p >> 16), uint8_t(p >> 8), uint8_t(p)}; std::fwrite(c, 1, 3, f); }
            std::fclose(f);
          }
        }
        if (vb && !pvb && cy == 0) { cx = 0; cy = 0; }
      }
      phb = hb; pvb = vb;
    }
  }
  double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("misaligned accesses (word-aligned doubles not counted): %ld\n", mis_n);
  for (auto &k : mis_kind) std::printf("  misaligned: %s: %ld\n", k.first.c_str(), k.second);
  std::printf("done: %u frames, %" PRIu64 " cycles in %.0f s (%.0f kHz), %ld bus records, last pc %08X, SDRAM %" PRIu64 " reads %" PRIu64 " writes\n",
              t->frame, cyc, s, double(cyc - c0) / s / 1000.0, nrec, last_pc, sd.reads, sd.writes);
  (void)nunm;
  if (trf) std::fclose(trf);
  if (!save_fn.empty()) {
    VerilatedSave os;
    os.open(save_fn.c_str());
    os << *t;
    uint64_t n = sd.rq.size();
    os.write(&cyc, sizeof cyc);
    os.write(sd.m.data(), sd.m.size() * 2);
    os.write(sd.row, sizeof sd.row);
    os.write(&n, sizeof n);
    for (auto &e : sd.rq) { os.write(&e.first, 8); os.write(&e.second, 2); }
    os.write(pal.data(), pal.size()); os.write(regs.data(), regs.size()); os.write(gpr, sizeof gpr);
    os.write(dd.m.data(), dd.m.size() * 8);
    os.write(&dd.wr_left, sizeof dd.wr_left); os.write(&dd.wr_addr, sizeof dd.wr_addr);
    n = dd.rq.size();
    os.write(&n, sizeof n);
    for (auto &b : dd.rq) os.write(&b, sizeof b);
    os.close();
    std::printf("saved %s at frame %u, cycle %" PRIu64 "\n", save_fn.c_str(), t->frame, cyc);
  }
  delete t;
  return 0;
}
