// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// m3_tilegen bench (M0-E): every dump is loaded into a memory model behind the
// m2_sdram-contract port, the palette and registers go in through the CPU
// ports, and each line is built, swapped and scanned out. The scanned pixels
// are compared with tg_ref.h's param model under the same switches.
//
//   tb_tilegen [--cfg sm|mame] [--lat MIN MAX] [--busy PCT] [--realtime]
//              [--mutate K] [--max N] [--ppm DIR] DIR...
//
// --lat: acknowledge delay in clocks, uniform per request (hostile: R73).
// --busy: percent of requests that wait a further 50-400 clocks.
// --realtime: a line every 4,100 clocks (656 pixels at 16 MHz on 100 MHz),
//   pixels at 16/100 of the clock, go and swap on time whether or not the line
//   is done; reports the slowest line and overruns.
// Without --realtime each line is built to completion (lockstep), so hostile
// delays test correctness only.
// --mutate K: also K synthetic variants of each dump (tg::mutate).
// --recover: each frame first in real time behind the given memory (overruns
//   expected and counted), then again line by line behind a quick memory, which
//   must be exact: an abandoned line must leave nothing behind.
#include "Vm3_tilegen.h"
#include "verilated.h"
#include "tg_ref.h"

#include <algorithm>
#include <dirent.h>
#include <map>

using namespace tg;

static Vm3_tilegen *top;
static uint64_t cyc = 0;
static std::vector<uint8_t> mem;          // the 1 MiB VRAM image
static Rng *rng;
static int lat_min = 2, lat_max = 2, busy_pct = 0;
static long proto_err = 0, nreqs = 0;
// request census (R103): by kind, and how many a line would need with each pattern burst fetched once and the
// name words in 16-byte (eight-word) bursts
#include <unordered_set>
static std::unordered_set<uint32_t> ln_pat, ln_name16;
static long n_pat = 0, n_name = 0, n_other = 0, u_pat = 0, u_name16 = 0;
static void census_line() { u_pat += (long)ln_pat.size(); u_name16 += (long)ln_name16.size(); ln_pat.clear(); ln_name16.clear(); }

// memory model state
static bool m_pend = false, m_prev = false, m_ack_next = false, m_long = false, m_lo_next = false, m_lo_done = false;
static uint64_t m_due = 0;
static uint32_t m_addr = 0;

static uint64_t burst(uint32_t waddr) {
  uint32_t b = (waddr * 2) & 0xFFFF8u;
  uint64_t v = 0;
  for (int k = 7; k >= 0; k--) v = (v << 8) | mem[b + (uint32_t)k];
  return v;
}

static void tick() {
  // drive this cycle's acknowledge
  top->p_lo = 0;
  if (m_ack_next) {
    top->p_ack = 1;
    top->p_dout = burst(m_long ? m_addr + 4 : m_addr);   // an eight-word read: its last four words
    m_ack_next = false;
  } else if (m_lo_next) {
    top->p_ack = 0;
    top->p_lo = 1;                                       // its first four, without an acknowledge
    top->p_dout = burst(m_addr);
    m_lo_next = false;
  } else {
    top->p_ack = 0;
    top->p_dout = ((uint64_t)rng->next() << 32) | rng->next();   // garbage outside the acknowledge
  }
  top->clk = 1;
  top->eval();
  top->clk = 0;
  top->eval();
  cyc++;
  // sample the request after the edge
  bool req = top->p_req;
  if (req && !m_prev) {
    if (m_pend) { if (proto_err++ < 5) std::printf("protocol: request while one is pending (cycle %llu)\n", (unsigned long long)cyc); }
    m_pend = true;
    m_addr = (uint32_t)top->p_addr;
    m_long = top->p_long; m_lo_done = !m_long;
    if (m_long && (m_addr & 7)) { if (proto_err++ < 5) std::printf("protocol: eight-word read not aligned\n"); }
    int d = lat_min + (int)rng->below((uint32_t)(lat_max - lat_min + 1));
    if (busy_pct && rng->chance(busy_pct)) d += 50 + (int)rng->below(351);
    m_due = cyc + (uint64_t)d;
    nreqs++;
    {
      uint32_t const w = m_addr & 0x7FFFF;          // VRAM word address
      if ((w >> 14) == 0x1F) { n_name++; ln_name16.insert(w >> 3); }
      else if ((w >> 11) >= 0x7B) n_other++;        // F6000 line scroll, F7000 stencil
      else { n_pat++; ln_pat.insert(w >> 2); }
    }
  } else if (m_pend) {
    if (!req) { if (proto_err++ < 5) std::printf("protocol: request dropped before acknowledge (cycle %llu)\n", (unsigned long long)cyc); m_pend = false; }
    else if ((uint32_t)top->p_addr != m_addr) { if (proto_err++ < 5) std::printf("protocol: address moved while pending\n"); }
  }
  if (m_pend && !m_lo_done && cyc + 5 >= m_due) { m_lo_done = true; m_lo_next = true; }
  else if (m_pend && m_lo_done && !m_lo_next && cyc + 1 >= m_due) { m_pend = false; m_ack_next = true; }
  m_prev = req;
}

static void idle_inputs() {
  top->reg_we = 0; top->pal_we = 0; top->swap = 0; top->go = 0; top->sc_rd = 0; top->name_inv = 0;
}

// collected pixels
static Frame got;

static void reset_dut() {
  idle_inputs();
  top->rst_n = 0;
  for (int i = 0; i < 4; i++) tick();
  top->rst_n = 1;
  m_pend = m_ack_next = m_prev = m_lo_next = false;
  for (int i = 0; i < 600 && !top->ready; i++) tick();
}

static void load(const Dump &d) {
  mem.assign(d.vram.begin(), d.vram.begin() + 0x100000);
  idle_inputs();
  for (int i = 0; i < 32768; i++) {
    top->pal_we = 1;
    top->pal_addr = i;
    top->pal_wdata = d.le16(0x100000 + 4 * (uint32_t)i);
    tick();
  }
  top->pal_we = 0;
  static const int regs[] = {0x20, 0x40, 0x44, 0x60, 0x64, 0x68, 0x6C};
  for (int r : regs) {
    top->reg_we = 1;
    top->reg_addr = r >> 2;
    top->reg_wdata = d.reg((uint32_t)r);
    tick();
  }
  top->reg_we = 0;
  tick();
}

struct Stats { long lines = 0, overruns = 0; int max_cycles = 0; long sum_cycles = 0; int max_reqs = 0; };

// Render the frame. Pixel x of line y is requested with sc_rd and arrives three clocks later.
static void run_frame(bool realtime, Stats &st) {
  got.assign((size_t)W * H, Pix{});
  const int LINE = 4100;
  // a FIFO of x positions in flight
  std::vector<int> fly;
  int y_scan = -1, x_next = 0;
  auto out = [&]() {
    if (top->o_v && !fly.empty()) {
      int x = fly.front();
      fly.erase(fly.begin());
      if (y_scan >= 0 && y_scan < H && x < W) {
        Pix &p = got[(size_t)y_scan * W + x];
        p.valid = top->o_drawn;
        p.above = top->o_above;
        p.rgb = p.valid ? ((uint32_t)top->o_r << 16) | ((uint32_t)top->o_g << 8) | top->o_b : 0;
      }
    }
  };
  auto finish_line_stats = [&]() {
    st.lines++;
    st.max_cycles = std::max(st.max_cycles, (int)top->st_cycles);
    st.sum_cycles += top->st_cycles;
    st.max_reqs = std::max(st.max_reqs, (int)top->st_reqs);
  };
  if (!realtime) {
    // line 0 first
    census_line(); top->go = 1; top->go_y = 0; tick(); top->go = 0; out();
    while (top->busy) { tick(); out(); }
    finish_line_stats();
    for (int y = 0; y < H; y++) {
      top->swap = 1; tick(); top->swap = 0; out();
      y_scan = y;
      if (y + 1 < H) { census_line(); top->go = 1; top->go_y = y + 1; tick(); top->go = 0; out(); }
      // scan line y at one pixel every 6 clocks, building y + 1 meanwhile
      x_next = 0;
      int k = 0;
      while (x_next < W || !fly.empty() || top->busy) {
        if (x_next < W && (k++ % 6) == 0) { top->sc_rd = 1; top->sc_x = x_next; fly.push_back(x_next); x_next++; }
        tick();
        top->sc_rd = 0;
        out();
      }
      if (y + 1 < H) finish_line_stats();
    }
  } else {
    // real time: a line every LINE clocks; line y + 1 is started as line y starts being shown
    for (int yl = -1; yl < H; yl++) {
      if (yl >= 0) {
        // the build of line yl began a period ago
        if (top->busy) st.overruns++;
        finish_line_stats();
        top->swap = 1; tick(); top->swap = 0; out();
      }
      y_scan = yl;
      if (yl + 1 < H) { census_line(); top->go = 1; top->go_y = yl + 1; tick(); top->go = 0; out(); }
      x_next = 0;
      long acc = 0;
      for (int c = 2; c < LINE; c++) {
        acc += 16;
        bool ce = acc >= 100;
        if (ce) acc -= 100;
        // the visible pixels start after the horizontal blank (160 pixels, 1,000 clocks)
        if (yl >= 0 && ce && x_next < W && c > 1000) { top->sc_rd = 1; top->sc_x = x_next; fly.push_back(x_next); x_next++; }
        tick();
        top->sc_rd = 0;
        out();
      }
      while (!fly.empty()) { tick(); out(); }
    }
  }
}

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

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  Cfg cfg = Cfg::sm();
  bool realtime = false, recover = false;
  int mutate_k = 0, maxn = 1 << 30;
  std::string ppm;
  std::vector<std::string> dirs;
  uint64_t seed = 1;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--cfg" && i + 1 < argc) cfg = std::string(argv[++i]) == "mame" ? Cfg::mame() : Cfg::sm();
    else if (a == "--lat" && i + 2 < argc) { lat_min = std::atoi(argv[++i]); lat_max = std::atoi(argv[++i]); }
    else if (a == "--busy" && i + 1 < argc) busy_pct = std::atoi(argv[++i]);
    else if (a == "--realtime") realtime = true;
    else if (a == "--recover") recover = true;
    else if (a == "--mutate" && i + 1 < argc) mutate_k = std::atoi(argv[++i]);
    else if (a == "--max" && i + 1 < argc) maxn = std::atoi(argv[++i]);
    else if (a == "--seed" && i + 1 < argc) seed = std::strtoull(argv[++i], nullptr, 0);
    else if (a == "--ppm" && i + 1 < argc) ppm = argv[++i];
    else if (a[0] != '+') dirs.push_back(a);
  }
  rng = new Rng(seed);
  top = new Vm3_tilegen;
  reset_dut();
  long frames = 0, bad = 0, badpx = 0;
  Stats st;
  for (auto &dir : dirs) {
    int n = 0;
    for (auto &b : bases(dir)) {
      if (n++ >= maxn) break;
      Dump d0;
      if (!d0.load(b)) { std::printf("%s: load failed\n", b.c_str()); bad++; continue; }
      for (int k = 0; k <= mutate_k; k++) {
        Dump d = d0;
        std::string tag = b;
        if (k) { mutate(d, seed * 1000003ull + (uint64_t)k * 7919u + std::hash<std::string>{}(b)); tag += "~" + std::to_string(k); }
        load(d);
        Stats fs;
        if (recover) {
          // a real-time frame behind the given (hostile) memory, overruns expected; then the same frame
          // built line by line behind a quick memory must be exact: an abandoned line leaves nothing behind
          Stats junk;
          run_frame(true, junk);
          st.overruns += junk.overruns;
          int a0 = lat_min, a1 = lat_max, b0 = busy_pct;
          lat_min = lat_max = 2; busy_pct = 0;
          run_frame(false, fs);
          lat_min = a0; lat_max = a1; busy_pct = b0;
        } else
          run_frame(realtime, fs);
        Frame ref;
        param(d, cfg, ref);
        int first = -1;
        long e = diff(ref, got, &first);
        frames++;
        st.lines += fs.lines; st.overruns += fs.overruns; st.sum_cycles += fs.sum_cycles;
        st.max_cycles = std::max(st.max_cycles, fs.max_cycles); st.max_reqs = std::max(st.max_reqs, fs.max_reqs);
        if (e) {
          bad++; badpx += e;
          const Pix &r = ref[(size_t)first], &g = got[(size_t)first];
          std::printf("%s: %ld px differ; first (%d,%d) ref %d/%d/%06X got %d/%d/%06X\n", tag.c_str(), e, first % W,
                      first / W, r.valid, r.above, r.rgb, g.valid, g.above, g.rgb);
          if (!ppm.empty()) {
            std::string base = ppm + "/" + tag.substr(tag.find_last_of('/') + 1);
            write_ppm(base + "_rtl.ppm", got);
            write_ppm(base + "_ref.ppm", ref);
          }
        }
        if (fs.overruns && realtime) std::printf("%s: %ld overruns, slowest line %d clocks\n", tag.c_str(), fs.overruns, fs.max_cycles);
      }
    }
  }
  std::printf("frames %ld, differing %ld (%ld px), protocol errors %ld, requests %ld\n", frames, bad, badpx, proto_err, nreqs);
  census_line();
  std::printf("census: pattern %ld (once a line: %ld), name %ld (in 8-word bursts, once a line: %ld), other %ld\n",
              n_pat, u_pat, n_name, u_name16, n_other);
  std::printf("lines %ld: slowest %d clocks, mean %.0f, most requests %d, overruns %ld\n", st.lines, st.max_cycles,
              st.lines ? (double)st.sum_cycles / st.lines : 0.0, st.max_reqs, st.overruns);
  std::printf("%s\n", (bad || proto_err || (realtime && !recover && st.overruns)) ? "FAIL" : "PASS");
  delete top;
  return (bad || proto_err) ? 1 : 0;
}
