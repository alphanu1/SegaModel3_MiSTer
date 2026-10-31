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
// r3d_walker against the reference model's event trace, frame by frame.
//
//   Vr3d_walker --dumps DIR --vrom FILE [--lat N] [--stall P] [--frames N] [--seed S]
//
// The memory answers a request after 1-3 cycles and returns its words N cycles
// later, one a cycle; words past the end of a region come back as 0xDEADBEEF
// so the walker's masking is exercised. P is the chance, in percent, that the
// event consumer is not ready in a cycle. Reports mismatches and the walker's
// cycles per frame.

#include "Vr3d_walker.h"
#include "verilated.h"
#include "r3d_geo.h"
#include <zlib.h>
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace r3d;
namespace fs = std::filesystem;

static bool load_dump(std::string const &fn, r3d_mem &m) {
  gzFile g = gzopen(fn.c_str(), "rb");
  if (!g) return false;
  bool ok = gzread(g, m.cull_lo.data(), 0x400000) == 0x400000 && gzread(g, m.cull_hi.data(), 0x100000) == 0x100000 &&
            gzread(g, m.poly.data(), 0x400000) == 0x400000;
  gzclose(g);
  return ok;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  std::string dumps, vrom;
  int lat = 8, stall = 20;
  long max_frames = -1;
  unsigned seed = 1;
  for (int i = 1; i < argc; i++) {
    std::string const a = argv[i];
    auto nx = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--dumps") dumps = nx();
    else if (a == "--vrom") vrom = nx();
    else if (a == "--lat") lat = std::atoi(nx().c_str());
    else if (a == "--stall") stall = std::atoi(nx().c_str());
    else if (a == "--frames") max_frames = std::atol(nx().c_str());
    else if (a == "--seed") seed = unsigned(std::atoi(nx().c_str()));
  }
  if (dumps.empty()) { std::fprintf(stderr, "need --dumps\n"); return 2; }
  r3d_mem mem;
  if (!vrom.empty()) {
    std::FILE *f = std::fopen(vrom.c_str(), "rb");
    if (f) { size_t n = std::fread(mem.vrom.data(), 4, mem.vrom.size(), f); (void)n; std::fclose(f); }
  }
  std::vector<std::string> files;
  for (auto const &e : fs::directory_iterator(dumps))
    if (e.path().extension() == ".gz") files.push_back(e.path().string());
  std::sort(files.begin(), files.end());
  if (max_frames >= 0 && size_t(max_frames) < files.size()) files.resize(size_t(max_frames));

  std::mt19937 rng(seed);
  Vr3d_walker *t = new Vr3d_walker;
  auto tick = [&]() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); };
  t->rst = 1; t->start = 0; t->mem_ack = 0; t->mem_rvalid = 0; t->ev_ready = 0;
  tick(); tick();
  t->rst = 0;

  uint64_t frames_ok = 0, events_total = 0, mism_frames = 0;
  std::vector<uint64_t> cycles;
  std::vector<uint64_t> reads;
  for (size_t fi = 0; fi < files.size(); fi++) {
    if (!load_dump(files[fi], mem)) { std::fprintf(stderr, "bad dump %s\n", files[fi].c_str()); return 1; }
    std::vector<std::array<uint32_t, 4>> ref;
    {
      r3d_geo geo(mem, arith::rtl, false);
      geo.trace = &ref;
      geo.frame(uint32_t(fi));
    }
    std::vector<std::array<uint32_t, 4>> got;
    // memory model state
    struct beat { uint64_t at; uint32_t data; };
    std::deque<beat> q;
    int ack_wait = -1;
    uint64_t cyc = 0, nreads = 0;
    t->start = 1; tick(); t->start = 0;
    bool done = false;
    while (!done && cyc < 50000000) {
      // drive inputs for this cycle from current outputs
      t->mem_ack = 0;
      if (t->mem_req) {
        if (ack_wait < 0) ack_wait = int(rng() % 3) + 1;
        if (--ack_wait == 0) {
          t->mem_ack = 1;
          ack_wait = -1;
          nreads++;
          uint32_t const a = t->mem_addr;
          for (uint32_t k = 0; k <= t->mem_len; k++) {
            uint32_t const w = a + k;
            uint32_t d;
            if (a & 0x800000) d = (w - 0x800000) < 0x40000 ? mem.cull_hi[w - 0x800000] : 0xDEADBEEFu;
            else d = w < 0x100000 ? mem.cull_lo[w] : 0xDEADBEEFu;
            q.push_back({cyc + uint64_t(lat) + k, d});
          }
        }
      }
      t->mem_rvalid = 0;
      if (!q.empty() && q.front().at <= cyc) { t->mem_rvalid = 1; t->mem_rdata = q.front().data; q.pop_front(); }
      t->ev_ready = (int(rng() % 100) >= stall) ? 1 : 0;
      t->eval();
      if (t->ev_valid && t->ev_ready) {
        got.push_back({uint32_t(t->ev_type), t->ev_a, t->ev_b, t->ev_c});
        if (t->ev_type == 7) done = true;
      }
      tick();
      cyc++;
    }
    // compare: the payload words a type does not use are don't-care
    auto norm = [](std::array<uint32_t, 4> e) {
      switch (e[0]) {
        case 1: case 3: break;
        case 4: case 5: e[2] = e[3] = 0; break;
        default: e[1] = e[2] = e[3] = 0; break;
      }
      return e;
    };
    size_t bad = 0, first = size_t(-1);
    size_t const n = std::min(ref.size(), got.size());
    for (size_t i = 0; i < n; i++)
      if (norm(ref[i]) != norm(got[i])) { bad++; if (first == size_t(-1)) first = i; }
    bad += std::max(ref.size(), got.size()) - n;
    events_total += ref.size();
    cycles.push_back(cyc);
    reads.push_back(nreads);
    if (bad == 0 && t->overflow == 0) frames_ok++;
    else {
      mism_frames++;
      if (mism_frames <= 5) {
        std::printf("%s: %zu events differ (ref %zu, rtl %zu), overflow %d", fs::path(files[fi]).filename().c_str(), bad, ref.size(), got.size(), int(t->overflow));
        if (first != size_t(-1)) {
          auto a = norm(ref[first]), b = norm(got[first]);
          std::printf("; first at %zu: ref %u %08X %08X %08X rtl %u %08X %08X %08X", first, a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]);
        }
        std::printf("\n");
      }
    }
  }
  std::sort(cycles.begin(), cycles.end());
  std::sort(reads.begin(), reads.end());
  auto p = [](std::vector<uint64_t> const &v, double q) { return v.empty() ? 0.0 : double(v[std::min(v.size() - 1, size_t(q * (v.size() - 1) + 0.5))]); };
  std::printf("r3d_walker: %zu frames, %" PRIu64 " match the model exactly (%" PRIu64 " events), memory latency %d, consumer stall %d%%\n",
              files.size(), frames_ok, events_total, lat, stall);
  std::printf("walker cycles per frame p50 %.0f p99 %.0f max %.0f (%.1f%% of a 50 MHz frame at p99); requests p50 %.0f p99 %.0f\n",
              p(cycles, .5), p(cycles, .99), p(cycles, 1), 100.0 * p(cycles, .99) / 869265, p(reads, .5), p(reads, .99));
  delete t;
  return mism_frames ? 1 : 0;
}
