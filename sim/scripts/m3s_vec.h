// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Replay vectors (cap2vec.py) and the bench memory: a sparse word map in which
// an unwritten word reads as a hash of its address, so a wrong address in a
// move shows up as wrong data at the destination.

#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

struct VRec {
  char k;
  uint32_t a = 0, b = 0, c = 0, d = 0;
  int n = 0;            // fields after the letter
};

static inline std::vector<VRec> load_vec(const std::string &path) {
  std::vector<VRec> v;
  FILE *f = std::fopen(path.c_str(), "r");
  if (!f) { std::perror(path.c_str()); std::exit(2); }
  char line[256];
  while (std::fgets(line, sizeof line, f)) {
    VRec r;
    r.k = line[0];
    unsigned x[4] = {0, 0, 0, 0};
    r.n = std::sscanf(line + 1, "%x %x %x %x", &x[0], &x[1], &x[2], &x[3]);
    r.a = x[0]; r.b = x[1]; r.c = x[2]; r.d = x[3];
    v.push_back(r);
  }
  std::fclose(f);
  return v;
}

struct BenchMem {
  std::unordered_map<uint32_t, uint32_t> m;
  static uint32_t hash(uint32_t a) {
    uint32_t h = a * 0x9E3779B1u;
    h ^= h >> 15; h *= 0x85EBCA77u; h ^= h >> 13;
    return h;
  }
  uint32_t rd(uint32_t a) const {
    auto it = m.find(a & ~3u);
    return it == m.end() ? hash(a & ~3u) : it->second;
  }
  void wr(uint32_t a, uint32_t d, unsigned be = 0xF) {
    uint32_t o = rd(a), mask = 0;
    for (int i = 0; i < 4; i++) if (be & (1u << i)) mask |= 0xFFu << (8 * i);
    m[a & ~3u] = (o & ~mask) | (d & mask);
  }
};
