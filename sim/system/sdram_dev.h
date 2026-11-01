// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// The MiSTer 128 MB SDRAM module at its pins, as m2_sdram drives it (two
// 64 MB chips on one select, the byte mask on A12:A11): flat storage, read
// data `lat` cycles after the READ. From sim/viewer/tb_m3v.cpp's model. The
// array index is m2_sdram's word address ({chip, bank, row, column}), so
// m[byte >> 1] is the little-endian 16-bit word at that byte.
#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>

struct sdram_dev {
  std::vector<uint16_t> m = std::vector<uint16_t>(size_t(1) << 26);
  uint32_t row[2][4] = {};
  int lat = 2;                                    // matches rd_lat_sel = 2
  std::deque<std::pair<uint64_t, uint16_t>> rq;
  uint64_t reads = 0, writes = 0;
  long log_writes = 0;                            // print this many writes (debug)
  static uint32_t idx(uint32_t chip, uint32_t bank, uint32_t r, uint32_t c) { return (chip << 25) | (bank << 23) | (r << 10) | c; }

  // the data for the cycle `cyc`, before the clock edge
  uint16_t dq(uint64_t cyc) {
    while (!rq.empty() && rq.front().first < cyc) rq.pop_front();
    return (!rq.empty() && rq.front().first == cyc) ? rq.front().second : 0;
  }
  // the command on the pins in cycle `cyc`
  void cmd(uint64_t cyc, bool cke, bool cs_n, bool ras_n, bool cas_n, bool we_n, uint32_t ba, uint32_t a, uint16_t dq_o) {
    if (!cke) return;
    uint32_t const chip = cs_n ? 1u : 0u;
    uint32_t const c = (uint32_t(ras_n) << 2) | (uint32_t(cas_n) << 1) | uint32_t(we_n);
    if (c == 3) row[chip][ba] = a & 0x1fff;
    else if (c == 5) { rq.push_back({cyc + uint64_t(lat), m[idx(chip, ba, row[chip][ba], a & 0x3ff)]}); reads++; }
    else if (c == 4) {
      uint16_t &w = m[idx(chip, ba, row[chip][ba], a & 0x3ff)];
      if (!((a >> 11) & 1)) w = uint16_t((w & 0xff00) | (dq_o & 0x00ff));
      if (!((a >> 12) & 1)) w = uint16_t((w & 0x00ff) | (dq_o & 0xff00));
      writes++;
      if (log_writes > 0) { log_writes--; std::printf("sdram write cyc %llu byte %07X data %04X mask %u\n", (unsigned long long)cyc, idx(chip, ba, row[chip][ba], a & 0x3ff) * 2, dq_o, (a >> 11) & 3); }
    }
  }
};
