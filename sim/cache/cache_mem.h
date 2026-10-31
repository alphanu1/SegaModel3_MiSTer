// SPDX-License-Identifier: GPL-3.0-or-later
//
// Memory controller model for the cache benches: the port of
// rtl/cpu/ppc_cache_pkg.sv. Random acknowledge delay, first-word latency and
// beat spacing; critical word first, wrapping; one transaction at a time.
// Every single access is logged, in the order the port takes it.

#pragma once
#include <cstdint>
#include <cstdio>
#include <random>
#include <unordered_map>
#include <vector>

static inline uint32_t mem_init(uint32_t wa) {          // a word's content before anyone writes it
  uint32_t x = wa * 0x9E3779B1u + 0x7F4A7C15u;
  x ^= x >> 15; x *= 0x2C1B3C6Du; x ^= x >> 12;
  return x;
}

struct Memory {
  std::unordered_map<uint32_t, uint32_t> w;              // by word address
  uint32_t rd(uint32_t a) const { auto it = w.find(a >> 2); return it == w.end() ? mem_init(a >> 2) : it->second; }
  void wr(uint32_t a, uint32_t d, unsigned be) {
    uint32_t o = rd(a), m = 0;
    for (int l = 0; l < 4; l++) if (be & (8 >> l)) m |= 0xFFu << (24 - 8 * l);
    w[a >> 2] = (o & ~m) | (d & m);
  }
};

struct Single { bool we; uint32_t addr; unsigned be; uint32_t data; };

struct MemAgent {
  // configuration
  int lat_lo = 12, lat_hi = 15;       // first word, cycles after the acknowledge
  int lat_wide = 10;                  // percent of transactions with a latency drawn from 1..40
  int gap_max = 1;                    // extra cycles between beats
  int ack_max = 3;
  std::mt19937 *rng = nullptr;
  Memory *mem = nullptr;
  std::vector<Single> log;
  // statistics
  uint64_t n_fill = 0, n_wb = 0, n_rd = 0, n_wr = 0;
  // error
  int err = 0;

  // state
  enum { IDLE, ACKWAIT, RBEATS, WBEATS, SRD } st = IDLE;
  int cnt = 0, delay = 0, beat = 0;
  unsigned cmd = 0; uint32_t addr = 0; unsigned be = 0;

  int rnd(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(*rng); }
  int first_lat() { return rnd(0, 99) < lat_wide ? rnd(1, 40) : rnd(lat_lo, lat_hi); }

  // outputs for this cycle, computed from the DUT's current outputs
  bool ack = false, rvalid = false, wnext = false; uint32_t rdata = 0;

  // call once per cycle before the edge
  void step(bool m_req, unsigned m_cmd, uint32_t m_addr, unsigned m_be, uint32_t m_wdata) {
    ack = rvalid = wnext = false; rdata = 0xDEADBEEF;
    switch (st) {
      case IDLE:
        if (m_req) { st = ACKWAIT; delay = rnd(0, ack_max); }
        else break;
        /* fall through */
      case ACKWAIT:
        if (!m_req) { std::printf("MEM: request dropped before acknowledge\n"); err++; st = IDLE; break; }
        if (delay-- > 0) break;
        ack = true; cmd = m_cmd; addr = m_addr; be = m_be;
        if (cmd == 0) { st = RBEATS; beat = 0; delay = first_lat(); n_fill++; }
        else if (cmd == 1) {
          st = WBEATS; beat = 0; delay = rnd(1, 4); n_wb++;
          if (addr & 31) { std::printf("MEM: line write not aligned %08X\n", addr); err++; }
        } else if (cmd == 2) {
          st = SRD; delay = first_lat(); n_rd++;
          log.push_back({false, addr & ~3u, be, 0});
        } else {
          n_wr++;
          log.push_back({true, addr & ~3u, be, m_wdata});
          mem->wr(addr, m_wdata, be);
          st = IDLE;
        }
        break;
      case RBEATS:
        if (--delay > 0) break;
        rvalid = true;
        rdata = mem->rd((addr & ~31u) | ((((addr >> 2) + beat) & 7) << 2));
        if (++beat == 8) st = IDLE; else delay = 1 + rnd(0, gap_max);
        break;
      case WBEATS:
        if (--delay > 0) break;
        wnext = true;
        mem->wr((addr & ~31u) | (beat << 2), m_wdata, 0xF);
        if (++beat == 8) st = IDLE; else delay = 1 + rnd(0, gap_max);
        break;
      case SRD:
        if (--delay > 0) break;
        rvalid = true; rdata = mem->rd(addr);
        st = IDLE;
        break;
    }
  }
  bool idle() const { return st == IDLE; }
};
