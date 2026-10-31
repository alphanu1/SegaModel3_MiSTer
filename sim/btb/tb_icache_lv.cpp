// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Directed fetch timing of rtl/cpu/ppc_icache.sv, for pricing the R52 levers
// against what is built: with every line resident, a jump (a request without
// i_seq) to word k of a line, then 24 sequential words; and jumps alone
// (same line, a new line in the set's MRU way, a new line in another way).
// Reports cycles from request to i_valid for the jump and the extra cycles
// the sequential words take over one a word.
//   tb_icache_lv

#include <cstdio>
#include <cstdint>
#include <random>
#include "Vppc_icache.h"
#include "verilated.h"
#include "cache_mem.h"

static Vppc_icache *t;
static MemAgent ma;
static uint64_t cyc = 0;
static void tick()
{
	ma.step(t->m_req, t->m_cmd, t->m_addr, t->m_be, 0);
	t->m_ack = ma.ack; t->m_rvalid = ma.rvalid; t->m_rdata = ma.rdata;
	t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++;
}
// one fetch in the core's protocol; returns cycles from request to i_valid
static int fetch(uint32_t a, bool seq)
{
	t->i_req = 1; t->i_addr = a; t->i_seq = seq; t->i_wimg = 0;
	tick();
	t->i_req = 0;
	int n = 1;
	while (!t->i_valid) { tick(); n++; if (n > 1000) { std::printf("hang at %08x\n", a); return -1; } }
	return n;
}
// a run: requests issued back to back, the next in the cycle the answer is seen
static uint64_t run(uint32_t a0, int nw, bool first_seq)
{
	uint64_t c0 = cyc;
	for (int i = 0; i < nw; i++) fetch(a0 + 4 * i, i ? true : first_seq);
	return cyc - c0;
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	std::mt19937 rng(1);
	Memory mem;
	t = new Vppc_icache;
	ma.rng = &rng; ma.mem = &mem; ma.ack_max = 0; ma.lat_wide = 0; ma.gap_max = 0; ma.lat_lo = ma.lat_hi = 12;
	t->rst = 1; t->i_req = 0; t->hid_ice = 1; t->hid_ilock = 0; t->hid_icfi = 0; t->icbi_req = 0;
	for (int i = 0; i < 4; i++) tick();
	t->rst = 0;
	for (int i = 0; i < 200; i++) tick();
	// warm: lines at 0x1000-0x13FF (sets 0-31, way A) and 0x5000-0x53FF (the same sets, way B)
	run(0x1000, 256, false);
	run(0x5000, 256, false);
	for (int i = 0; i < 50; i++) tick();
	std::printf("jump to word k of a resident line, then 24 sequential words (ideal: jump latency + 24)\n");
	for (int k = 0; k < 8; k++)
	{
		run(0x5000, 8, false);					// make 0x5000's lines MRU in their sets
		uint32_t const a = 0x1100 + 4 * k;		// way A, not MRU
		uint64_t const c0 = cyc;
		int const j = fetch(a, false);
		uint64_t const s = run(a + 4, 24, true);
		std::printf("  k=%d: jump %d cycles, 24 sequential words %llu cycles (+%llu), total %llu\n", k, j,
			(unsigned long long)s, (unsigned long long)(s - 24), (unsigned long long)(cyc - c0));
		run(0x1100 + 4 * k, 1, false);			// leave a known state
	}
	// jumps alone
	run(0x1200, 4, false);
	int const same = fetch(0x1204, false);			// same line as the last one served
	run(0x1300, 4, false);
	int const mru = fetch(0x1200, false);			// a new line whose set's MRU way it is
	run(0x5200, 4, false);
	int const other = fetch(0x1200, false);		// a new line, not its set's MRU way
	std::printf("jump latency: same line %d, new line in MRU way %d, new line in another way %d (sequential: 1)\n", same, mru, other);
	delete t;
	return 0;
}
