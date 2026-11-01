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
// Bench for ppc_icache's pre-read port (PRE = 1, study D-30, R61 design C), in
// the core's protocol: sequential runs and jumps over 24 pages (lines are
// evicted and refilled), memory never written. A jump to T is made as the
// core's BTB makes it: T's way and page are what an earlier answer for T gave
// (i_way, i_pa, i_arr), as the BTB entry stores them; the pre-read goes with
// a sequential request B (i_pv), and T is requested in the next cycle with
// i_pre. Some jumps present a wrong way or page (a stale entry), some have no
// pre-read, and some have the pre-read without T's request after it (the
// core's prediction killed). Every word must be memory's, every i_arr answer
// must carry its own page, and a pre-read T whose stored line is resident
// must be answered in one cycle. icbi and HID0[ICFI] invalidate now and then.
//   tb_icache_pre <seed> <fetches>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <random>
#include "Vppc_icache.h"
#include "verilated.h"
#include "cache_mem.h"

// (a target last fetched with I = 1 is stored with RSV_TAG as its page, as the core does: its
// pre-read must never hit, not even a way that is invalid, whose tag is RSV_TAG)
int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	unsigned seed = argc > 1 ? std::atoi(argv[1]) : 1;
	long nf = argc > 2 ? std::atol(argv[2]) : 200000;
	std::mt19937 rng(seed);
	auto rnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
	Memory mem;
	MemAgent ma; ma.rng = &rng; ma.mem = &mem;
	Vppc_icache *t = new Vppc_icache;
	uint64_t cyc = 0;
	auto tick = [&]() {
		ma.step(t->m_req, t->m_cmd, t->m_addr, t->m_be, 0);
		t->m_ack = ma.ack; t->m_rvalid = ma.rvalid; t->m_rdata = ma.rdata;
		t->clk = 0; t->eval(); t->clk = 1; t->eval(); cyc++;
	};
	t->rst = 1; t->i_req = 0; t->i_pv = 0; t->i_pre = 0; t->hid_ice = 1; t->hid_ilock = 0; t->hid_icfi = 0; t->icbi_req = 0;
	for (int i = 0; i < 4; i++) tick();
	t->rst = 0;
	for (int i = 0; i < 200; i++) tick();
	long errors = 0, jumps = 0, pre_jumps = 0, pre_fast = 0, stale = 0, stale_fast = 0, resident = 0, res_slow = 0, orphans = 0;
	struct Seen { int way; uint32_t pa; bool arr; };
	std::map<uint32_t, Seen> seen;                      // what the BTB would hold for each target
	uint32_t pc = 0x00010000;
	uint32_t hot[128];
	for (int k = 0; k < 128; k++) hot[k] = 0x00010000u + 0x1000u * (k / 32) + (uint32_t(rnd(0, 1023)) << 2);
	auto page = [&]() { return 0x00010000u + 0x1000u * rnd(0, 23); };
	auto settle = [&]() { for (int i = 0; i < 3; i++) tick(); };
	// one request; returns cycles to i_valid and records what the answer said
	auto issue = [&](uint32_t a, bool seq, bool pv, uint32_t tgt, int pway, uint32_t ppa, bool pre, unsigned wimg = 0) -> int {
		t->i_req = 1; t->i_addr = a; t->i_seq = seq; t->i_wimg = wimg;
		t->i_pv = pv; t->i_pidx = (tgt >> 2) & 0x3FF; t->i_pway = pway; t->i_ppa = ppa; t->i_pre = pre;
		tick();
		t->i_req = 0; t->i_pv = 0; t->i_pre = 0;
		int n = 1;
		while (!t->i_valid) { tick(); n++; if (n > 2000) { std::printf("hang at %08x\n", a); std::exit(1); } }
		if (t->i_data != mem.rd(a) && errors++ < 10)
			std::printf("fetch %08x: %08x, memory %08x (cycle %llu)\n", a, t->i_data, mem.rd(a), (unsigned long long)cyc);
		if (t->i_arr && t->i_pa != (a >> 12) && errors++ < 10)
			std::printf("fetch %08x: i_pa %05x from the arrays\n", a, t->i_pa);
		seen[a] = { int(t->i_way), uint32_t(t->i_pa), bool(t->i_arr) };
		return n;
	};
	issue(pc, false, false, 0, 0, 0, false);
	for (long f = 0; f < nf; f++)
	{
		bool const phase_a = f < nf / 5;               // first a working set of one page: it fits
		int const r = phase_a ? rnd(3, 999) : rnd(0, 999);
		if (r < 2) {                                     // icbi of a random set
			t->icbi_req = 1; t->icbi_set = rnd(0, 127);
			while (!t->icbi_ack) tick();
			t->icbi_req = 0; tick();
			continue;
		}
		if (r < 3) { t->hid_icfi = 1; tick(); t->hid_icfi = 0; for (int i = 0; i < 140; i++) tick(); continue; }
		if (r < 800 && (pc & 0x1C) != 0x1C) { pc += 4; issue(pc, (pc & 0xFFF) != 0, false, 0, 0, 0, false); continue; }
		uint32_t tgt;
		int const q = rnd(0, 99);
		if (phase_a) tgt = hot[rnd(0, 31)];
		else if (q < 96) { int const k = rnd(0, 31) + 32 * int(((pc >> 12) - 0x10) & 3); tgt = q < 92 ? hot[k] : hot[rnd(0, 127)]; }
		else tgt = page() | (uint32_t(rnd(0, 1023)) << 2);
		jumps++;
		uint32_t const b = pc + 4;
		bool const bseq = (b & 0xFFF) != 0 && (b & 0x1F) != 0;   // a sequential request inside the line
		auto it = seen.find(tgt);
		int const kind = rnd(0, 9);
		if (bseq && it != seen.end() && kind < 8)
		{	// the BTB's jump: pre-read with B, then T
			int way = it->second.way; uint32_t ppa = it->second.arr ? it->second.pa : 0xF00FFu;
			bool bad = false;
			if (kind == 0) { way ^= rnd(1, 3); bad = true; }
			if (kind == 1) { ppa = 0x80000u | uint32_t(rnd(0, 0x7FFFF)); bad = true; }   // a page never fetched
			int const nb = issue(b, true, true, tgt, way, ppa, false);
			if (kind == 2) { pc = b; orphans++; continue; }   // the prediction dropped: the stream goes on at B
			// is T's line resident in the stored way? (a probe through the tags is not possible
			// here; count the one-cycle answers instead, and require them for a fresh entry)
			// (B must have been a fast-path request: after a dropped pre-read, or a slow answer, the
			// next sequential word is not, and the pre-read is not taken)
			bool const fresh = !bad && it->second.arr && nb == 1;
			int const n = issue(tgt, false, false, 0, 0, 0, true);
			pre_jumps++; pre_fast += n == 1;
			if (bad) { stale++; stale_fast += n == 1; }
			if (fresh) { resident++; res_slow += n != 1; }
			if (fresh && phase_a && f > nf / 20 && n != 1) { if (errors++ < 10) std::printf("fresh pre-read of %08x in a resident set took %d cycles\n", tgt, n); }
		}
		else issue(tgt, false, false, 0, 0, 0, false, (!phase_a && rnd(0, 15) == 0) ? 4u : 0u);   // some cache-inhibited
		pc = tgt;
		if (rnd(0, 63) == 0) settle();
	}
	// a wrong way or page must never be answered in one cycle (no way holds a line twice, and the
	// wrong pages are never fetched); a fresh entry is answered in one cycle unless evicted
	// (a "stale" way can be the line's way again after a refill there: its words are checked like any)
	std::printf("pre-read I-cache seed %u: %ld fetches, %ld jumps, %ld pre-read (%.1f%% in 1 cycle), fresh %ld (%ld slower: evicted), "
		"stale %ld (%ld in 1 cycle), dropped %ld; %llu cycles; %s\n",
		seed, nf, jumps, pre_jumps, pre_jumps ? 100.0 * pre_fast / pre_jumps : 0.0, resident, res_slow, stale, stale_fast, orphans,
		(unsigned long long)cyc, errors ? "FAIL" : "ok");
	delete t;
	return errors ? 1 : 0;
}
