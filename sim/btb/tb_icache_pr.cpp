// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench for the pre-read I-cache prototype (rtl/cpu/ppc_icache_pr.sv): a fetch
// stream in the core's protocol, sequential runs and jumps over 24 pages (so
// lines are evicted and refilled), memory never written. A jump is made as
// the core's soft redirect would: in the cycle the branch word is seen, the
// sequential request goes out with i_pv and the target's index; the cycle
// after (when that answer is seen) the target is requested with i_pre when it
// is in the branch's page. Every word must be memory's; reported: jump
// latency with and without the pre-read.
//   tb_icache_pr <seed> <fetches> [no_pre]

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <random>
#include "Vppc_icache_pr.h"
#include "verilated.h"
#include "cache_mem.h"

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	unsigned seed = argc > 1 ? std::atoi(argv[1]) : 1;
	long nf = argc > 2 ? std::atol(argv[2]) : 200000;
	bool no_pre = argc > 3 && std::atoi(argv[3]);
	std::mt19937 rng(seed);
	auto rnd = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };
	Memory mem;
	MemAgent ma; ma.rng = &rng; ma.mem = &mem;
	Vppc_icache_pr *t = new Vppc_icache_pr;
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
	long errors = 0, jumps = 0, pre_jumps = 0;
	uint64_t lat_pre = 0, lat_np = 0, n_np = 0, pre1 = 0;
	uint32_t pc = 0x00010000;
	// a working set that fits the cache: 4 pages of 32 targets each, and a cold
	// region over 24 pages for evictions now and then
	uint32_t hot[128];
	for (int k = 0; k < 128; k++) hot[k] = 0x00010000u + 0x1000u * (k / 32) + (uint32_t(rnd(0, 1023)) << 2);
	auto page = [&]() { return 0x00010000u + 0x1000u * rnd(0, 23); };
	auto issue = [&](uint32_t a, bool seq, bool pv, uint32_t pidx, bool pre) -> int {
		t->i_req = 1; t->i_addr = a; t->i_seq = seq; t->i_wimg = 0;
		t->i_pv = pv; t->i_pidx = pidx; t->i_pre = pre;
		tick();
		t->i_req = 0; t->i_pv = 0; t->i_pre = 0;
		int n = 1;
		while (!t->i_valid) { tick(); n++; if (n > 2000) { std::printf("hang at %08x\n", a); std::exit(1); } }
		if (t->i_data != mem.rd(a) && errors++ < 10)
			std::printf("fetch %08x: %08x, memory %08x (cycle %llu)\n", a, t->i_data, mem.rd(a), (unsigned long long)cyc);
		return n;
	};
	issue(pc, false, false, 0, false);
	for (long f = 0; f < nf; f++)
	{
		int const r = rnd(0, 99);
		if (r < 80 && (pc & 0x1C) != 0x1C) { pc += 4; issue(pc, (pc & 0xFFF) != 0, false, 0, false); continue; }
		// a jump: in page (a short branch) or to another page (a call)
		uint32_t tgt;
		if (r < 99) { int const k = rnd(0, 31) + 32 * int(((pc >> 12) - 0x10) & 3); tgt = r < 96 ? hot[k] : hot[rnd(0, 127)]; }
		else tgt = page() | (uint32_t(rnd(0, 1023)) << 2);
		bool const same = (tgt >> 12) == (pc >> 12);
		jumps++;
		if (!no_pre && rnd(0, 3))
		{	// the soft redirect: the sequential request with the pre-read, then the target
			bool const seq = ((pc + 4) & 0xFFF) != 0;
			issue(pc + 4, seq, true, (tgt >> 2) & 0x3FF, false);
			bool const pre = same && seq;
			int const n = issue(tgt, false, false, 0, pre);
			if (pre) { pre_jumps++; lat_pre += n; pre1 += n == 1; }
			else { lat_np += n; n_np++; }
		}
		else { int const n = issue(tgt, false, false, 0, false); lat_np += n; n_np++; }
		pc = tgt;
	}
	std::printf("pre-read I-cache seed %u: %ld fetches, %ld jumps, %ld with the pre-read: latency %.2f (%.1f%% in 1 cycle); others %.2f; %llu cycles; %s\n",
		seed, nf, jumps, pre_jumps, pre_jumps ? double(lat_pre) / pre_jumps : 0.0, pre_jumps ? 100.0 * pre1 / pre_jumps : 0.0,
		n_np ? double(lat_np) / n_np : 0.0, (unsigned long long)cyc, errors ? "FAIL" : "ok");
	delete t;
	return errors ? 1 : 0;
}
