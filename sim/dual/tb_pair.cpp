// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench for rtl/cpu/ppc_pair_proto.sv (VAR 4, and VAR 1 / 2 order only): random
// words pushed one (or two, FETCH2) a cycle with random holds and clears;
// the instructions leaving ID must be the pushed stream in order, none lost
// or repeated, and a pair only where the rule allows (VAR 4: the second a
// branch the first does not feed through CR, LR or CTR).
//   tb_pair <seed> <cycles>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <random>
#include <deque>
#include <map>
#include "Vppc_pair_proto.h"
#include "verilated.h"

#ifndef FETCH2
#define FETCH2 0
#endif

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	unsigned seed = argc > 1 ? std::atoi(argv[1]) : 1;
	long cycles = argc > 2 ? std::atol(argv[2]) : 100000;
	std::mt19937 rng(seed);
	auto rnd = [&](unsigned m) { return unsigned(rng() % m); };
	Vppc_pair_proto *d = new Vppc_pair_proto;
	// words: addi, add, lwz, stw, cmpw (CR0 / CR7), bc on CR0 / CR7, b, bdnz, mtctr, blr
	auto word = [&]() -> uint32_t {
		unsigned r1 = rnd(32), r2 = rnd(32), r3 = rnd(32);
		switch (rnd(10))
		{
		case 0: return (14u << 26) | (r1 << 21) | (r2 << 16) | rnd(65536);
		case 1: return (31u << 26) | (r1 << 21) | (r2 << 16) | (r3 << 11) | (266u << 1);
		case 2: return (32u << 26) | (r1 << 21) | (r2 << 16) | rnd(65536);
		case 3: return (36u << 26) | (r1 << 21) | (r2 << 16) | rnd(65536);
		case 4: return (31u << 26) | ((rnd(2) ? 7u : 0u) << 23) | (r2 << 16) | (r3 << 11);
		case 5: return (16u << 26) | (12u << 21) | ((rnd(2) ? 28u : 2u) << 16) | (rnd(16384) << 2);
		case 6: return (18u << 26) | (rnd(1 << 24) << 2);
		case 7: return (16u << 26) | (16u << 21) | (rnd(16384) << 2);
		case 8: return (31u << 26) | (r1 << 21) | (9u << 16) | (467u << 1);
		default: return (19u << 26) | (20u << 21) | (16u << 1);
		}
	};
	std::deque<uint32_t> want;		// PCs pushed, not yet left
	std::map<uint32_t, uint32_t> wd;	// PC -> word
	uint32_t pcn = 0;
	long errors = 0, out = 0, pairs = 0;
	d->rst = 1; d->clk = 0; d->eval(); d->clk = 1; d->eval(); d->rst = 0;
	for (long c = 0; c < cycles; c++)
	{
		bool const clr = rnd(200) == 0;
		unsigned np = rnd(4) ? 1 : 0;
		if (FETCH2 && np && rnd(2)) np = 2;
		if (want.size() + np > 4) np = 0;
		d->clear = clr; d->hold = rnd(5) == 0;
		d->push = (np >= 1 ? 1 : 0) | (np >= 2 ? 2 : 0);
		uint32_t const x0 = word(), x1 = word();
		d->w_insn = uint64_t(x0) | (uint64_t(x1) << 32);
		d->w_pc = uint64_t(pcn) | (uint64_t(pcn + 4) << 32);
		d->clk = 0; d->eval();
		unsigned const pop = d->pop;
		d->clk = 1; d->eval();
		if (pop && !clr)
		{
			uint32_t const g[2] = { d->o_pc0, d->o_pc1 };
			for (unsigned k = 0; k < pop; k++)
			{
				if (want.empty() || want.front() != g[k])
					if (errors++ < 10) std::printf("cycle %ld: op %u left with pc %08x, expected %08x\n", c, k, g[k], want.empty() ? 0 : want.front());
				if (!want.empty()) want.pop_front();
			}
			out += pop; pairs += pop == 2;
#if VAR4
			if (pop == 2)
			{	// the rule: entry 1 a branch, entry 0 not; no CR field, CTR or LR from 0 to 1
				uint32_t const a = wd[g[0]], b = wd[g[1]];
				unsigned const pa = a >> 26, pb = b >> 26;
				bool const bbr = pb == 16 || pb == 18 || (pb == 19 && ((b >> 1) & 0x3ff) == 16);
				bool const abr = pa == 16 || pa == 18 || (pa == 19 && ((a >> 1) & 0x3ff) == 16);
				bool const acmp = pa == 31 && ((a >> 1) & 0x3ff) == 0, amtctr = pa == 31 && ((a >> 1) & 0x3ff) == 467;
				bool const bcr = pb == 16 && !((b >> 23) & 1), bctr = (pb == 16 || pb == 19) && !((b >> 23) & 1);
				bool const dep = (acmp && bcr && ((a >> 23) & 7) == ((b >> 18) & 7)) || (amtctr && bctr);
				if ((!bbr || abr || dep) && errors++ < 10) std::printf("cycle %ld: illegal pair %08x %08x\n", c, a, b);
			}
#endif
		}
		if (clr) want.clear();
		else { if (np >= 1) { want.push_back(pcn); wd[pcn] = x0; } if (np >= 2) { want.push_back(pcn + 4); wd[pcn + 4] = x1; } }
		pcn += 4 * np;
	}
	std::printf("pair proto seed %u: %ld cycles, %ld out, %ld pairs, %s\n", seed, cycles, out, pairs, errors ? "FAIL" : "ok");
	delete d;
	return errors ? 1 : 0;
}
