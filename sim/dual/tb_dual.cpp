// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench for the dual-issue prototype (rtl/cpu/ppc_dual_proto.sv): random adds,
// subtracts and logical ops, paired under the pairing rule (the younger op of
// a pair reads nothing the older writes; one writer per register; with one
// write port at most one writer per pair), every result leaving WB compared
// with in-order execution. Build with the prototype's -G parameters and
// -DDUAL=, -DBW2=, -DPMUX=.
//   tb_dual <seed> <cycles>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <random>
#include <deque>
#include "Vppc_dual_proto.h"
#include "verilated.h"

#ifndef DUAL
#define DUAL 1
#endif
#ifndef BW2
#define BW2 1
#endif
#ifndef PMUX
#define PMUX 0
#endif

struct uop { bool v = false, rdw = false, xs = false, yreg = false; int rd = 0, ra = 0, rb = 0, rs = 0, kind = 0; uint32_t imm = 0, ltt = 0; };

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	unsigned seed = argc > 1 ? std::atoi(argv[1]) : 1;
	long cycles = argc > 2 ? std::atol(argv[2]) : 100000;
	std::mt19937 rng(seed);
	auto rnd = [&](unsigned m) { return unsigned(rng() % m); };
	Vppc_dual_proto *d = new Vppc_dual_proto;
	uint32_t gpr[32] = {};
	struct exp { bool v; uint32_t a, b; bool va, vb; };
	std::deque<exp> pipe;			// results expected at WB, 4 cycles after entering ID's input
	long errors = 0, checked = 0, pairs = 0;
	for (long c = 0; c < cycles + 8; c++)
	{
		uop u[2];
		bool const init = c < 32;
		int nslot = (DUAL && !init && rnd(3)) ? 2 : 1;
		bool const b_older = nslot == 2 && rnd(2);
		for (int s = 0; s < nslot; s++)
		{
			uop &x = u[s];
			x.v = c < cycles;
			x.rd = init ? int(c) : int(rnd(32)); x.rdw = init || rnd(8) != 0;
			x.ra = rnd(32); x.rb = rnd(32); x.rs = rnd(32); x.xs = rnd(2); x.yreg = !init && rnd(2);
			x.imm = rng(); x.kind = init ? 2 : rnd(3); x.ltt = init ? 0xA : rnd(16);
		}
		if (nslot == 2)
		{	// the pairing rule
			uop &o = b_older ? u[1] : u[0], &y = b_older ? u[0] : u[1];
			auto reads = [&](uop const &q, int r) { return (q.xs ? q.rs : q.ra) == r || (q.yreg && q.rb == r); };
			if (o.rdw && reads(y, o.rd)) o.rdw = false;
			if (o.rdw && y.rdw && o.rd == y.rd) y.rdw = false;
			if (!BW2 && o.rdw && y.rdw) y.rdw = false;
			if (PMUX)
			{	// at most three GPRs read between them: drop y-register reads until it fits
				auto n = [&](uop const &q) { return 1 + (q.yreg ? 1 : 0); };
				if (n(u[0]) + n(u[1]) > 3) { u[1].yreg = false; }
			}
			pairs++;
		}
		// slot B's operand x is rA unless xs; slot A likewise
		// drive
		for (int s = 0; s < 2; s++)
		{
			d->n_v = (d->n_v & ~(1u << s)) | ((u[s].v ? 1u : 0u) << s);
			d->n_rdw = (d->n_rdw & ~(1u << s)) | ((u[s].rdw && u[s].v ? 1u : 0u) << s);
			d->n_xs = (d->n_xs & ~(1u << s)) | ((u[s].xs ? 1u : 0u) << s);
			d->n_yreg = (d->n_yreg & ~(1u << s)) | ((u[s].yreg ? 1u : 0u) << s);
			d->n_ld = 0; d->n_rc = 0;
			d->n_rd = (d->n_rd & ~(31u << (5 * s))) | (unsigned(u[s].rd) << (5 * s));
			d->n_ra = (d->n_ra & ~(31u << (5 * s))) | (unsigned(u[s].ra) << (5 * s));
			d->n_rb = (d->n_rb & ~(31u << (5 * s))) | (unsigned(u[s].rb) << (5 * s));
			d->n_rs = (d->n_rs & ~(31u << (5 * s))) | (unsigned(u[s].rs) << (5 * s));
			d->n_imm = (d->n_imm & ~(0xffffffffull << (32 * s))) | (uint64_t(u[s].imm) << (32 * s));
			// op 0 add / 1 subf on the adder (sel 1), 2 logical (sel 0, table)
			unsigned const op = u[s].kind == 1 ? 1 : 0, sel = u[s].kind == 2 ? 0 : 1, ltt = u[s].kind == 2 ? u[s].ltt : 0;
			d->n_op = (d->n_op & ~(63u << (6 * s))) | (op << (6 * s));
			d->n_sel = (d->n_sel & ~(127u << (7 * s))) | (sel << (7 * s));
			d->n_ltt = (d->n_ltt & ~(15u << (4 * s))) | (ltt << (4 * s));
			d->n_mask = 0;
		}
		if (PMUX)
		{	// ports: 0 A's x field, 1 A's rB, 2 B's x field (or B's rB when A has two)
			// fields: 0 A.ra 1 A.rb 2 A.rs 3 B.ra 4 B.rb 5 B.rs
			unsigned f[3] = { u[0].xs ? 2u : 0u, 1u, u[1].xs ? 5u : 3u }, pidx[4] = { 0, 1, 2, 2 };
			if (!u[0].yreg) { f[1] = 4; pidx[3] = 1; }
			else pidx[3] = 2;	// B has no y register then (dropped above)
			d->n_pmap = f[0] | (f[1] << 3) | (f[2] << 6);
			d->n_pidx = pidx[0] | (pidx[1] << 2) | (pidx[2] << 4) | (pidx[3] << 6);
		}
		d->b_older = b_older;
		d->ld_word = 0; d->ld_word_b = 0;
		// reference: in order
		exp e{ c < cycles, 0, 0, false, false };
		auto exec = [&](uop const &q) {
			uint32_t const x = q.xs ? gpr[q.rs] : gpr[q.ra], y = q.yreg ? gpr[q.rb] : q.imm;
			uint32_t r;
			if (q.kind == 0) r = x + y; else if (q.kind == 1) r = ~x + y;
			else { r = 0; for (int i = 0; i < 32; i++) r |= ((q.ltt >> ((((x >> i) & 1) << 1) | ((y >> i) & 1))) & 1u) << i; }
			return r;
		};
		uint32_t ra_ = 0, rb_ = 0;
		if (nslot == 2 && b_older) { rb_ = exec(u[1]); if (u[1].rdw && u[1].v) gpr[u[1].rd] = rb_; ra_ = exec(u[0]); if (u[0].rdw && u[0].v) gpr[u[0].rd] = ra_; }
		else { ra_ = exec(u[0]); if (u[0].rdw && u[0].v) gpr[u[0].rd] = ra_; if (nslot == 2) { rb_ = exec(u[1]); if (u[1].rdw && u[1].v) gpr[u[1].rd] = rb_; } }
		e.va = u[0].v && u[0].rdw; e.vb = nslot == 2 && u[1].v && u[1].rdw; e.a = ra_; e.b = rb_;
		if (!BW2 && nslot == 2 && e.vb) { e.a = rb_; e.va = true; e.vb = false; }
		if (nslot == 1) { d->n_v &= 1; d->n_rdw &= 1; }
		pipe.push_back(e);
		d->clk = 0; d->eval(); d->clk = 1; d->eval();
		// a uop given on this edge is in ID next cycle and leaves WB four edges on
		if (pipe.size() > 3)
		{
			exp const &w = pipe.front();
			d->clk = 0; d->eval();
			if (w.v && w.va && d->wb_a != w.a && errors++ < 10) std::printf("cycle %ld: slot A WB %08x, expected %08x\n", c, d->wb_a, w.a);
			if (w.v && w.vb && d->wb_b != w.b && errors++ < 10) std::printf("cycle %ld: slot B WB %08x, expected %08x\n", c, d->wb_b, w.b);
			checked += w.va + w.vb;
			pipe.pop_front();
		}
	}
	std::printf("dual proto seed %u: %ld cycles, %ld pairs, %ld results checked, %s\n", seed, cycles, pairs, checked, errors ? "FAIL" : "ok");
	delete d;
	return errors ? 1 : 0;
}
