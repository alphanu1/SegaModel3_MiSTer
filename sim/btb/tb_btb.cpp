// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Bench for rtl/cpu/ppc_btb.sv against sim/btb/btb_model.h.
//
//   tb_btb random <seed> <cycles>
//       random inputs every cycle; every output compared with the cycle model
//   tb_btb trace <file> [r52_hits]
//       a branch trace from the Supermodel census (M3BRTRACE, 16-byte records):
//       1. the lv_ 256-entry BTB of the CPI model (every taken branch looked up
//          and written, hit when PC and target match), run through the RTL;
//          with r52_hits it must give exactly the CPI model's count over the
//          same window, which starts empty;
//       2. the built policy (rtl/cpu/ppc_btb.sv header) for designs C and D,
//          the RTL table against the model's table, branch by branch.
// Build with -GN=, -GXW=, -GSTYLE= (the bench reads N and XW from argv too).

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <random>
#include <string>
#include "Vppc_btb.h"
#include "verilated.h"
#include "btb_model.h"

#ifndef BTB_N
#define BTB_N 256
#endif
#ifndef BTB_XW
#define BTB_XW 0
#endif

static Vppc_btb *dut;
static btb_model *mdl;
static uint64_t cyc = 0, errors = 0;

static void apply(btb_in const &in)
{
	dut->rst = in.rst; dut->rd_pc = in.rd_pc & 0x3fffffff; dut->ctx = in.ctx; dut->cap = in.cap; dut->cur_pc = 0;
	dut->p_take = in.p_take; dut->kill = in.kill;
	dut->w_en = in.w_en; dut->w_v = in.w_v; dut->w_pc = in.w_pc & 0x3fffffff; dut->w_ctx = in.w_ctx;
	dut->w_reg = in.w_reg; dut->w_tgt = in.w_tgt & 0x3fffffff; dut->w_x = in.w_x;
	dut->flush = in.flush;
}

static void check(const char *where)
{
	bool bad = dut->p_v != mdl->p_v || dut->busy != mdl->busy();
	if (mdl->p_v && (dut->p_tgt != (mdl->p_tgt & 0x3fffffff) || dut->p_reg != mdl->p_reg || (BTB_XW && dut->p_x != mdl->p_x))) bad = true;
	if (bad && errors++ < 10)
		std::printf("MISMATCH %s cycle %llu: rtl p_v %d tgt %08x reg %d x %x busy %d / model p_v %d tgt %08x reg %d x %x busy %d\n", where,
			(unsigned long long)cyc, dut->p_v, dut->p_tgt, dut->p_reg, (unsigned)dut->p_x, dut->busy, mdl->p_v, mdl->p_tgt, mdl->p_reg, mdl->p_x, mdl->busy());
}

// one clock: inputs are applied, the edge taken in both, outputs compared
static void clock(btb_in const &in, const char *where = "")
{
	apply(in);
	dut->clk = 0; dut->eval();
	dut->clk = 1; dut->eval();
	mdl->edge(in);
	cyc++;
	check(where);
}

static void reset()
{
	btb_in r; r.rst = true;
	clock(r, "reset");
	btb_in z;
	while (dut->busy) clock(z, "sweep");
}

static int run_random(unsigned seed, uint64_t n)
{
	std::mt19937 rng(seed);
	auto rnd = [&](unsigned m) { return unsigned(rng() % m); };
	// a pool of 80 addresses: 8 base, 8 sharing their index with a random tag,
	// 8 differing from them in one tag bit (any bit, the top included), and 56
	// spread over the table
	uint32_t pool[80];
	for (int k = 0; k < 8; k++) pool[k] = rng() & 0x3fffffff;
	for (int k = 8; k < 16; k++) pool[k] = (pool[k - 8] & (BTB_N - 1)) | (rng() & ~uint32_t(BTB_N - 1) & 0x3fffffff);
	int iw = 0; while ((1 << iw) < BTB_N) iw++;
	for (int k = 16; k < 24; k++) pool[k] = pool[k - 16] ^ (1u << (iw + rnd(30 - iw)));
	for (int k = 24; k < 80; k++) pool[k] = rng() & 0x3fffffff;
	auto pick = [&]() { return rnd(2) ? pool[rnd(24)] : pool[24 + rnd(56)]; };
	reset();
	uint64_t caps = 0, hits = 0;
	for (uint64_t i = 0; i < n; i++)
	{
		btb_in in;
		in.rd_pc = rnd(8) ? pick() : (rng() & 0x3fffffff);
		in.ctx = rnd(4) ? 0 : rnd(4);
		in.cap = rnd(3) != 0;
		in.p_take = rnd(3) == 0;
		in.kill = rnd(16) == 0;
		in.w_en = rnd(3) == 0;
		in.w_v = rnd(5) != 0;
		in.w_pc = rnd(6) ? pick() : in.rd_pc;
		in.w_ctx = rnd(4) ? 0 : rnd(4);
		in.w_reg = rnd(4) == 0;
		in.w_tgt = rng() & 0x3fffffff;
		in.w_x = BTB_XW ? (rng() & ((1u << BTB_XW) - 1)) : 0;
		in.flush = rnd(4000) == 0;
		in.rst = rnd(20000) == 0;
		bool const was = mdl->p_v;
		clock(in, "random");
		caps += in.cap; hits += !was && mdl->p_v;
		if (errors > 10) break;
	}
	std::printf("random seed %u: %llu cycles, %llu captures, %llu predictions, %s\n", seed, (unsigned long long)n,
		(unsigned long long)caps, (unsigned long long)hits, errors ? "FAIL" : "ok");
	return errors ? 1 : 0;
}

struct rec { uint32_t pc, npc, op; uint16_t n; uint8_t fl, kind; };

static bool is_refetch(uint32_t op)
{
	unsigned const p = op >> 26, x = (op >> 1) & 0x3ff;
	return p == 17 || (p == 19 && (x == 150 || x == 50)) || (p == 31 && (x == 146 || x == 210 || x == 242 || x == 306 || x == 978 || x == 1010));
}

// a lookup through the RTL: three cycles; returns p_v, with target and reg
static bool rtl_lookup(uint32_t wpc, unsigned ctx, uint32_t &tgt, bool &reg)
{
	btb_in a; a.rd_pc = wpc; clock(a, "lookup");
	btb_in b; b.rd_pc = wpc + 1; b.cap = true; b.ctx = ctx; clock(b, "lookup");
	bool const v = dut->p_v;
	tgt = dut->p_tgt; reg = dut->p_reg;
	return v;
}
// the cycle after a lookup: clear the prediction, and write if asked
static void rtl_after(bool wr, bool v, uint32_t wpc, unsigned ctx, bool reg, uint32_t tgt)
{
	btb_in c; c.p_take = true; c.rd_pc = wpc ^ 0x155;
	c.w_en = wr; c.w_v = v; c.w_pc = wpc; c.w_ctx = ctx; c.w_reg = reg; c.w_tgt = tgt;
	clock(c, "write");
}

static int run_trace(const char *path, long long expect_r52)
{
	std::FILE *f = std::fopen(path, "rb");
	if (!f) { std::printf("cannot open %s\n", path); return 2; }
	std::vector<rec> rs;
	{
		uint8_t b[16];
		while (std::fread(b, 16, 1, f) == 1)
		{
			rec r; std::memcpy(&r.pc, b, 4); std::memcpy(&r.npc, b + 4, 4); std::memcpy(&r.op, b + 8, 4);
			std::memcpy(&r.n, b + 12, 2); r.fl = b[14]; r.kind = b[15]; rs.push_back(r);
		}
		std::fclose(f);
	}
	uint64_t nbr = 0, ntk = 0;
	for (auto const &r : rs) if (r.kind < 4) { nbr++; ntk += r.fl & 1; }
	std::printf("trace %s: %zu records, %llu branches, %llu taken\n", path, rs.size(), (unsigned long long)nbr, (unsigned long long)ntk);

	// 1. the lv_ BTB (direct mapped, every taken branch, PC and target)
	int fails = 0;
	if (BTB_N == 256 || true)
	{
		reset();
		btb_policy_table mt(BTB_N);
		uint64_t look = 0, hit_r = 0, hit_m = 0;
		for (auto const &r : rs)
		{
			if (r.kind >= 4 || !(r.fl & 1)) continue;
			uint32_t const w = r.pc >> 2, t = r.npc >> 2;
			uint32_t tg; bool rg;
			bool const v = rtl_lookup(w, 0, tg, rg);
			bool const hr = v && tg == t;
			auto *e = mt.look(w, 0);
			bool const hm = e && e->tgt == t;
			look++; hit_r += hr; hit_m += hm;
			if (hr != hm && errors++ < 10) std::printf("R52 MISMATCH at branch %llu pc %08x: rtl %d model %d\n", (unsigned long long)look, r.pc, hr, hm);
			rtl_after(true, true, w, 0, false, t);
			mt.write(w, true, 0, false, t);
		}
		std::printf("lv_ BTB %d entries: taken %llu, hits rtl %llu, model %llu (%.2f%%)", BTB_N, (unsigned long long)look,
			(unsigned long long)hit_r, (unsigned long long)hit_m, 100.0 * hit_m / (look ? look : 1));
		if (expect_r52 >= 0) std::printf(", CPI model %lld: %s", expect_r52, (long long)hit_m == expect_r52 && hit_r == hit_m ? "match" : "DIFFERENT");
		std::printf("\n");
		if (hit_r != hit_m || (expect_r52 >= 0 && (long long)hit_m != expect_r52)) fails++;
	}

	// 2. the built policy, designs C (k = 0, 2) and D (k = 1, 3); k = 2, 3 take the
	// direction from the entry (every taken branch written; a hit not taken invalidates)
	for (int k = 0; k < 4; k++)
	{
		bool const dyn = k >= 2, dc = k == 0 || k == 2;
		reset();
		btb_policy_table mt(BTB_N);
		struct { uint32_t pc = 0; bool redir = true, seq = false; } last;
		uint64_t look = 0, hit = 0, good = 0, wrong = 0, ntk_hit = 0, taken = 0, flushes = 0, cyc0 = cyc;
		uint64_t good_reg = 0, taken_reg = 0, taken_stat = 0, wrong_reg = 0, incap_taken = 0;
		for (auto const &r : rs)
		{
			bool const is_br = r.kind < 4;
			bool const after = r.n == 1 && last.pc == r.pc - 4;	// the record before is pc - 4
			bool redir = false, seq = false;
			if (is_br)
			{
				bool const tk = r.fl & 1, seq1 = r.fl & 2, seq2 = r.fl & 4, stat = r.fl & 32, reg = r.kind >= 2;
				unsigned const ctx = (r.fl >> 3) & 3;
				bool const prev_redir = after && last.redir;
				seq = seq1 && !prev_redir;
				bool const prev_seq = after ? last.seq : (seq2 && !(r.n == 2 && last.redir));
				bool const capable = dc ? (seq && prev_seq) : seq;
				uint32_t const w = r.pc >> 2, t = r.npc >> 2;
				bool hard = false;
				bool wr = false, wv = false; uint32_t wt = 0;
				if (capable)
				{
					look++;
					uint32_t tg; bool rg;
					bool const hr = rtl_lookup(w, ctx, tg, rg);
					auto *e = mt.look(w, ctx);
					if (hr != (e != nullptr) || (e && (tg != (e->tgt & 0x3fffffff) || rg != e->reg)))
						if (errors++ < 10) std::printf("POLICY MISMATCH design %c%s pc %08x: rtl %d %08x %d model %d\n", dc ? 'C' : 'D', dyn ? "y" : "", r.pc, hr, tg, rg, e != nullptr);
					if (e)
					{
						hit++;
						if (tk && e->tgt == t) { good++; good_reg += reg; }
						else { wrong++; wrong_reg += reg; hard = true; if (!tk) { ntk_hit++; if (reg || dyn) { wr = true; wv = false; wt = e->tgt; } } }
					}
					else if (reg) hard = tk;
					else hard = stat ? !tk : tk;
				}
				else hard = reg ? tk : (stat ? !tk : tk);
				if (tk) { taken++; taken_reg += reg; taken_stat += !reg && stat; incap_taken += !capable && (reg || stat); }
				if (tk && (stat || reg || dyn)) { wr = true; wv = true; wt = t; }
				if (capable) rtl_after(wr, wv, w, ctx, reg, wt);
				else if (wr) { btb_in c; c.w_en = true; c.w_v = wv; c.w_pc = w; c.w_ctx = ctx; c.w_reg = reg; c.w_tgt = wt; clock(c, "write"); }
				if (wr) mt.write(w, wv, ctx, reg, wt);
				redir = tk || hard;
			}
			else
			{
				redir = is_refetch(r.op);
				if (dc && r.kind == 8)
				{
					flushes++; mt.flush();
					btb_in fl; fl.flush = true; clock(fl, "flush");
					btb_in z; while (dut->busy) clock(z, "sweep");
				}
			}
			last.pc = r.pc; last.redir = redir; last.seq = seq;
			if (errors > 10) break;
		}
		std::printf("design %c%s, %d entries: lookups %llu (%.1f%% of taken), hits %llu, right %llu (%.2f%% of taken), wrong %llu (not taken %llu), flushes %llu, %llu cycles\n",
			dc ? 'C' : 'D', dyn ? "y" : "", BTB_N, (unsigned long long)look, 100.0 * look / (taken ? taken : 1), (unsigned long long)hit, (unsigned long long)good,
			100.0 * good / (taken ? taken : 1), (unsigned long long)wrong, (unsigned long long)ntk_hit, (unsigned long long)flushes, (unsigned long long)(cyc - cyc0));
		std::printf("  taken: static-taken b / bc %llu, bclr / bcctr %llu; right hits on bclr / bcctr %llu, wrong %llu; taken and eligible but not looked up %llu\n",
			(unsigned long long)taken_stat, (unsigned long long)taken_reg, (unsigned long long)good_reg, (unsigned long long)wrong_reg, (unsigned long long)incap_taken);
	}
	std::printf("%s\n", errors || fails ? "FAIL" : "ok");
	return errors || fails ? 1 : 0;
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	dut = new Vppc_btb;
	mdl = new btb_model(BTB_N, BTB_XW);
	int rc = 2;
	if (argc >= 4 && !std::strcmp(argv[1], "random")) rc = run_random(unsigned(std::atoi(argv[2])), std::strtoull(argv[3], nullptr, 10));
	else if (argc >= 3 && !std::strcmp(argv[1], "trace")) rc = run_trace(argv[2], argc >= 4 ? std::atoll(argv[3]) : -1);
	else std::printf("usage: tb_btb random <seed> <cycles> | trace <file> [r52_hits]\n");
	delete dut; delete mdl;
	return rc;
}
