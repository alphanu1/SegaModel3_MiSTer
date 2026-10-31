// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// Cycle model of rtl/cpu/ppc_btb.sv, written from its header: every output
// at every cycle. Each edge is computed from the state before it, then
// committed. Also the branch policy the core would run on top of it (the
// header's "Policy"), shared with the Supermodel CPI model's lv2_ designs.

#pragma once
#include <cstdint>
#include <vector>

struct btb_in
{
	uint32_t rd_pc = 0;	// [31:2] as a word address
	unsigned ctx = 0;
	bool cap = false, p_take = false, kill = false;
	bool w_en = false, w_v = false, w_reg = false;
	uint32_t w_pc = 0, w_tgt = 0, w_x = 0;
	unsigned w_ctx = 0;
	bool flush = false, rst = false;
};

struct btb_model
{
	struct ent { bool v = false; uint32_t tag = 0; unsigned ctx = 0; bool reg = false; uint32_t tgt = 0, x = 0; };
	int n, iw, xw;
	std::vector<ent> m;
	ent q;			// the entry read at the last edge
	uint32_t rpc_q = 0;
	bool rdw_q = false;
	bool sw_act = false;
	unsigned sw_cnt = 0;
	// outputs
	bool p_v = false, p_reg = false;
	uint32_t p_tgt = 0, p_x = 0;

	btb_model(int entries, int extra) : n(entries), iw(0), xw(extra), m(entries)
	{
		while ((1 << iw) < n) iw++;
		// the RAM's power-up content is unknown: garbage entries until the reset sweep
		for (auto &e : m) { e.v = true; e.tag = 0x5a5a5a5a; }
	}
	unsigned idx(uint32_t wpc) const { return wpc & (n - 1); }
	uint32_t tag(uint32_t wpc) const { return wpc >> iw; }
	bool busy() const { return sw_act; }

	void edge(btb_in const &in)
	{
		// compare cycle (state before the edge)
		bool const hit = q.v && q.tag == tag(rpc_q) && q.ctx == in.ctx && !rdw_q && !sw_act;
		bool np_v = p_v, np_reg = p_reg;
		uint32_t np_tgt = p_tgt, np_x = p_x;
		if (in.rst || in.flush || sw_act || in.kill) np_v = false;
		else if (in.cap && hit && (!p_v || in.p_take)) { np_v = true; np_tgt = q.tgt; np_reg = q.reg; }
		else if (in.p_take) np_v = false;
		if (xw && in.cap && hit && (!p_v || in.p_take)) np_x = q.x;	// loads whatever the kill
		// sweep
		bool nsw = sw_act; unsigned ncnt = sw_cnt;
		if (in.rst || in.flush) { nsw = true; ncnt = 0; }
		else if (sw_act) { ncnt = (sw_cnt + 1) & (n - 1); if (sw_cnt == unsigned(n - 1)) nsw = false; }
		// write
		bool const we = sw_act || (in.w_en && !in.rst && !in.flush);
		unsigned const wa = sw_act ? sw_cnt : idx(in.w_pc);
		ent wd;
		wd.v = in.w_v && !sw_act; wd.tag = tag(in.w_pc); wd.ctx = in.w_ctx; wd.reg = in.w_reg;
		wd.tgt = in.w_tgt; wd.x = xw ? in.w_x : 0;
		// read
		unsigned const ra = idx(in.rd_pc);
		ent nq = m[ra];
		bool const nrdw = we && wa == ra;
		// commit
		if (we) m[wa] = wd;
		q = nq; rdw_q = nrdw; rpc_q = in.rd_pc;
		sw_act = nsw; sw_cnt = ncnt;
		p_v = np_v; p_tgt = np_tgt; p_reg = np_reg; p_x = np_x;
	}
};

// The policy table without timing (what the CPI model's lv2_ designs run).
struct btb_policy_table
{
	struct ent { bool v = false; uint32_t pc = 0, tgt = 0; unsigned ctx = 0; bool reg = false; };
	std::vector<ent> t;
	unsigned mask;
	explicit btb_policy_table(int n) : t(n), mask(n - 1) {}
	ent *look(uint32_t wpc, unsigned ctx) { ent &e = t[wpc & mask]; return (e.v && e.pc == wpc && e.ctx == ctx) ? &e : nullptr; }
	void write(uint32_t wpc, bool v, unsigned ctx, bool reg, uint32_t tgt) { ent &e = t[wpc & mask]; e.v = v; e.pc = wpc; e.ctx = ctx; e.reg = reg; e.tgt = tgt; }
	void flush() { for (auto &e : t) e.v = false; }
};
