// SPDX-License-Identifier: GPL-3.0-or-later
// Sega Model 3 core for MiSTer FPGA -- Copyright (C) 2026 alphanu1
//
// tb_scsp_dual: one scsp_dual against two m2_scsp, a chip each, on the same
// register and sound-RAM write streams (scsp_dual_harness.sv).
//
// STEP (default): writes land at each chip's own sample boundary -- for an
// m2_scsp between its samples (step_mode), for the dual where it waits for
// that chip (at_go, at_ch) -- and every output sample of both chips, every
// slot's sample, every 68000 read's answer, chip 0's interrupt level at each
// boundary and its MIDI-out bytes are compared.
//
// FREE=1: both sides run free (out_ready held), each chip's writes for sample
// n pushed once that chip has made sample n - 1. Reports the clocks a sample
// takes (a pair, for the dual) -- the clock budget -- and nothing else.
//
//   SCSP_CAP=<dir>  <dir>/c0, <dir>/c1: regs.bin, ramw.bin (c0/midi.txt)
//   SCSP_N=<samples> (default 44100)
//   LAT_LO / LAT_HI  memory latency, clocks (default 6 / 40); SCSP_SEED
//   MHZ=<clock>      the budget line's clock (default 100)
#include "Vscsp_dual_harness.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>

static Vscsp_dual_harness *t;
#ifdef PROF
// PROF builds (--public-flat-rw): where each engine's clocks go, by microcode
// address. Single (scsp_uc.svh): 4 DISPATCH, 5 / 12 the DSP waits; dual
// (scsp_d_uc.svh): 21 DISPATCH, 2 DSPDONE, 22 DSPIDLE. Both: 28..47 the
// 68000's calls, 48.. a slot's program, 59 / 98 its FETCHED wait.
#include "Vscsp_dual_harness___024root.h"
struct Prof { uint64_t slot = 0, fetch = 0, disp = 0, call = 0, dspwait = 0, other = 0, op = 0, dsp = 0; };
static Prof pR, pD;
static void prof_one(Prof &p, int u, bool hold, bool opb, bool dspb, bool dual) {
	if (dspb) p.dsp++;
	if (opb) { p.op++; return; }
	if (u >= 48) { if ((u == 59 || u == 98) && hold) p.fetch++; else p.slot++; }
	else if (u >= 28) p.call++;
	else if (u == (dual ? 21 : 4)) p.disp++;
	else if (dual ? (u == 2 || u == 22) : (u == 5 || u == 12)) { if (hold) p.dspwait++; else p.other++; }
	else p.other++;
}
#define RP(x) t->rootp->scsp_dual_harness__DOT__g_ref__BRA__0__KET____DOT__u__DOT__##x
#define DP(x) t->rootp->scsp_dual_harness__DOT__u_d__DOT__##x
static void prof() {
	prof_one(pR, RP(upc), RP(hold), RP(op_busy), RP(dsp_busy), false);
	prof_one(pD, DP(upc), DP(hold), DP(op_busy), DP(dsp_busy), true);
}
static void prof_print(const char *nm, const Prof &p, double per) {
	printf("  %s, clocks a sample: slots %.0f + fetch waits %.1f, dispatch %.0f, DSP waits %.0f, 68000 calls %.1f / ops %.1f, rest %.0f; DSP busy %.0f\n",
	       nm, p.slot / per, p.fetch / per, p.disp / per, p.dspwait / per, p.call / per, p.op / per, p.other / per, p.dsp / per);
}
#endif

struct Rec { uint32_t n; uint16_t off, data, mask, pad; };
struct RamW { uint32_t n, a, dm; };

static uint64_t cyc = 0;
static uint16_t ramR[2][0x40000], ramD[2][0x40000];
static uint32_t rng_s = 1;
static int lat_lo = 6, lat_hi = 40;
static int lat() { rng_s = rng_s * 1103515245u + 12345u; return lat_lo + int((rng_s >> 8) % unsigned(lat_hi - lat_lo + 1)); }

// packed [1:0][W-1:0] ports: element c
template <class T> static uint64_t el(T v, int w, int c) { return (uint64_t(v) >> (w * c)) & ((w == 64) ? ~0ull : ((1ull << w) - 1)); }
template <class T> static void setel(T &v, int w, int c, uint64_t x) {
	uint64_t m = ((1ull << w) - 1) << (w * c);
	v = T((uint64_t(v) & ~m) | ((x << (w * c)) & m));
}
static void set64(VlWide<4> &v, int c, uint64_t x) { v[2 * c] = uint32_t(x); v[2 * c + 1] = uint32_t(x >> 32); }
static int bit(uint64_t v, int c) { return int((v >> c) & 1); }

struct Port { bool busy = false; int wait = 0; bool was = false; };
static Port rsm[2], rdm[2], dsm, ddm;
static uint64_t line_of(const uint16_t *ram, uint32_t widx) {
	widx &= ~3u; uint64_t v = 0;
	for (int i = 0; i < 4; i++) v |= uint64_t(ram[(widx + i) & 0x3ffff]) << (16 * i);
	return v;
}

struct Out { int32_t rl, rr; int16_t l, r; };
static std::deque<Out> outR[2], outD[2];
static std::deque<std::pair<int, int32_t>> slR[2], slD[2];
static std::deque<int> moR, moD;
static uint64_t nR[2], nD[2];
static uint64_t lastR[2], lastD, gapR[2], gapD, sumgapD;
// the FIFO a clock budget needs: the deepest run of samples later than the
// budget, in samples (deficit = max(0, deficit + gap / budget - 1))
static double budget_clk = 2267.57, defD = 0, defDmax = 0, defR[2] = {0, 0}, defRmax[2] = {0, 0};
static std::vector<uint64_t> histD(64, 0), histR(64, 0);
static uint64_t d_fills = 0, d_dmops = 0, r_fills = 0, r_dmops = 0;

// FREE: 68000 RAM writes land in the arrays at once; their snoops follow, one
// every 3 clocks a side (the caches may serve a stale line meanwhile -- the
// budget run does not compare outputs)
static std::deque<std::pair<int, uint32_t>> snqR, snqD;
static int snR = 0, snD = 0;
static bool snq_on = false;
static void tick() {
	t->clk = 0; t->eval();
	if (snq_on) {
		t->r_snp = 0; t->d_snp = 0;
		if (!snqR.empty() && ++snR >= 3) {
			snR = 0; auto x = snqR.front(); snqR.pop_front();
			t->r_snp = 1 << x.first; setel(t->r_snp_addr, 18, x.first, x.second);
		}
		if (!snqD.empty() && ++snD >= 3) {
			snD = 0; auto x = snqD.front(); snqD.pop_front();
			t->d_snp = 1; t->d_snp_ch = x.first; t->d_snp_addr = x.second;
		}
	}
	// the memory ports, served on the falling half
	t->r_sm_ack = 0; t->r_dm_ack = 0; t->d_sm_ack = 0; t->d_dm_ack = 0;
	for (int c = 0; c < 2; c++) {
		bool rq = bit(t->r_sm_req, c);
		if (rq && !rsm[c].was && !rsm[c].busy) { rsm[c].busy = true; rsm[c].wait = lat(); r_fills++; }
		if (rsm[c].busy && --rsm[c].wait <= 0) {
			t->r_sm_ack |= 1 << c; set64(t->r_sm_data, c, line_of(ramR[c], uint32_t(el(t->r_sm_line, 16, c)) << 2));
			rsm[c].busy = false;
		}
		rsm[c].was = rq;
		bool dq = bit(t->r_dm_req, c);
		if (dq && !rdm[c].was && !rdm[c].busy) { rdm[c].busy = true; rdm[c].wait = lat(); r_dmops++; }
		if (rdm[c].busy && --rdm[c].wait <= 0) {
			t->r_dm_ack |= 1 << c; rdm[c].busy = false;
			uint32_t w = uint32_t(el(t->r_dm_addr, 18, c)) & 0x3ffff;
			if (bit(t->r_dm_we, c)) ramR[c][w] = uint16_t(el(t->r_dm_wdata, 16, c));
			else set64(t->r_dm_data, c, line_of(ramR[c], w));
		}
		rdm[c].was = dq;
	}
	{
		bool rq = t->d_sm_req;
		if (rq && !dsm.was && !dsm.busy) { dsm.busy = true; dsm.wait = lat(); d_fills++; }
		if (dsm.busy && --dsm.wait <= 0) {
			t->d_sm_ack = 1; dsm.busy = false;
			uint64_t v = line_of(ramD[t->d_sm_ch], uint32_t(t->d_sm_line) << 2);
			t->d_sm_data = v;
		}
		dsm.was = rq;
		bool dq = t->d_dm_req;
		if (dq && !ddm.was && !ddm.busy) { ddm.busy = true; ddm.wait = lat(); d_dmops++; }
		if (ddm.busy && --ddm.wait <= 0) {
			t->d_dm_ack = 1; ddm.busy = false;
			uint32_t w = t->d_dm_addr & 0x3ffff;
			if (t->d_dm_we) ramD[t->d_dm_ch][w] = t->d_dm_wdata;
			else t->d_dm_data = line_of(ramD[t->d_dm_ch], w);
		}
		ddm.was = dq;
	}
	t->r_mout_ack = t->r_mout_valid; t->d_mout_ack = t->d_mout_valid;
	if (t->r_mout_valid) moR.push_back(t->r_mout_data);
	if (t->d_mout_valid) moD.push_back(t->d_mout_data);
	t->eval();
	t->clk = 1; t->eval();
	cyc++;
#ifdef PROF
	prof();
#endif
	// what came out on this edge
	for (int c = 0; c < 2; c++) {
		if (bit(t->r_ov, c)) {
			outR[c].push_back({int32_t(el(t->r_raw_l, 32, c)), int32_t(el(t->r_raw_r, 32, c)), int16_t(el(t->r_l, 16, c)), int16_t(el(t->r_r, 16, c))});
			nR[c]++;
			uint64_t g = cyc - lastR[c]; lastR[c] = cyc; gapR[c] = std::max(gapR[c], g); if (c == 0) histR[std::min<uint64_t>(63, g / 128)]++;
			defR[c] = std::max(0.0, defR[c] + double(g) / budget_clk - 1.0); defRmax[c] = std::max(defRmax[c], defR[c]);
		}
		if (bit(t->d_ov, c)) {
			outD[c].push_back({int32_t(el(t->d_raw_l, 32, c)), int32_t(el(t->d_raw_r, 32, c)), int16_t(el(t->d_l, 16, c)), int16_t(el(t->d_r, 16, c))});
			nD[c]++;
			if (c == 0) { uint64_t g = cyc - lastD; lastD = cyc; gapD = std::max(gapD, g); sumgapD += g; histD[std::min<uint64_t>(63, g / 128)]++;
				defD = std::max(0.0, defD + double(g) / budget_clk - 1.0); defDmax = std::max(defDmax, defD); }
		}
		if (bit(t->r_sv, c)) slR[c].push_back({int(el(t->r_slot, 5, c)), int32_t(el(t->r_smp, 32, c))});
	}
	if (t->d_sv) slD[t->d_slot >> 5].push_back({int(t->d_slot & 31), int32_t(t->d_smp)});
}

template <class T> struct Stream {
	FILE *f = nullptr; T cur{}; bool ok = false;
	void open(const std::string &p) { f = fopen(p.c_str(), "rb"); ok = f && fread(&cur, sizeof(T), 1, f) == 1; }
	void next() { ok = fread(&cur, sizeof(T), 1, f) == 1; }
};

static std::deque<uint16_t> rdR[2], rdD[2];
static void ref_access(int c, const Rec &r) {
	setel(t->r_addr, 11, c, (r.off >> 1) & 0x7ff);
	setel(t->r_din, 16, c, r.data);
	setel(t->r_be, 2, c, ((r.mask & 0xff00) ? 2 : 0) | ((r.mask & 0x00ff) ? 1 : 0));
	setel(t->r_we, 1, c, r.pad == 1 ? 0 : 1);
	setel(t->r_req, 1, c, 1); tick(); setel(t->r_req, 1, c, 0);
	int g = 0; while (!bit(t->r_ack, c)) { tick(); if (++g > 2000000) { printf("FAIL: ref %d not acked\n", c); exit(1); } }
	if (r.pad == 1) rdR[c].push_back(uint16_t(el(t->r_dout, 16, c)));
}
static void dual_access(int c, const Rec &r) {
	t->d_ch = c; t->d_addr = (r.off >> 1) & 0x7ff; t->d_din = r.data;
	t->d_be = ((r.mask & 0xff00) ? 2 : 0) | ((r.mask & 0x00ff) ? 1 : 0);
	t->d_we = r.pad == 1 ? 0 : 1;
	t->d_req = 1; tick(); t->d_req = 0;
	int g = 0; while (!t->d_ack) { tick(); if (++g > 2000000) { printf("FAIL: dual not acked\n"); exit(1); } }
	if (r.pad == 1) rdD[c].push_back(t->d_dout);
}
static void ram_write(uint16_t ram[2][0x40000], int c, const RamW &w) {
	uint32_t a = (w.a & 0x7fffe) >> 1; uint16_t dv = w.dm & 0xffff, m = w.dm >> 16;
	ram[c][a] = (ram[c][a] & ~m) | (dv & m);
}

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	const char *cap = getenv("SCSP_CAP");
	if (!cap) { fprintf(stderr, "SCSP_CAP=<dir with c0/ and c1/>\n"); return 2; }
	std::string d = cap;
	uint64_t N = getenv("SCSP_N") ? strtoull(getenv("SCSP_N"), nullptr, 0) : 44100;
	if (getenv("LAT_LO")) lat_lo = atoi(getenv("LAT_LO"));
	if (getenv("LAT_HI")) lat_hi = atoi(getenv("LAT_HI"));
	if (getenv("SCSP_SEED")) rng_s = strtoul(getenv("SCSP_SEED"), nullptr, 0);
	// FREE=dual (or 1) drives only the dual, FREE=ref only the two references
	const char *fe = getenv("FREE");
	const bool FREE = fe && strcmp(fe, "0");
	const bool FREE_REF = FREE && !strcmp(fe, "ref");
	const double MHZ = getenv("MHZ") ? atof(getenv("MHZ")) : 100.0;
	budget_clk = MHZ * 1e6 / 44100.0;

	Stream<Rec> regs[2]; Stream<RamW> rw[2];
	for (int c = 0; c < 2; c++) {
		regs[c].open(d + "/c" + std::to_string(c) + "/regs.bin");
		rw[c].open(d + "/c" + std::to_string(c) + "/ramw.bin");
		if (!regs[c].f) { fprintf(stderr, "no %s/c%d/regs.bin\n", cap, c); return 2; }
	}
	FILE *fm = fopen((d + "/c0/midi.txt").c_str(), "r");
	unsigned long long mi_n = 0; unsigned mi_b = 0;
	auto next_midi = [&]() -> bool {
		char dir[8];
		while (fm && fscanf(fm, "%llu %7s %x", &mi_n, dir, &mi_b) == 3) if (!strcmp(dir, "in")) return true;
		return false;
	};
	auto rewind_midi = [&]() { fclose(fm); fm = fopen((d + "/c0/midi.txt").c_str(), "r"); };
	bool mi_ok = next_midi();

	t = new Vscsp_dual_harness;
	t->d_tick44 = 0; t->rst_n = 0; t->step_mode = FREE ? 0 : 1; t->out_ready = 1; t->tick44 = 0; t->cpu_tick = 0; t->tph = 0;
	t->r_req = 0; t->d_req = 0; t->r_go = 0; t->d_go = 0; t->r_snp = 0; t->d_snp = 0;
	t->r_midi_valid = 0; t->d_midi_valid = 0;
	for (int i = 0; i < 4; i++) tick();
	t->rst_n = 1;
	for (int i = 0; i < 40; i++) tick();

	uint64_t ipl_bad = 0, ipl_chk = 0;
	if (!FREE) {
		auto at = [&](int c, uint64_t n) {
			int g = 0;
			while (!(t->d_at_go && t->d_at_ch == c && t->d_quiet)) { tick(); if (++g > 5000000) { printf("FAIL: dual never at chip %d's boundary (n %llu)\n", c, (unsigned long long)n); exit(1); } }
		};
		auto dual_go = [&](int c) {
			t->d_go = 1; int g = 0; while (t->d_at_go && t->d_at_ch == c) { tick(); if (++g > 100000) { printf("FAIL: go\n"); exit(1); } } t->d_go = 0;
		};
		auto dual_writes = [&](int c, uint64_t n) {
			while (regs[c].ok && regs[c].cur.n <= n) { dual_access(c, regs[c].cur); regs[c].next();
				int h = 0; while (!t->d_quiet) { tick(); if (++h > 2000000) { printf("FAIL: dual quiet\n"); exit(1); } } }
			while (rw[c].ok && rw[c].cur.n <= n) {
				ram_write(ramD, c, rw[c].cur);
				t->d_snp = 1; t->d_snp_ch = c; t->d_snp_addr = (rw[c].cur.a & 0x7fffe) >> 1; tick(); t->d_snp = 0; tick(); tick();
				rw[c].next();
			}
			int g = 0; while (!t->d_quiet) { tick(); if (++g > 2000000) { printf("FAIL: dual quiet\n"); exit(1); } }
		};
		// TWO PASSES over the same streams: the references alone (the dual held at
		// its first boundary), then the dual alone (the references idle). Run
		// together, the dual waited at each boundary for the slower references,
		// and its DSP finished in that wait: a turn that did not wait for its own
		// DSP passed (a planted fault, R-entry). Alone, it goes on as soon as
		// the bench has applied a boundary's writes.
		std::vector<int> ipl_ref;
		auto rewind = [&]() {
			for (int c = 0; c < 2; c++) {
				regs[c].f && fclose(regs[c].f); rw[c].f && fclose(rw[c].f);
				regs[c].open(d + "/c" + std::to_string(c) + "/regs.bin");
				rw[c].open(d + "/c" + std::to_string(c) + "/ramw.bin");
			}
			if (fm) rewind_midi();
		};
		(void)rewind;
		// ---------------- pass 1: the references
		for (uint64_t n = 0; n < N; n++) {
			std::vector<unsigned> midi;
			while (mi_ok && mi_n <= n) { midi.push_back(mi_b); mi_ok = next_midi(); }
			for (int c = 0; c < 2; c++) {
				if (c == 0) for (unsigned b : midi) {
					t->midi_data = b; t->r_midi_valid = 1;
					int g = 0; do { tick(); if (++g > 1000) { printf("FAIL: ref MIDI\n"); return 1; } } while (!t->r_midi_ack);
					t->r_midi_valid = 0; tick();
				}
				while (regs[c].ok && regs[c].cur.n <= n) { ref_access(c, regs[c].cur); regs[c].next();
					int g = 0; while (!bit(t->r_quiet, c)) { tick(); if (++g > 2000000) { printf("FAIL: ref quiet\n"); return 1; } } }
				while (rw[c].ok && rw[c].cur.n <= n) {
					ram_write(ramR, c, rw[c].cur);
					setel(t->r_snp, 1, c, 1); setel(t->r_snp_addr, 18, c, (rw[c].cur.a & 0x7fffe) >> 1); tick(); setel(t->r_snp, 1, c, 0); tick(); tick();
					rw[c].next();
				}
				int g = 0; while (!bit(t->r_quiet, c)) { tick(); if (++g > 2000000) { printf("FAIL: ref quiet\n"); return 1; } }
			}
			ipl_ref.push_back(int(el(t->r_ipl, 3, 0)));
			uint64_t b0 = nR[0], b1 = nR[1];
			t->r_go = 3;
			int g = 0;
			while (nR[0] == b0 || nR[1] == b1) {
				tick();
				for (int c = 0; c < 2; c++) if (!bit(t->r_quiet, c) || nR[c] != (c ? b1 : b0)) t->r_go &= ~(1 << c);
				if (++g > 5000000) { printf("FAIL: ref no sample\n"); return 1; }
			}
			t->tick44 = 1; tick(); t->tick44 = 0;
			if ((n % 44100) == 0 && n) { printf("  .. reference %llu samples\n", (unsigned long long)n); fflush(stdout); }
		}
		// ---------------- pass 2: the dual
		rewind();
		mi_ok = next_midi();
		for (uint64_t n = 0; n <= N; n++) {
			const bool last = n == N;
			std::vector<unsigned> midi;
			while (!last && mi_ok && mi_n <= n) { midi.push_back(mi_b); mi_ok = next_midi(); }
			for (int c = 0; c < 2; c++) {
				at(c, n);
				if (!last) {
					if (c == 0) for (unsigned b : midi) {
						t->midi_data = b; t->d_midi_valid = 1;
						int h = 0; do { tick(); if (++h > 1000) { printf("FAIL: dual MIDI\n"); return 1; } } while (!t->d_midi_ack);
						t->d_midi_valid = 0; tick();
						at(c, n);
					}
					dual_writes(c, n);
					if (c == 0) { ipl_chk++; if (int(t->d_ipl & 7) != ipl_ref[n]) { if (ipl_bad < 10) printf("  n %llu: ipl ref %d dual %d\n", (unsigned long long)n, ipl_ref[n], int(t->d_ipl & 7)); ipl_bad++; } }
				}
				dual_go(c);
			}
			if (last) break;
			t->d_tick44 = 1; tick(); t->d_tick44 = 0;
			if ((n % 44100) == 0 && n) { printf("  .. dual %llu samples\n", (unsigned long long)n); fflush(stdout); }
		}
		// the dual's last outputs
		for (int i = 0; i < 20000 && (nD[0] < N || nD[1] < N); i++) tick();
		// ---------------- compare
		uint64_t bad[2] = {0, 0}, nz[2] = {0, 0}, sbad[2] = {0, 0}, rbad = 0, nrd = 0;
		for (int c = 0; c < 2; c++) {
			if (outR[c].size() != N || outD[c].size() < N) { printf("FAIL: chip %d: %zu reference samples, %zu dual, want %llu\n", c, outR[c].size(), outD[c].size(), (unsigned long long)N); return 1; }
			for (uint64_t n = 0; n < N; n++) {
				const Out &a = outR[c][n], &b = outD[c][n];
				if (a.rl || a.rr) nz[c]++;
				if (a.rl != b.rl || a.rr != b.rr || a.l != b.l || a.r != b.r) {
					if (bad[c] < 10) printf("  chip %d sample %llu: ref %d %d (%d %d) dual %d %d (%d %d)\n", c, (unsigned long long)n, a.rl, a.rr, a.l, a.r, b.rl, b.rr, b.l, b.r);
					bad[c]++;
				}
			}
			size_t k = std::min(slR[c].size(), slD[c].size());
			for (size_t i = 0; i < k; i++) if (slR[c][i] != slD[c][i]) { if (sbad[c] < 5) printf("  chip %d slot sample %zu: ref %d:%d dual %d:%d\n", c, i, slR[c][i].first, slR[c][i].second, slD[c][i].first, slD[c][i].second); sbad[c]++; }
			// (the dual's flush turn makes chip 0's slots of sample N too: only fewer is wrong)
			if (slD[c].size() < slR[c].size()) { printf("  chip %d: %zu reference slot samples, %zu dual\n", c, slR[c].size(), slD[c].size()); sbad[c] += 1; }
			nrd += rdR[c].size();
			if (rdR[c] != rdD[c]) { for (size_t i = 0; i < std::min(rdR[c].size(), rdD[c].size()); i++) if (rdR[c][i] != rdD[c][i]) rbad++; if (rdR[c].size() != rdD[c].size()) rbad++; }
		}
		uint64_t mobad = (moR == moD) ? 0 : 1;
		printf("tb_scsp_dual %s, latency %d..%d: %llu samples a chip\n", cap, lat_lo, lat_hi, (unsigned long long)N);
		for (int c = 0; c < 2; c++)
			printf("  chip %d: %llu non-zero, %llu differ; slot samples %zu, %llu differ\n", c, (unsigned long long)nz[c], (unsigned long long)bad[c], slR[c].size(), (unsigned long long)sbad[c]);
		printf("  reads %llu, %llu differ; chip 0 IPL %llu boundaries, %llu differ; MIDI out %zu / %zu bytes, %s\n",
		       (unsigned long long)nrd, (unsigned long long)rbad, (unsigned long long)ipl_chk, (unsigned long long)ipl_bad, moR.size(), moD.size(), mobad ? "DIFFER" : "equal");
		printf("  memory: reference %llu slot fills, %llu DSP accesses; dual %llu / %llu\n",
		       (unsigned long long)r_fills, (unsigned long long)r_dmops, (unsigned long long)d_fills, (unsigned long long)d_dmops);
		bool fail = bad[0] || bad[1] || sbad[0] || sbad[1] || rbad || ipl_bad || mobad;
		printf("%s\n", fail ? "FAIL" : "PASS");
		return fail ? 1 : 0;
	}

	// ---------------- FREE: the clock budget
	{
		uint64_t c0 = cyc; lastD = cyc; lastR[0] = lastR[1] = cyc;
		snq_on = true;
		// separate cursors for the two sides
		Stream<Rec> rs2[2]; Stream<RamW> rw2[2];
		for (int c = 0; c < 2; c++) { rs2[c].open(d + "/c" + std::to_string(c) + "/regs.bin"); rw2[c].open(d + "/c" + std::to_string(c) + "/ramw.bin"); }
		while (FREE_REF ? (nR[0] < N || nR[1] < N) : (nD[0] < N || nD[1] < N)) {
			bool any = false;
			for (int c = 0; c < 2; c++) {
				// the reference chip c: its writes for sample nR[c]
				while (FREE_REF && nR[c] < N && rs2[c].ok && rs2[c].cur.n <= nR[c]) { ref_access(c, rs2[c].cur); rs2[c].next(); any = true; }
				while (FREE_REF && nR[c] < N && rw2[c].ok && rw2[c].cur.n <= nR[c]) {
					ram_write(ramR, c, rw2[c].cur);
					snqR.push_back({c, (rw2[c].cur.a & 0x7fffe) >> 1});
					rw2[c].next();
				}
				while (!FREE_REF && nD[c] < N && regs[c].ok && regs[c].cur.n <= nD[c]) { dual_access(c, regs[c].cur); regs[c].next(); any = true; }
				while (!FREE_REF && nD[c] < N && rw[c].ok && rw[c].cur.n <= nD[c]) {
					ram_write(ramD, c, rw[c].cur);
					snqD.push_back({c, (rw[c].cur.a & 0x7fffe) >> 1});
					rw[c].next();
				}
			}
			if (!any) tick();
			if (cyc - c0 > 40000ull * (N + 10)) { printf("FAIL: too slow\n"); return 1; }
		}
		double budget = budget_clk;
		printf("tb_scsp_dual FREE (%s) %s, latency %d..%d, %llu samples a chip; budget at %.4g MHz: %.0f clocks\n",
		       FREE_REF ? "two m2_scsp" : "scsp_dual", cap, lat_lo, lat_hi, (unsigned long long)N, MHZ, budget);
		uint64_t over = 0;
		if (FREE_REF) {
			for (int c = 0; c < 2; c++)
				printf("  m2_scsp chip %d: clocks a sample, mean %.0f, max %llu; FIFO the budget needs %.1f samples\n",
				       c, double(lastR[c] - c0) / double(nR[c]), (unsigned long long)gapR[c], defRmax[c]);
			for (int i = 0; i < 64; i++) if (i * 128 >= budget) over += histR[i];
			printf("  chip 0 samples over budget: %llu of %llu\n", (unsigned long long)over, (unsigned long long)nR[0]);
		} else {
			printf("  scsp_dual: clocks a sample pair, mean %.0f, max %llu; FIFO the budget needs %.1f samples\n",
			       double(sumgapD) / double(nD[0]), (unsigned long long)gapD, defDmax);
			printf("  pair intervals (128-clock bins): ");
			for (int i = 0; i < 64; i++) if (histD[i]) printf("%d:%llu ", i * 128, (unsigned long long)histD[i]);
			for (int i = 0; i < 64; i++) if (i * 128 >= budget) over += histD[i];
			printf("\n  pairs over budget: %llu of %llu\n", (unsigned long long)over, (unsigned long long)nD[0]);
		}
#ifdef PROF
		if (FREE_REF) prof_print("m2_scsp chip 0", pR, double(nR[0]));
		else prof_print("scsp_dual, a pair", pD, double(nD[0]));
#endif
	}
	return 0;
}
