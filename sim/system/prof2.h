// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// tb_m3_system --prof2 FILE: where the CPU's cycles go, frame by frame (study R107).
//
// Every cycle is given one cause, by ppc_core's issue stage (ID to EX), in this order:
//   issue      EX takes a uop (h_adv, EX not held, no redirect)
//   EX held    data (MEM/WB wait on the D-cache's answer, or the D-cache not ready), mul/div,
//              FPU (fp_wait), other holds; data-held cycles are split by what the CPU's memory
//              port is doing: the D-cache's transaction (fill, cast-out, single read or write,
//              by region), the I-cache's fill ahead of it, or nothing (inside the D-cache)
//   load-use   ID's uop waits on a load in EX
//   FP dep     ID's uop waits on an FP result (h_adv low without load-use)
//   redirect   the redirect cycle itself
//   starved    ID empty: during an I-cache fill (I-miss), during a fetch-translation slow
//              cycle, or otherwise (the refill after a redirect, the fetch queue's own gaps)
// Uops issued but not retired are the wrong path. Idle is the share of cycles whose last
// retirement lies in one of the game's wait loops (study R104's windows).
//
// Memory, at the CPU's port (ppc_cpu, master 0 of m3_bus): every transaction from its request
// to its last beat; the line fill's first and last beat, a single read's beat, a cast-out's last
// beat taken, each as a histogram, split by whether another SDRAM port was pending or in flight
// when the request was made. At m2_sdram: each port's cycles in flight and waiting, its CAS
// cycles and its row-miss cycles (precharge, activate, tRCD), and refresh.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

struct Prof2 {
  std::FILE *f = nullptr;
  bool dt2 = false;                // Daytona 2's wait loops, else VF3's
  // per frame
  enum { C_ISSUE, C_DW, C_MD, C_FP, C_HOLD_O, C_LU, C_FPDEP, C_REDIR, C_IMISS, C_FSLOW, C_FBUB, C_OTHER, NC };
  uint64_t c[NC] = {}, cyc = 0, ins = 0, uops = 0, idle = 0, iss = 0, bins = 0;
  uint64_t sp_hit[6] = {}, sp_closed[6] = {}, sp_conf[6] = {}; uint8_t st_prev = 0, gr_prev = 0;
  // data wait by the port's activity: [who 0 D 1 I 2 none][cmd][region class]
  enum { R_RAM, R_CROM, R_IO, R_R3D, R_OTH, NR };
  uint64_t dw[3][4][NR] = {};
  uint64_t dw_dma = 0;             // the bus held by the 53C810 while the D-cache asks
  // SDRAM
  uint64_t sp_inf[6] = {}, sp_wait[6] = {}, sp_cas[6] = {}, sp_miss[6] = {}, s_ref = 0, s_idle = 0;
  uint64_t sport_st[8] = {};
  // transactions (whole run)
  struct H { uint64_t n = 0, sum = 0; uint32_t b[257] = {}; void add(uint64_t v) { n++; sum += v; b[v > 256 ? 256 : v]++; }
             double mean() const { return n ? double(sum) / double(n) : 0.0; }
             uint32_t pct(double p) const { uint64_t t = 0, k = uint64_t(p * double(n)); for (int i = 0; i <= 256; i++) { t += b[i]; if (t > k) return uint32_t(i); } return 256; } };
  // [kind][contended]: 0 D fill first beat, 1 D fill last beat, 2 I fill first, 3 I fill last,
  // 4 single read (RAM/CROM), 5 single read (I/O, Real3D), 6 cast-out (last beat taken), 7 single write
  H h[8][2];
  uint64_t fr_n[8] = {};           // per frame counts of the same kinds
  uint64_t tot_n[8] = {};
  // the CPU port's transaction in progress
  bool tx = false; uint64_t t0 = 0; int tcmd = 0, twho = 0, treg = 0, tbeats = 0; bool tcont = false; bool tfirst = false;
  uint32_t taddr = 0; bool prev_req = false;
  // per-stage clocks of a line fill (study R112), from the request at the CPU's port: 1 the bus's decode
  // (B_DEC), 2 s_req, 3 the port's p_req, 4 m2_sdram's pend (the mailbox), 5 granted (inflight), 6 S_DISPATCH,
  // 7 the first S_RD (the READ goes out at the next edge), 8 the first p_lo / p_ack, 9 s_rvalid, 10 m_rvalid
  enum { NSTG = 11 };
  uint64_t ts[NSTG] = {}; bool tsv[NSTG] = {}; int trow = -1;
  H hs[2][NSTG];                   // [I side][stage]: the clocks from the stage before
  H hrow[3];                       // stage 6 to 7 by the dispatch's outcome: 0 row open, 1 bank closed, 2 another row
  H hall[2];                       // [I side]: request to m_rvalid for the fills counted here
  uint64_t st_part[2] = {};
  int tpst = 0; H hps[2][10];
  uint64_t w2_cyc = 0, w2_ins = 0;
  std::map<uint32_t, uint64_t> pch, dwi;   // dwi: data wait with the port idle, by the D-cache's state
  bool tgot = false; uint64_t bw1[2][12] = {};   // CPU cycles waiting for the bus, by the bus state while the 53C810 has it
  uint8_t bk_own[8] = {}; int tbk = 0; uint64_t conf_by[2][8] = {};   // a fill's row conflict: the port that opened the row       // s_req to p_req by the port's state at s_req (T_IDLE 0 ... T_PW 8)
  uint64_t fill_reg[2][5] = {};     // the fills counted, by region (RAM, CROM, I/O, Real3D, other)        // fills whose chain was not whole (answered by the port's read-ahead, or other)
  // sequential fills: an I fill of the line after the previous I fill's; a D fill of the line after
  // one of the last four D fills' (a stream); per frame
  uint32_t i_last = 0, d_last[4] = {}; int d_lp = 0; uint64_t i_seq = 0, d_seq = 0;
  // a next-line stream buffer at the port, modelled (study R107): after each line fill of line L
  // (I or D side, one buffer each) line L + 1 is read ahead; the next fill on that side hits it if it
  // is L + 1 and no write to L + 1 came in between. For each hit, the clocks from the last beat of
  // the fill that started the read-ahead to the hit's request (a read-ahead needs ~45 to be whole)
  struct SB { uint32_t ln = 0; bool v = false; uint64_t t_end = 0; uint64_t fills = 0, hits = 0, inv = 0; H gap; } sb[2];
  uint64_t t_last_beat = 0;
  // frames of game work (VF3: exit of a vblank poll to the next entry)
  uint32_t last_pc = 0; bool in_poll = false; uint64_t gf_start = 0, gf_busy = 0, gf_ins = 0; long gf_vb = 0;
  std::vector<uint64_t> gf_busy_all;
  // game frames by the Real3D command port: from one 88000000 write (a frame submitted) to the next,
  // the cycles, the busy cycles and the busy instructions
  uint32_t sub_last = 0; bool sub_init = false; uint64_t sf_cyc = 0, sf_busy = 0, sf_ins = 0, fr_sub = 0;
  struct SF { uint64_t cyc, busy, ins; };
  std::vector<SF> sf_all;

  static int region(uint32_t a) {
    if (a < 0x00800000) return R_RAM;
    if (a >= 0xFF000000) return R_CROM;
    if ((a >> 24) >= 0x84 && (a >> 24) <= 0x9C) return R_R3D;
    if ((a >> 28) == 0xF || (a >> 24) == 0xC0) return R_IO;
    return R_OTH;
  }
  bool in_wait(uint32_t pc) const {
    if (dt2) return (pc >= 0x4C6CC && pc <= 0x4C6D4) || (pc >= 0x4C73C && pc <= 0x4C744) || (pc >= 0x4C754 && pc <= 0x4C760);
    return (pc >= 0x1E928 && pc <= 0x1E930) || (pc >= 0x1E97C && pc <= 0x1E984) || (pc >= 0x1E9DC && pc <= 0x1E9E4) ||
           (pc >= 0x1E9FC && pc <= 0x1EA04) || (pc >= 0x2518 && pc <= 0x251C);
  }
  bool in_vpoll(uint32_t pc) const {
    if (dt2) return pc >= 0x4C6CC && pc <= 0x4C6D4;
    return (pc >= 0x1E928 && pc <= 0x1E930) || (pc >= 0x1E97C && pc <= 0x1E984);
  }

  template <class R> void tick(R *q, uint64_t now, bool rt_valid, uint32_t rt_pc, uint32_t frame) {
#define K(x) q->m3_system__DOT__u_cpu__DOT__u_core__DOT__##x
#define U(x) q->m3_system__DOT__u_cpu__DOT__##x
#define S(x) q->m3_system__DOT__u_sdram__DOT__##x
    cyc++;
    uint8_t const ev = q->m3_system__DOT__g_mon__DOT__u_mon__DOT__ev_q;   // a clock late, as --prof
    if (ev & 1) ins++;
    if (ev & 2) uops++;
    if (rt_valid) last_pc = rt_pc;
    if (rt_valid && !in_wait(rt_pc)) pch[rt_pc >> 6]++;   // busy retirements by 64-byte block (R112)
    // Daytona 2's wait at 52A4-52C4 (a RAM flag at 700100, with a time-base time-out), not one of R104's
    // windows: its cycles and retirements, counted apart (R112)
    if (dt2 && !in_wait(last_pc) && last_pc >= 0x52A4 && last_pc <= 0x52C4) { w2_cyc++; if (ev & 1) w2_ins++; }
    bool const w = in_wait(last_pc);
    if (w) idle++;
    else if (ev & 1) bins++;
    // game frames
    if (rt_valid) {
      bool const p = in_vpoll(rt_pc);
      if (p && !in_poll) { gf_busy_all.push_back(gf_busy); }
      if (!p && in_poll) { gf_busy = 0; gf_ins = 0; gf_start = now; gf_vb = frame; }
      in_poll = p;
    }
    if (!w) gf_busy++;
    sf_cyc++; if (!w) { sf_busy++; if (ev & 1) sf_ins++; }
    {
      uint32_t const ns = q->m3_system__DOT__n_submit;
      if (!sub_init) { sub_last = ns; sub_init = true; sf_cyc = sf_busy = sf_ins = 0; }
      else if (ns != sub_last) {
        sub_last = ns; fr_sub++;
        sf_all.push_back({sf_cyc, sf_busy, sf_ins});
        sf_cyc = sf_busy = sf_ins = 0;
      }
    }
    // the issue stage
    bool const idv = K(id_valid), exh = K(ex_hold), lu = K(load_use), rd = K(redirect), ha = K(h_adv);
    int cause;
    if (ha && !exh && !rd) { cause = C_ISSUE; iss++; }
    else if (exh) {
      if (K(mem_hold) || (K(ex_valid) && K(ex_ls) && !K(md_wait) && !K(fp_wait))) cause = C_DW;
      else if (K(md_wait)) cause = C_MD;
      else if (K(fp_wait)) cause = C_FP;
      else cause = C_HOLD_O;
    }
    else if (idv && lu) cause = C_LU;
    else if (idv && !ha) cause = C_FPDEP;
    else if (rd) cause = C_REDIR;
    else if (!idv) {
      bool const ifill = U(im_req) || (U(own) && !U(od));
      cause = ifill ? C_IMISS : K(f_slow) ? C_FSLOW : C_FBUB;
    } else cause = C_OTHER;
    if (!w) c[cause]++;
    // the CPU's memory port: transactions
    bool const mreq = q->m3_system__DOT__m_req & 1, mack = q->m3_system__DOT__m_ack & 1;
    bool const mrv = q->m3_system__DOT__m_rvalid & 1, mwn = q->m3_system__DOT__m_wnext & 1;
    uint8_t const pend = S(pend), inf = S(inflight);
    if (!tx && mreq && !prev_req) {
      tx = true; t0 = now; tcmd = q->m3_system__DOT__m_cmd & 3; taddr = uint32_t(q->m3_system__DOT__m_addr);
      twho = U(sel_d) ? 0 : 1; treg = region(taddr); tbeats = 0; tfirst = false;
      for (int k = 0; k < NSTG; k++) tsv[k] = false;
      tgot = false;
      ts[0] = now; tsv[0] = true; trow = -1;
      tcont = ((pend | inf) & 0x3E) != 0;
      if (tcmd == 0) {
        SB &b = sb[twho];
        b.fills++;
        if (b.v && b.ln == (taddr >> 5)) { b.hits++; b.gap.add(now - b.t_end); }
        b.v = false;
      } else if (tcmd == 1 || tcmd == 3) {
        for (auto &b : sb) if (b.v && b.ln == (taddr >> 5)) { b.v = false; b.inv++; }
      }
      if (tcmd == 0) {
        uint32_t const ln = taddr >> 5;
        if (twho) { if (ln == i_last + 1) i_seq++; i_last = ln; }
        else { bool sq = false; for (auto v : d_last) if (ln == v + 1) sq = true; if (sq) d_seq++; d_last[d_lp++ & 3] = ln; }
      }
    }
    prev_req = mreq;
    if (tx) {
      int kind_first = -1, kind_last = -1;
      if (tcmd == 0) { kind_first = twho ? 2 : 0; kind_last = twho ? 3 : 1; }
      // the CPU's request waiting for the bus: what the bus is doing for the 53C810 (B_SDR: its SDRAM access;
      // B_BEAT / B_R3D / B_R3DR / B_INT...: an internal target, mostly the Real3D's host port), every kind of
      // transaction; and with tbeats == 0 && no bus grant yet
      {
        uint8_t const b = q->m3_system__DOT__u_bus__DOT__bs; bool const wh = q->m3_system__DOT__u_bus__DOT__who;
        bool const got = (b == 1 || b == 2 || b >= 3) && !wh && b != 0;
        if (!tgot) { if (got) tgot = true; else if (b != 0 && wh) bw1[twho][b > 11 ? 11 : b]++; else bw1[twho][0]++; }
      }
      if (tcmd == 0) {
        bool sc[NSTG] = {};
        { uint8_t const b = q->m3_system__DOT__u_bus__DOT__bs; sc[1] = (b == 1 || b == 2) && !q->m3_system__DOT__u_bus__DOT__who; }   // B_DEC, or B_SDR (FAST)
        sc[2] = q->m3_system__DOT__s_req;
        // the fill's own request (the port in T_LR, or in T_IDLE taking it with FREQ): a posted write's
        // burst in flight on port 0 is not this fill's
        { uint8_t const pst = q->m3_system__DOT__u_sport__DOT__st; sc[3] = (q->m3_system__DOT__p_req & 1) && (pst == 2 || pst == 0); }
        sc[4] = S(pend) & 1;
        sc[5] = S(inflight) & 1;
        sc[6] = S(state) == 3 && S(grant) == 0;
        sc[7] = S(state) == 8 && S(grant) == 0;
        sc[8] = (q->m3_system__DOT__p_lo | q->m3_system__DOT__p_ack) & 1;
        sc[9] = q->m3_system__DOT__s_rvalid;
        sc[10] = mrv;
        if (tsv[6] && !tsv[7] && trow < 0) {
          uint8_t const s2 = S(state); trow = s2 == 8 ? 0 : (s2 == 6 || s2 == 7) ? 1 : 2;
          if (trow == 2) conf_by[twho][bk_own[tbk] > 6 ? 7 : bk_own[tbk]]++;
        }
        if (sc[6] && !tsv[6]) tbk = (S(xfer_addr) >> 23) & 7;
        for (int k = 1; k < NSTG; k++) if (!tsv[k] && tsv[k - 1] && sc[k]) {
          ts[k] = now; tsv[k] = true;
          if (k == 2) tpst = q->m3_system__DOT__u_sport__DOT__st;   // what the port is doing when the fill reaches it
        }
        if (mrv && !tfirst) {
          bool whole = true; for (int k = 0; k < NSTG; k++) whole = whole && tsv[k];
          if (whole) {
            for (int k = 1; k < NSTG; k++) hs[twho][k].add(ts[k] - ts[k - 1]);
            hps[twho][tpst > 8 ? 9 : tpst].add(ts[3] - ts[2]);
            if (trow >= 0) hrow[trow].add(ts[7] - ts[6]);
            hall[twho].add(now - ts[0]); fill_reg[twho][treg]++;
          } else st_part[twho]++;
        }
      }
      if (mrv) {
        tbeats++;
        if (!tfirst && kind_first >= 0) { h[kind_first][tcont].add(now - t0); tfirst = true; }
        if (tcmd == 2) { int k = (treg == R_RAM || treg == R_CROM) ? 4 : 5; h[k][tcont].add(now - t0); fr_n[k]++; tot_n[k]++; tx = false; }
        else if (tbeats == 8) {
          h[kind_last][tcont].add(now - t0); fr_n[kind_last]++; tot_n[kind_last]++; tx = false;
          if (tcmd == 0) { SB &b = sb[twho]; b.v = true; b.ln = (taddr >> 5) + 1; b.t_end = now; }
        }
      }
      if (mwn) { tbeats++; if (tbeats == 8) { h[6][tcont].add(now - t0); fr_n[6]++; tot_n[6]++; tx = false; } }
      if (tcmd == 3 && mack) { h[7][tcont].add(now - t0); fr_n[7]++; tot_n[7]++; tx = false; }
    }
    // data wait by the port's activity
    if (cause == C_DW && !w) {
      if (tx && twho == 0) dw[0][tcmd][treg]++;
      else if (tx && twho == 1) dw[1][tcmd][treg]++;
      else if (U(dm_req)) dw[0][U(dm_cmd) & 3][region(U(dm_addr))]++;
      else {
        dw[2][0][0]++;
#define DC(x) q->m3_system__DOT__u_cpu__DOT__u_dc__DOT__##x
        uint32_t const key = uint32_t(DC(x_v)) | (uint32_t(DC(x_ph)) << 1) | (uint32_t(DC(f_act)) << 3) | (uint32_t(DC(f_ph)) << 4) |
                             (uint32_t(DC(mq_v)) << 6) | (uint32_t(DC(dq_v)) << 7) | (uint32_t(DC(r_v)) << 8) | (uint32_t(DC(x_k) & 7) << 9) |
                             (uint32_t(DC(f_k) & 7) << 12) | (uint32_t(DC(sb_v)) << 15) | (uint32_t(DC(uq_v)) << 16);
#undef DC
        dwi[key]++;
      }
      if ((q->m3_system__DOT__m_req & 2) && !(tx)) dw_dma++;
    }
    // SDRAM ports
    uint8_t const st = S(state), gr = S(grant);
    if (st == 6) bk_own[(S(xfer_addr) >> 23) & 7] = gr;   // S_ACT: the port that opens the bank's row
    for (int k = 0; k < 6; k++) {
      if ((inf >> k) & 1) sp_inf[k]++;
      else if ((pend >> k) & 1) sp_wait[k]++;
    }
    if (gr < 6) {
      if (st == 8 || st == 9) sp_cas[gr]++;
      if (st >= 4 && st <= 7) sp_miss[gr]++;
    }
    if (st_prev == 3 && gr_prev < 6) {
      if (st == 8 || st == 9) sp_hit[gr_prev]++; else if (st == 6) sp_closed[gr_prev]++; else sp_conf[gr_prev]++;
    }
    st_prev = st; gr_prev = gr;
    if (st >= 11) s_ref++;
    if (st == 1) s_idle++;
    sport_st[q->m3_system__DOT__u_sport__DOT__st & 7]++;
#undef K
#undef U
#undef S
  }

  void header() {
    if (!f) return;
    std::fprintf(f, "# frame cycles ins uops idle busy_ins issue dwait muldiv fpu hold_o loaduse fpdep redir imiss fslow fbubble other"
                    " | dw D:fillRAM fillCROM castout rdRAM rdCROM rdIO rdR3D wrRAM wrIO wrR3D | dw I-fill | dw none | dw dma"
                    " | n Dfill Ifill rdmem rdio castout wr | sdram inf0-5 wait0-5 cas0-5 miss0-5 ref idle | sport idle rd lr wr lw zero | submits | seq Ifill Dfill | dispatch hit/closed/conflict p0-5\n");
  }
  void frame(uint32_t fr) {
    if (!f) { reset(); return; }
    std::fprintf(f, "%u %llu %llu %llu %llu %llu", fr, (unsigned long long)cyc, (unsigned long long)ins, (unsigned long long)uops, (unsigned long long)idle, (unsigned long long)bins);
    for (int k = 0; k < NC; k++) std::fprintf(f, " %llu", (unsigned long long)c[k]);
    auto d = [&](int a, int b, int r) { std::fprintf(f, " %llu", (unsigned long long)dw[a][b][r]); };
    std::fprintf(f, " |"); d(0, 0, R_RAM); d(0, 0, R_CROM); std::fprintf(f, " %llu", (unsigned long long)(dw[0][1][R_RAM] + dw[0][1][R_OTH] + dw[0][1][R_CROM]));
    d(0, 2, R_RAM); d(0, 2, R_CROM); d(0, 2, R_IO); d(0, 2, R_R3D); d(0, 3, R_RAM); d(0, 3, R_IO); d(0, 3, R_R3D);
    uint64_t ifl = 0; for (int b = 0; b < 4; b++) for (int r = 0; r < NR; r++) ifl += dw[1][b][r];
    std::fprintf(f, " | %llu | %llu | %llu |", (unsigned long long)ifl, (unsigned long long)dw[2][0][0], (unsigned long long)dw_dma);
    std::fprintf(f, " %llu %llu %llu %llu %llu %llu |", (unsigned long long)fr_n[1], (unsigned long long)fr_n[3], (unsigned long long)fr_n[4],
                 (unsigned long long)fr_n[5], (unsigned long long)fr_n[6], (unsigned long long)fr_n[7]);
    for (auto v : sp_inf) std::fprintf(f, " %llu", (unsigned long long)v);
    for (auto v : sp_wait) std::fprintf(f, " %llu", (unsigned long long)v);
    for (auto v : sp_cas) std::fprintf(f, " %llu", (unsigned long long)v);
    for (auto v : sp_miss) std::fprintf(f, " %llu", (unsigned long long)v);
    std::fprintf(f, " %llu %llu |", (unsigned long long)s_ref, (unsigned long long)s_idle);
    for (int k = 0; k < 6; k++) std::fprintf(f, " %llu", (unsigned long long)sport_st[k]);
    std::fprintf(f, " | %llu | %llu %llu |", (unsigned long long)fr_sub, (unsigned long long)i_seq, (unsigned long long)d_seq);
    for (int k = 0; k < 6; k++) std::fprintf(f, " %llu/%llu/%llu", (unsigned long long)sp_hit[k], (unsigned long long)sp_closed[k], (unsigned long long)sp_conf[k]);
    std::fprintf(f, "\n"); std::fflush(f);
    reset();
  }
  void reset() {
    std::memset(c, 0, sizeof c); cyc = ins = uops = idle = iss = bins = 0;
    std::memset(sp_hit, 0, sizeof sp_hit); std::memset(sp_closed, 0, sizeof sp_closed); std::memset(sp_conf, 0, sizeof sp_conf); fr_sub = 0; i_seq = d_seq = 0; std::memset(dw, 0, sizeof dw); dw_dma = 0;
    std::memset(sp_inf, 0, sizeof sp_inf); std::memset(sp_wait, 0, sizeof sp_wait); std::memset(sp_cas, 0, sizeof sp_cas);
    std::memset(sp_miss, 0, sizeof sp_miss); s_ref = s_idle = 0; std::memset(sport_st, 0, sizeof sport_st); std::memset(fr_n, 0, sizeof fr_n);
  }
  void report() {
    if (!f) return;
    char const *nm[8] = {"D fill, first beat", "D fill, last beat", "I fill, first beat", "I fill, last beat",
                         "single read RAM/CROM", "single read I/O, Real3D", "cast-out, last beat taken", "single write, ack"};
    for (int k = 0; k < 8; k++)
      for (int cn = 0; cn < 2; cn++)
        std::fprintf(f, "# lat %-26s %s: n %llu mean %.1f p10 %u p50 %u p90 %u p99 %u\n", nm[k], cn ? "contended" : "alone    ",
                     (unsigned long long)h[k][cn].n, h[k][cn].mean(), h[k][cn].pct(0.1), h[k][cn].pct(0.5), h[k][cn].pct(0.9), h[k][cn].pct(0.99));
    std::fprintf(f, "# submits (cycles/busy/busy instructions from the last; the first is partial):");
    for (auto &e : sf_all) std::fprintf(f, " %llu/%llu/%llu", (unsigned long long)e.cyc, (unsigned long long)e.busy, (unsigned long long)e.ins);
    std::fprintf(f, "\n");
    for (int k = 0; k < 2; k++)
      std::fprintf(f, "# stream %s: fills %llu, hits on the line after the last fill %llu (%.1f%%), lost to a write %llu; gap from that fill's last beat: mean %.0f p10 %u p50 %u p90 %u\n",
                   k ? "I" : "D", (unsigned long long)sb[k].fills, (unsigned long long)sb[k].hits, sb[k].fills ? 100.0 * double(sb[k].hits) / double(sb[k].fills) : 0.0,
                   (unsigned long long)sb[k].inv, sb[k].gap.mean(), sb[k].gap.pct(0.1), sb[k].gap.pct(0.5), sb[k].gap.pct(0.9));
    {
      char const *sn[NSTG] = {"", "request -> bus decode", "bus decode -> s_req", "s_req -> port p_req", "p_req -> pend (mailbox)",
                              "pend -> granted", "granted -> S_DISPATCH", "S_DISPATCH -> first S_RD", "S_RD -> first p_lo",
                              "p_lo -> s_rvalid", "s_rvalid -> m_rvalid"};
      for (int w = 0; w < 2; w++) {
        std::fprintf(f, "# stage %s fill: n %llu (not whole %llu; RAM %llu, CROM %llu), request to first m_rvalid mean %.2f p50 %u\n", w ? "I" : "D",
                     (unsigned long long)hall[w].n, (unsigned long long)st_part[w], (unsigned long long)fill_reg[w][0], (unsigned long long)fill_reg[w][1],
                     hall[w].mean(), hall[w].pct(0.5));
        for (int k = 1; k < NSTG; k++)
          std::fprintf(f, "# stage %s %-26s mean %6.2f p10 %u p50 %u p90 %u p99 %u\n", w ? "I" : "D", sn[k], hs[w][k].mean(),
                       hs[w][k].pct(0.1), hs[w][k].pct(0.5), hs[w][k].pct(0.9), hs[w][k].pct(0.99));
      }
      char const *pn[10] = {"T_IDLE", "T_RD", "T_LR", "T_WR", "T_LW", "T_ZERO", "T_HIT", "T_RA", "T_PW", "?"};
      for (int w = 0; w < 2; w++) for (int k = 0; k < 10; k++) if (hps[w][k].n)
        std::fprintf(f, "# stage %s s_req -> p_req, port in %-7s n %llu mean %.2f p50 %u p90 %u\n", w ? "I" : "D", pn[k],
                     (unsigned long long)hps[w][k].n, hps[w][k].mean(), hps[w][k].pct(0.5), hps[w][k].pct(0.9));
      for (int w = 0; w < 2; w++) {
        std::fprintf(f, "# stage %s fill row conflicts, the row opened by port:", w ? "I" : "D");
        for (int k = 0; k < 8; k++) std::fprintf(f, " %d:%llu", k, (unsigned long long)conf_by[w][k]);
        std::fprintf(f, " (6: the loader)\n");
      }
      char const *bn[12] = {"idle/turn", "B_DEC", "B_SDR", "B_INT", "B_INTW", "B_SCSI", "B_CFG", "B_JTAG", "B_BEAT", "B_R3D", "B_R3DR", "B_SEC"};
      for (int w = 0; w < 2; w++) {
        std::fprintf(f, "# stage %s side waiting for the bus (all transactions), cycles by the 53C810's bus state:", w ? "I" : "D");
        for (int k = 0; k < 12; k++) if (bw1[w][k]) std::fprintf(f, " %s %llu", bn[k], (unsigned long long)bw1[w][k]);
        std::fprintf(f, "\n");
      }
      char const *rn[3] = {"row open", "bank closed", "another row open"};
      for (int r = 0; r < 3; r++)
        std::fprintf(f, "# stage S_DISPATCH -> S_RD, %-16s n %llu mean %.2f\n", rn[r], (unsigned long long)hrow[r].n, hrow[r].mean());
    }
    {
      std::vector<std::pair<uint64_t, uint32_t>> v; for (auto &e : pch) v.push_back({e.second, e.first});
      std::sort(v.rbegin(), v.rend());
      std::vector<std::pair<uint64_t, uint32_t>> u; for (auto &e : dwi) u.push_back({e.second, e.first});
      std::sort(u.rbegin(), u.rend());
      std::fprintf(f, "# dw port idle by D-cache state (x_v x_ph f_act f_ph mq_v dq_v r_v x_k f_k sb_v uq_v):");
      for (size_t k = 0; k < u.size() && k < 16; k++) { uint32_t const y = u[k].second;
        std::fprintf(f, " [%u %u %u %u %u %u %u %u %u %u %u]:%llu", y & 1, (y >> 1) & 3, (y >> 3) & 1, (y >> 4) & 3, (y >> 6) & 1, (y >> 7) & 1,
                     (y >> 8) & 1, (y >> 9) & 7, (y >> 12) & 7, (y >> 15) & 1, (y >> 16) & 1, (unsigned long long)u[k].first); }
      std::fprintf(f, "\n");
      std::fprintf(f, "# wait2 (Daytona 2 52A4-52C4, in busy): cycles %llu instructions %llu\n", (unsigned long long)w2_cyc, (unsigned long long)w2_ins);
      std::fprintf(f, "# pc blocks (64 B, busy retirements):");
      for (size_t k = 0; k < v.size() && k < 60; k++) std::fprintf(f, " %X:%llu", v[k].second << 6, (unsigned long long)v[k].first);
      std::fprintf(f, "\n");
    }
    std::fprintf(f, "# game frames (busy cycles, vblank-poll exit to entry):");
    for (auto v : gf_busy_all) std::fprintf(f, " %llu", (unsigned long long)v);
    std::fprintf(f, "\n");
    std::fflush(f);
  }
};
