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
// r3d_vpu kernels against host float in MAME's order, random operands. The
// bench holds the side memories (T&L inputs and vertex store, projection queue
// and output) and reads them a cycle after the address, as r3d_geo does.

#include "Vr3d_vpu.h"
#include "verilated.h"
#include "r3d_ucode.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>

using namespace r3d_uc;
static float bf(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }
static uint32_t fb(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
static Vr3d_vpu *t;
static uint64_t cyc = 0;
static uint32_t side_in[4][128], side_out[4][128];
static void tick() {
  t->clk = 0; t->eval();
  unsigned const a = t->sa_addr;
  unsigned const swa = t->sw_addr, swe = t->sw_we;
  uint32_t swd[4]; for (int l = 0; l < 4; l++) swd[l] = t->sw_data[l];
  t->clk = 1; t->eval();
  for (int l = 0; l < 4; l++) { if (swe & (1u << l)) side_out[l][swa] = swd[l]; t->sa_data[l] = side_in[l][a]; }
  t->eval();
  cyc++;
}
static void wr(unsigned l, unsigned a, float v) {
  for (int k = 0; k < 4; k++) t->ext_wdata[k] = 0;
  t->ext_we = 1u << l; t->ext_waddr = a; t->ext_wdata[l] = fb(v); tick(); t->ext_we = 0;
}
static void wbc(unsigned bank, unsigned a, float v) {
  t->ext_bwe = 1; t->ext_bbank = bank; t->ext_baddr = a; t->ext_bdata = fb(v); tick(); t->ext_bwe = 0;
}
static float rd(unsigned l, unsigned a) { t->ext_raddr = a; tick(); tick(); return bf(t->ext_rdata[l]); }
static uint64_t run(unsigned entry) {
  uint64_t c0 = cyc;
  t->start = 1; t->entry = entry; tick(); t->start = 0;
  while (t->busy) tick();
  tick();
  return cyc - c0;
}
static uint64_t nbad = 0, nchk = 0;
static void chk(char const *what, float got, float want) {
  nchk++;
  if (fb(got) == fb(want) || (std::isnan(got) && std::isnan(want))) return;
  if (nbad++ < 10) std::printf("%s: rtl %08X want %08X\n", what, fb(got), fb(want));
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  t = new Vr3d_vpu;
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> U(-2.0f, 2.0f);
  auto r = [&]() { float v = U(rng); if ((rng() & 31) == 0) v = 0.0f; if ((rng() & 63) == 0) v = -0.0f; return v * std::ldexp(1.0f, int(rng() % 9) - 4); };
  t->rst = 1; tick(); tick(); t->rst = 0;
  {
    // the micro-code's constants (bank 1, R_BC_K..), as the controller loads them
    float const kc[8] = {0.0f, 1.0f, 0.5f, 255.0f, 99999.0f, -1.0f, 2.0f, 0.000001f};
    for (int k = 0; k < 8; k++) wbc(1, R_BC_K + k, kc[k]);
  }
  int iters = argc > 1 ? std::atoi(argv[1]) : 200;
  uint64_t lens[8] = {};
  for (int it = 0; it < iters; it++) {
    // ---- MM: S' = M x S, M in broadcast, S column j in lane j of stack slot sp_a, S' into slot sp_d
    float M[4][4], S[4][4], C[4][4], P[4][4];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { M[i][j] = r(); S[i][j] = r(); C[i][j] = r(); P[i][j] = r(); }
    for (int i = 0; i < 4; i++) for (int k = 0; k < 4; k++) wbc(k, R_BC_MB + i, M[i][k]);
    unsigned const sa = unsigned(it) & 31, sd = unsigned(it + 7) & 31;
    for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) wr(j, 128 + sa * 4 + k, S[k][j]);
    t->sp_a = sa; t->sp_d = sd;
    lens[0] = run(K_MM);
    float Sn[4][4];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
      volatile float s = (M[i][0] * S[0][j]) + (M[i][1] * S[1][j]);
      s = s + (M[i][2] * S[2][j]); s = s + (M[i][3] * S[3][j]);
      Sn[i][j] = s;
    }
    // the result slot through COPYB (broadcast) is checked by MODEL below; read it back too
    t->sp_a = sd; run(K_COPYB);
    // ---- MODEL: tr = top x C, vp = tr x P (top is S' from the broadcast copy)
    for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) { wr(j, R_CO + k, C[k][j]); wr(j, R_PR + k, P[k][j]); }
    lens[1] = run(K_MODEL);
    float TRm[4][4], VP[4][4];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
      volatile float s = (Sn[i][0] * C[0][j]) + (Sn[i][1] * C[1][j]); s = s + (Sn[i][2] * C[2][j]); s = s + (Sn[i][3] * C[3][j]);
      TRm[i][j] = s; chk("MODEL tr", rd(j, R_TR + i), s);
    }
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) {
      volatile float s = (TRm[i][0] * P[0][j]) + (TRm[i][1] * P[1][j]); s = s + (TRm[i][2] * P[2][j]); s = s + (TRm[i][3] * P[3][j]);
      VP[i][j] = s;
    }
    // ---- TL: four vertices per lane, inputs and results in the side memories
    float L[3] = {r(), r(), r()}, Ip = r(), amb = std::fabs(r());
    Ip = std::clamp(std::fabs(Ip), 0.0f, 1.0f);
    wbc(0, R_BC_L0, L[0]); wbc(0, R_BC_L1, L[1]); wbc(0, R_BC_L2, L[2]); wbc(0, R_BC_IP, Ip); wbc(0, R_BC_AMB, amb);
    wbc(0, R_BC_L2N, -L[2]);
    float vin[4][4][6];
    for (int l = 0; l < 4; l++) for (int v = 0; v < 4; v++) {
      for (int q = 0; q < 6; q++) { vin[l][v][q] = r() * 64.0f; side_in[l][v * 8 + (q < 3 ? q : q + 1)] = fb(vin[l][v][q]); }
      side_in[l][v * 8 + 7] = 0x80000000u;   // F = -0: not fixed shading
    }
    lens[2] = run((it & 1) ? K_TL : K_TL3);
    for (int l = 0; l < 4; l++) for (int v = 0; v < ((it & 1) ? 4 : 3); v++) {
      float const *in = vin[l][v];
      for (int j = 0; j < 4; j++) {
        volatile float s = (in[0] * VP[0][j]) + (in[1] * VP[1][j]); s = s + (in[2] * VP[2][j]); s = s + (1.0f * VP[3][j]);
        chk("TL p", bf(side_out[l][v * 8 + j]), s);
      }
      // Supermodel's form (SM_LIGHT): n x T, sunFactor and its inverted copy
      // clamped to [0, 1], min(sf Ip + amb, 1) * 255
      float n[3];
      for (int i = 0; i < 3; i++) { volatile float s = (in[3] * TRm[0][i]) + (in[4] * TRm[1][i]); s = s + (in[5] * TRm[2][i]); n[i] = s; }
      volatile float dp = (n[0] * L[0]) + (-0.0f); dp = (n[1] * L[1]) + dp;
      volatile float d1 = (n[2] * L[2]) + dp, d2 = (n[2] * -L[2]) + (-dp);
      for (int c = 0; c < 2; c++) {
        float dd = std::clamp((float)(c ? d2 : d1), 0.0f, 1.0f);
        volatile float in2 = (dd * Ip) + amb;
        float iv = std::min((float)in2, 1.0f) * 255.0f;
        chk("TL int", bf(side_out[l][v * 8 + 4 + c]), iv);
      }
    }
    // ---- PROJ: four vertices per lane
    float vw = 495.75f, vx = r(), vh = 384.0f, vy = r();
    wbc(0, R_BC_VW, vw); wbc(0, R_BC_VX, vx); wbc(0, R_BC_VH, vh); wbc(0, R_BC_VY, vy);
    wbc(0, R_BC_ZS, 99999.0f); wbc(0, R_BC_ZO, 1.0f);   // MAME's depth map
    float pin[4][4][8];
    for (int l = 0; l < 4; l++) for (int v = 0; v < 4; v++) for (int q = 0; q < 8; q++) {
      pin[l][v][q] = (q == 3) ? std::fabs(r()) * 100.0f + 0.001f : r() * 100.0f;
      side_in[l][v * 8 + q] = fb(pin[l][v][q]);
    }
    unsigned const pk[3] = {K_PROJ1, K_PROJ3, K_PROJ}, prow[3] = {1, 3, 4};
    lens[3] = run(pk[it % 3]);
    for (int l = 0; l < 4; l++) for (unsigned v = 0; v < prow[it % 3]; v++) {
      float const *p = pin[l][v];
      volatile float oow = 1.0f / p[3];
      volatile float x = p[0] * oow, y = p[1] * oow, z = p[2] * oow, u = p[4] * oow, vv = p[5] * oow;
      volatile float X = (((x * 0.5f) + 0.5f) * vw) + vx, Y = (((y * 0.5f) + 0.5f) * vh) + vy, Z = (((z * 0.5f) + 0.5f) * 99999.0f) + 1.0f;
      uint32_t const *o = &side_out[l][v * 8];
      chk("PROJ x", bf(o[0]), X); chk("PROJ y", bf(o[1]), Y); chk("PROJ z", bf(o[2]), Z); chk("PROJ w", bf(o[3]), p[3]);
      chk("PROJ u", bf(o[4]), u); chk("PROJ v", bf(o[5]), vv); chk("PROJ shade", bf(o[6]), p[6]); chk("PROJ shade'", bf(o[7]), p[7]);
      chk("PROJ oow", rd(l, R_PJB + v * R_PJ_W), oow);
    }
    // ---- CLIP: one crossing, t = |N / (X - Y)| with X = pw - pa, Y = cw - ca, N = E - F
    float pw = r(), pa = r(), cw = r(), ca = r(), E = r(), F = r();
    for (int k = 0; k < 4; k++) t->ext_wdata[k] = 0;
    t->ext_we = 0xF; t->ext_waddr = R_CS; t->ext_wdata[0] = fb(pw); t->ext_wdata[1] = fb(cw); t->ext_wdata[2] = fb(E); tick();
    t->ext_waddr = R_CS + 1; t->ext_wdata[0] = fb(pa); t->ext_wdata[1] = fb(ca); t->ext_wdata[2] = fb(F); tick(); t->ext_we = 0;
    float la[4][2], lb[4][2];
    for (int l = 0; l < 4; l++) for (int s = 0; s < 2; s++) { la[l][s] = r(); lb[l][s] = r(); wr(l, R_LB + s * 3, la[l][s]); wr(l, R_LB + s * 3 + 1, lb[l][s]); }
    lens[4] = run(K_CLIP);
    volatile float X = pw - pa, Y = cw - ca, Nn = E - F, Z = X - Y, tt = std::fabs(Nn / Z);
    chk("CLIP wdiv", rd(0, R_CZ), Z);
    for (int l = 0; l < 4; l++) for (int s = 0; s < 2; s++) {
      volatile float d = lb[l][s] - la[l][s]; volatile float rr = la[l][s] + (d * tt);
      chk("CLIP lerp", rd(l, R_LB + s * 3 + 2), rr);
    }
    float ga[4], gb[4], gc[4];
    for (int l = 0; l < 4; l++) { ga[l] = r(); gb[l] = r(); gc[l] = r(); wr(l, R_GA, ga[l]); wr(l, R_GB, gb[l]); wr(l, R_GC, gc[l]); }
    run(K_ADD); for (int l = 0; l < 4; l++) chk("ADD", rd(l, R_GO), ga[l] + gc[l]);
    run(K_SUB); for (int l = 0; l < 4; l++) chk("SUB", rd(l, R_GO), ga[l] - gc[l]);
    run(K_MUL); for (int l = 0; l < 4; l++) chk("MUL", rd(l, R_GO), ga[l] * gb[l]);
    lens[5] = run(K_DIV); for (int l = 0; l < 4; l++) chk("DIV", rd(l, R_GO), gb[l] / ga[l]);
  }
  std::printf("r3d_vpu: %llu results checked, %llu wrong; cycles MM %llu MODEL %llu TL %llu PROJ %llu CLIP %llu DIV %llu\n",
              (unsigned long long)nchk, (unsigned long long)nbad, (unsigned long long)lens[0], (unsigned long long)lens[1],
              (unsigned long long)lens[2], (unsigned long long)lens[3], (unsigned long long)lens[4], (unsigned long long)lens[5]);
  delete t;
  return nbad ? 1 : 0;
}
