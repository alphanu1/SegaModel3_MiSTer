// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// tb_m3_inputs (make -f sim/system/system.mk test_inputs): m3_inputs (with
// m3_cab_inputs) against a reference written from the ports as MAME 0.289
// model3.cpp and Supermodel 036af1c ReadInputs give them, per profile:
// random joystick words, sticks, triggers and paddle, each held long enough
// for the registers to follow, every output compared. The driving block's
// smoothing, sensitivity and wheel modes are Model 2's (tb_m2_cab_inputs
// there); here the defaults (Standard, Full, smoothing Off, pedals Normal)
// and the shifter's steps are checked.
#include "Vm3_inputs.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <algorithm>

static Vm3_inputs *t;
static void tick() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); }
static long n_chk = 0, n_bad = 0;
static void chk(const char *what, unsigned got, unsigned want, int prof, uint16_t j0) {
  n_chk++;
  if (got != want) {
    if (n_bad < 20) printf("FAIL prof %d j0 %04x %s: got %02x want %02x\n", prof, j0, what, got, want);
    n_bad++;
  }
}
static unsigned ob(int v) { return (unsigned)(v + 128) & 0xFF; }          // 0x80 + signed axis
static unsigned lo(uint16_t a) { return a & 0xFF; }
static unsigned hi(uint16_t a) { return a >> 8; }
static int sx(unsigned b) { return (int8_t)b; }
// R879's dead zone: up, down, left, right from the stick
static unsigned adir(uint16_t a) {
  int x = sx(lo(a)), y = sx(hi(a));
  return ((y < -48) << 3) | ((y > 48) << 2) | ((x < -48) << 1) | (x > 48);
}
// bit b of v
static unsigned B(unsigned v, int b) { return (v >> b) & 1; }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  t = new Vm3_inputs;
  std::mt19937 rng(argc > 1 ? atoi(argv[1]) : 1);
  t->rst_n = 0; tick(); t->rst_n = 1;
  int gear = 1;                       // the reference shifter (reset: first gear)
  uint16_t pj0 = 0;
  for (int it = 0; it < 40000; it++) {
    int prof = rng() % 5;             // 4: an unknown profile (reads as generic)
    if (it % 400 == 0) { t->rst_n = 0; tick(); t->rst_n = 1; gear = 1; pj0 = 0; }
    uint16_t j0 = rng() & 0xFFF0, j1 = rng() & 0xFFF0;
    if (rng() % 3 == 0) j0 |= rng() & 0xF;
    if (rng() % 3 == 0) j1 |= rng() & 0xF;
    uint16_t la0 = rng(), la1 = rng(), ra0 = rng();
    if (rng() % 4 == 0) la0 = 0;
    if (rng() % 4 == 0) ra0 = 0;
    unsigned lt = rng() % 4 ? 0 : rng() & 0xFF, rt = rng() % 4 ? 0 : rng() & 0xFF;
    unsigned pad = rng() % 8 ? 0 : rng() & 0xFF;
    unsigned idle_c = rng() % 2 ? 0xFF : 0xAF, idle_d = 0xFF;
    unsigned pflg = rng() & 1;
    t->prof = prof == 4 ? 7 : prof; t->pflg = pflg; t->idle_c = idle_c; t->idle_d = idle_d;
    t->joystick_0 = j0; t->joystick_1 = j1; t->joy_analog = la0; t->joy_analog_1 = la1; t->joy_ranalog = ra0;
    t->trig_lt = lt; t->trig_rt = rt; t->paddle = pad;
    t->st_pedswap = 0; t->st_ssens = 0; t->st_ssm = 0; t->st_rsteer = 0; t->st_wheel = 0;
    // the shifter steps on the edges of Gear Up (12) / Gear Down (13), whatever the profile
    // (both at once: Gear Down wins, as Model 2's m2_cab_inputs)
    { int g = gear;
      if (B(j0, 12) && !B(pj0, 12) && gear != 4) g = gear + 1;
      if (B(j0, 13) && !B(pj0, 13) && gear != 0) g = gear - 1;
      gear = g; }
    pj0 = j0;
    for (int k = 0; k < 8; k++) tick();   // the OSD synchronisers, the registered stages
    unsigned in0, in2, in3, tb = 1, sb = 1, an[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    unsigned d0 = (j0 & 0xF) | adir(la0), d1 = (j1 & 0xF) | adir(la1);
    auto dirs = [](unsigned d) { return (B(d, 1) << 7) | (B(d, 0) << 6) | (B(d, 3) << 5) | (B(d, 2) << 4); };
    if (prof == 1) {                  // driving: Coin Start Test Service VR1 VR4 Accel Brake GearUp GearDown VR2 VR3
      in0 = 0xFF & ~(B(j0, 4) | B(j1, 4) << 1 | B(j0, 6) << 2 | B(j0, 7) << 3 | B(j0, 5) << 4 | B(j1, 5) << 5);
      static const unsigned gv[5] = {0xF0, 0xA0, 0x90, 0xE0, 0xD0};   // Supermodel shift4: 1 ~0x50, 2 ~0x60, 3 ~0x10, 4 ~0x20
      in2 = gv[gear] | (0xF & ~(B(j0, 8) | B(j0, 14) << 1 | B(j0, 15) << 2 | B(j0, 9) << 3));
      in3 = idle_d;
      // steering: the paddle if it moves, the left stick past 12, else the d-pad's ramp (not modelled: skipped)
      int x = sx(lo(la0));
      bool stick = x > 12 || x < -12;
      if (pad) an[0] = pad;
      else if (stick) an[0] = std::min(255, std::max(0, 128 + x));
      else an[0] = 0x100;             // the ramp: not checked
      int y = sx(hi(ra0)); unsigned mag = y == -128 ? 127 : (unsigned)abs(y); unsigned pm = (mag << 1) | (mag >> 6);
      unsigned sa = y < 0 ? pm : 0, sbk = y < 0 ? 0 : pm;
      an[1] = B(j0, 10) ? 0xFF : std::max(rt, sa);
      an[2] = B(j0, 11) ? 0xFF : std::max(lt, sbk);
    } else if (prof == 2) {           // guns: Trigger Trigger2 Start Coin Test Service
      in0 = 0xFF & ~(B(j0, 7) | B(j1, 7) << 1 | B(j0, 8) << 2 | B(j0, 9) << 3 | B(j0, 6) << 4 | B(j1, 6) << 5);
      in2 = 0xFF & ~(B(j0, 4) | B(j0, 5) << 1);
      in3 = 0xFF & ~(B(j1, 4) | B(j1, 5) << 1);
      an[0] = ob(sx(lo(la0))); an[1] = ob(sx(hi(la0))); an[2] = ob(sx(lo(la1))); an[3] = ob(sx(hi(la1)));
    } else if (prof == 3) {           // fishing: Cast Select Reel Start Coin Test Service
      in0 = 0xFF & ~(B(j0, 8) | B(j1, 8) << 1 | B(j0, 9) << 2 | B(j0, 10) << 3 | B(j0, 7) << 4 | B(j1, 7) << 5);
      in2 = pflg ? (0xFF & ~(B(j0, 4) | B(j0, 5) << 1)) : (0xCF | B(j0, 4) << 4 | B(j0, 5) << 5);
      in3 = idle_d;
      an[0] = ob(sx(hi(la0))); an[1] = ob(sx(lo(la0)));
      an[3] = B(j0, 6) ? 0xFF : rt;
      an[4] = B(j0, 2) ? 0xFF : B(j0, 3) ? 0x00 : ob(sx(hi(ra0)));
      an[5] = B(j0, 0) ? 0xFF : B(j0, 1) ? 0x00 : ob(sx(lo(ra0)));
    } else {                          // generic: B1 B2 B3 B4 Start Coin Test Service, P2 the same
      in0 = 0xFF & ~(B(j0, 9) | B(j1, 9) << 1 | B(j0, 10) << 2 | B(j0, 11) << 3 | B(j0, 8) << 4 | B(j1, 8) << 5);
      in2 = idle_c & ~(dirs(d0) | ((j0 >> 4) & 0xF));
      in3 = idle_d & ~(dirs(d1) | ((j1 >> 4) & 0xF));
      tb = !B(j1, 10); sb = !B(j1, 11);
    }
    chk("in0", t->in0, in0, prof, j0);
    chk("in2", t->in2, in2, prof, j0);
    chk("in3", t->in3, in3, prof, j0);
    chk("test_b", t->test_b_n, tb, prof, j0);
    chk("service_b", t->service_b_n, sb, prof, j0);
    for (int c = 0; c < 8; c++) {
      if (an[c] > 0xFF) continue;
      char nm[8]; snprintf(nm, sizeof nm, "an%d", c);
      chk(nm, (unsigned)((t->an >> (8 * c)) & 0xFF), an[c], prof, j0);
    }
    chk("drive", t->drive, prof == 1, prof, j0);
  }
  // the d-pad's ramp: right held reaches 0xFF, left 0x00, released back to 0x80
  t->prof = 1; t->joystick_0 = 0; t->joy_analog = 0; t->joy_ranalog = 0; t->paddle = 0; t->trig_lt = t->trig_rt = 0;
  t->rst_n = 0; tick(); t->rst_n = 1;
  auto steer = [&]() { return (unsigned)(t->an & 0xFF); };
  for (int k = 0; k < 8; k++) tick();
  chk("ramp rest", steer(), 0x80, 1, 0);
  t->joystick_0 = 1; for (int k = 0; k < 1024 * 140; k++) tick();
  chk("ramp right", steer(), 0xFF, 1, 1);
  t->joystick_0 = 2; for (int k = 0; k < 1024 * 270; k++) tick();
  chk("ramp left", steer(), 0x00, 1, 2);
  t->joystick_0 = 0; for (int k = 0; k < 1024 * 140; k++) tick();
  chk("ramp back", steer(), 0x80, 1, 0);
  printf("m3_inputs: %ld checks, %ld fail\n", n_chk, n_bad);
  delete t;
  return n_bad ? 1 : 0;
}
