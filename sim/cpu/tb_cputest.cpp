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
// The board test system (rtl/dbg/m3_cputest.sv) in simulation, with its UART
// output decoded bit by bit. What this prints is what the board must send:
// tools/uart_capture.py's capture of the same build is compared line by line
// (tools/cputest-compare.py). Built with a short UART divisor for speed; the
// report's values do not depend on it.
//
//   tb_cputest [runs]

#include "Vm3_cputest.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int runs = argc > 1 ? atoi(argv[1]) : 2;
  const int DIV = 8;                       // matches -GDIVISOR in the Makefile

  Vm3_cputest top;
  top.clk = 0; top.rst = 1; top.tb_tick = 0;
  for (int i = 0; i < 4; i++) { top.clk = 0; top.eval(); top.clk = 1; top.eval(); }
  top.rst = 0;

  std::string line;
  int reports = 0, phase = -1, bit = 0, cnt = 0;
  uint8_t byte = 0;
  for (uint64_t cyc = 0; cyc < 400000000ull; cyc++) {
    top.clk = 0; top.eval(); top.clk = 1; top.eval();
    bool tx = top.uart_tx;
    if (phase < 0) {                       // idle: wait for a start bit
      if (!tx) { phase = 0; cnt = DIV / 2; }
      continue;
    }
    if (--cnt > 0) continue;
    cnt = DIV;
    if (phase == 0) { phase = 1; bit = 0; byte = 0; continue; }      // middle of the start bit
    if (bit < 8) { byte |= uint8_t(tx) << bit; bit++; continue; }
    phase = -1;                             // stop bit
    if (byte == '\n') {
      printf("%s\n", line.c_str());
      if (line.rfind("P 0A", 0) == 0 && ++reports >= runs) return 0;
      line.clear();
    } else if (byte != '\r') line += char(byte);
  }
  printf("tb_cputest: did not finish\n");
  return 1;
}
