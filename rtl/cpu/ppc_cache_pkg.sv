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
// Shared by ppc_icache and ppc_dcache (603e manual chapter 3; study D-15):
// 16 KiB, 4 ways, 128 sets, 32-byte lines; physical tags PA[0-19].
//
// Memory port (one per cache; a controller arbitrates). One transaction at a
// time. m_req and its fields are held until the cycle m_ack is high.
//   MC_LINE_RD  line fill. m_addr is the critical word's address; eight
//               m_rvalid beats of 32 bits follow, critical word first,
//               wrapping within the line (figure 3-3, in words).
//   MC_LINE_WR  cast-out. m_addr is the line's base; the controller takes
//               eight beats in order, word 0 first, one per m_wnext; m_wdata
//               shows the beat it takes next.
//   MC_RD       single read, m_be lanes of the word at m_addr; one m_rvalid.
//   MC_WR       single write of m_wdata's m_be lanes; done at m_ack.
// Lanes are big-endian: m_be[3] is bits 31:24, the byte at offset 0.
// Requests are completed in the order they are accepted, and a read issued
// after a write sees it. The first read beat comes no earlier than the cycle
// after m_ack.

package ppc_cache_pkg;

  localparam logic [1:0] MC_LINE_RD = 2'd0;
  localparam logic [1:0] MC_LINE_WR = 2'd1;
  localparam logic [1:0] MC_RD      = 2'd2;
  localparam logic [1:0] MC_WR      = 2'd3;

  // Tree pseudo-LRU, 3 bits a set (HID0[ICFI]/[DCFI] in table 2-2: "the PLRU
  // bits point to way L0"; all zero names way 0). Bit 0 picks the half to
  // replace (0: ways 0-1), bit 1 the way in the low half, bit 2 in the high.
  function automatic logic [1:0] plru_victim(input logic [2:0] p);
    plru_victim = p[0] ? {1'b1, p[2]} : {1'b0, p[1]};
  endfunction

  // an access to way w points every bit on its path away from it
  function automatic logic [2:0] plru_touch(input logic [2:0] p, input logic [1:0] w);
    plru_touch    = p;
    plru_touch[0] = !w[1];
    if (!w[1]) plru_touch[1] = !w[0];
    else       plru_touch[2] = !w[0];
  endfunction

  // the lowest invalid way, and whether there is one (invalid ways are filled
  // first: manual 8.1.1, "if all lines in the set are marked as valid, the LRU
  // line is replaced")
  function automatic logic [2:0] first_free(input logic [3:0] inv);
    first_free = inv[0] ? 3'b100 : inv[1] ? 3'b101 : inv[2] ? 3'b110 : inv[3] ? 3'b111 : 3'b000;
  endfunction

  function automatic logic [1:0] enc4(input logic [3:0] h);
    enc4 = {h[3] | h[2], h[3] | h[1]};
  endfunction

endpackage
