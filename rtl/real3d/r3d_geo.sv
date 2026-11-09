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
// Real3D geometry (M2): display-list walk to triangle list, Step 1.0, or the
// Step 1.5 node layout and 13.11 vertices with step15.
//
// Behaviour and FP32 operation order follow MAME 0.289 model3_v.cpp
// (BSD-3-Clause, R. Belmont, Ville Linde) through the reference model in
// sim/real3d/r3d_geo.cpp, which the bench compares this against triangle by
// triangle. Every FP operation runs on r3d_vpu, the one pool (4 multipliers,
// 4 adders, the correctly rounded divide): transform, light, clip (its divide
// included), projection, matrix stack and model set-up, and the frustum set-up's
// arithmetic. Beside it only the integer CORDIC of the correctly rounded
// set-up (one shared adder), or the square root of the cheap one.
//
// Four machines run at once, coupled through memories:
//   main     walker events: viewport set-up, matrix stack (in the engine's
//            register file blocks), model set-up, then T&L of each chunk of
//            up to 16 new vertices (inputs from the T&L staging memory,
//            results into the vertex store, ring of 4 chunks)
//   decoder  the model's polygons into chunks a chunk ahead: headers, shared
//            vertex slots (MLAB), 17.7 and s.7 conversion straight into the
//            staging memory, polygon records into rings of 32
//   polygon  per polygon of a transformed chunk: assemble from the vertex
//            store with per-plane outcodes, MAME's clip (W, -X, +X, -Y, +Y,
//            -Z, +Z) one crossing per CLIP kernel, then the vertices into the
//            projection queue (2 x 16); a full queue is projected (PROJ)
//   emitter  triangle fans out of the projection output buffer (2 x 16)
//   (SETUP_OUT: a set-up port between them, the triangles' set-up on the
//   engine, four a kernel, and the emitter sends set-up records)
// The engine is shared by main and polygon, one kernel at a time, granted
// in turn. Main runs up to two chunks ahead of the polygon machine; a shared
// vertex is reused from the store only within its chunk and the one before.
//
// Parameters (study D-19: Supermodel where MAME and Supermodel differ):
// FRUSTUM_SM = 1 (default) Supermodel's frustum: viewport words 8-11 in float,
// FrustumRZ (w = depth, z = near 1e-3, no far plane), depth z/w * 0.5 + 0.5;
// four subtracts and divides on the pool. FRUSTUM_SM = 0 MAME's, where
// SETUP_CR selects 1 correctly rounded tan(atan2(y, x) / 2) as MAME's libm
// computes it (r3d_setup_cr) or 0 the algebraic y / (sqrt(x^2 + y^2) + x).
// CULL = 1 (default) Supermodel's node culling; 0 MAME's walk of every node.
// SUBNORM selects FP units that keep subnormals (0: flush to zero; none reach
// the geometry in 800 frames).
// The other D-19 rows (m2-notes difference list; 1 = Supermodel, 0 = MAME):
// SM_FLAT (18 as built) vertex normals (2b + 1) / 255, not b / 127; row 18's
// face normals for flat polygons are a model switch only (study D-25);
// SM_FIXED (20) fixed shading from the vertex's x byte; SM_DOUBLE (21)
// double-sided polygons emitted twice, the inverted copy first (flag bit 0);
// SM_LIGHT (19, 22) Supermodel's lighting per vertex: normal matrix, light
// (n5, n6, -n4) in MAME's view space, sunFactor clamped to [0, 1] (sun clamp
// on, as VF3 and Bass Fishing set it), I = min(sf d + a, 1), shade 255 I;
// SM_TRANS (17) transparency-0 polygons dropped; SM_CTAB (12) the colour-table
// base from the walker (node words 3, 7, 8), 12-bit index.
// SM_CULL (study R62) Supermodel's back-face test, dot(view vertex 0, view
// face normal) > 0, beside the pool (r3d_face); FACE_DROP 1 drops what faces
// away (a single-sided back face before its clip, the inverted copy or the
// polygon of a double-sided one), 0 sends every record with its cull flag.
// LOD_WORD the LOD word in each polygon record (r3d_face LOD; viewport word 3
// read with the viewport, header word 5 into the facing test's slot).

module r3d_geo
  import r3d_ucode_pkg::*;
#(
  parameter bit FRUSTUM_SM = 1'b1,
  parameter bit SETUP_CR   = 1'b1,
  parameter bit CULL       = 1'b1,
  parameter bit SUBNORM    = 1'b0,
  parameter bit SM_FLAT    = 1'b1,
  parameter bit SM_FIXED   = 1'b1,
  parameter bit SM_DOUBLE  = 1'b1,
  parameter bit SM_LIGHT   = 1'b1,
  parameter bit SM_TRANS   = 1'b1,
  parameter bit SM_CTAB    = 1'b1,
  // measurement only (m2-notes section 12): after each projected batch the
  // polygon machine runs the rasteriser's set-up kernels on the pool, one per
  // four triangles (gen_ucode.py --setup-kernels); results unused
  // the projection output buffer in block RAM, read a cycle ahead: an ALM
  // and M10K trade (m2-notes section 14), on by default (study D-32)
  parameter bit OUT_M10K   = 1'b1,
  // the T&L staging and projection queue memories (16 of 32 x 32, read
  // registered either way) in block RAM, and the engine's broadcast file:
  // ALM and M10K trades (m2-notes section 14). SIDE_M10K on by default
  // (study D-32), BC_M10K off
  parameter bit SIDE_M10K  = 1'b1,
  parameter bit BC_M10K    = 1'b1,   // on since study R98 (-140 ALM, +8 M10K)
  // FP_DSP: the pool's and the facing test's FP adders shift in DSP blocks
  // (r3d_fp_add DSP_SHIFT, study R98: -316 ALM, +10 DSP, Fmax 111.7 -> 103.5
  // MHz standalone; 0 for a 110 MHz build)
  parameter bit FP_DSP     = 1'b0,
  parameter bit SETUP_LOAD = 1'b0,
  // with SETUP_LOAD (m2-notes section 14): SETUP_3R the engine's third
  // register read (the set-up kernels built with gen_ucode.py --3r). With
  // SM_CULL a back face gets no set-up and a double-sided polygon one (two
  // when d is exactly 0), by r3d_face's verdict
  parameter bit SETUP_3R   = 1'b0,
  // SETUP_PORT: the input port's load, the engine held idle while a kernel's
  // inputs go in through the external port, a lane word a cycle (18 cycles
  // for SETUP4, 12 for SETUP2)
  parameter bit SETUP_PORT = 1'b0,
  parameter bit SM_CULL    = 1'b1,
  parameter bit FACE_DROP  = 1'b1,
  // LOD_WORD (study D-33, R67): the polygon record carries the LOD word B =
  // d NP (-cota) kz^2 as header word 4 (5 + 6 n words), from r3d_face (needs
  // SM_CULL; 0 without it). 0 keeps R62's 4 + 6 n record.
  parameter bit LOD_WORD   = 1'b0,
  // SETUP_OUT (study D-28 option (a), R60, R65): the rasteriser's triangle
  // set-up on the pool. The output is set-up records (the emitter's header
  // comment), not polygon records. Takes port C (SETUP_3R) and
  // the ROM with the set-up kernels (r3d_ucode_su). SO_M10K: the set-up output
  // memory in block RAM (else MLAB).
  parameter bit SETUP_OUT  = 1'b0,
  parameter bit SO_M10K    = 1'b1
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        start,
  input  logic        step15,       // Step 1.5 and later: ten-word nodes, 13.11 vertices
  output logic        busy,
  output logic [7:0]  err,          // sticky: [0] walker limit [1] matrix stack [2] clip buffer

  output logic        cull_req,
  output logic [23:0] cull_addr,
  output logic [2:0]  cull_len,
  input  logic        cull_ack,
  input  logic        cull_rvalid,
  input  logic [31:0] cull_rdata,

  output logic        pram_req,
  output logic [23:0] pram_addr,
  output logic [2:0]  pram_len,
  input  logic        pram_ack,
  input  logic        pram_rvalid,
  input  logic [31:0] pram_rdata,

  output logic        vrom_req,
  output logic [23:0] vrom_addr,
  output logic [2:0]  vrom_len,
  input  logic        vrom_ack,
  input  logic        vrom_rvalid,
  input  logic [31:0] vrom_rdata,

  // polygon records, a word a beat (the emitter's header comment); out_last
  // marks a record's last word
  output logic        out_valid,
  input  logic        out_ready,
  output logic [31:0] out_data,
  output logic        out_last
);

  // ===========================================================================
  // walker and the culling-RAM port it shares with the geometry's reads
  // ===========================================================================
  logic        w_req, w_ack, w_rvalid, w_overflow, w_busy;
  logic [23:0] w_addr;
  logic [2:0]  w_len;
  logic        ev_valid, ev_ready;
  logic [2:0]  ev_type;
  logic [31:0] ev_a, ev_b, ev_c;
  logic        w_start;

  logic        cu_v, cu_out;
  r3d_walker #(.CULL(CULL), .CTAB(SM_CTAB)) u_walker (
    .clk, .rst, .start(w_start), .step15, .busy(w_busy), .overflow(w_overflow),
    .mem_req(w_req), .mem_addr(w_addr), .mem_len(w_len), .mem_ack(w_ack),
    .mem_rvalid(w_rvalid), .mem_rdata(cull_rdata),
    .ev_valid, .ev_ready, .ev_type, .ev_a, .ev_b, .ev_c, .cull_valid(cu_v), .cull_out(cu_out));

  logic        g_req, g_ack, g_rvalid;
  logic [23:0] g_addr;
  logic [2:0]  g_len;
  logic [1:0]  own;             // 0 none, 1 walker, 2 geometry
  logic [2:0]  own_beats;
  logic        own_acked;

  always_comb begin
    cull_req = 1'b0; cull_addr = w_addr; cull_len = w_len;
    if (own == 2'd1 && !own_acked) begin cull_req = w_req; cull_addr = w_addr; cull_len = w_len; end
    if (own == 2'd2 && !own_acked) begin cull_req = g_req; cull_addr = g_addr; cull_len = g_len; end
  end
  assign w_ack = (own == 2'd1) && !own_acked && cull_ack;
  assign g_ack = (own == 2'd2) && !own_acked && cull_ack;
  assign w_rvalid = (own == 2'd1) && own_acked && cull_rvalid;
  assign g_rvalid = (own == 2'd2) && own_acked && cull_rvalid;

  always_ff @(posedge clk) begin
    if (rst) begin
      own <= 2'd0; own_acked <= 1'b0;
    end else begin
      if (own == 2'd0) begin
        own_acked <= 1'b0;
        if (g_req) begin own <= 2'd2; own_beats <= g_len; end
        else if (w_req) begin own <= 2'd1; own_beats <= w_len; end
      end else if (!own_acked) begin
        if (cull_ack) own_acked <= 1'b1;
      end else if (cull_rvalid) begin
        if (own_beats == 3'd0) own <= 2'd0;
        else own_beats <= own_beats - 3'd1;
      end
    end
  end

  // ===========================================================================
  // helpers: number formats and IEEE comparisons
  // ===========================================================================
  // unsigned integer to FP32, times 2^eadj; exact
  function automatic logic [31:0] itof24(input logic sgn, input logic [23:0] mag, input logic [7:0] ebase);
    logic [4:0]  msb;
    logic        nz;
    logic [23:0] m;
    begin
      msb = 5'd0; nz = 1'b0;
      for (int i = 0; i < 24; i++) if (mag[i]) begin msb = 5'(i); nz = 1'b1; end
      m = mag << (5'd23 - msb);
      itof24 = nz ? {sgn, ebase + {3'd0, msb}, m[22:0]} : 32'd0;
    end
  endfunction
  function automatic logic [31:0] itof16(input logic [15:0] mag, input logic [7:0] ebase);
    logic [3:0]  msb;
    logic        nz;
    logic [15:0] m;
    begin
      msb = 4'd0; nz = 1'b0;
      for (int i = 0; i < 16; i++) if (mag[i]) begin msb = 4'(i); nz = 1'b1; end
      m = mag << (4'd15 - msb);
      itof16 = nz ? {1'b0, ebase + {4'd0, msb}, m[14:0], 8'd0} : 32'd0;
    end
  endfunction

  function automatic logic fisnan(input logic [31:0] a);
    fisnan = (a[30:23] == 8'hff) && (a[22:0] != 23'd0);
  endfunction
  // flt(r, b) for r >= +0 finite (the culling radius): b is above r only if
  // it is positive, not NaN and larger in magnitude
  function automatic logic rlt(input logic [31:0] r, input logic [31:0] b);
    rlt = !b[31] && !fisnan(b) && (r[30:0] < b[30:0]);
  endfunction

  // s.7 vertex normal / 127, correctly rounded (MAME: (float)(int8) / 127.0f):
  // 1/127 = 2^-7 * 1.000000100000010000001..., so n/127 for 0 < n < 127 is n * 2^-7
  // with the bits of n repeated every 7 places, rounded; 127 gives 1.0
  function automatic logic [31:0] ntab(input logic [7:0] nb);
    logic        s;
    logic [6:0]  n;
    logic [2:0]  msb;
    logic [27:0] rep;
    logic [27:0] sh;
    logic [23:0] m;
    begin
      s = nb[7];
      n = s ? 7'(8'd0 - nb) : nb[6:0];
      if (nb == 8'h80) ntab = 32'hbf810204;                     // -128 / 127
      else if (n == 7'd0) ntab = 32'h0000_0000;
      else if (n == 7'd127) ntab = {s, 31'h3f80_0000};
      else begin
        msb = 3'd0;
        for (int i = 0; i < 7; i++) if (n[i]) msb = 3'(i);
        rep = {n, n, n, n};                                      // n * (1 + 2^-7 + 2^-14 + 2^-21)
        sh = rep << (3'd6 - msb);                                // leading one at bit 27
        // 24-bit mantissa sh[27:4]; the tail repeats n, never a tie: round on bit 3
        m = sh[27:4] + {23'd0, sh[3]};
        ntab = {s, 8'(8'd120 + {5'd0, msb} - 8'd7 + 8'd7), m[22:0]};
      end
    end
  endfunction

  // m * (float)(1.0 / 255.0), correctly rounded, for m = 0..255: Supermodel's
  // BYTE_TO_FLOAT (2b + 1) / 255 (vertex normals, fixed shade), the unsigned
  // fixed shade and the ambient byte. K = 2^-8 * 1.00000001000000010000001b
  // In two steps, a register between them: k255a forms the product and its
  // leading one {zero, s, sh 4, p 32}, k255b shifts, rounds and packs.
  function automatic logic [37:0] k255a(input logic s, input logic [7:0] m);
    logic [31:0] p;
    logic [3:0]  sh;
    begin
      // m (2^23 + 2^15 + 2^7 + 1): the first three copies do not overlap, so one add (study R109; it was a
      // four-operand sum on the tq_w -> tc_k path)
      p = {1'b0, m, m, m, 7'd0} + {24'd0, m};
      // the product's leading one is m's, one place up only for m = 255
      // (checked for every m), so it does not wait for the sum
      sh = 4'd0;
      for (int i = 1; i < 8; i++) if (m[i]) sh = 4'(i);
      if (m == 8'hff) sh = 4'd8;
      k255a = {m == 8'd0, s, sh, p};
    end
  endfunction
  function automatic logic [31:0] k255b(input logic [37:0] a);
    logic [31:0] p;
    logic [24:0] q;
    logic [8:0]  rem, half;
    logic [3:0]  sh;
    logic [4:0]  e;
    begin
      p = a[31:0]; sh = a[35:32];                      // sh = 0..8
      e = 5'd23 + {1'b0, sh};
      q = 25'(p >> sh);
      rem = p[8:0] & 9'((10'd1 << sh) - 10'd1);
      half = (sh == 4'd0) ? 9'd0 : 9'(10'd1 << (sh - 4'd1));
      if (sh != 4'd0 && (rem > half || (rem == half && q[0]))) q = q + 25'd1;
      if (q[24]) begin q = q >> 1; e = e + 5'd1; end
      k255b = a[37] ? 32'd0 : {a[36], 8'(8'd96 + {3'd0, e}), q[22:0]};
    end
  endfunction
  function automatic logic [31:0] k255(input logic s, input logic [7:0] m);
    k255 = k255b(k255a(s, m));
  endfunction
  // the signed byte b as (2b + 1) / 255 (first step)
  function automatic logic [37:0] b2fa(input logic [7:0] b);
    logic [8:0] o;
    begin
      o = {b, 1'b1};                                   // 2b + 1, two's complement
      b2fa = k255a(b[7], b[7] ? 8'(9'd0 - o) : o[7:0]);
    end
  endfunction
  function automatic logic [31:0] b2f(input logic [7:0] b);
    b2f = k255b(b2fa(b));
  endfunction
  // Supermodel's diffuse, max(0, min(n7, 1)): NaN, zero and negatives give +0
  function automatic logic [31:0] dclamp(input logic [31:0] a);
    if ((a[30:23] == 8'hff && a[22:0] != 23'd0) || a[31] || a[30:0] == 31'd0) dclamp = 32'd0;
    else if (a[30:0] > 31'h3f80_0000) dclamp = 32'h3f80_0000;
    else dclamp = a;
  endfunction

  localparam logic [31:0] F_ONE = 32'h3f80_0000, F_TWO = 32'h4000_0000, F_M1 = 32'hbf80_0000;
  localparam logic [31:0] F_255 = 32'h437f_0000, F_WPLANE = 32'h3586_37bd;
  localparam logic [31:0] F_P22 = 32'hbf80_00a8, F_P23 = 32'hc000_0054;

  // inside tests for the 7 clip planes (0 W, 1 -X, 2 +X, 3 -Y, 4 +Y, 5 -Z, 6 +Z):
  // !flt(w, W_PLANE) and fle(-a, w), fle(a, w) for a = x, y, z, each pair from
  // one magnitude compare of a against w (miter against the fle form:
  // 200 M cases, 0 different)
  function automatic logic [1:0] opair(input logic [31:0] a, input logic [31:0] w);
    logic ok, bz, lt, eq;
    ok = !fisnan(a) && !fisnan(w);
    bz = a[30:0] == 31'd0 && w[30:0] == 31'd0;
    lt = a[30:0] < w[30:0];
    eq = a[30:0] == w[30:0];
    opair[1] = ok && (bz || (w[31] ? (a[31] && !lt) : (a[31] || lt || eq)));     // a <= w
    opair[0] = ok && (bz || (w[31] ? (!a[31] && !lt) : (!a[31] || lt || eq)));   // -a <= w
  endfunction
  function automatic logic [6:0] outc(input logic [31:0] x, input logic [31:0] y,
                                      input logic [31:0] z, input logic [31:0] w);
    outc[0] = fisnan(w) || (!w[31] && w[30:0] >= F_WPLANE[30:0]);
    outc[2:1] = opair(x, w);
    outc[4:3] = opair(y, w);
    outc[6:5] = opair(z, w);
  endfunction

  // ===========================================================================
  // engine, shared by main (m_) and the polygon machine (p_)
  // ===========================================================================
  logic        own_m, own_p, own_s;
  logic [1:0]  last_o;           // the last owner: 0 main, 1 polygon, 2 set-up
  logic        m_req, p_req, s_req;
  logic        vk_busy /* verilator public_flat_rd */;
  // one ext-port register set; only the owner drives it
  logic        vk_start;
  logic [9:0]  vk_entry /* verilator public_flat_rd */;
  logic [4:0]  vk_spa, vk_spd;
  logic [1:0]  vk_cls;           // 0 plain, 1 T&L, 2 projection, 3 set-up (SETUP_OUT)
  logic [3:0]  xw_we;
  logic [7:0]  xw_addr;
  logic [31:0] xw_data [4];
  logic        xb_we;
  logic [1:0]  xb_bank;
  logic [4:0]  xb_addr;
  logic [31:0] xb_data;
  logic [31:0] mb_data;          // M_MBW: the node matrix word for broadcast bank k, entry MB + i
  logic [6:0]  x_raddr;
  logic [31:0] x_rdata [4] /* verilator public_flat_rd */;
  logic [6:0]  sa_addr, sw_addr;
  logic [31:0] sa_data [4], sw_data [4];
  logic [3:0]  sw_we;

  r3d_vpu #(.SUBNORM(SUBNORM), .SM_LIGHT(SM_LIGHT), .SETUP_3R(SETUP_3R || SETUP_OUT), .BC_M10K(BC_M10K),
            .SU_ROM(SETUP_OUT), .FP_DSP(FP_DSP)) u_vpu (
    .clk, .rst, .start(vk_start), .entry(vk_entry), .sp_a(vk_spa), .sp_d(vk_spd), .busy(vk_busy),
    .ext_we(xw_we), .ext_waddr(xw_addr), .ext_wdata(xw_data),
    .ext_bwe(xb_we), .ext_bbank(xb_bank), .ext_baddr(xb_addr), .ext_bdata(xb_data),
    .ext_raddr(x_raddr), .ext_rdata(x_rdata),
    .sa_addr, .sa_data, .sw_we, .sw_addr, .sw_data);

  // grant in turn (main, polygon, set-up), only while the engine is idle; the
  // owner releases by dropping its request once its kernel is done. Without
  // SETUP_OUT (s_req never set) this is the two-way turn of main and polygon.
  always_ff @(posedge clk) begin
    if (rst) begin
      own_m <= 1'b0; own_p <= 1'b0; own_s <= 1'b0; last_o <= 2'd0;
    end else begin
      // a release hands the engine straight to the next machine that waits
      if (own_m && !m_req) begin
        own_m <= 1'b0;
        if (!vk_busy) begin
          if (p_req) begin own_p <= 1'b1; last_o <= 2'd1; end
          else if (s_req) begin own_s <= 1'b1; last_o <= 2'd2; end
        end
      end
      if (own_p && !p_req) begin
        own_p <= 1'b0;
        if (!vk_busy) begin
          if (s_req) begin own_s <= 1'b1; last_o <= 2'd2; end
          else if (m_req) begin own_m <= 1'b1; last_o <= 2'd0; end
        end
      end
      if (own_s && !s_req) begin
        own_s <= 1'b0;
        if (!vk_busy) begin
          if (m_req) begin own_m <= 1'b1; last_o <= 2'd0; end
          else if (p_req) begin own_p <= 1'b1; last_o <= 2'd1; end
        end
      end
      if (!own_m && !own_p && !own_s && !vk_busy) begin
        case (last_o)
          2'd0: if (p_req) begin own_p <= 1'b1; last_o <= 2'd1; end
                else if (s_req) begin own_s <= 1'b1; last_o <= 2'd2; end
                else if (m_req) begin own_m <= 1'b1; last_o <= 2'd0; end
          2'd1: if (s_req) begin own_s <= 1'b1; last_o <= 2'd2; end
                else if (m_req) begin own_m <= 1'b1; last_o <= 2'd0; end
                else if (p_req) begin own_p <= 1'b1; last_o <= 2'd1; end
          default: if (m_req) begin own_m <= 1'b1; last_o <= 2'd0; end
                else if (p_req) begin own_p <= 1'b1; last_o <= 2'd1; end
                else if (s_req) begin own_s <= 1'b1; last_o <= 2'd2; end
        endcase
      end
    end
  end

  // engine kernel context: which side memories, which banks
  logic [1:0]  e_cls;
  logic        e_tbank, e_pbuf, e_sbuf;
  logic [1:0]  e_ring;
  logic        k_tbank, k_pbuf, k_sbuf;  // set by the owner with vk_start (k_sbuf: the set-up output half)
  logic [1:0]  k_ring;
  always_ff @(posedge clk) if (vk_start) begin
    e_cls <= vk_cls; e_tbank <= k_tbank; e_pbuf <= k_pbuf; e_ring <= k_ring; e_sbuf <= k_sbuf;
  end

  // ===========================================================================
  // side memories, one per lane
  // ===========================================================================
  // T&L staging (decoder writes): positions and normals, {bank, row, comp}
  logic [3:0]  ts_we;
  logic [4:0]  ts_wa;
  logic [31:0] ts_wp, ts_wn;
  // projection queue (polygon machine writes): x y z w | u v shade, {buf, row, comp}.
  // SETUP_OUT: its upper half {1, buf, row, comp} is the set-up staging, the
  // set-up kernel's inputs, written a word at a time by the set-up port (one of
  // the two memories, pq_wea / pq_web) and read by the kernel through the side port
  localparam int PQA = SETUP_OUT ? 6 : 5;
  logic [3:0]  pq_we, pq_wea, pq_web;
  logic [PQA-1:0] pq_wa;
  logic [31:0] pq_wa_d, pq_wb_d;
  // vertex store (engine writes in T&L): x y | z w | shade, {ring, row, comp}
  logic [4:0]  vs_ra;            // {ring, row, half}
  logic [31:0] vsa_q [4], vsb_q [4], vsc_q [4];   // (x, y) (z, w) (shade, shade')
  // projection output (engine writes in PROJ), {buf, row, comp}
  logic [5:0]  po_ra;
  logic [31:0] po_q [4];
  // set-up output (SETUP_OUT; engine writes in SETUP4 / SETUP2), {half, word}
  logic [5:0]  so_ra;
  logic [31:0] so_q [4];

  logic        sa_hi;
  always_ff @(posedge clk) sa_hi <= sa_addr[2];
  genvar gl;
  generate
  for (gl = 0; gl < 4; gl++) begin : g_side
    logic [31:0] tp_q, tn_q, qa_q, qb_q;
    r3d_ram #(.W(32), .A(5), .SYNC(1'b1), .BLOCK(SIDE_M10K)) u_tsp (.clk, .we(ts_we[gl]), .wa(ts_wa), .wd(ts_wp),
      .ra({e_tbank, sa_addr[4:3], sa_addr[1:0]}), .rq(tp_q));
    r3d_ram #(.W(32), .A(5), .SYNC(1'b1), .BLOCK(SIDE_M10K)) u_tsn (.clk, .we(ts_we[gl]), .wa(ts_wa), .wd(ts_wn),
      .ra({e_tbank, sa_addr[4:3], sa_addr[1:0]}), .rq(tn_q));
    logic [PQA-1:0] pq_ra;
    if (SETUP_OUT) begin : g_pqs
      assign pq_ra = {e_cls == 2'd3, e_pbuf, sa_addr[4:3], sa_addr[1:0]};
    end else begin : g_pqn
      assign pq_ra = {e_pbuf, sa_addr[4:3], sa_addr[1:0]};
    end
    r3d_ram #(.W(32), .A(PQA), .SYNC(1'b1), .BLOCK(SIDE_M10K)) u_pqa (.clk, .we(SETUP_OUT ? pq_wea[gl] : pq_we[gl]), .wa(pq_wa), .wd(pq_wa_d),
      .ra(pq_ra), .rq(qa_q));
    r3d_ram #(.W(32), .A(PQA), .SYNC(1'b1), .BLOCK(SIDE_M10K)) u_pqb (.clk, .we(SETUP_OUT ? pq_web[gl] : pq_we[gl]), .wa(pq_wa), .wd(pq_wb_d),
      .ra(pq_ra), .rq(qb_q));
    assign sa_data[gl] = (e_cls == 2'd1) ? (sa_hi ? tn_q : tp_q) : (sa_hi ? qb_q : qa_q);

    logic vs_w;
    assign vs_w = sw_we[gl] && e_cls == 2'd1;
    // the vertex store's write address through a LUT of its own (keep), as u_lvl's (R104 fix 21): straight
    // from sw_addr it missed hold by 0.030 / 0.059 ns at the slow corners in a standalone fit (study R109)
    (* keep = 1 *) wire [4:0] vs_wa = {e_ring, sw_addr[4:3], sw_addr[0]};
    r3d_ram #(.W(32), .A(5)) u_vsa (.clk, .we(vs_w && sw_addr[2:1] == 2'd0), .wa(vs_wa),
      .wd(sw_data[gl]), .ra(vs_ra), .rq(vsa_q[gl]));
    r3d_ram #(.W(32), .A(5)) u_vsb (.clk, .we(vs_w && sw_addr[2:1] == 2'd1), .wa(vs_wa),
      .wd(sw_data[gl]), .ra(vs_ra), .rq(vsb_q[gl]));
    r3d_ram #(.W(32), .A(5)) u_vsc (.clk, .we(vs_w && sw_addr[2]), .wa(vs_wa),
      .wd(sw_data[gl]), .ra(vs_ra), .rq(vsc_q[gl]));
    r3d_ram #(.W(32), .A(6), .SYNC(OUT_M10K), .BLOCK(OUT_M10K)) u_po (.clk, .we(sw_we[gl] && e_cls == 2'd2), .wa({e_pbuf, sw_addr[4:0]}),
      .wd(sw_data[gl]), .ra(po_ra), .rq(po_q[gl]));
    // SETUP_OUT: the set-up kernel's results, a 32-word slot a lane in each of two halves
    if (SETUP_OUT) begin : g_so
      r3d_ram #(.W(32), .A(6), .SYNC(1'b1), .BLOCK(SO_M10K)) u_so (.clk, .we(sw_we[gl] && e_cls == 2'd3), .wa({e_sbuf, sw_addr[4:0]}),
        .wd(sw_data[gl]), .ra(so_ra), .rq(so_q[gl]));
    end else begin : g_noso
      assign so_q[gl] = 32'd0;
    end
  end
  endgenerate

  // ===========================================================================
  // set-up units beside the pool: CORDIC (SETUP_CR) or square root
  // ===========================================================================
  logic        sq_in, sq_busy, sq_ov;
  logic [31:0] sq_x, sq_q;
  logic        cr_in, cr_busy, cr_ov;
  logic [31:0] cr_y, cr_x, cr_q;
  generate if (!FRUSTUM_SM && SETUP_CR) begin : g_cr
    r3d_setup_cr u_cr (.clk, .rst, .in_valid(cr_in), .y(cr_y), .x(cr_x), .busy(cr_busy), .out_valid(cr_ov), .result(cr_q), .bad());
    assign sq_busy = 1'b0; assign sq_ov = 1'b0; assign sq_q = 32'd0;
  end else if (!FRUSTUM_SM) begin : g_alg
    r3d_fsqrt u_sqrt (.clk, .rst, .in_valid(sq_in), .x(sq_x), .busy(sq_busy), .out_valid(sq_ov), .result(sq_q));
    assign cr_busy = 1'b0; assign cr_ov = 1'b0; assign cr_q = 32'd0;
  end else begin : g_sm
    assign sq_busy = 1'b0; assign sq_ov = 1'b0; assign sq_q = 32'd0;
    assign cr_busy = 1'b0; assign cr_ov = 1'b0; assign cr_q = 32'd0;
  end endgenerate

  // ===========================================================================
  // the back-face test beside the pool (SM_CULL): the model matrix from main,
  // vertex 0 and the face normal from the decoder, a verdict a polygon in ring
  // order into fv (read by the polygon machine)
  // ===========================================================================
  logic        f_twe /* verilator public_flat_rd */, f_thold /* verilator public_flat_rd */, f_tclr, f_tset;
  logic [1:0]  f_mbbusy;
  logic        f_ready, f_idle;
  logic        f_hold /* verilator public_flat_rd */, f_we /* verilator public_flat_rd */, f_commit /* verilator public_flat_rd */;
  logic        f_vv /* verilator public_flat_rd */, f_gt /* verilator public_flat_rd */, f_lt /* verilator public_flat_rd */;
  logic [2:0]  f_word /* verilator public_flat_rd */;
  logic [23:0] f_d /* verilator public_flat_rd */;
  logic        tbuf_m /* verilator public_flat_rd */, tbuf_d /* verilator public_flat_rd */;    // the model's matrix buffer, as main and the decoder count models
  logic [1:0]  ft_row /* verilator public_flat_rd */, ft_col /* verilator public_flat_rd */;
  // LOD_WORD: the viewport's -cota in (main, M_VPW), B a polygon out in ring order
  logic        f_cwe;
  logic [31:0] f_cd;
  logic        f_bv;
  logic [31:0] f_bw;
  // kz^2: 1 with Supermodel's frustum, else MAME's constant kz = -F_P23 squared (FP32)
  localparam logic [31:0] F_KZ2 = FRUSTUM_SM ? 32'h3f80_0000 : 32'h4080_00a8;
  generate if (SM_CULL) begin : g_face
    r3d_face #(.LOD(LOD_WORD), .KZ2(F_KZ2), .FP_DSP(FP_DSP)) u_face (.clk, .rst, .step15,
      .t_we(f_twe), .t_buf(tbuf_m), .t_row(ft_row), .t_col(ft_col), .t_d(x_rdata[ft_col]), .t_hold(f_thold),
      .t_clr(f_tclr), .t_set(f_tset), .mb_busy(f_mbbusy),
      .in_ready(f_ready), .in_hold(f_hold), .in_we(f_we), .in_word(f_word), .in_d(f_d),
      .in_commit(f_commit), .in_mb(tbuf_d),
      .v_valid(f_vv), .v_gt(f_gt), .v_lt(f_lt), .idle(f_idle),
      .c_we(f_cwe), .c_d(f_cd), .b_valid(f_bv), .b_word(f_bw));
  end else begin : g_noface
    assign f_thold = 1'b0; assign f_mbbusy = 2'b00; assign f_ready = 1'b1; assign f_hold = 1'b0;
    assign f_vv = 1'b0; assign f_gt = 1'b0; assign f_lt = 1'b0; assign f_idle = 1'b1;
    assign f_bv = 1'b0; assign f_bw = 32'd0;
  end endgenerate

  // ===========================================================================
  // model stream
  // ===========================================================================
  logic        ms_start, ms_stop, ms_ready, ms_wv, ms_pop, ms_creq, ms_cv;
  logic [23:0] ms_addr;
  logic [31:0] ms_w, ms_cd;
  logic [19:0] ms_caddr;
  r3d_mstream u_ms (
    .clk, .rst, .start(ms_start), .addr(ms_addr), .stop(ms_stop), .ready_start(ms_ready),
    .w_valid(ms_wv), .w_data(ms_w), .w_pop(ms_pop),
    .c_req(ms_creq), .c_addr(ms_caddr), .c_valid(ms_cv), .c_data(ms_cd),
    .pram_req, .pram_addr, .pram_len, .pram_ack, .pram_rvalid, .pram_rdata,
    .vrom_req, .vrom_addr, .vrom_len, .vrom_ack, .vrom_rvalid, .vrom_rdata);

  // ===========================================================================
  // polygon rings (32 polygons), written by the decoder
  // ===========================================================================
  // assembly record: {double-sided, ex4, unlit, scale, nv[2:0], ref3..ref0 (6 each)}
  logic        ra_we;
  logic [5:0]  ra_wa, ra_ra;
  logic [30:0] ra_wd, ra_q;
  r3d_ram #(.W(31), .A(5)) u_rasm (.clk, .we(ra_we), .wa(ra_wa[4:0]), .wd(ra_wd), .ra(ra_ra[4:0]), .rq(ra_q));
  logic [2:0]  rs_wd, su_q;      // SETUP_LOAD, SETUP_OUT: the polygon's {fixed shade, double-sided, textured}
  generate if (SETUP_LOAD || SETUP_OUT) begin : g_su
    r3d_ram #(.W(3), .A(5)) u_rsu (.clk, .we(ra_we), .wa(ra_wa[4:0]), .wd(rs_wd), .ra(ra_ra[4:0]), .rq(su_q));
  end else begin : g_nosu
    assign su_q = 3'd0;
  end endgenerate
  // emission record: {layer 1, fog 5, fixed 1, double 1, colour 32, transparency 6, param 6, fmt 3,
  // tex_h 3, tex_w 3, tex_y 5, tex_x 6, page 1, tex_en 1, poly 20, alpha 1, pass 2}; param's bit 0 (45) is
  // texture alpha alone (study R135; it was the page again); and above the layer bit (97) texture inversion
  // (header[6] bits 30:24 = 2 on a textured polygon: Supermodel's TranslatorMapOffset, study R135)
  logic        re_we;
  logic [5:0]  re_wa, re_ra;
  logic [97:0] re_wd, re_q;
  // stored without bit 23 (tex_en), which repeats bit 48
  logic [96:0] re_sq;
  r3d_ram #(.W(97), .A(5)) u_remi (.clk, .we(re_we), .wa(re_wa[4:0]), .wd({re_wd[97:24], re_wd[22:0]}), .ra(re_ra[4:0]), .rq(re_sq));
  assign re_q = {re_sq[96:23], re_sq[47], re_sq[22:0]};
  // u/v words, {polygon, vertex}, block RAM
  logic        pu_we;
  logic [7:0]  pu_wa, pu_ra;
  logic [31:0] pu_wd, pu_q;
  r3d_ram #(.W(32), .A(8), .SYNC(1'b1), .BLOCK(1'b1)) u_puv (.clk, .we(pu_we), .wa(pu_wa), .wd(pu_wd), .ra(pu_ra), .rq(pu_q));

  // ===========================================================================
  // queues between the machines
  // ===========================================================================
  // decoder -> main: chunk descriptors, two deep (the staging memory's banks)
  logic [5:0]  dq_start [2], dq_end [2];
  logic [4:0]  dq_tl [2];
  logic [1:0]  dq_ring [2];
  logic        dq_bank [2], dq_last [2];
  logic [1:0]  dq_cnt /* verilator public_flat_rd */;
  logic        dq_wp, dq_rp, dq_push, dq_pop;
  logic [5:0]  dq_wstart, dq_wend;
  logic [4:0]  dq_wtl;
  logic [1:0]  dq_wring;
  logic        dq_wbank, dq_wlast;
  // main -> polygon: transformed chunks, four deep
  logic [5:0]  cq_start [4], cq_end [4];
  logic [1:0]  cq_ring [4];
  logic [2:0]  cq_cnt /* verilator public_flat_rd */;
  logic [1:0]  cq_wp, cq_rp;
  logic        cq_push, cq_pop;
  // polygon -> emitter: projected batches, two deep (the output buffer's halves)
  logic        eb_buf [2];
  logic [4:0]  eb_np [2];
  logic [5:0]  eb_first [2];
  logic [1:0]  eb_cnt;
  logic        eb_wp, eb_rp, eb_push, eb_pop;
  logic        eb_wbuf;
  logic [4:0]  eb_wnp;
  logic [5:0]  eb_wfirst;
  logic [1:0]  po_busy;
  // batch polygon list: {buf, slot} -> {verdict 2, count, base, ring index}; with
  // SETUP_OUT {fixed shade, double-sided, textured} above them
  localparam int PMW = SETUP_OUT ? 20 : 17;
  logic        pm_we;
  logic [4:0]  pm_wa, pm_ra;
  logic [PMW-1:0] pm_wd, pm_q;
  r3d_ram #(.W(PMW), .A(5)) u_pqm (.clk, .we(pm_we), .wa(pm_wa), .wd(pm_wd), .ra(pm_ra), .rq(pm_q));

  // ===========================================================================
  // state
  // ===========================================================================
  localparam int SDEPTH = 32;

  typedef enum logic [6:0] {
    M_IDLE, M_FRI0, M_FRI1, M_EV, M_CR, M_CRW, M_K, M_KW, M_OWN,
    M_VP0, M_VP1, M_VP2, M_VP3, M_VP4, M_VP5, M_VP6, M_VPD, M_VPW, M_VPC, M_VPK,
    M_CRS0, M_CRS1, M_CRS2,
    M_AL0, M_AL1, M_AL2, M_AL3, M_AL4, M_AL5, M_AL6, M_AL7,
    M_PJ0, M_PJ1, M_PJ2, M_PJ3, M_PJ4, M_VP7, M_SM0,
    M_CU0, M_CU1, M_CU2, M_CU3,
    M_PUSH, M_POP, M_TR0, M_MAT0, M_MAT1, M_MBW, M_MBK, M_MBD,
    M_MODEL0, M_MODEL1, M_MREL, M_CHW, M_CHT, M_CHD, M_CHX, M_CHE, M_CHG, M_PJW, M_END,
    M_MODELW, M_FT0, M_FT1, M_FT2, M_VR0, M_VR1, M_VP8
  } mst_t;
  mst_t mst /* verilator public_flat_rd */;
  mst_t mret, kret, cret;

  typedef enum logic [3:0] {D_IDLE, D_HDR, D_CHK, D_REUSE, D_NEWV, D_INV, D_COL, D_COLW, D_COMMIT,
                            D_END, D_CLOSE, D_HOLD} dst_t;
  dst_t dst /* verilator public_flat_rd */;
  dst_t d_after;

  typedef enum logic [5:0] {
    P_IDLE, P_POLY, P_A, P_B, P_C, P_D,
    P_CL0, P_CL1, P_CLN, P_CLA,
    P_X0, P_X1, P_X2, P_X3, P_X4, P_X5, P_X6, P_X7, P_X8, P_X9, P_XA, P_XB, P_XC,
    P_Q0, P_Q1, P_PJ0, P_PJ1, P_PJ2, P_PJ3, P_GAP, P_SU0, P_SU1, P_SU2
  } pst_t;
  pst_t pst /* verilator public_flat_rd */;
  pst_t pj_ret;

  typedef enum logic [2:0] {E_IDLE, E_P0, E_P1, E_P2, E_OUT, E_VR} est_t;
  est_t est;

  assign busy = (mst != M_IDLE) || start;
  assign ev_ready = (mst == M_EV);

  // ===========================================================================
  // main
  // ===========================================================================
  logic [1:0]  pass;
  logic [31:0] vaddr, mbase;
  logic [31:0] cr_addr32;
  logic [2:0]  cr_len, cr_beat;
  logic [4:0]  cr_dst;           // first viewport-word slot of a culling read
  logic [4:0]  cnt;
  logic [1:0]  lane;
  logic        bc_top_ok;        // the broadcast copy of the top is current
  logic [4:0]  sp, top_slot;     // stack level; the slot holding its matrix
  // level -> slot: a pushed level without a matrix of its own shares the slot below
  logic        lv_we;
  logic [4:0]  lv_wa, lv_wd, lv_ra, lv_q;
  // the MLAB's write address through a LUT of its own (keep), as the core's SPR array: straight from
  // lv_wa it missed hold by 0.08-0.10 ns at both slow corners in a 100 MHz build (study R104, t100d3)
  (* keep = 1 *) wire [4:0] lv_wa_k = lv_wa;
  r3d_ram #(.W(5), .A(5)) u_lvl (.clk, .we(lv_we), .wa(lv_wa_k), .wd(lv_wd), .ra(lv_ra), .rq(lv_q));
  assign lv_ra = sp - 5'd1;
  logic [15:0] cu_r;
  logic        m_drain;
  logic [31:0] vq_r;             // a viewport word, registered before its conversion
  logic        amb_ld;           // M_VPW: the ambient byte into the decoder's k255 (tc_k)
  logic [7:0]  amb_b;            // the ambient byte, registered beside amb_ld (vq_r[15:8] then)
  logic [15:0] cv_in;            // the one int16 converter: viewport words (M_VPW), u and v (P_B, P_C)
  logic [7:0]  cv_eb;
  logic [31:0] cv_q;
  logic        cu_o;
  logic        tr_mode;          // M_MBW: a translation, not a matrix
  logic [1:0]  cnt_q;            // projection queue: component pair
  // viewport and matrix words: [0..3] node words 4-7, [4..11] 12-19, [12] 20,
  // [13] 26, [14] 36, [16..27] a matrix (coordinate, node, or a translation in
  // 16..18), [28..31] the frustum tangents; set-up temporaries reuse 16..27
  logic        vw_we;
  logic [4:0]  vw_wa, vw_ra;
  logic [31:0] vw_wd, vw_q;
  r3d_ram #(.W(32), .A(5)) u_vpw (.clk, .we(vw_we), .wa(vw_wa), .wd(vw_wd), .ra(vw_ra), .rq(vw_q));

  // the viewport record: main's request, the emitter's word slot and done
  logic        vr_go, vr_done;
  logic [4:0]  vr_ra;
  assign f_twe = SM_CULL && mst == M_FT1;

  // polygon-machine state the main machine waits on
  logic        p_quiet, e_quiet;
  logic        d_quiet;

  // decoder handshake
  logic        d_req, d_ack, f_init;
  logic [1:0]  pass_d;
  logic [19:0] m_ctab, d_ctab;   // the model's colour-table base (SM_CTAB)

  // the chunk main is transforming
  logic [5:0]  c_start, c_end;
  logic [4:0]  c_tl;
  logic [1:0]  c_ring;
  logic        c_bank, c_last;

  // culling-RAM read addresses of the viewport and matrix states: one adder
  logic [31:0] ra_sum;
  always_comb begin
    logic [31:0] b;
    logic [5:0]  o;
    b = vaddr; o = 6'd8;
    case (mst)
      M_VP0: o = 6'd4;
      M_VP1: o = 6'd12;
      M_VP2: o = 6'd20;
      M_VP3: o = 6'd26;
      M_VP4: o = 6'd36;
      M_VR0: o = 6'd32;
      M_VP8: o = 6'd3;
      M_VP5: begin b = mbase; o = 6'd0; end
      M_VP6: b = mbase;
      M_MAT0: b = cr_addr32;
      default: ;                                       // M_VP7: vaddr + 8
    endcase
    ra_sum = b + {26'd0, o};
  end

  // viewport-word read address for each main state (lane l pairs (y, x):
  // 0 (w12, w13), 1 (w16, -w17), 2 (w14, w15), 3 (w18, -w19) = vpw (4, 5), (8, 9), (6, 7), (10, 11))
  function automatic logic [4:0] vy_of(input logic [1:0] l);
    vy_of = (l == 2'd0) ? 5'd4 : (l == 2'd1) ? 5'd8 : (l == 2'd2) ? 5'd6 : 5'd10;
  endfunction
  always_comb begin
    vw_ra = 5'd0;
    case (mst)
      M_VPW: case (cnt)
               5'd0: vw_ra = 5'd1; 5'd1: vw_ra = 5'd2; 5'd2: vw_ra = 5'd0; 5'd3: vw_ra = 5'd3;
               5'd4: vw_ra = 5'd14; 5'd5, 5'd7: vw_ra = 5'd12; 5'd11: vw_ra = 5'd0;
               5'd12: vw_ra = LOD_WORD ? 5'd15 : 5'd13; default: vw_ra = 5'd13;
             endcase
      M_VPC: case (cnt[1:0]) 2'd0: vw_ra = 5'd22; 2'd1: vw_ra = 5'd26; default: vw_ra = 5'd21; endcase
      // CF row i = cnt[3:2], lane l = cnt[1:0]: row 3 = (w0 w1 w2), else w(3 + i + 3l);
      // then PA lane l = -w(2l) of the planes, PB / PC lane l = -w(2l + 1)
      M_VPK: if (!CULL) ; else if (cnt < 5'd16) vw_ra = (cnt[3:2] == 2'd3) ? 5'(16 + cnt[1:0]) : 5'd19 + {3'd0, cnt[3:2]} + 5'd3 * {3'd0, cnt[1:0]};
             else begin
               vw_ra = ((cnt[3:2] == 2'd0) ? 5'd4 : 5'd5) + {2'd0, cnt[1:0], 1'b0};
             end
      M_CRS0: if (!FRUSTUM_SM && SETUP_CR) vw_ra = vy_of(cnt[1:0]);
      M_CRS1: if (!FRUSTUM_SM && SETUP_CR) vw_ra = vy_of(cnt[1:0]) + 5'd1;
      M_AL0, M_AL6: if (!FRUSTUM_SM && !SETUP_CR) vw_ra = cnt[0] ? 5'(20 + lane) : vy_of(lane) + 5'd1;  // x (or the root)
      M_AL1, M_AL5: if (!FRUSTUM_SM && !SETUP_CR) vw_ra = vy_of(lane);                                   // y
      M_AL2: if (!FRUSTUM_SM && !SETUP_CR) vw_ra = 5'(16 + lane);                                         // xx
      M_PJ0: if (!FRUSTUM_SM) case (cnt[2:0])                                                // GA {l r t r}, GC {r l b l}
               3'd0: vw_ra = 5'd28; 3'd1: vw_ra = 5'd29; 3'd2: vw_ra = 5'd30; 3'd3: vw_ra = 5'd29;
               3'd4: vw_ra = 5'd29; 3'd5: vw_ra = 5'd28; 3'd6: vw_ra = 5'd31; default: vw_ra = 5'd28;
             endcase
      M_PJ1: if (!FRUSTUM_SM) case (cnt[2:0])                                                // GA {r t r t}, GC {l b l b}
               3'd0: vw_ra = 5'd29; 3'd1: vw_ra = 5'd30; 3'd2: vw_ra = 5'd29; 3'd3: vw_ra = 5'd30;
               3'd4: vw_ra = 5'd28; 3'd5: vw_ra = 5'd31; 3'd6: vw_ra = 5'd28; default: vw_ra = 5'd31;
             endcase
      M_PJ2: if (!FRUSTUM_SM) vw_ra = (cnt[1:0] == 2'd3) ? 5'd18 : 5'(16 + cnt[1:0]);       // GA {l-r, r-l, t-b, t-b}
      M_PJ3: if (!FRUSTUM_SM) vw_ra = (cnt[1:0] == 2'd1) ? 5'd20 : 5'd21;                   // GB {2, r+l, 2, t+b}
      // Supermodel set-up: vpw 28 cv, 29 cw, 30 io, 31 jo; 16..19 left right bottom top
      M_SM0: if (FRUSTUM_SM) vw_ra = cnt[2] ? (cnt[1] ? 5'd29 : 5'd28) : (cnt[1] ? 5'd30 : 5'd31);   // SC {jo jo io io}, SB {cv cv cw cw}
      M_MBW: vw_ra = (cnt[3:2] == 2'd3) ? 5'(16 + cnt[1:0]) : 5'd19 + {3'd0, cnt[3:2]} + 5'd3 * {3'd0, cnt[1:0]};
      M_VR1: vw_ra = vr_ra;                                  // the viewport record's words (emitter)
      default: ;
    endcase
  end

  // M[i][k] -> bank k, entry MB + i: rows (w3 w6 w9 0) (w4 w7 w10 0) (w5 w8 w11 0) (w0 w1 w2 1)
  // for a matrix; the identity with row 3 = (tx ty tz 1) for a translation
  always_comb begin
    logic [1:0] i, k;
    i = cnt[3:2]; k = cnt[1:0];
    if (k == 2'd3) mb_data = (i == 2'd3) ? F_ONE : 32'd0;
    else if (i == 2'd3) mb_data = vw_q;
    else if (tr_mode) mb_data = (i == k) ? F_ONE : 32'd0;
    else mb_data = vw_q;
  end

  // set-up tangent l, r, t, b from vpw 28..31 (r and b negated)
  function automatic logic [31:0] negf(input logic [31:0] a); negf = {~a[31], a[30:0]}; endfunction

  // ===========================================================================
  // polygon machine state
  // ===========================================================================
  logic [5:0]  cp, pc_end;
  logic [1:0]  pc_ring;
  logic [2:0]  as_i, a_nv, a_wi;
  logic [31:0] ax, ay, az, aw, ash, ash2;       // the vertex being assembled / a new clip vertex
  logic        acc;
  logic [6:0]  cur_and, cur_or, nxt_and, nxt_or;
  logic        cb_cur, src_buf, clip_zero, pside;
  logic [4:0]  cb_n, cb_m, ci, src_n, ei;
  logic [2:0]  cplane;
  logic [4:0]  pq_n, pq_np;
  logic        pbuf;
  logic [5:0]  pq_first;
  logic        pp_busy;
  logic [7:0]  su_t, su_u, su_rt, su_ru;   // SETUP_LOAD: triangles queued / to set up (textured, not)
  logic [4:0]  su_w;                       // SETUP_PORT: input load cycles left
  logic        su_k;                       // SETUP_PORT: the kernel waiting is SETUP4
  // clip pool: {buf, index} -> {outcodes 7, w z y x} | {shade' shade v u}
  // Vertices stay where they are written (assembly: entries 0..n-1, a clip's
  // new vertex: the next free entry); each plane's vertex list is a list of
  // entries (two lists, current and next), so an inside vertex costs an index
  // write, not a 263-bit copy
  logic        cba_we, cbb_we;
  logic [4:0]  cb_wa, cb_ra;
  // {list, index} -> entry, in an MLAB (it was 160 flip-flops and a 32-way select): one write and one read
  // a cycle, each at the address the polygon machine's state names, the read unregistered as before
  logic        cl_we;
  logic [4:0]  cl_wa, cl_wd, cl_ra, cl_q;
  logic [4:0]  c_ent;            // next free entry
  // the entries the pool is read at, kept in registers so no list read sits
  // in front of the pool's: vertex ci, vertex ci - 1, the list's last vertex,
  // the next list's last so far, and vertex ei of the queued list
  logic [4:0]  e_cur, e_prev, e_last, e_nlast, e_qe;
  logic [134:0] cba_wd, cba_q;
  logic [127:0] cbb_wd, cbb_q;
  r3d_ram #(.W(135), .A(5)) u_cba (.clk, .we(cba_we), .wa(cb_wa), .wd(cba_wd), .ra(cb_ra), .rq(cba_q));
  r3d_ram #(.W(128), .A(5)) u_cbb (.clk, .we(cbb_we), .wa(cb_wa), .wd(cbb_wd), .ra(cb_ra), .rq(cbb_q));

  // the clip lists' port: assembly writes list 0 entry a_wi (P_C); a clip appends to the next list (P_CLN
  // an inside vertex, P_XB a new one); the reads are the current list's first and next entries (P_CL0,
  // P_CLN) and the queued list's (P_Q0, P_Q1). P_CLN reads list cb_cur and writes !cb_cur.
  always_comb begin
    cl_we = 1'b0; cl_wa = {!cb_cur, cb_m[3:0]}; cl_wd = e_cur;
    case (pst)
      P_C:   begin cl_we = 1'b1; cl_wa = {2'b00, a_wi}; cl_wd = {2'b00, a_wi}; end
      P_CLN: cl_we = cba_q[8'd128 + {5'd0, cplane}] && cb_m < 5'd16;
      P_XB:  begin cl_we = cb_m < 5'd16 && c_ent != 5'd31; cl_wd = c_ent; end
      default: ;
    endcase
    case (pst)
      P_CLN:   cl_ra = {cb_cur, 4'(ci + 5'd1)};
      P_Q0:    cl_ra = {src_buf, 4'd0};
      P_Q1:    cl_ra = {src_buf, 4'(ei + 5'd1)};
      default: cl_ra = {cb_cur, 4'd0};
    endcase
  end
  r3d_ram #(.W(5), .A(5)) u_clst (.clk, .we(cl_we), .wa(cl_wa), .wd(cl_wd), .ra(cl_ra), .rq(cl_q));

  // the polygon machine's record of the polygon it is on
  logic [5:0]  ref_i;             // the vertex being read, registered one step ahead
  function automatic logic [5:0] ref_of(input logic [30:0] r, input logic [2:0] k);
    ref_of = 6'(r >> (6 * k));
  endfunction
  assign ra_ra = cp;

  // ===========================================================================
  // SETUP_OUT: the set-up port (study R60, R65; m2-notes section 15). It takes
  // the projected batches in the emitter's place: per polygon record its fan
  // triangles (v0, v[i-1], v[i]) out of the projection output; per triangle the
  // three y words, MAME's sort (rtl_tri::setup), then the sorted vertices' x, y,
  // 1/w, (u/w, v/w,) shade a word a cycle into its lane of the set-up staging (a
  // triangle a lane, two staging halves); a triangle with a non-finite x, y or
  // 1/w is dropped. Four triangles (or the batch's last) make a group: one
  // SETUP4 (any textured) or SETUP2 kernel on the engine, its results into the
  // set-up output half of the same number. The emitter sends each triangle's
  // record from there, in order, through the triangle queue.
  // ===========================================================================
  typedef enum logic [2:0] {S_IDLE, S_P0, S_P1, S_T0, S_Y, S_G, S_TE} sst_t;
  sst_t s_st /* verilator public_flat_rd */;
  typedef enum logic [1:0] {K_IDLE, K_OWN, K_RUN, K_REL} kst_t;
  kst_t kst;
  logic        sbuf, s_pop;
  logic [4:0]  snp, spe;
  logic [5:0]  s_poly;
  logic [3:0]  s_base;
  logic [4:0]  s_cnt, s_ti;
  logic        s_gt, s_lt, s_tex, s_dbl, s_fix, s_inv, s_two, s_cf, s_pf;
  logic [2:0]  s_yc;
  logic [31:0] ya_r, yb_r, yc_r;
  logic        s_c1;
  logic [3:0]  sq0, sq1, sq2;            // the sorted vertices' places in the projection output half
  logic [4:0]  s_k;                      // staging word (3 j + v)
  logic [1:0]  s_v;                      // sorted vertex
  logic [2:0]  s_j;                      // x, y, 1/w, (u/w, v/w,) shade
  logic        s_dv, s_dchk, s_nonfin;   // a gathered word arriving (read last cycle)
  logic [4:0]  s_dk;
  logic [1:0]  s_dql;                    // the lane of the word read last cycle
  logic [1:0]  sl;                       // triangles in the group being gathered
  logic        sg, s_gtex;               // its staging half; any of it textured
  logic [1:0]  stg_busy, g_tex;          // a staging half closed, waiting for its kernel
  logic [2:0]  g_n [2];
  logic        skb;                       // the next half to set up
  logic        so_set, so_setb, so_clr, so_clrb;
  logic [2:0]  so_setn;
  logic [1:0]  so_full;                  // a set-up output half with results the emitter has not sent
  logic [2:0]  so_n [2];
  // triangle queue: {textured, cull flag, inverted copy, polygon}
  logic        tq_we;
  logic [4:0]  tq_wp, tq_rp, tq_cnt;
  logic [8:0]  tq_wd, tq_q;
  logic [31:0] po_d;
  logic        s_blk;
  logic [3:0]  s_qa, s_qb, s_qc, s_qy, s_qg;
  logic [2:0]  s_comp;
  generate if (SETUP_OUT) begin : g_tq
    // the write address through a LUT of its own (keep), as u_lvl's (R104 fix 21): straight from tq_wp it
    // missed hold by 0.034 / 0.064 ns at the slow corners in a 110 MHz release fit (lw15a_s2)
    (* keep = 1 *) wire [3:0] tq_wa = tq_wp[3:0];
    r3d_ram #(.W(9), .A(4)) u_tq (.clk, .we(tq_we), .wa(tq_wa), .wd(tq_wd), .ra(tq_rp[3:0]), .rq(tq_q));
  end else begin : g_notq
    assign tq_q = 9'd0;
  end endgenerate
  assign tq_cnt = tq_wp - tq_rp;
  // the port's read of the projection output, the cycle after its address: the
  // block RAM registers it; the MLAB form (OUT_M10K = 0) reads unregistered, so
  // the word is registered here
  generate if (OUT_M10K) begin : g_pod
    assign po_d = po_q[s_dql];
  end else begin : g_podr
    logic [31:0] po_dr;
    always_ff @(posedge clk) po_dr <= po_q[(s_st == S_Y) ? s_qy[1:0] : s_qg[1:0]];
    assign po_d = po_dr;
  end endgenerate
  // the port writes the projection queue's memories only when the polygon
  // machine cannot write them the cycle after (it writes from P_Q1, entered
  // only from P_Q0)
  assign s_blk = (pst == P_Q1) || (pst == P_Q0 && !po_busy[pbuf]);
  always_comb begin
    s_qa = s_base; s_qb = s_base + 4'(s_ti - 5'd1); s_qc = s_base + 4'(s_ti);
    s_qy = (s_yc == 3'd0) ? s_qa : (s_yc == 3'd1) ? s_qb : s_qc;
    s_qg = (s_v == 2'd0) ? sq0 : (s_v == 2'd1) ? sq1 : sq2;
    case (s_j)
      3'd0, 3'd1, 3'd2: s_comp = s_j;
      3'd3: s_comp = s_tex ? 3'd4 : (s_inv && !s_fix) ? 3'd7 : 3'd6;
      3'd4: s_comp = 3'd5;
      default: s_comp = (s_inv && !s_fix) ? 3'd7 : 3'd6;      // shade, or the inverted copy's shade'
    endcase
  end
  // a < b on FP32 values that are not NaN (+0 and -0 equal)
  function automatic logic flt(input logic [31:0] a, input logic [31:0] b);
    if (a[30:0] == 31'd0 && b[30:0] == 31'd0) flt = 1'b0;
    else if (a[31] != b[31]) flt = a[31];
    else if (!a[31]) flt = a[30:0] < b[30:0];
    else flt = a[30:0] > b[30:0];
  endfunction
  // the sort compares registered words only (a compare straight off the
  // projection output's block RAM missed 10 ns at both slow corners)
  logic s_cba, s_cca, s_ccb;
  assign s_cba = flt(yb_r, ya_r);
  assign s_cca = flt(yc_r, ya_r);
  assign s_ccb = flt(yc_r, yb_r);


  // the oldest polygon still needed: the decoder may not overwrite it
  logic [5:0]  e_poly;
  logic        e_active;
  logic [5:0]  ring_tail, ch_wp, ch_start;
  logic        ring_full, ring_hi;
  always_comb begin
    if (e_active) ring_tail = e_poly;
    else if (SETUP_OUT && (tq_cnt != 5'd0 || tq_we)) ring_tail = tq_we && tq_cnt == 5'd0 ? tq_wd[5:0] : tq_q[5:0];
    else if (eb_cnt != 2'd0) ring_tail = eb_first[eb_rp];
    else if (pq_np != 5'd0) ring_tail = pq_first;
    else if (pp_busy) ring_tail = cp;
    else if (cq_cnt != 3'd0) ring_tail = cq_start[cq_rp];
    else if (dq_cnt != 2'd0) ring_tail = dq_start[dq_rp];
    else ring_tail = ch_start;
  end
  // the rings hold 32 polygons (one MLAB deep; the indices count to 64, the
  // memories take the low five bits): 64 cost 133 ALM more for 0.3% of a frame
  // each source's distance tested side by side and the flags chosen by the tail's priority (study R109):
  // the eight-way select, the subtract and the compares were in series into the polygon machine's state
  // and the decoder's D_CHK enables. (Registering the flags instead is not safe: the tail is not
  // monotone, it falls back from the emitter's polygon to its batch's first when the emitter idles.)
  function automatic logic [1:0] rflags(input logic [5:0] wp, input logic [5:0] t);   // {>= 24, >= 16}
    logic [5:0] d;
    d = 6'(wp - t);
    rflags = {d >= 6'd24, d >= 6'd16};
  endfunction
  logic [1:0] rf;
  always_comb begin
    if (e_active) rf = rflags(ch_wp, e_poly);
    else if (SETUP_OUT && (tq_cnt != 5'd0 || tq_we)) rf = (tq_we && tq_cnt == 5'd0) ? rflags(ch_wp, tq_wd[5:0]) : rflags(ch_wp, tq_q[5:0]);
    else if (eb_cnt != 2'd0) rf = rflags(ch_wp, eb_first[eb_rp]);
    else if (pq_np != 5'd0) rf = rflags(ch_wp, pq_first);
    else if (pp_busy) rf = rflags(ch_wp, cp);
    else if (cq_cnt != 3'd0) rf = rflags(ch_wp, cq_start[cq_rp]);
    else if (dq_cnt != 2'd0) rf = rflags(ch_wp, dq_start[dq_rp]);
    else rf = rflags(ch_wp, ch_start);
  end
  assign ring_full = rf[1];
  assign ring_hi   = rf[0];
`ifdef VERILATOR
  always_ff @(posedge clk)
    if (!rst && rf != {6'(ch_wp - ring_tail) >= 6'd24, 6'(ch_wp - ring_tail) >= 6'd16}) $fatal(1, "geo: ring flags side by side");
`endif

  // outcodes of ax..aw (assembly and new clip vertices share one set)
  logic [6:0] a_oc;
  assign a_oc = outc(ax, ay, az, aw);

  // clip: the axis value of a stored vertex for plane cplane, sign applied
  function automatic logic [31:0] paxis(input logic [2:0] p, input logic [127:0] v);
    logic [31:0] a;
    case (p)
      3'd1, 3'd2: a = v[31:0];
      3'd3, 3'd4: a = v[63:32];
      default: a = v[95:64];
    endcase
    paxis = p[0] ? {~a[31], a[30:0]} : a;
  endfunction

  // verdicts {d > 0, d < 0} by ring index; fv_wp the next polygon to get one
  logic [5:0]  fv_wp;
  logic [1:0]  fv_q;
  logic        fv_rdy;
  // the write address through a kept LUT, as u_fb's and u_tq's (+0.072 ns hold at fast -40C in lw15a_s1)
  (* keep = 1 *) wire [4:0] fv_wa = fv_wp[4:0];
  r3d_ram #(.W(2), .A(5)) u_fv (.clk, .we(f_vv), .wa(fv_wa), .wd({f_gt, f_lt}), .ra(cp[4:0]), .rq(fv_q));
  always_ff @(posedge clk) begin
    if (rst || f_init) fv_wp <= '0;
    else if (f_vv) fv_wp <= fv_wp + 6'd1;
  end
  logic [5:0]  fv_dist;
  assign fv_dist = fv_wp - cp;
  assign fv_rdy = !SM_CULL || (fv_dist != 6'd0 && !fv_dist[5]);

  // LOD_WORD: B by ring index (fb_wp the next polygon to get one), read by the emitter
  logic [5:0]  fb_wp;
  logic [31:0] fb_q;
  logic        fb_rdy;
  generate if (LOD_WORD && SM_CULL) begin : g_fb
    logic [5:0] fb_dist;
    // the write address through a LUT of its own (keep), as u_lvl's (R104 fix 21): straight from fb_wp it
    // missed hold by 0.010 / 0.039 ns at the slow corners in a 110 MHz release fit (lw15a_s1)
    (* keep = 1 *) wire [4:0] fb_wa = fb_wp[4:0];
    r3d_ram #(.W(32), .A(5)) u_fb (.clk, .we(f_bv), .wa(fb_wa), .wd(f_bw), .ra(e_poly[4:0]), .rq(fb_q));
    always_ff @(posedge clk) begin
      if (rst || f_init) fb_wp <= '0;
      else if (f_bv) fb_wp <= fb_wp + 6'd1;
    end
    assign fb_dist = fb_wp - e_poly;
    assign fb_rdy = fb_dist != 6'd0 && !fb_dist[5];
  end else begin : g_nofb
    assign fb_wp = '0; assign fb_q = 32'd0; assign fb_rdy = 1'b1;
  end endgenerate

  // ===========================================================================
  // main machine and polygon machine (one block: they share the engine port)
  // ===========================================================================
  always_ff @(posedge clk) begin
    if (rst) begin
      mst <= M_IDLE; pst <= P_IDLE;
      err <= '0;
      w_start <= 1'b0; vk_start <= 1'b0; cr_in <= 1'b0; sq_in <= 1'b0; g_req <= 1'b0;
      xw_we <= '0; xb_we <= 1'b0; vw_we <= 1'b0;
      m_req <= 1'b0; p_req <= 1'b0; m_drain <= 1'b0;
      d_req <= 1'b0; f_init <= 1'b0;
      cq_push <= 1'b0; cq_pop <= 1'b0; dq_pop <= 1'b0; eb_push <= 1'b0;
      pq_we <= '0; pm_we <= 1'b0; cba_we <= 1'b0; cbb_we <= 1'b0;
      pq_n <= '0; pq_np <= '0; pbuf <= 1'b0; pp_busy <= 1'b0; su_t <= '0; su_u <= '0;
      cu_v <= 1'b0; f_tclr <= 1'b0; f_tset <= 1'b0; tbuf_m <= 1'b0; vr_go <= 1'b0; f_cwe <= 1'b0;
      pq_wea <= '0; pq_web <= '0;
      s_st <= S_IDLE; kst <= K_IDLE; s_req <= 1'b0; s_pop <= 1'b0; s_dv <= 1'b0; sl <= '0; sg <= 1'b0; s_gtex <= 1'b0;
      skb <= 1'b0; stg_busy <= '0; so_set <= 1'b0; tq_we <= 1'b0; tq_wp <= '0;
    end else begin
      w_start <= 1'b0; vk_start <= 1'b0; cr_in <= 1'b0; sq_in <= 1'b0; cu_v <= 1'b0; f_init <= 1'b0;
      f_tclr <= 1'b0; f_tset <= 1'b0; f_cwe <= 1'b0;
      xw_we <= '0; xb_we <= 1'b0; vw_we <= 1'b0; lv_we <= 1'b0;
      cq_push <= 1'b0; cq_pop <= 1'b0; dq_pop <= 1'b0; eb_push <= 1'b0;
      pq_we <= '0; pm_we <= 1'b0; cba_we <= 1'b0; cbb_we <= 1'b0;
      pq_wea <= '0; pq_web <= '0; s_dv <= 1'b0; s_pop <= 1'b0; so_set <= 1'b0; tq_we <= 1'b0;
      if (tq_we) tq_wp <= tq_wp + 5'd1;
      if (w_overflow) err[0] <= 1'b1;

      // =======================================================================
      // main
      // =======================================================================
      case (mst)
        M_IDLE: if (start) begin mst <= M_FRI0; m_req <= 1'b1; end

        // stack level 0 = identity, in slot 0; the micro-code's constants
        // into broadcast bank 1 (R_BC_K + 0..7); SMFR's constant registers
        // and the projection rows' constant lanes (Supermodel's frustum)
        M_FRI0: if (own_m) begin
          sp <= '0; top_slot <= '0; bc_top_ok <= 1'b0; f_init <= 1'b1; cnt <= '0; vk_cls <= 2'd0; tbuf_m <= 1'b0;
          lv_we <= 1'b1; lv_wa <= '0; lv_wd <= '0;
          mst <= M_FRI1;
        end
        M_FRI1: begin
          xw_we <= 4'hf;
          for (int l = 0; l < 4; l++) xw_data[l] <= 32'd0;
          case (cnt)
            5'd0, 5'd1, 5'd2, 5'd3: begin
              xw_addr <= 8'd128 + 8'(cnt[1:0]);
              for (int l = 0; l < 4; l++) xw_data[l] <= (cnt[1:0] == 2'(l)) ? F_ONE : 32'd0;
            end
            5'd4: begin xw_addr <= 8'(R_SONE); for (int l = 0; l < 4; l++) xw_data[l] <= F_ONE; end
            5'd5: begin xw_addr <= 8'(R_SSGN); xw_data[0] <= F_ONE; xw_data[1] <= F_ONE; xw_data[2] <= F_M1; xw_data[3] <= F_M1; end
            5'd6: begin xw_addr <= 8'(R_SF0); xw_data[1] <= F_ONE; xw_data[2] <= F_ONE; end
            5'd7: xw_addr <= 8'(R_PR) + 8'd0;
            5'd8: xw_addr <= 8'(R_PR) + 8'd1;
            5'd9: begin xw_addr <= 8'(R_PR) + 8'd2; xw_data[3] <= F_M1; end
            default: begin xw_addr <= 8'(R_PR) + 8'd3; xw_data[2] <= 32'h3a83_126f; end   // 1e-3f
          endcase
          if (!FRUSTUM_SM && cnt >= 5'd4) xw_we <= 4'h0;
          xb_we <= cnt < 5'd8; xb_bank <= 2'd1; xb_addr <= 5'(R_BC_K) + 5'(cnt[2:0]);
          case (cnt[2:0])
            3'd0: xb_data <= 32'h0000_0000;   // +0
            3'd1: xb_data <= F_ONE;           // 1.0
            3'd2: xb_data <= 32'h3f00_0000;   // 0.5
            3'd3: xb_data <= F_255;           // 255.0
            3'd4: xb_data <= 32'h47c3_4f80;   // 99999.0 (MAME's far - near)
            3'd5: xb_data <= F_M1;            // -1.0
            3'd6: xb_data <= F_TWO;           // 2.0
            default: xb_data <= F_WPLANE;     // 1e-6f, MAME's W_PLANE
          endcase
          cnt <= cnt + 5'd1;
          if (cnt == 5'd10) begin cnt <= '0; m_req <= 1'b0; w_start <= 1'b1; mst <= M_PJW; end
        end

        // ---- dispatch a walker event --------------------------------------
        M_EV: if (ev_valid) begin
          tr_mode <= (ev_type == 3'd3);
          case (ev_type)
            3'd1: begin vaddr <= ev_a; mbase <= ev_b; pass <= ev_c[1:0]; mst <= M_VP0; end
            3'd2: mst <= M_PUSH;
            // ty, tz held in two registers idle here: the walker may load its
            // next event once this one is taken
            3'd3: begin vw_we <= 1'b1; vw_wa <= 5'd16; vw_wd <= ev_a; vq_r <= ev_b; cr_addr32 <= ev_c; cnt <= 5'd1; mst <= M_TR0; end
            3'd4: begin cr_addr32 <= ev_a; cr_len <= 3'd7; cr_dst <= 5'd16; cret <= M_MAT0; mst <= M_CR; end
            3'd5: begin ms_addr <= ev_a[23:0]; m_ctab <= ev_b[19:0]; mst <= M_MODELW; end
            3'd6: mst <= M_POP;
            3'd0: begin cu_r <= ev_a[15:0]; m_req <= 1'b1; mst <= M_CU0; end
            default: begin m_drain <= 1'b1; mst <= M_END; end
          endcase
        end

        // ---- helper: culling-RAM read of cr_len+1 words into vpw[cr_dst..]
        //      (zeros where MAME's pointer is invalid or past the region end)
        M_CR: begin
          cr_beat <= 3'd0;
          if ((cr_addr32[31:18] == 14'h0020) || (cr_addr32[31:20] == 12'h000)) begin
            g_req <= 1'b1; g_addr <= cr_addr32[23:0]; g_len <= cr_len;
          end
          mst <= M_CRW;
        end
        M_CRW: begin
          logic hi, lo, past;
          hi = (cr_addr32[31:18] == 14'h0020);
          lo = (cr_addr32[31:20] == 12'h000);
          // the regions are 2^18 and 2^20 words: past the end only from the last eight
          past = (hi ? &cr_addr32[17:3] : &cr_addr32[19:3]) && ({1'b0, cr_addr32[2:0]} + {1'b0, cr_beat} >= 4'd8);
          if (g_ack) g_req <= 1'b0;
          if (!(hi || lo) || g_rvalid) begin
            vw_we <= 1'b1; vw_wa <= cr_dst + 5'(cr_beat);
            vw_wd <= ((hi || lo) && !past) ? cull_rdata : 32'd0;
            cr_beat <= cr_beat + 3'd1;
            if (cr_beat == cr_len) mst <= cret;
          end
        end

        // ---- helper: run a kernel (owner) ----------------------------------
        M_K: begin vk_start <= 1'b1; mst <= M_KW; end
        M_KW: if (!vk_busy && !vk_start) mst <= kret;
        // ---- helper: wait for the engine
        M_OWN: if (own_m) mst <= mret;
        // ---- helper: release it, then the next event
        M_MREL: begin m_req <= 1'b0; vk_cls <= 2'd0; mst <= M_PJW; end
        M_PJW: if (!own_m) mst <= M_EV;

        // ===================================================================
        // viewport: MAME draw_viewport's parameters, then the projection
        // ===================================================================
        M_VP0: begin cr_addr32 <= ra_sum;         cr_len <= 3'd3; cr_dst <= 5'd0;  cret <= M_VP1; mst <= M_CR; end
        M_VP1: begin cr_addr32 <= ra_sum;         cr_len <= 3'd7; cr_dst <= 5'd4;  cret <= M_VP2; mst <= M_CR; end
        M_VP2: begin cr_addr32 <= ra_sum;         cr_len <= 3'd0; cr_dst <= 5'd12; cret <= M_VP3; mst <= M_CR; end
        M_VP3: begin cr_addr32 <= ra_sum;         cr_len <= 3'd0; cr_dst <= 5'd13; cret <= M_VP4; mst <= M_CR; end
        M_VP4: begin cr_addr32 <= ra_sum;         cr_len <= 3'd0; cr_dst <= 5'd14; cret <= M_VP5; mst <= M_CR; end
        M_VP5: begin cr_addr32 <= ra_sum;         cr_len <= 3'd7; cr_dst <= 5'd16; cret <= M_VP6; mst <= M_CR; end
        M_VP6: begin cr_addr32 <= ra_sum;         cr_len <= 3'd3; cr_dst <= 5'd24; cret <= FRUSTUM_SM ? M_VP7 : LOD_WORD ? M_VP8 : M_VPD; mst <= M_CR; m_drain <= 1'b1; end
        // Supermodel's frustum: words 8-11 (cv cw io jo) into vpw 28..31
        M_VP7: begin cr_addr32 <= ra_sum;         cr_len <= 3'd3; cr_dst <= 5'd28; cret <= LOD_WORD ? M_VP8 : M_VPD; mst <= M_CR; end
        // LOD_WORD: word 3 (cota) into vpw 15, for the facing test's C at M_VPW
        M_VP8: if (LOD_WORD) begin cr_addr32 <= ra_sum; cr_len <= 3'd0; cr_dst <= 5'd15; cret <= M_VPD; mst <= M_CR; end
        // the projection constants change: wait until every queued polygon of
        // the last viewport is out
        M_VPD: if (p_quiet && e_quiet && !out_valid && d_quiet) begin
          m_drain <= 1'b0; m_req <= 1'b1; cnt <= '0; mret <= M_VPW; mst <= M_OWN;
        end
        // broadcast scalars (bank 0): the word is read at cnt, written at cnt + 1
        M_VPW: begin
          vq_r <= vw_q;
          if (cnt != 5'd0) begin
            xb_we <= 1'b1; xb_bank <= 2'd0;
            case (cnt)
              5'd1: begin xb_addr <= 5'(R_BC_L0); xb_data <= vq_r; end                      // node[5]
              5'd2: begin xb_addr <= 5'(R_BC_L1); xb_data <= vq_r; end                      // node[6]
              // node[4]: Supermodel's light z is -node[4] in MAME's view space
              5'd3: begin xb_addr <= 5'(R_BC_L2); xb_data <= SM_LIGHT ? negf(vq_r) : vq_r; end
              5'd4: begin xb_addr <= 5'(R_BC_IP); xb_data <= SM_LIGHT ? dclamp(vq_r) : vq_r; end   // node[7]
              // the ambient byte: MAME's / 256 here; Supermodel's * (1 / 255) in
              // two steps, written at the end
              // cnt 5..9 through the one int16 converter (cv_q); the ambient's
              // * (1 / 255) through the decoder's k255 (tc_k, idle here)
              5'd5: begin xb_addr <= 5'(R_BC_AMB); xb_data <= cv_q; end
              5'd6: begin xb_addr <= 5'(R_BC_VW); xb_data <= cv_q; end
              5'd7: begin xb_addr <= 5'(R_BC_VX); xb_data <= cv_q; end
              5'd8: begin xb_addr <= 5'(R_BC_VH); xb_data <= cv_q; end
              5'd9: begin xb_addr <= 5'(R_BC_VY); xb_data <= cv_q; end
              // the depth map: Supermodel z/w * 0.5 + 0.5 (times 1, plus -0), MAME's * 99999 + 1
              5'd10: begin xb_addr <= 5'(R_BC_ZS); xb_data <= FRUSTUM_SM ? F_ONE : 32'h47c3_4f80; end
              5'd11: begin xb_addr <= 5'(R_BC_ZO); xb_data <= FRUSTUM_SM ? 32'h8000_0000 : F_ONE; end
              5'd12: begin xb_addr <= 5'(R_BC_L2N); xb_data <= SM_LIGHT ? vq_r : negf(vq_r); end   // -L2
              default: begin xb_we <= SM_LIGHT; xb_addr <= 5'(R_BC_AMB); xb_data <= tc_kq; end
            endcase
          end
          cnt <= cnt + 5'd1;
          if (cnt == 5'd13) begin cnt <= '0; mst <= M_VPC; end
          // LOD_WORD: -cota (word 3, read at cnt 12) into the facing test; every
          // polygon of the last viewport has had its Q product (M_VPD)
          if (LOD_WORD && cnt == 5'd13) begin f_cwe <= SM_CULL; f_cd <= negf(vq_r); end
        end
        // coord = diag(m0[0][1], m0[1][2], -m0[2][0], 1): lane j, row k
        M_VPC: begin
          xw_we <= 4'hf; xw_addr <= 8'(R_CO) + 8'(cnt[1:0]);
          for (int l = 0; l < 4; l++) xw_data[l] <= 32'd0;
          case (cnt[1:0])
            2'd0: xw_data[0] <= vw_q;
            2'd1: xw_data[1] <= vw_q;
            2'd2: xw_data[2] <= negf(vw_q);
            default: xw_data[3] <= F_ONE;
          endcase
          cnt <= cnt + 5'd1;
          lane <= '0;
          if (cnt == 5'd3) begin cnt <= '0; mst <= CULL ? M_VPK : FRUSTUM_SM ? M_SM0 : (SETUP_CR ? M_CRS0 : M_AL0); end
        end
        // culling terms, a lane a cycle: the full coordinate matrix (lane l,
        // column l, rows CF + 0..3), then the plane terms PA (lane p: -w(2p)),
        // PB (lanes 0, 2: -w1, -w5) and PC (lanes 1, 3: -w3, -w7)
        M_VPK: if (CULL) begin
          for (int l = 0; l < 4; l++) xw_data[l] <= 32'd0;
          if (cnt < 5'd16) begin
            xw_we <= 4'(1 << cnt[1:0]); xw_addr <= 8'(R_CF) + 8'(cnt[3:2]);
            if (cnt[1:0] != 2'd3) for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
          end else begin
            logic [3:0] k;
            k = 4'(cnt - 5'd16);
            xw_we <= 4'(1 << k[1:0]);
            xw_addr <= (k[3:2] == 2'd0) ? 8'(R_PA) : (k[3:2] == 2'd1) ? 8'(R_PB) : 8'(R_PC);
            if (k[3:2] == 2'd0 || (k[3:2] == 2'd1 && !k[0]) || (k[3:2] == 2'd2 && k[0]))
              for (int l = 0; l < 4; l++) xw_data[l] <= negf(vw_q);
          end
          cnt <= cnt + 5'd1;
          if (cnt == 5'd27) begin cnt <= '0; mst <= FRUSTUM_SM ? M_SM0 : SETUP_CR ? M_CRS0 : M_AL0; end
        end

        // ---- correctly rounded set-up: ht = tan(RN(atan2(y, x)) / 2), a lane at a time
        M_CRS0: if (SETUP_CR) begin cr_y <= vw_q; mst <= M_CRS1; end
        M_CRS1: if (SETUP_CR) begin cr_x <= cnt[0] ? negf(vw_q) : vw_q; cr_in <= 1'b1; mst <= M_CRS2; end
        M_CRS2: if (SETUP_CR && cr_ov) begin
          vw_we <= 1'b1; vw_wa <= 5'(28 + cnt[1:0]); vw_wd <= cr_q;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd3) begin cnt <= '0; mst <= M_PJ0; end
          else mst <= M_CRS0;
        end

        // ---- algebraic set-up, lane l: ht = y / (sqrt(x*x + y*y) + x)
        M_AL0: if (!SETUP_CR) begin                                     // GA = GB = x, a lane a cycle
          xw_we <= 4'(1 << lane); xw_addr <= cnt[1] ? 8'(R_GB) : 8'(R_GA);
          for (int l = 0; l < 4; l++) xw_data[l] <= lane[0] ? negf(vw_q) : vw_q;
          lane <= lane + 2'd1;
          if (lane == 2'd3) begin
            if (cnt[1]) begin cnt <= '0; vk_entry <= K_MUL; kret <= M_AL1; mst <= M_K; end
            else cnt[1] <= 1'b1;
          end
        end
        M_AL1: if (!SETUP_CR) begin                                     // xx into vpw 16 + l; GA = GB = y; MUL
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else if (cnt < 5'd6) begin
            vw_we <= 1'b1; vw_wa <= 5'd14 + cnt; vw_wd <= x_rdata[2'(cnt - 5'd2)];
            cnt <= cnt + 5'd1; lane <= '0;
          end else begin
            xw_we <= 4'(1 << lane); xw_addr <= cnt[3] ? 8'(R_GB) : 8'(R_GA);
            for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
            lane <= lane + 2'd1;
            if (lane == 2'd3) begin
              if (cnt[3]) begin cnt <= '0; vk_entry <= K_MUL; kret <= M_AL2; mst <= M_K; end
              else cnt[3] <= 1'b1;
            end
          end
        end
        M_AL2: if (!SETUP_CR) begin                                     // GC = yy (all lanes), GA = xx; ADD
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else if (cnt == 5'd2) begin
            xw_we <= 4'hf; xw_addr <= 8'(R_GC); for (int l = 0; l < 4; l++) xw_data[l] <= x_rdata[l];
            cnt <= 5'd3; lane <= '0;
          end else begin
            xw_we <= 4'(1 << lane); xw_addr <= 8'(R_GA); for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
            lane <= lane + 2'd1;
            if (lane == 2'd3) begin cnt <= '0; vk_entry <= K_ADD; kret <= M_AL3; mst <= M_K; end
          end
        end
        M_AL3: if (!SETUP_CR) begin                                     // square root of each lane into vpw 20 + l
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; lane <= '0; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else if (cnt == 5'd2) begin if (!sq_busy && !sq_in) begin sq_in <= 1'b1; sq_x <= x_rdata[lane]; cnt <= 5'd3; end end
          else if (cnt == 5'd3) cnt <= 5'd4;
          else if (sq_ov) begin
            vw_we <= 1'b1; vw_wa <= 5'(20 + lane); vw_wd <= sq_q;
            lane <= lane + 2'd1;
            cnt <= 5'd2;
            if (lane == 2'd3) begin cnt <= 5'd1; lane <= '0; mst <= M_AL6; end
          end
        end
        M_AL6: if (!SETUP_CR) begin                                     // GA = root, GC = x; ADD
          xw_we <= 4'(1 << lane); xw_addr <= cnt[0] ? 8'(R_GA) : 8'(R_GC);
          for (int l = 0; l < 4; l++) xw_data[l] <= (!cnt[0] && lane[0]) ? negf(vw_q) : vw_q;
          lane <= lane + 2'd1;
          if (lane == 2'd3) begin
            if (!cnt[0]) begin cnt <= '0; vk_entry <= K_ADD; kret <= M_AL4; mst <= M_K; end
            else cnt[0] <= 1'b0;
          end
        end
        M_AL4: if (!SETUP_CR) begin                                     // GA = den (GO, all lanes), GB = y; DIV
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else begin
            xw_we <= 4'hf; xw_addr <= 8'(R_GA); for (int l = 0; l < 4; l++) xw_data[l] <= x_rdata[l];
            lane <= '0; mst <= M_AL5;
          end
        end
        M_AL5: if (!SETUP_CR) begin
          xw_we <= 4'(1 << lane); xw_addr <= 8'(R_GB); for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
          lane <= lane + 2'd1;
          if (lane == 2'd3) begin cnt <= '0; vk_entry <= K_DIV; kret <= M_AL7; mst <= M_K; end
        end
        M_AL7: if (!SETUP_CR) begin                                     // ht into vpw 28 + l
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else begin
            vw_we <= 1'b1; vw_wa <= 5'd26 + cnt; vw_wd <= x_rdata[2'(cnt - 5'd2)];
            cnt <= cnt + 5'd1;
            if (cnt == 5'd5) begin cnt <= '0; mst <= M_PJ0; end
          end
        end

        // ---- projection from l = ht0, r = -ht1, t = ht2, b = -ht3 (vpw 28..31)
        // SUB: GA {l r t r} - GC {r l b l} -> l-r, r-l, t-b into vpw 16..18
        // ADD: GA {r t r t} + GC {l b l b} -> r+l, t+b into vpw 20, 21
        // DIV: GB {2, r+l, 2, t+b} / GA {l-r, r-l, t-b, t-b} -> P00 P02 P11 P12
        M_PJ0: if (!FRUSTUM_SM) begin
          xw_we <= 4'(1 << cnt[1:0]); xw_addr <= cnt[2] ? 8'(R_GC) : 8'(R_GA);
          for (int l = 0; l < 4; l++) xw_data[l] <= vw_ra[0] ? negf(vw_q) : vw_q;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd7) begin cnt <= '0; vk_entry <= K_SUB; kret <= M_PJ1; mst <= M_K; end
        end
        M_PJ1: if (!FRUSTUM_SM) begin
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else if (cnt < 5'd5) begin
            vw_we <= 1'b1; vw_wa <= 5'd14 + cnt; vw_wd <= x_rdata[2'(cnt - 5'd2)];
            cnt <= (cnt == 5'd4) ? 5'd8 : cnt + 5'd1;
          end else begin
            xw_we <= 4'(1 << cnt[1:0]); xw_addr <= cnt[2] ? 8'(R_GC) : 8'(R_GA);
            for (int l = 0; l < 4; l++) xw_data[l] <= vw_ra[0] ? negf(vw_q) : vw_q;
            cnt <= cnt + 5'd1;
            if (cnt == 5'd15) begin cnt <= '0; vk_entry <= K_ADD; kret <= M_PJ2; mst <= M_K; end
          end
        end
        M_PJ2: if (!FRUSTUM_SM) begin
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else if (cnt == 5'd2) begin vw_we <= 1'b1; vw_wa <= 5'd20; vw_wd <= x_rdata[0]; cnt <= 5'd3; end
          else if (cnt == 5'd3) begin vw_we <= 1'b1; vw_wa <= 5'd21; vw_wd <= x_rdata[1]; cnt <= 5'd8; end
          else begin
            xw_we <= 4'(1 << cnt[1:0]); xw_addr <= 8'(R_GA);
            for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
            cnt <= cnt + 5'd1;
            if (cnt == 5'd11) begin cnt <= '0; mst <= M_PJ3; end
          end
        end
        M_PJ3: if (!FRUSTUM_SM) begin
          xw_we <= 4'(1 << cnt[1:0]); xw_addr <= 8'(R_GB);
          for (int l = 0; l < 4; l++) xw_data[l] <= cnt[0] ? vw_q : F_TWO;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd3) begin cnt <= '0; vk_entry <= K_DIV; kret <= M_PJ4; mst <= M_K; end
        end
        // the projection columns: lane0 (P00 0 0 0), lane1 (0 P11 0 0),
        // lane2 (P02 P12 P22 -1), lane3 (0 0 P23 0)
        M_PJ4: if (!FRUSTUM_SM) begin
          if (cnt == 5'd0) begin x_raddr <= 7'(R_GO); cnt <= 5'd1; end
          else if (cnt == 5'd1) cnt <= 5'd2;
          else begin
            xw_we <= 4'hf;
            for (int l = 0; l < 4; l++) xw_data[l] <= 32'd0;
            case (cnt[1:0])
              2'd2: begin xw_addr <= 8'(R_PR) + 8'd0; xw_data[0] <= x_rdata[0]; xw_data[2] <= x_rdata[1]; end
              2'd3: begin xw_addr <= 8'(R_PR) + 8'd1; xw_data[1] <= x_rdata[2]; xw_data[2] <= x_rdata[3]; end
              2'd0: begin xw_addr <= 8'(R_PR) + 8'd2; xw_data[2] <= F_P22; xw_data[3] <= F_P23; end
              default: begin xw_addr <= 8'(R_PR) + 8'd3; xw_data[2] <= F_M1; end
            endcase
            cnt <= cnt + 5'd1;
            if (cnt == 5'd5) begin cnt <= '0; mst <= M_VR0; end
          end
        end

        // ---- Supermodel's frustum (Mat4::FrustumRZ): SC {jo jo io io}, SB
        // {cv cv cw cw} a lane a cycle, then the SMFR kernel writes the
        // projection rows (their constant lanes are loaded at frame start)
        M_SM0: if (FRUSTUM_SM) begin
          xw_we <= 4'(1 << cnt[1:0]); xw_addr <= cnt[2] ? 8'(R_SB) : 8'(R_SC);
          for (int l = 0; l < 4; l++) xw_data[l] <= vw_q;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd7) begin cnt <= '0; vk_entry <= K_SMFR; kret <= M_VR0; mst <= M_K; end
        end

        // ---- the viewport record (R62): words 0x20-0x25 into vpw 0-5, then
        // the emitter sends them with the depth constant (output quiet here)
        M_VR0: begin cr_addr32 <= ra_sum; cr_len <= 3'd5; cr_dst <= 5'd0; cret <= M_VR1; mst <= M_CR; end
        M_VR1: begin
          vr_go <= 1'b1;
          if (vr_done) begin vr_go <= 1'b0; mst <= M_MREL; end
        end

        // ---- node culling (CULL): Supermodel's sphere test, verdict to the walker
        M_CU0: if (CULL && own_m) begin
          if (!bc_top_ok) begin vk_entry <= K_COPYB; vk_spa <= top_slot; kret <= M_CU1; bc_top_ok <= 1'b1; mst <= M_K; end
          else mst <= M_CU1;
        end
        M_CU1: if (CULL) begin vk_entry <= K_CULLU; kret <= M_CU2; mst <= M_K; end
        M_CU2: if (CULL) begin vk_entry <= K_CULLT; kret <= M_CU3; cnt <= '0; cu_o <= 1'b0; mst <= M_K; end
        M_CU3: if (CULL) begin
          logic [31:0] a1, rr;
          logic [5:0]  e6;
          a1 = {1'b0, cu_r, 15'd0};
          e6 = a1[30:25];
          rr = {1'b0, (e6 <= 6'd31) ? 8'({2'b00, e6} + 8'd127) : 8'({2'b00, e6} + 8'd63), a1[24:2]};
          x_raddr <= 7'(R_CT);
          cnt <= cnt + 5'd1;
          if (cnt == 5'd2) begin
            cu_v <= 1'b1;
            cu_out <= rlt(rr, x_rdata[0]) || rlt(rr, x_rdata[1]) || rlt(rr, x_rdata[2]) || rlt(rr, x_rdata[3]);
            cnt <= '0; mst <= M_MREL;
          end
        end

        // ===================================================================
        // matrix stack: in the engine's register file blocks
        // ===================================================================
        // push: the new level shares the top's slot; pop: back to the level
        // below, whose broadcast copy is still current if it shared the slot
        M_PUSH: begin
          if (sp == 5'(SDEPTH - 1)) err[1] <= 1'b1;
          else begin lv_we <= 1'b1; lv_wa <= sp + 5'd1; lv_wd <= top_slot; sp <= sp + 5'd1; end
          mst <= M_EV;
        end
        M_POP: begin
          sp <= sp - 5'd1; top_slot <= lv_q;
          if (lv_q != top_slot) bc_top_ok <= 1'b0;
          mst <= M_EV;
        end
        M_TR0: begin                                    // translation into vpw 16..18
          vw_we <= 1'b1; vw_wa <= 5'(16 + cnt); vw_wd <= (cnt == 5'd1) ? vq_r : cr_addr32;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd2) begin cnt <= '0; m_req <= 1'b1; mret <= M_MBW; mst <= M_OWN; end
        end
        M_MAT0: begin cr_addr32 <= ra_sum; cr_len <= 3'd3; cr_dst <= 5'd24; cret <= M_MAT1; mst <= M_CR; end
        M_MAT1: begin cnt <= '0; m_req <= 1'b1; mret <= M_MBW; mst <= M_OWN; end
        // M[i][k] -> bank k, entry MB + i: rows (w3 w6 w9 0) (w4 w7 w10 0) (w5 w8 w11 0) (w0 w1 w2 1)
        // for a matrix; the identity with row 3 = (tx ty tz 1) for a translation
        // the matrix goes in through the external port, a word a cycle, with
        // the engine held (R50's side channel saved 0.4% of a frame at p99
        // for its write port's ALMs)
        M_MBW: begin
          xb_we <= 1'b1; xb_bank <= cnt[1:0]; xb_addr <= 5'(R_BC_MB) + 5'(cnt[3:2]); xb_data <= mb_data;
          cnt <= cnt + 5'd1;
          if (cnt == 5'd15) begin cnt <= '0; mst <= M_MBK; end
        end
        M_MBK: begin vk_entry <= K_MM; vk_spa <= top_slot; vk_spd <= sp; kret <= M_MBD; mst <= M_K; end
        M_MBD: begin top_slot <= sp; lv_we <= 1'b1; lv_wa <= sp; lv_wd <= sp; bc_top_ok <= 1'b1; mst <= M_MREL; end

        // ===================================================================
        // model: vp = top x coord x projection, then T&L of each chunk
        // ===================================================================
        // a model: its matrix buffer alternates; the one two models back must
        // have no polygon left in the facing test before it is cleared
        M_MODELW: if (!SM_CULL || !f_mbbusy[!tbuf_m]) begin
          tbuf_m <= !tbuf_m; f_tclr <= SM_CULL;
          d_req <= !d_req; m_req <= 1'b1; mst <= M_MODEL0;
        end
        M_MODEL0: if (own_m) begin
          if (!bc_top_ok) begin vk_entry <= K_COPYB; vk_spa <= top_slot; kret <= M_MODEL1; bc_top_ok <= 1'b1; mst <= M_K; end
          else mst <= M_MODEL1;
        end
        M_MODEL1: begin vk_entry <= K_MODEL; kret <= SM_CULL ? M_FT0 : M_CHD; mst <= M_K; end
        // the model matrix (TR rows 0-3, columns 0-2) into the facing test, a
        // word a cycle through the external read port, the engine idle
        M_FT0: begin x_raddr <= 7'(R_TR); ft_row <= '0; ft_col <= '0; mst <= M_FT2; end
        M_FT2: mst <= M_FT1;                               // the read's data
        M_FT1: if (!f_thold) begin
          if (ft_col == 2'd2) begin
            ft_col <= '0;
            if (ft_row == 2'd3) begin f_tset <= 1'b1; mst <= M_CHD; end
            else begin ft_row <= ft_row + 2'd1; x_raddr <= 7'(R_TR) + {5'd0, ft_row} + 7'd1; mst <= M_FT2; end
          end else ft_col <= ft_col + 2'd1;
        end
        M_CHD: begin m_req <= 1'b0; mst <= M_CHX; end
        M_CHX: if (!own_m) mst <= M_CHW;
        // the decoder's next chunk, when the vertex store has room for it
        // (main runs at most two chunks ahead of the polygon machine)
        M_CHW: if (dq_cnt != 2'd0 && cq_cnt < 3'd3) begin
          c_start <= dq_start[dq_rp]; c_end <= dq_end[dq_rp]; c_tl <= dq_tl[dq_rp];
          c_ring <= dq_ring[dq_rp]; c_bank <= dq_bank[dq_rp]; c_last <= dq_last[dq_rp];
          if (dq_tl[dq_rp] != 5'd0) begin m_req <= 1'b1; mret <= M_CHT; mst <= M_OWN; end
          else mst <= M_CHE;                              // nothing to transform
        end
        M_CHT: begin
          vk_entry <= (c_tl > 5'd12) ? K_TL : K_TL3;
          k_tbank <= c_bank; k_ring <= c_ring; vk_cls <= 2'd1;
          kret <= M_CHE; mst <= M_K;
        end
        // the chunk to the polygon machine; release the engine
        M_CHE: begin cq_push <= 1'b1; dq_pop <= 1'b1; m_req <= 1'b0; vk_cls <= 2'd0; mst <= M_CHG; end
        M_CHG: if (!own_m) mst <= c_last ? M_EV : M_CHW;
        // the frame ends with the model stream idle too: its last prefetch, asked
        // for or in flight, drained, so busy low leaves no read open at a port
        M_END: if (d_quiet && dq_cnt == 2'd0 && p_quiet && e_quiet && !out_valid && !w_busy && f_idle && ms_ready) begin
          m_drain <= 1'b0; mst <= M_IDLE;
        end

        default: mst <= M_IDLE;
      endcase

      // =======================================================================
      // polygon machine
      // =======================================================================
      case (pst)
        P_IDLE: begin
          if (cq_cnt != 3'd0) begin
            cp <= cq_start[cq_rp]; pc_end <= cq_end[cq_rp]; pc_ring <= cq_ring[cq_rp]; pp_busy <= 1'b1;
            pst <= P_POLY;
          end else if (pq_np != 5'd0 && (m_drain || ring_hi)) begin
            pj_ret <= P_IDLE; pst <= P_PJ0;
          end
        end
        P_POLY: begin
          if (cp == pc_end) begin cq_pop <= 1'b1; pp_busy <= 1'b0; pst <= P_GAP; end
          else if (ra_q[29]) cp <= cp + 6'd1;            // texture size >= 6, or dropped: no triangles
          else if (!fv_rdy) ;                            // its facing verdict is not out yet
          else if (SM_CULL && FACE_DROP && !ra_q[30] && fv_q[1]) cp <= cp + 6'd1;   // a back face
          else begin
            as_i <= '0; a_nv <= ra_q[26:24]; acc <= 1'b1; cur_and <= 7'h7f; cur_or <= 7'h00;
            ref_i <= ref_of(ra_q, 3'd0);
            pst <= P_A;
          end
        end
        // assemble vertex as_i: x z shade, then y w; then u v and the outcodes
        // go into the pool while the next vertex's x z shade are read
        P_A, P_C: begin
          ax <= vsa_q[ref_i[1:0]]; az <= vsb_q[ref_i[1:0]]; ash <= vsc_q[ref_i[1:0]];
          if (pst == P_C) begin
            cba_we <= 1'b1; cbb_we <= 1'b1; cb_wa <= {2'b00, a_wi}; 
            cba_wd <= {a_oc, aw, az, ay, ax};
            cbb_wd <= {ra_q[28] ? F_255 : ash2, ra_q[28] ? F_255 : ash, uv_v, uv_u};
            cur_and <= cur_and & a_oc; cur_or <= cur_or | a_oc;
            if (a_oc != 7'h7f) acc <= 1'b0;
          end
          if (pst == P_C && a_wi + 3'd1 == a_nv) begin
            // the last vertex: accepted polygons go straight to the queue (it
            // reads the pool from vertex 0); the clip reads the vertex just written
            cb_n <= {2'b00, a_nv};
            if (acc && a_oc == 7'h7f) begin src_buf <= 1'b0; src_n <= {2'b00, a_nv}; pst <= P_Q0; end
            else pst <= P_D;
          end else pst <= P_B;
        end
        P_B: begin
          ay <= vsa_q[ref_i[1:0]]; aw <= vsb_q[ref_i[1:0]]; ash2 <= vsc_q[ref_i[1:0]];
          // (u * 256, or u / 8 * 256: converted in P_C from pu_qr)
          a_wi <= as_i;
          as_i <= as_i + 3'd1;
          ref_i <= ref_of(ra_q, as_i + 3'd1);
          pst <= P_C;
        end
        P_D: begin
          cb_n <= {2'b00, a_nv};
          if (acc) begin src_buf <= 1'b0; src_n <= {2'b00, a_nv}; pst <= P_Q0; end
          else begin
            cplane <= 3'd0; cb_cur <= 1'b0; clip_zero <= 1'b0; c_ent <= {2'b00, a_nv}; e_last <= {2'b00, 3'(a_nv - 3'd1)};
            pst <= P_CL0;
          end
        end

        // ---- clip: MAME's frustum_clip_all, plane by plane ------------------
        P_CL0: begin
          if (cplane == 3'd7 || clip_zero) begin
            src_buf <= cb_cur; src_n <= clip_zero ? 5'd0 : cb_n; pst <= P_Q0;
          end else if (cb_n == 5'd0) begin
            cplane <= cplane + 3'd1;
          end else if (cur_and[cplane]) begin
            cplane <= cplane + 3'd1;                         // every vertex inside: unchanged
          end else if (!cur_or[cplane]) begin
            clip_zero <= 1'b1;                               // every vertex outside: nothing left
          end else begin
            // prev = vertex n-1
            pside <= cba_q[8'd128 + {5'd0, cplane}];
            ci <= '0; cb_m <= '0; nxt_and <= 7'h7f; nxt_or <= 7'h00;
            e_cur <= cl_q; e_prev <= e_last;
            pst <= P_CL1;
          end
        end
        P_CL1: begin
          if (cba_q[8'd128 + {5'd0, cplane}] != pside) begin p_req <= 1'b1; pst <= P_X0; end
          else pst <= P_CLN;
        end
        // current vertex inside: copy it (read and write the pool in one cycle)
        P_CLN: begin
          logic cs;
          cs = cba_q[8'd128 + {5'd0, cplane}];
          if (cs) begin
            if (cb_m < 5'd16) begin
              e_nlast <= e_cur;
              nxt_and <= nxt_and & cba_q[134:128]; nxt_or <= nxt_or | cba_q[134:128];
            end else err[2] <= 1'b1;
            cb_m <= cb_m + 5'd1;
          end
          pside <= cs;
          ci <= ci + 5'd1;
          e_prev <= e_cur; e_cur <= cl_q;
          if (ci + 5'd1 == cb_n) pst <= P_CLA;
          else pst <= P_CL1;
        end
        P_CLA: begin
          cb_n <= (cb_m > 5'd16) ? 5'd16 : cb_m;
          cb_cur <= !cb_cur; e_last <= e_nlast;
          cur_and <= nxt_and; cur_or <= nxt_or;
          cplane <= cplane + 3'd1;
          pst <= P_CL0;
        end

        // ---- one crossing on the engine: prev (index ci - 1) and cur (ci)
        P_X0: if (own_p) pst <= P_X1;
        P_X1: begin                                        // prev set 0
          xw_we <= 4'hf; xw_addr <= 8'(R_LB) + 8'd0;
          for (int l = 0; l < 4; l++) xw_data[l] <= cba_q[32 * l +: 32];
          ax <= cba_q[127:96];                             // pw
          ay <= (cplane == 3'd0) ? 32'd0 : paxis(cplane, cba_q[127:0]);   // pa
          pst <= P_X2;
        end
        P_X2: begin                                        // prev set 1
          xw_we <= 4'hf; xw_addr <= 8'(R_LB) + 8'd3;
          for (int l = 0; l < 4; l++) xw_data[l] <= cbb_q[32 * l +: 32];
          pst <= P_X3;
        end
        P_X3: begin                                        // cur set 0
          xw_we <= 4'hf; xw_addr <= 8'(R_LB) + 8'd1;
          for (int l = 0; l < 4; l++) xw_data[l] <= cba_q[32 * l +: 32];
          pst <= P_X4;
        end
        P_X4: begin                                        // cur set 1
          xw_we <= 4'hf; xw_addr <= 8'(R_LB) + 8'd4;
          for (int l = 0; l < 4; l++) xw_data[l] <= cbb_q[32 * l +: 32];
          pst <= P_X5;
        end
        P_X5: begin                                        // CS: pw, cw, E
          xw_we <= 4'hf; xw_addr <= 8'(R_CS);
          xw_data[0] <= ax; xw_data[1] <= cba_q[127:96]; xw_data[2] <= (cplane == 3'd0) ? F_WPLANE : ax; xw_data[3] <= 32'd0;
          pst <= P_X6;
        end
        P_X6: begin                                        // CS + 1: pa, ca, F
          xw_we <= 4'hf; xw_addr <= 8'(R_CS) + 8'd1;
          xw_data[0] <= ay; xw_data[1] <= (cplane == 3'd0) ? 32'd0 : paxis(cplane, cba_q[127:0]);
          xw_data[2] <= (cplane == 3'd0) ? ax : ay; xw_data[3] <= 32'd0;
          vk_entry <= K_CLIP; vk_cls <= 2'd0; vk_start <= 1'b1;
          pst <= P_X7;
        end
        P_X7: if (!vk_busy && !vk_start) begin x_raddr <= 7'(R_CZ); pst <= P_X8; end
        P_X8: begin x_raddr <= 7'(R_LB) + 7'd2; pst <= P_X9; end
        P_X9: begin
          // wdiv in lane 0
          x_raddr <= 7'(R_LB) + 7'd5;
          if (x_rdata[0][30:0] == 31'd0) begin clip_zero <= 1'b1; p_req <= 1'b0; pst <= P_XC; end
          else pst <= P_XA;
        end
        P_XA: begin                                        // the new vertex, set 0
          ax <= x_rdata[0]; ay <= x_rdata[1]; az <= x_rdata[2]; aw <= x_rdata[3];
          pst <= P_XB;
        end
        P_XB: begin                                        // set 1; append
          if (cb_m < 5'd16 && c_ent != 5'd31) begin
            cba_we <= 1'b1; cbb_we <= 1'b1; cb_wa <= c_ent; e_nlast <= c_ent; c_ent <= c_ent + 5'd1;
            cba_wd <= {a_oc, aw, az, ay, ax};
            cbb_wd <= {x_rdata[3], x_rdata[2], x_rdata[1], x_rdata[0]};
            nxt_and <= nxt_and & a_oc; nxt_or <= nxt_or | a_oc;
          end else err[2] <= 1'b1;
          cb_m <= cb_m + 5'd1;
          p_req <= 1'b0;
          pst <= P_CLN;
        end
        P_XC: if (!own_p) pst <= P_CL0;

        // ---- queue the polygon's vertices for projection ---------------------
        P_Q0: begin
          if (src_n < 5'd3) begin cp <= cp + 6'd1; pst <= P_POLY; end
          else if (pq_n + src_n > 5'd16) begin
            pj_ret <= P_Q0; pst <= P_PJ0;                     // project what is queued first
          end else if (po_busy[pbuf]) begin
            // the emitter is still on this half's last batch: its polygon list
            // is read until it pops
          end else begin
            pm_we <= 1'b1; pm_wa <= {pbuf, pq_np[3:0]};
            pm_wd <= PMW'({su_q, SM_CULL ? fv_q : 2'b00, src_n, pq_n[3:0], cp});
            if (SETUP_LOAD) begin
              logic [7:0] m;
              // set-ups per triangle, one per record not facing away: both
              // copies of a double-sided polygon without the facing test
              if (ra_q[30]) m = (SM_CULL && fv_q != 2'b00) ? 8'd1 : 8'd2;
              else m = (SM_CULL && fv_q[1]) ? 8'd0 : 8'd1;
              if (su_q[0]) su_t <= su_t + (8'(src_n) - 8'd2) * m;
              else su_u <= su_u + (8'(src_n) - 8'd2) * m;
            end
            if (pq_np == 5'd0) pq_first <= cp;
            pq_np <= pq_np + 5'd1;
            ei <= '0; cnt_q <= '0; e_qe <= cl_q;
            pst <= P_Q1;
          end
        end
        // vertex ei into its lane, two components a cycle: (x u) (y v) (z shade) (w shade')
        P_Q1: begin
          logic [4:0] q;
          q = pq_n + ei;
          pq_we <= 4'(1 << q[1:0]); pq_wa <= PQA'({pbuf, q[3:2], cnt_q});
          pq_wea <= 4'(1 << q[1:0]); pq_web <= 4'(1 << q[1:0]);
          pq_wa_d <= cba_q[32 * cnt_q +: 32];
          pq_wb_d <= cbb_q[32 * cnt_q +: 32];
          cnt_q <= cnt_q + 2'd1;
          if (cnt_q == 2'd3) begin
            ei <= ei + 5'd1; e_qe <= cl_q;
            if (ei + 5'd1 == src_n) begin pq_n <= pq_n + src_n; cp <= cp + 6'd1; pst <= P_POLY; end
          end
        end

        // ---- project the queue: wait for its output half, then PROJ ----------
        P_PJ0: if (!po_busy[pbuf] && eb_cnt != 2'd2) begin p_req <= 1'b1; pst <= P_PJ1; end
        P_PJ1: if (own_p) begin
          if (pq_n <= 5'd4) vk_entry <= K_PROJ1;
          else if (pq_n <= 5'd12) vk_entry <= K_PROJ3;
          else vk_entry <= K_PROJ;
          k_pbuf <= pbuf; vk_cls <= 2'd2; vk_start <= 1'b1;
          pst <= P_PJ2;
        end
        P_PJ2: if (!vk_busy && !vk_start) begin
          p_req <= 1'b0; vk_cls <= 2'd0;
          eb_push <= 1'b1; eb_wbuf <= pbuf; eb_wnp <= pq_np; eb_wfirst <= pq_first;
          pbuf <= !pbuf; pq_n <= '0; pq_np <= '0;
          pst <= P_PJ3;
          if (SETUP_LOAD) begin
            su_rt <= su_t; su_ru <= su_u; su_t <= '0; su_u <= '0;
            if (su_t != 8'd0 || su_u != 8'd0) begin p_req <= 1'b1; pst <= P_SU0; end
          end
        end
        // SETUP_LOAD: a kernel per four triangles, textured first
        P_SU0: if (SETUP_LOAD) begin
          if (su_rt != 8'd0) begin
            vk_entry <= K_SETUP4; vk_start <= !SETUP_PORT; su_rt <= (su_rt > 8'd4) ? su_rt - 8'd4 : 8'd0;
            su_w <= 5'd18; pst <= SETUP_PORT ? P_SU2 : P_SU1;
          end else if (su_ru != 8'd0) begin
            vk_entry <= K_SETUP2; vk_start <= !SETUP_PORT; su_ru <= (su_ru > 8'd4) ? su_ru - 8'd4 : 8'd0;
            su_w <= 5'd12; pst <= SETUP_PORT ? P_SU2 : P_SU1;
          end else begin p_req <= 1'b0; pst <= P_PJ3; end
        end
        P_SU1: if (SETUP_LOAD && !vk_busy && !vk_start) pst <= P_SU0;
        P_SU2: if (SETUP_LOAD) begin                     // the inputs loading: engine idle
          su_w <= su_w - 5'd1;
          if (su_w == 5'd1) begin vk_start <= 1'b1; pst <= P_SU1; end
        end
        P_PJ3: if (!own_p) pst <= pj_ret;
        P_GAP: pst <= P_IDLE;

        default: pst <= P_IDLE;
      endcase

      // =======================================================================
      // set-up port (SETUP_OUT)
      // =======================================================================
      if (SETUP_OUT) begin
        // a gathered word into its lane of the staging half: {1, half, word}
        if (s_dv) begin
          pq_wea <= s_dk[2] ? 4'd0 : 4'(1 << sl);
          pq_web <= s_dk[2] ? 4'(1 << sl) : 4'd0;
          pq_wa <= PQA'({1'b1, sg, s_dk[4:3], s_dk[1:0]});
          pq_wa_d <= po_d; pq_wb_d <= po_d;
          if (s_dchk && po_d[30:23] == 8'hff) s_nonfin <= 1'b1;
        end
        case (s_st)
          S_IDLE: if (eb_cnt != 2'd0 && !s_pop) begin
            sbuf <= eb_buf[eb_rp]; snp <= eb_np[eb_rp]; spe <= '0;
            s_st <= S_P0;
          end
          S_P0: begin
            if (spe == snp) begin
              // the batch is staged: its last group goes to the engine as it is
              if (sl != 2'd0) begin
                stg_busy[sg] <= 1'b1; g_n[sg] <= {1'b0, sl}; g_tex[sg] <= s_gtex;
                sg <= !sg; sl <= '0; s_gtex <= 1'b0;
              end
              s_pop <= 1'b1; s_st <= S_IDLE;
            end else begin
              s_poly <= pm_q[5:0]; s_base <= pm_q[9:6]; s_cnt <= pm_q[14:10]; s_lt <= pm_q[15]; s_gt <= pm_q[16];
              s_tex <= pm_q[PMW-3]; s_dbl <= pm_q[PMW-2]; s_fix <= pm_q[PMW-1];
              s_st <= S_P1;
            end
          end
          // the records, as the emitter's E_P1: the inverted copy, then the polygon
          S_P1: begin
            logic dbl, sc, so;
            dbl = SM_DOUBLE && s_dbl;
            sc = dbl && !(FACE_DROP && s_lt);
            so = !(FACE_DROP && s_gt);
            s_inv <= sc; s_two <= sc && so;
            s_cf <= sc ? s_lt : s_gt; s_pf <= s_gt;
            s_ti <= 5'd2;
            if (sc || so) s_st <= S_T0;
            else begin spe <= spe + 5'd1; s_st <= S_P0; end
          end
          // a triangle: its staging half free, room in the triangle queue
          S_T0: if (!stg_busy[sg] && tq_cnt <= 5'd14) begin
            s_yc <= '0; s_nonfin <= 1'b0; s_st <= S_Y;
          end
          // the three y words (a, b, c issued in s_yc 0-2, each back a cycle
          // later), then MAME's sort: if (b < a) swap a b; if (c < b) { swap
          // b c; if (b < a) swap a b }
          S_Y: begin
            s_yc <= s_yc + 3'd1;
            s_dql <= s_qy[1:0];
            case (s_yc)
              3'd1: ya_r <= po_d;
              3'd2: yb_r <= po_d;
              3'd3: begin yc_r <= po_d; s_c1 <= s_cba; end
              3'd4: begin
                logic c2, c3;
                logic [3:0] lo, hi;
                lo = s_c1 ? s_qb : s_qa; hi = s_c1 ? s_qa : s_qb;
                c2 = s_c1 ? s_cca : s_ccb;                    // c < hi
                c3 = s_c1 ? s_ccb : s_cca;                    // c < lo
                if (!c2) begin sq0 <= lo; sq1 <= hi; sq2 <= s_qc; end
                else if (!c3) begin sq0 <= lo; sq1 <= s_qc; sq2 <= hi; end
                else begin sq0 <= s_qc; sq1 <= lo; sq2 <= hi; end
                s_k <= '0; s_v <= '0; s_j <= '0;
                s_st <= S_G;
              end
              default: ;
            endcase
          end
          // the staging words, k = 3 j + v, a read a cycle
          S_G: if (!s_blk) begin
            s_dv <= 1'b1; s_dk <= s_k; s_dql <= s_qg[1:0]; s_dchk <= s_j <= 3'd2;
            s_k <= s_k + 5'd1;
            if (s_v == 2'd2) begin s_v <= '0; s_j <= s_j + 3'd1; end
            else s_v <= s_v + 2'd1;
            if (s_k == (s_tex ? 5'd17 : 5'd11)) s_st <= S_TE;
          end
          // the last word written: queue the triangle (or drop it), close a full group
          S_TE: if (!s_dv) begin
            if (!s_nonfin) begin
              tq_we <= 1'b1; tq_wd <= {s_tex, SM_CULL && s_cf, s_inv, s_poly};
              if (sl == 2'd3) begin
                stg_busy[sg] <= 1'b1; g_n[sg] <= 3'd4; g_tex[sg] <= s_gtex || s_tex;
                sg <= !sg; sl <= '0; s_gtex <= 1'b0;
              end else begin
                sl <= sl + 2'd1; s_gtex <= s_gtex || s_tex;
              end
            end
            if (s_ti + 5'd1 < s_cnt) begin s_ti <= s_ti + 5'd1; s_st <= S_T0; end
            else if (s_two) begin
              s_inv <= 1'b0; s_two <= 1'b0; s_cf <= s_pf; s_ti <= 5'd2; s_st <= S_T0;
            end else begin spe <= spe + 5'd1; s_st <= S_P0; end
          end
          default: s_st <= S_IDLE;
        endcase

        // a closed group on the engine: SETUP4 or SETUP2, staging half skb in,
        // set-up output half skb out (once the emitter has sent its last group)
        case (kst)
          K_IDLE: if (stg_busy[skb] && !so_full[skb]) begin s_req <= 1'b1; kst <= K_OWN; end
          K_OWN: if (own_s) begin
            vk_entry <= g_tex[skb] ? K_SETUP4 : K_SETUP2; vk_cls <= 2'd3; k_pbuf <= skb; k_sbuf <= skb;
            vk_start <= 1'b1; kst <= K_RUN;
          end
          K_RUN: if (!vk_busy && !vk_start) begin
            s_req <= 1'b0; vk_cls <= 2'd0;
            so_set <= 1'b1; so_setb <= skb; so_setn <= g_n[skb];
            stg_busy[skb] <= 1'b0; skb <= !skb;
            kst <= K_REL;
          end
          default: if (!own_s) kst <= K_IDLE;
        endcase
      end
    end
  end


  // ---- the int16 converter, shared: viewport words in M_VPW (the polygon
  // machine is idle there, M_VPD), else u (P_B) and v (P_C) of the vertex
  // being assembled, times 256 (or / 8 * 256)
  // (the selects are registered a cycle ahead, from M_VPW's next cnt)
  logic       cv_vp, cv_lo, cv_b;
  logic [7:0] cv_ebv;
  always_ff @(posedge clk) begin
    logic [4:0] n;
    n = cnt + 5'd1;
    cv_vp  <= (mst == M_VPW);
    cv_b   <= (n == 5'd5);                                   // the ambient byte
    cv_lo  <= (n == 5'd6 || n == 5'd7);                      // the low half
    cv_ebv <= (n == 5'd5) ? 8'd119 : (n == 5'd6 || n == 5'd8) ? 8'd125 : 8'd123;
    amb_ld <= SM_LIGHT && mst == M_VPW && n == 5'd5;
    // M_VPW loads vq_r from vw_q every cycle, so this is vq_r[15:8] while amb_ld
    // is high: a register beside the decoder, not the main machine's widely used
    // vq_r (vq_r -> tc_k was the slowest path at -40C)
    amb_b  <= vw_q[15:8];
  end
  always_comb begin
    cv_in = cv_b ? {8'd0, vq_r[15:8]} : cv_lo ? vq_r[15:0] : vq_r[31:16];
    cv_eb = cv_ebv;
  end
  assign cv_q = itof16(cv_in, cv_eb);
  // the vertex's u/v word (study R109): registered off the block RAM and converted in P_C, both halves
  // at once, on a converter of their own. It was the block RAM's output through the shared converter's
  // select and the conversion into au (P_B) and cbb_wd (P_C); the word is the same one in both (the
  // address is held through P_B), so P_C has it registered. +1 converter, the u register gone.
  logic [31:0] pu_qr;
  always_ff @(posedge clk) pu_qr <= pu_q;
  wire  [7:0]  uv_eb = ra_q[27] ? 8'd135 : 8'd132;
  wire  [31:0] uv_u = itof16(pu_qr[31:16], uv_eb), uv_v = itof16(pu_qr[15:0], uv_eb);

  // ---- memory read addresses of the polygon machine ------------------------
  always_comb begin
    // vertex store: {ring, row, half}
    vs_ra = {ref_i[5:4], ref_i[3:2], (pst == P_B)};
    // u/v word of vertex as_i, read in P_A (data in P_B, held in P_C)
    pu_ra = {cp, as_i[1:0]};
    // clip pool
    case (pst)
      P_CL0: cb_ra = e_last;
      P_X1, P_X2: cb_ra = e_prev;
      P_Q0, P_Q1: cb_ra = e_qe;
      default: cb_ra = e_cur;
    endcase
  end

  // ---- queue counters (main <-> polygon <-> emitter) -----------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      cq_cnt <= '0; cq_wp <= '0; cq_rp <= '0;
    end else begin
      if (cq_push) begin cq_start[cq_wp] <= c_start; cq_end[cq_wp] <= c_end; cq_ring[cq_wp] <= c_ring; cq_wp <= cq_wp + 2'd1; end
      if (cq_pop) cq_rp <= cq_rp + 2'd1;
      cq_cnt <= cq_cnt + {2'd0, cq_push} - {2'd0, cq_pop};
    end
  end
  assign p_quiet = (pst == P_IDLE) && cq_cnt == 3'd0 && pq_np == 5'd0;

  // ===========================================================================
  // emitter: a word stream, one record at a time (m2-notes section 15)
  // polygon record, 4 + 6 nv words:
  //   word 0  {nv 5, transparency 6, param 6, fmt 3, tex_h 3, tex_w 3, page,
  //            tex_en, inverted copy, alpha buffer, pass 2}
  //   word 1  {0, tex_y 5, tex_x 6, polygon number 20}
  //   word 2  colour (header colour or the colour-table word)
  //   word 3  {0, texture inversion (study R135), faces away, translucency
  //            layer (header[6] bit 17), fog intensity (header[6] bits 11-15)}
  //   then nv vertices of 6 words: x, y, 1/w, u/w, v/w, shade
  // viewport record, 6 words (nv = 0): {0, pass 2}, node words 0x20 (scroll
  // fog), 0x22 (fog colour), 0x23 (fog density), 0x25 (fog ambient, start),
  // then kz (FP32, the depth scale: 1 for Supermodel's frustum)
  // The triangles are the fan (v0, v[i-1], v[i]). A double-sided polygon is
  // its inverted copy, then itself; with FACE_DROP only the one facing the
  // viewer (both when d is exactly 0), and no single-sided back face.
  // SETUP_OUT: a set-up record a triangle in place of the polygon record (R65's
  // SETUP4, m2-notes section 15), in the same order, the fan's triangles of
  // each record:
  //   words 0-3 (0-4 with LOD_WORD) the polygon header, word 0 with n = 3
  //   x1 y1 x2 y2 x3 y3   screen x, y sorted by y (MAME's sort, rtl_tri::setup)
  //   det
  //   pdx, pdy, ps of 1/w, u/w, v/w, shade (textured, 23 words) or 1/w, shade
  //   (17), FP32 in rtl_tri::setup's order; a flat triangle (|det| < 0.00001)
  //   has pdx = pdy = +0 and ps the first sorted vertex's parameter
  // A triangle with a non-finite x, y or 1/w is not sent.
  // ===========================================================================
  logic        ebuf;
  logic [4:0]  enp, pe;
  logic [3:0]  eb_base;
  logic [4:0]  e_cnt;
  logic [2:0]  e_h;              // header word
  logic [2:0]  e_c;              // vertex word, 0..5
  logic [3:0]  e_q;              // vertex in the output buffer half
  logic [4:0]  e_v;              // vertices left
  logic        e_inv;            // the inverted copy of a double-sided polygon: shade' (unless fixed)
  logic        e_vtx;            // in the vertex words
  logic        e_gt, e_lt;       // the polygon's verdict
  logic        e_two;            // the polygon's own record follows the copy
  logic        e_cf, e_pf;       // cull flag of this record, of the polygon's own record
  logic [2:0]  vr_k;             // viewport record word
  assign pm_ra = SETUP_OUT ? {sbuf, spe[3:0]} : {ebuf, pe[3:0]};
  logic        out_go;
  assign out_go = !out_valid || out_ready;
  // the output buffer's component of vertex word k: x y 1/w at 0-2, u/w v/w at 4-5, shade 6
  function automatic logic [2:0] vcomp(input logic [2:0] k);
    vcomp = (k < 3'd3) ? k : k + 3'd1;
  endfunction
  // the viewport record's word k from vpw 0..5 (node words 0x20..0x25)
  assign vr_ra = (vr_k == 3'd1) ? 5'd0 : (vr_k == 3'd2) ? 5'd2 : (vr_k == 3'd3) ? 5'd3 : 5'd5;

  // OUT_M10K: the output buffer is read a cycle ahead, the next beat's word:
  // the record's first vertex during the header, the word after a beat, the
  // same word in a stall
  always_comb begin
    logic [3:0] q;
    logic [2:0] c;
    q = e_q; c = vcomp(e_c);
    if (OUT_M10K) begin
      if (est != E_OUT || !e_vtx) begin q = eb_base; c = 3'd0; end
      else if (out_go) begin
        if (e_c == 3'd5) begin q = e_q + 4'd1; c = 3'd0; end
        else c = vcomp(e_c + 3'd1);
      end
    end
    po_ra = {ebuf, q[3:2], (c == 3'd6 && e_inv && !re_q[90]) ? 3'd7 : c};
    // SETUP_OUT: the set-up port reads the projection output
    if (SETUP_OUT) po_ra = (s_st == S_Y) ? {sbuf, s_qy[3:2], 3'd1} : {sbuf, s_qg[3:2], s_comp};
    re_ra = e_poly;
  end
  // the record's header word e_h, the viewport record's word vr_k (both emitter paths)
  logic [31:0] e_hw, e_vw;
  always_comb begin
    case (e_h)
      3'd0: e_hw = {e_cnt, re_q[56:51], re_q[50:45], re_q[44:42], re_q[41:39], re_q[38:36], re_q[24], re_q[23],
                    e_inv, re_q[2], re_q[1:0]};
      3'd1: e_hw = {1'b0, re_q[35:31], re_q[30:25], re_q[22:3]};
      3'd2: e_hw = re_q[88:57];
      3'd3: e_hw = {24'd0, re_q[97], SM_CULL && e_cf, re_q[96], re_q[95:91]};
      default: e_hw = e_inv ? negf(fb_q) : fb_q;              // LOD_WORD: B (the copy's -B)
    endcase
    case (vr_k)
      3'd0: e_vw = {30'd0, pass};
      3'd5: e_vw = FRUSTUM_SM ? F_ONE : negf(F_P23);
      default: e_vw = vw_q;
    endcase
  end
  assign e_quiet = (est == E_IDLE) && eb_cnt == 2'd0 &&
                   (!SETUP_OUT || (s_st == S_IDLE && sl == 2'd0 && stg_busy == 2'd0 && kst == K_IDLE &&
                                   tq_cnt == 5'd0 && !tq_we && so_full == 2'd0 && !so_set));

  // ---- SETUP_OUT: the emitter's set-up records -------------------------------
  // per triangle: the polygon header (word 0 with n = 3; word 4 the LOD word
  // with LOD_WORD), then from its lane of the set-up output half: x1 y1 x2 y2
  // x3 y3, det, and per parameter pdx, pdy, ps; a flat triangle (|det| <
  // 0.00001) sends pdx = pdy = +0 and ps = the first sorted vertex's parameter
  // (output word 8 + 4 p + 3). The output memory is read a cycle ahead.
  logic [1:0]  el;                // the triangle's lane
  logic        ebs;               // its set-up output half
  logic [2:0]  e_k;               // x1 .. det (0-6)
  logic        e_pl;              // in the parameters
  logic [1:0]  e_sp, e_sc;        // parameter, coefficient (pdx, pdy, ps)
  logic        e_flat, e_tex;
  logic [31:0] so_lq;
  assign so_lq = so_q[el];
  localparam logic [30:0] F_EPS = 31'h3727_c5ac;   // 0.00001f
  always_comb begin
    logic [4:0] cur, nxt;
    cur = !e_pl ? {2'b00, e_k} : {3'(e_sp) + 3'd2, (e_flat && e_sc == 2'd2) ? 2'd3 : e_sc};
    if (!e_pl) nxt = (e_k == 3'd6) ? 5'd8 : {2'b00, e_k + 3'd1};
    else if (e_sc == 2'd2) nxt = {3'(e_sp) + 3'd3, 2'd0};
    else nxt = {3'(e_sp) + 3'd2, (e_flat && e_sc == 2'd1) ? 2'd3 : e_sc + 2'd1};
    so_ra = {ebs, (est != E_OUT || !e_vtx) ? 5'd0 : out_go ? nxt : cur};
  end
  always_ff @(posedge clk) begin
    if (rst) so_full <= '0;
    else begin
      if (so_set) begin so_full[so_setb] <= 1'b1; so_n[so_setb] <= so_setn; end
      if (so_clr) so_full[so_clrb] <= 1'b0;
    end
  end


  always_ff @(posedge clk) begin
    if (rst) begin
      est <= E_IDLE; out_valid <= 1'b0; e_active <= 1'b0; eb_pop <= 1'b0; vr_done <= 1'b0;
      so_clr <= 1'b0; el <= '0; ebs <= 1'b0; tq_rp <= '0;
    end else begin
      eb_pop <= 1'b0; vr_done <= 1'b0; so_clr <= 1'b0;
      if (out_valid && out_ready) out_valid <= 1'b0;
      if (SETUP_OUT) begin
        case (est)
          E_IDLE: if (vr_go && !vr_done) begin
            vr_k <= '0; est <= E_VR;
          end else if (tq_cnt != 5'd0 && so_full[ebs]) begin
            e_poly <= tq_q[5:0]; e_inv <= tq_q[6]; e_cf <= tq_q[7]; e_tex <= tq_q[8];
            e_cnt <= 5'd3; e_active <= 1'b1; e_h <= '0; e_vtx <= 1'b0;
            est <= E_P2;
          end
          E_VR: if (out_go) begin
            out_valid <= 1'b1;
            out_data <= e_vw;
            out_last <= vr_k == 3'd5;
            vr_k <= vr_k + 3'd1;
            if (vr_k == 3'd5) begin vr_done <= 1'b1; est <= E_IDLE; end
          end
          // (LOD_WORD: the record waits for its polygon's B)
          E_P2: if (!LOD_WORD || fb_rdy) est <= E_OUT;
          E_OUT: if (out_go) begin
            out_valid <= 1'b1;
            out_last <= 1'b0;
            if (!e_vtx) begin
              out_data <= e_hw;
              e_h <= e_h + 3'd1;
              if (e_h == (LOD_WORD ? 3'd4 : 3'd3)) begin e_vtx <= 1'b1; e_k <= '0; e_pl <= 1'b0; e_sp <= '0; e_sc <= '0; end
            end else if (!e_pl) begin
              out_data <= so_lq;
              if (e_k == 3'd6) begin e_flat <= so_lq[30:0] < F_EPS; e_pl <= 1'b1; end
              e_k <= e_k + 3'd1;
            end else begin
              out_data <= (e_flat && e_sc != 2'd2) ? 32'd0 : so_lq;
              if (e_sc == 2'd2) begin
                e_sc <= '0; e_sp <= e_sp + 2'd1;
                if (e_sp == (e_tex ? 2'd3 : 2'd1)) begin
                  out_last <= 1'b1; e_active <= 1'b0; tq_rp <= tq_rp + 5'd1;
                  if (3'(el) + 3'd1 == so_n[ebs]) begin el <= '0; ebs <= !ebs; so_clr <= 1'b1; so_clrb <= ebs; end
                  else el <= el + 2'd1;
                  est <= E_IDLE;
                end
              end else e_sc <= e_sc + 2'd1;
            end
          end
          default: est <= E_IDLE;
        endcase
      end else
      case (est)
        E_IDLE: if (vr_go && !vr_done) begin
          vr_k <= '0; est <= E_VR;
        end else if (eb_cnt != 2'd0 && !eb_pop) begin
          ebuf <= eb_buf[eb_rp]; enp <= eb_np[eb_rp]; e_poly <= eb_first[eb_rp]; pe <= '0; e_active <= 1'b1;
          est <= E_P0;
        end
        E_VR: if (out_go) begin
          out_valid <= 1'b1;
          out_data <= e_vw;
          out_last <= vr_k == 3'd5;
          vr_k <= vr_k + 3'd1;
          if (vr_k == 3'd5) begin vr_done <= 1'b1; est <= E_IDLE; end
        end
        E_P0: begin
          if (pe == enp) begin eb_pop <= 1'b1; e_active <= 1'b0; est <= E_IDLE; end
          else begin
            e_poly <= pm_q[5:0]; eb_base <= pm_q[9:6]; e_cnt <= pm_q[14:10]; e_gt <= pm_q[16]; e_lt <= pm_q[15];
            est <= E_P1;
          end
        end
        // the records: the inverted copy (faces away when d < 0), then the
        // polygon itself (faces away when d > 0)
        // (LOD_WORD: a record waits for its polygon's B)
        E_P1: begin
          logic dbl, sc, so;
          dbl = SM_DOUBLE && re_q[89];
          sc = dbl && !(FACE_DROP && e_lt);
          so = !(FACE_DROP && e_gt);
          e_inv <= sc; e_two <= sc && so;
          e_cf <= sc ? e_lt : e_gt; e_pf <= e_gt;
          e_h <= '0; e_vtx <= 1'b0;
          if (sc || so) begin if (!LOD_WORD || fb_rdy) est <= E_OUT; end
          else begin pe <= pe + 5'd1; est <= E_P0; end
        end
        E_P2: begin e_cf <= e_pf; e_h <= '0; e_vtx <= 1'b0; est <= E_OUT; end
        E_OUT: if (out_go) begin
          out_valid <= 1'b1;
          out_last <= 1'b0;
          if (!e_vtx) begin
            out_data <= e_hw;
            e_h <= e_h + 3'd1;
            if (e_h == (LOD_WORD ? 3'd4 : 3'd3)) begin e_vtx <= 1'b1; e_q <= eb_base; e_c <= '0; e_v <= e_cnt; end
          end else begin
            out_data <= po_q[e_q[1:0]];
            e_c <= e_c + 3'd1;
            if (e_c == 3'd5) begin
              e_c <= '0; e_q <= e_q + 4'd1; e_v <= e_v - 5'd1;
              if (e_v == 5'd1) begin
                out_last <= 1'b1;
                if (e_two) begin e_inv <= 1'b0; e_two <= 1'b0; est <= E_P2; end
                else begin pe <= pe + 5'd1; est <= E_P0; end
              end
            end
          end
        end
        default: est <= E_IDLE;
      endcase
    end
  end

  // batches: polygon -> emitter
  always_ff @(posedge clk) begin
    if (rst) begin
      eb_cnt <= '0; eb_wp <= '0; eb_rp <= '0; po_busy <= '0;
    end else begin
      if (eb_push) begin eb_buf[eb_wp] <= eb_wbuf; eb_np[eb_wp] <= eb_wnp; eb_first[eb_wp] <= eb_wfirst; eb_wp <= !eb_wp; end
      if (SETUP_OUT ? s_pop : eb_pop) eb_rp <= !eb_rp;
      eb_cnt <= eb_cnt + {1'b0, eb_push} - {1'b0, SETUP_OUT ? s_pop : eb_pop};
      if (eb_push) po_busy[eb_wbuf] <= 1'b1;
      if (SETUP_OUT ? s_pop : eb_pop) po_busy[SETUP_OUT ? sbuf : ebuf] <= 1'b0;
    end
  end

  // ===========================================================================
  // decoder: the model's polygons into chunks, running ahead of the engine
  // ===========================================================================
  logic [9:0]  h0;
  logic [6:0]  h1;
  logic [1:0]  h2;
  logic [7:0]  h3;
  logic        p_fixed;           // this polygon: its vertices take fixed shading
  assign p_fixed = SM_FIXED && h1[5] && !h1[3];
  logic [7:0]  xb_r;              // a new vertex's x byte, for its fixed shade
  logic [31:0] h4, h6;
  logic [7:0]  h5;
  logic [2:0]  hk;
  logic        md_first;
  logic [31:0] colour;
  logic [7:0]  chunk;
  logic [1:0]  ring;
  logic        tbank;
  logic [19:0] poly_seq;
  logic [4:0]  tl_n, ch_n;
  logic [2:0]  nv, iv, nre;
  logic [1:0]  wk;
  logic [1:0]  kk;
  logic        tq_v;              // a staging word waiting for conversion
  logic [1:0]  tq_lane;
  logic [4:0]  tq_wa;
  logic [31:0] tq_w;
  logic        tq_fw;             // the F word (comp 3): -0, or the fixed shade of byte tq_w[7:0]
  logic        tq_fix, tq_spec;
  logic        tq_nz;             // staged normal zero (fixed shading)
  logic        tc_v;              // the conversion's second step
  logic [1:0]  tc_lane, tc_sel;
  logic [4:0]  tc_wa;
  logic [31:0] tc_nw;
  // the position word's conversion in two halves (timing, study R104: the negate, the leading-one
  // search and the shift in one cycle were -0.81 ns at 100 MHz): the magnitude and its leading one
  // here, the shift and the exponent as the word is written (ts_wp), where only a register moved
  logic        tc_ps, tc_pz;      // sign; the word is 0 (the F word, or a zero magnitude)
  logic [23:0] tc_pm;             // the magnitude
  logic [4:0]  tc_pmsb;           // its leading one
  logic [7:0]  tc_peb;            // the exponent base
  // the byte's 1/255 product as a table of the byte (study R109): k255 is a function of eight bits, so
  // only the sign and the byte are registered, and the 256 words are constants (folded at synthesis;
  // kept in logic, not a block RAM). The product's add, the shift, the round and the pack ran tq_w ->
  // tc_k -> xb_data and ts_wn over two cycles.
  logic        tc_ks;
  logic [7:0]  tc_km;
  (* romstyle = "logic" *) logic [30:0] k255t [256];
  genvar gk;
  generate for (gk = 0; gk < 256; gk = gk + 1) begin : g_k255t
    wire [31:0] kw = k255(1'b0, 8'(gk));
    assign k255t[gk] = kw[30:0];
  end endgenerate
`ifdef VERILATOR
  logic [31:0] tc_wpc;            // the one-cycle conversion, for the check
  logic [31:0] tc_kchk;           // k255 as functions, for the table's check
  logic        tc_kv;
  always_ff @(posedge clk) begin
    if (rst) tc_kv <= 1'b0; else if (tq_v || amb_ld) tc_kv <= 1'b1;
    if (!rst && tc_kv && tc_kq != tc_kchk) $fatal(1, "geo: the k255 table");
  end
`endif
  logic [31:0] tc_kq;             // k255(tc_ks, tc_km): the staged normal or fixed shade, or the ambient
  assign tc_kq = (tc_km == 8'd0) ? 32'd0 : {tc_ks, k255t[tc_km]};
  // vertex slots by reference: eight raw-word entries, MAME's prev[k] is entry
  // pmap[k] and the polygon's vertex[k] entry cmap[k]. A reuse copies the
  // reference, not the words; a new vertex takes an entry no other cur slot
  // holds (at most three of eight); prev becomes cur at commit
  logic [2:0]  pmap [4], cmap [4];
  logic [2:0]  palloc;
  // per slot: transform reference and its validity (prev: vref*, cur: nref*)
  logic [5:0]  vref [4], nref [4];
  logic [7:0]  vref_chunk [4], nref_chunk [4];
  logic        vref_ok [4], nref_ok [4];
  // per slot: lit with fixed shading (its store entry), its producer's specular bit
  logic        vkfix [4], nkfix [4], vspec [4], nspec [4];
  logic        retx [4];          // cur slot: a reused vertex to transform again
  logic        vin [4];           // prev slot's transform is in the store (window: this chunk and the one before)
  always_ff @(posedge clk) for (int v = 0; v < 4; v++) vin[v] <= vref_ok[v] && (chunk - vref_chunk[v]) < 8'd2;
  // a reused vertex's store entry serves this polygon if it was lit the same
  // way: fixed shading only from a fixed entry, the dot product only from a
  // dot entry
  function automatic logic reuse_ok(input logic [1:0] v);
    reuse_ok = vin[v] && (p_fixed ? vkfix[v] : !vkfix[v]);
  endfunction
  // the entries' raw words, one memory {word (0 x, 1 y, 2 z, 3 uv), entry}:
  // stream words written as they arrive, read one a cycle
  logic        sl_we;
  logic [4:0]  sl_wa, sl_ra;
  logic [31:0] sl_q;
  r3d_ram #(.W(32), .A(5)) u_sl (.clk, .we(sl_we), .wa(sl_wa), .wd(ms_w), .ra(sl_ra), .rq(sl_q));
  // source slot of cur slot k: the k-th reuse bit of h0
  function automatic logic [1:0] rsrc(input logic [3:0] bits, input logic [1:0] k);
    logic [2:0] c;
    rsrc = k;
    c = 3'd0;
    for (int v = 0; v < 4; v++) if (bits[v]) begin
      if (c == {1'b0, k}) rsrc = 2'(v);
      c = c + 3'd1;
    end
  endfunction

  // the facing test's inputs (SM_CULL), raw words into its fill slot: the
  // face normal at header words 1-3, vertex 0's x y z as it streams in (a new
  // vertex 0) or from its slot entry during the header (a reused one)
  logic [1:0]  fk;                // reused vertex 0: words fed
  logic        f_nw, f_vw, f_rw, d_fstall;
  logic [1:0]  v0src;
  assign v0src = rsrc(h0[3:0], 2'd0);
  // hk_nw: header word hk goes to the facing test (1-3, or with LOD_WORD 5), a register beside hk (c7_s1: the
  // decode in series with dst and the stall, into ms_pop, the stream's rp and xb_r, -1.16 to -1.41 ns)
  logic hk_nw;
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && dst == D_HDR && hk_nw != ((hk >= 3'd1 && hk <= 3'd3) || (LOD_WORD && hk == 3'd5))) $fatal(1, "geo: hk_nw");
`endif
  always_comb begin
    // normal word, or with LOD_WORD header word 5 (NP, slot word 7)
    f_nw = SM_CULL && dst == D_HDR && ms_wv && hk_nw;
    f_vw = SM_CULL && dst == D_NEWV && ms_wv && iv == 3'd0 && wk != 2'd3;             // new vertex 0
    f_rw = SM_CULL && ((dst == D_HDR && hk >= 3'd1) || dst == D_CHK) && h0[3:0] != 4'd0 && fk != 2'd3 && !f_nw && f_ready;
    f_we = (f_nw || f_vw || f_rw) && f_ready;     // never into a slot still in use
    f_word = f_nw ? 3'(hk + 3'd2) : f_vw ? {1'b0, wk} : {1'b0, fk};
    f_d = f_rw ? sl_q[31:8] : ms_w[31:8];
    d_fstall = (f_nw && (!f_ready || f_hold)) || (f_vw && f_hold);
    f_commit = SM_CULL && dst == D_COMMIT;
  end
  assign ms_pop = ((dst == D_HDR) || (dst == D_NEWV)) && ms_wv && !d_fstall;
  assign d_quiet = (dst == D_IDLE) && (d_req == d_ack);

  always_comb begin
    logic [1:0] s;
    logic [7:0] used;
    s = rsrc(h0[3:0], kk);
    // a free entry for new vertex iv: lowest one no other cur slot holds
    used = '0;
    for (int j = 0; j < 4; j++) if (2'(j) != iv[1:0]) used[cmap[j]] = 1'b1;
    palloc = 3'd0;
    for (int e = 7; e >= 0; e--) if (!used[e]) palloc = 3'(e);
    sl_we = (dst == D_NEWV) && ms_wv;
    sl_wa = {wk, (wk == 2'd0) ? palloc : cmap[iv[1:0]]};
    // D_REUSE: the source's u/v; D_INV: the cur vertex's x, y, z, then x again (its byte)
    sl_ra = (dst == D_INV) ? {(wk == 2'd3) ? 2'd0 : wk, cmap[iv[1:0]]} :
            (SM_CULL && (dst == D_HDR || dst == D_CHK)) ? {fk, pmap[v0src]} : {2'd3, pmap[s]};
  end

  // the vertices a polygon needs transformed: its new ones, and reused ones whose transform is not in the
  // store (window: this chunk and the one before). Registered (study R109): h0, h1 (p_fixed), vin and vkfix
  // hold for the header's seven words before D_CHK, and for D_HOLD's two cycles after D_CLOSE moves the
  // chunk on (vin follows chunk a cycle later), so the registered count is the one D_CHK would compute.
  // The count, its add and the compare with the chunk's room ran into every nref and dst enable.
  function automatic logic [4:0] need_of(input logic [9:0] h);
    logic [2:0] nvv, vii, k_;
    nvv = h[6] ? 3'd4 : 3'd3;
    vii = 3'd0;
    for (int v = 0; v < 4; v++) if (h[v]) vii = vii + 3'd1;
    need_of = (vii < nvv) ? 5'(nvv - vii) : 5'd0;
    k_ = 3'd0;
    for (int v = 0; v < 4; v++) if (h[v]) begin
      if (k_ < nvv && !reuse_ok(2'(v))) need_of = need_of + 5'd1;
      k_ = k_ + 3'd1;
    end
  endfunction
  logic [4:0] need_q;
  always_ff @(posedge clk) need_q <= need_of(h0);
`ifdef VERILATOR
  always_ff @(posedge clk) if (!rst && dst == D_CHK && need_q != need_of(h0)) $fatal(1, "geo: need registered");
`endif

  always_ff @(posedge clk) begin
    if (rst) begin
      dst <= D_IDLE; d_ack <= 1'b0;
      ms_start <= 1'b0; ms_stop <= 1'b0; ms_creq <= 1'b0; dq_push <= 1'b0;
      chunk <= '0; ring <= '0; tbank <= 1'b0; ch_wp <= '0; ch_start <= '0; poly_seq <= '0;
      for (int i = 0; i < 4; i++) pmap[i] <= 3'(i);
      ts_we <= '0; pu_we <= 1'b0; ra_we <= 1'b0; re_we <= 1'b0; tq_v <= 1'b0; tc_v <= 1'b0;
      for (int i = 0; i < 4; i++) begin vref_ok[i] <= 1'b0; vref[i] <= '0; vref_chunk[i] <= '0; vkfix[i] <= 1'b0; vspec[i] <= 1'b0; end
    end else begin
      ms_start <= 1'b0; ms_stop <= 1'b0; ms_creq <= 1'b0; dq_push <= 1'b0;
      ts_we <= '0; pu_we <= 1'b0; ra_we <= 1'b0; re_we <= 1'b0;
      // a staging word is converted the cycle after it is taken (17.7 to FP32)
      // The F word (comp 3): -0, or the fixed shade (signed (2b + 1) / 255, or
      // with the specular bit b / 255); its position word is unused.
      // step 1: the position converted, the byte's / 255 product formed (or
      // MAME's s.7 / 127 table); step 2: rounded, written
      tq_v <= 1'b0; tq_fw <= 1'b0;
      tc_v <= tq_v;
      // the 1/255 product of one byte: the F word's fixed shade (b / 255 with
      // the specular bit, else (2b + 1) / 255), the normal byte, or in M_VPW
      // the ambient byte (the decoder is idle there, M_VPD)
      if (tq_v || amb_ld) begin
        logic [7:0] kb;
        kb = amb_ld ? amb_b : tq_w[7:0];
        // |2b + 1| of the signed byte b: 2b + 1 is odd, so -(2b + 1) is {~b, 1}
        // in nine bits, and its low eight bits are {~b[6:0], 1}; no subtract
        // (the subtract's carry chain made this the slowest path at -40C)
        if (amb_ld || (tq_fw && tq_spec)) begin tc_ks <= 1'b0; tc_km <= kb; end
        else begin tc_ks <= kb[7]; tc_km <= {kb[6:0] ^ {7{kb[7]}}, 1'b1}; end
`ifdef VERILATOR
        if (amb_ld || (tq_fw && tq_spec)) tc_kchk <= k255(1'b0, kb);
        else tc_kchk <= k255(kb[7], {kb[6:0] ^ {7{kb[7]}}, 1'b1});
`endif
      end
      if (tq_v) begin
        begin
          logic [23:0] mg;
          logic [4:0]  ms;
          logic        nzm;
          mg = tq_w[31] ? (~tq_w[31:8] + 24'd1) : tq_w[31:8];
          ms = 5'd0; nzm = 1'b0;
          for (int i = 0; i < 24; i++) if (mg[i]) begin ms = 5'(i); nzm = 1'b1; end
          tc_ps <= tq_w[31]; tc_pm <= mg; tc_pmsb <= ms; tc_pz <= tq_fw || !nzm;
          tc_peb <= step15 ? 8'd116 : 8'd120;
`ifdef VERILATOR
          tc_wpc <= tq_fw ? 32'd0 : itof24(tq_w[31], mg, step15 ? 8'd116 : 8'd120);
`endif
        end
        tc_lane <= tq_lane; tc_wa <= tq_wa;
        tc_sel <= tq_fw ? (tq_fix ? 2'd1 : 2'd2) : tq_nz ? 2'd3 : 2'd0;   // 0 normal, 1 product, 2 -0, 3 zero
        if (!SM_FLAT) tc_nw <= ntab(tq_w[7:0]);
      end
      if (tc_v) begin
        ts_we <= 4'(1 << tc_lane); ts_wa <= tc_wa;
        begin
          logic [23:0] m;
          m = tc_pm << (5'd23 - tc_pmsb);
          ts_wp <= tc_pz ? 32'd0 : {tc_ps, tc_peb + {3'd0, tc_pmsb}, m[22:0]};
`ifdef VERILATOR
          if ((tc_pz ? 32'd0 : {tc_ps, tc_peb + {3'd0, tc_pmsb}, m[22:0]}) != tc_wpc) $fatal(1, "geo: itof24 in two halves");
`endif
        end
        case (tc_sel)
          2'd0: ts_wn <= SM_FLAT ? tc_kq : tc_nw;
          2'd1: ts_wn <= tc_kq;
          2'd2: ts_wn <= 32'h8000_0000;
          default: ts_wn <= 32'd0;
        endcase
      end
      if (f_init) begin poly_seq <= '0; ch_wp <= '0; ch_start <= '0; tbuf_d <= 1'b0; end
      if (f_rw && !f_hold) fk <= fk + 2'd1;
      case (dst)
        D_IDLE: if (d_req != d_ack && ms_ready) begin
          d_ack <= d_req;
          tbuf_d <= !tbuf_d;
          ms_start <= 1'b1;
          pass_d <= pass;
          d_ctab <= m_ctab;
          md_first <= 1'b1;
          // shared vertices of another model are transformed again
          for (int i = 0; i < 4; i++) vref_ok[i] <= 1'b0;
          tl_n <= '0; ch_n <= '0; ch_start <= ch_wp; hk <= '0; hk_nw <= 1'b0;
          dst <= D_HDR;
        end
        D_HDR: if (ms_wv && !d_fstall) begin
          case (hk)
            3'd0: begin h0 <= ms_w[9:0]; fk <= '0; end
            3'd1: h1 <= ms_w[6:0];
            3'd2: h2 <= ms_w[1:0];
            3'd3: h3 <= ms_w[7:0];
            default: ;
          endcase
          case (hk)
            3'd4: h4 <= ms_w;
            3'd5: h5 <= ms_w[7:0];
            3'd6: h6 <= ms_w;
            default: ;
          endcase
          hk <= hk + 3'd1;
          hk_nw <= (hk <= 3'd2) || (LOD_WORD && hk == 3'd4);   // word hk + 1
          if (hk == 3'd6) begin hk <= '0; hk_nw <= 1'b0; dst <= D_CHK; end
        end
        D_CHK: begin
          logic [2:0] nvv, vii;
          logic [4:0] need;
          logic       ok;
          nvv = h0[6] ? 3'd4 : 3'd3;
          vii = 3'd0;
          for (int v = 0; v < 4; v++) if (h0[v]) vii = vii + 3'd1;
          need = need_q;
          ok = 1'b1;
          if (md_first && h0[3:0] != 4'd0) ok = 1'b0;
          // (the facing test's reused vertex 0 is fed during the header; a
          // write held off by the unit can leave a word for here)
          else if (h6 == 32'd0) ok = 1'b0;
          else if (h0[9:8] == 2'b11) ok = 1'b0;
          // the cur slots loaded every D_CHK cycle (study R109), not only as it moves on: they are copies
          // of the prev slots and of h0, which hold through D_CHK, and nothing reads them before the next
          // D_CHK unless it moves on to D_REUSE, D_NEWV or D_INV. Their enables were behind ring_full,
          // the close test and ok (the ring's distance into some 40 enables).
          nv <= nvv; nre <= vii; kk <= '0; wk <= 2'd0;
          for (int i = 0; i < 4; i++) begin
            nref[i] <= vref[i]; nref_ok[i] <= vref_ok[i]; nref_chunk[i] <= vref_chunk[i]; retx[i] <= 1'b0;
            nkfix[i] <= vkfix[i]; nspec[i] <= vspec[i];
          end
          for (int i = 0; i < 4; i++) cmap[i] <= pmap[i];
          iv <= (vii < nvv) ? vii : 3'd0;
          if (ring_full || (SM_CULL && h0[3:0] != 4'd0 && fk != 2'd3 && ok)) begin
            // the polygon ring is nearly full: wait for the emitter; or vertex
            // 0's words are not all in the facing test yet
          end else if (!ok) begin
            dst <= D_END;
          end else if ((tl_n + need > 5'd16 || ch_n == 5'd16) && ch_n != 5'd0) begin
            d_after <= D_CHK; dq_wlast <= 1'b0; dst <= D_CLOSE;   // close the chunk, then this polygon
          end else begin
            md_first <= 1'b0;
            dst <= (vii != 3'd0) ? D_REUSE : (vii < nvv) ? D_NEWV : D_INV;
          end
        end
        // vertex[k] = prev[k-th reuse bit], a slot a cycle; its u/v word into the ring
        D_REUSE: begin
          logic [1:0] s_;
          s_ = rsrc(h0[3:0], kk);
          cmap[kk] <= pmap[s_];
          pu_we <= 1'b1; pu_wa <= {ch_wp, kk}; pu_wd <= sl_q;
          nref[kk] <= vref[s_]; nref_ok[kk] <= vref_ok[s_]; nref_chunk[kk] <= vref_chunk[s_];
          nspec[kk] <= vspec[s_];
          nkfix[kk] <= ({1'b0, kk} < nv) ? p_fixed : vkfix[s_];
          retx[kk] <= ({1'b0, kk} < nv) && !reuse_ok(s_);
          kk <= kk + 2'd1;
          if ({1'b0, kk} + 3'd1 == nre) dst <= (nre < nv) ? D_NEWV : D_INV;
        end
        // new vertices: x, y, z, u/v words, one a cycle from the stream; x y z
        // go straight into the staging memory (position and normal)
        D_NEWV: if (ms_wv && !d_fstall) begin
          tq_v <= 1'b1; tq_lane <= tl_n[1:0]; tq_wa <= {tbank, tl_n[3:2], wk};
          tq_nz <= p_fixed;
          tq_fix <= p_fixed; tq_spec <= h0[7];
          if (wk == 2'd0) begin xb_r <= ms_w[7:0]; cmap[iv[1:0]] <= palloc; end
          if (wk != 2'd3) tq_w <= ms_w;
          else begin
            tq_fw <= 1'b1; tq_w <= {24'd0, xb_r};
            pu_we <= 1'b1; pu_wa <= {ch_wp, iv[1:0]}; pu_wd <= ms_w;
          end
          wk <= wk + 2'd1;
          if (wk == 2'd3) begin
            nref[iv[1:0]] <= {ring, tl_n[3:0]}; nref_ok[iv[1:0]] <= 1'b1; nref_chunk[iv[1:0]] <= chunk;
            nkfix[iv[1:0]] <= p_fixed; nspec[iv[1:0]] <= h0[7];
            retx[iv[1:0]] <= 1'b0;
            tl_n <= tl_n + 5'd1;
            iv <= iv + 3'd1;
            if (iv + 3'd1 == nv) begin dst <= D_INV; iv <= 3'd0; end
          end
        end
        // reused vertices whose transform has left the store: stage them from
        // their slot words, a component a cycle
        D_INV: begin
          if (iv == nv) begin
            dst <= h1[1] ? D_COMMIT : D_COL;
            colour <= {8'd0, h4[31:8]};
          end else if (retx[iv[1:0]]) begin
            logic [31:0] w;
            w = sl_q;
            tq_v <= 1'b1; tq_lane <= tl_n[1:0]; tq_wa <= {tbank, tl_n[3:2], wk};
            tq_w <= (wk == 2'd3) ? {24'd0, sl_q[7:0]} : w;
            tq_fw <= wk == 2'd3; tq_fix <= p_fixed; tq_spec <= nspec[iv[1:0]];
            tq_nz <= p_fixed;
            wk <= wk + 2'd1;
            if (wk == 2'd3) begin
              wk <= '0;
              nref[iv[1:0]] <= {ring, tl_n[3:0]}; nref_ok[iv[1:0]] <= 1'b1; nref_chunk[iv[1:0]] <= chunk;
              nkfix[iv[1:0]] <= p_fixed;
              tl_n <= tl_n + 5'd1;
              iv <= iv + 3'd1;
            end
          end else iv <= iv + 3'd1;
        end
        D_COL: begin
          ms_creq <= 1'b1;
          ms_caddr <= SM_CTAB ? d_ctab + {8'd0, h4[19:8]} : 20'h400 + {9'd0, h4[18:8]};
          dst <= D_COLW;
        end
        D_COLW: if (ms_cv) begin colour <= ms_cd; dst <= D_COMMIT; end
        D_COMMIT: begin
          logic ex4, alpha, talpha, cm, drop;
          ex4 = h6[10] && (h3[5:3] >= 3'd6 || h3[2:0] >= 3'd6);
          // texture alpha: h6[0] on Step 1.0; from Step 1.5 any of h6[2:0] (Supermodel's
          // TextureAlpha: Daytona 2's smoke sets bit 1 or 2 alone, with formats 1 and 3;
          // VF3 keeps bit 0, study R100 hand-off 5); or polygon translucency
          talpha = step15 ? (|h6[2:0]) : h6[0];
          alpha = talpha || !h6[23];
          cm = h6[10] && (h6[9:7] != 3'd7) && h3[7];
          drop = SM_TRANS && !h6[23] && h6[22:18] == 5'd0;   // Supermodel: alpha byte 0, not drawn
          ra_we <= 1'b1; ra_wa <= ch_wp;
          rs_wd <= {p_fixed, SM_DOUBLE && h1[4], h6[10]};
          ra_wd <= {SM_DOUBLE && h1[4], ex4 || drop, h6[16], h1[6], nv, nref[3], nref[2], nref[1], nref[0]};
          re_we <= 1'b1; re_wa <= ch_wp;
          re_wd <= {h6[10] && h6[30:24] == 7'd2, h6[17], h6[15:11], p_fixed, SM_DOUBLE && h1[4], colour, h6[23] ? 6'd32 : {1'b0, h6[22:18]}, cm, h6[31], h6[10], h2[0], h2[1], talpha,
                    h6[9:7], h3[2:0], h3[5:3], h5[4:0], h4[4:0], h5[7], h4[6], h6[10], poly_seq, alpha, pass_d};
          ch_wp <= ch_wp + 6'd1;
          ch_n <= ch_n + 5'd1;
          poly_seq <= poly_seq + 20'd1;
          // MAME's vertex[] becomes prev for the next polygon
          for (int i = 0; i < 4; i++) pmap[i] <= cmap[i];
          for (int i = 0; i < 4; i++) begin
            vref[i] <= nref[i]; vref_ok[i] <= nref_ok[i]; vref_chunk[i] <= nref_chunk[i];
            vkfix[i] <= nkfix[i]; vspec[i] <= nspec[i];
          end
          if (ex4 || h1[2]) dst <= D_END;
          else dst <= D_HDR;
        end
        D_END: begin
          ms_stop <= 1'b1;
          d_after <= D_IDLE; dq_wlast <= 1'b1; dst <= D_CLOSE;  // the model's last chunk, empty or not
        end
        // hand the chunk to main; start the next one only into a free bank
        D_CLOSE: if (dq_cnt != 2'd2) begin
          dq_push <= 1'b1;
          dq_wstart <= ch_start; dq_wend <= ch_wp; dq_wtl <= tl_n; dq_wring <= ring; dq_wbank <= tbank;
          ch_n <= '0; tl_n <= '0; ch_start <= ch_wp;
          chunk <= chunk + 8'd1; ring <= ring + 2'd1; tbank <= !tbank;
          dst <= D_HOLD;
        end
        D_HOLD: if (!dq_push && dq_cnt != 2'd2) dst <= d_after;
        default: dst <= D_IDLE;
      endcase
    end
  end

  // ---- chunk descriptors: decoder -> main, two deep
  always_ff @(posedge clk) begin
    if (rst) begin
      dq_cnt <= '0; dq_wp <= '0; dq_rp <= '0;
    end else begin
      if (dq_push) begin
        dq_start[dq_wp] <= dq_wstart; dq_end[dq_wp] <= dq_wend; dq_tl[dq_wp] <= dq_wtl;
        dq_ring[dq_wp] <= dq_wring; dq_bank[dq_wp] <= dq_wbank; dq_last[dq_wp] <= dq_wlast;
        dq_wp <= !dq_wp;
      end
      if (dq_pop) dq_rp <= !dq_rp;
      dq_cnt <= dq_cnt + {1'b0, dq_push} - {1'b0, dq_pop};
    end
  end

endmodule
