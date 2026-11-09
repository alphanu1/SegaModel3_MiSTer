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
// Real3D rasteriser, the top level (block 9; D-20, D-28, D-29). The
// geometry's record stream in, a finished 3D layer in DDR3 out, bit for bit
// render_d25 (depth on 1/w, the normal's cull flag, fog, translucency 0):
//
//   records -> viewport records: r3d_rast_fogvp -> the fog table (K, S, fc)
//           -> polygon records:  r3d_rast_store (D-20: binned by band, DDR3)
//   per band, from the store: r3d_rast_intake -> the triangle -> r3d_rast_front
//     -> spans -> r3d_rast_zspan (1/w) -> r3d_rast_zband (Z, contour verdicts)
//     -> r3d_rast_interp (shade, u, v, fog's r and n; MIP: the level)
//     -> r3d_rast_fogp (P) beside the texel addresses (r3d_rast_texel, stages 1-2)
//     -> r3d_rast_tcache on the texture port
//     -> r3d_rast_texel (the filter) -> r3d_rast_colour (fog) -> r3d_rast_band
//        (D-29's blend, the flush into the layer)
//
// The chain from the intake to the band is the M2.5 viewer's (m3v_raster,
// R74), with fog, the generation flag and MIP's wires added. Per-triangle data
// sits in two M10K tables by the triangle's tag; per-pixel data waits in M10K
// rings by a 7-bit pixel number (study 8.4: storage out of flops).
//
// Fog. A viewport record's fog record goes to a 128-entry table at {bank,
// viewport number}; each polygon record carries its viewport's number (the
// store packs it into word 3). The table is read as the pixel leaves the
// interpolants (R to F1): K goes to fogp, which runs free on F1's held
// inputs, so its P is ready 3 cycles after the pixel reaches F1 whatever the
// cache does; S and fc go to ring D1 at F1, P to ring D2 at fogp's output.
// The colour stage reads both by pixel number. Scroll fog (the band's clear
// colour and what an undrawn pixel shows) is an input of the frame: the
// geometry's viewport record does not carry the rectangle, the priority or
// node word 0x24 that render_d25's rule needs.
//
// Frames. f_begin (when f_ready) starts a frame's records into a free store
// bank; f_end after its last record latches fb_base and the scroll fog for
// that bank. A full bank is replayed when the band stage is free: the band
// buffers cleared, the texture cache emptied, then band by band. A band ends
// when the store's reader is at its BANDEND and nothing is in flight before
// the band FIFO (the viewer's idle); the band stage then flushes it while the
// next band draws. done pulses when the last band's flush is in the layer,
// and the bank is released. Binning frame N + 1 runs beside frame N's replay.
//
// Memory ports:
//   store   m2_ddr3's request contract (D-20's switch, r3d_rast_store)
//   texture pipelined reads (m3_ddr3p's P port): t_req, t_addr held to t_ack,
//           four beats a line later, in order, on t_rvalid
//   layer   m2_ddr3's request contract, write bursts (r3d_rast_band)
// None of them is touched before the first f_begin, so a core gates DDR3 on
// hps_io and RESET (R71) by holding f_begin.

/* verilator lint_off UNUSEDSIGNAL */
module r3d_rast #(
  parameter bit MIP = 1'b0,         // nearest mip level (study D-33): a fifth header word
  parameter int SB  = 8,            // texture cache sets: 8 = 32 KiB, 9 = 64 KiB
  parameter int LB  = 19,           // store: log2 beats a list's share (study R136: 17 dropped Daytona 2's records)
  parameter bit PAIR = 1'b1,        // store: a pass's opaque and alpha lists share 2^(LB+1) beats (study R136)
  parameter logic [24:0] RUNO = 25'(1 << (LB + 4)),   // store: the run lists, words from store_base
  parameter int BB  = 12,           // store: log2 beats a bin's run list (study R136: 12)
  parameter bit FOV = 1'b1,         // study R131: a pair enters the front while the one before it walks
  parameter bit TSPLIT = 1'b1,      // study R131: ring A carries the texel's stage-2 banks and fractions
  parameter bit L8K = 1'b1,         // study R134: an L8 texel of 0xFF has alpha 0 (Supermodel); 0: MAME's decode
  // study R135: Supermodel's texture decode (format 2, contour on formats 1-4), contour's dilation and test,
  // texture alpha and translucency on textured polygons (r3d_rast_texel, r3d_rast_colour); 0: MAME's
  parameter bit SMTX = 1'b1
) (
  input  logic        clk,
  input  logic        rst,

  // ---- the geometry's records
  input  logic        i_valid,
  output logic        i_ready,
  input  logic [31:0] i_data,
  input  logic        i_last,
  output logic        f_ready,      // a store bank is free
  output logic        f_binning,    // a frame's records are being stored
  input  logic        f_begin,
  input  logic        f_end,
  input  logic [24:0] fb_base,      // latched at f_end: the layer this frame draws into
  input  logic [8:0]  sf_x0, sf_x1, sf_y0, sf_y1,
  input  logic [23:0] sf_col, sf_out,
  output logic        done,         // a pulse: a frame's layer is complete
  output logic        busy,         // a frame is being replayed

  input  logic [24:0] store_base,
  input  logic [24:0] tex_base,

  // ---- store (m2_ddr3 contract)
  output logic        s_req,
  output logic        s_we,
  output logic [24:0] s_addr,
  output logic [7:0]  s_blen,
  output logic [63:0] s_din,
  output logic [7:0]  s_be,
  input  logic        s_wacc,
  input  logic        s_rvalid,
  input  logic        s_ack,
  input  logic [63:0] s_dout,

  // ---- texture (pipelined reads)
  output logic        t_req,
  output logic [24:0] t_addr,
  input  logic        t_ack,
  input  logic        t_rvalid,
  input  logic [63:0] t_dout,

  // ---- layer (m2_ddr3 contract, write bursts)
  output logic        l_req,
  output logic [24:0] l_addr,
  output logic [7:0]  l_blen,
  output logic [63:0] l_din,
  input  logic        l_wacc,
  input  logic        l_ack,

  // ---- counts
  output logic [31:0] n_pix,        // pixels written to the band, all frames
  output logic [31:0] n_frames,
  output logic [31:0] f_cycles,     // the last frame's replay, cycles
  output logic [31:0] f_eobw,       // the last frame: cycles spent draining for ends of band
  output logic [15:0] st_rec, st_drop, st_run,
  output logic [17:0] st_pair,
  output logic [2:0]  st_ovf,
  output logic [15:0] st_novf,      // study R136: frames whose binning overflowed the store, since reset
  output logic        vp_ovf        // more than 64 viewports in a frame
);

  localparam logic [2:0] K_SOLID = 3'd1, K_SOLID_TRANS = 3'd2, K_TEX = 3'd3, K_TEX_CM = 3'd4,
                         K_CONTOUR = 3'd5, K_TEX_ALPHA = 3'd6;

  // ===========================================================================
  // records in: viewport records to the fog unit, polygon records to the store
  // ===========================================================================
  logic        rt_mid, rt_vp;              // inside a record; it is a viewport record
  wire         i_isvp = rt_mid ? rt_vp : (i_data[31:27] == 5'd0);
  logic        st_wready, fv_ready;
  assign i_ready = i_isvp ? fv_ready : st_wready;
  wire         i_take = i_valid && i_ready;
  logic [6:0]  vpn;                         // viewport records this frame
  logic [5:0]  cur_vp, fv_vp;
  logic        st_bbank;
  always_ff @(posedge clk) begin
    if (rst) begin rt_mid <= 1'b0; vpn <= '0; vp_ovf <= 1'b0; end
    else begin
      if (f_begin) begin vpn <= '0; vp_ovf <= 1'b0; end
      if (i_take) begin
        if (!rt_mid) begin
          rt_vp <= i_isvp;
          if (i_isvp) begin
            fv_vp <= vpn[5:0]; cur_vp <= vpn[5:0];
            if (vpn != 7'd64) vpn <= vpn + 7'd1; else vp_ovf <= 1'b1;
            if (vpn == 7'd64) begin fv_vp <= 6'd63; cur_vp <= 6'd63; end
          end
        end
        rt_mid <= !i_last;
      end
    end
  end

  // the fog unit and the table: {K 32, S 14, fc 24} at {bank, viewport}
  logic               fv_ov;
  logic        [31:0] fv_k;
  logic signed [13:0] fv_s;
  logic        [23:0] fv_fc;
  r3d_rast_fogvp u_fv (
    .clk, .rst, .i_valid(i_valid && i_isvp), .i_ready(fv_ready), .i_data, .i_last,
    .o_valid(fv_ov), .o_k(fv_k), .o_s(fv_s), .o_fc(fv_fc));
  (* ramstyle = "M10K, no_rw_check" *) logic [69:0] tfog [128];
  logic [6:0]  tf_ra;
  logic [69:0] tf_q;
  logic        en_f;
  always_ff @(posedge clk) begin
    if (fv_ov) tfog[{st_bbank, fv_vp}] <= {fv_k, fv_s, fv_fc};
    if (en_f) tf_q <= tfog[tf_ra];
  end

  // ===========================================================================
  // the store
  // ===========================================================================
  logic        st_pready, st_pgo, st_pbank, st_rel, st_pbusy;
  logic        r_valid, r_ready, r_last, r_new, st_eob, st_next;
  logic [31:0] r_data;
  logic [5:0]  band;
  r3d_rast_store #(.MIP(MIP), .LB(LB), .PAIR(PAIR), .RUNO(RUNO), .BB(BB)) u_store (
    .clk, .rst, .base(store_base),
    .b_ready(f_ready), .b_begin(f_begin), .b_end(f_end), .b_bank(st_bbank), .b_busy(f_binning),
    .w_valid(i_valid && !i_isvp), .w_ready(st_wready), .w_data(i_data), .w_last(i_last),
    .w_vp(cur_vp), .w_vpv(vpn != 7'd0),
    .p_ready(st_pready), .p_go(st_pgo), .p_bank(st_pbank), .p_release(st_rel),
    .o_valid(r_valid), .o_ready(r_ready), .o_data(r_data), .o_last(r_last), .o_new(r_new),
    .o_band(band), .o_eob(st_eob), .o_next(st_next), .p_busy(st_pbusy),
    .m_req(s_req), .m_we(s_we), .m_addr(s_addr), .m_blen(s_blen), .m_din(s_din), .m_be(s_be),
    .m_wacc(s_wacc), .m_rvalid(s_rvalid), .m_ack(s_ack), .m_dout(s_dout),
    .n_rec(st_rec), .n_pair(st_pair), .n_run(st_run), .n_drop(st_drop), .ovf(st_ovf), .n_ovff(st_novf));

  // the frame's layer and scroll fog, by bank
  logic [24:0] bk_fb [2];
  logic [8:0]  bk_x0 [2], bk_x1 [2], bk_y0 [2], bk_y1 [2];
  logic [23:0] bk_col [2], bk_out [2];
  always_ff @(posedge clk) if (f_end) begin
    bk_fb[st_bbank] <= fb_base;
    bk_x0[st_bbank] <= sf_x0; bk_x1[st_bbank] <= sf_x1; bk_y0[st_bbank] <= sf_y0; bk_y1[st_bbank] <= sf_y1;
    bk_col[st_bbank] <= sf_col; bk_out[st_bbank] <= sf_out;
  end

  // ===========================================================================
  // intake: the triangle gathered, then held until the front takes it
  // ===========================================================================
  logic        g_valid, g_ready, g_last;
  logic [4:0]  g_tag;
  logic [39:0] g_data;
  r3d_rast_intake #(.SETUP4(1'b1), .MIP(MIP)) u_in (
    .clk, .rst, .i_valid(r_valid), .i_ready(r_ready), .i_data(r_data), .i_last(r_last),
    .o_valid(g_valid), .o_ready(g_ready), .o_tag(g_tag), .o_data(g_data), .o_last(g_last));

  logic               t_full;
  logic        [2:0]  hk;
  logic        [31:0] th0, th1, th2, th3;
  logic        [20:0] th4;                  // MIP: {highest level, K}
  logic        [31:0] tv [6];
  logic signed [8:0]  pe [4];
  logic signed [39:0] pc [4], pdx [4], pdy [4];
  assign g_ready = !t_full;
  wire t_go;
  // the plane's slot: p, except an untextured triangle's shade (p = 1) into slot 3, where table 1's
  // shade field is read from for both kinds (th0 is the header's word 0, in before the planes)
  wire [1:0] g_pp = g_tag[3:2] - 2'd2;
  wire [1:0] g_ps = (!th0[4] && g_pp == 2'd1) ? 2'd3 : g_pp;

  always_ff @(posedge clk) begin
    if (rst) begin
      t_full <= 1'b0; hk <= 3'd0;
    end else begin
      if (g_valid && g_ready) begin
        if (g_tag == 5'd31) hk <= hk + 3'd1;
        if (g_last) begin t_full <= 1'b1; hk <= 3'd0; end
      end
      if (t_go) t_full <= 1'b0;
    end
  end
  // the record's words load outside the reset: the intake's o_valid is low through it, so its beats
  // cannot load (rst_rast into these ~1,000 enables was a failing family in three full fits)
`ifdef VERILATOR
  /* verilator lint_off PROCASSINIT */
  logic rst_q = 1'b0;
  /* verilator lint_on PROCASSINIT */
  always_ff @(posedge clk) begin
    rst_q <= rst;
    if (rst && rst_q && g_valid) $fatal(1, "rast: a record word in reset");
  end
`endif
  always_ff @(posedge clk) begin
    begin
      if (g_valid && g_ready) begin
        if (g_tag == 5'd31) begin                     // the header's beats
          if (hk == 3'd0) th0 <= g_data[31:0];
          // an untextured triangle's u and v planes go into table 1 as zeros: cleared here, before its
          // planes arrive (its shade plane, plane 1, is written into slot 3, the textured shade's)
          if (hk == 3'd0 && !g_data[4]) begin
            pe[1] <= '0; pc[1] <= '0; pdx[1] <= '0; pdy[1] <= '0;
            pe[2] <= '0; pc[2] <= '0; pdx[2] <= '0; pdy[2] <= '0;
          end
          if (hk == 3'd1) th1 <= g_data[31:0];
          if (hk == 3'd2) th2 <= g_data[31:0];
          if (hk == 3'd3) th3 <= g_data[31:0];
          if (hk == 3'd4) th4 <= g_data[20:0];
        end else if (g_tag <= 5'd5) tv[g_tag[2:0]] <= g_data[31:0];
        else if (g_tag >= 5'd8) begin                 // tag 8 + 4p + k: k 0 e, 1 c, 2 dx, 3 dy
          case (g_tag[1:0])
            2'd0: pe[g_ps] <= g_data[8:0];
            2'd1: pc[g_ps] <= g_data;
            2'd2: pdx[g_ps] <= g_data;
            default: pdy[g_ps] <= g_data;
          endcase
        end
      end
    end
  end

  // kind (r3d_rast.h kind_of): w0[20:15] = {colormod, contour, texture, mirror v, mirror u, page}
  wire       k_tex = th0[18], k_ctr = th0[19], k_cm = th0[20], t_alpha = th0[2];
  // study R135: the record's param bit 0 (th0[15]) is texture alpha alone, word 3 bit 14 (th3) texture
  // inversion; the colour's alpha mode {inversion, alpha test (contour, not format 7), texture alpha,
  // colour modulation}
  wire [3:0] t_am = {th3[14], k_ctr && th0[14:12] != 3'd7, th0[15], k_cm};
  wire [2:0] kind = k_tex ? (t_alpha ? K_TEX_ALPHA : k_ctr ? K_CONTOUR : k_cm ? K_TEX_CM : K_TEX)
                          : (t_alpha ? K_SOLID_TRANS : K_SOLID);
  wire [1:0] kind_zk = (kind == K_CONTOUR) ? 2'd2 :
                       (kind == K_SOLID || kind == K_TEX || kind == K_TEX_CM) ? 2'd1 : 2'd0;
  wire [5:0] t_vp = th3[13:8];
  wire [4:0] t_fogint = th3[4:0];
  wire       t_gen = th3[7];                        // the pass's first record: a new Z generation

  // ===========================================================================
  // front, a triangle at a time; the triangle tables
  // ===========================================================================
  logic        f_tready, f_busy, f_svalid, f_sready;
  logic [8:0]  f_sy, f_sx0, f_sx1;
  logic [7:0]  tag_n, cur_tag, f_stag, f_stag_n;
  logic [1:0]  cur_zk;
  logic signed [39:0] zc, zdx, zdy;
  logic signed [8:0]  ze;

  // FOV (study R131): the pair's tag goes with it through the front, and what the span path takes at
  // s_take (the Z plane, the Z kind) waits in table Z by that tag, so the next pair enters the divider
  // while this one walks. A pass's first record (a new Z generation) still waits for the front to empty,
  // so the generation lands on the first span taken after it, as before.
  assign t_go = t_full && f_tready && ((FOV && !t_gen) || (!f_busy && !f_svalid));

  // the triangle's words are in tv beats before the record ends and hold until t_go (PRE_DIV, study R109)
  r3d_rast_front #(.PRE_DIV(1'b1), .TGW(8)) u_front (
    .clk, .rst,
    .t_valid(t_go), .t_ready(f_tready),
    .t_x1(tv[0]), .t_y1(tv[1]), .t_x2(tv[2]), .t_y2(tv[3]), .t_x3(tv[4]), .t_y3(tv[5]),
    .t_b0({band, 3'd0}), .t_b1({band, 3'd0} + 9'd8), .t_tag(tag_n),
    .s_valid(f_svalid), .s_ready(f_sready), .s_y(f_sy), .s_x0(f_sx0), .s_x1(f_sx1),
    .s_tag(f_stag), .s_tag_n(f_stag_n),
    .busy(f_busy));

  // table Z: {Z kind, the 1/w plane} by tag, written at t_go, read registered at the tag the span
  // register will hold, so its word is the span's whenever the Z walk takes it
  (* ramstyle = "M10K, no_rw_check" *) logic [130:0] tabz [256];
  logic [130:0] tz_q;
  always_ff @(posedge clk) begin
    if (t_go) tabz[tag_n] <= {kind_zk, pc[0], pdx[0], pdy[0], pe[0]};
    tz_q <= tabz[f_stag_n];
  end
  always_comb begin
    {cur_zk, zc, zdx, zdy, ze} = tz_q;
    cur_tag = f_stag;
  end

  // table 1, read as the pixel enters the interpolants: shade, u, v (c 40,
  // dx and dy [30:0], e 9), the 1/w plane's e, textured; MIP the LOD terms
  localparam int LW  = MIP ? 21 : 1;       // (one constant bit without MIP: no zero-width field)
  localparam int T1W = 3 * 111 + 9 + 1 + LW;
  (* ramstyle = "M10K, no_rw_check" *) logic [T1W-1:0] tab1 [256];
  // table 2, read as the pixel leaves them: the texture's words {page, x / 32,
  // y / 32, w, h, mirror u, mirror v, format}, the colour's {kind,
  // transparency, colour}, fog's {viewport, intensity}; then (study R135) the
  // contour bit and the alpha mode (two M10K as before: 72 of 80 bits)
  localparam int T2W = 23 + 3 + 6 + 24 + 6 + 5 + 5;
  (* ramstyle = "M10K, no_rw_check" *) logic [T2W-1:0] tab2 [256];
  function automatic logic [110:0] pl(input logic [39:0] c, input logic [39:0] dx, input logic [39:0] dy, input logic [8:0] e);
    pl = {c, dx[30:0], dy[30:0], e};
  endfunction
  logic [7:0]     t1_ra, t2_ra;
  logic [T1W-1:0] t1_q, t1_wd;
  logic [T2W-1:0] t2_q;
  always_comb begin
    // the shade from slot 3; u and v from slots 1 and 2, zero for an untextured triangle (cleared at
    // its header), so no select is built here (it was 222 LUTs)
    t1_wd[T1W-1 -: 343] = {pl(pc[3], pdx[3], pdy[3], pe[3]),
                           pl(pc[1], pdx[1], pdy[1], pe[1]),
                           pl(pc[2], pdx[2], pdy[2], pe[2]),
                           pe[0], th0[4]};
    t1_wd[LW-1:0] = MIP ? LW'(th4) : '0;
  end
  always_ff @(posedge clk) begin
    if (t_go) begin
      tab1[tag_n] <= t1_wd;
      tab2[tag_n] <= {th0[5], th1[25:20], th1[30:26], th0[8:6], th0[11:9], th0[16], th0[17], th0[14:12],
                      kind, th0[26:21], th2[23:0], t_vp, t_fogint, k_ctr, t_am};
    end
    t1_q <= tab1[t1_ra];
    t2_q <= tab2[t2_ra];
  end

  logic gen_carry;                          // a new generation waits for the next span
  always_ff @(posedge clk) begin
    if (rst) tag_n <= 8'd0;
    else if (t_go) tag_n <= tag_n + 8'd1;
  end

  // ===========================================================================
  // the Z walk on the 1/w plane; the Z band
  // ===========================================================================
  logic        za_sready, za_pvalid, za_pready, za_plast;
  logic [8:0]  za_py, za_px;
  logic [1:0]  za_pon;
  logic [23:0] za_z0, za_z1;
  logic signed [41:0] za_v0, za_v1;
  wire  span_ok = f_sx0 < f_sx1;
  wire  tq_full;
  assign f_sready = (za_sready && !tq_full) || (f_svalid && !span_ok);
  wire  s_take = f_svalid && span_ok && za_sready && !tq_full;

  r3d_rast_zspan u_za (
    .clk, .rst,
    .s_valid(s_take), .s_ready(za_sready), .s_y(f_sy), .s_x0(f_sx0), .s_x1(f_sx1),
    .s_c(zc), .s_dx(zdx), .s_dy(zdy), .s_e(ze),
    .p_valid(za_pvalid), .p_ready(za_pready), .p_y(za_py), .p_x(za_px), .p_on(za_pon),
    .p_z0(za_z0), .p_z1(za_z1), .p_v0(za_v0), .p_v1(za_v1), .p_last(za_plast));

  // the span's {new generation, tag, Z kind}; an MLAB (R87's fit put it in an M10K)
  (* ramstyle = "MLAB, no_rw_check" *) logic [10:0] tq [8];
  logic [2:0]  tq_wp, tq_rp;
  logic [3:0]  tq_n;
  assign tq_full = tq_n >= 4'd7;
  wire  [10:0] tq_h = tq[tq_rp];
  logic        zs_ready, zfirst;
  // a skid between the Z walk and the Z band: the walk moves whenever the skid is
  // empty, so its p_ready comes from registers, not through the Z band's ready (its
  // scoreboard test fanned out into every enable of the walk: the -40C path after
  // R87's two were moved). The skid takes the pair whenever it is empty; it is
  // full when the Z band refused the pair offered.
  localparam int SKW = 1 + 9 + 9 + 2 + 24 + 24 + 42 + 42 + 2 + 8;
  wire         pair_v = za_pvalid && (tq_n != 4'd0);
  logic        sk_v;
  logic [SKW-1:0] sk_d;
  wire         pair_take = pair_v && !sk_v;
  assign za_pready = pair_take;
  wire  [SKW-1:0] up_d = {tq_h[10] && zfirst, za_py, za_px, za_pon, za_z0, za_z1, za_v0, za_v1, tq_h[9:8], tq_h[7:0]};
  wire  [SKW-1:0] zb_d = sk_v ? sk_d : up_d;
  wire         zb_v = sk_v || pair_v;
  logic        zb_new;
  logic [8:0]  zb_y, zb_x;
  logic [1:0]  zb_on, zb_zk;
  logic [23:0] zb_z0, zb_z1;
  logic signed [41:0] zb_v0, zb_v1;
  logic [7:0]  zb_tag;
  always_comb {zb_new, zb_y, zb_x, zb_on, zb_z0, zb_z1, zb_v0, zb_v1, zb_zk, zb_tag} = zb_d;
  always_ff @(posedge clk) begin
    if (rst) sk_v <= 1'b0;
    else sk_v <= zb_v && !zs_ready;
    if (!sk_v) sk_d <= up_d;
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      tq_wp <= '0; tq_rp <= '0; tq_n <= '0; gen_carry <= 1'b0; zfirst <= 1'b1;
    end else begin
      if (t_go && t_gen) gen_carry <= 1'b1;
      if (s_take) begin tq[tq_wp] <= {gen_carry, cur_zk, cur_tag}; tq_wp <= tq_wp + 3'd1; gen_carry <= 1'b0; end
      if (pair_take && za_plast) tq_rp <= tq_rp + 3'd1;
      if (pair_take) zfirst <= za_plast;
      tq_n <= tq_n + {3'd0, s_take} - {3'd0, pair_take && za_plast};
    end
  end

  logic        zo_valid, zo_ready, zo_ctr;
  logic [8:0]  zo_y, zo_x;
  logic [1:0]  zo_on;
  logic signed [41:0] zo_p0, zo_p1;
  logic [7:0]  zo_tag;
  logic        cv_valid, cv_zw;            // contour verdicts, from the colour stage
  r3d_rast_zband #(.OOW(1'b1), .TRANS(0), .PW(42), .TW(8)) u_zband (
    .clk, .rst,
    .s_valid(zb_v), .s_ready(zs_ready), .s_new(zb_new),
    .s_y(zb_y), .s_x(zb_x), .s_on(zb_on), .s_z0(zb_z0), .s_z1(zb_z1),
    .s_p0(zb_v0), .s_p1(zb_v1), .s_zk(zb_zk), .s_tag(zb_tag),
    .s_stip(1'b0), .s_trans(5'd0), .s_pinv(1'b0),
    .o_valid(zo_valid), .o_ready(zo_ready), .o_y(zo_y), .o_x(zo_x), .o_on(zo_on),
    .o_p0(zo_p0), .o_p1(zo_p1), .o_tag(zo_tag), .o_ctr(zo_ctr),
    .c_valid(cv_valid), .c_zw(cv_zw));

  // ===========================================================================
  // the front domain: pairs into pixels, the interpolants, fog's P, the texel
  // addresses; all on en_f
  // ===========================================================================
  // H, the pair in hand; it gives a pixel each en_f, lane 0 first
  logic        h_v, h_l, h_ctr;
  logic [8:0]  h_y, h_x;
  logic [1:0]  h_on;
  logic signed [41:0] h_p0, h_p1;
  logic [7:0]  h_tag;
  wire  h_last = h_l || !h_on[1];
  wire  h_go   = h_v && en_f;
  assign zo_ready = !h_v || (en_f && h_last);
  always_ff @(posedge clk) begin
    if (rst) h_v <= 1'b0;
    else if (zo_valid && zo_ready) begin
      h_v <= 1'b1; h_y <= zo_y; h_x <= zo_x; h_on <= zo_on; h_p0 <= zo_p0; h_p1 <= zo_p1;
      h_tag <= zo_tag; h_ctr <= zo_ctr; h_l <= !zo_on[0];
    end else if (h_go) begin
      if (h_last) h_v <= 1'b0; else h_l <= 1'b1;
    end
  end

  // P, the pixel; table 1 read alongside (the address held while P waits)
  logic        p_v, p_ctr;
  logic [8:0]  p_y, p_x;
  logic signed [41:0] p_o2;
  logic [7:0]  p_tag;
  assign t1_ra = en_f ? h_tag : p_tag;
  always_ff @(posedge clk) begin
    if (rst) p_v <= 1'b0;
    else if (en_f) begin
      p_v <= h_v;
      p_y <= h_y; p_x <= h_x + {8'd0, h_l}; p_o2 <= h_l ? h_p1 : h_p0; p_tag <= h_tag; p_ctr <= h_ctr;
    end
  end
  wire [110:0] t1_s = t1_q[T1W-1 -: 111], t1_u = t1_q[T1W-112 -: 111], t1_v = t1_q[T1W-223 -: 111];
  wire [8:0]   t1_eo = t1_q[LW+9 -: 9];
  wire         t1_tex = t1_q[LW];
  wire [16:0]  t1_lk  = MIP ? t1_q[16:0] : 17'd0;
  wire [3:0]   t1_lvm = MIP ? t1_q[20:17] : 4'd0;
  function automatic logic signed [39:0] sx31(input logic [30:0] d);
    sx31 = {{9{d[30]}}, d};
  endfunction

  logic        n_valid, n_tex, n_pos;
  logic [8:0]  n_tag;                       // {contour, triangle tag}
  logic [8:0]  n_x, n_y;
  logic [31:0] n_u, n_v, n_shade;
  logic [24:0] n_r;
  logic [5:0]  n_n;
  logic signed [8:0] n_eo;
  logic [3:0]  n_lv;
  r3d_rast_interp #(.TW(9), .MIP(MIP)) u_interp (
    .clk, .rst,
    .i_valid(p_v && en_f), .i_ready(), .i_tag({p_ctr, p_tag}), .i_x(p_x), .i_y(p_y),
    .i_o2(p_o2), .i_eo(t1_eo), .i_tex(t1_tex),
    .i_sc(t1_s[110:71]), .i_sdx(sx31(t1_s[70:40])), .i_sdy(sx31(t1_s[39:9])), .i_se(t1_s[8:0]),
    .i_uc(t1_u[110:71]), .i_udx(sx31(t1_u[70:40])), .i_udy(sx31(t1_u[39:9])), .i_ue(t1_u[8:0]),
    .i_vc(t1_v[110:71]), .i_vdx(sx31(t1_v[70:40])), .i_vdy(sx31(t1_v[39:9])), .i_ve(t1_v[8:0]),
    .i_lk(t1_lk), .i_lvm(t1_lvm),
    .o_valid(n_valid), .o_ready(en_f), .o_tag(n_tag), .o_x(n_x), .o_y(n_y), .o_tex(n_tex),
    .o_u(n_u), .o_v(n_v), .o_shade(n_shade), .o_r(n_r), .o_n(n_n), .o_eo(n_eo), .o_pos(n_pos), .o_lv(n_lv));

  // R, the pixel out of the interpolants; table 2 read alongside
  logic        r_v, r_ctr, r_tex, r_pos;
  logic [8:0]  r_x, r_y;
  logic [31:0] r_u, r_vv, r_shade;
  logic [7:0]  r_tag;
  logic [24:0] r_r;
  logic [5:0]  r_n;
  logic signed [8:0] r_eo;
  logic [3:0]  r_lv;
  assign t2_ra = en_f ? n_tag[7:0] : r_tag;
  always_ff @(posedge clk) begin
    if (rst) r_v <= 1'b0;
    else if (en_f) begin
      r_v <= n_valid;
      r_x <= n_x; r_y <= n_y; r_u <= n_u; r_vv <= n_v; r_shade <= n_shade;
      r_tag <= n_tag[7:0]; r_ctr <= n_tag[8]; r_tex <= n_tex;
      r_r <= n_r; r_n <= n_n; r_eo <= n_eo; r_pos <= n_pos; r_lv <= MIP ? n_lv : 4'd0;
    end
  end
  // table 2's fields
  wire        a_page = t2_q[71];
  wire [5:0]  a_tx = t2_q[70:65];
  wire [4:0]  a_ty = t2_q[64:60];
  wire [2:0]  a_tw = t2_q[59:57], a_th = t2_q[56:54];
  wire        a_mu = t2_q[53], a_mv = t2_q[52];
  wire [2:0]  a_fmt = t2_q[51:49];
  wire [2:0]  a_kind = t2_q[48:46];
  wire [5:0]  a_trans = t2_q[45:40];
  wire [23:0] a_col = t2_q[39:16];
  wire [5:0]  a_vp = t2_q[15:10];
  wire [4:0]  a_fogint = t2_q[9:5];
  wire        a_ctr = t2_q[4];
  wire [3:0]  a_am = t2_q[3:0];
  // the fog table, read R to F1 (on en_f, so it holds with the pixel)
  assign tf_ra = {st_pbank, a_vp};

  // the pixel rings, by pixel number, written as R moves on: A the texel's
  // inputs, B the colour's, C the band's
  logic [6:0] pn;                           // the next pixel number
  wire        r_go = en_f && r_v;
  localparam int LVW = 4;                  // the level (0 without MIP; ring A has the room)
  // ring A: the filter's inputs. TSPLIT: {banks, fu, fv, format, contour}, written at F2 from the address
  // instance and read at the filter's stage 1 (SMTX: going into stage 1); else {u, v, the texture's words,
  // contour, level} at R. Ring B: the colour's {kind, transparency, colour, shade, fog intensity, alpha mode}.
  localparam int RAW = TSPLIT ? 8 + 8 + 8 + 3 + 1 : 21 + 21 + 23 + 1 + LVW, RBW = 3 + 6 + 24 + 32 + 5 + 4, RCW = 9 + 9 + 1;
  (* ramstyle = "M10K, no_rw_check" *) logic [RAW-1:0] ring_a [128];
  (* ramstyle = "M10K, no_rw_check" *) logic [RBW-1:0] ring_b [128];
  (* ramstyle = "M10K, no_rw_check" *) logic [RCW-1:0] ring_c [128];
  logic [6:0]     ra_a, ra_b, ra_c;
  logic [RAW-1:0] rq_a, ra_wd;
  logic [RBW-1:0] rq_b, rq_bm;
  logic [6:0]  tb_ne;                      // the filter's next tag (o_tag a cycle early)
  logic [RCW-1:0] rq_c;
  logic [7:0]  tf_bk, tf_fu, tf_fv;
  logic [2:0]  tf_fmt;
  logic        tf_ctr;
  logic        f2_v;
  logic [6:0]  f2_n;
  generate if (TSPLIT) begin : g_raw
    always_comb ra_wd = {tf_bk, tf_fu, tf_fv, tf_fmt, tf_ctr};
    always_ff @(posedge clk) if (f2_v && en_f) ring_a[f2_n] <= ra_wd;
  end else begin : g_raw0
    always_comb begin
      ra_wd[RAW-1 -: 66] = {r_u[20:0], r_vv[20:0], a_page, a_tx, a_ty, a_tw, a_th, a_mu, a_mv, a_fmt, a_ctr};
      ra_wd[LVW-1:0] = r_lv;
    end
    always_ff @(posedge clk) if (r_go) ring_a[pn] <= ra_wd;
  end endgenerate
  always_ff @(posedge clk) begin
    if (r_go) begin
      ring_b[pn] <= {a_kind, a_trans, a_col, r_shade, a_fogint, a_am};
      ring_c[pn] <= {r_x, r_y, r_ctr && (a_kind == K_CONTOUR)};
    end
    rq_a <= ring_a[ra_a];
    // ring B is read a cycle early, at the filter's next tag, and registered again, as D1 and D2 are: the
    // M10K's output ran through the kind's decode and the 8 x 8 product into b1 (two full fits at 110 MHz).
    // The entry was written at R, long before, and is not rewritten until its number comes round.
    rq_bm <= ring_b[tb_ne];
    rq_b <= rq_bm;
    rq_c <= ring_c[ra_c];
  end

  // F: the texel path's address stages (its filter is not used)
  logic [19:0] fa0, fa1, fa2, fa3;
  r3d_rast_texel #(.TW(1), .MIP(MIP), .PART(TSPLIT ? 1 : 0), .L8K(L8K), .SMTX(SMTX)) u_tf (
    .clk, .rst, .en(en_f),
    .i_valid(r_v), .i_tag(1'b0), .i_u(r_u), .i_v(r_vv), .i_page(a_page), .i_tx(a_tx), .i_ty(a_ty),
    .i_tw(a_tw), .i_th(a_th), .i_mu(a_mu), .i_mv(a_mv), .i_fmt(a_fmt), .i_ctr(a_ctr), .i_lv(r_lv),
    .m_a0(fa0), .m_a1(fa1), .m_a2(fa2), .m_a3(fa3), .m_re(),
    .m_q0(16'd0), .m_q1(16'd0), .m_q2(16'd0), .m_q3(16'd0),
    .o2_bk(tf_bk), .o2_fu(tf_fu), .o2_fv(tf_fv), .o2_fmt(tf_fmt), .o2_ctr(tf_ctr), .o_tagr(),
    .i3_bk(8'd0), .i3_fu(8'd0), .i3_fv(8'd0), .i3_fmt(3'd0), .i3_ctr(1'b0), .i1_f(20'd0),
    .o_valid(), .o_tag(), .o_tag_e(), .o_texel());
  // its two stages' valid, pixel number and texture flag; F1 also holds fog's inputs
  logic       f1_v, f1_t, f2_t, f1_pos;
  logic [6:0] f1_n;
  logic [24:0] f1_r;
  logic [5:0]  f1_nn;
  logic signed [8:0] f1_eo;
  always_ff @(posedge clk) begin
    if (rst) begin f1_v <= 1'b0; f2_v <= 1'b0; pn <= '0; end
    else if (en_f) begin
      f1_v <= r_v; f1_n <= pn; f1_t <= r_tex;
      f1_r <= r_r; f1_nn <= r_n; f1_eo <= r_eo; f1_pos <= r_pos;
      f2_v <= f1_v; f2_n <= f1_n; f2_t <= f1_t;
      if (r_v) pn <= pn + 7'd1;
    end
  end

  // fog's P: free running on F1's held inputs, three stages; ring D2 by the
  // pixel number carried alongside. S and fc to ring D1 while the pixel is in F1.
  logic [12:0] fp_p;
  r3d_rast_fogp u_fogp (
    .clk, .en(1'b1), .i_r(f1_r), .i_n(f1_nn), .i_eo(f1_eo), .i_pos(f1_pos), .i_k(tf_q[69:38]), .o_p(fp_p));
  logic       fd1_v, fd2_v, fd3_v;
  logic [6:0] fd1_n, fd2_n, fd3_n;
  always_ff @(posedge clk) begin
    if (rst) begin fd1_v <= 1'b0; fd2_v <= 1'b0; fd3_v <= 1'b0; end
    else begin
      fd1_v <= f1_v; fd2_v <= fd1_v; fd3_v <= fd2_v;
    end
    fd1_n <= f1_n; fd2_n <= fd1_n; fd3_n <= fd2_n;
  end
  (* ramstyle = "M10K, no_rw_check" *) logic [37:0] ring_d1 [128];
  // ring_d2 in M10K, old data on a read of the entry written (study R98: -40 ALM
  // of MLAB, +1 M10K)
  (* ramstyle = "M10K" *) logic [12:0] ring_d2 [128];
  // D1 and D2 are read a cycle early, at the filter's next tag (tb_ne), and registered again, so the
  // M10Ks' outputs do not run into the colour stage's fog sum (timing, study R104). The entries were
  // written long before (at the texel fetch), so the earlier read sees the same words.
  logic [37:0] rq_d1, rq_d1m;
  logic [12:0] rq_d2, rq_d2m;
  always_ff @(posedge clk) begin
    if (f1_v) ring_d1[f1_n] <= tf_q[37:0];
    if (fd3_v) ring_d2[fd3_n] <= fp_p;
    rq_d1m <= ring_d1[tb_ne];
    rq_d2m <= ring_d2[tb_ne];
    rq_d1 <= rq_d1m;
    rq_d2 <= rq_d2m;
  end

  // ===========================================================================
  // the texture cache
  // ===========================================================================
  logic        tc_iready /*verilator public_flat_rd*/, tc_ovalid, tc_oready;
  logic [15:0] tq0, tq1, tq2, tq3;
  logic [6:0]  tc_pay;
  logic        tc_freq, inval;
  logic [17:0] tc_faddr;
  assign en_f = !f2_v || tc_iready;
  r3d_rast_tcache #(.SB(SB), .WAYS(4), .DB(6), .FB(3), .PW(7)) u_tc (
    .clk, .rst, .inval,
    .i_valid(f2_v), .i_ready(tc_iready), .i_a0(fa0), .i_a1(fa1), .i_a2(fa2), .i_a3(fa3),
    .i_use(f2_t ? 4'b1111 : 4'b0000), .i_pay(f2_n),
    .o_valid(tc_ovalid), .o_ready(tc_oready), .o_q0(tq0), .o_q1(tq1), .o_q2(tq2), .o_q3(tq3), .o_pay(tc_pay),
    .f_req(tc_freq), .f_addr(tc_faddr), .f_ack(t_ack), .f_rvalid(t_rvalid), .f_rdata(t_dout));
  assign t_req  = tc_freq;
  assign t_addr = tex_base + {5'd0, tc_faddr, 2'b00};

  // ===========================================================================
  // the back domain: never stalls
  // ===========================================================================
  wire tc_pop = tc_ovalid && tc_oready;
  logic [6:0] tb_n2;                       // the filter's stage-1 pixel (TSPLIT: ring A's read, registered into its stage 3)
  assign ra_a = TSPLIT ? tb_n2 : tc_pay;
  // the cache's words wait four cycles for the filter's stage 3 (an MLAB line,
  // read synchronously: the word written four cycles ago, registered; in M10K,
  // study R98: -40 ALM of MLAB, +2 M10K; the read is never at the written address)
  (* ramstyle = "M10K, no_rw_check" *) logic [63:0] wdl [8];
  logic [2:0]  wdp;
  // (read a cycle earlier and registered once more, so the M10K's output does not run into the
  // filter's decode and differences: timing, study R104)
  logic [63:0] wq, wq_m;
  always_ff @(posedge clk) begin
    wdl[wdp] <= {tq3, tq2, tq1, tq0};
    wdp <= wdp + 3'd1;
    wq_m <= wdl[3'(wdp - 3'd2)];
    wq <= wq_m;
  end
  // study R135: each word's {bit 15, nibbles all ones}, for the filter's dilation selects in its stage 2:
  // a copy of the line's flags read a cycle earlier (the word the filter's stage 1 holds), so its stage 2
  // forms the selects from registers. 8 x 20 bits.
  logic [19:0] wf_m;
  generate if (TSPLIT && SMTX) begin : g_wdf
    function automatic logic [4:0] flw(input logic [15:0] w);
      flw = {w[15], &w[15:12], &w[11:8], &w[7:4], &w[3:0]};
    endfunction
    (* ramstyle = "MLAB, no_rw_check" *) logic [19:0] wdf [8];   // the read is never at the written address
    always_ff @(posedge clk) begin
      wdf[wdp] <= {flw(tq3), flw(tq2), flw(tq1), flw(tq0)};
      wf_m <= wdf[3'(wdp - 3'd1)];
    end
  end else begin : g_nowdf
    assign wf_m = '0;
  end endgenerate

  logic       b_in_v;
  logic [6:0] b_in_n;
  always_ff @(posedge clk) begin
    if (rst) b_in_v <= 1'b0;
    else b_in_v <= tc_pop;
    b_in_n <= tc_pay;
  end
  logic [20:0] b_u, b_v;
  logic [22:0] b_t;
  logic        b_ctr;
  logic [3:0]  b_lv;
  generate if (TSPLIT) begin : g_rqa
    always_comb begin b_u = '0; b_v = '0; b_t = '0; b_ctr = 1'b0; b_lv = '0; end    // the filter takes stage 3 from ring A
  end else begin : g_rqa0
    always_comb begin
      b_u = rq_a[RAW-1 -: 21]; b_v = rq_a[RAW-22 -: 21];
      b_t = rq_a[RAW-43 -: 23];
      b_ctr = rq_a[LVW];
      b_lv = MIP ? rq_a[3:0] : 4'd0;
    end
  end endgenerate
  logic        tb_valid;
  logic [6:0]  tb_n;
  logic [31:0] tb_texel;
  r3d_rast_texel #(.TW(7), .MIP(MIP), .PART(TSPLIT ? 2 : 0), .L8K(L8K), .SMTX(SMTX)) u_tb (
    .clk, .rst, .en(1'b1),
    .i_valid(b_in_v), .i_tag(b_in_n), .i_u({11'd0, b_u}), .i_v({11'd0, b_v}),
    .i_page(b_t[22]), .i_tx(b_t[21:16]), .i_ty(b_t[15:11]), .i_tw(b_t[10:8]), .i_th(b_t[7:5]),
    .i_mu(b_t[4]), .i_mv(b_t[3]), .i_fmt(b_t[2:0]), .i_ctr(b_ctr), .i_lv(b_lv),
    .m_a0(), .m_a1(), .m_a2(), .m_a3(), .m_re(),
    .m_q0(wq[15:0]), .m_q1(wq[31:16]), .m_q2(wq[47:32]), .m_q3(wq[63:48]),
    .o2_bk(), .o2_fu(), .o2_fv(), .o2_fmt(), .o2_ctr(), .o_tagr(tb_n2),
    .i3_bk(rq_a[RAW-1 -: 8]), .i3_fu(rq_a[RAW-9 -: 8]), .i3_fv(rq_a[RAW-17 -: 8]), .i3_fmt(rq_a[3:1]),
    .i3_ctr(rq_a[0]), .i1_f(wf_m),
    .o_valid(tb_valid), .o_tag(tb_n), .o_tag_e(tb_ne), .o_texel(tb_texel));

  // the colour's inputs from rings B, D1 and D2, a cycle on
  assign ra_b = tb_n;
  logic        cb_v;
  logic [6:0]  cb_n;
  logic [31:0] cb_texel;
  always_ff @(posedge clk) begin
    if (rst) cb_v <= 1'b0; else cb_v <= tb_valid;
    cb_n <= tb_n; cb_texel <= tb_texel;
  end
`ifdef VERILATOR
  logic [RBW-1:0] rq_bc;
  always_ff @(posedge clk) begin
    rq_bc <= ring_b[ra_b];
    if (!rst && cb_v && rq_b != rq_bc) $fatal(1, "rast: ring B read early");
  end
`endif
  logic        co_v, co_ok, co_zw, co_fbv;
  logic [6:0]  co_n;
  logic [7:0]  co_fb;
  logic [23:0] co_rgb;
  r3d_rast_colour #(.TW(7), .SMTX(SMTX)) u_col (
    .clk, .rst, .en(1'b1),
    .i_valid(cb_v), .i_tag(cb_n), .i_kind(rq_b[RBW-1 -: 3]), .i_texel(cb_texel),
    .i_color(rq_b[64:41]), .i_trans(rq_b[70:65]), .i_shade(rq_b[40:9]),
    .i_fogint(rq_b[8:4]), .i_am(rq_b[3:0]), .i_fc(rq_d1[23:0]), .i_fs({{4{rq_d1[37]}}, rq_d1[37:24]}), .i_fp(rq_d2),
    .o_valid(co_v), .o_tag(co_n), .o_ok(co_ok), .o_zw(co_zw), .o_fbv(co_fbv), .o_fb(co_fb), .o_rgb(co_rgb));

  // ring C a cycle on: the pixel's place; the contour verdict
  assign ra_c = co_n;
  logic        cc_v, cc_ok, cc_zw, cc_fbv;
  logic [7:0]  cc_fb;
  logic [23:0] cc_rgb;
  always_ff @(posedge clk) begin
    if (rst) cc_v <= 1'b0; else cc_v <= co_v;
    cc_ok <= co_ok; cc_zw <= co_zw; cc_fbv <= co_fbv; cc_fb <= co_fb; cc_rgb <= co_rgb;
  end
  wire [8:0] cc_x = rq_c[18:10], cc_y = rq_c[9:1];
  assign cv_valid = cc_v && rq_c[0];
  assign cv_zw    = cc_ok && cc_zw;

  // ===========================================================================
  // the band FIFO, and the band stage
  // ===========================================================================
  // {eob, y, x, blend, factor, rgb}; written by the back domain, and by eob
  logic eob;
  localparam int BW = 1 + 9 + 9 + 1 + 8 + 24;
  (* ramstyle = "MLAB, no_rw_check" *) logic [BW-1:0] bq [32];
  logic [5:0] bq_wp, bq_rp;
  logic [5:0] bk_n;                        // pixels between the cache's output and the FIFO
  wire  [5:0] bq_n = bq_wp - bq_rp;
  wire        bq_push = (cc_v && cc_ok) || eob;
  // registered (the count's compare into the cache's read was a worst path):
  // pixels stop at 27 in all, ends of band at 31
  logic tc_ok;
  always_ff @(posedge clk) if (rst) tc_ok <= 1'b0; else tc_ok <= (6'(bq_n) + bk_n) < 6'd26;
  assign tc_oready = tc_ok;
  logic       bd_ready;
  wire        bq_pop = (bq_n != '0) && bd_ready;
  logic [5:0] eob_band;
  wire  [BW-1:0] bq_wd = eob ? {1'b1, eob_band, 3'd0, 9'd0, 1'b0, 8'd0, 24'd0} : {1'b0, cc_y, cc_x, cc_fbv, cc_fb, cc_rgb};
  // the next head's index chosen from two registers (study R109; the add after the band's ready ran into
  // the MLAB's read and bq_h)
  logic [5:0]    bq_rp1;                   // bq_rp + 1
  wire  [5:0]    bq_rn = bq_pop ? bq_rp1 : bq_rp;
  // the write's bypass test for both heads, ahead of the band's ready
  wire           bq_eq0 = bq_wp[4:0] == bq_rp[4:0], bq_eq1 = bq_wp[4:0] == bq_rp1[4:0];
  logic [BW-1:0] bq_h;
  always_ff @(posedge clk) begin
    if (bq_push) bq[bq_wp[4:0]] <= bq_wd;
    bq_h <= (bq_push && (bq_pop ? bq_eq1 : bq_eq0)) ? bq_wd : bq[bq_rn[4:0]];
  end
  always_ff @(posedge clk) begin
    if (rst) begin bq_wp <= '0; bq_rp <= '0; bq_rp1 <= 6'd1; bk_n <= '0; n_pix <= '0; end
    else begin
      if (bq_push) bq_wp <= bq_wp + 6'd1;
      if (bq_pop) begin bq_rp <= bq_rp1; bq_rp1 <= bq_rp1 + 6'd1; end
      bk_n <= bk_n + 6'(tc_pop) - 6'(cc_v);
      if (cc_v && cc_ok) n_pix <= n_pix + 32'd1;
    end
  end

  logic bd_busy, f_start;
  r3d_rast_band u_band (
    .clk, .rst,
    .f_start, .fb_base(bk_fb[st_pbank]),
    .sf_x0(bk_x0[st_pbank]), .sf_x1(bk_x1[st_pbank]), .sf_y0(bk_y0[st_pbank]), .sf_y1(bk_y1[st_pbank]),
    .sf_col(bk_col[st_pbank]), .sf_out(bk_out[st_pbank]),
    .busy(bd_busy),
    .p_valid(bq_n != '0), .p_ready(bd_ready), .p_eob(bq_h[BW-1]), .p_band(bq_h[BW-2 -: 6]),
    .p_x(bq_h[41:33]), .p_row(bq_h[BW-8 -: 3]), .p_rgb(bq_h[23:0]), .p_bl(bq_h[32]), .p_f(bq_h[31:24]),
    .m_req(l_req), .m_addr(l_addr), .m_blen(l_blen), .m_din(l_din), .m_wacc(l_wacc), .m_ack(l_ack));

  // ===========================================================================
  // idle: nothing in flight from the store's reader to the band FIFO, for
  // fifteen cycles. The interpolants (17 cycles from P to n_valid) and the
  // texture cache (a fill's latency) hold pixels with no valid showing, so
  // each has a count of the pixels inside it: the viewer's quiet window alone
  // ended band 2 of VF3 frame 1800 with a pixel still in the interpolants.
  // ===========================================================================
  logic [3:0] quiet /* verilator public_flat_rd */;
  logic [5:0] ip_n;                        // pixels inside the interpolants
  logic [7:0] tc_n;                        // pixels inside the texture cache
  always_ff @(posedge clk) begin
    if (rst) begin ip_n <= '0; tc_n <= '0; end
    else begin
      ip_n <= ip_n + 6'(p_v && en_f) - 6'(n_valid && en_f);
      tc_n <= tc_n + 8'(f2_v && tc_iready) - 8'(tc_pop);
    end
  end
  wire busy_now = (ip_n != '0) || (tc_n != '0) || r_valid || g_valid || t_full || f_busy || f_svalid || za_pvalid ||
                  (tq_n != 4'd0) || sk_v || !zs_ready || zo_valid || h_v || p_v || r_v || f1_v || f2_v ||
                  n_valid || !tc_iready || tc_ovalid || (bk_n != '0) || b_in_v || tb_valid || cb_v || co_v || cc_v;
  always_ff @(posedge clk) begin
    if (rst) quiet <= 4'd0;
    else if (busy_now) quiet <= 4'd0;
    else if (quiet != 4'd15) quiet <= quiet + 4'd1;
  end
  wire idle = (quiet >= 4'd15);
  // for the bench's stall counts (no logic: nothing reads them)
  wire dbg_cache /* verilator public_flat_rd */ = f2_v && !tc_iready;                 // the cache refuses a pixel
  wire dbg_back  /* verilator public_flat_rd */ = tc_ovalid && !tc_oready;            // the band FIFO is full
  wire dbg_store /* verilator public_flat_rd */ = (q == Q_RUN) && !st_eob && !r_valid && r_ready; // the intake waits for the store
  wire dbg_zb    /* verilator public_flat_rd */ = zb_v && !zs_ready;                  // the Z band refuses a pair
  wire dbg_px    /* verilator public_flat_rd */ = f2_v && tc_iready;                  // a pixel into the cache
  wire dbg_run   /* verilator public_flat_rd */ = (q == Q_RUN) || (q == Q_EOB);

  // ===========================================================================
  // the replay sequencer
  // ===========================================================================
  typedef enum logic [2:0] { Q_IDLE, Q_CLR, Q_RUN, Q_EOB, Q_FLUSH } qst_t;
  qst_t q;
  logic [31:0] cyc, eobw;
  always_ff @(posedge clk) begin
    if (rst) begin
      q <= Q_IDLE; f_start <= 1'b0; inval <= 1'b0; eob <= 1'b0; st_next <= 1'b0; st_pgo <= 1'b0; st_rel <= 1'b0;
      done <= 1'b0; n_frames <= '0; f_cycles <= '0; f_eobw <= '0;
    end else begin
      f_start <= 1'b0; inval <= 1'b0; eob <= 1'b0; st_next <= 1'b0; st_pgo <= 1'b0; st_rel <= 1'b0; done <= 1'b0;
      cyc <= cyc + 32'd1;
      case (q)
        Q_IDLE: if (st_pready && !bd_busy && !st_pgo) begin
          st_pgo <= 1'b1; f_start <= 1'b1; inval <= 1'b1; cyc <= '0; eobw <= '0;
          q <= Q_CLR;
        end
        Q_CLR: if (!bd_busy && !f_start) q <= Q_RUN;
        Q_RUN: if (st_eob) begin
          // an end of band takes a FIFO entry: pixels stop at 27 (tc_oready), ends at 31,
          // so the 32 entries never wrap (empty bands end faster than a flush runs)
          if (idle && bq_n < 6'd31) begin eob <= 1'b1; eob_band <= band; st_next <= 1'b1; q <= Q_EOB; end
          else eobw <= eobw + 32'd1;
        end
        Q_EOB: q <= (eob_band == 6'd47) ? Q_FLUSH : Q_RUN;   // st_next lands; o_eob falls
        Q_FLUSH: if (!bd_busy && !eob && bq_n == '0) begin
          done <= 1'b1; st_rel <= 1'b1; n_frames <= n_frames + 32'd1; f_cycles <= cyc; f_eobw <= eobw;
          q <= Q_IDLE;
        end
        default: q <= Q_IDLE;
      endcase
    end
  end
  assign busy = (q != Q_IDLE);

endmodule
