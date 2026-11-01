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
// M2.5 viewer: the 3D layer, band by band, from the geometry's set-up records
// (SETUP4 with the LOD word: a 5-word header, r3d_geo SETUP_OUT and LOD_WORD,
// study R75, R76), by the rasteriser's blocks (study R66, R67) with textures
// from DDR3 (D-33) at the nearest mip level:
//
//   record -> r3d_rast_intake -> the triangle (header, vertices, four planes)
//          -> r3d_rast_front -> spans -> r3d_rast_zspan on the 1/w plane
//          -> r3d_rast_zband (Z test; contour Z writes wait for the colour's
//             alpha verdict) -> pixels one a clock
//          -> r3d_rast_interp (shade, u, v, and the mip level, MIP = 1:
//             render_d25 mip = 1, lod = 1)
//          -> texel addresses (r3d_rast_texel, first instance, stages 1-2, at
//             the pixel's level)
//          -> r3d_rast_tcache (32 KiB) on m3_ddr3p's pipelined read port
//          -> r3d_rast_texel (second instance, fed by the cache, the filter)
//          -> r3d_rast_colour (fog off) -> FIFO -> r3d_rast_band (the two
//             band buffers, D-29's blend, the flush into the DDR3 layer on
//             m2_ddr3's request contract, D-20)
//
// Per-triangle data goes into two M10K tables by the triangle's tag when the
// front takes it: the planes the interpolants read (one read, as the pixel
// enters them), and the texture and colour words (read as the pixel leaves
// them). Per-pixel data from there to the band waits in three M10K rings by a
// 7-bit pixel number, which is all that rides through the cache and the two
// pipelines behind it (study 8.4: storage out of flops, built once and shared).
//
// Why two texel instances: the cache sits between the texel path's address
// stages and its filter, and the block has no port there. The first instance
// gives the four bank addresses (its filter is unused and pruned); the second
// recomputes the addresses it needs for corner selection and takes the cache's
// words four cycles after the cache gives them, when its own stage 3 holds the
// pixel. The cost is the address stages once more; a split port in the block
// would remove it.
//
// Timing domains: the front (from the zband's output to the cache's input)
// moves on en_f, which is the cache taking a pixel or having none offered; the
// back (cache output to the band FIFO) never stalls: the cache's output is
// taken only while the band FIFO has room for every pixel in flight behind it.

/* verilator lint_off UNUSEDSIGNAL */
module m3v_raster (
  input  logic        clk,
  input  logic        rst,

  // set-up records (m3v_store's words 1.., the geometry's record), and the band
  input  logic        r_valid,
  output logic        r_ready,
  input  logic [31:0] r_data,
  input  logic        r_last,
  input  logic [5:0]  band,
  input  logic        new_gen,      // pulse: the next pair starts a Z generation
  output logic        idle,         // nothing in flight before the band FIFO

  // the frame and the bands
  input  logic        f_start,      // a frame: the band buffers cleared, the texture cache emptied
  input  logic        eob,          // pulse: band `band` is complete (after idle)
  input  logic [24:0] fb_base,      // 64-bit word address of the layer being drawn
  output logic        busy,         // the band stage clears, flushes or holds pixels

  // the layer (m2_ddr3 contract, write bursts)
  output logic        m_req,
  output logic [24:0] m_addr,
  output logic [7:0]  m_blen,
  output logic [63:0] m_din,
  input  logic        m_wacc,
  input  logic        m_ack,

  // texture RAM in DDR3, tiled (a line of 4 x 4 texels is 4 beats, row k beat k)
  input  logic [24:0] tex_base,
  output logic        t_req,
  output logic [24:0] t_addr,
  input  logic        t_ack,
  input  logic        t_rvalid,
  input  logic [63:0] t_dout,

  output logic [31:0] n_pix          // pixels written to the band
);

  localparam logic [2:0] K_SOLID = 3'd1, K_SOLID_TRANS = 3'd2, K_TEX = 3'd3, K_TEX_CM = 3'd4,
                         K_CONTOUR = 3'd5, K_TEX_ALPHA = 3'd6;

  // ===========================================================================
  // intake: the triangle gathered, then held until the front takes it
  // ===========================================================================
  logic        i_valid, i_ready, i_last;
  logic [4:0]  i_tag;
  logic [39:0] i_data;
  r3d_rast_intake #(.SETUP4(1'b1), .MIP(1'b1)) u_in (
    .clk, .rst, .i_valid(r_valid), .i_ready(r_ready), .i_data(r_data), .i_last(r_last),
    .o_valid(i_valid), .o_ready(i_ready), .o_tag(i_tag), .o_data(i_data), .o_last(i_last));

  logic               t_full;
  logic        [2:0]  hk;
  logic        [31:0] th0, th1, th2;
  logic        [20:0] th4;            // the fifth header beat: the LOD terms {highest level, K} (MIP)
  logic        [31:0] tv [6];
  logic signed [8:0]  pe [4];
  logic signed [39:0] pc [4], pdx [4], pdy [4];
  assign i_ready = !t_full;
  wire t_go;

  always_ff @(posedge clk) begin
    if (rst) begin
      t_full <= 1'b0; hk <= 3'd0;
    end else begin
      if (i_valid && i_ready) begin
        if (i_tag == 5'd31) begin                     // the header's five beats (beat 3 unused)
          if (hk == 3'd0) th0 <= i_data[31:0];
          if (hk == 3'd1) th1 <= i_data[31:0];
          if (hk == 3'd2) th2 <= i_data[31:0];
          if (hk == 3'd4) th4 <= i_data[20:0];
          hk <= hk + 3'd1;
        end else if (i_tag <= 5'd5) tv[i_tag[2:0]] <= i_data[31:0];
        else if (i_tag >= 5'd8) begin                 // tag 8 + 4p + k: k 0 e, 1 c, 2 dx, 3 dy
          case (i_tag[1:0])
            2'd0: pe[i_tag[3:2] - 2'd2] <= i_data[8:0];
            2'd1: pc[i_tag[3:2] - 2'd2] <= i_data;
            2'd2: pdx[i_tag[3:2] - 2'd2] <= i_data;
            default: pdy[i_tag[3:2] - 2'd2] <= i_data;
          endcase
        end
        if (i_last) begin t_full <= 1'b1; hk <= 3'd0; end
      end
      if (t_go) t_full <= 1'b0;
    end
  end

  // kind (r3d_rast.h kind_of): w0[20:15] = {colormod, contour, texture, mirror v, mirror u, page}
  wire       k_tex = th0[18], k_ctr = th0[19], k_cm = th0[20], t_alpha = th0[2];
  wire [2:0] kind = k_tex ? (t_alpha ? K_TEX_ALPHA : k_ctr ? K_CONTOUR : k_cm ? K_TEX_CM : K_TEX)
                          : (t_alpha ? K_SOLID_TRANS : K_SOLID);
  // Z: written (solid, texture, colour-modulated), by the alpha verdict (contour), or not
  wire [1:0] kind_zk = (kind == K_CONTOUR) ? 2'd2 :
                       (kind == K_SOLID || kind == K_TEX || kind == K_TEX_CM) ? 2'd1 : 2'd0;
  // planes: 0 1/w; textured 1 u/w, 2 v/w, 3 shade; untextured 1 shade
  wire [1:0] sp = th0[4] ? 2'd3 : 2'd1;

  // ===========================================================================
  // front, a triangle at a time; the triangle tables
  // ===========================================================================
  logic        f_tready, f_busy, f_svalid, f_sready;
  logic [8:0]  f_sy, f_sx0, f_sx1;
  logic [7:0]  tag_n, cur_tag;
  logic [1:0]  cur_zk;
  logic signed [39:0] zc, zdx, zdy;
  logic signed [8:0]  ze;

  assign t_go = t_full && f_tready && !f_busy && !f_svalid;

  r3d_rast_front u_front (
    .clk, .rst,
    .t_valid(t_go), .t_ready(f_tready),
    .t_x1(tv[0]), .t_y1(tv[1]), .t_x2(tv[2]), .t_y2(tv[3]), .t_x3(tv[4]), .t_y3(tv[5]),
    .t_b0({band, 3'd0}), .t_b1({band, 3'd0} + 9'd8),
    .s_valid(f_svalid), .s_ready(f_sready), .s_y(f_sy), .s_x0(f_sx0), .s_x1(f_sx1),
    .busy(f_busy));

  // table 1, the planes the interpolants read: shade, u, v (c 40, dx and dy
  // [30:0], the bits r3d_rast_interp uses, e 9), the 1/w plane's e, textured,
  // and the LOD terms (21 bits: 343 + 21 bits is ten M10K, not nine)
  localparam int T1W = 3 * 111 + 9 + 1 + 21;
  (* ramstyle = "M10K, no_rw_check" *) logic [T1W-1:0] tab1 [256];
  // table 2, read as the pixel leaves the interpolants: the texture's words
  // {page, x / 32, y / 32, w, h, mirror u, mirror v, format} and the colour's
  // {kind, transparency, colour}
  localparam int T2W = 23 + 3 + 6 + 24;
  (* ramstyle = "M10K, no_rw_check" *) logic [T2W-1:0] tab2 [256];
  function automatic logic [110:0] pl(input logic [39:0] c, input logic [39:0] dx, input logic [39:0] dy, input logic [8:0] e);
    pl = {c, dx[30:0], dy[30:0], e};
  endfunction
  logic [7:0]     t1_ra, t2_ra;
  logic [T1W-1:0] t1_q;
  logic [T2W-1:0] t2_q;
  always_ff @(posedge clk) begin
    if (t_go) begin
      tab1[tag_n] <= {pl(pc[sp], pdx[sp], pdy[sp], pe[sp]),
                      th0[4] ? pl(pc[1], pdx[1], pdy[1], pe[1]) : 111'd0,
                      th0[4] ? pl(pc[2], pdx[2], pdy[2], pe[2]) : 111'd0,
                      pe[0], th0[4], th4};
      tab2[tag_n] <= {th0[5], th1[25:20], th1[30:26], th0[8:6], th0[11:9], th0[16], th0[17], th0[14:12],
                      kind, th0[26:21], th2[23:0]};
    end
    t1_q <= tab1[t1_ra];
    t2_q <= tab2[t2_ra];
  end

  always_ff @(posedge clk) begin
    if (rst) tag_n <= 8'd0;
    else if (t_go) begin
      tag_n <= tag_n + 8'd1;
      cur_tag <= tag_n; cur_zk <= kind_zk;
      zc <= pc[0]; zdx <= pdx[0]; zdy <= pdy[0]; ze <= pe[0];
    end
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

  // the span's {tag, Z kind}
  logic [9:0] tq [8];
  logic [2:0] tq_wp, tq_rp;
  logic [3:0] tq_n;
  assign tq_full = tq_n >= 4'd7;
  wire  [9:0] tq_h = tq[tq_rp];
  logic       zs_ready;
  wire        pair_v = za_pvalid && (tq_n != 4'd0);
  wire        pair_take = pair_v && zs_ready;
  assign za_pready = pair_take;
  always_ff @(posedge clk) begin
    if (rst) begin
      tq_wp <= '0; tq_rp <= '0; tq_n <= '0;
    end else begin
      if (s_take) begin tq[tq_wp] <= {cur_zk, cur_tag}; tq_wp <= tq_wp + 3'd1; end
      if (pair_take && za_plast) tq_rp <= tq_rp + 3'd1;
      tq_n <= tq_n + {3'd0, s_take} - {3'd0, pair_take && za_plast};
    end
  end

  logic gen_pend;
  always_ff @(posedge clk) begin
    if (rst) gen_pend <= 1'b0;
    else if (new_gen) gen_pend <= 1'b1;
    else if (pair_take) gen_pend <= 1'b0;
  end

  logic        zo_valid, zo_ready, zo_ctr;
  logic [8:0]  zo_y, zo_x;
  logic [1:0]  zo_on;
  logic signed [41:0] zo_p0, zo_p1;
  logic [7:0]  zo_tag;
  logic        cv_valid, cv_zw;            // contour verdicts, from the colour stage
  r3d_rast_zband #(.OOW(1'b1), .TRANS(0), .PW(42), .TW(8)) u_zband (
    .clk, .rst,
    .s_valid(pair_v), .s_ready(zs_ready), .s_new(gen_pend),
    .s_y(za_py), .s_x(za_px), .s_on(za_pon), .s_z0(za_z0), .s_z1(za_z1),
    .s_p0(za_v0), .s_p1(za_v1), .s_zk(tq_h[9:8]), .s_tag(tq_h[7:0]),
    .s_stip(1'b0), .s_trans(5'd0), .s_pinv(1'b0),
    .o_valid(zo_valid), .o_ready(zo_ready), .o_y(zo_y), .o_x(zo_x), .o_on(zo_on),
    .o_p0(zo_p0), .o_p1(zo_p1), .o_tag(zo_tag), .o_ctr(zo_ctr),
    .c_valid(cv_valid), .c_zw(cv_zw));

  // ===========================================================================
  // the front domain: pairs into pixels, the interpolants, the texel addresses
  // ===========================================================================
  logic en_f;

  // H, the pair in hand; it gives a pixel each en_f, lane 0 first
  logic        h_v, h_l, h_ctr;
  logic [8:0]  h_y, h_x;
  logic [1:0]  h_on;
  logic signed [41:0] h_p0, h_p1;
  logic [7:0]  h_tag;
  wire  h_last = h_l || !h_on[1];          // the lane in hand is the pair's last
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
  wire [8:0]   t1_eo = t1_q[30:22];
  wire         t1_tex = t1_q[21];
  wire [3:0]   t1_lvm = t1_q[20:17];
  wire [16:0]  t1_lk = t1_q[16:0];
  function automatic logic signed [39:0] sx31(input logic [30:0] d);
    sx31 = {{9{d[30]}}, d};
  endfunction

  logic        n_valid, n_tex;
  logic [8:0]  n_tag;                       // {contour, triangle tag}
  logic [8:0]  n_x, n_y;
  logic [31:0] n_u, n_v, n_shade;
  logic [3:0]  n_lv;
  r3d_rast_interp #(.TW(9), .MIP(1'b1)) u_interp (
    .clk, .rst,
    .i_valid(p_v && en_f), .i_ready(), .i_tag({p_ctr, p_tag}), .i_x(p_x), .i_y(p_y),
    .i_o2(p_o2), .i_eo(t1_eo), .i_tex(t1_tex),
    .i_sc(t1_s[110:71]), .i_sdx(sx31(t1_s[70:40])), .i_sdy(sx31(t1_s[39:9])), .i_se(t1_s[8:0]),
    .i_uc(t1_u[110:71]), .i_udx(sx31(t1_u[70:40])), .i_udy(sx31(t1_u[39:9])), .i_ue(t1_u[8:0]),
    .i_vc(t1_v[110:71]), .i_vdx(sx31(t1_v[70:40])), .i_vdy(sx31(t1_v[39:9])), .i_ve(t1_v[8:0]),
    .i_lk(t1_lk), .i_lvm(t1_lvm),
    .o_valid(n_valid), .o_ready(en_f), .o_tag(n_tag), .o_x(n_x), .o_y(n_y), .o_tex(n_tex),
    .o_u(n_u), .o_v(n_v), .o_shade(n_shade), .o_r(), .o_n(), .o_eo(), .o_pos(), .o_lv(n_lv));

  // R, the pixel out of the interpolants; table 2 read alongside
  logic        r_v, r_ctr, r_tex;
  logic [8:0]  r_x, r_y;
  logic [31:0] r_u, r_vv, r_shade;
  logic [3:0]  r_lv;
  logic [7:0]  r_tag;
  assign t2_ra = en_f ? n_tag[7:0] : r_tag;
  always_ff @(posedge clk) begin
    if (rst) r_v <= 1'b0;
    else if (en_f) begin
      r_v <= n_valid;
      r_x <= n_x; r_y <= n_y; r_u <= n_u; r_vv <= n_v; r_shade <= n_shade; r_lv <= n_lv;
      r_tag <= n_tag[7:0]; r_ctr <= n_tag[8]; r_tex <= n_tex;
    end
  end
  // table 2's fields
  wire        a_page = t2_q[55];
  wire [5:0]  a_tx = t2_q[54:49];
  wire [4:0]  a_ty = t2_q[48:44];
  wire [2:0]  a_tw = t2_q[43:41], a_th = t2_q[40:38];
  wire        a_mu = t2_q[37], a_mv = t2_q[36];
  wire [2:0]  a_fmt = t2_q[35:33];
  wire [2:0]  a_kind = t2_q[32:30];
  wire [5:0]  a_trans = t2_q[29:24];
  wire [23:0] a_col = t2_q[23:0];

  // the pixel rings, by pixel number, written as R moves on: A the texel's
  // inputs (with MIP the level and u, v [27:0], the bits r3d_rast_texel's mip
  // form reads: 83 bits, three M10K, not two), B the colour's, C the band's
  logic [6:0] pn;                           // the next pixel number
  wire        r_go = en_f && r_v;
  localparam int RAW = 4 + 28 + 28 + 23, RBW = 3 + 6 + 24 + 32, RCW = 9 + 9 + 1;
  (* ramstyle = "M10K, no_rw_check" *) logic [RAW-1:0] ring_a [128];
  (* ramstyle = "M10K, no_rw_check" *) logic [RBW-1:0] ring_b [128];
  (* ramstyle = "M10K, no_rw_check" *) logic [RCW-1:0] ring_c [128];
  logic [6:0]     ra_a, ra_b, ra_c;
  logic [RAW-1:0] rq_a;
  logic [RBW-1:0] rq_b;
  logic [RCW-1:0] rq_c;
  always_ff @(posedge clk) begin
    if (r_go) begin
      ring_a[pn] <= {r_lv, r_u[27:0], r_vv[27:0], a_page, a_tx, a_ty, a_tw, a_th, a_mu, a_mv, a_fmt};
      ring_b[pn] <= {a_kind, a_trans, a_col, r_shade};
      ring_c[pn] <= {r_x, r_y, r_ctr && (a_kind == K_CONTOUR)};
    end
    rq_a <= ring_a[ra_a];
    rq_b <= ring_b[ra_b];
    rq_c <= ring_c[ra_c];
  end

  // F: the texel path's address stages (its filter is not used)
  logic [19:0] fa0, fa1, fa2, fa3;
  r3d_rast_texel #(.TW(1), .MIP(1'b1)) u_tf (
    .clk, .rst, .en(en_f),
    .i_valid(r_v), .i_tag(1'b0), .i_u(r_u), .i_v(r_vv), .i_page(a_page), .i_tx(a_tx), .i_ty(a_ty),
    .i_tw(a_tw), .i_th(a_th), .i_mu(a_mu), .i_mv(a_mv), .i_fmt(a_fmt), .i_lv(r_lv),
    .m_a0(fa0), .m_a1(fa1), .m_a2(fa2), .m_a3(fa3), .m_re(),
    .m_q0(16'd0), .m_q1(16'd0), .m_q2(16'd0), .m_q3(16'd0),
    .o_valid(), .o_tag(), .o_texel());
  // its two stages' valid, pixel number and texture flag, alongside
  logic       f1_v, f2_v, f1_t, f2_t;
  logic [6:0] f1_n, f2_n;
  always_ff @(posedge clk) begin
    if (rst) begin f1_v <= 1'b0; f2_v <= 1'b0; pn <= '0; end
    else if (en_f) begin
      f1_v <= r_v; f1_n <= pn; f1_t <= r_tex;
      f2_v <= f1_v; f2_n <= f1_n; f2_t <= f1_t;
      if (r_v) pn <= pn + 7'd1;
    end
  end

  // ===========================================================================
  // the texture cache
  // ===========================================================================
  logic        tc_iready, tc_ovalid, tc_oready;
  logic [15:0] tq0, tq1, tq2, tq3;
  logic [6:0]  tc_pay;
  logic        tc_freq, inval;
  logic [17:0] tc_faddr;
  assign en_f = !f2_v || tc_iready;
  r3d_rast_tcache #(.SB(8), .WAYS(4), .DB(6), .FB(3), .PW(7)) u_tc (
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
  assign ra_a = tc_pay;
  // the cache's words wait four cycles for B's stage 3 (an MLAB line, read
  // synchronously: the word written four cycles ago, registered)
  (* ramstyle = "MLAB, no_rw_check" *) logic [63:0] wdl [8];
  logic [2:0]  wdp;
  logic [63:0] wq;
  always_ff @(posedge clk) begin
    wdl[wdp] <= {tq3, tq2, tq1, tq0};
    wdp <= wdp + 3'd1;
    wq <= wdl[3'(wdp - 3'd3)];
  end

  logic       b_in_v;
  logic [6:0] b_in_n;
  always_ff @(posedge clk) begin
    if (rst) b_in_v <= 1'b0;
    else b_in_v <= tc_pop;
    b_in_n <= tc_pay;
  end
  wire [3:0]  b_lv = rq_a[RAW-1 -: 4];
  wire [27:0] b_u = rq_a[RAW-5 -: 28], b_v = rq_a[RAW-33 -: 28];
  logic        tb_valid;
  logic [6:0]  tb_n;
  logic [31:0] tb_texel;
  r3d_rast_texel #(.TW(7), .MIP(1'b1)) u_tb (
    .clk, .rst, .en(1'b1),
    .i_valid(b_in_v), .i_tag(b_in_n), .i_u({4'd0, b_u}), .i_v({4'd0, b_v}),
    .i_page(rq_a[22]), .i_tx(rq_a[21:16]), .i_ty(rq_a[15:11]), .i_tw(rq_a[10:8]), .i_th(rq_a[7:5]),
    .i_mu(rq_a[4]), .i_mv(rq_a[3]), .i_fmt(rq_a[2:0]), .i_lv(b_lv),
    .m_a0(), .m_a1(), .m_a2(), .m_a3(), .m_re(),
    .m_q0(wq[15:0]), .m_q1(wq[31:16]), .m_q2(wq[47:32]), .m_q3(wq[63:48]),
    .o_valid(tb_valid), .o_tag(tb_n), .o_texel(tb_texel));

  // the colour's inputs from ring B, a cycle on
  assign ra_b = tb_n;
  logic        cb_v;
  logic [6:0]  cb_n;
  logic [31:0] cb_texel;
  always_ff @(posedge clk) begin
    if (rst) cb_v <= 1'b0; else cb_v <= tb_valid;
    cb_n <= tb_n; cb_texel <= tb_texel;
  end
  logic        co_v, co_ok, co_zw, co_fbv;
  logic [6:0]  co_n;
  logic [7:0]  co_fb;
  logic [23:0] co_rgb;
  r3d_rast_colour #(.TW(7)) u_col (
    .clk, .rst, .en(1'b1),
    .i_valid(cb_v), .i_tag(cb_n), .i_kind(rq_b[RBW-1 -: 3]), .i_texel(cb_texel),
    .i_color(rq_b[55:32]), .i_trans(rq_b[61:56]), .i_shade(rq_b[31:0]),
    .i_fogint(5'd0), .i_fc(24'd0), .i_fs(18'sd0), .i_fp(13'd0),
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
  localparam int BW = 1 + 9 + 9 + 1 + 8 + 24;
  (* ramstyle = "MLAB, no_rw_check" *) logic [BW-1:0] bq [32];
  logic [5:0] bq_wp, bq_rp;
  logic [5:0] bk_n;                        // pixels between the cache's output and the FIFO
  wire  [5:0] bq_n = bq_wp - bq_rp;
  wire        bq_push = (cc_v && cc_ok) || eob;
  assign tc_oready = (6'(bq_n) + bk_n) < 6'd28;
  logic       bd_ready;
  wire        bq_pop = (bq_n != '0) && bd_ready;
  // the head read synchronously, a cycle ahead, the write forwarded (an
  // asynchronous MLAB read is not trusted on the board: the DDR3 probe core)
  wire  [BW-1:0] bq_wd = eob ? {1'b1, band, 3'd0, 9'd0, 1'b0, 8'd0, 24'd0} : {1'b0, cc_y, cc_x, cc_fbv, cc_fb, cc_rgb};
  wire  [5:0]    bq_rn = bq_rp + 6'(bq_pop);
  logic [BW-1:0] bq_h;
  always_ff @(posedge clk) begin
    if (bq_push) bq[bq_wp[4:0]] <= bq_wd;
    bq_h <= (bq_push && bq_wp[4:0] == bq_rn[4:0]) ? bq_wd : bq[bq_rn[4:0]];
  end
  always_ff @(posedge clk) begin
    if (rst) begin bq_wp <= '0; bq_rp <= '0; bk_n <= '0; n_pix <= '0; end
    else begin
      if (bq_push) bq_wp <= bq_wp + 6'd1;
      if (bq_pop) bq_rp <= bq_rp + 6'd1;
      bk_n <= bk_n + 6'(tc_pop) - 6'(cc_v);
      if (cc_v && cc_ok) n_pix <= n_pix + 32'd1;
    end
  end

  logic bd_busy;
  r3d_rast_band u_band (
    .clk, .rst,
    .f_start, .fb_base,
    .sf_x0(9'd0), .sf_x1(9'd0), .sf_y0(9'd0), .sf_y1(9'd0), .sf_col(24'd0), .sf_out(24'd0),
    .busy(bd_busy),
    .p_valid(bq_n != '0), .p_ready(bd_ready), .p_eob(bq_h[BW-1]), .p_band(bq_h[BW-2 -: 6]),
    .p_x(bq_h[41:33]), .p_row(bq_h[BW-8 -: 3]), .p_rgb(bq_h[23:0]), .p_bl(bq_h[32]), .p_f(bq_h[31:24]),
    .m_req, .m_addr, .m_blen, .m_din, .m_wacc, .m_ack);
  assign busy = bd_busy || (bq_n != '0);

  // a frame empties the texture cache (its texture RAM is another frame's)
  always_ff @(posedge clk) inval <= f_start;

  // ===========================================================================
  // idle: nothing in flight from the intake to the band FIFO, for eight cycles
  // ===========================================================================
  logic [3:0] quiet;
  wire busy_now = r_valid || i_valid || t_full || f_busy || f_svalid || za_pvalid ||
                  (tq_n != 4'd0) || !zs_ready || zo_valid || h_v || p_v || r_v || f1_v || f2_v ||
                  n_valid || !tc_iready || tc_ovalid || (bk_n != '0) || b_in_v || tb_valid || cb_v || co_v || cc_v;
  always_ff @(posedge clk) begin
    if (rst) quiet <= 4'd0;
    else if (busy_now) quiet <= 4'd0;
    else if (quiet != 4'd15) quiet <= quiet + 4'd1;
  end
  assign idle = (quiet >= 4'd15);

endmodule
