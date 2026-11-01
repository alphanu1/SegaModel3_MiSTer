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
// M2.5 viewer (study D-31): one Real3D frame from a Supermodel dump, drawn by
// the Real3D RTL with no CPU, and shown. Everything on one 100 MHz clock
// (study D-28). Everything below the MiSTer framework, so the Verilator bench
// runs this module whole.
//
//   ioctl -> m3v_loader -> SDRAM (VROM, culling RAM, texture RAM; D-4)
//                       -> DDR3 (polygon RAM; D-6)
//   r3d_geo, its three read ports on m2_sdram (culling, VROM) and DDR3 (polygon),
//     with the set-up on its pool (SETUP_OUT, study R76) and the LOD word
//     (LOD_WORD, R75): one set-up record a triangle
//     -> m3v_store (TEMPORARY polygon store, DDR3)
//     -> per 8-row band: m3v_raster (nearest mip level) -> the 3D layer in DDR3
//   m2_video_timing + m2_fb_read: the layer scanned out at 496 x 384, 57.52 Hz
//   m3v_overlay: status in hex
//
// SDRAM (byte addresses): VROM at 0 (geometry word 0x800000 up; below it reads
// zero, as MAME's image), frame slot k at 0x2000000 + 0x1000000 k: 8C at +0,
// 8E at +0x400000, texture RAM at +0x500000.
// DDR3 (64-bit words from DDR_BASE): the 3D layer's two buffers at 0 and
// 0x20000 (512 pixels of 32 bits a row), polygon RAM slot k at 0x100000 +
// 0x80000 k, the store's lists at 0x800000.
//
// The frame is drawn into the buffer not shown, then the two swap at the next
// vertical blank. With `loop` it is drawn again and again (the picture does
// not change; the overlay's counters do).

module m3v_core #(
  parameter int          COL_BITS = 10,
  parameter bit          DUAL     = 1'b1,         // the 128 MB module (two chips, one select)
  parameter logic [28:0] DDR_BASE = 29'h0600_0000, // byte 0x30000000, the cores' DDR3 (Model 2 R654)
  parameter bit          PIPE     = 1'b1,         // m3_ddr3p: reads in flight (0: one at a time, as m2_ddr3)
  parameter bit          PROBE    = 1'b1          // the DDR3 probe (D-33, D-34)
) (
  input  logic        clk,
  input  logic        mem_rst_n,     // memory: out of reset on PLL lock, and stays out
  input  logic        rst,           // the viewer: reset, downloads

  // ioctl (hps_io WIDE)
  input  logic        ioctl_download,
  input  logic [15:0] ioctl_index,
  input  logic        ioctl_wr,
  input  logic [26:0] ioctl_addr,
  input  logic [15:0] ioctl_dout,
  output logic        ioctl_wait,

  // SDRAM
  input  logic [2:0]  rd_lat_sel,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i,

  // DDR3
  output logic        DDRAM_CLK,
  input  logic        DDRAM_BUSY,
  output logic [7:0]  DDRAM_BURSTCNT,
  output logic [28:0] DDRAM_ADDR,
  output logic [63:0] DDRAM_DIN,
  output logic [7:0]  DDRAM_BE,
  output logic        DDRAM_WE,
  output logic        DDRAM_RD,
  input  logic [63:0] DDRAM_DOUT,
  input  logic        DDRAM_DOUT_READY,

  // video
  output logic        ce_pix,
  output logic        v_hs, v_vs, v_hblank, v_vblank,
  output logic [7:0]  v_r, v_g, v_b,

  // controls (pulses, levels)
  input  logic        next_frame,
  input  logic        redraw,
  input  logic        loop,
  input  logic        overlay_en,
  input  logic        probe_en,
  input  logic        probe_pause,   // the probe's case (a): scan-out paused while idle
  input  logic        ddr_ok,        // Main has finished the load: DDR3 may be touched (m3_viewer.sv)
  output logic        drawing,

  // for the bench
  output logic [31:0] frames_done,
  output logic        fb_show_o
);

  localparam int AW = (DUAL ? 1 : 0) + 2 + 13 + COL_BITS;   // 26: 128 MB
  localparam logic [24:0] FB1 = 25'h02_0000, POLY = 25'h10_0000, LISTS = 25'h80_0000, TEX = 25'hC0_0000;

  // ===========================================================================
  // SDRAM: the loader's write port, the culling and VROM read ports
  // ===========================================================================
  logic          mem_ready;
  logic          ld_wr_req, ld_wr_ack;
  logic [AW:1]   ld_wr_addr;
  logic [15:0]   ld_wr_din;
  logic [1:0]    ld_wr_be;
  logic [1:0]    p_req, p_lo, p_ack;
  logic [1:0][AW:1] p_addr;
  logic [1:0][63:0] p_dout;

  m2_sdram #(.COL_BITS(COL_BITS), .NP(2), .T_REFI(770), .ACK_HOLD(1), .DUAL(DUAL), .MASK_ON_A(DUAL)) u_sdram (
    .clk, .rst_n(mem_rst_n), .ready(mem_ready), .rd_lat_sel,
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm,
    .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(ld_wr_req), .wr_addr(ld_wr_addr), .wr_din(ld_wr_din), .wr_be(ld_wr_be), .wr_ack(ld_wr_ack),
    .p_req, .p_we(2'b00), .p_addr, .p_din('0), .p_be('0), .p_dout, .p_ack,
    .p_long(2'b11), .p_lo,
    .dbg_req(), .dbg_grant());

  // ===========================================================================
  // DDR3: m3_ddr3p. Port L (m2_ddr3's contract, D-20) behind the arbiter's six
  // masters; pipelined read port 0 the probe's
  // ===========================================================================
  // 0 scan-out reader, 1 loader, 2 polygon RAM reads, 3 store, 4 band flush,
  // 5 the probe's pattern writes
  localparam int NDP = 6;
  localparam logic [24:0] PROBE_BASE = 25'h120_0000;
  logic [NDP-1:0]       d_req, d_we, d_wacc, d_rvalid, d_ack;
  logic [NDP-1:0][24:0] d_addr;
  logic [NDP-1:0][7:0]  d_blen;
  logic [NDP-1:0][63:0] d_din;
  logic        dm_req, dm_we, dm_wacc, dm_rvalid, dm_ack, dm_wnext;
  logic [24:0] dm_addr;
  logic [7:0]  dm_blen, dm_be;
  logic [63:0] dm_din, dm_dout;
  logic [15:0] ddr_lat_max;
  logic [5:0]  ddr_acc, ddr_acc_max;
  logic        ddr_lat_v;
  logic [15:0] ddr_lat;
  logic [3:0]  ddr_lat_src;
  // pipelined read ports: 0 the probe, 1 the texture cache's fills
  logic [1:0]       pp_req, pp_ack, pp_rvalid;
  logic [1:0][24:0] pp_addr;
  logic [1:0][7:0]  pp_blen;

  m3v_ddr_arb #(.NP(NDP)) u_darb (
    .clk, .rst(!mem_rst_n),
    .req(d_req), .we(d_we), .addr(d_addr), .blen(d_blen), .din(d_din),
    .wacc(d_wacc), .rvalid(d_rvalid), .ack(d_ack),
    .m_req(dm_req), .m_we(dm_we), .m_addr(dm_addr), .m_blen(dm_blen), .m_din(dm_din), .m_be(dm_be),
    .m_wacc(dm_wacc), .m_rvalid(dm_rvalid), .m_ack(dm_ack));

  m3_ddr3p #(.PIPE(PIPE), .NP(2)) u_ddr3 (
    .clk, .rst_n(mem_rst_n), .base(DDR_BASE),
    .req(dm_req), .we(dm_we), .addr(dm_addr), .blen(dm_blen), .din(dm_din), .be(dm_be),
    .wnext(dm_wnext), .wacc(dm_wacc), .rvalid(dm_rvalid), .ack(dm_ack),
    .p_req(pp_req), .p_addr(pp_addr), .p_blen(pp_blen), .p_we(2'b00), .p_din('0), .p_ack(pp_ack), .p_rvalid(pp_rvalid), .p_wacc(),
    .dout(dm_dout),
    .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .dbg_lat_v(ddr_lat_v), .dbg_lat(ddr_lat), .dbg_lat_src(ddr_lat_src), .dbg_lat_max(ddr_lat_max),
    .dbg_acc(ddr_acc), .dbg_acc_max(ddr_acc_max),
    .dbg_take(), .dbg_take_src(), .dbg_done(), .dbg_done_src(), .dbg_idle(), .dbg_st(), .dbg_rdy(), .dbg_rq());

  // the probe: pattern writes on arbiter port 5, reads on the pipelined port
  logic        so_off;
  logic [4:0]  pr_idx;
  logic [31:0] pr_val;
  generate if (PROBE) begin : g_probe
    m3v_probe #(.RBASE(PROBE_BASE), .SRC(4'd1)) u_probe (
      .clk, .rst, .en(probe_en && ddr_ok), .pause(probe_pause), .drawing, .so_off,
      .w_req(d_req[5]), .w_addr(d_addr[5]), .w_blen(d_blen[5]), .w_din(d_din[5]), .w_wacc(d_wacc[5]), .w_ack(d_ack[5]),
      .p_req(pp_req[0]), .p_addr(pp_addr[0]), .p_blen(pp_blen[0]), .p_ack(pp_ack[0]), .p_rvalid(pp_rvalid[0]), .p_dout(dm_dout),
      .lat_v(ddr_lat_v), .lat(ddr_lat), .lat_src(ddr_lat_src), .lat_max(ddr_lat_max), .acc(ddr_acc),
      .rd_idx(pr_idx), .rd_val(pr_val), .busy());
  end else begin : g_noprobe
    assign so_off = 1'b0; assign pr_val = '0;
    assign d_req[5] = 1'b0; assign d_addr[5] = '0; assign d_blen[5] = 8'd1; assign d_din[5] = '0;
    assign pp_req[0] = 1'b0; assign pp_addr[0] = '0; assign pp_blen[0] = 8'd1;
  end endgenerate
  assign d_we[5] = 1'b1;

  // ===========================================================================
  // loader
  // ===========================================================================
  logic        vrom_loaded, frames_loaded, ld_ovf;
  logic [2:0]  n_frames;
  m3v_loader #(.AW(AW)) u_ld (
    .clk, .rst(!mem_rst_n), .mem_ready(mem_ready && ddr_ok),   // DDR3 (and so the loader) only once Main has released the port
    .ioctl_download, .ioctl_index, .ioctl_wr, .ioctl_addr, .ioctl_dout, .ioctl_wait,
    .poly_base(POLY), .tex_base(TEX),
    .sdr_wr_req(ld_wr_req), .sdr_wr_addr(ld_wr_addr), .sdr_wr_din(ld_wr_din), .sdr_wr_be(ld_wr_be), .sdr_wr_ack(ld_wr_ack),
    .m_req(d_req[1]), .m_addr(d_addr[1]), .m_din(d_din[1]), .m_ack(d_ack[1]),
    .vrom_loaded, .frames_loaded, .n_frames, .overflow(ld_ovf));
  assign d_we[1] = 1'b1;
  assign d_blen[1] = 8'd1;

  // ===========================================================================
  // geometry and its memory ports
  // ===========================================================================
  logic [2:0]  slot;
  logic        geo_start, geo_busy /* verilator public_flat_rd */;
  logic [7:0]  geo_err;
  logic        cr_req, cr_ack, cr_rv, pr_req, pr_ack, pr_rv, vr_req, vr_ack, vr_rv;
  logic [23:0] cr_addr, pr_addr, vr_addr;
  logic [2:0]  cr_len /* verilator public_flat_rd */, pr_len /* verilator public_flat_rd */, vr_len /* verilator public_flat_rd */;
  logic [31:0] cr_rd, pr_rd, vr_rd;
  logic        g_valid /* verilator public_flat_rd */, g_ready /* verilator public_flat_rd */, g_last /* verilator public_flat_rd */;
  logic [31:0] g_data /* verilator public_flat_rd */;

  r3d_geo #(.SETUP_OUT(1'b1), .LOD_WORD(1'b1)) u_geo (
    .clk, .rst, .start(geo_start), .step15(1'b0), .busy(geo_busy), .err(geo_err),
    .cull_req(cr_req), .cull_addr(cr_addr), .cull_len(cr_len), .cull_ack(cr_ack), .cull_rvalid(cr_rv), .cull_rdata(cr_rd),
    .pram_req(pr_req), .pram_addr(pr_addr), .pram_len(pr_len), .pram_ack(pr_ack), .pram_rvalid(pr_rv), .pram_rdata(pr_rd),
    .vrom_req(vr_req), .vrom_addr(vr_addr), .vrom_len(vr_len), .vrom_ack(vr_ack), .vrom_rvalid(vr_rv), .vrom_rdata(vr_rd),
    .out_valid(g_valid), .out_ready(g_ready), .out_data(g_data), .out_last(g_last));

  // culling RAM: 8C words 0..0xFFFFF, 8E words 0x800000..0x83FFFF
  logic [23:0] cb_addr, vb_addr;
  logic [AW:1] cb_sdr, vb_sdr;
  logic        cb_zero, vb_zero;
  wire  [AW:1] slot_w = AW'(26'h100_0000) + AW'({slot, 23'd0});   // SLOT(k) / 2
  always_comb begin
    cb_zero = 1'b0;
    if (cb_addr < 24'h10_0000) cb_sdr = slot_w + AW'({cb_addr, 1'b0});
    else if (cb_addr >= 24'h80_0000 && cb_addr < 24'h84_0000)
      cb_sdr = slot_w + AW'(26'h20_0000) + AW'({cb_addr - 24'h80_0000, 1'b0});
    else begin cb_sdr = '0; cb_zero = 1'b1; end
    vb_zero = !vb_addr[23];
    vb_sdr  = AW'({vb_addr[22:0], 1'b0});
  end
  logic [15:0] n_cull, n_vrom, n_pram;
  m3v_sdr_rd #(.AW(AW)) u_crd (
    .clk, .rst,
    .g_req(cr_req), .g_addr(cr_addr), .g_len(cr_len), .g_ack(cr_ack), .g_rvalid(cr_rv), .g_rdata(cr_rd),
    .blk_addr(cb_addr), .blk_sdr(cb_sdr), .blk_zero(cb_zero),
    .p_req(p_req[0]), .p_addr(p_addr[0]), .p_lo(p_lo[0]), .p_ack(p_ack[0]), .p_dout(p_dout[0]), .n_req(n_cull));
  m3v_sdr_rd #(.AW(AW)) u_vrd (
    .clk, .rst,
    .g_req(vr_req), .g_addr(vr_addr), .g_len(vr_len), .g_ack(vr_ack), .g_rvalid(vr_rv), .g_rdata(vr_rd),
    .blk_addr(vb_addr), .blk_sdr(vb_sdr), .blk_zero(vb_zero),
    .p_req(p_req[1]), .p_addr(p_addr[1]), .p_lo(p_lo[1]), .p_ack(p_ack[1]), .p_dout(p_dout[1]), .n_req(n_vrom));
  m3v_ddr_rd u_prd (
    .clk, .rst, .base(POLY + {3'd0, slot, 19'd0}),
    .g_req(pr_req), .g_addr(pr_addr), .g_len(pr_len), .g_ack(pr_ack), .g_rvalid(pr_rv), .g_rdata(pr_rd),
    .m_req(d_req[2]), .m_addr(d_addr[2]), .m_blen(d_blen[2]), .m_rvalid(d_rvalid[2]), .m_ack(d_ack[2]), .m_dout(dm_dout),
    .n_req(n_pram));
  assign d_we[2] = 1'b0;
  assign d_din[2] = '0;

  // ===========================================================================
  // store, raster (the set-up is the geometry's, SETUP_OUT)
  // ===========================================================================
  logic [15:0] su_tris;              // set-up records stored
  logic        st_clear, st_wbusy, band_go, band_busy, rs_idle, new_gen, st_ovf;
  logic [5:0]  band;
  logic        r_valid, r_ready, r_last;
  logic [31:0] r_data;
  logic [7:0][15:0] st_cnt;
  m3v_store #(.LIST_BASE(LISTS), .LOD(1'b1)) u_store (
    .clk, .rst, .clear(st_clear),
    .w_valid(g_valid), .w_ready(g_ready), .w_data(g_data), .w_last(g_last), .w_busy(st_wbusy),
    .band_go, .band, .band_busy, .rs_idle, .new_gen,
    .r_valid, .r_ready, .r_data, .r_last,
    .m_req(d_req[3]), .m_we(d_we[3]), .m_addr(d_addr[3]), .m_blen(d_blen[3]), .m_din(d_din[3]),
    .m_wacc(d_wacc[3]), .m_rvalid(d_rvalid[3]), .m_ack(d_ack[3]), .m_dout(dm_dout),
    .cnt(st_cnt), .n_tris(su_tris), .overflow(st_ovf));

  logic        f_start, eob, rs_busy;
  logic        fb_show;
  logic [31:0] n_pix;
  m3v_raster u_rast (
    .clk, .rst,
    .r_valid, .r_ready, .r_data, .r_last, .band, .new_gen, .idle(rs_idle),
    .f_start, .eob, .fb_base(fb_show ? 25'd0 : FB1), .busy(rs_busy),
    .m_req(d_req[4]), .m_addr(d_addr[4]), .m_blen(d_blen[4]), .m_din(d_din[4]),
    .m_wacc(d_wacc[4]), .m_ack(d_ack[4]),
    .tex_base(TEX + {2'd0, slot, 20'd0}),
    .t_req(pp_req[1]), .t_addr(pp_addr[1]), .t_ack(pp_ack[1]), .t_rvalid(pp_rvalid[1]), .t_dout(dm_dout),
    .n_pix);
  assign pp_blen[1] = 8'd4;
  assign d_we[4] = 1'b1;

  // ===========================================================================
  // the sequencer
  // ===========================================================================
  typedef enum logic [3:0] { Q_IDLE, Q_CLR, Q_GEO, Q_GEOW, Q_BAND, Q_BANDW, Q_FLUSH, Q_FLUSHW, Q_SWAP } qst_t;
  qst_t q;
  logic        go_pend, swap_pend, fb_ok;
  logic [31:0] cyc, geo_cyc, geo_cyc_l, frame_cyc_l, pix_l;
  logic        loaded_d;
  logic [2:0]  slot_n;               // the frame asked for; taken at the next frame's start
  wire         loaded = vrom_loaded && frames_loaded && n_frames != 3'd0 && ddr_ok;
  logic        vblank_start;
  wire         swap_now = swap_pend && vblank_start;

  always_ff @(posedge clk) begin
    if (rst) begin
      q <= Q_IDLE; go_pend <= 1'b0; geo_start <= 1'b0; band_go <= 1'b0; st_clear <= 1'b0;
      f_start <= 1'b0; eob <= 1'b0; swap_pend <= 1'b0; slot <= 3'd0; slot_n <= 3'd0; loaded_d <= 1'b0;
      frames_done <= '0; geo_cyc_l <= '0; frame_cyc_l <= '0; pix_l <= '0;
    end else begin
      geo_start <= 1'b0; band_go <= 1'b0; st_clear <= 1'b0; f_start <= 1'b0; eob <= 1'b0;
      loaded_d <= loaded;
      if (loaded && !loaded_d) begin go_pend <= 1'b1; slot_n <= 3'd0; end
      if (redraw) go_pend <= 1'b1;
      if (next_frame && loaded) begin
        go_pend <= 1'b1;
        slot_n <= (slot_n + 3'd1 >= n_frames) ? 3'd0 : slot_n + 3'd1;
      end
      cyc <= cyc + 32'd1;
      case (q)
        Q_IDLE: if (loaded && (go_pend || loop) && !swap_pend) begin
          go_pend <= 1'b0; cyc <= '0; slot <= slot_n;
          st_clear <= 1'b1; f_start <= 1'b1;
          q <= Q_CLR;
        end
        Q_CLR: if (!rs_busy && !f_start) begin geo_start <= 1'b1; q <= Q_GEO; end
        Q_GEO: q <= Q_GEOW;                 // busy rises
        Q_GEOW: if (!geo_busy && !st_wbusy && !g_valid) begin
          geo_cyc <= cyc; band <= 6'd0; q <= Q_BAND;
        end
        Q_BAND: begin band_go <= 1'b1; q <= Q_BANDW; end
        // a band ends: the band stage flushes it while the next draws
        Q_BANDW: if (!band_go && !band_busy && rs_idle) begin eob <= 1'b1; q <= Q_FLUSH; end
        Q_FLUSH: begin
          if (band == 6'd47) q <= Q_FLUSHW;
          else begin band <= band + 6'd1; q <= Q_BAND; end
        end
        Q_FLUSHW: if (!rs_busy && !eob) begin swap_pend <= 1'b1; q <= Q_SWAP; end
        Q_SWAP: begin
          frames_done <= frames_done + 32'd1;
          geo_cyc_l <= geo_cyc; frame_cyc_l <= cyc; pix_l <= n_pix;
          q <= Q_IDLE;
        end
        default: q <= Q_IDLE;
      endcase
      if (swap_now) swap_pend <= 1'b0;
    end
  end
  assign drawing = (q != Q_IDLE);

  // ===========================================================================
  // video: timing, the layer's scan-out, the overlay
  // ===========================================================================
  logic [6:0] ce_acc;
  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin ce_acc <= '0; ce_pix <= 1'b0; end
    else if (ce_acc + 7'd16 >= 7'd100) begin ce_acc <= ce_acc + 7'd16 - 7'd100; ce_pix <= 1'b1; end
    else begin ce_acc <= ce_acc + 7'd16; ce_pix <= 1'b0; end
  end

  logic [9:0] hcnt, vcnt;
  logic       hblank, vblank, hsync, vsync;
  m2_video_timing u_vt (
    .clk, .ce_pix, .rst_n(mem_rst_n), .interlace(1'b0), .ntsc_lines(1'b0), .mid_lines(1'b0), .narrow(1'b0),
    .hcnt, .vcnt, .hblank, .vblank, .hsync, .vsync, .visible(), .line_start(), .line_number(),
    .vblank_start, .field(), .ypos(), .vtotal());

  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin fb_show <= 1'b0; fb_ok <= 1'b0; end
    else if (swap_now) begin fb_show <= !fb_show; fb_ok <= 1'b1; end
  end
  assign fb_show_o = fb_show;

  // one pixel behind the counters, as Model 2 R779
  logic [9:0] x_d, y_d;
  logic       hb_d, vb_d, hs_d, vs_d;
  always_ff @(posedge clk) if (ce_pix) begin
    x_d <= hcnt; y_d <= vcnt; hb_d <= hblank; vb_d <= vblank; hs_d <= hsync; vs_d <= vsync;
  end

  // the line three ahead of the beam (Model 2 m2_raster3d, R655 / R657)
  logic [9:0] fl_sy_q;
  logic       line_req;
  logic [8:0] line_y;
  wire        fl_top = (y_d >= 10'd385) && (y_d < 10'd389);
  wire  [9:0] fl_tgt = fl_top ? y_d - 10'd385 : (y_d + 10'd3 < 10'd384) ? y_d + 10'd3 : 10'd384;
  always_ff @(posedge clk) begin
    if (!mem_rst_n) begin fl_sy_q <= '0; line_req <= 1'b0; line_y <= '0; end
    else begin
      fl_sy_q <= y_d;
      line_req <= 1'b0;
      // the probe's case (a) holds the scan-out off: no line is fetched
      // nothing until Main has released the DDR3 port (a command sent before is
      // lost or wedges the port: the DDR3 probe core, on the board)
      if (y_d != fl_sy_q && fl_tgt < 10'd384 && !so_off && ddr_ok) begin line_req <= 1'b1; line_y <= fl_tgt[8:0]; end
    end
  end

  logic [23:0] fb_col;
  logic        fb_hit;
  logic [15:0] fb_lines, fb_late;
  m2_fb_read #(.WIDTH(496), .STRIDE(512), .LA(3), .V_TOTAL(424)) u_fbr (
    .clk, .rd_clk(clk), .rst_n(mem_rst_n),
    .fb_sel(fb_show), .line_req, .line_y, .il(1'b0), .line_f(1'b0), .line_ready(), .hungry(),
    .m_req(d_req[0]), .m_we(d_we[0]), .m_addr(d_addr[0]), .m_blen(d_blen[0]),
    .m_rvalid(d_rvalid[0]), .m_dout(dm_dout), .m_ack(d_ack[0]),
    .rd_buf(y_d[1:0]), .rd_x(x_d[8:0]), .rd_col(fb_col), .rd_hit(fb_hit),
    .dbg_lines(fb_lines), .dbg_late(fb_late), .dbg_st(), .dbg_busy(), .dbg_acks_seen());
  assign d_din[0] = '0;

  // the running cycle count, latched once a video frame so it reads steadily
  logic [31:0] cyc_v;
  always_ff @(posedge clk) if (vblank_start) cyc_v <= cyc;
  logic [15:0][31:0] ov_val;
  logic ov_box, ov_ink;
  always_comb begin
    ov_val[0]  = frames_done;
    ov_val[1]  = {vrom_loaded, frames_loaded, mem_ready, fb_show, 1'b0, n_frames, 1'b0, slot, q, geo_err, 2'd0, band};
    ov_val[2]  = geo_cyc_l;
    ov_val[3]  = frame_cyc_l;
    ov_val[4]  = {su_tris, 16'd0};     // triangles stored; the low half was m3v_setup's drop count (R76: none to show)
    ov_val[5]  = pix_l;
    ov_val[6]  = {n_cull, n_vrom};
    ov_val[7]  = {n_pram, 13'd0, st_ovf, ld_ovf, 1'b0};
    ov_val[8]  = {st_cnt[0], st_cnt[1]};
    ov_val[9]  = {st_cnt[2], st_cnt[3]};
    ov_val[10] = {st_cnt[4], st_cnt[5]};
    ov_val[11] = {st_cnt[6], st_cnt[7]};
    ov_val[12] = {ddr_lat_max, 10'd0, ddr_acc_max};   // slowest first beat (cycles); most reads the bridge held
    ov_val[13] = {fb_lines, fb_late};
    ov_val[14] = cyc_v;
    ov_val[15] = 32'h3D5C_0025;           // M2.5 viewer
  end
  // rows 0-3 these values in hex; rows 4-11 the probe's in decimal digits
  // (sim/viewer/notes.md has the layout)
  logic [5:0]  ov_idx;
  logic        ov_hi;
  logic [31:0] ov_q, ov_v;
  assign pr_idx = 5'(ov_idx - 6'd16);
  always_ff @(posedge clk) begin ov_q <= ov_val[ov_idx[3:0]]; ov_hi <= ov_idx >= 6'd16; end
  assign ov_v = ov_hi ? pr_val : ov_q;
  m3v_overlay #(.NR(PROBE ? 12 : 4)) u_ov (.clk, .en(overlay_en), .x(x_d), .y(y_d), .idx(ov_idx), .val(ov_v), .box(ov_box), .ink(ov_ink));

  // the pixel: the layer where drawn, black elsewhere; the overlay on top
  always_ff @(posedge clk) if (ce_pix) begin
    logic [23:0] c;
    c = (fb_ok && fb_hit) ? fb_col : 24'h000000;
    if (ov_box) c = ov_ink ? 24'hffffff : {1'b0, c[23:17], 1'b0, c[15:9], 1'b0, c[7:1]} & 24'h3f3f3f;
    {v_r, v_g, v_b} <= (hb_d || vb_d) ? 24'd0 : c;
    v_hs <= hs_d; v_vs <= vs_d; v_hblank <= hb_d; v_vblank <= vb_d;
  end

endmodule
