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
// Model 3 tile generator (M0-E, study 5.4): four 64 x 64 layers of 8 x 8 tiles,
// 4 or 8 bits a pixel, per-layer enable, scroll and above/below-3D priority,
// line scroll, the stencil in 32-pixel spans over layer pairs, and the 32 K
// palette with its suppress bit.
//
// A line is built ahead of the beam into one of two line buffers: the fetch
// engine (m3_tilegen_fetch, one engine for all four layers) walks the layers in
// draw order and the emitter here writes every opaque pixel, so a later layer
// simply overwrites an earlier one. MAME's draw order (below: 3, 2, 1, 0; above:
// 3, 2, 0, 1) is also Supermodel's wherever the two can differ, because its
// pairs never overlap. Each line-buffer word is {drawn, above, pair, colour}:
// one colour and one priority bit is all the compositor needs, since an
// above-3D pixel covers everything and a below-3D one shows only where the 3D
// does not.
//
// Timing: one clock (100 MHz) for everything. The scanout reads one pixel per
// sc_rd (the pixel enable) and clears it behind itself; output arrives three
// clocks later. The controller pulses swap (the built line becomes the shown
// one) and then go for the next line, at least one clock apart.
//
// CPU side: registers as the chip sees them (little-endian values, i.e. the
// PowerPC word byte-reversed, as Supermodel's WriteRegister receives them);
// palette writes as an entry index and the low 16 bits of its little-endian
// word. Tile RAM writes go straight to the SDRAM image.

module m3_tilegen #(
  parameter bit MASK_SCREEN = 1'b1,
  parameter bit SCROLL_SM   = 1'b1,
  parameter bit RS_SWAP     = 1'b1,
  parameter bit STALE       = 1'b1,
  parameter bit CODE_WRAP   = 1'b0,
  parameter bit EXPAND_SM   = 1'b1,   // 5 to 8 bits as x * 255 / 31 (SM) / (x << 3) | (x >> 2) (MAME)
  parameter bit OFFSET      = 1'b1,   // the pair colour offsets F1180040 / F1180044 (SM; MAME has none)
  parameter int AW          = 25,
  parameter logic [AW:1] VBASE = '0
) (
  input  logic          clk,
  input  logic          rst_n,
  output logic          ready,

  input  logic          reg_we,
  input  logic [5:0]    reg_addr,     // word index in F1180000-FF
  input  logic [31:0]   reg_wdata,
  input  logic          pal_we,
  input  logic [14:0]   pal_addr,
  input  logic [15:0]   pal_wdata,

  input  logic          swap,
  input  logic          go,
  input  logic [8:0]    go_y,
  output logic          busy,
  output logic          overrun,

  input  logic          sc_rd,
  input  logic [8:0]    sc_x,
  output logic          o_v,          // the pixel read three clocks ago
  output logic          o_drawn,
  output logic          o_above,
  output logic [7:0]    o_r,
  output logic [7:0]    o_g,
  output logic [7:0]    o_b,

  output logic          p_req,
  output logic [AW:1]   p_addr,
  output logic          p_long,       // eight-word reads (the name row buffer, R103)
  input  logic [63:0]   p_dout,
  input  logic          p_lo,
  input  logic          p_ack,
  input  logic          name_inv,     // a CPU write to the name tables (F10F8000-FFFFF)

  output logic [15:0]   st_cycles,    // clocks the last line took
  output logic [15:0]   st_reqs       // SDRAM requests it made
);

  // -------------------------------------------------------------- registers
  logic [7:0]       pri;
  logic [23:0]      off0, off1;
  logic [3:0][31:0] lreg;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pri <= '0; off0 <= '0; off1 <= '0; lreg <= '0;
    end else if (reg_we) begin
      case (reg_addr)
        6'h08: pri  <= reg_wdata[15:8];
        6'h10: off0 <= reg_wdata[23:0];
        6'h11: off1 <= reg_wdata[23:0];
        6'h18: lreg[0] <= reg_wdata;
        6'h19: lreg[1] <= reg_wdata;
        6'h1A: lreg[2] <= reg_wdata;
        6'h1B: lreg[3] <= reg_wdata;
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------ fetch
  logic        q_valid, q_pop, q_is4, q_above, q_pair, flush;
  logic [8:0]  q_x;
  logic [3:0]  q_n;
  logic [2:0]  q_col;
  logic [63:0] q_data;
  logic [10:0] q_palhi;
  logic [15:0] n_req;
  logic        f_busy;

  m3_tilegen_fetch #(
    .MASK_SCREEN(MASK_SCREEN), .SCROLL_SM(SCROLL_SM), .RS_SWAP(RS_SWAP), .STALE(STALE),
    .CODE_WRAP(CODE_WRAP), .AW(AW), .VBASE(VBASE)
  ) u_fetch (
    .clk, .rst_n, .go(go && ready), .y(go_y), .busy(f_busy), .overrun,
    .pri, .lreg,
    .p_req, .p_addr, .p_long, .p_dout, .p_lo, .p_ack, .inv(name_inv),
    .q_valid, .q_pop, .q_x, .q_n, .q_col, .q_data, .q_palhi, .q_is4, .q_above, .q_pair,
    .flush, .n_req
  );

  // ---------------------------------------------------------------- emitter
  logic        ec_v, ec_is4, ec_above, ec_pair, ec_bank;
  logic [8:0]  ec_x;
  logic [3:0]  ec_n, ec_i;
  logic [2:0]  ec_col;
  logic [63:0] ec_data;
  logic [10:0] ec_palhi;
  logic        front, wbank;

  wire       ec_last = ec_v && (ec_i == ec_n - 4'd1);
  // An abandoned line (go while busy) stops writing at once.
  wire       kill    = (go && ready && f_busy) || flush;
  assign q_pop = q_valid && (!ec_v || ec_last) && !kill;

  wire [2:0] pc = ec_col + ec_i[2:0];
  logic [7:0] pix8;
  logic [3:0] pix4;
  always_comb begin
    // 8-bit: the burst is two little-endian words, pixel 0 in bits 31:24 of the first
    case (pc)
      3'd0: pix8 = ec_data[31:24]; 3'd1: pix8 = ec_data[23:16]; 3'd2: pix8 = ec_data[15:8];  3'd3: pix8 = ec_data[7:0];
      3'd4: pix8 = ec_data[63:56]; 3'd5: pix8 = ec_data[55:48]; 3'd6: pix8 = ec_data[47:40]; default: pix8 = ec_data[39:32];
    endcase
    // 4-bit: one little-endian word, pixel 0 in bits 31:28
`ifdef TG_FAULT_BPP
    pix4 = ec_data[{pc, 2'b00} +: 4];
`else
    pix4 = ec_data[{~pc, 2'b00} +: 4];
`endif
  end
  wire [14:0] pal_ra = ec_is4 ? {ec_palhi, pix4} : {ec_palhi[10:4], pix8};

  // E1: palette read in flight
  logic        e1_v, e1_above, e1_pair, e1_bank;
  logic [8:0]  e1_x;
  // E2: line-buffer write
  logic        e2_we, e2_bank;
  logic [8:0]  e2_x;
  logic [17:0] e2_d;
  logic [14:0] pal_cq;
  logic        pal_fq;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ec_v <= 1'b0; e1_v <= 1'b0; e2_we <= 1'b0; ec_i <= '0;
    end else begin
      if (kill) begin
        ec_v <= 1'b0; e1_v <= 1'b0; e2_we <= 1'b0;
      end else begin
        if (q_pop) begin
          ec_v <= 1'b1; ec_i <= '0;
          ec_x <= q_x; ec_n <= q_n; ec_col <= q_col; ec_data <= q_data; ec_palhi <= q_palhi;
          ec_is4 <= q_is4; ec_above <= q_above; ec_pair <= q_pair; ec_bank <= wbank;
        end else if (ec_last) begin
          ec_v <= 1'b0;
        end else if (ec_v) begin
          ec_i <= ec_i + 4'd1;
        end
        e1_v     <= ec_v;
        e1_x     <= ec_x + {5'd0, ec_i};
        e1_above <= ec_above;
        e1_pair  <= ec_pair;
        e1_bank  <= ec_bank;
`ifdef TG_FAULT_SUPPRESS
        e2_we    <= e1_v;
`else
        e2_we    <= e1_v && !pal_fq;
`endif
        e2_x     <= e1_x;
        e2_bank  <= e1_bank;
        e2_d     <= {1'b1, e1_above, e1_pair, pal_cq};
      end
    end
  end

  // 32 K x 16 palette in M10K, as two memories so the 15 colour bits can pack
  // into 2 K x 5 blocks (48, m3_tilegen_pal) and the suppress bit into 8 K x 1 (4).
  m3_tilegen_pal u_pal_c (
    .clk, .we(pal_we), .wa(pal_addr), .wd(pal_wdata[14:0]), .ra(pal_ra), .q(pal_cq));
  r3d_ram #(.W(1), .A(15), .SYNC(1'b1), .BLOCK(1'b1)) u_pal_f (
    .clk, .we(pal_we), .wa(pal_addr), .wd(pal_wdata[15]), .ra(pal_ra), .rq(pal_fq));

  // ----------------------------------------------------------- line buffers
  logic        init;
  logic [8:0]  init_x;
  logic        s1_v, s1_bank, s2_v;
  logic [8:0]  s1_x;
  logic [17:0] lb_q [2];
  logic [17:0] s2_e;

  genvar gb;
  generate
    for (gb = 0; gb < 2; gb++) begin : g_lb
      localparam bit B = (gb == 1);
      wire        e_here = e2_we && !kill && (e2_bank == B);
      wire        c_here = s1_v && (s1_bank == B);
      wire        we_b   = e_here || c_here || init;
      wire [8:0]  wa_b   = e_here ? e2_x : (init ? init_x : s1_x);
      wire [17:0] wd_b   = e_here ? e2_d : 18'd0;
      r3d_ram #(.W(18), .A(9), .SYNC(1'b1), .BLOCK(1'b1)) u_lb (
        .clk, .we(we_b), .wa(wa_b), .wd(wd_b), .ra(sc_x), .rq(lb_q[gb]));
    end
  endgenerate

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      init <= 1'b1; init_x <= '0; front <= 1'b0; wbank <= 1'b1;
      s1_v <= 1'b0; s2_v <= 1'b0; o_v <= 1'b0;
      st_cycles <= '0; st_reqs <= '0;
    end else begin
      if (init) begin
        init_x <= init_x + 9'd1;
        if (init_x == 9'd511) init <= 1'b0;
      end
      if (swap) front <= ~front;
      if (go && ready) wbank <= ~front;
      s1_v    <= sc_rd && ready;
      s1_x    <= sc_x;
      s1_bank <= front;
      s2_v    <= s1_v;
      s2_e    <= lb_q[s1_bank];
      o_v     <= s2_v;
      o_drawn <= s2_e[17];
      o_above <= s2_e[16];
      o_r     <= col8(s2_e[4:0],   s2_e[15] ? off1[7:0]   : off0[7:0]);
      o_g     <= col8(s2_e[9:5],   s2_e[15] ? off1[15:8]  : off0[15:8]);
      o_b     <= col8(s2_e[14:10], s2_e[15] ? off1[23:16] : off0[23:16]);
      // telemetry
      if (go && ready) st_cycles <= '0;
      else if (busy && st_cycles != 16'hFFFF) st_cycles <= st_cycles + 16'd1;
      if (!busy) st_reqs <= n_req;
    end
  end

  assign ready = !init;
  // A line is done when its last pixel is written, not when the fetch stops.
  assign busy  = f_busy || ec_v || e1_v || e2_we;

  // ------------------------------------------------------------------ colour
  function automatic logic [7:0] ex(input logic [4:0] v);
    logic [7:0] s;
`ifdef TG_FAULT_EXPAND
    if (1'b1) s = {v, v[4:2]};
`else
    if (!EXPAND_SM) s = {v, v[4:2]};
`endif
    else begin
      case (v)
        5'd0: s = 8'd0; 5'd1: s = 8'd8; 5'd2: s = 8'd16; 5'd3: s = 8'd24; 5'd4: s = 8'd32; 5'd5: s = 8'd41;
        5'd6: s = 8'd49; 5'd7: s = 8'd57; 5'd8: s = 8'd65; 5'd9: s = 8'd74; 5'd10: s = 8'd82; 5'd11: s = 8'd90;
        5'd12: s = 8'd98; 5'd13: s = 8'd106; 5'd14: s = 8'd115; 5'd15: s = 8'd123; 5'd16: s = 8'd131;
        5'd17: s = 8'd139; 5'd18: s = 8'd148; 5'd19: s = 8'd156; 5'd20: s = 8'd164; 5'd21: s = 8'd172;
        5'd22: s = 8'd180; 5'd23: s = 8'd189; 5'd24: s = 8'd197; 5'd25: s = 8'd205; 5'd26: s = 8'd213;
        5'd27: s = 8'd222; 5'd28: s = 8'd230; 5'd29: s = 8'd238; 5'd30: s = 8'd246; default: s = 8'd255;
      endcase
    end
    ex = s;
  endfunction

  // value + 2 x signed offset, clamped to 0-255 (Supermodel's ColourOffsetRegister)
  function automatic logic [7:0] col8(input logic [4:0] v, input logic [7:0] o);
    logic [9:0] sum;
    logic [7:0] e;
    e = ex(v);
    if (!OFFSET) col8 = e;
    else begin
`ifdef TG_FAULT_OFFSET
      sum = {2'b00, e} + {o[7], o[7], o};
`else
      sum = {2'b00, e} + {o[7], o, 1'b0};
`endif
      if (sum[9])      col8 = 8'd0;      // negative
      else if (sum[8]) col8 = 8'd255;
      else             col8 = sum[7:0];
    end
  endfunction

endmodule
