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
// The DDR3 probe core (study D-33, D-34), below the framework so the Verilator
// bench runs it whole. Two clocks:
//
//   clk    fixed (100 MHz): the settings, the screen, the reporter (screen
//          and UART), the video
//   clk_d  the DDR3 clock, from a reconfigurable PLL: 100, 125, 85 or 62.5 MHz
//          (the emu rewrites the PLL when cfg_tgl toggles): dp_eng, the master
//          and the measurement
//
// Settings (status bits, see ddr3_probe.sv): automatic sweep or manual; for
// manual, direction, address order, burst, in flight, background; the clock.
// Any change resets the measurement: the engine is held in reset, the master
// left to go idle, the clock rewritten if it changed (and its lock awaited),
// the screen redrawn for the mode, the start line sent, the engine let go.
//
// Crossings: the settings and the engine's reset are quasi-static levels,
// synchronised in dp_eng; its idle and the clock's lock are synchronised here;
// a window's results and the progress counters cross by toggle handshakes, and
// stay put on the far side until the handshake completes.

module dp_core
  import dp_text_pkg::*;
#(
  parameter logic [28:0] DDR_BASE = 29'h0600_0000,   // byte 0x30000000 (Model 2 R654)
  parameter int          TICK     = 10_000_000,      // 0.1 s at clk: the debug lines' period
  parameter int          LOCKW    = 100_000,         // cycles of steady lock before the engine runs (1 ms)
  parameter logic [47:0] BUILD    = "000000",
  parameter int unsigned UDIV     = 868,             // 115200 baud at 100 MHz
  parameter int          WIN0     = 100_000_000,     // dp_eng's windows at clock settings 0..3
  parameter int          WIN1     = 125_000_000,
  parameter int          WIN2     = 85_000_000,
  parameter int          WIN3     = 62_500_000
) (
  input  logic        clk,
  input  logic        por_n,          // the fixed clock locked: the PLL's request and record only
  input  logic        rst_n,          // the fixed clock locked; RESET; the OSD's reset
  input  logic        ddr_ok,         // Main has finished the load: DDR3 may be touched (see ddr3_probe.sv)
  input  logic [4:0][31:0] dbg_emu,   // the emu's pin-level debug: RESET rises, cycles since, RESET at the first take, RD to take, RD to beat

  input  logic [16:0] status,         // status[16:1] (bit 0 unused here)

  input  logic        clk_d,
  input  logic        d_locked,       // the DDR3 clock's PLL locked (asynchronous)
  output logic [1:0]  clk_sel,        // the DDR3 clock wanted: 0 100, 1 125, 2 85, 3 62.5 MHz
  output logic        cfg_tgl,        // toggles: rewrite the PLL for clk_sel

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

  output logic        ce_pix,
  output logic        v_hs, v_vs, v_hblank, v_vblank,
  output logic [7:0]  v_r, v_g, v_b,

  output logic        uart_tx
);

  // ===========================================================================
  // the engine, on clk_d
  // ===========================================================================
  // its master's reset: the PLL's lock, synchronised to clk_d
  (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) logic [2:0] dl_d;
  always_ff @(posedge clk_d) dl_d <= {dl_d[1:0], d_locked};

  logic        rst_req;
  logic [16:0] cur;                    // the settings in force: {skip, lpath, init1, manual, wr, seq, burst 3, infl 3, bg 2, clk 2, 0}
  logic        m_idle;
  logic        r_tgl, r_ack, d_req, d_ack;
  logic [4:0]  e_k;      logic [5:0] e_n;  logic [1:0] e_bg;  logic [7:0] e_ln;  logic e_seq, e_wr;
  logic [31:0] e_bytes, e_bgb;  logic [15:0] e_lmin, e_lmax;  logic [8:0] e_lmed;  logic [5:0] e_ac;
  logic [15:0] e_pass, e_err;
  logic [31:0] e_iw, e_rd, e_bt, e_busy, e_rdy, e_lrv, e_rq;  logic [3:0] e_st;  logic [6:0] e_acc, e_oc;  logic [7:0] e_mst;

  dp_eng #(.DDR_BASE(DDR_BASE), .NCASE(NCASE), .WIN0(WIN0), .WIN1(WIN1), .WIN2(WIN2), .WIN3(WIN3)) u_eng (
    .clk(clk_d), .mem_rst_n(dl_d[2]), .rst_req,
    .s_manual(cur[13]), .s_wr(cur[12]), .s_seq(cur[11]), .s_burst(cur[10:8]), .s_infl(cur[7:5]), .s_bg(cur[4:3]), .s_clk(cur[2:1]),
    .s_skip(cur[16]), .s_lpath(cur[15]), .s_init1(cur[14]),
    .DDRAM_CLK, .DDRAM_BUSY, .DDRAM_BURSTCNT, .DDRAM_ADDR, .DDRAM_DIN, .DDRAM_BE,
    .DDRAM_WE, .DDRAM_RD, .DDRAM_DOUT, .DDRAM_DOUT_READY,
    .m_idle_o(m_idle),
    .r_tgl, .r_ack, .r_k(e_k), .r_n(e_n), .r_bg(e_bg), .r_ln(e_ln), .r_seq(e_seq), .r_wr(e_wr),
    .r_bytes(e_bytes), .r_bgb(e_bgb), .r_lmin(e_lmin), .r_lmax(e_lmax), .r_lmed(e_lmed), .r_ac(e_ac),
    .r_pass(e_pass), .r_err(e_err),
    .d_req, .d_ack, .d_iw(e_iw), .d_rd(e_rd), .d_bt(e_bt), .d_busy(e_busy), .d_st(e_st), .d_acc(e_acc), .d_oc(e_oc),
    .d_rdy(e_rdy), .d_lrv(e_lrv), .d_mst(e_mst), .d_rq(e_rq));

  // ===========================================================================
  // the fixed side: settings, sequencing, the crossings
  // ===========================================================================
  (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) logic [2:0] lk_s, id_s, rt_s, da_s;
  logic [16:0] st_s1, want;
  always_ff @(posedge clk) begin
    lk_s <= {lk_s[1:0], d_locked}; id_s <= {id_s[1:0], m_idle};
    rt_s <= {rt_s[1:0], r_tgl}; da_s <= {da_s[1:0], d_ack};
    st_s1 <= status; want <= {st_s1[14], st_s1[15], st_s1[16], st_s1[1], st_s1[2], st_s1[3], st_s1[6:4], st_s1[9:7], st_s1[11:10], st_s1[13:12], 1'b0};
  end

  // the screen's characters: 21 rows x 48, M10K; the static sweep, then the reporter
  (* ramstyle = "M10K, no_rw_check" *) logic [5:0] scr [1024] /* verilator public_flat_rd */;
  logic        c_we, r_we;
  logic [9:0]  c_wa, c_ra, r_wa;
  logic [5:0]  c_wd, c_q, r_wd;
  always_ff @(posedge clk) begin
    if (c_we) scr[c_wa] <= c_wd;
    else if (r_we) scr[r_wa] <= r_wd;
    c_q <= scr[c_ra];
  end

  typedef enum logic [3:0] { P_HOLD, P_IDLE, P_CFG, P_LOCK, P_SCR, P_START, P_RUN, P_RES, P_DBG } pst_t;
  pst_t ps /* verilator public_flat_rd */;
  logic [2:0]  applied;                // the clock the PLL holds: 0-3, 4 not yet set
  logic [16:0] wt;
  logic [23:0] tick;
  logic [15:0] t10, since;
  logic        first_done, dbg_wait;
  logic        go_start, go_dbg, go_case, go_man, rep_busy /* verilator public_flat_rd */;
  // the fixed side's copies of the engine's reports
  logic [4:0]  f_k /* verilator public_flat_rd */;   logic [5:0] f_n;  logic [1:0] f_bg;  logic [7:0] f_ln;  logic f_seq, f_wr;
  logic [31:0] f_bytes /* verilator public_flat_rd */, f_bgb /* verilator public_flat_rd */;  logic [15:0] f_lmin /* verilator public_flat_rd */, f_lmax /* verilator public_flat_rd */;  logic [8:0] f_lmed /* verilator public_flat_rd */;  logic [5:0] f_ac /* verilator public_flat_rd */;
  logic [15:0] f_pass, f_err /* verilator public_flat_rd */;
  logic [31:0] f_iw, f_rd, f_bt, f_busy, f_rdy, f_lrv, f_rq;  logic [3:0] f_st;  logic [6:0] f_acc, f_oc;  logic [7:0] f_mst;
  wire         running = (ps == P_START) || (ps == P_RUN) || (ps == P_RES) || (ps == P_DBG);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ps <= P_HOLD; rst_req <= 1'b1; wt <= '0;
      c_we <= 1'b0; cur <= '0; r_ack <= 1'b0; d_req <= 1'b0; tick <= '0; t10 <= '0; since <= '0;
      first_done <= 1'b0; dbg_wait <= 1'b0; go_start <= 1'b0; go_dbg <= 1'b0; go_case <= 1'b0; go_man <= 1'b0;
      f_k <= '0; f_pass <= '0; f_err <= '0;
    end else begin
      c_we <= 1'b0; go_start <= 1'b0; go_dbg <= 1'b0; go_case <= 1'b0; go_man <= 1'b0;
      if (tick == 24'(TICK - 1)) begin
        tick <= '0; t10 <= t10 + 16'd1;
        if (since != 16'hffff) since <= since + 16'd1;
        // the debug lines: every tick until the first window ends, and while
        // none has for three seconds
        if (running && ps != P_START && (!first_done || since >= 16'd30) && !dbg_wait) begin d_req <= !d_req; dbg_wait <= 1'b1; end
        // while waiting for DDR3 (the gate): the line anyway, the emu's RESET
        // fields live, the engine's at zero
        if (ps == P_START && !ddr_ok && !rep_busy) go_dbg <= 1'b1;
      end else tick <= tick + 24'd1;

      // a settings change: back to the start
      if (ps != P_HOLD && want != cur) begin ps <= P_HOLD; rst_req <= 1'b1; wt <= '0; end
      else case (ps)
        // the engine held; the master let go idle
        P_HOLD: begin
          rst_req <= 1'b1; wt <= wt + 17'd1;
          if (wt >= 17'd64 && (id_s[2] || !lk_s[2])) begin cur <= want; wt <= '0; ps <= P_IDLE; end
        end
        P_IDLE: begin
          if ({1'b0, cur[2:1]} != applied) begin wt <= '0; ps <= P_CFG; end
          else begin wt <= '0; ps <= P_SCR; end
        end
        // the clock rewritten: wait for the lock to go (or 1 ms), then for it to hold
        P_CFG: begin
          wt <= wt + 17'd1;
          if (!lk_s[2] || wt == 17'(LOCKW)) begin wt <= '0; ps <= P_LOCK; end
        end
        P_LOCK: begin
          wt <= lk_s[2] ? wt + 17'd1 : '0;
          if (wt == 17'(LOCKW)) begin wt <= '0; ps <= P_SCR; end
        end
        // the screen for the mode; the start line; the engine let go
        P_SCR: begin
          c_we <= 1'b1; c_wd <= cur[13] ? stext_m(wt[9:0]) : stext(wt[9:0]); c_wa <= wt[9:0];
          wt <= wt + 17'd1;
          if (wt == 17'd1023) begin first_done <= 1'b0; since <= '0; dbg_wait <= 1'b0; d_req <= da_s[2]; ps <= P_START; end
        end
        // the engine let go only once DDR3 may be touched
        P_START: if (!rep_busy && !go_start && ddr_ok) begin
          go_start <= 1'b1; rst_req <= 1'b0; r_ack <= rt_s[2]; ps <= P_RUN;
        end
        P_RUN: begin
          if (rt_s[2] != r_ack && !rep_busy) begin
            // a window's results: stable on the far side until r_ack follows
            f_k <= e_k; f_n <= e_n; f_bg <= e_bg; f_ln <= e_ln; f_seq <= e_seq; f_wr <= e_wr;
            f_bytes <= e_bytes; f_bgb <= e_bgb; f_lmin <= e_lmin; f_lmax <= e_lmax; f_lmed <= e_lmed; f_ac <= e_ac;
            f_pass <= e_pass; f_err <= e_err;
            ps <= P_RES;
          end else if (dbg_wait && da_s[2] == d_req && !rep_busy) begin
            f_iw <= e_iw; f_rd <= e_rd; f_bt <= e_bt; f_busy <= e_busy; f_st <= e_st; f_acc <= e_acc; f_oc <= e_oc;
            f_rdy <= e_rdy; f_lrv <= e_lrv; f_mst <= e_mst; f_rq <= e_rq;
            ps <= P_DBG;
          end
        end
        P_RES: begin
          if (cur[13]) go_man <= 1'b1; else go_case <= 1'b1;
          r_ack <= rt_s[2]; first_done <= 1'b1; since <= '0;
          ps <= P_RUN;
        end
        P_DBG: begin go_dbg <= 1'b1; dbg_wait <= 1'b0; ps <= P_RUN; end
        default: ps <= P_HOLD;
      endcase
    end
  end

  // the PLL's request and what it holds: a core reset must not rewrite the
  // PLL (a toggle back to its reset value would), so these see power-on only
  always_ff @(posedge clk) begin
    if (!por_n) begin applied <= 3'd4; clk_sel <= 2'd0; cfg_tgl <= 1'b0; end
    else begin
      if (rst_n && ps == P_IDLE && {1'b0, cur[2:1]} != applied && !(want != cur)) begin clk_sel <= cur[2:1]; cfg_tgl <= !cfg_tgl; end
      if (rst_n && ps == P_LOCK && wt == 17'(LOCKW) && !(want != cur)) applied <= {1'b0, cur[2:1]};
    end
  end

  // ===========================================================================
  // the reporter, and the fields it prints
  // ===========================================================================
  logic [5:0]  f_idx;
  logic [31:0] f_val;
  logic [15:0] drops /* verilator public_flat_rd */;
  wire  [1:0]  ck = applied[1:0];
  always_comb begin
    case (f_idx)
      6'd31: f_val = f_rdy;
      6'd32: f_val = f_lrv;
      6'd33: f_val = 32'(f_mst);
      6'd34: f_val = f_rq;
      6'd35: f_val = dbg_emu[0];
      6'd36: f_val = dbg_emu[1];
      6'd37: f_val = dbg_emu[2];
      6'd38: f_val = dbg_emu[3];
      6'd39: f_val = dbg_emu[4];
      6'd0:  f_val = f_iw;
      6'd1:  f_val = f_rd;
      6'd2:  f_val = f_bt;
      6'd3:  f_val = f_busy;
      6'd4:  f_val = 32'(f_st);
      6'd5:  f_val = 32'(f_k);
      6'd6:  f_val = 32'(f_n);
      6'd7:  f_val = 32'(f_bg);
      6'd8:  f_val = 32'(f_ln);
      6'd9:  f_val = f_bytes;
      6'd10: f_val = (f_lmin > 16'd999) ? 32'd999 : 32'(f_lmin);
      6'd11: f_val = 32'(f_lmed);
      6'd12: f_val = 32'(f_lmax);
      6'd13: f_val = 32'(f_ac);
      6'd14: f_val = f_bgb;
      6'd15: f_val = 32'(f_pass);
      6'd16: f_val = 32'(f_err);
      6'd17: f_val = 32'(drops);
      6'd18: f_val = 32'(f_seq);
      6'd19: f_val = {DDR_BASE, 3'd0};
      6'd20: f_val = 32'(f_acc);
      6'd21: f_val = 32'(f_oc);
      6'd22: f_val = 32'(t10);
      6'd23: f_val = (ck == 2'd1) ? 32'd125 : (ck == 2'd2) ? 32'd85 : (ck == 2'd3) ? 32'd62 : 32'd100;
      6'd24: f_val = (f_k == 5'(NCASE - 1)) ? 32'd0 : 32'(f_k) + 32'd1;
      6'd25: f_val = f_wr ? 32'd87 : 32'd82;                                  // W, R
      6'd26: f_val = f_seq ? 32'd83 : 32'd82;                                 // S, R
      6'd27: f_val = (f_bg == 2'd0) ? 32'd45 : 32'd76;                         // -, L
      6'd28: f_val = (f_bg == 2'd0) ? 32'd45 : (f_bg == 2'd1) ? 32'd32 : 32'd83;   // -, space, S
      6'd29: f_val = {15'd0, status[16:1], 1'b0};
      6'd30: f_val = cur[13] ? 32'd77 : 32'd65;                               // M, A
      default: f_val = '0;
    endcase
  end

  dp_report #(.BUILD(BUILD), .DIVISOR(UDIV)) u_rep (
    .clk, .rst_n,
    .go_start, .go_dbg, .go_case, .go_man, .busy(rep_busy), .row_case(f_k + 5'd2),
    .f_idx, .f_val,
    .c_we(r_we), .c_wa(r_wa), .c_wd(r_wd),
    .uart_tx, .drops);

  // ===========================================================================
  // video: 496 x 384 at 57.52 Hz (Model 2's timing, as the viewer), the text
  // ===========================================================================
  logic [6:0] ce_acc;
  always_ff @(posedge clk) begin
    if (!rst_n) begin ce_acc <= '0; ce_pix <= 1'b0; end
    else if (ce_acc + 7'd16 >= 7'd100) begin ce_acc <= ce_acc + 7'd16 - 7'd100; ce_pix <= 1'b1; end
    else begin ce_acc <= ce_acc + 7'd16; ce_pix <= 1'b0; end
  end

  logic [9:0] hcnt, vcnt;
  logic       hblank, vblank, hsync, vsync;
  m2_video_timing u_vt (
    .clk, .ce_pix, .rst_n, .interlace(1'b0), .ntsc_lines(1'b0), .mid_lines(1'b0), .narrow(1'b0),
    .hcnt, .vcnt, .hblank, .vblank, .hsync, .vsync, .visible(), .line_start(), .line_number(),
    .vblank_start(), .field(), .ypos(), .vtotal());

  logic [9:0] x_d, y_d;
  logic       hb_d, vb_d, hs_d, vs_d;
  always_ff @(posedge clk) if (ce_pix) begin
    x_d <= hcnt; y_d <= vcnt; hb_d <= hblank; vb_d <= vblank; hs_d <= hsync; vs_d <= vsync;
  end

  // cell and pixel in it, from the beam: x / 12 as (x 683) >> 13 and y / 18 as
  // (y 911) >> 14, exact for x < 2048 and y < 1024, registered stage by stage;
  // it settles within a pixel's six clocks
  localparam int TX = 2, TY = 3;
  logic [9:0]  ax, ay;  logic in_a;
  logic [19:0] mx, my;  logic [9:0] bx, by;  logic in_b;
  always_ff @(posedge clk) begin
    ax <= x_d - 10'(TX); ay <= y_d - 10'(TY);
    in_a <= (x_d >= 10'(TX)) && (x_d < 10'(TX + 492)) && (y_d >= 10'(TY)) && (y_d < 10'(TY + 378));
    mx <= 20'(ax) * 20'd683; my <= 20'(ay) * 20'd911; bx <= ax; by <= ay; in_b <= in_a;
  end
  logic [5:0] col;  logic [4:0] row;  logic [9:0] cx2, cy2;  logic in_t;
  always_ff @(posedge clk) begin
    col <= 6'(mx >> 13); row <= 5'(my >> 14); cx2 <= bx; cy2 <= by; in_t <= in_b;
  end
  logic [3:0] sx2;  logic [4:0] sy2;  logic in_t2;
  always_ff @(posedge clk) begin
    c_ra <= {row, 5'd0} + {1'b0, row, 4'd0} + 10'(col);
    sx2 <= 4'(cx2 - {col, 3'd0} - {1'b0, col, 2'd0});
    sy2 <= 5'(cy2 - {row, 4'd0} - {4'd0, row, 1'b0});
    in_t2 <= in_t;
  end
  logic [3:0] sx3;  logic [4:0] sy3;  logic in_t3;
  always_ff @(posedge clk) begin sx3 <= sx2; sy3 <= sy2; in_t3 <= in_t2; end   // c_q now
  wire [4:0] grow = glyph(c_q, 3'(sy3 >> 1));
  wire       ink  = in_t3 && sx3 < 4'd10 && sy3 < 5'd14 && grow[3'd4 - 3'(sx3 >> 1)];

  always_ff @(posedge clk) if (ce_pix) begin
    // the title row in yellow, the debug row in cyan, the rest white, on dark blue
    {v_r, v_g, v_b} <= (hb_d || vb_d) ? 24'd0 :
                       ink ? ((y_d < 10'(TY + 18)) ? 24'hffff40 : (y_d >= 10'(TY + 360)) ? 24'h60ffff : 24'hffffff) : 24'h101830;
    v_hs <= hs_d; v_vs <= vs_d; v_hblank <= hb_d; v_vblank <= vb_d;
  end

endmodule
