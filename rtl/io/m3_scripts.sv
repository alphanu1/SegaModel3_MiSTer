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
// The NCR 53C810 as Model 3 uses it (study 5.2, M0-F): a DMA engine. VF3, VF3 Team
// Battle and Bass Fishing share one Sega driver (sim/scripts/notes.md). It builds a
// list of memory moves ending in an INT, sets DMODE.MAN, writes DSP, and then
// single-steps each move with DCNTL = SSM | STD. Each step ends in a single-step
// interrupt; the ISR reads ISTAT, DSTAT and DSP, and steps again until the next
// word in memory is the INT, which is never run. The capture's only executed
// opcode is MMOV.
//
// Built to the LSI53C810A manual (DB14-000168-00), which wins where the emulators
// disagree with it:
//   - DSP points past an instruction as soon as it is fetched (2.7.1.5).
//   - DCNTL.STD stays set until an interrupt (5-46); both emulators keep it set.
//   - A single step raises SSI, so ISTAT.DIP (DMA), not SIP (MAME sets both).
//   - DSTAT.DFE is the FIFO-empty status; reading DSTAT clears its interrupt bits
//     and DIP, and releases IRQ (MAME releases at an ISTAT read instead).
//   - A run started by a DSP write with MAN clear honours SSM (both emulators run
//     on). VF3 sets MAN, so this path is not used by the captured games.
//
// What is built (D-27: "everything else" in 3,500 ALM):
//   - SCRIPTS: MMOV, and INT (INT_OP) in its unconditional form. Every other
//     opcode, an INT with a carry, data or phase test or INTFLY, a zero count, and
//     a move whose addresses or count are not word multiples raise IID and halt.
//     (The chip moves bytes; the Sega driver builds counts as words times four.)
//   - Registers: ISTAT, DSTAT, DSP, DMODE, DIEN, DCNTL; TEMP, DBC/DCMD and DSPS
//     read back with RD_ALL. Other offsets read 0 and ignore writes.
//   - PCI configuration: ID, command, latency timer and BAR1 (the 256-byte memory
//     window). The bridge decodes the window from win_en and win_base: VF3 puts it
//     at C0000000, Bass Fishing at F9000000.
//
// One datapath (study 8.4): the move's source and destination pointers are DSPS
// and TEMP (the manual's holding registers) and the write count is DBC. Each pointer
// has its own incrementer: one adder shared through the port's address measured 16
// ALM larger (notes.md), as a constant increment sits in the register's carry chain.
// The data passes through a 16-word FIFO in MLAB (the chip's own is 80 bytes).
//
// PORTS
//
// Register window (CPU side): one access per s_req, held until the one-cycle
// s_ack; s_rdata is valid with it. s_addr is the word offset, s_be big-endian
// (s_be[3] is the byte at offset 0). The chip's registers are little-endian, so
// register byte N is in lane 3 - (N & 3): a stwbrx of DSP writes it in one word.
//
// PCI configuration: the same handshake; c_idx is the dword, c_be and c_wdata
// are in PCI order (c_be[0] is bits 7:0). The bridge does the byte reversal.
//
// Memory: ppc_cache_pkg's protocol, so the engine can share the CPU's memory bus
// or sit on a bridge port. One transaction at a time; m_req and its fields held
// until m_ack; m_req is low for at least a cycle between transactions (a
// rising-edge port such as m2_sdram sees each one). Line reads are issued only
// at line-aligned addresses, so the critical word is word 0. LINE = 0 uses
// single reads and writes only.

module m3_scripts
  import ppc_cache_pkg::*;
#(
  parameter bit ENABLE = 1'b1,   // D-14: 0 leaves a window that reads 0 and a port that never asks
  parameter bit LINE   = 1'b1,   // eight-word line reads and writes where the alignment allows
  parameter bit INT_OP = 1'b1,   // decode INT (not run by the captured games)
  parameter bit RD_ALL = 1'b0    // TEMP, DBC/DCMD, DSPS readable
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        c_req,
  input  logic        c_we,
  input  logic [5:0]  c_idx,
  input  logic [3:0]  c_be,
  input  logic [31:0] c_wdata,
  output logic        c_ack,
  output logic [31:0] c_rdata,
  output logic        win_en,
  output logic [31:8] win_base,

  input  logic        s_req,
  input  logic        s_we,
  input  logic [6:2]  s_addr,
  input  logic [3:0]  s_be,
  input  logic [31:0] s_wdata,
  output logic        s_ack,
  output logic [31:0] s_rdata,

  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  output logic [31:0] m_wdata,
  input  logic        m_ack,
  input  logic        m_wnext,
  input  logic        m_rvalid,
  input  logic [31:0] m_rdata,

  output logic        irq,
  output logic        busy
);

  generate
  if (ENABLE) begin : g_on
    m3_scripts_core #(.LINE(LINE), .INT_OP(INT_OP), .RD_ALL(RD_ALL)) u_core (
      .clk, .rst,
      .c_req, .c_we, .c_idx, .c_be, .c_wdata, .c_ack, .c_rdata, .win_en, .win_base,
      .s_req, .s_we, .s_addr, .s_be, .s_wdata, .s_ack, .s_rdata,
      .m_req, .m_cmd, .m_addr, .m_be, .m_wdata, .m_ack, .m_wnext, .m_rvalid, .m_rdata,
      .irq, .busy);
  end else begin : g_off
    // No device: configuration reads all ones (no device at this slot), the
    // window acknowledges with 0 so a stray access cannot hang the CPU.
    always_ff @(posedge clk) begin
      s_ack <= s_req && !s_ack && !rst;
      c_ack <= c_req && !c_ack && !rst;
    end
    assign s_rdata  = 32'd0;
    assign c_rdata  = 32'hFFFF_FFFF;
    assign win_en   = 1'b0;
    assign win_base = 24'd0;
    assign m_req    = 1'b0;
    assign m_cmd    = MC_RD;
    assign m_addr   = 32'd0;
    assign m_be     = 4'h0;
    assign m_wdata  = 32'd0;
    assign irq      = 1'b0;
    assign busy     = 1'b0;
  end
  endgenerate

endmodule

module m3_scripts_core
  import ppc_cache_pkg::*;
#(
  parameter bit LINE   = 1'b1,
  parameter bit INT_OP = 1'b1,
  parameter bit RD_ALL = 1'b0
)(
  input  logic        clk,
  input  logic        rst,
  input  logic        c_req,
  input  logic        c_we,
  input  logic [5:0]  c_idx,
  input  logic [3:0]  c_be,
  input  logic [31:0] c_wdata,
  output logic        c_ack,
  output logic [31:0] c_rdata,
  output logic        win_en,
  output logic [31:8] win_base,
  input  logic        s_req,
  input  logic        s_we,
  input  logic [6:2]  s_addr,
  input  logic [3:0]  s_be,
  input  logic [31:0] s_wdata,
  output logic        s_ack,
  output logic [31:0] s_rdata,
  output logic        m_req,
  output logic [1:0]  m_cmd,
  output logic [31:0] m_addr,
  output logic [3:0]  m_be,
  output logic [31:0] m_wdata,
  input  logic        m_ack,
  input  logic        m_wnext,
  input  logic        m_rvalid,
  input  logic [31:0] m_rdata,
  output logic        irq,
  output logic        busy
);

  function automatic logic [31:0] bswap(input logic [31:0] x);
    bswap = {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction

  // ------------------------------------------------------------------ registers
  logic [31:2] dsp;
  logic [31:0] dsps, temp;              // MMOV source and destination pointers
  logic [23:0] dbc;                     // MMOV byte count; words still to write in [23:2]
  logic [7:0]  dcmd, dmode, dien, dcntl;
  logic        st_ssi, st_sir, st_iid;  // DSTAT interrupt bits
  logic        is_srst, is_sigp, is_sem;
  logic        halted, irq_l;
  logic [2:0]  cmd;
  logic [7:0]  lat;
  logic [31:8] bar1;
  logic [21:0] rd_left;                 // words still to read

  wire dip = st_ssi | st_sir | st_iid;

  // ------------------------------------------------------------------ FIFO (16 words, MLAB)
  (* ramstyle = "MLAB, no_rw_check" *) logic [31:0] fifo [0:15];
  logic [3:0] wp, rp;
  logic [4:0] cnt;
  wire [31:0] fifo_q = fifo[rp];
  wire        f_push, f_pop;

  // ------------------------------------------------------------------ engine and port
  typedef enum logic [2:0] { E_IDLE, E_F0, E_F1, E_F2, E_CHK, E_MOVE, E_END } est_t;
  est_t est;

  localparam logic [1:0] K_FETCH = 2'd0, K_RD = 2'd1, K_WR = 2'd2;
  logic       p_out, p_acked;           // a transaction is outstanding; its acknowledge has come
  logic [2:0] p_beats;                  // beats still to come after the acknowledge
  logic [1:0] p_kind;
  logic       p_line;

  wire p_beat = p_acked && (m_rvalid || m_wnext);
  wire p_last = p_beat && p_beats == 3'd0;
  wire p_wack = m_req && m_ack && m_cmd == MC_WR;          // a single write is done at its acknowledge
  wire p_done = p_last || p_wack;

  wire [29:0] step = p_line ? 30'd8 : 30'd1;               // words a transaction moves

  assign f_push = p_beat && p_kind == K_RD;
  assign f_pop  = (p_beat && p_kind == K_WR) || p_wack;

  always_ff @(posedge clk) if (f_push) fifo[wp] <= m_rdata;
  assign m_wdata = fifo_q;

  // what the move asks for next (decided with the port idle)
  wire wr_line_ok = LINE && temp[4:2] == 3'd0 && dbc[23:5] != 19'd0;
  wire do_lwr = wr_line_ok && cnt >= 5'd8;
  wire do_swr = !wr_line_ok && cnt != 5'd0;
  wire do_lrd = LINE && dsps[4:2] == 3'd0 && rd_left[21:3] != 19'd0 && cnt <= 5'd8;
  wire do_srd = !do_lrd && rd_left != 22'd0 && cnt != 5'd16;
  // With the port idle, the words still to write are those still to read plus the FIFO's, so
  // while any remain one of the four above can go: a move with nothing to issue is done.

  // decode
  wire is_mmov  = dcmd[7:5] == 3'b110 && dcmd[4:1] == 4'd0;
  wire is_int   = INT_OP && dcmd[7:3] == 5'b10011;
  wire int_bad  = dbc[21] | dbc[20] | dbc[18] | dbc[17];    // carry test, INTFLY, data or phase compare
  wire mov_bad  = dbc == 24'd0 || dbc[1:0] != 2'd0 || dsps[1:0] != 2'd0 || temp[1:0] != 2'd0;

  assign busy = est != E_IDLE || p_out;

  // ------------------------------------------------------------------ CPU accesses
  wire s_go = s_req && !s_ack;
  wire s_wr = s_go && s_we;
  wire s_rd = s_go && !s_we;
  wire [4:0] s_w = s_addr;
  wire w_istat = s_wr && s_w == 5'h05 && s_be[3];
  wire w_dsp   = s_wr && s_w == 5'h0B && !busy;
  wire w_dsp3  = w_dsp && s_be[0];                         // byte 2F, DSP[31:24]: starts unless MAN
  wire w_dmode = s_wr && s_w == 5'h0E && s_be[3];
  wire w_dien  = s_wr && s_w == 5'h0E && s_be[2];
  wire w_dcntl = s_wr && s_w == 5'h0E && s_be[0];
  wire r_dstat = s_rd && s_w == 5'h03 && s_be[3];
  wire srst    = w_istat && s_wdata[30];

  // a start: DSP's top byte with MAN clear, or DCNTL.STD, when not halted
  wire start_dsp = w_dsp3 && !dmode[0];
  wire start_std = w_dcntl && s_wdata[2] && !halted && !busy && !srst;
  wire start     = (start_dsp || start_std) && !srst;

  // interrupt raised this cycle by the engine
  logic       e_int;
  logic [3:0] e_bits;                   // {SSI, SIR, -, IID}
  always_comb begin
    e_int = 1'b0; e_bits = 4'd0;
    case (est)
      E_F1: if (p_done && !is_mmov) begin
        if (is_int && !int_bad) begin
          if (dbc[19]) begin e_int = 1'b1; e_bits = {dcntl[4], 1'b1, 2'b00}; end
        end else begin e_int = 1'b1; e_bits = 4'b0001; end
      end
      E_CHK: if (mov_bad) begin e_int = 1'b1; e_bits = 4'b0001; end
      E_END: if (dcntl[4]) begin e_int = 1'b1; e_bits = 4'b1000; end
      default: ;
    endcase
  end

  always_ff @(posedge clk) begin
    s_ack <= s_go && !rst;
    c_ack <= c_req && !c_ack && !rst;

    // ---------------- PCI configuration
    if (c_req && !c_ack && c_we) begin
      if (c_idx == 6'd1 && c_be[0]) cmd <= c_wdata[2:0];
      if (c_idx == 6'd3 && c_be[1]) lat <= c_wdata[15:8];
      if (c_idx == 6'd5) begin
        if (c_be[1]) bar1[15:8]  <= c_wdata[15:8];
        if (c_be[2]) bar1[23:16] <= c_wdata[23:16];
        if (c_be[3]) bar1[31:24] <= c_wdata[31:24];
      end
    end

    // ---------------- CPU writes
    if (w_istat) {is_srst, is_sigp, is_sem} <= s_wdata[30:28];
    if (w_dmode) dmode <= s_wdata[31:24];
    if (w_dien)  dien  <= s_wdata[23:16];
    if (w_dcntl) dcntl <= s_wdata[7:0];
    if (w_dsp) begin
      if (s_be[3]) dsp[7:2]   <= s_wdata[31:26];
      if (s_be[2]) dsp[15:8]  <= s_wdata[23:16];
      if (s_be[1]) dsp[23:16] <= s_wdata[15:8];
      if (s_be[0]) dsp[31:24] <= s_wdata[7:0];
    end
    if (w_dsp3) halted <= 1'b0;
    if (r_dstat) begin
      st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;
      irq_l  <= 1'b0;
    end

    // ---------------- interrupts (after the read that clears: one raised in the same cycle stays)
    if (e_int) begin
      if (e_bits[3]) st_ssi <= 1'b1;
      if (e_bits[2]) st_sir <= 1'b1;
      if (e_bits[0]) st_iid <= 1'b1;
      if ((e_bits[3] && dien[3]) || (e_bits[2] && dien[2]) || (e_bits[0] && dien[0])) irq_l <= 1'b1;
      if (e_bits[2] || e_bits[0]) halted <= 1'b1;
      dcntl[2] <= 1'b0;                                    // STD stays set until an interrupt
    end

    // ---------------- memory port
    if (m_req && m_ack) begin
      m_req <= 1'b0;
      p_acked <= m_cmd != MC_WR;
      p_beats <= p_line ? 3'd7 : 3'd0;
    end else if (p_beat) begin
      p_beats <= p_beats - 3'd1;
    end
    if (p_done) p_out <= 1'b0;
    if (p_done) p_acked <= 1'b0;
    if (p_done && est != E_IDLE) begin                    // not after a software reset
      case (p_kind)
        K_FETCH: dsp  <= dsp + 30'd1;
        K_RD:    dsps[31:2] <= dsps[31:2] + step;
        default: temp[31:2] <= temp[31:2] + step;
      endcase
    end
    // the FIFO only moves during a move; a software reset empties it (DSTAT reads 0x80, DFE),
    // and the beats of a transaction it cut short are dropped
    if (srst) begin
      wp <= 4'd0; rp <= 4'd0; cnt <= 5'd0;
    end else if (est != E_IDLE) begin
      if (f_push) wp <= wp + 4'd1;
      if (f_pop)  rp <= rp + 4'd1;
      cnt <= cnt + {4'd0, f_push} - {4'd0, f_pop};
    end

    // ---------------- engine
    case (est)
      E_IDLE: if (start) est <= E_F0;
      E_F0, E_F1, E_F2: begin
        if (!p_out) begin
          m_req <= 1'b1; m_cmd <= MC_RD; m_addr <= {dsp, 2'b00}; m_be <= 4'hF;
          p_out <= 1'b1; p_kind <= K_FETCH; p_line <= 1'b0;
        end
        if (p_last) begin
          case (est)
            E_F0: begin {dcmd, dbc} <= bswap(m_rdata); est <= E_F1; end
            E_F1: begin
              dsps <= bswap(m_rdata);
              if (is_mmov) est <= E_F2;
              else if (e_int) est <= E_IDLE;
              else est <= E_END;                         // an INT not taken
            end
            default: begin temp <= bswap(m_rdata); est <= E_CHK; end
          endcase
        end
      end
      E_CHK: begin
        rd_left <= dbc[23:2];
        if (mov_bad) est <= E_IDLE;
        else est <= E_MOVE;
      end
      E_MOVE: if (!p_out && !p_done) begin
        if (do_lwr || do_swr) begin
          m_req <= 1'b1; m_cmd <= do_lwr ? MC_LINE_WR : MC_WR; m_addr <= {temp[31:2], 2'b00}; m_be <= 4'hF;
          p_out <= 1'b1; p_kind <= K_WR; p_line <= do_lwr;
          dbc[23:2] <= dbc[23:2] - (do_lwr ? 22'd8 : 22'd1);
        end else if (do_lrd || do_srd) begin
          m_req <= 1'b1; m_cmd <= do_lrd ? MC_LINE_RD : MC_RD; m_addr <= {dsps[31:2], 2'b00}; m_be <= 4'hF;
          p_out <= 1'b1; p_kind <= K_RD; p_line <= do_lrd;
          rd_left <= rd_left - (do_lrd ? 22'd8 : 22'd1);
        end else begin
          est <= E_END;
        end
      end
      E_END: if (dcntl[4]) est <= E_IDLE;
             else est <= E_F0;
      default: est <= E_IDLE;
    endcase

    // ---------------- software reset: operating registers to their defaults; a
    // transaction in flight still completes on the port
    if (srst) begin
      est <= E_IDLE;
      dsp <= 30'd0; dsps <= 32'd0; temp <= 32'd0; dbc <= 24'd0; dcmd <= 8'd0;
      dmode <= 8'd0; dien <= 8'd0; dcntl <= 8'd0;
      st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;
      halted <= 1'b0; irq_l <= 1'b0;
    end

    if (rst) begin
      est <= E_IDLE;
      m_req <= 1'b0; m_cmd <= MC_RD; m_addr <= 32'd0; m_be <= 4'h0;
      p_out <= 1'b0; p_acked <= 1'b0; p_beats <= 3'd0; p_kind <= K_FETCH; p_line <= 1'b0;
      wp <= 4'd0; rp <= 4'd0; cnt <= 5'd0;
      dsp <= 30'd0; dsps <= 32'd0; temp <= 32'd0; dbc <= 24'd0; dcmd <= 8'd0; rd_left <= 22'd0;
      dmode <= 8'd0; dien <= 8'd0; dcntl <= 8'd0;
      st_ssi <= 1'b0; st_sir <= 1'b0; st_iid <= 1'b0;
      is_srst <= 1'b0; is_sigp <= 1'b0; is_sem <= 1'b0;
      halted <= 1'b0; irq_l <= 1'b0;
      cmd <= 3'd0; lat <= 8'd0; bar1 <= 24'd0;
    end
  end

  // ------------------------------------------------------------------ read data
  always_ff @(posedge clk) begin
    if (s_rd) begin
      case (s_w)
        // DFE: the FIFO is empty whenever DSTAT may be read (the manual allows only ISTAT while
        // SCRIPTS run, every move completes, and a software reset empties it)
        5'h03: s_rdata <= {1'b1, 3'b000, st_ssi, st_sir, 1'b0, st_iid, 24'd0};
        5'h05: s_rdata <= {1'b0, is_srst, is_sigp, is_sem, 3'b000, dip, 24'd0};
        5'h07: s_rdata <= RD_ALL ? bswap(temp) : 32'd0;
        5'h09: s_rdata <= RD_ALL ? bswap({dcmd, dbc}) : 32'd0;
        5'h0B: s_rdata <= bswap({dsp, 2'b00});
        5'h0C: s_rdata <= RD_ALL ? bswap(dsps) : 32'd0;
        5'h0E: s_rdata <= {dmode, dien, 8'd0, dcntl};
        default: s_rdata <= 32'd0;
      endcase
    end
    if (c_req && !c_ack && !c_we) begin
      case (c_idx)
        6'd0: c_rdata <= 32'h0001_1000;
        6'd1: c_rdata <= {29'd0, cmd};
        6'd3: c_rdata <= {16'd0, lat, 8'd0};
        6'd5: c_rdata <= {bar1, 8'd0};
        default: c_rdata <= 32'd0;
      endcase
    end
  end

  assign irq      = irq_l && !dcntl[1];                    // DCNTL.IRQD masks the pin, not the interrupt
  assign win_en   = cmd[1];
  assign win_base = bar1;

endmodule
