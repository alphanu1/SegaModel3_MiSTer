// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// THE SOUND LINK'S FPGA SIDE (study D-53): the main CPU's sound port out to the HPS, the HPS's
// records and commands in to the DSP engine (m3_scsp_dsp, inside), and the engine's four channels
// out to the core's audio, D samples after the HPS was let render them. One port on m3_ddr_arb,
// to arcsnd's block in DDR3 (version 2, arcsnd docs/PROTOCOL.md):
//
//   +0x00     {MIDI entries written 10 (above 6 zero), fs 16, D 16, "M" 8, version 4 (2), 0 2,
//             epoch 2}, written every 32 samples
//   +0x1000   MIDI ring, 512 entries {0 16, fs 16, 0 23, data port 1, byte 8}
//   +0x40000  record ring (the server's), 512 records of 512 bytes: 36 engine commands (MIXS of
//             both chips, then the four dry sums, all kind 0), then the tag {commands written
//             32; 0 14, epoch 2, p 16}; record p is taken up at fs = p
//   +0x100000 command ring (the server's), 131,072 entries (the attach alone is about 67,000)
//   an entry, and a record's command: {kind 3, 0 3, address 16, 0 9; 0 8, data 24}, the engine's
//   command as the HPS lays it out (arcsnd's dsplink_map)
//
// fs, the 44.1 kHz count, is 16 bits (the server widens it): exactly CLK_HZ / 44,100 clocks a
// sample on average, from a fraction counter reduced by the common factor. The server renders the
// board's sample once fs has passed it and files it as record p = its fs + D - 1. The bridge takes
// record p up at fs = p: the record, its commands straight into the engine as the beats come (one
// a clock, as the engine takes them) and its tag checked; then the commands up to the count the
// tag gives (all written before it); then the engine runs; and at the next tick, fs = p + 1, D
// samples after the server could first render it, the channels go to the audio. A record whose
// tag is not its own is not there yet: its commands went in for nothing (the next try writes
// them all again) and it is tried again each sample (the output holds); past SLIP (128) samples
// late it is skipped and the next tried at once, so the bridge only ever passes records that are
// not there. When it has fallen behind it runs samples back to back and the audio takes the
// newest. After a reset the record ring's tags are cleared to the old epoch's complement before
// anything is read; the epoch, kept through resets, moves on.
//
// AREA: no FIFO and no select on the engine's command port (DDR3's beats are the engine's
// commands); fs and p 16 bits, the clear counted on p; the addresses the block's high bits and
// the counters; no poll of the server; the engine's output is the audio.

`timescale 1ns/1ps

module m3_sndbridge #(
  parameter int          CLK_HZ    = 120_000_000,
  parameter int          D         = 256,               // the play delay, samples (D-55)
  parameter logic [24:0] BLOCK     = 25'h1E4_0000,      // the block's DDR3 word (byte 0x3F200000; the window starts at 0x30000000)
  parameter int          RING_BITS = 15,
  parameter bit          DBG       = 1'b0               // the bench's ports (0 in the core: tied off)
) (
  input  logic        clk,
  input  logic        rst,

  // the main CPU's writes to the MIDI port F0080000 (data) / F0080004 (control)
  input  logic        midi_we,
  input  logic        midi_data,          // 1: the data port
  input  logic  [7:0] midi_byte,

  // DDR3: one m3_ddr_arb port (m2_ddr3's contract)
  output logic        d_req,
  output logic        d_we,
  output logic [24:0] d_addr,
  output logic  [7:0] d_blen,
  output logic [63:0] d_din,
  output logic  [7:0] d_be,
  input  logic        d_wacc,
  input  logic        d_rvalid,
  input  logic        d_ack,
  input  logic [63:0] d_dout,

  // the core's audio: (front + rear) / 2 a side
  output logic signed [15:0] audio_l,
  output logic signed [15:0] audio_r,

  // for the bench (DBG): sim_ff skips to the next tick while nothing happens
  input  logic        sim_ff,
  output logic        dbg_idle,
  output logic [15:0] dbg_cyc,
  output logic        dbg_tick,
  output logic [15:0] dbg_fs,
  output logic [15:0] dbg_p,
  output logic        dbg_pop,          // dbg_q holds channel dbg_ch (front L, front R, rear L, rear R)
  output logic  [1:0] dbg_ch,
  output logic [17:0] dbg_q,
  output logic        dbg_skip,
  output logic  [1:0] dbg_epoch,
  output logic  [1:0] dbg_d18
);

  // ------------------------------------------------------------------ the sample clock
  function automatic int gcd(input int a, input int b);
    int x, y, t;
    x = a; y = b;
    for (int i = 0; i < 64; i++) if (y != 0) begin t = x % y; x = y; y = t; end
    return x;
  endfunction
  localparam int PER = CLK_HZ / 44100;                 // clocks a sample, at least
  localparam int REM = CLK_HZ % 44100;
  localparam int G   = (REM == 0) ? 44100 : gcd(44100, REM);
  localparam int MOD = 44100 / G, RR = REM / G;        // the fraction RR / MOD, reduced
  localparam int PB  = $clog2(PER + 1), EB = $clog2(MOD + 1);

  logic [PB-1:0] cyc;
  logic [EB-1:0] err;
  logic          tick;
  logic   [15:0] fs;
  assign tick = cyc == '0;
  wire           carry = err >= EB'(MOD - RR);         // the fraction passes 1 this sample
  always_ff @(posedge clk) begin
    if (rst) begin
      cyc <= PB'(PER - 1); err <= '0; fs <= '0;
    end else if (tick) begin
      fs <= fs + 16'd1;
      // PER clocks a sample, PER + 1 when the fraction carries (one adder, its constant chosen)
      err <= err + (carry ? EB'(RR - MOD) : EB'(RR));
      cyc <= carry ? PB'(PER) : PB'(PER - 1);
    end else if (DBG && sim_ff && cyc > PB'(1)) cyc <= PB'(1);
    else cyc <= cyc - 1'b1;
  end

  // the epoch: moves at every reset and is kept through it (power-up 0)
  /* verilator lint_off PROCASSINIT */
  logic [1:0] epoch = 2'd0;             // a power-up value (Quartus keeps it), never reset
  logic       rst_q = 1'b0;
  /* verilator lint_on PROCASSINIT */
  always_ff @(posedge clk) begin
    rst_q <= rst;
    if (rst && !rst_q) epoch <= epoch + 2'd1;
  end

  // ------------------------------------------------------------------ the DDR3 port and the samples
  typedef enum logic [2:0] { J_NONE, J_CLR, J_MIDI, J_HDR, J_REC, J_CMD } j_t;
  typedef enum logic [2:0] { S_CLR, S_WAIT, S_REC, S_CMD, S_GO, S_RUN, S_HOLD, S_POP } s_t;
  j_t          job;
  s_t          s;
  logic        hdr_pend;
  logic        m_full;
  logic  [8:0] m_byte;                  // {data port, byte}
  logic [15:0] m_fs;
  logic  [9:0] m_count;
  logic  [5:0] beat;                    // a record's beat
  logic [15:0] p;                       // the record taken up next: its fs (in the clear, its slot)
  logic [16:0] c, c_end;                // the command entries taken; the record's count
  logic        rec_ok, retry;           // the record's tag matched; wait for the next tick
  logic        want_rec, want_cmd, jd;  // a read asked of the port; the port's read finished
  logic  [2:0] pop;                     // the channels read: front L, rear L, front R, rear R
  logic  [1:0] sel_q;
  logic        skip;
  logic signed [15:0] f0;
  logic        e_go, e_busy, e_done, e_ready;
  logic [17:0] e_q;
  wire  [15:0] lag  = fs - p;           // >= 0: due; >= 128: past SLIP
  wire         due  = !lag[15];
  wire         late = !lag[15] && lag[14:7] != '0;

  // the engine: DDR3's beats are its commands, a record's first 36 and the commands up to the count
  wire rec_cmd = d_rvalid && job == J_REC && beat < 6'd36;
  wire cmd_cmd = d_rvalid && job == J_CMD && c != c_end;
  wire [1:0] e_sel = {pop[0], pop[1]};   // 0, 2, 1, 3
  m3_scsp_dsp #(.RING_BITS(RING_BITS)) u_dsp (
    .clk, .rst_n(!rst),
    .c_valid(rec_cmd || cmd_cmd), .c_ready(e_ready), .c_kind(d_dout[63:61]),
    .c_addr(d_dout[56:41]), .c_data(d_dout[23:0]),
    .go(e_go), .busy(e_busy), .done(e_done), .out_sel(e_sel), .out_q(e_q), .dac18(e_d18));

  always_comb begin
    // the word written: the clear's (the old epoch's complement where a record's tag has its epoch),
    // the header, or a MIDI entry
    case (job)
      // (the clear's word is kind 7, which the engine ignores: the bridge takes a cleared record's
      // beats as commands whenever it looks at a slot not yet written, and kind 0 would write TEMP)
      J_CLR:   d_din = {3'd7, 43'd0, ~epoch, 16'd0};
      J_HDR:   d_din = {6'd0, m_count, m_fs, 16'(D), 8'h4D, 4'd2, 2'd0, epoch};   // m_fs: fs as the job began
      default: d_din = {16'd0, m_fs, 23'd0, m_byte};
    endcase
  end
  assign d_be = 8'hFF;
  wire [16:0] mix = {f0[15], f0} + {e_q[17], e_q[17:2]};   // front + rear, halved (floor)

  always_ff @(posedge clk) begin
    if (rst) begin
      s <= S_CLR; p <= 16'(D - 1 - 512); c <= '0; c_end <= '0; rec_ok <= 1'b0; retry <= 1'b0;
      want_rec <= 1'b0; want_cmd <= 1'b0; jd <= 1'b0; pop <= '0; sel_q <= '0;
      job <= J_NONE; d_req <= 1'b0; d_we <= 1'b0; d_addr <= BLOCK; d_blen <= 8'd1; beat <= '0;
      hdr_pend <= 1'b0; m_full <= 1'b0; m_count <= '0;
      e_go <= 1'b0; audio_l <= '0; audio_r <= '0; skip <= 1'b0;
    end else begin
      e_go <= 1'b0; skip <= 1'b0; jd <= 1'b0; sel_q <= e_sel;
      if (tick) begin
        retry <= 1'b0;
        if (fs[4:0] == 5'd31) hdr_pend <= 1'b1;
      end
      // the sound port: one byte held until it is out (the CPU waits 320 us between bytes)
      if (midi_we && !m_full) begin m_full <= 1'b1; m_byte <= {midi_data, midi_byte}; m_fs <= fs; end

      // ---- the DDR3 port: a job at a time, the clear first, then MIDI, the header, the reads
      if (job == J_NONE) begin
        beat <= '0;
        d_req <= s == S_CLR || m_full || hdr_pend || want_rec || want_cmd;
        if (s == S_CLR) begin
          job <= J_CLR; d_we <= 1'b1; d_blen <= 8'd64; d_addr <= BLOCK | 25'({1'b1, p[8:0], 6'd0});
        end else if (m_full) begin
          job <= J_MIDI; d_we <= 1'b1; d_blen <= 8'd1; d_addr <= BLOCK | 25'({7'd1, m_count[8:0]});
        end else if (hdr_pend) begin
          job <= J_HDR; d_we <= 1'b1; d_blen <= 8'd1; d_addr <= BLOCK; m_fs <= fs;
        end else if (want_rec) begin
          job <= J_REC; want_rec <= 1'b0; d_we <= 1'b0; d_blen <= 8'd37; d_addr <= BLOCK | 25'({1'b1, p[8:0], 6'd0});
        end else if (want_cmd) begin
          // to the next 64-entry boundary (a 68000 clearing a ring sends one a sample or more, and
          // the attach 70,000 at once); the entries past the record's count are not used
          job <= J_CMD; want_cmd <= 1'b0; d_we <= 1'b0; d_blen <= {1'b0, 7'd64 - {1'b0, c[5:0]}}; d_addr <= BLOCK | 25'({1'b1, c});
        end
      end else if (d_ack) begin
        d_req <= 1'b0; job <= J_NONE;
        case (job)
          J_CLR: begin
            // the ring's 512 slots of 64 beats, p counting them from D - 1 - 512: done, it is D - 1,
            // the first record looked for (a session files its first at or after it; the bridge
            // passes the missing ones between)
            p <= p + 16'd1;
            if (p[8:0] + 9'd1 == 9'(D - 1)) begin s <= S_WAIT; hdr_pend <= 1'b1; end
          end
          J_MIDI: begin m_full <= 1'b0; m_count <= m_count + 10'd1; end
          J_HDR:  hdr_pend <= 1'b0;
          default: jd <= 1'b1;
        endcase
      end
      // read beats: a record's tag checked; commands taken, up to the count
      if (d_rvalid) beat <= beat + 6'd1;
      if (d_rvalid && job == J_REC && beat == 6'd36) begin
        rec_ok <= d_dout[17:0] == {epoch, p};
        c_end <= d_dout[48:32];
      end
      if (cmd_cmd) c <= c + 17'd1;

      // ---- the samples
      case (s)
        S_WAIT: if (due && !retry) begin s <= S_REC; want_rec <= 1'b1; rec_ok <= 1'b0; end
        S_REC: if (jd) begin
          if (rec_ok) begin s <= S_CMD; want_cmd <= c != c_end; end
          else begin
            // not written yet: again next sample; past SLIP a gap, and the next record at once
            s <= S_WAIT;
            if (late) begin p <= p + 16'd1; skip <= 1'b1; end
            else retry <= 1'b1;
          end
        end
        S_CMD: if (c == c_end && !want_cmd && job == J_NONE) begin e_go <= 1'b1; s <= S_RUN; end
               else if (jd) want_cmd <= c != c_end;
        S_RUN: if (e_done) s <= S_HOLD;
        // the channels go out at the next tick, or at once if this sample is already late
        S_HOLD: if (tick || lag != '0) begin s <= S_POP; pop <= '0; end
        S_POP: begin
          // the channels read in the order front L, rear L, front R, rear R (each a clock behind its
          // select): (front + rear) / 2 a side, from the top 16 bits
          pop <= pop + 3'd1;
          case (pop)
            3'd1, 3'd3: f0 <= e_q[17:2];
            3'd2: audio_l <= mix[16:1];
            3'd4: begin audio_r <= mix[16:1]; p <= p + 16'd1; s <= S_WAIT; pop <= '0; end
            default: ;
          endcase
        end
        default: ;
      endcase
    end
  end

  logic [1:0] e_d18;
  assign dbg_d18   = DBG ? e_d18 : 2'd0;
  assign dbg_skip  = DBG ? skip : 1'b0;
  assign dbg_tick  = DBG ? tick : 1'b0;
  assign dbg_cyc   = DBG ? 16'(cyc) : 16'd0;
  assign dbg_fs    = DBG ? fs : 16'd0;
  assign dbg_p     = DBG ? p : 16'd0;
  assign dbg_pop   = DBG ? s == S_POP && pop != 3'd0 : 1'b0;
  assign dbg_ch    = DBG ? sel_q : 2'd0;
  assign dbg_q     = DBG ? e_q : 18'd0;
  assign dbg_epoch = DBG ? epoch : 2'd0;
  assign dbg_idle  = DBG ? job == J_NONE && !m_full && !hdr_pend && !midi_we &&
                           ((s == S_WAIT && (retry || !due)) || (s == S_HOLD && lag == '0)) : 1'b0;

  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, sim_ff, skip, e_d18, e_busy, e_ready, mix[0], d_wacc, d_dout[60:57], d_dout[40:24], d_dout[31:18], sel_q};   // writes are one beat, or the clear's constant
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
