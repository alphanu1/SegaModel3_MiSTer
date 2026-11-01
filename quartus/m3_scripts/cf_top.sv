// Constant-fold check of m3_scripts' arithmetic idioms (build rule: constant-fold new idioms): the
// expressions as written in the RTL, on constants; quartus_map's "stuck at"
// report gives the bits Quartus computed. Expected values in the comments.
module cf_top (
  output logic [29:0] an_line,   // 0x3FFFFFF8 + step(8) -> 0 (wraps)
  output logic [29:0] an_one,    // 0x00040003 + step(1) -> 0x00040004
  output logic [29:0] an_dsp,    // 0x3FFFFFFF + 30'd1 -> 0
  output logic [21:0] dec_line,  // 0x000010 - 8 -> 8
  output logic [21:0] dec_one,   // 0 - 1 -> 0x3FFFFF
  output logic [4:0]  cnt_a,     // 15 + 1 - 0 -> 16
  output logic [4:0]  cnt_b,     // 16 + 0 - 1 -> 15
  output logic [31:0] sw,        // bswap(0x12345678) -> 0x78563412
  output logic [2:0]  flags      // {8 >= 8, 0x0001F != 0 in [23:5], 8 <= 8} -> 7
);
  function automatic logic [31:0] bswap(input logic [31:0] x);
    bswap = {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction
  logic [31:0] ma1, ma2;
  logic p1, p0, push, pop;
  logic [23:0] dbc;
  logic [4:0] c15, c16;
  assign ma1 = 32'hFFFF_FFE0; assign ma2 = 32'h0010_000C; assign p1 = 1'b1; assign p0 = 1'b0;
  assign dbc = 24'h000040;    // 16 words in [23:2]
  assign c15 = 5'd15; assign c16 = 5'd16; assign push = 1'b1; assign pop = 1'b1;
  wire [29:0] step1 = p1 ? 30'd8 : 30'd1;          // as m3_scripts' step
  wire [29:0] step0 = p0 ? 30'd8 : 30'd1;
  wire [31:2] dsp = 30'h3FFF_FFFF;
  assign an_line = ma1[31:2] + step1;
  assign an_one  = ma2[31:2] + step0;
  assign an_dsp  = dsp + 30'd1;
  assign dec_line = dbc[23:2] - (p1 ? 22'd8 : 22'd1);
  wire [21:0] zero = 22'd0;
  assign dec_one  = zero - (p0 ? 22'd8 : 22'd1);
  assign cnt_a = c15 + {4'd0, push} - {4'd0, p0};
  assign cnt_b = c16 + {4'd0, p0} - {4'd0, pop};
  assign sw = bswap(32'h1234_5678);
  wire [23:0] d2 = 24'h0003E0;
  assign flags = {c16 - 5'd8 >= 5'd8, d2[23:5] != 19'd0, c16 - 5'd8 <= 5'd8};
endmodule
