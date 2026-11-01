// Constant-fold check of r3d_texup's arithmetic idioms (build rule: constant-fold
// new idioms): the expressions as written in the RTL, on constants. Expected
// values in the comments (Supermodel's mipXBase / mipYBase tables).
module cf_top (
  output logic [10:0] xb1,     // lv 1, x 2016: 1024 + 1008 = 2032
  output logic [10:0] xb5,     // lv 5, x 2016: 1984 + 63 = 2047
  output logic [10:0] xb11,    // lv 11, x 0: 2047
  output logic [9:0]  yb1,     // lv 1, y 992: 512 + 496 = 1008
  output logic [9:0]  yb9,     // lv 9, y 992: 1022 + 1 = 1023
  output logic [9:0]  yb4,     // lv 4, y 32: 960 + 2 = 962
  output logic [1:0]  sg_a,    // signed(5 - 7) <= 2: 1
  output logic [1:0]  sg_b,    // signed(0x10 - 0x7FFF0) = 32 <= 2: 0
  output logic [1:0]  sg_c,    // signed(0x40003 - 0x40000) = 3 <= 2: 0
  output logic [5:0]  tw16,    // 16-bit, 8 x 8: 32
  output logic [5:0]  tw8,     // 8-bit, 2 x 2: 1
  output logic [5:0]  tw8b,    // 8-bit, 8 x 4: 8
  output logic [5:0]  to16,    // 16-bit, ltx 3, yy 5, xx 6: 16 + 1 + 6 = 23
  output logic [5:0]  to8,     // 8-bit, ltx 2, yy 5, xx 7: 4 + 3 = 7
  output logic [5:0]  to8b,    // 8-bit, ltx 1, yy 7, xx 1: 3
  output logic [9:0]  tc,      // 1 << (12 - 3) = 512
  output logic [2:0]  xl       // (1 << 2) - 1 = 3
);
  function automatic logic [10:0] xbase(input logic [3:0] lv, input logic [10:0] hx);
    xbase = (lv == 4'd0) ? hx : (11'd0 - (11'd1 << (4'd11 - lv))) + (hx >> lv);
  endfunction
  function automatic logic [9:0] ybase(input logic [3:0] lv, input logic [9:0] hy);
    ybase = (lv == 4'd0) ? hy : (lv >= 4'd10) ? 10'd1023 : (10'd0 - (10'd1 << (4'd10 - lv))) + (hy >> lv);
  endfunction
  assign xb1  = xbase(4'd1, 11'd2016);
  assign xb5  = xbase(4'd5, 11'd2016);
  assign xb11 = xbase(4'd11, 11'd0);
  assign yb1  = ybase(4'd1, 10'd992);
  assign yb9  = ybase(4'd9, 10'd992);
  assign yb4  = ybase(4'd4, 10'd32);
  localparam int PW = 19;
  function automatic logic sg(input logic [PW-1:0] f, input logic [PW-1:0] t);
    sg = $signed(f - t) <= $signed(PW'(2));
  endfunction
  assign sg_a = {1'b0, sg(19'd5, 19'd7)};
  assign sg_b = {1'b0, sg(19'h10, 19'h7FFF0)};
  assign sg_c = {1'b0, sg(19'h40003, 19'h40000)};
  function automatic logic [5:0] tw(input logic b16, input logic [1:0] ltx, input logic [1:0] lty);
    logic [2:0] lt_sum;
    lt_sum = {1'b0, ltx} + {1'b0, lty};
    tw = b16 ? (6'd1 << (lt_sum - 3'd1)) : (6'd1 << (lt_sum - 3'd2));
  endfunction
  assign tw16 = tw(1'b1, 2'd3, 2'd3);
  assign tw8  = tw(1'b0, 2'd1, 2'd1);
  assign tw8b = tw(1'b0, 2'd3, 2'd2);
  function automatic logic [5:0] toff(input logic b16, input logic [1:0] ltx, input logic [2:0] yy, input logic [2:0] xx);
    if (b16) toff = ({4'd0, yy[2:1]} << ltx) + {5'd0, yy[0]} + {3'd0, xx[2:1], 1'b0};
    else     toff = ({4'd0, yy[2:1]} << (ltx - 2'd1)) + {4'd0, xx[2:1]};
  endfunction
  assign to16 = toff(1'b1, 2'd3, 3'd5, 3'd6);
  assign to8  = toff(1'b0, 2'd2, 3'd5, 3'd7);
  assign to8b = toff(1'b0, 2'd1, 3'd7, 3'd1);
  wire [3:0] lw = 4'd12; wire [1:0] ltx = 2'd3;
  assign tc = (10'd1 << (lw - {2'd0, ltx}));
  assign xl = (3'd1 << 2'd2) - 3'd1;
endmodule
