// Constant-fold check of m3_tilegen's arithmetic idioms (build rule: constant-fold new idioms): the
// expressions as written in the RTL, on constants; quartus_map's "stuck at"
// report gives the bits Quartus computed. Expected values in the comments.
module cf_top (
  output logic [9:0] xlo,     // rs16 = 0x1A5: 1024 - 2 * 0x1A5 = 182
  output logic [7:0] c_neg,   // SM 5-to-8 of 16 (131) + 2 x (-125) -> 0
  output logic [7:0] c_pos,   // 255 + 2 x 127 -> 255
  output logic [7:0] c_mid,   // 82 + 2 x 5 -> 92
  output logic [7:0] c_m2,    // 131 + 2 x (-3) -> 125
  output logic [7:0] p5,      // pal5bit(16) = 132
  output logic [8:0] wrap16,  // 5 - 16 mod 512 = 501
  output logic [8:0] xhi,     // 495 - 2 x 0x0B5 = 133
  output logic [3:0] nch      // min(8 - 3, 32 - 9, 495 + 1 - 489) = 5
);
  logic [15:0] rs16, rs16b;
  assign rs16  = 16'h01A5;
  assign rs16b = 16'h00B5;
  wire [10:0] xlo_neg = 11'd1024 - {1'b0, rs16[8:0], 1'b0};
  assign xlo = xlo_neg[9:0];
  assign xhi = 10'd495 - {rs16b[8:0], 1'b0};

  function automatic logic [7:0] col8(input logic [7:0] e, input logic [7:0] o);
    logic [9:0] sum;
    sum = {2'b00, e} + {o[7], o, 1'b0};
    if (sum[9])      col8 = 8'd0;
    else if (sum[8]) col8 = 8'd255;
    else             col8 = sum[7:0];
  endfunction
  assign c_neg = col8(8'd131, 8'h83);
  assign c_pos = col8(8'd255, 8'h7F);
  assign c_mid = col8(8'd82, 8'h05);
  assign c_m2  = col8(8'd131, 8'hFD);
  logic [4:0] v;
  assign v  = 5'd16;
  assign p5 = {v, v[4:2]};
  logic [8:0] mapx;
  assign mapx = 9'd5;
  assign wrap16 = mapx - 9'd16;

  logic [9:0] x, xh;
  logic [8:0] mx;
  assign x = 10'd489; assign xh = 10'd495; assign mx = 9'd3;
  wire [3:0] n_tile = 4'd8 - {1'b0, mx[2:0]};
  wire [5:0] n_span = 6'd32 - {1'b0, x[4:0]};
  wire [9:0] n_end  = xh - x + 10'd1;
  always_comb begin
    nch = n_tile;
    if ({2'b00, nch} > n_span) nch = n_span[3:0];
    if ({6'd0, nch} > n_end)   nch = n_end[3:0];
  end
endmodule
