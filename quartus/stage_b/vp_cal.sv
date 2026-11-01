// Virtual-pin calibration for the stage B standalone fits: 170 inputs
// registered to 170 outputs (340 virtual pins, 170 registers, no logic). Its
// ALM count less 85 (two registers an ALM) is what the pins cost.
module vp_cal (
  input  logic         clk,
  input  logic [169:0] a,
  output logic [169:0] q
);
  always_ff @(posedge clk) q <= a;
endmodule
