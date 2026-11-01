// SPDX-License-Identifier: GPL-3.0-or-later
// ppc_core's carry-free DBAT compare (dbat_hit) against bat_match on x + y,
// for any BL, contiguous or not (generated: run.sh copies both functions)
module dbat_tb (input logic [31:0] x, y, input logic [14:0] bepi, input logic [10:0] bl, input logic valid,
                output logic bad, hit);
  function automatic logic bat_match(input logic [14:0] ea_hi, input logic [14:0] bepi,
                                     input logic [10:0] bl, input logic valid);
    bat_match = valid && (ea_hi[14:11] == bepi[14:11]) && (((ea_hi[10:0] ^ bepi[10:0]) & ~bl) == 11'd0);
  endfunction
  function automatic logic dbat_hit(input logic [31:17] x, input logic [31:17] y, input logic [31:17] s,
                                    input logic [14:0] bepi, input logic [10:0] bl, input logic valid);
    logic [14:0] um, rs, ok;                          // unmasked; a run's lowest; the bit's test
    logic [14:0] gx;
    um = {4'b1111, ~bl};
    for (int j = 0; j < 15; j++) begin
      rs[j] = um[j] && (j == 0 || !um[(j + 14) % 15]);
      gx[j] = (x[17 + j] & y[17 + j]) | ((x[17 + j] ^ y[17 + j]) & ~bepi[j]);
    end
    for (int j = 0; j < 15; j++)
      if (!um[j])     ok[j] = 1'b1;
      else if (rs[j]) ok[j] = s[17 + j] == bepi[j];
      else            ok[j] = (x[17 + j] ^ y[17 + j] ^ bepi[j]) == gx[(j + 14) % 15];
    dbat_hit = valid && &ok;
  endfunction
  wire [31:0] s = x + y;
  assign hit = bat_match(s[31:17], bepi, bl, valid);
  assign bad = dbat_hit(x[31:17], y[31:17], s[31:17], bepi, bl, valid) != bat_match(s[31:17], bepi, bl, valid);
endmodule
