// SPDX-License-Identifier: GPL-3.0-or-later
// ppc_pdec.sv's table generator and check (run.sh)
module eq (input logic [31:0] i_data, output logic bad);
  logic ua, ub, us, xs, az, yreg, rc, xer, fpw, fhi, fuc, fdbl, ill; logic [2:0] immk; logic [4:0] rk; logic [5:0] op;
  logic ua2, ub2, us2, xs2, az2, yreg2, rc2, xer2, fpw2, fhi2, fuc2, fdbl2, ill2; logic [2:0] immk2; logic [4:0] rk2; logic [5:0] op2;
  pdx3 a (.i_data, .ua, .ub, .us, .xs, .az, .yreg, .rc, .xer, .fpw, .fhi, .fuc, .fdbl, .ill, .immk, .rk, .op);
  ppc_pdec b (.w(i_data), .ua(ua2), .ub(ub2), .us(us2), .xs(xs2), .az(az2), .yreg(yreg2), .rc(rc2), .xer(xer2), .fpw(fpw2),
              .fhi(fhi2), .fuc(fuc2), .fdbl(fdbl2), .ill(ill2), .immk(immk2), .rk(rk2), .op(op2));
  assign bad = {ua, ub, us, xs, az, yreg, rc, xer, fpw, fhi, fuc, fdbl, ill, immk, rk, op} !=
               {ua2, ub2, us2, xs2, az2, yreg2, rc2, xer2, fpw2, fhi2, fuc2, fdbl2, ill2, immk2, rk2, op2};
endmodule
