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
// FPSCR update. Combinational: the current FPSCR and what one instruction
// did, in; the new FPSCR, CR1 (for Rc=1) and the mcrfs field, out. The
// register itself is the core's.
//
//  - Sticky exception bits are ORed in; FX is set when any of them goes from
//    0 to 1, except for mtfsf and mtfsfi [PEM Table 2-4].
//  - FEX and VX are always recomputed and never written [PEM p.2-7].
//  - FR, FI, FPRF / FPCC are written when the unit says so.
//  - mtfsf: the fields FM selects, from frB[32-63]; FX from frB[32] when FM[0]
//    is set, not by the 0 -> 1 rule. mtfsfi: one field from IMM. mtfsb0 /
//    mtfsb1: one bit; bits 1 and 2 (FEX, VX) cannot be written. mtfsb1 that
//    sets an exception bit from 0 sets FX (it is not exempt). mcrfs: the field
//    goes to the CR, then its exception bits (not FEX, VX) are cleared
//    [PEM 8-122 .. 8-137].
//  - UNDEFINED reserved bit 20 [PEM p.2-9]: written by mtfsf / mtfsfi /
//    mtfsb and read back, as Supermodel's field masks do (study D-17).

module ppc_fpu_fpscr
  import ppc_fpu_pkg::*;
(
  input  fs_op_t      op,
  input  logic [31:0] fpscr_in,
  input  fpu_flags_t  flags,      // FS_FLAGS
  input  logic [7:0]  fm,         // mtfsf FM; fm[7] is FM[0], field 0
  input  logic [31:0] frb_lo,     // mtfsf: frB[32-63]
  input  logic [2:0]  fld,        // mtfsfi crfD, mcrfs crfS
  input  logic [3:0]  imm,        // mtfsfi IMM
  input  logic [4:0]  bitn,       // mtfsb0 / mtfsb1 crbD (IBM numbering)
  output logic [31:0] fpscr_out,
  output logic [3:0]  cr1,        // FX FEX VX OX of the new FPSCR
  output logic [3:0]  crf         // mcrfs: the field before it is cleared
);

  localparam logic [31:0] EXC_BITS = 32'h1FF8_0700;
  localparam logic [31:0] VX_BITS  = 32'h01F8_0700;
  localparam logic [31:0] FEXVX    = 32'h6000_0000;

  function automatic logic [31:0] summaries(input logic [31:0] f);
    logic [31:0] r;
    logic vx, fex;
    vx  = |(f & VX_BITS);
    fex = (vx & f[7]) | (f[28] & f[6]) | (f[27] & f[5]) | (f[26] & f[4]) | (f[25] & f[3]);
    r = f;
    r[30] = fex;
    r[29] = vx;
    return r;
  endfunction

  // the causes in FPSCR positions
  wire [31:0] causes = {3'b000, flags.exc.ox, flags.exc.ux, flags.exc.zx, flags.exc.xx,
                        flags.exc.vxsnan, flags.exc.vxisi, flags.exc.vxidi, flags.exc.vxzdz,
                        flags.exc.vximz, flags.exc.vxvc, 9'd0,
                        flags.exc.vxsqrt, flags.exc.vxcvi, 8'd0};

  // one nibble mask per field: field i is FPSCR bits 31-4i .. 28-4i
  function automatic logic [31:0] field_mask(input logic [2:0] i);
    return 32'hF000_0000 >> {i, 2'b00};
  endfunction

  logic [31:0] m, f;
  always_comb begin
    f   = fpscr_in;
    crf = 4'd0;
    m   = 32'd0;
    case (op)
      FS_FLAGS: begin
        if (|(causes & ~fpscr_in & EXC_BITS)) f[31] = 1'b1;
        f = f | causes;
        if (flags.frfi_we) f[18:17] = {flags.fr, flags.fi};
        if (flags.fprf_we) f[16:12] = flags.fprf;
        if (flags.fpcc_we) f[15:12] = flags.fprf[3:0];
      end
      FS_MTFSF: begin
        for (int i = 0; i < 8; i++)
          if (fm[7 - i]) m = m | field_mask(3'(i));
        m = m & ~FEXVX;
        f = (fpscr_in & ~m) | (frb_lo & m);
      end
      FS_MTFSFI: begin
        m = field_mask(fld) & ~FEXVX;
        f = (fpscr_in & ~m) | (({imm, 28'd0} >> {fld, 2'b00}) & m);
      end
      FS_MTFSB0: begin
        m = 32'h8000_0000 >> bitn;
        if (bitn != 5'd1 && bitn != 5'd2) f = fpscr_in & ~m;
      end
      FS_MTFSB1: begin
        m = 32'h8000_0000 >> bitn;
        if (bitn != 5'd1 && bitn != 5'd2) begin
          if (|(m & EXC_BITS & ~fpscr_in)) f[31] = 1'b1;
          f = f | m;
        end
      end
      FS_MCRFS: begin
        crf = 4'((fpscr_in >> (5'd28 - {fld, 2'b00})) & 32'hF);
        case (fld)
          3'd0:    m = 32'h9000_0000;
          3'd1:    m = 32'h0F00_0000;
          3'd2:    m = 32'h00F0_0000;
          3'd3:    m = 32'h0008_0000;
          3'd5:    m = 32'h0000_0700;
          default: m = 32'd0;
        endcase
        f = fpscr_in & ~m;
      end
      default: ;
    endcase
    fpscr_out = summaries(f);
  end

  assign cr1 = fpscr_out[31:28];

endmodule
