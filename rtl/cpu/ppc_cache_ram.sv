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
// One simple dual-port M10K for the caches: a write port and a read port,
// read address registered in the block, output unregistered (one cycle).
// Never cleared: the caches sweep their tags instead.
//
// A read of the address written on the same edge is undefined on the part
// (no_rw_check). The caches never rely on it, and simulation returns the
// inverted word in that case (PPC_CACHE_SIM) so that any reliance shows up as a wrong value.

module ppc_cache_ram #(
  parameter int W  = 8,
  parameter int D  = 1024,
  parameter int AW = $clog2(D)
)(
  input  logic          clk,
  input  logic          we,
  input  logic [AW-1:0] wa,
  input  logic [W-1:0]  wd,
  input  logic [AW-1:0] ra,
  output logic [W-1:0]  q
);

  (* ramstyle = "M10K, no_rw_check" *) logic [W-1:0] m [0:D-1];

  always_ff @(posedge clk) begin
    if (we) m[wa] <= wd;
`ifdef PPC_CACHE_SIM
    q <= (we && wa == ra) ? ~m[ra] : m[ra];
`else
    q <= m[ra];
`endif
  end

endmodule
