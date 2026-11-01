# Third-party code

Everything in this repository not written for it, with its source, the exact
commit, its licence and what was changed. Reference clones that are read but
never copied (MAME, Supermodel, SloPPC603) are listed in the design study's
sources section instead.

## MiSTer framework (Template_MiSTer)

- Source: https://github.com/MiSTer-devel/Template_MiSTer
- Commit: `0874acdf053ff42c8dd179685d4ef95f577ce429` (2026-08-15)
- Licence: GPL-2.0-or-later (the template's `LICENSE`; individual `sys/` files
  carry their own headers)
- Taken: `sys/` unchanged; `Template.sv` as `Model3.sv`, `Template.qsf`,
  `Template.qpf`, `Template.sdc` and `Template.srf` as the `Model3.*` project
  files; `rtl/pll.v`, `rtl/pll.qip`, `rtl/pll/`; `rtl/mycore.v`, `rtl/lfsr.v` and
  `rtl/cos.sv` as `rtl/video/` (the test picture).
- Changed: `Model3.sv` is the stage-A system (`rtl/sys/m3_system.sv`), with the CPU
  bring-up system behind `M3_CPUTEST`; both drive `UART_TXD`;
  the core name in `CONF_STR`; the PLL gains a second output (`outclk_1`, the
  CPU clock, and the stage-A system's 100 MHz) and a third (`outclk_2`, 100 MHz at
  180 degrees, `SDRAM_CLK`) in `rtl/pll.v` and `rtl/pll/pll_0002.v`; `Model3.sdc` adds the CPU
  reset synchroniser's false path; `files.qip` lists this project's sources.

## Model 2 core (same author)

- Source: https://github.com/alphanu1/sega-model2-mister
- Taken: `rtl/dbg/m2_uart_tx.sv` as `rtl/dbg/m3_uart_tx.sv` (renamed, timescale
  removed); `tools/uart_capture.py` (comment changes only).
- Licence: GPL-3.0-or-later.
- Also taken, at `47edd9dc00`: `rtl/tgp/fp_mul.sv`, `fp_add.sv` and `fp_div.sv` as
  `rtl/real3d/r3d_fp_mul.sv`, `r3d_fp_add.sv` and `r3d_fp_div.sv` for the Real3D
  geometry (module names prefixed, timescale removed; used with SUBNORM = 1).

- Also taken, at `47edd9dc00`: `rtl/io/m2_93c46.sv` as `rtl/io/m3_93c46.sv` (renamed;
  a write-busy time `T_WP` added, sim/system/notes.md). `rtl/io/m3_io.sv` is written for
  Model 3 from `rtl/io/m2_315_5649.sv` (the same chip): the port semantics are the same,
  the bus lanes, port B, the DIP and analog defaults are Model 3's, and the port
  function is a mux.
- Also taken earlier, unchanged: `rtl/mem/m2_sdram.sv`, `rtl/video/m2_video_timing.sv`
  (used by the stage-A system, `rtl/sys/m3_system.sv`).

## Supermodel, Model 3 system behaviour (transcribed, stage A)

- Source: https://github.com/trzy/supermodel, commit `036af1c`
- Files: `Src/Model3/Model3.cpp` (the system registers, IRQ timing, Real3D status),
  `Src/Model3/JTAG.cpp` / `JTAG.h` (the Step 1.0 scan chain, ID codes and register
  lengths), `Src/Model3/IRQ.cpp`
- Licence: GPL-3.0-or-later, the same as this project.
- Taken: behaviour only, re-expressed in `rtl/sys/m3_bus.sv` and `rtl/sys/m3_jtag.sv`
  (no code copied); the instrumentation is `tools/mame-census/supermodel-bus-trace.patch`.

## MAME 0.289, Real3D geometry (behaviour transcribed)

- Source: https://github.com/mamedev/mame, tag `mame0289` (`f34f0250`)
- Files: `src/mame/sega/model3_v.cpp` (copyright-holders R. Belmont, Ville Linde)
  and the frustum clipper in `src/devices/video/poly.h` (Ville Linde, Aaron Giles)
- Licence: BSD-3-Clause, compatible with GPL-3.0-or-later; attribution kept in
  the file headers.
- Taken: the display-list walk, matrix stack, polygon decode, transform, light,
  clip, divide and viewport, re-expressed operation by operation in
  `sim/real3d/r3d_geo.cpp` so the RTL can be built to the same FP32 order, and the
  walk again as a state machine in `rtl/real3d/r3d_walker.sv`. No MAME code is
  copied verbatim; the MAME-side instrumentation is a patch in
  `tools/mame-census/mame0289-real3d-tris.patch`.
- Changed: vertex slots persist across models (MAME reads an uninitialised one,
  see `sim/real3d/m2-notes.md`); the reciprocal and the frustum set-up are the
  RTL's own arithmetic, checked against MAME's results.

## MAME 0.289, Real3D rasteriser (behaviour transcribed)

- Source: https://github.com/mamedev/mame, tag `mame0289` (`f34f0250`)
- Files: `src/mame/sega/model3_v.cpp` (copyright-holders R. Belmont, Ville Linde):
  `draw_opaque_triangles`, `draw_alpha_triangles`, the `draw_scanline_*` functions
  and `get_texture`; `src/devices/video/poly.h` (Ville Linde, Aaron Giles):
  `render_triangle` and `round_coordinate`; `src/emu/video/rgbutil.h` and
  `rgbutil.cpp` (Vas Crabb, Ryan Holtz): the generic `rgbaint_t` scale, blend and
  clamp, and the SSE2 `bilinear_filter`.
- Licence: BSD-3-Clause, compatible with GPL-3.0-or-later; attribution kept in
  the file headers.
- Taken: the draw order, plane set-up, scan conversion, the eight texel decodes,
  mirroring, the bilinear filter, the colour, shade and blend arithmetic,
  re-expressed in `sim/real3d/r3d_rast.cpp` (the MAME form, bit-exact on 800 frames)
  and as the basis of the RTL form in `sim/real3d/r3d_rast_rtl.cpp`. No MAME code is
  copied verbatim; the MAME-side instrumentation is
  `tools/mame-census/mame0289-real3d-frames.patch`.
- Changed: the RTL form evaluates each interpolant exactly per pixel instead of
  accumulating floats, in fixed-point edges and block fixed-point planes
  (`sim/real3d/m3-notes.md`).

Supermodel (GPL-3.0-or-later) is instrumented only, through
`tools/mame-census/supermodel-real3d-frames.patch`; nothing of it is in the core.

## Supermodel, Real3D rasteriser behaviour (transcribed, D-19)

- Source: https://github.com/trzy/supermodel, commit `036af1c`
- Files: `Src/Graphics/New3D/R3DShaderCommon.h`, `R3DShaderTriangles.h`,
  `R3DShaderQuads.h`, `R3DFrameBuffers.cpp`, `New3D.cpp`, `PolyHeader.cpp`,
  `Model.h`, `TextureBank.cpp`
- Licence: GPL-3.0-or-later, the same as this project.
- Taken: texel decode, sampling and wrap rules, alpha test and dilation, the two
  translucency layers and their composite, depth rule, layered-polygon bias and
  stencil, back-face test, fog and scroll fog, mip level and level bases,
  re-expressed in `sim/real3d/r3d_rast_sm.cpp` as a behavioural model. No Supermodel
  code is copied verbatim.

### Real3D geometry, the shared-pool pass

- `r3d_fp_mul.sv` and `r3d_fp_add.sv` (Model 2 core, `47edd9dc00`, above) now run
  with SUBNORM = 0 by default (flush to zero; SUBNORM = 1 stays a parameter of
  `r3d_vpu`), and the adder with its PIPE_A2 align register. No code was changed.
- `r3d_fp_div.sv` is no longer instantiated: the clip's and the set-up's divides run
  on the engine's correctly rounded divide (`r3d_recip.sv`, my own).
- Supermodel (https://github.com/trzy/Supermodel, `036af1c` with my fixes,
  GPL-3.0-or-later): behaviour only, no code. The default frustum follows
  `CNew3D::RenderViewport` / `CalcViewport` and `Mat4::FrustumRZ`; node culling
  follows its block culling, as before.

## Model 2 core: the SCSP (same author)

- Source: https://github.com/alphanu1/sega-model2-mister, branch `model2a`, commit
  `47edd9dc00647e850ea10f4bc2691f4f668c22d0` (2026-10-31), from the local
  repository.
- Licence: GPL-3.0-or-later. The SCSP RTL is my own, written against MAME 0.289's
  `src/devices/sound/scsp.cpp`, `scsp.h`, `scspdsp.cpp`, `scspdsp.h`
  (BSD-3-Clause, copyright-holders ElSemi, R. Belmont; scsp.cpp thanks-to
  kingshriek), whose arithmetic, tables and order of operations it follows. The
  BSD-3 notice those files require:

      license:BSD-3-Clause
      copyright-holders:ElSemi, R. Belmont
      thanks-to: kingshriek

      Redistribution and use in source and binary forms, with or without
      modification, are permitted provided that the following conditions are
      met: (1) redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer; (2)
      redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution; (3) neither the
      name of the copyright holder nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE
      COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
      WARRANTIES ARE DISCLAIMED.

- Taken unchanged: `rtl/sound/scsp/m2_scsp.sv`, `m2_scsp_regs.sv`,
  `m2_scsp_dsp.sv`, the microcode `scsp_ucf.hex`, `scsp_uce.hex`, `scsp_uc.svh`,
  the tables `scsp_*.hex`, `scsp_tables.svh`, `scsp_lfonoise.svh`; and the benches
  and tools `sim/sound/tb_m2_scsp.cpp`, `tb_m2_scsp_dsp.cpp`,
  `tb_m2_scsp_level.cpp`, `scsp_ref.h` (MAME's two files ported standalone, BSD-3
  header kept), `scsp_randcap.cpp`, `scsp_tables_gen.cpp`, `scsp_level_check.cpp`,
  `scsp_area_top.sv`.
- Changed: `m2_scsp_cache.sv` gains a line-width parameter `LW` (default 16, as
  before) and `m2_scsp_level.sv` a slot-width parameter `SW` (default 5, as
  before), so the two-chip engine can put the chip in their addresses;
  `sim/sound/scsp_uasm.py` gains a `dual` mode that writes the two-chip engine's
  microcode (`scsp_d_ucf.hex`, `scsp_d_uce.hex`, `scsp_d_uc.svh`) and leaves the
  single-chip output byte-identical.
- New for Model 3, derived from the above: `rtl/sound/scsp/scsp_dual.sv` (one engine
  for both chips) and `scsp_dual_dsp.sv` (one DSP for both, from `m2_scsp_dsp.sv`),
  `scsp_pair.sv` (the board's view, either build), `sim/sound/tb_scsp_dual.cpp`, `scsp_dspmix.py`
  and `scsp_dual_harness.sv`, `m3_scsp_cap.lua`, `scsp_capsqueeze.py`.
- What differs on Model 3 (MAME 0.289 `model3.cpp`, BSD-3-Clause, R. Belmont, Ville
  Linde, read for the board wiring): two SCSPs on the 68000's bus (0x100000 and
  0x300000), each with its own 512 KB of sound RAM (0x000000 and 0x200000); only
  the first drives the 68000's interrupts and the MIDI link (`scsp2` has neither
  callback); the 68000's map adds a sample-ROM bank latch at 0x400000. The chip,
  its clock (22.5792 MHz) and its DSP are the same.

## Model 2 core: memory and video for the M2.5 viewer (same author)

- Source: https://github.com/alphanu1/sega-model2-mister (local clone
  `/home/ben/source/sega-model2-mister`)
- Commit: `47edd9dc00647e850ea10f4bc2691f4f668c22d0`
- Licence: GPL-3.0-or-later.
- Taken unchanged: `rtl/mem/m2_sdram.sv` (CAS 2, the 128 MB module's dual-chip
  handling R891, eight-word reads R787), `rtl/mem/m2_ddr3.sv` (the DDRAM master),
  `rtl/video/m2_fb_read.sv` (the 3D layer's line scan-out from DDR3) and
  `rtl/video/m2_video_timing.sv` (MAME's 656 x 424 / 496 x 384 raster at 16 MHz),
  into `rtl/mem/` and `rtl/video/`; `sim/mem/sdram_model.sv` into `sim/viewer/`
  (kept for protocol checks; the viewer's bench models the SDRAM in C++).
- Followed, not copied: `m2_ddr3_arb.sv`'s grant rules (`rtl/viewer/m3v_ddr_arb.sv`
  generalises them to five masters), `m2_rom_loader.sv`'s FIFO and `ioctl_wait`
  gating (`rtl/viewer/m3v_loader.sv`), `m2_raster3d.sv`'s line look-ahead and
  `Model2.sv`'s pixel enable, SDRAM pin wiring and PLL phase (`rtl/viewer/m3v_core.sv`,
  `rtl/viewer/m3_viewer.sv`); `m2_ddr3.sv`'s consumer contract and its R362, R660
  and R661 rules (`rtl/mem/m3_ddr3p.sv`, the pipelined DDR3 master, new code,
  which the viewer and the DDR3 probe core use in place of `m2_ddr3.sv`).

## MiSTer framework PLL, for the M2.5 viewer

- From this repository's own copy of Template_MiSTer `0874acd` (`rtl/pll.v`,
  `rtl/pll.qip`, `rtl/pll/`), into `rtl/viewer/pll/`.
- Changed: `outclk_0` 100 MHz and `outclk_1` 100 MHz at 5000 ps (SDRAM_CLK,
  Model 2's arrangement), in `pll_0002.v` and the wizard's retrieval comments in
  `pll.v`.

## MiSTerDDR3Test (Robert Peip): the DDR3 probe core's variable clock

- Source: https://github.com/RobertPeip/MiSTerDDR3Test
- Commit: `1a11970d7befa7cf12f08bef422555958fe23f60` (2024-09-23)
- Licence: GPL-3.0 (the repository's `LICENSE`), compatible with GPL-3.0-or-later.
- Taken: `rtl/pll.v` and `rtl/pll/pll_0002.v` (the reconfigurable 125 MHz
  Altera PLL) as `rtl/probe/pll_var.v` and `rtl/probe/pll_var/pll_var_0002.v`,
  and the PLL rewrite sequence of `DDR3Test.sv` (M, C0 and K through the
  framework's `pll_cfg`: 125, 100, 85 and 62.5 MHz) into `rtl/probe/ddr3_probe.sv`.
- Changed: the modules renamed `pll_var` / `pll_var_0002`; the top brings
  `locked` out; a new minimal `rtl/probe/pll_var.qip`; the rewrite is started by
  a toggle from the probe (`cfg_tgl`, synchronised to CLK_50M) rather than by a
  status change, and the clock settings are reordered (0 = 100 MHz).

## Tile generator (M0-E): MAME and Supermodel behaviour, Model 2's design

- MAME 0.289 (https://github.com/mamedev/mame, tag `mame0289`, `f34f0250`,
  BSD-3-Clause, copyright-holders R. Belmont, Ville Linde): `src/mame/sega/model3_v.cpp`
  `draw_layer`, `screen_update_model3`, `MODEL3_TILE_INFO4/8`, `model3_palette_w`, the
  `char4_layout` / `char8_layout` decodes; and `src/emu/drawgfx.cpp`
  `gfx_element::decode`, `src/emu/tilemap.h` (code modulo the element count).
  Re-expressed statement by statement in `sim/tilegen/tg_ref.h` (`mame_literal`) and as
  switches of its `param` model; no MAME code is copied verbatim. The dump
  instrumentation is `tools/mame-census/mame0289-tilegen-dump.patch`.
- Supermodel (https://github.com/trzy/supermodel, `036af1c`, GPL-3.0-or-later):
  `Src/Model3/TileGen.cpp` `DrawLine`, `Draw4BitTilePair`, `Draw8BitTilePair`,
  `GetColour32` and the register accessors, `TileGen.h`'s `ColourOffsetRegister`, and
  the hardware notes in `Src/Graphics/Render2D.cpp` and `TileGen.h`. Re-expressed in
  `sim/tilegen/tg_ref.h` (`sm_literal`, `param`); behaviour only, no code. The dump
  instrumentation is `tools/mame-census/supermodel-tilegen-dump.patch`.
- Model 2 core (https://github.com/alphanu1/sega-model2-mister, `47edd9dc00`,
  GPL-3.0-or-later, same author): `rtl/video/m2_tile_fetch.sv`, `m2_tile_decode.sv`,
  `m2_tile_mixer.sv`, `m2_palette.sv`, `m2_char_cache.sv` read for their design (a
  fetch engine ahead of an emitter, line banks, skipping masked columns, R823, R906);
  nothing copied, because Model 3's tile generator is a different chip (see
  `sim/tilegen/notes.md`). `rtl/real3d/r3d_ram.sv` (this project) is reused for every
  memory.
