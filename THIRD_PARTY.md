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
- Changed: `Model3.sv` adds the CPU bring-up system and drives `UART_TXD`;
  the core name in `CONF_STR`; the PLL gains a second output (`outclk_1`, the
  CPU clock) in `rtl/pll.v` and `rtl/pll/pll_0002.v`; `Model3.sdc` adds the CPU
  reset synchroniser's false path; `files.qip` lists this project's sources.

## Model 2 core (same author)

- Source: https://github.com/alphanu1/sega-model2-mister
- Taken: `rtl/dbg/m2_uart_tx.sv` as `rtl/dbg/m3_uart_tx.sv` (renamed, timescale
  removed); `tools/uart_capture.py` (comment changes only).
- Licence: GPL-3.0-or-later.
- Also taken, at `47edd9dc00`: `rtl/tgp/fp_mul.sv`, `fp_add.sv` and `fp_div.sv` as
  `rtl/real3d/r3d_fp_mul.sv`, `r3d_fp_add.sv` and `r3d_fp_div.sv` for the Real3D
  geometry (module names prefixed, timescale removed; used with SUBNORM = 1).

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
