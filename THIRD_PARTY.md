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
