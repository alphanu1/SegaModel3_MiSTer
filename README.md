<!-- SPDX-License-Identifier: GPL-3.0-or-later
     Sega Model 3 core for MiSTer FPGA — Copyright (C) 2026 alphanu1 -->

# Sega Model 3 for MiSTer

An FPGA recreation of Sega's Model 3 arcade board for
[MiSTer](https://mister-devel.github.io/MkDocs_MiSTer/) on the DE10-Nano
(Cyclone V `5CSEBA6U23I7`). It follows my
[Model 1](https://github.com/alphanu1/sega-model1-mister) and
[Model 2](https://github.com/alphanu1/sega-model2-mister) cores on the same
chip.

**One core runs six games, on Step 1.0, 1.5 and 2.x boards:**

- **Virtua Fighter 3** (Japan, Revision D)
- **Sega Bass Fishing Deluxe** (USA)
- **Scud Race / Sega Super GT** (Deluxe)
- **The Lost World: Jurassic Park** (Revision A), with light guns from an
  analog stick or a mouse
- **Daytona USA 2: Battle on the Edge** (Japan, Revision A)
- **Sega Rally 2**

All six boot, play and have sound on my MiSTer. It is work in progress, not
a release: timing does not close yet, so a build can misbehave on another
board, and the list of what's left is below. There is no `releases/` folder
yet; every release `.rbf` will be tested on the board before it goes in.

## How it is split

Model 3 is far bigger than Model 2, and the Cyclone V is the same chip, so the
core is built to fit: about 41,300 of 41,910 ALM, every LAB in use.

- **In the FPGA:** the PowerPC 603e, the Real3D (geometry and rasteriser),
  the tile generator, the memory controllers, the system and PCI devices, the
  I/O board and the SCSP sound chips' effects DSPs.
- **On the ARM (the HPS):** the sound board's 68000, the two SCSPs' voices and
  the Digital Sound Board (Scud Race's Z80 board, Daytona 2's and Sega Rally
  2's 68000 board, with their MPEG music), in my fork of the
  [Arcade Sound Server](https://github.com/alphanu1/MiSTer_ArcadeSoundServer)
  (branch `model3`). The core and the server talk through a 2 MiB block of
  DDR3. Without the server the games run silent.

The DSPs stay in the FPGA because the whole sound board did not keep up on one
A9 core: the DSPs alone were about two thirds of the work, since each runs its
program every sample, playing or not. In the FPGA they cost about 570 ALM.

## Where it stands

| | |
|---|---|
| **CPU** | My own PowerPC 603e: integer, FPU, MMU with BATs and TLBs, the 603e's 16 KB D-cache with a bus snoop for the 53C810's DMA, and a 32 KB I-cache (twice the chip's, for speed). Checked against a reference model instruction by instruction, against the 603e manual first and MAME and Supermodel as cross-checks. The version register is per board: Step 1.x reads 0x00060103, Step 2.x 0x00071202. |
| **Real3D** | Display-list walker, geometry and a textured, mip-mapped rasteriser, built from MAME's and Supermodel's behaviour, checked frame by frame on recorded scenes (Sega Rally 2's attract: 57 frames, 0 triangles and 0 pixels wrong). VROM and the polygon store are in DDR3. |
| **Tile generator** | The 2D layers and palette, checked against both emulators' output. |
| **System** | Step 1.0, 1.5 and 2.x from one core, chosen by the MRA: the MPC105/106, the 53C810, the Step 2 DMA device, the 315-5881 security chip, JTAG, the EEPROM, the RTC and the backup RAM. |
| **Inputs** | Driving (wheel, pedals, the four-speed shifter, view and hand brake), fighting, light guns and fishing controls, from Model 2's input code and steering options, with Model 3's own light-gun and fishing profiles. |
| **Sound** | SCSP DSPs in the FPGA, everything else in the Arcade Sound Server (above). Volume on the OSD. |
| **Clock** | 120 MHz. The fit is about 3 ns short of setup at the slow corners, so this is a test clock until timing closes. |

### Speed on the board, at 120 MHz

- **Virtua Fighter 3:** 57-58 frames a second through the stages, with some
  scenes at 30, which is normal for it.
- **Sega Bass Fishing:** a solid 58.
- **Scud Race:** 57-58 in attract; the long views dip to 30 for about a
  second.
- **The Lost World:** 58 in attract.
- **Daytona USA 2:** swings between 60 and 30 in attract, and races at 30.
  A 130 MHz test build reached 60 in attract and in VF3, but not in a Daytona
  2 race.
- **Sega Rally 2:** tested at 130 MHz so far.

### What's left

- **Timing closure**, first: every corner at 120 MHz, then speed.
- **Daytona 2's race at 60.** The CPU on its own clock, and the second SDRAM
  as its own memory, are the next things to try.
- **Pictures:** Daytona 2's car shadow, Lost World's fog in the T-rex scene,
  Bass Fishing's water shadows and underwater tint, Scud Race's luminous
  polygons.
- **Sound:** Bass Fishing can hiss after loading it straight from another
  game.
- **The other Model 3 games**, which have not been tried.

`HANDOFF.md` has the day-to-day state and the measurements behind it.

## Playing it

1. Build the core (below) or use a release once there is one, as
   `/media/fat/_Arcade/cores/Model3.rbf`.
2. Put the MRAs from `mra/` in `/media/fat/_Arcade/` (or a folder inside it).
3. Put the MAME sets the MRAs name (`vf3.zip`, `bassdx.zip`, `scuddxo.zip`,
   `lostwsga.zip`, `daytona2.zip`, `srally2.zip`) in `/media/fat/games/mame/`.
   The sound server also looks in a set's parent zip.
4. For sound, install the Arcade Sound Server from my fork's `model3` branch
   and start it. It reads the sound ROMs from the same zips.

The OSD has the FPS counter (averaged over 1, 5 or 10 seconds), the volume,
the aspect ratio, the CPU board's DIP switches, and for the driving games the
pedals and steering options.

## Building

Quartus Prime **Lite 17.0.0 Build 595** is the only version I build with; the
MiSTer framework targets 17.0.x.

```
quartus_sh --flow compile Model3
```

The design fills the chip, so a build's timing depends on the fitter seed.
`Model3.qsf` builds without `MISTER_DUAL_SDRAM`, so the analog I/O board
works; one 128 MB SDRAM module is needed. `sys/` is MiSTer-devel's framework,
unchanged.

## Testing

Every block is simulated before it goes near the fitter, with Verilator and
iverilog:

```
make lint                          # the CPU and its bring-up system
make test                          # the CPU against its reference, and the bring-up system
make test_cpu_sub                  # the CPU with its caches, 200 seeds
make -f sim/system/system.mk ...   # the system blocks: bus, loader, I/O, security, JTAG, memory
```

The benches under `sim/` cover the CPU and FPU, the caches, the Real3D stage
by stage against models of MAME's and Supermodel's results, the tile
generator, the sound DSP and the system. `tools/mame-census/` holds the
patches I use to make MAME and Supermodel record bus traces, frames and
sound for the benches.

The board has the last word. When a bench and the board disagree, the board
wins, and the emulators are references for what the chips compute, not for
how fast or exactly how.

## ROMs

No ROM, piece of a ROM, or anything built from ROM data is in this
repository. The MRAs describe the layout; the bytes come from your own sets.

## Layout

```
Model3.sv          top level
rtl/cpu/           the PowerPC 603e: core, FPU, caches, MMU
rtl/real3d/        the Real3D: walker, geometry, rasteriser
rtl/video/         tile generator, compositor, video timing, frame buffer scan-out
rtl/sys/           system bus, loader, PCI devices, security, JTAG, extra memory
rtl/mem/           SDRAM and DDR3 controllers, DDR3 arbiter
rtl/io/            I/O board, inputs, EEPROM
rtl/sound/         the SCSP DSPs and the link to the sound server
rtl/dbg/           debug overlay, FPS counter, CPU bring-up system
sim/               benches, one directory per area
tools/             comparison scripts and emulator instrumentation
mra/               one MRA per game
docs/              the design study
```

## Documents

- **`docs/model3-step10-design-study.md`** is the main one: what fits, what has
  been checked against what, the decisions (§14) and, in §15, every finding,
  including the ones that turned out wrong, what I believed and how it was
  corrected.
- `HANDOFF.md`: the current state and everything open.
- `THIRD_PARTY.md`: everything not written for this core.

## Third party

Each is in `THIRD_PARTY.md` with its source, commit, licence and what was
changed:

- **[Template_MiSTer](https://github.com/MiSTer-devel/Template_MiSTer)**, the
  MiSTer framework (`sys/`, the project files and the PLL).
- **My [Model 2 core](https://github.com/alphanu1/sega-model2-mister)**: the
  SDRAM and DDR3 masters, the video timing, the floating-point units, the
  EEPROM, the inputs and the SCSP.
- **[MiSTerDDR3Test](https://github.com/RobertPeip/MiSTerDDR3Test)** by Robert
  Peip, for the DDR3 probe core's variable clock.
- **[MAME](https://www.mamedev.org/)** and
  **[Supermodel](https://github.com/trzy/supermodel)**, as references for what
  the hardware computes. Neither is linked or distributed; the 315-5881's
  cipher tables come from MAME (BSD-3-Clause).

## Licence

GPL-3.0-or-later. See `LICENSE`. Source files carry an SPDX header.
