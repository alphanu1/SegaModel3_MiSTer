# Sega Model 3 Step 1.0 — MiSTer core design study

Target: DE10-Nano, Cyclone V 5CSEBA6U23I7 — 41,910 ALM, 553 M10K (691 KiB), 112 DSP,
one SDRAM module (128 MB), 1 GB DDR3 on the HPS side.
Scope: **Step 1.0 only** — the 8 ROM sets that run on the original Model 3 board.
Started 2026-10-30. Decisions are in section 14, findings in section 15.

Evidence tags used throughout:

| Tag | Meaning |
|---|---|
| `[SRC]` | Read directly from source. File named. |
| `[CALC]` | Arithmetic from `[SRC]` figures. Working shown. |
| `[EST]` | Estimate. Method stated. Error bar is wide. |
| `[OPEN]` | Unknown. A gate in section 9 resolves it. |

Nothing in this document is measured on Model 3 hardware. No Step 1.0 RTL exists yet.
Figures marked as Model 2 come from my Model 2 core (`sega-model2-mister`) and its fit
reports.

---

## 1. Summary

1. **Step 1.0 is 8 sets, 3 titles** — Virtua Fighter 3 (3 revisions), VF3 Team Battle,
   Sega Bass Fishing (4 variants). MAME and Supermodel agree on the list independently. `[SRC]`
2. **The sound board is Model 2A's sound board with a second SCSP.** Same 68000 clock, same
   SCSP clock, same i8251 MIDI path, same 315-5649 I/O. This is the one block that
   transfers almost whole, from my own Model 2A RTL. `[SRC]`
3. **The hard part is the Real3D, not the CPU.** There is no dumped firmware to run. The
   core has to *reimplement behaviour* that Team Supermodel reverse-engineered, not
   execute the original code. Model 1 and Model 2 were different: there the geometry
   processor runs real, dumped microcode.
4. **Everything lives in SDRAM; DDR3 holds only the 3D layer (D-2, D-4).** ROMs are
   106.5 MiB for VF3 and 90.5 MiB for Bass Fishing. Bass Fishing fits a 128 MB module with
   room to spare. VF3 is **3.7–6.7 MiB over** once board state is counted, so something
   on the 3D side has to move to DDR3 with the 3D layer, or measured RAM sizes have to come
   in smaller than the address windows. Open question 2. `[CALC]`
5. **Decided: everything in fabric, 3D layer drawn into DDR3 (D-1, D-2).** The same
   approach as my Model 2 core. With Model 2's measured figures in place (M0-G, 8.1),
   everything except the PowerPC comes to about 29K–37K ALM, leaving roughly 4K–12K for
   the CPU under a practical 41K ceiling. My i960KB with its FPU measures 7.6K. The
   carried-over paper estimates had said 37.5K–60.5K, which left nothing. M10K is the
   other binding resource: Model 2 fits at 551/553. `[EST]` on the Model 3 deltas only.
6. **The PowerPC 603e is a new core.** None of my CPUs is a PowerPC; the i960KB in the
   Model 2 core is the only CPU I have built. The 603e is the largest unknown, and gate
   M0-A comes first. Clock plan (D-3): CPU 100 MHz, SDRAM 100 MHz, Real3D 50 MHz,
   integer ratios to keep clock-domain crossings down. Tight, and nothing in it is
   measured yet.
7. **Bring-up title: Virtua Fighter 3 (D-5).** It is the title that matters, and it
   exercises the 53C810 single-step quirk and the full 106.5 MiB ROM set from the start.
   That makes the SDRAM gap (7.3) a bring-up blocker, not a later problem: M0-B's Real3D
   RAM footprint has to be known before M1. Get Bass upright stays as the fallback
   reference, since it is the one set MAME runs cleanly. `[SRC]`

The partition is settled (section 14). M0-G (done, 8.1) says the non-CPU blocks leave
room for a CPU about the size of the i960. What is not settled is the CPU itself: M0-A
answers that before any integration work starts.

---

## 2. What "Step 1.0" means — the differences between steps

MAME's header defines the steps `[SRC model3.cpp lines 10-16]`:

| Step | PPC clock | 3D engine |
|---|---|---|
| 1.0 | 66 MHz | baseline |
| 1.5 | 100 MHz | faster |
| 2.0 | 166 MHz | even faster |
| 2.1 | 166 MHz | same as 2.0, differences unknown |

Beyond the clock, the source shows these concrete differences:

| Item | 1.0 | 1.5 | 2.0 / 2.1 | Source |
|---|---|---|---|---|
| CPU | PPC603E, 66 MHz, 1× 66 MHz bus | PPC603E, 100 MHz, 1.5× | PPC603R, 166 MHz, 2.5× | MAME `add_cpu_*`; Supermodel `Model3.cpp` 3160-3185 |
| PCI host bridge | MPC105 | MPC105 | MPC106 | MAME `add_pci_devices`; Supermodel |
| SCSI controller | NCR 53C810 on PCI slot 14 | same | not instantiated in MAME; Real3D DMA device instead | MAME `add_pci_devices` |
| Real3D PCI ID | 0x16C311DB | 0x16C311DB | 0x178611DB | Supermodel `Real3D.cpp` header |
| Real3D texture port (0x90000000) | **one word**: a VROM word address, block copied into the texture FIFO | three words (address, header, one of unknown meaning), uploaded at once | same as 1.5 | MAME `vrom_w`; Supermodel `singleWordTexturePort` |
| Polygon vertex position | **17.7 fixed point** | 13.11 | 13.11 | MAME `draw_model` |
| Culling node layout | **2 words shorter** | full | full | MAME `draw_block` (`offset = step < 0x15 ? 2 : 0`) |
| Comm board (M3COMM) | absent | present | present | MAME `model3_10` vs `model3_15`; Supermodel runs the net board only when stepping is not 1.0 |
| Security board (315-5881) | none | none | only on `_5881` configs | MAME |
| Sound extras | none | MPEG DSB on Scud Race | DSB2 on some | MAME `scud()`, `srally2()` |
| `max_quantum` | 600 Hz (tight CPU/device interleave) | default | default | MAME `model3_10` only |

Two things the sources dispute, both `[OPEN]`:

- **The 100 MHz clock for Step 1.5.** MAME's PCB notes, written from a Scud Race board and
  a VF3TB board, say neither carries a "Step 1.0" or "Step 1.5" marking, that the CPU
  board has no 50 or 100 MHz oscillator, and that the Scud Race CPU runs at 66 MHz. (The
  same notes list separate Step 1.5 CPU and video board part numbers, 837-12874 and
  837-12875, so Step 1.5 boards do exist.) MAME and Supermodel both still clock Step 1.5
  at 100 MHz.
  If the PCB notes are right, the 15 Step 1.5 sets may run at 66 MHz and the Step 1.0 core
  would play them with the 13.11 format and node-layout switches.
- **Texture-port word count on later steps.** MAME consumes three words per upload;
  Supermodel's comment says two.

Scud Race, Le Mans 24, The Lost World and Virtua Striker 2 are Step 1.5 in both emulators,
not Step 1.0 (R1).

---

## 3. Games

Both emulators agree. `[SRC model3.cpp lines 6326-6333; Supermodel Config/Games.xml]`

| Set | Title | Parent | MAME machine | MAME status note |
|---|---|---|---|---|
| `vf3` | Virtua Fighter 3 (Rev D) | — | `model3_10` | crashes sometimes; dips at startup; too slow in gameplay |
| `vf3c` | Virtua Fighter 3 (Rev C) | vf3 | `model3_10` | same |
| `vf3a` | Virtua Fighter 3 (Rev A) | vf3 | `model3_10` | same |
| `vf3tb` | Virtua Fighter 3 Team Battle | vf3 | `model3_10` | same; ROM comment says "step 1.0?" |
| `bassdx` | Sega Bass Fishing Deluxe | — | `model3_10` | "I/O board error (?)" |
| `getbassdx` | Get Bass: Deluxe (Japan) | bassdx | `model3_10` | "I/O board error (?)" |
| `getbassur` | Get Bass: Upright (Japan) | bassdx | `model3_10` | **works** |
| `getbass` | Get Bass: Standard (Japan) | bassdx | `getbass` (adds I/O CPU) | "I/O board error (?)" |

All eight carry `MACHINE_NOT_WORKING` in MAME. Supermodel's compatibility matrix lists all
eight with no major issue noted. Neither emulator has been run against these sets yet. For
rendering fidelity, Supermodel is the better oracle. `[SRC]`

Not in either emulator: **Boat Race GP**, which the Guru PCB notes list among the original
Model 3 games. No set exists to test against. `[SRC]`

### Per-title I/O

- **VF3 / VF3TB:** shared `model3` inputs, two players, four buttons and an 8-way stick.
- **Bass Fishing:** `Cast` and `Select` buttons, rod X/Y on AN1/AN0, reel on AN3, stick
  X/Y on AN5/AN4. Seven analog or digital channels in total. `[SRC input ports "bass"]`
- **`getbass` only:** a KL5C80A16 I/O CPU at 16 MHz (32 MHz / 2) with a 315-5296, a
  315-5649 and a 93C46 EEPROM, running a 64 KiB `iocpu` ROM. Only this set's ROM list has
  an `iocpu` region. The other three Bass sets have no I/O ROM in MAME. MAME's header
  flags `bassdx`, `getbass` and `getbassdx` alike with "I/O board error (?)", so even the
  set with the I/O CPU does not run cleanly there. `[SRC]`

---

## 4. Hardware breakdown

| Block | Part | Clock | MAME device | Notes |
|---|---|---|---|---|
| Main CPU | PowerPC 603e | 66 MHz, 66 MHz bus | `PPC603E` | FPU almost certainly used; the CPU writes float matrices into Real3D RAM |
| Host bridge / RAM | Motorola MPC105 | 66 MHz | `MPC105`, map type B | 8 MiB RAM (`work_ram` 0x800000) |
| DMA engine | NCR 53C810 | 33 MHz | `NCR53C810_PCI` at `pci:0e.0` | SCRIPTS used as a memory-move DMA engine. See 5.2 |
| GPU | Real3D Pro-1000 family | 33 MHz board oscillator; internal rates not documented | `MODEL3_REAL3D_PCI` + renderer | Reverse-engineered. See 5.3 |
| Tile generator | custom | 16 MHz pixel | MAME `draw_layer` | 4 layers. See 5.4 |
| Sound CPU | 68000 | 11.2896 MHz (45.1584 / 4) | `M68000` | 512 KiB program ROM |
| Sound chips | 2× SCSP | 22.5792 MHz (45.1584 / 2) | `SCSP` ×2 | 512 KiB RAM each |
| MIDI link | i8251 (uPD71051) | 8 MHz, 31.25 kHz baud clock | `I8251` | Main CPU to SCSP MIDI-in |
| I/O | Sega 315-5649 | — | `SEGA_315_5649` | 8 analog inputs, ports A-G |
| Backup | 128 KiB SRAM, 93C46 EEPROM, Epson RTC72421 | — | `NVRAM`, `EEPROM_93C46_16BIT`, `RTC72421` | |
| Cabinet LEDs | billboard | — | `SEGA_BILLBOARD` | Cosmetic; skip |

Video timing: `set_raw(32 MHz / 2, 656, 0..496, 424, 0..384)` = **57.52 Hz**, 24.39 kHz
horizontal. `[CALC 16e6 / (656 × 424)]` MAME's own comment says "57.5 Hz-ish, same as
Model 1/2/System 24?" — the horizontal rate matches the 24 kHz output my Model 1 and
Model 2 cores already produce, so the scaler chain is reusable.

### Memory map `[SRC model3.cpp; Supermodel Model3.cpp]`

| Address | Block |
|---|---|
| `00000000-007FFFFF` | RAM, 8 MiB |
| `84000000` | Real3D status |
| `88000000` | Real3D command port (a write submits the frame) |
| `8C000000-8C3FFFFF` | Culling RAM, 4 MiB |
| `8E000000-8E0FFFFF` | Display list RAM, 1 MiB |
| `90000000` | VROM / texture port (write only) |
| `94000000` | Texture FIFO, 1 MiB |
| `98000000-983FFFFF` | Polygon RAM, 4 MiB allocated in MAME; the CPU write window in `model3_10_mem` is only `98000000-980FFFFF` (1 MiB). Size in use `[OPEN]` |
| `9C000000` | Buffer config registers (4 words) |
| `C0000000-C00000FF` | 53C810 registers (Step 1.x); Lost World expects `C1000000` |
| `F0040000` | 315-5649 I/O |
| `F0080000` | i8251 UART and sound write |
| `F00C0000-F00DFFFF` | Backup SRAM, 128 KiB |
| `F0100000` | System registers (IRQ enable, IRQ status, CROM bank, JTAG) |
| `F0140000` | RTC |
| `F1000000-F10F7FFF` | Tile pattern RAM, 992 KiB |
| `F10F8000-F10FFFFF` | Tile name tables, 32 KiB |
| `F1100000-F111FFFF` | Palette, 128 KiB |
| `F1180000` | Tile generator registers |
| `F8FFF000` | MPC105 registers |
| `FF000000-FF7FFFFF` | Banked CROM, 16 banks of 8 MiB |
| `FF800000-FFFFFFFF` | Fixed CROM, 8 MiB. The low 6 MiB mirror CROM0 (MAME `ROM_COPY`); only the top 2 MiB are separate ROMs |

Sound 68000 map: RAM `000000`, SCSP1 regs `100000`, RAM2 `200000`, SCSP2 regs `300000`,
bank latch `400001` (bit 4 selects the sample bank), program ROM `600000`, sample ROM
`800000-FFFFFF`. MAME maps only each SCSP's RAM into the SCSP's own space; how sample ROM
reaches the SCSPs is `[OPEN]` (check with a MAME trace of the 68000 writes).

---

## 5. Subsystems and how each emulator models them

### 5.1 Main CPU — PowerPC 603e

**No PowerPC core exists in my stable.** My only CPU is the i960KB in the Model 2 core,
which runs at 40 MHz (Model 2 R744). One open 603e exists: SloPPC603
(github.com/kdedon/SloPPC603, GPL-2.0-or-later, reviewed at `419d8a4`). It has the MMU
VF3 needs but not the speed or the area (R21), so the 603e here is written from scratch
with SloPPC603 as a reference only (D-13), and is the largest unknown in the project. Section 8.1 carries no ALM figure for it, deliberately.

Clock plan (D-3): **CPU 100 MHz, SDRAM 100 MHz, Real3D 50 MHz.** 100 MHz is 1.5 times the
66 MHz the real chip runs at. That headroom is what lets a single-issue core match a part
that can dispatch two instructions per clock, if the core's CPI stays near 1.5 or below.
No core of mine has closed a CPU above 40 MHz, and my FP units run at 80 MHz in the Model 2
TGP (clk_sys, R744), so 100 MHz for a CPU that includes an FPU is a design target, not a
demonstrated figure. The part is a 5CSEBA6U23I7: speed grade 7, industrial temperature.
Cyclone V grades run from 6 (fastest) to 8 (slowest). My Kaneko16 SDRAM controller runs
at 96 MHz and Model 2's at 100 MHz, so SDRAM at 100 MHz is proven, but Model 3 puts many
more clients on it. `[OPEN]`

What a Step 1.0 CPU must provide that I can state from source:
- 66 MHz bus-visible behaviour with 8 MiB of RAM behind the MPC105.
- Floating point. `load_matrix` reads IEEE floats the CPU wrote to culling RAM.

What I cannot state yet: whether VF3 turns the MMU on, whether it uses BAT or TLB paths,
which FPU instructions it executes, whether it relies on the decrementer or timebase, and
whether it executes from RAM or straight out of CROM. (MAME's VF3 init carries four
commented-out NOP patches marked "what's this for?"; their purpose is unknown.) **Gate
M0-A1** answers this with an opcode, SPR and PC-region census from MAME across the eight
sets. A small ISA subset is the difference between a tractable core and an intractable
one, and the PC-region split decides how much instruction cache the core needs. The real
603e has 16 KiB instruction and 16 KiB data caches; a core that executes from CROM in
SDRAM without an instruction cache pays SDRAM latency on every fetch.

One census question has outsized effect on area: **single or double precision.** My
`fp_mul` and `fp_add` are IEEE single. The 603e FPU is double precision. If the eight sets
only ever execute single-precision operations (`fadds`, `fmuls` and so on) and no double
operation produces a result that differs from single rounding, the single-precision units
can be reused. If real double operations occur, the FPU grows by a 53-bit multiplier
and wider alignment. `[OPEN]`

MAME runs the PPC through its recompiler with `PPCDRC_COMPATIBLE_OPTIONS` and no fast-RAM
path, because "RAM is decoded by the MPC10x". `model3_10` also sets a 600 Hz maximum
quantum, so the CPU and devices interleave tightly in Step 1.0 only. `[SRC]`

### 5.2 PCI bridge and the 53C810

MAME instantiates the MPC105 with the work RAM pointer and an 8 MiB size, and maps board
devices ahead of the bridge's PCI windows. `[SRC machine_start]`

The 53C810 is the surprise. Supermodel's `53C810.cpp` header states `[SRC]`:

- Model 3 games use the SCRIPTS processor **only as a DMA engine** — memory-move and
  interrupt instructions in initiator mode.
- **VF3 does something odd:** it writes DSP, which starts SCRIPTS automatically, *then*
  sets single-step mode and expects an interrupt. Disabling auto-execution breaks
  Scud Race. The accepted workaround leaves invalid instructions being hit each time.
- Supermodel's save-state code notes that when the 53C810 register window is inactive,
  VF3 "waits on its DMA chain every frame and runs in slow motion". VF3 depends on the
  DMA completing every frame.

Consequences for the core:
1. A small SCRIPTS interpreter is required: block move, JUMP, CALL, RETURN, register
   read/write, INT. It does not need SCSI bus phases.
2. VF3's single-step behaviour is an unresolved hardware question, not just a bug in one
   emulator. Budget a gate for it.
3. **Interrupt routing disagrees.** MAME ORs the SCSI interrupt into the PPC external IRQ.
   Supermodel wires it as an NMI ("SCSI is actually a non-maskable interrupt"). `[OPEN]`

### 5.3 Real3D — the unknown

#### Clock and timing

The video board carries a single **33 MHz** oscillator, fanned out through a Pericom
PI49FCT3807S 1-to-10 clock driver. The Step 2 video board shows the same 33 MHz part.
`[SRC model3.cpp, video board notes]` Supermodel's status-packet notes agree on the time
base: the 23-bit `tot_clks` field "is a 33.33mhz clock value", the time the GPU took to
process the frame, which software uses to estimate frame rate. `[SRC Real3D.cpp]`

So 33.33 MHz is the one Real3D clock the games can observe. Neither source documents the
ASICs' internal pixel or geometry rate, or whether they multiply the clock on-chip, and I
am not giving a figure. The board notes mention a "Jupiter ASIC" ID check and Supermodel
names ASICs including "Mars"; which 315-58xx part is which is not established.

Consequence for the core: the status packet must report a plausible frame time in 33.33 MHz
ticks. That is cheap (count ticks from frame start to render-done), but whether any game
misbehaves on a wrong value is `[OPEN]`.

#### What the CPU sees

The CPU writes five regions in PCI space (table in section 4) and then writes the command
port. MAME's `command_w` flushes the texture FIFO and calls `submit`, and MAME renders at
that point. Supermodel says no command starts drawing: it renders once per emulated frame
from what is in memory, and treats the command-port write as "the CPU finished this
frame's data". `[SRC]`

Supermodel adds that the Real3D double-buffers: after a ping-pong flip about two-thirds
through the frame, writes to high culling RAM and polygon RAM must not alter the frame
being drawn. The sizes of the buffers come from the `9C000000` registers, which every game
examined writes at boot (reg 0 between 0x5FFC and 0xBFFC; reg 8 between 0x18000 and
0x40000). `[SRC Real3D.cpp]` Those values suggest the games use far less than the 4 MiB
culling and polygon windows; M0-B measures it, and it bears directly on the SDRAM budget
in section 7.

#### What MAME's renderer does, stage by stage `[SRC model3_v.cpp]`

1. **Display-list traversal.** Four priority passes over viewport nodes starting at
   `0x800000`. Nodes link to nodes, to models, or to lists of links. Nested lists deeper
   than a fixed limit are dropped (`m_list_depth > 2`).
2. **Matrix stack.** 256 entries of 4×4 floats. Each node applies a matrix from a table
   at `m_matrix_base_address`, or a translation.
3. **Polygon decode.** A 7-word header, then 3 or 4 vertices of 4 words each (x, y, z, and
   a packed u/v word). A header bit marks vertices reused from the previous polygon.
   Positions are 17.7 fixed point on Step 1.0. Normals are sign + 1.22 fixed point.
4. **Transform and light.** Full 4×4 vertex multiply, normal transform, one parallel light
   plus ambient, a "sun clamp". No fog, no spotlights.
5. **Clip.** Polygon clip against the view frustum (`frustum_clip_all`, a 10-vertex
   buffer).
6. **Project.** Divide by w, viewport transform.
7. **Triangle buffers.** 50,000 opaque and 15,000 alpha triangles per frame. Quads split
   into fans.
8. **Rasterise.** Perspective-correct textured scanlines, bilinear filtering,
   float Z-buffer cleared once per priority pass, then alpha triangles drawn back to front.
   Variants: solid, textured, colour-modulated, alpha-tested ("contour"), translucent.
9. **Textures.** Two sheets of 2048×1024 16-bit texels, 8 MiB total. Eight pixel formats
   (ARGB1555, several A4L4 and L8 variants, ARGB4444). Textures are decoded once into a
   cache keyed by position, size and format.
10. **Composite.** Tile layers with priority 0 draw first, then the 3D image, then tile
    layers with priority 1.

MAME's own TODO list: tilemap flash effect, fog, mipmapping, mipmap uploads smaller than a
tile, alpha value rechecks, spotlights, normal-vector transform. Backface culling is
computed and then forced off (`back_face = 0`). Depth bias is "TODO". The renderer is a
behavioural model, not a transcript of the silicon. `[SRC]`

#### Why this is a different problem from the Model 1 TGP

On Model 1 and Model 2 the geometry processor is a CPU executing dumped microcode, so
a faithful MB86233/4 reproduces the original behaviour for free. The Real3D has no such
firmware visible to us. Whatever correctness the core achieves comes from Supermodel's and
MAME's reverse engineering of the data formats. Where those are wrong, there is no
original program to fall back on. Verification therefore leans on image comparison, not
on instruction traces.

#### What a fabric implementation needs `[CALC from the MAME code above]`

| Stage | FP multiplies | Other |
|---|---|---|
| Vertex × view-projection | 16 | 12 adds |
| Normal transform | 9 | |
| Light | ~4 | clamp, scale |
| Perspective | 1 reciprocal, ~5 multiplies | |
| Viewport | ~3 | |
| **Per vertex** | **~37 multiplies, 1 reciprocal** | |

Illustrative throughput, not a requirement. For scale, the Model 2 TGP runs at 80 MHz
(R744), where 8 time-multiplexed multipliers is 640 M multiplies/s, or about 17.3 M
vertices/s. At 57.52 Hz that is roughly 301 K vertices per frame. MAME's triangle buffers
cap at 65 K triangles per frame, which is about 130 K vertices if every pair of triangles
is a quad, and up to 195 K if triangles share nothing.

Under D-3 the Real3D clock is **50 MHz**. Then 8 multipliers give 400 M multiplies/s,
about 10.8 M vertices/s, roughly 188 K vertices per frame: above the cap in the best
sharing case (130 K) and just under it if triangles share nothing (195 K). 16 multipliers
give about 376 K, which clears both. `[CALC]` **If real frames sit far below the cap, 8
multipliers are enough at 50 MHz; if not, 16 are.** M0-B measures it.

Fetch cost per polygon: a 7-word header (28 bytes) plus 16 bytes per new vertex, so 92
bytes for a quad with four new vertices and 60 bytes with two reused. `[CALC]`

### 5.4 Tile generator

Four layers, each 512×512 pixels from 8×8 tiles, 4 or 8 bits per pixel. `[SRC]`
- Per-layer enable and scroll X/Y (9 bits each) at `F1180060-6C`.
- Per-line row scroll (RAM offset `F6000`) and a per-line stencil mask in 32-pixel spans
  (offset `F7000`). Layers work in pairs: the mask selects which of the pair shows on
  each span, inverted for the odd layer.
- Per-layer bit depth and above/below-3D priority in register `F1180020`.
- Palette: 32 K entries, 1-5-5-5; a flag bit in the entry suppresses the pixel.
- Registers `F1180040` and `F1180044` (layer colour modulation) are stored but unused
  by MAME's renderer — the "tilemap flash effect" TODO.

This block is verifiable alone. Dump tile RAM, palette and registers from MAME at a known
frame, render, and diff pixels. It needs no CPU and no Real3D. `[EST: ~2K-3K ALM, using
Model 2A's S24TILE range as proxy]`

### 5.5 Sound

68000 plus two SCSPs on one bus. The main CPU talks to it through the i8251 at
31.25 kbaud, feeding the SCSP's MIDI input. `[SRC]`

MAME cheats here. `model3_sound_w` contains a `HACK: MIDI comms thru SCSP MCIEB?` that
fires a 1 ms timer interrupt when the UART control register's bit 5 is set. The RTL
must implement the real MIDI path, not the hack. Treat as a known fidelity risk. `[SRC]`

Reuse: my Model 2A work already contains the 68000 (fx68k), my own SCSP
(`rtl/sound/scsp` on the `model2a` branch) at 22.5792 MHz and the i8251 on this exact
clock tree. A second SCSP is the delta. srg320's Saturn SCSP is a comparison reference
only — it carries no licence, and no code is taken from it (R6).

### 5.6 I/O

The 315-5649 is the same device as on Model 2A. EEPROM, RTC and backup SRAM are plain.
Bass Fishing's `getbass` set needs the KL5C80A16 I/O CPU: a Z80-compatible part with
on-chip ports. Z80 cores are common, but this part's port wiring is specific (EEPROM on
port 2). Only `getbass` ships a ROM for it. For the other three Bass sets the I/O path
is `[OPEN]`.

---

## 6. Reuse from Model 2 and Model 1

| Item | Reuse | Note |
|---|---|---|
| 68000, SCSP, i8251, 315-5649, 93C46, NVRAM | high | Same devices, same clocks as Model 2A. My own SCSP, not srg320's. `[SRC]` |
| 24 kHz video output, scaler chain | high | Same horizontal rate as my Model 1 and Model 2 cores. |
| SDRAM controller and arbiter | medium | Model 2's controller runs at 100 MHz. Model 3 adds large ROM traffic (CROM code, VROM uploads) and more clients. |
| DDR3 3D-layer write path and compositing (D-2) | high | Same approach as the Model 2 core. |
| Texture fetch and filtering | medium-high | Model 2 already does perspective-correct, bilinear-filtered texturing (`m2_texel_bl.sv`). New for Model 3: eight texel formats, two 2048×1024 sheets, uploads through the texture FIFO. |
| Depth | low | Model 2 and Model 1 paint back to front from a sort; neither has a Z-buffer. The Real3D's float Z-buffer is new hardware and new DDR3 or M10K traffic. |
| MB86233 FP units (`fp_mul`, `fp_add`) | medium | Both single-precision, running at 80 MHz in the Model 2 TGP. The Real3D needs transform, clip and divide, not a programmable DSP. |
| Lockstep trace diffing | medium | Works for the CPU. Does not work for Real3D (no reference program). |
| Gate-first discipline (Model 1 M0) | high | Use it. |

---

## 7. ROM and memory budget

Decision D-4: ROMs and all board state live in SDRAM. DDR3 carries only the 3D layer, as
on Model 2, and only where it has to.

### 7.1 ROM `[CALC from ROM_START regions]`

| Region | VF3 family | Bass Fishing |
|---|---|---|
| Fixed CROM (4 × 512 KiB) | 2 MiB | 2 MiB |
| Banked CROM (4 MiB mask ROMs) | 64 MiB (16 ROMs) | 48 MiB (12 ROMs) |
| Video ROM (`user3` + `user4`, 16 × 2 MiB) | 32 MiB | 32 MiB |
| SCSP samples | 8 MiB | 8 MiB |
| Sound program | 0.5 MiB | 0.5 MiB |
| **ROM total** | **106.5 MiB** | **90.5 MiB** |

The 6 MiB `ROM_COPY` mirror of CROM0 into the fixed window is address decoding, not
storage.

### 7.2 Working state `[CALC]`, measured for VF3 (M0-B)

The "window" column is the address space. The "VF3" column is what VF3 actually writes
over boot, attract and five minutes of play (M0-B, R13), rounded up to an allocation.

| Item | Window MiB | VF3 MiB | VF3 measured |
|---|---|---|---|
| Main RAM | 8 | 8 | — |
| Culling RAM | 4 | 2 | highest word 0x0778F7, 1.87 MiB span, 889 KiB distinct |
| Display list RAM | 1 | 0.125 | 108.5 KiB span |
| Polygon RAM | 1–4 | 1.125 | highest word 0x040571, 1.005 MiB span |
| Texture FIFO | 1 | 0.125 | at most 64 K texels (128 KiB) uploaded per frame |
| Texture sheets (2 × 2048 × 1024 × 16 bit) | 8 | 8 | not measured |
| Tile pattern RAM + name tables | 1 | 1 | — |
| Palette | 0.125 | 0 | lives in M10K (7.4) |
| Backup SRAM | 0.125 | 0.125 | — |
| Sound RAM (2 × 512 KiB) | 1 | 1 | — |
| **Total** | **25.2–28.2** | **21.5** | |
| 3D colour buffer (496 × 384 × 32 bit) | 0.73, **in DDR3 (D-2)** | | |
| 3D Z buffer, same size | placement `[OPEN]`, see 8.3 | | |

### 7.3 Totals against a 128 MiB module

| | ROM | State | **Total** | Margin |
|---|---|---|---|---|
| VF3, full windows | 106.5 | 25.2–28.2 | 131.7–134.7 | 3.7–6.7 MiB over |
| **VF3, measured footprint** | 106.5 | 21.5 | **128.0** | **zero** |
| VF3, measured, texture sheets in DDR3 | 106.5 | 13.5 | 120.0 | 8 MiB spare |
| Bass Fishing, full windows | 90.5 | 25.2–28.2 | 115.7–118.7 | 9.3–12.3 MiB spare |

Measured, VF3 fits a 128 MiB module with nothing to spare. That is not a plan: VF3 Team
Battle and the other revisions are not measured, and one more allocation granule tips
it over. Ways to get margin:

The Real3D's own RAMs are 3D rendering data in the D-4 sense: the CPU and the 53C810
write them, and only the Real3D reads them. My Model 1 core keeps its polygon payload in
DDR3 (`m1_ddram_payload`, Model 1 `9c05b79`) because the 16-bit SDRAM was the contended
bus and prefetch hides DDR3 latency up to ~400 cycles. The question for Model 3 is
bandwidth: the core's DDR3 port gives about **100 MB/s** usable.

DDR3 demand at 57.52 Hz `[CALC from M0-B]`:

| Client | MB/s, 16-bit layer | MB/s, 32-bit layer |
|---|---|---|
| 3D layer writes, each pixel once (Z on chip, 8.3) | 21.9 | 43.8 |
| 3D layer scan-out | 21.9 | 43.8 |
| **3D layer total** | **43.8** | **87.6** |
| Polygon RAM reads (41% of polygon fetch), 16 K / 32 K polygons a frame | 28 / 56 | 28 / 56 |
| Texture FIFO, in and out | 15 | 15 |
| Texture sheets (texture-cache fills) | not measured | not measured |

Latency, client by client. Polygon RAM is read model by model, in order, and the culling
walk names each model before its vertices are needed, so reads can be issued ahead
exactly as Model 1 does: latency is not the problem there. The culling walk itself is a
chain of dependent reads, so DDR3 latency lands on every node. Texture fetch is the
worst case: every cache miss stalls the pixel pipe, which is why Model 2 kept textures
in SDRAM. A per-polygon texture prefetch (the texture rectangle is known at setup, ahead
of the fill) could hide it, but nothing of mine has done that yet.

Bandwidth: the 3D layer alone takes 44–88% of the port. Polygon RAM fits only at the measured 16 K
peak with a 16-bit layer and fills the port completely at VF3's quoted 32 K. Texture
sheets were already ruled out for DDR3 on my Model 2 core (Model 2 R557): texture fetch
is latency-bound and HPS DDR3 latency lengthens every miss. The FIFO frees only
0.125 MiB.

My Model 1 core settles the 3D layer and the polygon pattern (R15): it draws the 3D at
60 Hz into on-chip bands, flushes each finished band to DDR3 at 16 bits a pixel, scans
out from DDR3, and keeps its polygon payload in DDR3 on the same port. That is about
45–50 MB/s of DDR3 traffic on the board. Model 3 with a 16-bit layer plus polygon RAM is
72 MB/s at the measured peak and 100 at 32 K polygons: the same pattern at 1.5–2× the
traffic.

**Decision D-6: polygon RAM and the texture FIFO go to DDR3 with the 3D layer; texture
sheets and culling RAM stay in SDRAM.** M0-C confirms the traffic on the board before
RTL depends on it. VF3 is then 126.75 MiB in SDRAM, 1.25 MiB spare. Ways to get more
margin:

1. **Pack the SDRAM to the measured footprint, not allocation granules.** Exact spans
   (culling 1.87, polygon 1.006, display list 0.106 MiB) save about 0.27 MiB. Not enough
   alone, and VF3 Team Battle is unmeasured.
2. **Measure the texture sheet footprint.** If VF3 leaves part of the 8 MiB unused, that
   is the largest saving left in SDRAM. Next census item.
3. **Dual SDRAM.** My Model 2 study (R35) records the Jaguar core's answer to the same
   problem: DDR3 failed on latency, dual SDRAM worked. A second SDRAM for the Real3D
   (VROM, texture sheets, culling and polygon RAM) solves capacity and SDRAM bandwidth
   at once, at the cost of requiring the dual-SDRAM add-on.

### 7.4 M10K ledger (691 KiB total) `[CALC]` / `[EST]`

M10K is as binding as ALM on this part. Model 2 fits at 553/553 blocks (s1246), and its
quad store alone is 153 blocks.

| Item | KiB | Basis |
|---|---|---|
| Matrix stack, 256 × 16 words × 4 B | 16 | `[CALC]` |
| Palette, 32 K × 16 bit | 64 | `[CALC]` |
| CPU instruction and data caches | up to 32 | 603e has 16 + 16 KiB; size set by M0-A1 |
| Texture cache, banked for four texels per clock | 32+ | Model 2's cache as the starting point |
| DDR3 3D-layer write buffers | `[OPEN]` | Model 2's measured figure (M0-G) |
| Z line or band buffers | `[OPEN]` | Depends on open question 1 |
| Triangle / vertex buffers between geometry and rasteriser | `[OPEN]` | Sized by M0-B's per-frame counts |
| SDRAM and DDR3 client FIFOs | `[OPEN]` | M0-G |
| **Known sub-total** | **144+** | |

Tile pattern RAM (992 KiB) and sound RAM (1 MiB) do not fit in M10K. They go to SDRAM
behind line buffers.

---

## 8. Fabric budget and partition options

### 8.1 Fabric ledger — measured where Model 2 has the block (M0-G)

Measured figures are "ALMs needed" from the Model 2 fitter's per-entity table: `main`
seed s1633 (39,272 / 41,910 ALM, 551 / 553 M10K, 101 / 112 DSP) and, for the SCSP, the 2C
seed s1629 (39,340 ALM). Rows marked `[EST]` are where Model 3 differs, with the measured
Model 2 block they start from.

| Block | Optimistic | Pessimistic | Basis |
|---|---|---|---|
| Framework: `sys/` (scaler, OSD, audio out, HDMI PLL) + `hps_io` | 6,025 | 6,025 | Measured: sys_top minus emu 5,399 + hps_io 626 |
| 68000 (fx68k) | 1,859 | 1,859 | Measured |
| 2 × SCSP | 3,857 | 3,857 | Measured: m2_scsp 1,928 each on 2C |
| Sound board glue (bank latch, i8251 link, mixing) | 357 | 600 | Measured: 2A sound board 4,136 minus fx68k and SCSP |
| Tile generator, 4 layers | 1,100 | 2,000 | `[EST]` from Model 2's tilemap, 1,087 measured |
| SDRAM controller and arbiter, more clients | 750 | 1,500 | `[EST]` from m2_sdram, 754 measured |
| DDR3 path outside the rasteriser | 200 | 300 | Measured: m2_ddr3, arbiters, frame-buffer read ≈ 200 |
| CPU bus bridge (MPC105 decode, PCI windows) | 600 | 1,000 | `[EST]` from m2_cpu_bridge, 613 measured |
| I/O, i8251, NVRAM, RTC, 53C810 SCRIPTS | 500 | 1,500 | `[EST]`; Model 2's I/O is a Z80 board (1,206) and not comparable |
| Real3D command, status, texture upload | 300 | 800 | `[EST]`, no Model 2 equivalent |
| Real3D geometry (traversal, matrix stack, decode, transform, clip, light, project) | 6,000 | 8,000 | `[EST]` from Model 2 geometry, 5,896 measured (m2_geometry 5,401 incl. clipper 1,657, + m2_geo 494) |
| Rasteriser and texturing, with Z test | 7,500 | 10,000 | `[EST]` from m2_raster3d, 8,021 measured, of which the quad store is 1,689. A Z-buffer removes the depth sort; Z test and eight texel formats are added |
| **Total without CPU** | **29,048** | **37,441** | |
| PowerPC 603e core (new build) | ? | ? | `[OPEN]` — M0-A2. For scale, my i960KB measures 7,559 ALM, 11 M10K, 7 DSP including its FPU |

Model 2 has been fitted at 41,415 ALM used, so ~41,000 is a working ceiling. Against it
the optimistic total leaves **about 12,000 ALM for the CPU** and the pessimistic one
**about 3,500**. An i960-sized 603e fits the optimistic case with room; it does not fit
the pessimistic one. The rows that decide it are the rasteriser and geometry deltas,
which M0-B's polygon and overdraw numbers narrow, and the CPU itself, which M0-A2
measures.

M0-B (R13) moves weight between the two Real3D rows: geometry needs only two multipliers
at 50 MHz, which puts it at or below the low end, while the rasteriser needs two pixels
per clock for the p99 frame, which puts it at or above the high end. The total is not
re-estimated until the rasteriser has a design.

The first draft of this table, carried over from the Model 2A paper estimates, totalled
37,500–60,500 and left nothing for a CPU (R10).

M10K from the same reports: framework 51, i960 11, fx68k 6, Model 2 tilemap 16,
texture path 37 (bilinear texel 28 + span 9), quad store 151. A Z-buffered Real3D
needs a short triangle FIFO rather than a 151-block sorted store; that is the largest
single M10K saving available, and the Z traffic it costs goes to DDR3 (open question 1).

### 8.2 Partition — decided (D-1, D-2)

| Option | CPU | Real3D geometry | Rasteriser | Status |
|---|---|---|---|---|
| **A. All fabric** | fabric | fabric | fabric, 3D layer drawn into DDR3 | **Chosen** |
| B. Fabric CPU, HPS geometry | fabric | ARM | fabric | Not taken. Kept for the record |
| C. HPS CPU | ARM | either | fabric | Not taken |

Option B would have saved 6-12K ALM but needed an ARM speed gate. Under A the remaining
levers are:
1. Fold the Real3D vertex pipeline onto few multipliers. The 5.3 arithmetic suggests 8
   multipliers have large headroom, which supports the low end of the 6,000-12,000 row.
   M0-B confirms with real polygon counts.
2. Time-multiplex the two SCSPs through one datapath. `[OPEN]` — depends on how many of
   the 512 clocks per 44.1 kHz sample the SCSP datapath actually uses (22.5792 MHz ÷
   44.1 kHz = 512).
3. Rasteriser at the low end of its range, which the Model 2 measurements can confirm.

Whether A fits is decided by M0-A (CPU size) and M0-G (measured figures).

### 8.3 Clock plan — where it is tight (D-3)

The three clocks are integer ratios of one another (100, 100, 50), so CPU, SDRAM and
Real3D can come from one PLL on phase-aligned outputs, and crossings between them need
no asynchronous FIFOs. Model 2 runs the same way (100 / 80 / 40 off one PLL) and depends
on the exact ratios for its SDRAM handshake. Real3D can run either on its own 50 MHz
output or on the 100 MHz clock with a clock enable every second cycle; the enable form
turns paths inside it into 2-cycle paths, though it only helps where the logic is written
to take two cycles. The clocks this does not remove: the 16 MHz pixel clock, the sound
clocks (11.2896 MHz 68000, 22.5792 MHz SCSP) and the 33 MHz 53C810 all stay separate
unless run as enables of a faster clock. `[OPEN]` Whether the sound clocks can be derived
as enables of the 100 MHz clock is not checked; 22.5792 MHz does not divide 100 MHz
evenly.

Three places carry the risk, in order of how little I can say about them today.

1. **CPU Fmax at 100 MHz with an FPU.** My FP units run at 80 MHz after a 3-stage split
   of `fp_add`. A 603e FPU at 100 MHz needs more stages, and each stage adds latency that
   raises CPI, which eats the 1.5× headroom over 66 MHz. Gate M0-A2.
2. **SDRAM at 100 MHz with many clients.** Peak for a 16-bit module is 200 MB/s; my
   Model 1 notes put usable at 120-140 MB/s. With D-4 every ROM read is on this port too.
   Clients: PPC cache fills from RAM and CROM, CPU writes into culling, display-list and
   polygon RAM, geometry reads of the same, VROM reads for texture uploads, texture-cache
   fills (unless the sheets move to DDR3, 7.3), tile pattern fetch (worst case 4 layers ×
   1 B × 10.96 M pixels/s = 44 MB/s `[CALC]`), and sound RAM and sample ROM for the 68000
   and two SCSPs. That is more clients than Model 2. Needs a deadline arbiter budget
   before integration (gate M0-C).
3. **Rasteriser at 50 MHz.** One pixel per clock is 50 M pixels/s against 10.96 M visible
   pixels/s, so a ceiling of **4.56× overdraw** counting every pixel that is Z-tested,
   not just drawn. `[CALC]` Bilinear filtering needs four texels per pixel, so reaching
   one pixel per clock needs four texel reads per clock, for example a texture cache
   banked by texel parity. At one texel per clock the ceiling falls to 1.14×, barely
   one full screen. M0-B's overdraw measurement decides whether 4.56× is enough.

---

### 8.4 Area techniques carried from Model 1 and Model 2

Third-party cores are measured as their authors built them. My Model 1 and Model 2 cores
found room no other core's IP takes, all cycle-identical on their benches. Every Model 3
block, the CPU first, is designed with these from the start, and any third-party figure
(SloPPC603's 10,415 ALM included) is read as a ceiling these can cut, not a floor.

| Technique | Measured | Where |
|---|---|---|
| Register file in MLAB: a bank per write port, a copy per read port, a live-value table and a one-clock bypass | −796 ALM, −716 registers | Model 1 `7d95cdd` (V60) |
| Whole-word register writes, merged before the write, instead of per-bit write masks | ~1,030 ALM of a 2,289 ALM file | Model 1 `957fba3`; −547 ALM fitted, `a7abcbf` |
| Small storage out of flip-flops: stacks, EEPROM, FIFOs into MLAB or M10K | 1,920 / 1,024 / 272 flops each | Model 1 `a5c905c`, `e643842`, `29a5fa0` |
| Duplicated arithmetic built once: one multiplier, one barrel shifter, one FP pack, address adders shared across instruction groups that never overlap | 12 DSP; several hundred ALM | Model 1 `86927fb` |
| Idle registers shared between groups that are never live together | in `86927fb` | Model 1 |
| Converters and operators time-shared where the schedule has idle cycles; Quartus merges identical expressions but cannot see a schedule | −1,507 ALM on 2A, −1,359 on 2C | Model 2 R952 |
| One shared FP pool for several units; the reciprocal on the pool | −652 ALM for the reciprocal | Model 1 `640eb4e`; Model 2 `m2_fp_pool` |
| MLAB and M10K traded against each other as whichever binds | 330 ALM of MLAB moved to M10K | Model 1 `29a5fa0` |
| Per-game or per-board logic behind compile-time parameters so unused decode folds away | — | Model 2 R948 |

What does not work, also measured: registers alone buy about nothing (they share ALMs
with logic, Model 2 R952); small multiplies forced into logic cost 1,650 ALM and failed
timing (Model 2 R975); a DSP reciprocal square root was not worth 199 ALM at 98% full
(Model 1 `746bfbc`).

Method, from the same work: measure each change with `quartus_map` on the full design and
a standalone map and fit of the changed module; prove identity with an old-against-new
miter on every cycle; and run real-code traces, because per-opcode fuzzing passed a SET1
that corrupted 31 of 32 bits (Model 1 `a7abcbf`).

For the 603e this points at: GPRs and FPRs in MLAB with a live-value table; BATs,
segment registers and SPRs in MLAB; one shared adder and shifter across the integer
groups; the FP datapath on a shared pool with single precision fast and double precision
iterated through the same multiplier; TLB and caches in M10K.

## 9. Milestones and gates

M0 gates are independent and need no integration. Each reports numbers, not opinions.

**M0-A1 — PPC census.**
Instrument MAME's PPC core across the eight sets, boot to attract, and log:
- every distinct opcode and SPR touched (the ISA subset),
- FPU operations split into single and double, and whether any double result differs
  from single rounding,
- whether the MMU is enabled, and whether translation uses BATs or the TLB,
- decrementer and timebase use,
- where the PC executes (RAM, fixed CROM, banked CROM) and how often it changes region,
- instructions and MAME cycles executed per frame, busy and idle.
Output: the spec for the new core, the instruction-cache requirement, and the
instructions per second the games need.

*Results, VF3 only (2026-10-30, R12).* Tooling in `tools/mame-census/`. Three runs: 10 s
and 180 s boot plus attract, and 360 s with a coin and play from frame 5,200 (14.9
billion instructions over 20,709 frames).
- **ISA:** 134 distinct opcode classes. Integer core plus `lmw`/`stmw`, `lwbrx`/`stwbrx`
  (PCI byte swaps), `lwarx`/`stwcx.`, `divw`, `mullw`, condition-register logic, `rfi`,
  `isync`, `sync`. No string instructions and no cache-block instructions seen.
- **MMU on.** 99.2% of instructions run with IR = DR = 1. All 16 BAT registers and SDR1
  are written at boot. VF3 also takes **software data-TLB misses**: `tlbld` 68,878 times
  in 180 s (about 7 per frame), with DMISS (SPR 976) read and RPA (SPR 982) written by
  the handler, plus `mfsrin`. No `tlbli`, so instruction fetch stays inside BATs. The
  core needs BATs, a small data TLB, the 603e miss exceptions and `tlbld`.
- **Timebase and decrementer used:** `mftb` about 950 per frame; DEC read in a wait
  loop, written about once per second.
- **Code runs from RAM.** Of 14.9 billion instructions, all but 2.6 million (the reset
  vector at `FFF00000`) execute from main RAM. CROM is never executed after boot; it is
  read as data. Total code footprint 138 KiB.
- **I-cache working set:** distinct 32-byte lines per frame p50 1,323 (41 KiB), p99 1,591
  (50 KiB), max 2,017 (63 KiB). Well beyond the real part's 16 KiB.
- **FP:** single-precision ops per frame p99 150,440 (about 8.7 M/s). Double-precision
  p99 4,177 per frame, and real: `frsqrte` → `fmul` → `fnmsub` is a double Newton-Raphson
  reciprocal square root, and 97% of those `fmul` results are not representable in
  single. A single-only FPU would change those results. The double rate is low enough
  for a slow, iterative double path.
- **Throughput:** MAME charges 1,147,344 cycles per frame (66 MHz ÷ 57.52). Busy
  instructions per frame, excluding VF3's vblank and decrementer wait loops: p50
  307,703, p90 500,804, p99 1,073,777, max 1,177,370. At 57.52 Hz the p99 frame needs
  61.8 M instructions/s; a 100 MHz core needs CPI ≤ 1.6 for that frame and ≤ 3.5 for
  the p90 frame. Instruction counts are the game's own path and hold on hardware; MAME's
  cycle charging does not, and is not used here.
- **Throughput at 60 Hz** (R19). MAME renders VF3 at 30 Hz, which halves the per-frame
  game work above. Supermodel renders every frame; its PowerPC census (idle-skip off,
  the same wait loops excluded) gives busy instructions per frame p50 566,640, p90
  685,758, p99 1,147,344 (the whole frame: at least 1% of frames never reach the wait
  loop). A 100 MHz core needs CPI ≤ 3.07 at p50, ≤ 2.54 at p90 and ≤ 1.52 at p99.
- **CPI model** (R22): an in-order scalar 603e at 100 MHz, run on the busy instruction
  stream with its real load and store addresses (Supermodel, 18,900 frames each).
  Register scoreboard with load-use, multiply (3), divide (20), FP single (3), double
  multiply iterated (9) latencies; static branch prediction; 4-way caches, 32-byte lines;
  uncached I/O stores posted (2 cycles), loads 20.

  | Configuration | VF3 CPI / heaviest frame / frames over | Daytona 2 CPI / heaviest / over |
  |---|---|---|
  | Perfect caches | 1.36 / 90% / 0% | 1.48 / 73% / 0% |
  | 16 KiB, 24-cycle fill, blocking | 1.95 / 139% / 2.58% | 2.25 / 121% / 0.17% |
  | 16 KiB, + store buffer | 1.68 / 106% / 0.03% | 2.11 / 99% / 0% |
  | **16 KiB, + store buffer, early restart (12 to first word)** | **1.52 / 96% / 0%** | **1.79 / 86% / 0%** |
  | 32 KiB, same | 1.50 / 94% / 0% | 1.72 / 83% / 0% |
  | 16 KiB, 40-cycle fill, 20 to first word, store buffer | 1.62 / 103% / 0.02% | 2.00 / 95% / 0% |

  Not modelled: SDRAM contention from the Real3D and tile clients (covered by the
  40-cycle rows), TLB-miss handlers (about 7 a frame), CR fields as separate registers.
- **Interrupt rate:** about 420 `rfi` per frame, from MAME's per-scanline IRQ, which
  its own comment calls a guess. Real rate `[OPEN]`.

**M0-A2 — Core spike.**
Build the integer pipeline and the FPU datapath for the census subset only, no MMU yet.
Report ALM, DSP, M10K and Fmax against the 100 MHz target. Run a slice of the MAME trace
through it and measure CPI. Delivered throughput is Fmax ÷ CPI; compare it with the
instructions per second from M0-A1.
Pass: Fmax at or above 100 MHz (or a stated lower clock with throughput still sufficient),
and projected total ALM and M10K, with MMU and caches budgeted, under what section 8
allows.
*Kill criterion:* the spike alone, extrapolated to the full core, leaves the project over
41,910 ALM or 553 M10K using M0-G's measured figures for everything else.

**M0-B — Real3D traffic profile.**
Instrument `model3_v.cpp` to log, per frame: polygons, new vertices, triangles after clip,
matrix stack depth, VROM bytes read, texture uploads and their sizes, texel fetches, and
the highest culling, display-list and polygon RAM addresses written. Also log pixel
overdraw and every read of the Real3D status packet. Run VF3 and Get Bass attract.
Output: the real numbers behind section 5.3, the overdraw factor, and the Real3D RAM
footprint behind 7.3. Pass: peak polygons per frame and VROM bytes per frame fit the
bandwidth SDRAM and DDR3 can supply.

*Results, VF3 only (2026-10-30, R13).* Same 360 s run, 7,447 rendered frames in play.
- **Render cadence:** MAME renders every second frame (6,495 of 7,446 gaps are 2). The
  hardware renders every frame: VF3 runs at 60 Hz on the cabinet, and Sega Retro quotes
  about 32,000 polygons a scene at 60 fps on one stage. MAME's 30 Hz is an emulation
  artefact (it flags VF3 "too slow in gameplay"; Supermodel ties VF3 slow motion to the
  53C810 DMA). Every rate below is at one image per frame, 57.52 Hz.
- **Coverage caveat:** the run's peak is 16,283 polygons, half the 32,000 Sega Retro
  quotes, so the scripted play did not reach VF3's heaviest stage. Budget geometry for
  32 K polygons: about 96 K new vertices at worst, four multipliers at 50 MHz.
- **Real3D RAM footprint:** in 7.2. Culling 1.87 MiB, polygon 1.005 MiB, display list
  108.5 KiB, against 4 + 1–4 + 1 MiB windows.
- **Geometry:** per rendered frame, polygons p50 about 13 K, max 16,283; new vertices max
  47,415; triangles after clip p99 15,607, max 16,200. MAME's 65 K cap is never close.
  At 57.52 Hz the vertex peak is 2.73 M vertices/s, about 101 M multiplies/s at 37 per
  vertex: **two multipliers at 50 MHz cover it**, against the 8–16 the study assumed.
- **Model fetch:** 59% of polygons come straight from VROM, the rest from polygon RAM.
  Peak 1.19 MB of polygon data per frame, 68 MB/s at 57.52 Hz, all from SDRAM.
- **Texture uploads:** at most 64 K texels (128 KiB) per frame.
- **Overdraw** (summed triangle area ÷ 496 × 384, every pixel that is Z-tested): p50
  3.51×, p90 5.60×, p99 8.27×, max 12.28×. At 57.52 Hz, p99 is 90.6 M pixels/s.
  **One pixel per clock at 50 MHz (4.56×) covers about the median frame only; two pixels
  per clock (9.1×) covers p99.**
- **Texture sheets:** 7.5 of the 8 MiB written (8×8 tiles: page 0 31,395, page 1 30,030
  of 32,768); 5.93 MiB ever read. No SDRAM to be saved there.
- **Texel fetches** (four per bilinear pixel, Z-passing pixels only, MAME's draw order):
  per frame p50 1.38 M, p99 3.08 M. Distinct 4×4-texel blocks per frame p50 39,212
  (1.2 MiB), p99 62,357 (1.9 MiB): the per-frame texture working set is bigger than all
  the M10K on the part.
- **Texture-cache fills**, 4-way, 32-byte lines (4×4 texels), MB/s at 57.52 Hz:

  | Cache | p50 | p90 | p99 | max |
  |---|---|---|---|---|
  | 8 KiB | 186 | 259 | 508 | 892 |
  | 16 KiB | 135 | 246 | 429 | 650 |
  | 32 KiB | 114 | 194 | 304 | 475 |
  | 64 KiB | 94 | 174 | 200 | 365 |
  | 128 KiB | 80 | 154 | 174 | 183 |

  Even the floor, every distinct block fetched once a frame, is 72 MB/s at p50 and
  115 MB/s at p99. (R16)
- **Z pre-pass model** (R17): opaque textured pixels fetch texels only for the final
  winner of each priority pass, band by band (8 rows), triangle order within a band;
  alpha-tested and translucent polygons fetch as they draw. Fetches per frame p50
  0.80 M, p90 1.11 M, p99 1.72 M (from 1.38, 2.60, 3.08 M). Cache fills, MB/s at
  57.52 Hz:

  | Cache | p50 | p90 | p99 | max |
  |---|---|---|---|---|
  | 32 KiB | 72 | 153 | 253 | 447 |
  | 64 KiB | 59 | 139 | 163 | 345 |
  | 128 KiB | 53 | 119 | 146 | 151 |
- **Other sets** (R18). VF3 Team Battle in MAME, and Bass Fishing Deluxe through
  Supermodel replay (attract only; MAME stalls on its I/O board). Rendered frames:

  | Set | Polys p50 / p99 | Overdraw p50 / p99 | Fills 64 KiB p50 / p99 | Z pre-pass 64 KiB p50 / p99 | Z pre-pass 128 KiB p50 / p99 |
  |---|---|---|---|---|---|
  | vf3, MAME, play | 13,612 / 15,903 | 3.62 / 5.20 | 94 / 200 | 59 / 163 | 53 / 146 |
  | vf3, Supermodel replay, attract | 13,691 / 16,642 | 3.58 / 11.43 | 90 / 331 | 57 / 304 | 51 / 145 |
  | vf3, MAME, attract | 14,952 / 16,712 | 2.90 / 11.10 | 99 / 331 | 63 / 309 | 57 / 124 |
  | vf3tb, MAME, play | 13,892 / 15,644 | 3.35 / 7.17 | 105 / 267 | 60 / 159 | 50 / 135 |
  | bassdx, Supermodel replay, attract | 8,995 / 24,048 | 1.66 / 4.40 | 17 / 121 | 8 / 62 | 4 / 50 |

  Fills in MB/s at 57.52 Hz. VF3 Team Battle's ROMs (106.5 MiB) and Real3D RAM
  footprint match VF3's, so the 7.3 budget covers it. Bass Fishing is light on pixels
  and textures but peaks at 24 K polygons, above anything VF3 reached in these runs.
- **Daytona USA 2** is Step 2.1 (`model3_21_5881`: PPC603R at 166 MHz, MPC106,
  315-5881, DSB2). MAME stops after 8 frames. Supermodel runs it once the test menu's
  link setting is Single (saved in its NVRAM); without that it waits on the link and
  never uploads a texture. Measured through Supermodel (attract, R20):

  | | VF3 (play) | Daytona 2 (attract) |
  |---|---|---|
  | Busy instructions per frame p50 / p99 / max | 566,640 / 1,147,344 / 1,147,344 | 589,353 / 759,773 / 859,813 |
  | CPI needed at 100 MHz for 57.5 fps, p99 | 1.52 | 2.29 |
  | Polygons per frame p99 | 15,903 | 21,545 |
  | New vertices per frame p99 | 46,298 | 67,664 |
  | Overdraw p50 / p99 | 3.62 / 5.20 | 3.32 / 7.76 |
  | Texture fills, Z pre-pass, 64 KiB, p50 / p99 MB/s | 59 / 163 | 51 / 109 |
  | Geometry cycles p99, 4 multipliers at 50 MHz, share of a frame | 49% | 72% |
  | Rasteriser cycles p99, 2 pixels a clock at 50 MHz, share of a frame | 57% | 85% |

  Daytona 2's game work is lighter than VF3's: its 166 MHz CPU sits in the vblank wait
  three quarters of the time. Its 3D is heavier but inside the 50 MHz budget at p99.
  ROMs are the obstacle: 200.3 MiB, of which 104 MiB CROM, 64 MiB VROM, 32 MiB sound.
  Without sound that is 168 MiB, which needs two SDRAMs.
- Matrix stack depth and status-packet reads are not logged yet.

**M0-C — Memory plan.**
Prove the D-4 layout. On SDRAM: build the arbiter budget from M0-A1 and M0-B traffic and
run a synthetic load of every client at 100 MHz; report sustained MB/s per client and
worst-case latency for a 32-byte CPU cache-line fill from CROM. On DDR3: start from the
Model 2 core's measured figures for 3D-layer writes plus scan-out, add Z traffic if open
question 1 puts it there, and texture-sheet reads if 7.3 option 2 is taken. Pass: CPU
fetch latency keeps the 100 MHz core within the CPI M0-A2 assumes; every SDRAM client
meets its deadline; DDR3 traffic fits the port.

**M0-D — Sound.**
Wire two SCSPs and the 68000 from the Model 2A work, replay a captured sound-CPU command
stream, and diff audio output against MAME. Pass: resources fit and audio matches.

**M0-E — Tile generator, standalone.**
Dump tile RAM, palette and registers from MAME at five frames. Render in RTL under
Verilator, pixel-diff against MAME's `draw_layer`. Pass: zero differing pixels except the
documented flash effect.

**M0-F — 53C810 SCRIPTS.**
Capture the SCRIPTS programs VF3 and Get Bass run (MAME can log DSP and the instruction
stream). Pass: the list of distinct opcodes is within the subset in 5.2. Resolve VF3's
single-step quirk.

**M0-G — Measured budget. Done 2026-10-30, results in 8.1.**
From the Model 2 fitter report, pull measured ALM, DSP and M10K for the i960KB, MB86234,
renderer, 68000, SCSP, tile block, DDR3 path and I/O. Replace every `[EST]` row in
sections 7.4 and 8.1 with the measured figure, leaving a row `[EST]` only where Model 3
differs (second SCSP, Real3D geometry, Z-buffer, SCRIPTS interpreter). Pass: the rebuilt
totals, plus the M0-A CPU figure, are under 41,910 ALM and 553 M10K with margin for
routing. Fail: report by how much, and which row the overrun sits in. Needs nothing but
the Model 2 reports already on disk, so it can run first.

### After the gates

| Milestone | Work | Exit |
|---|---|---|
| M1 | CPU, bridge, board RAM, 53C810 DMA, tile generator, sound stub, Real3D status handshake stub | Boots to the service menu on `vf3` |
| M2 | Real3D traversal and decode (fabric) emitting triangle lists | Triangle list matches MAME's, frame for frame, over a VF3 attract. Do not start the rasteriser before this diff is clean |
| M3 | Rasteriser, textures, Z, alpha | VF3 attract image within tolerance of Supermodel |
| M4 | Sound, MIDI path, I/O, Bass Fishing sets, `getbass` I/O CPU | All eight sets boot; VF3 and Get Bass playable with sound |

VF3 needs the 7.3 SDRAM gap closed before M1 can load it, so M0-B's RAM footprint is on
the critical path.

The order of M2 before M3 repeats the Model 1 lesson: every rasteriser bug chased before
the geometry diff is clean turns out to be a geometry bug.

---

## 10. Risks

| Risk | Where | Mitigation |
|---|---|---|
| Fabric does not close (ALM or M10K) | 7.4, 8 | Gates M0-A and M0-G decide before any integration |
| VF3 does not fit a 128 MiB SDRAM module | 7.3 | M0-B measures the Real3D RAM footprint; texture sheets to DDR3 as the fallback |
| SDRAM port overloaded (ROM, CPU, Real3D RAM, tiles, sound) | 8.3 | Deadline arbiter budget in M0-C before integration |
| DDR3 port contention (3D colour writes, Z, scan-out) | 7 | M0-C from the Model 2 figures; overdraw from M0-B |
| Status-packet frame time (`tot_clks`) reported wrongly | 5.3 | Count 33.33 MHz ticks per frame; check against games in M1 |
| Real3D behaviour is inferred, not observed | 5.3 | Compare images against Supermodel; accept bounded fidelity |
| Status register is faked in MAME | 5.3 | MAME flips `m_real3d_status` on every read. Real hardware has state. Games poll it. Model busy/ready from Supermodel's ping-pong notes |
| VF3 SCRIPTS single-step quirk | 5.2 | M0-F |
| SCSI interrupt routing disputed (IRQ vs NMI) | 5.2 | Trace in MAME; try both |
| MIDI path hacked in MAME | 5.5 | Implement the real SCSP MIDI-in; M0-D |
| Oracle quality | 3 | MAME marks all eight sets not working. Use Supermodel for images, MAME for bus traces |
| Bass I/O ROM missing for three of four sets | 3 | `getbass` has `iocpu`; others need HLE or a dump |
| **No PowerPC core exists; the 603e is a new build** | 5.1 | Largest unknown. M0-A1 specifies it, M0-A2 spikes it before anything else is integrated |
| Double-precision FPU needed | 5.1 | M0-A1 census; reuse single-precision units if the census allows |
| Code executes from CROM, so SDRAM latency hits every fetch | 5.1 | M0-A1 PC-region census sizes the instruction cache |
| CPU Fmax at 100 MHz with FPU | 8.3 | M0-A2; fall back to a lower clock only if throughput still covers the M0-A1 requirement |
| Rasteriser overdraw ceiling 4.56× at 50 MHz, 1 pixel/clock | 8.3 | Four-texel-per-clock cache; overdraw measured in M0-B |
| CPU throughput: a fabric core at its Fmax may deliver less than a real 66 MHz 603e | 5.1 | The 603e dispatches up to two instructions per clock. My Model 1 measurement (CPI 9.83 against an assumed 3) shows clock-ratio arithmetic misleads. M0-A measures CPI on a real trace |
| Fog, mipmaps, spotlights absent in both references | 5.3 | Scope out; note in the README |

---

## 11. Licence

Project licence is GPL-3.0-or-later. Supermodel is GPL-3-or-later, so code and notes may
flow in. MAME is BSD-3-Clause; keep the attribution header on anything transcribed
(R. Belmont and Ville Linde are the listed copyright holders of `model3.cpp`). The 53C810
notes cite LSI/NCR technical manuals; read them, do not copy text. Every lifted RTL source
is checked before use and recorded in `THIRD_PARTY.md` with its commit. srg320's Saturn
SCSP is reference only.

---

## 12. Step 1.5 as a follow-on — not in scope

Supermodel lists 15 Step 1.5 sets (Le Mans 24, The Lost World ×2, Scud Race ×6, Virtua
Striker 2 ×6). Adding them to a Step 1.0 core needs:
- the 13.11 position format and full culling-node layout (a mode switch in decode),
- the three-word texture port,
- the CPU clock (100 MHz in both emulators; 66 MHz per the PCB notes — `[OPEN]`),
- optionally the M3COMM board. MAME's notes say the Twin/DX sets need Link ID = Single
  and Cabinet = Deluxe in Game Assignments, and that Dirt Devils (Step 2.1) needs
  Communication Mode = No Link, so link-less operation is workable.

The Scud Race MPEG sound board (DSB, Z80 plus MPEG decoder) is a separate block.
This is a follow-on, not a Step 1.0 deliverable.

---

## 13. Sources

Everything this study measures or cites, linked so the work can be checked. Emulators
and reverse-engineered notes describe behaviour; the manuals are the authority for the
CPU and bridge.

### Emulators (the behaviour references)

- **MAME 0.289**, tag `mame0289` (`f34f0250`), BSD-3-Clause; R. Belmont, Ville Linde:
  [`model3.cpp`](https://github.com/mamedev/mame/blob/mame0289/src/mame/sega/model3.cpp)
  (machine configs, memory maps, ROM lists, the Guru-Readme PCB notes),
  [`model3.h`](https://github.com/mamedev/mame/blob/mame0289/src/mame/sega/model3.h),
  [`model3_v.cpp`](https://github.com/mamedev/mame/blob/mame0289/src/mame/sega/model3_v.cpp)
  (Real3D traversal, decode, renderer),
  [`model3_m.cpp`](https://github.com/mamedev/mame/blob/mame0289/src/mame/sega/model3_m.cpp),
  [PowerPC core](https://github.com/mamedev/mame/tree/mame0289/src/devices/cpu/powerpc).
- **Supermodel** at [`036af1c`](https://github.com/trzy/Supermodel/tree/036af1c8d6640c5a60709c1f99144635ed82a184),
  GPL-3-or-later; Bart Trzynadlowski, Ville Linde, Stefano Teso, Ian Curtis:
  [`Real3D.cpp`](https://github.com/trzy/Supermodel/blob/036af1c8d6640c5a60709c1f99144635ed82a184/Src/Model3/Real3D.cpp),
  [`Model3.cpp`](https://github.com/trzy/Supermodel/blob/036af1c8d6640c5a60709c1f99144635ed82a184/Src/Model3/Model3.cpp),
  [`53C810.cpp`](https://github.com/trzy/Supermodel/blob/036af1c8d6640c5a60709c1f99144635ed82a184/Src/Model3/53C810.cpp),
  [`Games.xml`](https://github.com/trzy/Supermodel/blob/036af1c8d6640c5a60709c1f99144635ed82a184/Config/Games.xml),
  [`Docs/README.txt`](https://github.com/trzy/Supermodel/blob/036af1c8d6640c5a60709c1f99144635ed82a184/Docs/README.txt).
- The census patches that instrument both, and how to run them:
  [`tools/mame-census/`](../tools/mame-census/README.md).

### CPU and bridge (public manuals, the authority)

- MPC603e RISC Microprocessor User's Manual: [Rev 1, 1997 (bitsavers)](https://ftp.mirrorservice.org/sites/www.bitsavers.org/components/motorola/PowerPC/1997_MPC603EUM_MPC603e_EC603e_Users_Manual_Rev_1.pdf),
  [1995 edition (bitsavers)](https://bitsavers.mirrorservice.org/components/motorola/PowerPC/1995_MPC603EUM_PowerPC_603e_Users_Manual.pdf),
  [Rev 3, 2002 (Hackaday mirror)](https://cdn.hackaday.io/files/1665357125033440/MPC603e-manual.pdf).
- [IBM PID6 603e hardware specification (bitsavers)](https://bitsavers.mirrorservice.org/components/ibm/powerpc/G522-0268-00_PID6-603eHSr1_199612.pdf).
- Not yet linked here: the 32-bit PowerPC Programming Environments Manual (NXP
  MPCFPE32B), the MPC105 and MPC106 bridge user's manuals, and the LSI53C810A Technical
  Manual v2.1 (cited by Supermodel).

### Real3D (no programming manual is public)

- [R3D/PRO-1000 specification sheet (textfiles.com)](http://textfiles.com/computers/DOCUMENTATION/r3dspec.txt):
  one geometry processor with one, two or four pixel processors; 750 K polygons/s per unit;
  **50 M pixels/s per pixel processor**; 16-bit texels (4/8-bit mono, 16-bit colour with
  4-bit translucency or 1-bit contour), maps 32×32 to 512×512; trilinear MIP-mapping with
  microtexture; **22-bit floating-point Z (6-bit exponent, 16-bit mantissa)**; S-buffer
  edge anti-aliasing; per-pixel fog; flat, Gouraud and specular shading with a sun vector.
  A [product overview](http://textfiles.com/computers/DOCUMENTATION/r3d_prod.txt) sits beside it.
- [R3D/100 Architecture Overview, Nov 1996 (vgamuseum.info)](https://vgamuseum.info/images/doc/unreleased/r3d-100_architecture_overview_nov96.pdf),
  the PC chip set from the same family: 33.33 MHz crystal with on-chip PLLs; a separate
  texture-memory controller over up to 8 MB of CDRAM "with a proprietary caching approach
  that minimizes cache misses"; bilinear one memory cycle per pixel at 16 bits a texel;
  worst-case RAS cycles up to four memory cycles per texel; frame/depth buffer in MDRAM.
- Patents it lists (GE Aerospace, the Real3D lineage):
  [4821212](https://patents.google.com/patent/US4821212),
  [4825391](https://patents.google.com/patent/US4825391),
  [4855937](https://patents.google.com/patent/US4855937),
  [4868771](https://patents.google.com/patent/US4868771),
  [4888711](https://patents.google.com/patent/US4888711),
  [4896265](https://patents.google.com/patent/US4896265),
  [4905164](https://patents.google.com/patent/US4905164),
  [4910665](https://patents.google.com/patent/US4910665),
  [5191642](https://patents.google.com/patent/US5191642). Not yet read.
- [Mitsubishi's 1996 press release on the R3D/PRO-1000's graphics memory (Sega Retro)](https://segaretro.org/Press_release:_1996-08-06:_Mitsubishi%27s_Graphics_Memory_Products_Power_REAL_3D%27s_R3D/PRO-1000_Graphics_Engine).
- [Real3D: from tank simulators to graphics cards (CPU Shack)](https://www.cpushack.com/?p=19606), for the company history.

### Games

- [Virtua Fighter 3 (Sega Retro)](https://segaretro.org/Virtua_Fighter_3): 60 fps, about
  32,000 polygons a scene on its heaviest stage.

### CPU cores reviewed

- [SloPPC603](https://github.com/kdedon/SloPPC603/tree/419d8a484366dad43f74079c054304e6d2e6b00c),
  Kevin Dedon, GPL-2.0-or-later, reviewed at `419d8a4`: a reference only (D-13, R21).
- [Microwatt](https://github.com/antonblanchard/microwatt): 64-bit Power ISA 3.0, read for
  decode only.

### My earlier cores

- [Sega Model 1 core](https://github.com/alphanu1/sega-model1-mister) and
  [Sega Model 2 core](https://github.com/alphanu1/sega-model2-mister): the measured area,
  clocks and DDR3 figures in sections 6–8, the area techniques in 8.4, and
  `docs/model2a-design-study.md` (R744 clocks, R952 room savings, M2-E N64 RDP
  measurement, R557 on textures and DDR3, R35 on dual SDRAM).
- Model 2 fit reports used for M0-G: s1633 (`main`, 39,272 / 41,910 ALM, 551 / 553 M10K)
  and s1629 (2C).

Local copies of the Real3D documents, for reading only and not committed:
`sm3-builds/refs/`.

## 14. Decisions and open questions

### Decided (2026-10-30)

| ID | Decision |
|---|---|
| D-1 | CPU, Real3D geometry and rasteriser all in fabric. No HPS offload. |
| D-2 | The 3D layer is drawn into DDR3, the same approach as the Model 2 core. |
| D-3 | Clock plan: CPU 100 MHz, SDRAM 100 MHz, Real3D 50 MHz (100/100/50, integer ratios to reduce clock-domain crossings). A design target; no CPU of mine has closed above 40 MHz. |
| D-4 | ROMs and all board state live in SDRAM. DDR3 is used only for 3D rendering, and only where it has to be. ROMs in DDR3 (the earlier DM-1 proposal) is rejected. |
| D-5 | Bring-up title is Virtua Fighter 3 (`vf3`). Get Bass upright is the fallback reference. |
| D-6 | DDR3 holds the 3D layer (drawn in on-chip bands, 16 bits a pixel, as Model 1), polygon RAM and the texture FIFO. Texture sheets and culling RAM stay in SDRAM (latency). Confirmed on the board by M0-C. |
| D-7 | The SDRAM may run at 120 MHz (the module goes to 140, but 120 is the working ceiling). Usable bandwidth scales from ~120–140 MB/s at 100 MHz to ~145–170 MB/s. 120 is 6:5 against the 100 MHz CPU, so the SDRAM controller sits in its own clock domain with FIFOs at its ports, as Model 2's `m2_sdram` already does (Model 2 R568). Taken only if the M0-C budget needs it. |
| D-8 | Sound may be left out of the first builds if SDRAM space, SDRAM bandwidth or fabric needs it. That frees 9.5 MiB of SDRAM (8 MiB samples, 1 MiB sound RAM, 0.5 MiB program), the sound-RAM traffic, and about 6,000 ALM (2 × SCSP, 68000, glue). |
| D-9 | The Real3D memories sit behind a compile-time memory-placement parameter, so the texture sheets (and anything else the budget says) move to a second SDRAM once I have the dual-SDRAM add-on. Single-SDRAM remains the default build. |
| D-10 | Daytona USA 2 (Step 2.1) is a target for this core, at a reduced frame rate (20–30 fps) if that is what it takes. It needs the dual-SDRAM build (D-9), sound left out (D-8), and the Step 2.x deltas: MPC106 bridge, Real3D DMA device in place of the 53C810, 13.11 vertex format, full culling-node layout, three-word texture port, 315-5881 security board. |
| D-11 | Two second SDRAM modules are on order (rated to 150 MHz). The dual-SDRAM build is the main target for VF3 and Daytona 2; single-SDRAM builds stay possible with heavy frames slipping (R17). |
| D-12 | Build now, before the second SDRAM arrives: a single-SDRAM proof of concept running VF3 with sound left out (D-8). VF3 then needs 117.25 MiB of SDRAM (126.75 less 9.5 MiB of sound), 10.75 MiB spare. Heavy frames may slip on texture bandwidth (R17) until the dual-SDRAM build. |
| D-13 | The 603e is written from scratch, designed around the 8.4 area techniques. SloPPC603 is a reference only: read to understand the 603e's corner cases and to cross-check, never copied. Confirmed against the CPI model before RTL starts. |
| D-14 | One core for every step, driven by the game ID, as the Model 2 core is. Run-time selection: vertex format (17.7 / 13.11), culling-node layout, Real3D PCI ID, MPC105 or MPC106 register map, the SDRAM layout from the MRA. Compile-time parameters, with the game ID choosing inside what is built: 53C810 SCRIPTS and the Real3D DMA engine, the 315-5881 decrypter, the second SDRAM, sound. The CPU runs at 100 MHz for every step: the timebase and decrementer count from the 66 MHz bus on all steps, so only performance changes with the CPU clock. |
| D-15 | CPU target from the CPI model (R22): scalar in-order pipeline at 100 MHz with forwarding and static branch prediction; 16 KiB 4-way instruction and data caches, 32-byte lines, data write-back; a store buffer; critical-word-first fills with early restart; single-precision FP pipelined, double multiply iterated through the same multiplier. 32 KiB caches buy 1–4% and are not worth the M10K. |

### Open

1. Where does the 3D Z buffer live: DDR3, or banded on chip? M0-B's p99 frame is 90.6 M
   Z-tested pixels/s. With Z in DDR3 that is a 4 B Z read per pixel plus a Z and colour
   write per passing pixel, up to 12 B, about 1.09 GB/s. `[CALC]` That is beyond what a
   shared DDR3 port should be asked for, so the measurement leans hard towards Z (and
   colour) held on chip per band, with finished bands written to DDR3.
2. With D-6, VF3 has 1.25 MiB of SDRAM margin (7.3). Enough, or does it need tight
   packing, a smaller texture-sheet footprint, or dual SDRAM? Needs the texture-sheet
   footprint and the texel fetch rate (M0-B, next), and VF3 Team Battle measured.
4. SDRAM bandwidth, not only capacity. VROM model fetch (40 MB/s at the measured peak),
   CPU cache misses on a 63 KiB per-frame working set, tile fetch (44 MB/s worst) and
   texture-cache fills all land on one 16-bit SDRAM with ~120–140 MB/s usable. Texture
   fills are now measured (R16): 94 MB/s typical and 200 MB/s at p99 with a 64 KiB
   cache, on their own more than one SDRAM carries. Options, not exclusive:
   - **Dual SDRAM, textures on the second module.** Gives texture fills a whole SDRAM
     and moves 8 MiB off the first (VF3 margin 1.25 → 9.25 MiB). Still short at p99
     unless combined with the next item or a faster SDRAM clock.
   - **Z pre-pass per band.** Z is on chip per band anyway (8.3). Rasterising each band
     twice, Z only and then texture only where Z matches, cuts texture-cache fills by
     about a third typically and a fifth at p99 (64 KiB: 94 → 59 MB/s p50, 200 → 163
     p99; R17). Costs a second rasterising pass per band.
   - **Where that leaves a single SDRAM at 120 MHz** (D-7, ~145–170 MB/s usable): with
     the pre-pass and a 64 KiB cache, textures take 59 MB/s on a typical frame, beside
     VROM model fetch (40 MB/s peak), tile fetch (up to 44) and CPU misses. The typical
     frame just fits; the p90 frame (139 MB/s for textures alone) does not, and slips.
     On a second SDRAM of its own (D-9), the texture path fits through p99 at 128 KiB
     (146 MB/s) and nearly at 64 KiB (163).
   - Bigger texture cache: 128 KiB costs about 105 M10K and still leaves p99 at
     174 MB/s.
3. Can the PowerPC core, FPU included, close at 100 MHz? The i960 closes at 40 MHz and the
   FP units at 80. That is what M0-A2 tests. If it only closes lower, the throughput
   margin in 5.1 shrinks, and below about 66 MHz × the real chip's IPC it disappears.

---

## 15. Findings log

Each entry records what was believed, what is now known, and how it was established.
Wrong turns stay in.

**R1 — Scud Race is not Step 1.0.** I had taken Scud Race for an early Model 3 title
alongside VF3. Both MAME and Supermodel put it at Step 1.5, with Le Mans 24, The Lost
World and Virtua Striker 2. Step 1.0 is VF3 (three revisions), VF3 Team Battle and the
four Bass Fishing sets. Source: `model3.cpp` machine configs and Supermodel `Games.xml`.

**R2 — The device is 41,910 ALM, not 41,509.** The first draft carried 41,509. The
Model 2 fit report (s1246) reads 41,415 / 41,910. Every budget line had been about 400 ALM
too pessimistic.

**R3 — Model 2 runs 100 / 80 / 40, not 75 / 35.** The first draft used TGP 75 MHz and i960
35 MHz, which were Model 2's clocks before R744. Model 2 `main` runs SDRAM 100, clk_sys
(TGP and renderer) 80 and i960 40 MHz (`rtl/pll/pll.v`, R744). Throughput figures in 5.3
were redone at 80.

**R4 — Clock plan settled on 100 / 100 / 50.** It went from CPU 75 / geometry 35, to
100 / 100 / 40, to 100 / 100 / 50. The last step was to keep every clock an integer ratio
of the others so crossings need no asynchronous FIFOs, as on Model 2.

**R5 — ROMs stay in SDRAM (D-4).** An earlier draft proposed ROMs in DDR3 next to the 3D
layer (DM-1), because VF3 overflows 128 MiB. Rejected: everything lives in SDRAM and DDR3
is for 3D rendering only. Recounting the ROM regions from `ROM_START` (the 6 MiB CROM
mirror is decoding, not storage) and the Real3D RAMs (polygon RAM's CPU window is 1 MiB
in `model3_10_mem`) puts VF3 3.7–6.7 MiB over, not 6.7. Bass Fishing fits.

**R6 — srg320's SCSP is reference only.** The first draft listed it as a source to lift.
It has no licence, and on Model 2A it was used only to find faults in my own SCSP. Model 3
reuses my SCSP from the `model2a` branch.

**R7 — Model 2 already filters bilinearly; the Z-buffer is the new part.** I had treated
the Model 2 renderer as a weak proxy for the Real3D rasteriser. Model 2 already does
perspective-correct bilinear texturing (`m2_texel_bl.sv`); what it lacks is a Z-buffer,
since it paints from a depth sort. The texture path reuses well, depth does not.

**R8 — M10K is binding, not only ALM.** The first ledger stopped at 176 KiB of known
items. Model 2 fits at 553/553 M10K. M0-A2 and M0-G now carry M10K as a pass/fail
alongside ALM, and 7.4 lists the CPU caches, Z buffers, triangle buffers and FIFOs that
were missing.

**R9 — Speed grade.** The first draft called the I7 part the slowest grade. Cyclone V
grades run 6 to 8, with 8 the slowest; I7 is the middle grade at industrial temperature.

**R10 — M0-G: the paper budget was 8K–23K ALM too pessimistic.** The 8.1 ledger carried
the Model 2A paper ranges and totalled 37,500–60,500 ALM without a CPU, which left no
room at all. Model 2's measured per-entity figures (s1633 main, s1629 2C) put the same
blocks at 29,048–37,441. The biggest corrections: rasteriser 15,000–25,000 → 8,021
measured on Model 2 (7,500–10,000 with the Model 3 deltas); 68000 3,000–4,000 → 1,859;
SCSP 3,000–5,000 each → 1,928. The framework went the other way, 5,000 → 6,025. The CPU
now has about 3,500–12,000 ALM, against the i960's measured 7,559.

**R11 — Bring-up moved from Get Bass to VF3 (D-5).** Get Bass was picked because it is
the smallest set and the one MAME runs. VF3 is the title that matters. The cost is that
the SDRAM gap (7.3) and VF3's 53C810 single-step quirk (5.2) are now on the bring-up
path instead of after it. VF3 ROMs verified against MAME 0.289.

**R12 — M0-A1 on VF3: MMU, double precision and RAM-resident code are all real.** I had
hoped for a small ISA subset with no MMU and single-precision FP. VF3 runs with
translation on, uses BATs plus software data-TLB reloads (`tlbld`, about 7 per frame),
and computes a double-precision reciprocal square root whose results single precision
cannot reproduce. On the other side, code never executes from CROM, so instruction fetch
is from main RAM only, and peak demand is 1.18 M instructions per frame (CPI ≤ 1.6 at
100 MHz for p99). Established with the census patch in `tools/mame-census/`, VF3 only.

**R13 — M0-B on VF3: geometry is cheap, pixels are not.** The study sized geometry at
8–16 multipliers and treated 4.56× overdraw as probably enough. Measured: 47 K new
vertices per frame at peak needs two multipliers at 50 MHz, while overdraw reaches 8.27×
at p99, which needs two pixels per clock. The area balance moves from geometry to the
rasteriser, and Z-buffer bandwidth rules out Z in DDR3. Real3D RAM use is 3 MiB of the
7–10 MiB windows, which brings VF3 to exactly 128 MiB in SDRAM. MAME renders VF3 at 30 Hz;
the cabinet runs at 60 Hz, so MAME's cadence is an artefact and the pixel figures stand
at 57.52 Hz. The run's polygon peak (16 K) is half the 32 K quoted for VF3's heaviest
stage, so the geometry figures are a floor.

**R14 — DDR3 has room for the 3D layer and nothing else.** I had listed the texture
sheets, polygon RAM and texture FIFO as DDR3 candidates to give VF3 9.25 MiB of SDRAM
margin. The core's DDR3 port gives about 100 MB/s. The 3D layer's own writes and
scan-out take 43.8 MB/s at 16 bits a pixel and 87.6 at 32; polygon RAM reads add 28 MB/s
at the measured peak and 56 at VF3's quoted 32 K polygons; texture fetch was already
rejected for DDR3 on latency (Model 2 R557). On latency alone, polygon RAM
would be fine (prefetchable, as on Model 1); it is bandwidth that rules it out. Texture
sheets and culling RAM fail on latency. On that reading none of the three went to DDR3,
which left VF3 at zero SDRAM margin. Superseded in part by R15.

**R15 — Model 1 already runs the Model 3 DDR3 pattern at 60 Hz.** R14 ruled polygon RAM
out of DDR3 on bandwidth. My Model 1 core draws its 3D at 60 Hz in on-chip bands,
flushes them to DDR3 at 16 bits a pixel, scans out from DDR3 and keeps its polygon
payload on the same port, with prefetch hiding latency (Model 1 `311710f`, `9c05b79`).
Model 3's 16-bit layer plus polygon RAM is 72–100 MB/s against Model 1's ~45–50, the same
pattern at 1.5–2× the load. Polygon RAM and the texture FIFO go to DDR3 (D-6), checked
on the board by M0-C. Texture sheets and culling RAM stay in SDRAM on latency. VF3 margin
in SDRAM: 1.25 MiB.

**R16 — Texture fetch does not fit one SDRAM.** Texture traffic had been listed as
unmeasured. Simulating a 4-way texel cache over VF3 gameplay (7,447 rendered frames,
32-byte lines of 4×4 texels): 64 KiB gives 94 MB/s at p50 and 200 MB/s at p99; 128 KiB
gives 80 and 174. The per-frame working set is 1.2–1.9 MiB of distinct blocks, so no
on-chip cache removes it; fetching each block once a frame is 72–115 MB/s. One 16-bit
SDRAM at 100 MHz has 120–140 MB/s usable for everything. Texture sheets in DDR3 were
already out on latency. The texture sheets are 7.5 of 8 MiB written, so they give back
no SDRAM either. This makes dual SDRAM (textures alone on the second module) and a
per-band Z pre-pass the two levers worth measuring next. Established with the census in
`tools/mame-census/` at `-numprocessors 1`.

**R17 — A Z pre-pass helps, but does not put textures on one SDRAM.** Modelled in the
census: opaque texels fetched only for each pass's final winner, band by band. Texel
fetches drop 42% at p50 and 44% at p99; with a 64 KiB cache, fills drop from 94 to
59 MB/s at p50 and from 200 to 163 MB/s at p99. That makes the typical frame fit a
shared 120 MHz SDRAM and leaves heavy frames slipping. With textures on a second SDRAM
(D-9) and a 128 KiB cache, p99 is 146 MB/s, inside one module's budget. Plan: Z pre-pass
in the rasteriser from the start; single-SDRAM builds accept slips on heavy frames until
the dual-SDRAM build exists.

**R18 — Supermodel replay agrees with MAME, and the other Step 1.0 sets are no heavier
than VF3 on textures.** MAME cannot run Bass Fishing or Daytona 2, and Supermodel's
OpenGL renderer cannot count texels, so Supermodel now saves the Real3D memories every
30th frame and MAME's renderer replays them with the census on. Checked on VF3 attract
mode: Supermodel's frames through MAME give overdraw p99 11.4× against MAME's own 11.1×,
and 64 KiB texture fills of 90 against 99 MB/s typical and 331 against 331 at p99.
VF3 Team Battle matches VF3 in ROM size, RAM footprint and texture load. Bass Fishing
Deluxe in attract is a third of VF3's texture traffic but reaches 24 K polygons a frame.
Daytona 2 (Step 2.1) is not measured.

**R19 — VF3's CPU load is twice what MAME showed.** M0-A1 put VF3's busy instructions at
p50 307,703 per frame. MAME renders VF3 at 30 Hz, so the game's per-frame work was spread
over two frames. Supermodel renders at 60 Hz, as the cabinet does; counting its
interpreter with idle-skip off and the same wait loops excluded gives p50 566,640 and
p99 1,147,344, the whole frame. The p99 requirement (CPI ≤ 1.52 at 100 MHz) is unchanged;
the typical frame needs CPI ≤ 3.07, not 6. The same counting found the identical wait
loops in both emulators, which supports the method.

**R20 — Daytona 2 fits the CPU and the 3D budget; only memory stands in the way.** I had
assumed a Step 2.1 game at 166 MHz was out of reach for a 100 MHz core. Measured through
Supermodel with the link set to Single: its busy instructions per frame are p99 759,773,
lighter than VF3 (CPI ≤ 2.29 at 100 MHz), and its 3D at p99 takes 72% of a 50 MHz frame
for geometry on four multipliers and 85% for the rasteriser at two pixels a clock, with
texture fills of 109 MB/s at p99. Attract mode only, so racing with traffic may be
heavier. Its 168 MiB of non-sound ROM needs the second SDRAM (D-9, D-11). The Step 2.x
hardware deltas are listed in D-10.

**R21 — An open 603e exists, but not one that meets VF3 on this part.** The study said no
PowerPC core exists. SloPPC603 (Kevin Dedon, GPL-2.0-or-later, `419d8a4`, 2026-10-20) is a
603e-compatible SystemVerilog core fitted on the same 5CSEBA6U23I7 with Quartus 17. By its
own records: integer core 10,415 ALM, 52 M10K, 2 DSP, 68.5 MHz; with its COMPACT FPU
28,789 ALM at 50 MHz, FULL FPU 40,664 ALM and failing timing; CPI about 4.4 on nbench
and Embench. It implements BATs, segments, the page TLB with 603e software refill, 16 KiB
instruction and data caches, and verifies against DingusPPC. VF3 needs 33–66 M
instructions/s (R19), against its ~15 M/s, and the CPU budget is 3.5K–12K ALM (8.1).
The serialized load/store unit, single outstanding fetch and full double-precision FPU
are design choices for exactness, not tuning gaps. Decided (D-13): the CPU is
written from scratch for 100 MHz and CPI ≈ 1.3–1.5, with the 8.4 area techniques from
the start; SloPPC603 is a reference only, read to find faults and corner cases, never
copied.

**R22 — A scalar in-order 603e at 100 MHz carries VF3 and Daytona 2, if misses are cheap.**
SloPPC603's CPI of 4.4 (R21) raised the question of whether a simple core can reach the
CPI ≤ 1.52 VF3's heaviest frames need (R19). Modelled on each game's real instruction
stream, with its load and store addresses, the pipeline alone gives CPI 1.36 (VF3) and
1.48 (Daytona 2). Blocking 16 KiB caches with a 24-cycle line fill lift that to 1.95 and
2.25 and push 2.6% of VF3's frames over budget; cache size barely matters (32 KiB: 1.91).
A store buffer and early restart bring it back to 1.52 and 1.79 with no frame over budget,
and keep it there with a 40-cycle fill. The CPU design follows (D-15).
