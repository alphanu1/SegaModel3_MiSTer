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
- **(Corrected by R80: the captured games never auto-start SCRIPTS, and no quirk is needed.)** **VF3 does something odd:** it writes DSP, which starts SCRIPTS automatically, *then*
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
| Palette, 32 K × 16 bit | 64 | `[CALC]`. Measured 52 M10K (R77): 48 for colour as 2 K × 5-bit banks and 4 for the suppress bit. The 32 M10K used in later projections cannot hold 524,288 bits. |
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
| D-15 | CPU target from the CPI model (R22): scalar in-order pipeline at 100 MHz with forwarding and static branch prediction; 16 KiB 4-way instruction and data caches, 32-byte lines, data write-back; a store buffer; critical-word-first fills with early restart; single-precision FP pipelined, double multiply iterated through the same multiplier. 32 KiB caches buy 1–4% and are not worth the M10K. The FP clause is under review: the FPU was built as one pipelined double FMA (R29). |
| D-16 | The SDRAM controller starts from the Model 2 core's `m2_sdram` at CAS latency 2, as the Model 1 and Model 2 cores run. |
| D-17 | For the CPU, the 603e and PowerPC manuals are the authority. MAME and Supermodel are cross-checks for finding misreadings; where one disagrees with the manual, the manual wins and the difference is recorded here. Emulator behaviour is followed only where the manual leaves a result undefined (divide by zero, 0x80000000 / -1). |
| D-18 | The FPU is built complete, everything the games execute, and the CPU clock is allowed to fall below 120 MHz to carry it. The core is at 106.7 MHz in the full build (R41); the frame-budget grid (R42) is re-run at the clock reached, not at 120. |
| D-19 | For the Real3D, where MAME and Supermodel differ, Supermodel is followed: node culling, LOD choice, colour table, transparency width, light vector (`sim/real3d/m2-notes.md` difference list, rows 5, 6, 12, 17, 22), fog, lighting, translucency and the frustum set-up. MAME stays the bit-level reference wherever the two agree, because its triangle lists and software renderer can be matched exactly. M3's exit test is coverage against Supermodel's images plus a bit-exact test against my model. |
| D-20 | The rasteriser's off-chip polygon store for banded Z (R46) goes in DDR3 for now, so it can be tested on the board before the second SDRAM arrives. Its memory client sits behind a compile-time parameter, as D-9 does for the Real3D memories, so moving it to the second SDRAM is a parameter change plus that controller's port, with no change to the rasteriser. M0-C measures whether DDR3 carries the 180–216 MB/s total. |
| D-21 | A fast single-precision path at latency 4–6 sits beside R29's double FMA, which stays at L = 12 for doubles. It must give exactly the same results as the FMA alone: a single op whose operands are all exactly representable in single goes to the fast path, and any other goes to the double FMA, both computing exactly and rounding once (R29, choice 1). This is D-15's original plan, supported by R42. |
| D-22 | frsqrte is a fast pipelined estimate, as on the 603e (1-1-1, 603e Table 6-5), within the manual's 1/32 bound (PEM 8-82). The 603e's exact table is not documented. The estimate's significand fits in a single, so its consumers stay on D-21's fast path. This replaces R29's exact Supermodel form, which needs 97 cycles where R39 allows ≤ 36. fres stays as R29 built it. |
| D-23 | FPSCR bit 20 follows Supermodel: mtfsf, mtfsfi and mtfsb write it and mffs reads it back. The PEM lists it only as "Reserved" (PEM Rev 2, Table 2-4) and documents nothing more, so D-17's rule applies. R29 already builds it this way. |
| D-24 | Following R52, a 256-entry branch target buffer and 603e-style dual issue (two instructions dispatched a cycle to different units, 603e manual §6) are estimated and measured before either goes into the core. The BTB is built and verified standalone first. Dual issue is costed in area and Fmax from standalone prototypes of its register-file ports, forwarding and pairing logic, in variants, and then decided. |
| D-25 | D-19 refined: Real3D documentation and real-hardware footage outrank Supermodel. Supermodel is followed where it corrects real behaviour that MAME gets wrong (lighting values, culling, colour table, transparency). Its additions for a PC renderer are left out unless the hardware shows them, for example per-fragment lighting (R51: Gouraud) or anything that costs area for no measurable image change. So row 18's per-polygon normal memory (+252 ALM, no measurable change, R55) comes out, and row 6's LOD (400–600 ALM, about 0.1% of pixels) is not built. Both stay as model switches, to come back if the hardware footage shows them. Area keeps the first call on every block (R54). |
| D-26 | The core is built with `MISTER_DUAL_SDRAM`. The second SDRAM takes the analog I/O board's header, so there is no analog video and no analog audio jack; HDMI carries both. That drops the framework's VGA path (OSD for VGA, the VGA scaler output, csync and subcarrier, the analog audio DACs), which Model 2 measured at about 600 ALM and 5 M10K in vga_osd and vga_scaler_out alone (Model 2A study, framework list). Measured on this core in R57. Model 2's other framework switches (`MISTER_DOWNSCALE_NN`, `_DISABLE_ADAPTIVE`, `_DISABLE_YC`, `_DISABLE_ALSA`) are taken too, but **not** Model 2's audio-filter patch: this core keeps the framework's audio filter. |
| D-27 | Hard per-block ALM budgets, totalling about 40,830 against a 41,000 working ceiling. Framework plus CPU 17,780 (R57's measured 17,266, plus the D-21/D-22 FPU at +910 and a BTB at about +200, less about 600 of test harness). Real3D geometry 8,550. Rasteriser 8,000. Sound 3,000 (68000 plus both SCSPs on one engine plus glue). Everything else 3,500 (tile generator, SDRAM and DDR3 controllers, PCI bridge, I/O and SCRIPTS, Real3D command and texture upload). Dual issue does not fit and is not built; the CPU stays single-issue and gains CPI through cheap levers. Every block works to its number with §8.4 sharing first, and a block over its number is paid back from another block, recorded in an R-entry. |
| D-28 | The rasteriser runs at 100 MHz, every cycle, at half R46's width: 2 Z tests and 1 shaded, bilinear-textured pixel a clock (4 texels a clock), the same throughput as 4 + 2 at 50 MHz (VF3 p99 about 66% of a frame). Halving the lanes, texel ports and DSPs is how it meets D-27's 8,000 ALM. The whole Real3D, CPU and SDRAM then share one 100 MHz clock, with no crossings inside the core. D-3's 50 MHz Real3D clock is withdrawn (R50 already moved the geometry). Every rasteriser block closes at 100 MHz. |
| D-29 | Sound keeps FM: it is used, so `FM` stays on in the shared SCSP engine. The sound engine runs at 80 MHz, as Model 2's does (R59), from the main PLL with an 800 MHz VCO (100 / 80 / 50 MHz), with one crossing to the SDRAM. Translucency is the plain blend (R62 mode 0, MAME's), with no extra M10K; modes 1 and 2 stay as parameters. |
| D-30 | The CPU gains R61's set, budgeted at about 750 ALM and 2 M10K. The set is: a 64-entry BTB looked up one fetch ahead (design C, a 0-cycle hit), with the I-cache pre-read port; dynamic branch direction; and 2-word fetch. Together they lose 3 frames against the branch-folding 603e at 100 MHz (R61). It is built after the timing pass that makes 100 MHz close on every seed (R63), so that timing margin and the new fetch paths are measured together. Dual issue stays out (D-27). Revised after R70 (2026-11-01): the set is built now, without waiting for every seed to close. The CPU spike runs 108–110 MHz, and the remaining full-build failures include test-harness paths the real core does not have. So timing is measured with the additions in, and full-build closure is judged in the real core. Built 2026-11-01 (R78). Direction follows the 603e's own rule: the look-ahead of §2.3.4.4, with the y bit otherwise. R61's direction from the BTB entry is not the manual's behaviour, so it is not built. |
| D-31 | A milestone M2.5 goes in before M1: one real VF3 frame drawn on the MiSTer by the Real3D RTL, from a Supermodel dump of the Real3D memories loaded through the framework, with no CPU. It is untextured first, and textured once the rasteriser has textures. It is a separate MiSTer project (`quartus/m3_viewer/`), so the main core's build is untouched. Its exit is the board's image matching the bench's, which matches the RTL-form model. The dump is ROM-derived and never enters the repository. |
| D-32 | The geometry trades M10K for ALM to get back under its 7,600 target (R65): its output buffer (`OUT_M10K`, R65: −228 ALM, +4 M10K) and its side memories (`SIDE_M10K`, −336 ALM, +16 M10K) go into M10K by default, about −564 ALM for +20 M10K. The core's M10K projection is about 300–320 of 553 with a 32 KiB texture cache, and 365–385 with a 64 KiB cache and this trade, so M10K has the room and ALM does not. `BC_M10K` (−140, +8) stays available. |
| D-33 | Texture memory goes on DDR3 behind a pipelined read master (R67): no 16-bit SDRAM form carries VF3, and the best is 220.6% of a frame. Before the cache size (32 or 64 KiB) and the number of fills in flight (8 or 16) are fixed, the viewer measures two things on the board: DDR3 read latency, and whether the f2sdram bridge takes several pending reads. Texture sampling uses the nearest mip level by default; trilinear stays a parameter, to come back if hardware footage shows the blend. The two equal each other to 0.01 dB, and trilinear costs VF3 99.7% of a frame at 32 KiB and latency 30, against 75.9% for nearest. This goes against §7.3 and D-6. There, texture sheets were ruled out of DDR3 on my Model 2 core (Model 2 R557, latency), and the core's DDR3 port was taken as about 100 MB/s usable. Those figures came from `m2_ddr3`'s one-request-at-a-time form. If the probe shows the pipelined master cannot do much better, neither memory carries VF3's textures. The fallbacks are a per-polygon texture prefetch (§7.3), a larger cache, or textures split across DDR3 and the second SDRAM. |
| D-34 | With textures in DDR3 (D-33), the 3D layer and the binned polygon store may have to come out of DDR3 to the second SDRAM. That is decided by measurement, not now. The viewer's DDR3 probe measures texture-pattern reads, latency and MB/s at 1–16 in flight, in three cases: idle, with layer scan-out only, and with full drawing traffic. It also notes the framework scaler's share. R67's model puts VF3 nearest at 75.9% alone and 80.3% with 180 MB/s of other traffic, and it leaves the scaler out. If sharing costs too much, the move is cheap, because both clients already sit behind D-20's switchable request contract (`r3d_rast_band`'s flush on `m2_ddr3`/`m2_sdram`). A 16-bit layer is about 22 MB/s written and 22 MB/s scanned out, both sequential, which suits SDRAM. Until the measurement and the modules exist, D-6's placement stands. |
| D-35 | Timing is analysed at every corner (`TIMEQUEST_MULTICORNER_ANALYSIS ON`) in `Model3.qsf`, the viewer and the probe, against the MiSTer template's OFF. With it off, the fitter optimised and timed the slow 100C model only, and the −40C misses went unseen (R73, R79). Compiles take longer. Block spikes report both slow corners from their own timing scripts. |
| D-36 | Two-word fetch (`FETCH2`) is turned off, and the BTB and the 603e look-ahead stay. The projection (R81) is about 41.4K ALM with it, over the 41,000 ceiling, and about 40.6K without. It costs about 880 ALM for 5 frames lost in 56,700 (R78). It stays a parameter. The default changes only after the core regression has passed with `FETCH2 = 0`, `BTB = LA = 1`, a combination not yet benched on its own. Built 2026-11-01 (R83): the saving measured about 520 ALM, not 880. |
| D-37 | The first proof of concept is VF3's attract mode, without I/O or sound. Sound stays out (D-8). I/O is a stub: no coin, no buttons, service and test off, and the JVS/I/O board reads as idle, so the game runs its attract loop. Staged: (A) CPU, MPC105 bridge and memory map, board RAM, ROMs in SDRAM from the MRA, interrupts, the 53C810, the tile generator and video out, with the Real3D as a status-handshake stub. Pass: the warning and SEGA logo screens, which are tile layers (R84). (B) The Real3D fed live by the CPU and the 53C810: polygon RAM, textures and culling through the real geometry and rasteriser, composited with the tiles. Pass: the attract fights drawn. |
| D-38 | The CPU's long-term target is 110 MHz, closing at every corner, rather than buying back frames with area. At 110 MHz the CPI model loses 0 frames at singles-at-4 and 3 at the built fast path (R83). That is what two-word fetch (about 520 ALM) would buy at 100 MHz, so it stays off (D-36). The release keeps 100 MHz until 110 closes. Every CPU timing fix now aims for margin toward 9.09 ns, not just closure at 10 ns. This revises R58's 110 MHz from an aim to the target, after D-30. |
| D-39 | A misaligned integer access that crosses a 4 KB page is split in hardware inside a BAT block. It takes the alignment exception only when page-translated (MSR[DR]=1, no BAT hit) or when the crossing leaves the BAT block. The 603e manual's §4.5.6.1.1 is ambiguous for BAT regions; MAME splits and never traps; Supermodel has no MMU. Model 3's boot ROM treats every alignment exception as fatal, so no game relies on the trap: if the chip traps, games never do this, and if it splits, games that do it keep working. Misaligned accesses within a page are split in hardware as the manual says (R85). |
| D-40 | Test builds may run up to about 41,500 ALM with loose slack; a release must close every corner and come in under budget, which may need optimisation. The attract proof of concept (D-37) has no sound (about 4,360 ALM, R81) and no real I/O, so it projects at about 22K (stage A, R85) + geometry 7.9K + rasteriser about 8K + stage B about 1K, about 39K: well under. |
| D-41 | The CPU reports PVR 0x00060103 (603e rev 1.3, "Stretch") on Step 1.0, as MAME does; Supermodel uses 0x00071202 (603r). VF3's boot code and miss handler test the PVR. With version 6 and revision below 0x0103 they set W in every BAT and in the RPA, working round an early 603e bug, so the core's former 0x00060101 ran RAM write-through on a path neither emulator takes. At 1.3, RAM is copy-back. The board's real revision is unknown. The value is a parameter of `ppc_cpu`, set per step in the system (D-14). The CPU test harness keeps its own default, so its signatures do not move. Revised the same day: VF3 never flushes the D-cache before the 53C810 reads RAM. Its SCRIPTS lists and DMA sources can sit dirty, and the real board stays coherent only because the MPC105 snoops PCI master cycles and the 603e casts out the dirty line. Our D-cache has no snoop port. So board builds stay on 0x00060101 (write-through, DMA correct) until a snoop port is built and proven: DMA reads cast out dirty lines first, DMA writes invalidate. Then the system moves to 0x00060103. |
| D-42 | Daytona 2's proof of concept accepts its geometry at about 118% of a frame (R76). Heavy frames then take two frames to draw, about 30 fps there, which is fine for a proof of concept. Its ROM beyond the 128 MiB SDRAM (about 200 MiB in all) goes to DDR3 while the second SDRAM is awaited: video ROM first, since it is read model by model and prefetchable (R82's numbers leave room). I want one core for every step, as D-14 says, not separate RBFs as on Model 2. Per-step hardware is selected at run time from the game ID wherever area allows. |

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

**R23 — M0-A2, first slice: the integer pipeline fits in 2,513 ALM and runs at 77 MHz.**
`rtl/cpu/`: a five-stage in-order integer core (no MMU, caches, FPU or interrupts yet)
with GPRs and SPRs in MLAB (8.4) and every two-result instruction cracked so the
register file needs one write port. Checked against an independent reference model
written from the manual (`sim/cpu/ppc_iss.h`): 20,000 random programs, 39.9 M
instructions compared on every retirement, with up to 9 memory wait states, no
difference. Three planted faults were caught; one (a deleted forwarding path) only after
the register file's simulation was made to return the old value on a same-edge write, as
the MLAB built without read-during-write logic may. Found on the way: an arithmetic
shift inside a conditional became logical (caught by the model), and Quartus 17 dropped
module-level signals read by a forwarding function, reporting them unread (caught by the
analysis pass before the fit). Standalone fit (`quartus/cpu_spike`, seed 1): 2,513 ALM,
1,128 registers, 0 M10K, 3 DSP, Fmax 77.2 MHz slow 100 °C (−2.946 ns at 100 MHz). Worst
path: CR-bit select and operand forwarding through the adder's zero detect into CR0.

**R24 — The CPU bring-up build: monitoring over the board's UART.** The core now
exposes eight performance events, and `rtl/dbg/m3_cpu_mon.sv` counts them with cycles
and a CRC-32 retire signature, printing a report on the debug UART (ttyS1, 115200) as
the Model 2 core's stream does. `m3_cputest` runs one generated program from on-chip
memory again and again; its memories answer in one cycle as the simulation's do, so the
board must reproduce the simulated report exactly (`make test_cputest` →
`build/cputest/expected.txt`; `tools/cputest-capture.sh` and `tools/cputest-compare.py`).
Seed 1: 3,355 cycles, 1,539 instructions, signature E1058DC4, identical across runs once
the generated programs write every SPR they read (the 603e leaves them undefined at
reset, and a repeated run inherited the previous run's values). Full MiSTer build on the
Template_MiSTer framework (`Model3.sv`), CPU at 75 MHz: 9,990 ALM, 123 M10K, 36 DSP,
setup and hold met on every clock (CPU clock +0.082 ns). Quartus 17 rejected `parameter
string` for `$readmemh` file names; untyped parameters work.

**R25 — The 603e core runs correctly on the board.** The CPU bring-up build
(`7e11353`, RBF md5 32835d1a…) on the DE10-Nano at 75 MHz, captured over ttyS1 for
20 s: 1,394 complete reports, every one identical to the simulation in all ten values,
3,355 cycles, 1,539 instructions and retire signature E1058DC4 each run. The core's
behaviour on silicon, its MLAB register file included, matches the model-checked
simulation exactly for this program. CPI 2.18 on this random program is not a
real-code figure: 39% of its cycles wait on divides.

**R26 — Timing pass: 77 to 104–114 MHz standalone, and the 100 MHz board run.** Two
passes (`a6ccb96`, `62e0f14`) moved the integer core from 77.2 MHz to 101.65 MHz at a
100 MHz target, then to 109.8 / 114.2 / 104.3 MHz over three seeds at a 120 MHz target
(8.333 ns), at about 2,580 ALM. What moved it: forwarding into the operand registers so
EX keeps one 2:1 mux; register and SPR files read in ID; static prediction at fetch with
the prediction carried to EX; every redirect registered, its target chosen a cycle
late; the fetch queue as a shift register in its own module with RAM recognition off
(Quartus turned every other form of it into a RAM); predecode for the queue pop; a
one-hot result select; a truth-table logic unit; a tree count-leading-zeros; CR
written as mask and values; trap commit in WB. Cost: the board program went from 3,355
to 3,383 cycles (+0.8%), instructions and signature unchanged. I had expected the full
MiSTer build to close where the standalone fit does; at a 100 MHz target it reports
108.5 MHz for the CPU clock (+0.786 ns), because the fitter stops working on a path
once it meets the target. The first 100 MHz board build, before the second pass,
missed by 0.423 ns and was not loaded. The second (`62e0f14`, RBF md5 d7392dc3…) ran on
the DE10-Nano at 100 MHz: 1,395 reports in 20 s, all identical to the simulation,
3,383 cycles, 1,539 instructions, signature E1058DC4.

**R27 — The integer core closes 120 MHz on every seed, for 400 ALM.** Seven sweeps at
8.333 ns took the three seeds from 109.8 / 114.2 / 104.3 MHz to **126.0 / 125.3 /
121.6 MHz** (slow 100 °C), at 2,978–2,988 ALM (+15.5%), 2,010 registers, 0 M10K, 3 DSP.
The board program's cycles, instructions and signature did not move (3,383 / 1,539 /
E1058DC4), so the gain cost no CPI. What moved it:
- the EX producer forwarded into the operand registers at the end of its own EX cycle,
  so EX starts from registers;
- CR bits for BI, crbA and crbB captured in ID, with a registered select for a writer
  one step ahead; CTR==1 carried as a flag beside CTR;
- slw, srw, sraw and rlw* on one left and one right rotator with masks after; the
  rlw* mask formed in ID;
- everything step-independent predecoded at fetch (operand selects, immediate form,
  result kind, special-register writes, SPR index), 36 bits a queue entry;
- cmp/cmpl on the main adder's carry (Quartus had built `x < y` as a 5-level tree);
- the multiplier's partial products summed a cycle after the DSPs, which take the
  operand registers directly (latency still 3);
- GPRs written as a uop leaves MEM, not WB, which removes the WB forward.
Where the 400 ALM went: fetch queue +123, the second decoder +80, core logic +226, ALU
−92. Wrong turns: two sweeps fixed the paths they aimed at and still lost Fmax, because
the result OR tree and the decoder depth in ID were next and placement moved; the gain
came only when both were fixed together. A first move-from capture in ID put an opcode
compare in front of a 32-bit mux (−1.3 ns). Checked with 16,000 seeds per change and
about 35 planted faults, all caught. One gap found in the generator, not the core: rfi
straight after mtsrr1 was never generated, so a broken SRR1 forward went unseen at HEAD
too; the generator now emits it and the planted fault is caught. The spike times no
memory-input paths (virtual pins); with the inputs registered at the top it gives
127.3 / 125.6 MHz.

**R28 — Predecode at fetch is the CPU's critical path in the full build.** The first
full MiSTer build of the R27 core with the CPU at 120 MHz missed by 2.852 ns (89.4 MHz):
every one of the worst 3,000 paths ran from the test ROM's M10K, through the new fetch
predecode, into the fetch queue. The spike could not show it because its memory inputs
are virtual pins. With the board test's ROM and RAM outputs registered once more (two
cycles a request) the same build missed by 0.365 ns (114.97 MHz), every failing path now
from a plain `i_data` register through predecode into `qx[0]`. So predecode depth sets
the full-design Fmax, and the instruction cache must feed it from a register, not from
M10K directly; with one fetch outstanding that costs fetch rate, so the cache fetch
path has to be pipelined. At 100 MHz the build closes with +1.121 ns (112.6 MHz; 10,497
ALM, 123 M10K, 36 DSP; RBF md5 b3698cc8…) and ran on the board: 1,395 reports in 20 s,
all identical to the simulation (new program after the rfi generator change, 4,726
cycles, 1,539 instructions, signature 54DD0CB7). The test system's two-cycle memories
put 38% of cycles into fetch wait; that CPI is the test system's, not the core's.

**R29 — The FPU, built standalone: 120 MHz needs a 12-stage multiply-add and 3,520 ALM.**
This contradicts D-15's "single precision pipelined, double multiply iterated through the
same multiplier" and §8.4's last paragraph. I built one fully pipelined double-precision
fused multiply-add (FMA) instead. It runs every add, subtract, multiply, multiply-add and
frsp, single and double. Beside it sit an iterative divide/sqrt unit, a small unit
(compares, fsel, moves, mffs), the lfs/stfs conversions and the FPSCR update. The reason
was correctness first: single ops given double operands must be computed exactly from
the full doubles and rounded once (decision below). Until the CPI sweep with these
numbers is in, this is provisional, and so is D-15's FP clause.
- **Latency.** The 4–7 cycles the CPI sweep assumed do not close. Standalone fits at
  8.333 ns (slow 100 °C, Model3.qsf's retiming settings), three seeds each:
  - L = 8: 71 MHz;
  - L = 10: 106–115 MHz;
  - L = 11: 112.7 / 117.1 / 114.6 MHz;
  - **L = 12: 116.4 / 117.1 / 121.1 MHz.**

  One LUT level costs about 1.4 ns with routing on this part, so a stage holds about five.
  The remaining L = 12 misses are the stopper-bit decode in the add stage and the fanout of
  the coarse alignment. A one-stage move not yet tried should take the first.
- **Area (fit, L = 12):**

  | Unit | ALM | Registers | DSP |
  |---|---|---|---|
  | FMA | 2,895 | 3,462 | 4 |
  | Divide/sqrt | 626 | 336 | 0 |
  | Small unit, 2 cycles | 107 | 193 | 0 |
  | lfs/stfs | 104 | 0 | 0 |
  | FPSCR | 75 | 0 | 0 |
  | **Total** | **≈3,520** | 3,798 | 4 |

  No M10K. The target was under 2,500 ALM. Options not yet tried, with rough savings:
  - one subtract order and a one's-complement fix-up: −200 ALM;
  - radix-2 division: −150 ALM, at +28 cycles per fdiv;
  - each stage removed: about −100 ALM, at the cost of Fmax;
  - D-15's single-fast, double-iterated design.

  Against §8.1's CPU room (3,500 to 12,000 ALM) and the 2,980 ALM integer core (R27), this
  puts the CPU core at about 6,500 ALM before the MMU and caches.
- **Divide unit cycles** (normal operands, idle pipe, start to result):
  - fdiv: 38 (the 603e takes 33 at 66 MHz);
  - fdivs: 24 (603e 18);
  - fres: 24 (603e 18);
  - frsqrte: **97, blocking** (603e 3, pipelined);
  - specials: 8.

  frsqrte goes through a full square root and then a reciprocal. At VF3's 885 a frame
  that is about 86k cycles, ≈4.9% of a 100 MHz frame, so a table estimate would pay its
  way if the bits can be matched.
- **Verification.** An MPFR reference model was checked against host IEEE arithmetic in
  every rounding mode: 11.2M checks, 0 failures. The RTL was then checked against the
  model:
  - about 66M checks, 0 failures, over L = 12, 11, 10, 8 and 3 and both small-unit
    settings;
  - each check covers write enable, all 64 result bits, the new FPSCR and the CR field;
  - every run includes 133,632 directed cases at zeros, denormals, extremes, ties,
    overflow and underflow boundaries, NaN position and fctiw near ±2³¹.

  Six planted faults were all caught. The FX-on-any-cause fault passed every directed
  case, because those start from a clean FPSCR; only random FPSCRs found it.
- **Where the manual leaves a result undefined, D-17 was followed (Supermodel).** Each of
  these needs a board capture to confirm:
  - **Single ops on double operands:** exact, rounded once. A NaN result has its low 29
    bits cleared. The 603e may use a narrow single multiplier here, so silicon could
    differ.
  - **fres:** 1/x rounded once to single.
  - **frsqrte:** the square root rounded to double, then 1/s rounded to double.
  - **fres and frsqrte flags:** FR and FI come from the last rounding; XX is unchanged.
  - **High words:** for fctiw and fctiwz, 0 when the result is in range, otherwise the sign
    extension of the clamped word. FPRF is unchanged. mffs reads a high word of 0.
  - **Reserved FPSCR bit 20:** writable and read back, as Supermodel's masks do. **This
    is open:** my reading of the manual was to hold it at 0.
  - **NI = 1:** a denormal result becomes zero of the same sign, judged after rounding.
    Inputs are not flushed.
  - **mtfsb1:** sets FX only on a 0→1 change.
  - **stfs below 2⁻¹⁴⁹:** host round-to-nearest.
- **Integration constraints:**
  - Divide results can complete after later FMA results. FR, FI and FPRF go to whichever
    op writes last, so the core must apply FPSCR updates in program order.
  - The divider's result enters the FMA's back end, which needs a one-cycle bubble in FMA
    issue.
  - Compare plus FPSCR update in one cycle measured 8.66 ns, so the small unit's flags are
    registered before the FPSCR.
- **Quartus 17 findings.** Verilator accepted all of the following, so they were checked
  only by `--analyze_file` and fits:
  - `wire <typedef> x = …` is rejected;
  - non-constant `'{field: value}` struct patterns are rejected;
  - side-band delay pipes were inferred as `altshift_taps` (3 M10K) until marked
    `AUTO_SHIFT_REGISTER_RECOGNITION OFF`;
  - retiming moved stage registers into critical paths until marked `preserve`.

  A 17-case constant fold of the signed-cast idioms matched Quartus exactly.
- **Reuse:** Model 2's fp_add split into align, add, normalise and round, with the
  alignment shift taken from the exponents alone (R739). The sticky bit comes from a
  trailing-zero count, not a second shifter (R512). fp_div's restoring division is
  reused. Divider results go through the FMA's rounder, which takes m2_fp_pool's shared
  reciprocal one step further.

**R30 — FP census and FPU latency sweep: round-to-nearest only, FPRF is read, and frsqrte
latency decides the worst frame.** Measured on VF3 (MAME, 360 s with play; Supermodel
attract as a cross-check), Daytona 2 and Bass Fishing (Supermodel attract, 18,900
frames each by the R22 method). New Bass Fishing wait loops are in the census README.
The R22 configurations were re-run inside the new runs and reproduce R22 exactly.
- **FPSCR is never left at its defaults.** The control byte is 0 (RN = 0, every enable
  off, NI off) at every FP op and after every write, in every title. MSR FE0 = FE1 = 0
  throughout, and no FP op ran with MSR[FP] = 0. mtfsfi, mtfsb0 and mtfsb1 never run.
  - Daytona 2's 583k `mtfsf FF` come from a double maths library that saves FPSCR with
    mffs, writes 0 and then restores it; it never tests a bit of the saved value.
  - So the exception-enabled paths in R29 are correct but unexercised. Bit 20 and the
    NI choice cannot change these games.
- **FPRF must be exact.** `mcrfs` reads only fields 3 and 4, in sequences such as
  `frsp; mcrfs cr7,cr4; mcrfs cr6,cr3; bt` or `fadds; mcrfs cr0,cr4; beq`. That is a
  zero, sign, NaN or class test through FPCC and the C bit:
  - Daytona 2: up to 5,128 a frame;
  - VF3: 8.4M in 360 s;
  - Bass Fishing: 18.0M in its run.

  Each `mcrfs` depends on the FP op just before it, so the core must interlock it on
  that op's FPSCR update.
- **Special values.**
  - SNaN: none in any title.
  - QNaN inputs: Daytona 2 has many (median 88, p99 101 a frame); VF3 has few (p99 6).
  - Infinity results: from x/0 and frsqrte(0).
  - Inputs below single normal range: tens of thousands (VF3 fmuls 20,741, fmadds
    6,616, fadds 2,355). Double denormals: none in VF3.
  - fctiw: always RN; VF3 has 746k exact .5 ties, so round-half-even matters, and
    5,374 out of range.
- **Single ops with double operands: 98–100% are an emulator artefact.** Their non-single
  operands come almost entirely from frsqrte and fres, which both emulators compute
  exactly in double. On the 603e both are estimates.
  - True double-to-single cases are about 5 a frame in VF3 and 24 in Daytona 2.
  - Rounding the double result to single, against R29's rounding once from the exact
    operands: at most 9 differences in about 2G ops (VF3 fmadds and fmsubs), and 0 in
    the other titles.
  - So R12's point (single-only arithmetic would change results) still holds for the
    double Newton step. Most of VF3's frsqrte results, though, feed single fmuls (91%).
- **The emulators' double fmadd family is unfused.** It differs from the manual's fused
  result on 2–13% of results:
  - Daytona 2 fnmsub: 9.1% of 26.1M;
  - Daytona 2 fmsub: 12.9%;
  - Bass Fishing fnmsub: 6.0%.

  Under D-17 the manual wins and R29's FMA is fused, so these results will differ from
  Supermodel. That is a known difference, not a fault. MAME's fnmsubs gives the wrong
  zero sign (it computes b − a·c) 1.66M times in VF3.
- **CPI sweep.** The baseline is the D-15 configuration (16 KiB 4-way caches, store
  buffer, early restart). A fully pipelined FMA costs about 0.07 CPI per cycle of
  latency, and serialising on FPSCR costs at most 0.02. **frsqrte is what costs.**

  | Configuration | VF3 at 100 MHz: CPI / heaviest frame | VF3 at 120 MHz | Daytona 2 at 100 MHz |
  |---|---|---|---|
  | No FP latency (base) | 1.51 / 96% | 1.56 / 82% | 1.78 / 85% |
  | FMA L = 4, radix-2 divide, frsqrte 115 cycles | 1.91 / 139% | 1.95 / 118% | 2.06 / 98% |
  | FMA L = 7, radix-4 divide, frsqrte iterative | 1.94 / 140% | 1.98 / 118% | 2.10 / 100% |
  | FMA L = 4, frsqrte and fres a pipelined table | 1.57 / 101% | 1.61 / 86% | 1.82 / 87% |
  | FMA L = 6, frsqrte and fres a pipelined table | 1.71 / 115% | 1.76 / 98% | 1.95 / 94% |
  | Fully blocking, L = 6, radix 2 | 2.50 / 193%, 19.6% of frames over | 2.54 / 163% | 2.46 / 118% |

  - VF3's heaviest frames run about 5,900 frsqrte, so each frsqrte cycle is worth about
    0.34% of a 100 MHz frame.
  - Bass Fishing's p99 frame is 30–43% of budget. Its 4–9 frames over budget execute
    Supermodel's whole frame without reaching a wait loop, so the FPU does not decide
    them.

- **The FPU as built (R29) costs more than the sweep's range suggested.** A second pass
  ran the measured unit: L = 12 FMA, and on one divider fdiv 38, fdivs 24, fres 24 and
  frsqrte 97 cycles. Each cell is CPI / median / p99 / heaviest frame / % of frames over
  budget:

  | | VF3 at 100 MHz | VF3 at 120 MHz | Daytona 2 at 120 MHz | Bass Fishing at 120 MHz |
  |---|---|---|---|---|
  | Built: L = 12, frsqrte 97 | 2.43 / 84 / 113 / 190 / 14.8% | 2.47 / 71 / 96 / 160 / 0.19% | 2.61 / 74 / 90 / 103 / 0.05% | 2.40 / 9 / 35 / 91 / 0 |
  | L = 8, frsqrte 97 | 2.13 / 72 / 97 / 160 / 0.28% | 2.18 / 62 / 82 / 136 / 0.06% | 2.34 / 66 / 81 / 93 / 0 | 2.20 / 8 / 32 / 88 / 0 |
  | L = 12, frsqrte 24 | 2.21 / 76 / 100 / 165 / 0.98% | 2.25 / 65 / 85 / 139 / 0.07% | 2.45 / 70 / 85 / 98 / 0 | 2.19 / 8 / 32 / 89 / 0 |

  - Each FMA cycle is worth about 0.08 CPI here, against 0.07 below L = 7.
  - A 24-cycle frsqrte is worth about as much as cutting L from 12 to 8.
  - At 120 MHz the built FPU leaves VF3 at p99 96%. About 36 frames go over, peaking at
    160%: those are the frames with up to 5,900 frsqrte.
  - Daytona 2 only just fits at 120 MHz.
  - The census does not see operand values, so the 8-cycle special-operand early-outs
    are not modelled; that makes these figures slightly pessimistic.
  - Still to measure: whether the real 603e at 66 MHz also runs over on those VF3 frames,
    and single and double ops at different latencies.
- **Open, for my decision:** frsqrte (and fres) as 603e-style pipelined estimates, or
  exact as Supermodel does.
  - The 603e's estimate tables are not public; the manual gives only the precision. A
    table estimate therefore matches neither silicon nor Supermodel bit for bit.
  - The exact form matches Supermodel, but blocks for about 100 cycles and puts VF3's
    heaviest frames 18–40% over budget.
- Two fixes in the census tools:
  - the CPI model's line loop wrapped at the top of the address space and hung, now
    fixed; earlier results are unaffected;
  - Daytona 2 is not fully deterministic between runs (0.0005% of busy instructions).

  Established with `M3FP` and `M3CPI_FPU` in `tools/mame-census/`, outputs in
  `sm3-builds/census/`.

**R31 — 603e MMU and interrupts: translated hits cost no cycles, for 1,660 ALM.** Built
from the 603e manual (Rev 3) and the PEM:
- the BATs and segment registers;
- 64-entry two-way ITLB and DTLB, reloaded by software;
- the three TLB-miss exceptions, with their SRR1 and miss-SPR contents;
- the TGPR shadow;
- DSI and ISI;
- the decrementer;
- the external interrupt as a level input.

The reference model was built first, then the RTL.
- **Data side.** The DBATs and a 4-entry micro-TLB of allowed translations translate in
  EX, so a hit costs 0 cycles. The micro-TLB tag is compared against x + y without the
  adder, by a carry-free A + B = K test. A micro-TLB miss replays from MEM: a two-cycle
  lookup, then a refetch from the uop that replayed, about 6 cycles.
- **Fetch side.** Real mode, the IBATs and a 2-entry micro-TLB translate beside the
  request. A miss is retried after a two-cycle ITLB lookup, about 4 cycles.
- **Other costs.** MMU writes with translation on take about 4 cycles to refetch, and
  isync about 3 (it was a no-op). Real mode costs nothing: the pre-MMU RTL gives
  identical cycle counts on the board program.
- **Fit.** The standalone spike runs 121.7 / 122.3 / 120.7 MHz over three seeds at
  8.333 ns. It uses 4,640–4,651 ALM (+1,660, +56%), 3,270 registers, 17,136 MLAB bits,
  0 M10K and 3 DSP.
  - It took eleven fit revisions. With the TLB compared in EX the core ran at 79 MHz.
  - From revision 5 the limit was the integer core's own forwarding paths, pushed out
    by placement in a bigger design. Each was fixed and checked on the bench:
    - the stmw register index taken from a counter;
    - a CR compare merge without the field decode;
    - registered step-zero and last-uop flags;
    - the multiply and divide result as a flat AND-OR;
    - mcrf of a forwarded CR0 resolved in MEM.
  - The registered-input variant runs 117–119 MHz, limited by those integer paths and
    not by predecode. So R28's predecode path is placement-sensitive, not structural.
- **Verification.** 34,000 seeds pass, 123.5M instructions at wait states 1–9, over four
  kinds of program:
  - integer;
  - translated, with random layouts and the manual's own TLB-miss handlers;
  - translated with interrupts;
  - real mode with interrupts.

  All 36 GPRs are compared at every retirement and memory at the end. The ordered stream
  of data accesses on both sides is compared too, which catches an access that should
  never have reached memory.
  - 46 planted faults caught. One more is caught only at 12,000 seeds.
  - One planted fault (a divide started during a replay hold) is missed in 2,000 seeds.
    The bench had found that same bug itself in an earlier revision.
  - Five real RTL bugs were found and fixed during the work.
- **Manual ambiguities and the choices made:**
  - **SRR1[15] = 1 means a store.** Tables 4-5 and 4-17 and the example handler say so;
    tables 5-4 and 5-10 say the opposite. MAME follows 5-4.
  - **DMISS, IMISS, DCMP and ICMP are read/write.** Section 2.1.2 says so; chapters 1
    and 5 say read-only.
  - **SRR1[WAY] uses replacement order** (the way not loaded most recently), not use
    order. Use order would depend on speculative fetches.
  - **A store hitting a C = 0 entry** reports the way that hit, so the manual's handler
    does not load a second copy of the page.
  - **A double TLB hit takes way 0.**
  - **Interrupts are taken before the instruction in EX.** This is legal under PEM
    6.1.2.4.2; the 603e lets that instruction complete first.
  - **The manual's example miss handlers need four corrections:**
    - SPRs 976–982, not 1010–1015;
    - srwi, not srw;
    - the ISI path's TGPR mask 0x0002, not 0x8000;
    - bdnzf tested on CR0[EQ].
- **The emulators.**
  - Supermodel has no MMU: tlbld halts it, and VF3 runs anyway. That suggests VF3's
    translations are identity wherever they matter. This is inferred, not checked on
    the cabinet.
  - MAME differs from the manual in eight ways: SRR1[15], WAY and KEY never set, a random
    tlbld way, miss SPRs read-only, no PP check on a TLB hit, no DAR on a BAT DSI,
    BRPN masked instead of ORed, and an N = 1 fetch reported as SRR1[4]. The ~7 tlbld
    a frame in MAME's census come from MAME's own TLB model.
- **Board test program changed.** With translation built, random mtmsr and rfi could
  turn on IR, DR or TGPR and run the program through random BATs. The generator now
  clears IR, DR, TGPR, POW, RI and LE in what it writes to MSR. The new expected values
  are 4,728 cycles, 1,539 instructions and signature 1284CEB8.
- **Still to do:** privilege checks, MSR[LE], misaligned accesses and the alignment
  exception, translation for the cache-block instructions, trace, IABR, SMI, machine
  check, and a measure of how many of VF3's data accesses fall outside its DBATs.

**R32 — The MMU core on the board: 1,395 of 1,395 reports identical; the data request is a
late signal.** The first full MiSTer build of the R31 core at 100 MHz missed by 0.322 ns
(96.9 MHz, 11,992 ALM). All 89 failing paths ran from the EX operand registers through
the address add and translation into the test RAM's M10K write enables. That is the
data-side counterpart of R28, and the standalone spike cannot see it for the same
reason: its memory ports are virtual pins.

With the test system's writes posted, the same RTL closes at 100 MHz with +0.990 ns
(111.0 MHz, 12,011 ALM, 123 M10K, 36 DSP; RBF md5 2222bab2…). Posted means each write
is registered and lands a cycle later, and a read of that word in the same cycle takes
the posted bytes. On the board it gave 1,395 reports in 20 s, all identical to the
simulation: 4,728 cycles, 1,539 instructions, signature 1284CEB8.

For the cache: d_req, d_we and d_addr leave the core straight from EX translation. The
data cache must therefore write its data and tag RAMs from a registered request, after
the tag check, never from EX. Its index can still come from EX, because a 4 KiB way
needs only untranslated address bits.

**R33 — FPU timing follow-up: L = 12 is at the edge of 120 MHz, now 3,332 ALM.** Six more
fits (fit18–fit23) went after the R29 misses. The best, fit19, is kept as the RTL:
**+0.141 / −0.406 / +0.058 ns** (seeds 1 / 2 / 3, 8.333 ns) at **3,332 ALM** (−149).
- **Changes kept:**
  - The S | stopper OR is gone: the cap now comes from s_den with a stopper bit at S4, so
    the count gives min(lz, s_den) directly.
  - The coarse alignment's guard, round and sticky are split out at S2.
  - The S6 normalise-to-lsb path is cut, the lfs/stfs unit is pipelined, and the small
    unit compares in halves.
- **What still fails** is S1 on seed 2: the denormal pre-normalise shift into the DSP
  input register, which Quartus retimed. The same path passes on seeds 1 and 3, so
  what is left is placement noise of about ±0.5 ns, not a long stage.
- **Wrong turns, each measured and reverted:**
  - **One subtract order with a one's-complement fix-up.** Estimated at −200 ALM,
    measured −22 ALM, and S5 got 1.06 ns worse. The estimate left out the 162-bit XOR,
    the increment chain and the count correction.
  - **The divider's classification moved into CHK:** −1.25 ns.
  - **Exponent-zero gating of the operand counts:** −1.18 ns. The A, B and C operand
    registers fan out to all three units at once.
- **Checks on the final RTL:**
  - 16.8M random checks over four seeds plus 133,632 directed cases per seed, all with
    0 failures;
  - all six planted faults caught;
  - the map counts match fit19 exactly.

  The divider cycle counts are unchanged from R29.
- **Untried, for the core:** give the FMA, the divider and the small unit their own
  copies of the operand registers at the core boundary, and pin the S1 register with
  `preserve`.

**R34 — Against a modelled 66 MHz 603e, only frsqrte's latency makes the core drop frames
the cabinet would not.** Run on the same 18,900-frame runs as R30.
- **The reference model.** A real 603e at 66 MHz:
  - FPU: singles and double adds 1-1-1, double multiplies 2-1-1;
  - divides: fdivs and fres 18 cycles, fdiv 33, all blocking;
  - frsqrte: 3 cycles, pipelined;
  - caches and store buffer as D-15;
  - memory either the same in nanoseconds or 1.5× slower;
  - issue: scalar, or dual-dispatch to different units, which is the closer of the two
    to the chip.

  It is a model, not a measurement: rename and completion limits are ignored, and the
  cabinet's memory timing is unknown.
- **The cabinet misses vblank too.** VF3 runs at CPI 1.30 on the dual model, with
  6.3% of frames over budget. 664 of its 1,187 over-budget frames never reach the wait
  loop even in Supermodel, which runs one instruction per cycle. They come in pairs, so
  the game itself runs below 60 Hz there.
- **The core's heaviest frames are the cabinet's too.** VF3's 35 heaviest frames at
  120 MHz with the built FPU (R29: L = 12, frsqrte 97) are each isolated, and sit at
  scene cuts: 15 open a scene, 14 close one and 6 are mid-scene peaks. The attract loop
  repeats every 14,840 frames, and seven of them recur exactly. 25 of the 35 are also
  over budget on the dual model, peaking at 123% against the core's 160%.
- **The measure that matters** for lag-free running: frames the core misses while the
  modelled cabinet makes them (dual model, normal memory / slow memory):

  | Core at 120 MHz | VF3 | Daytona 2 | Bass Fishing |
  |---|---|---|---|
  | Built: unified L = 12, frsqrte 97 | 10 / 2 | 9 / 9 | 0 |
  | Unified L = 12, frsqrte 24 | 0 / 0 | 0 / 0 | 0 |
  | Unified L = 8, frsqrte 97 | 0 / 0 | 0 / 0 | 0 |
  | Single and double at separate latencies (18 options: single 4–8, double 12 pipelined or 16 blocking, frsqrte 97 / 24 / pipelined estimate) | 0 in 17, 2 / 0 in one (single 8, double blocking, frsqrte 97) | 0 | 0 |

  Daytona 2's real CPU is a 166 MHz Step 2.1 part, so its true bar is stricter: no frame
  over budget at all. The core with frsqrte 24 has none: heaviest frame 98%.
- **Conclusion:** the unified L = 12 FMA (R29) is good enough once frsqrte comes down
  to about 24 cycles. Going back to D-15's single-fast design would save CPI (VF3 1.66
  at single latency 4 with a pipelined frsqrte, against 2.25), but drops no fewer
  frames. **frsqrte is the decision:**
  - **Supermodel's exact form** cannot reach 24 cycles: a square root and a reciprocal,
    each rounded to double.
  - **A shorter estimate meets the manual's precision.** The 603e's own estimate is a
    3-cycle table result that the manual specifies only to within 1/32.
- **The model is generous to scene changes.** The hottest code outside the wait loops
  is a byte-swapped store copy (`stwbrx` at 0x209C4 and 0x32720), probably an upload to
  the Real3D, which the model counts as 2-cycle posted stores. The real cost depends on
  the bridge and the Real3D's write acceptance.

**R35 — M2 groundwork: an FP32 geometry model matches MAME's triangles bit for bit; R13's
two multipliers do not cover it.** The golden triangle lists come from Supermodel dumps of
the Real3D memories, taken every 30th attract frame and replayed through an instrumented
MAME renderer (`tools/mame-census/mame0289-real3d-tris.patch`): 400 VF3 frames and 400
Bass Fishing frames. Each records every triangle as the rasteriser receives it, plus the
viewport, model and pre-clip polygon records.

A C++ model in `sim/real3d/` walks, decodes, transforms, lights, clips and projects in
FP32, in MAME's operation order and with no fused multiply-add.
- **It matches every field on every triangle:** about 21M triangle vertices and 27M
  pre-clip vertices, with or without subnormal flushing. MAME builds as plain
  single-precision steps, so an FP32 datapath in its order reproduces it exactly. That
  makes M2's exit test a bit-exact diff, not a tolerance.
- **One set-up stage is not exact by construction.** MAME's frustum set-up, once per
  viewport, takes tanf of atan2/2. Exactness needs a correctly rounded set-up unit,
  about 800 ALM against about 250 for y / (√(x² + y²) + x). The cheaper form keeps
  screen x and y within 0.053 px on every VF3 vertex, but no frame is then bit-exact.
  **Open:** bit-exact or 1/16 px. MAME's formula is MAME's construction, not documented
  silicon.
- **MAME reads an uninitialised vertex slot** in 308 VF3 polygons (0.006%), which
  changes the triangle count in 16 of 400 frames. The model keeps vertex slots across
  models, as Supermodel does, and the diff excludes those polygons. So "frame for frame
  against MAME" needs that exclusion in M2's exit test.
- **Measured load per frame, p99 (VF3 / Bass Fishing):**

  | | VF3 | Bass Fishing |
  |---|---|---|
  | Polygons | 16,612 | 23,916 |
  | New vertices | 46,278 | 46,404 |
  | Triangles | 15,494 | 16,589 |
  | Matrix stack depth (maximum) | 8 | 11 |

  Viewport chains reach 28 in VF3, and pointer lists nest at most 1 deep. No subnormals
  reach the geometry, so flush-to-zero FP units serve.
- **Correction to R13.** Its "two multipliers at 50 MHz" was already at 100% use, and
  the adders bind as hard as the multipliers. Share of a 50 MHz frame at p99, counting
  operations only:

  | Multipliers + adders | VF3 | Bass Fishing |
  |---|---|---|
  | 2 + 2 | 81% | 88% |
  | 3 + 3 | 59% | 63% |
  | 4 + 4 | 47% | 54%, limited by memory fetch |

  The plan is 4 + 4, because Model 2's pool runs at 65–70% occupancy and Daytona 2 is
  heavier.

  Per new vertex the cost is 12 + 9 + 5 multiplies. Each projected vertex adds 1
  reciprocal and 8 multiplies, and a node with a matrix costs 36. The reciprocal is
  correctly rounded: it matches `1.0f/x` on every mantissa.
- **Correction to §8.3 risk 2 and open question 4.** Polygons average 72 B in VF3 and
  60 B in Bass Fishing. At p99 VF3 reads 29 MB/s of polygon RAM and 43 MB/s of VROM,
  but Bass Fishing reads **80 MB/s of VROM**, twice the 40 MB/s carried for VF3.
  - MAME's walk culls no nodes: 59–68% of polygons reach the clipper, and most are
    rejected whole.
  - Supermodel-style node culling would roughly halve the fetch and the arithmetic, but
    it changes what is drawn relative to MAME. It is one of five rows in the
    MAME/Supermodel difference list (`sim/real3d/m2-notes.md`) that change the image:
    node culling, LOD, colour table, transparency width and light vector.
- **Area estimate for the geometry:** 6,200–7,650 ALM, 6–9 M10K and 9–10 DSP (4
  multipliers, 5–6 for the reciprocal). That is inside §8.1's 6,000–8,000 row.
- **First RTL: the display-list walker,** `rtl/real3d/r3d_walker.sv`.
  - It matches the model's event trace on all 800 frames (2.56M events) at memory
    latency 1, 8 and 20 cycles, with 0–50% consumer stalls.
  - It takes 7.1% (VF3) and 10.1% (Bass Fishing) of a 50 MHz frame at p99.
  - Standalone fit at 50 MHz, three seeds: 741–744 ALM, 0 M10K, 0 DSP, 98.7–106.7 MHz.
  - Quartus 17 left its two tables in registers (2,000–3,080 ALM) while a word was read
    in the cycle it was written. Allowing that put them in MLAB.
- **Wrong turn.** From the viewport words I believed MAME drew VF3 2.08× magnified,
  since MAME halves the plane angle while Supermodel's words 8–11 give tan 22.5°.
  Deterministic Supermodel frame grabs refuted it: they show MAME's scale. The factor in
  Supermodel that compensates is not yet found.

**R36 — The 603e caches: 120 MHz on every seed, 36 M10K, but a load hit takes 2 cycles.**
Standalone instruction and data caches to D-15, in `rtl/cpu/ppc_icache.sv` and
`rtl/cpu/ppc_dcache.sv`: 16 KiB, 4-way, 32-byte lines, tree PLRU, data write-back with
a store buffer, critical word first with early restart, and the 603e cache operations
and HID0 bits.
- **Fit:** 121.7 / 126.0 / 126.7 MHz over three seeds.
  - D-cache: 712–717 ALM, 18 M10K.
  - I-cache: 440–444 ALM, 18 M10K.
  - Each cache's M10K are 16 blocks of data (4 ways × 4 byte lanes) and 2 of tags.
  - There are no valid bits: an invalid line carries a reserved tag in Model 3's I/O
    page, which is always uncached. That keeps four 20-bit tags in two blocks.
- **R28 and R32 hold inside the caches too.** The tag M10K's output arrives about 2.3 ns
  into the cycle, so:
  - **a load hit takes 2 cycles**, request to data, and the core's MEM stage holds one
    cycle per load as it does on the test RAM;
  - misses and cache operations are decided a cycle after the compare, from registers.

  Four fits took the caches from 92.7 MHz to 121.7 MHz.
- **Measured cycles:**
  - I-cache, sequential hits with the core's `i_seq` hint: 1.00 a fetch. A redirect
    costs 2 cycles; a new line 2, or 4 when way prediction misses.
  - D-cache: stores are posted, 1 a cycle; a store then a load of the same word takes 3.
  - Misses get their first word at L + 3 (I) and L + 4 (D), where L is the controller's
    latency after acknowledge (12–15 cycles).
  - The D-cache does not block during a fill: only the set being filled waits. That
    follows R22's model, not the 603e (3.2.2).

  **R22 assumed a 1-cycle load hit, so the CPI effect is being measured before the core
  is wired to the caches.**
- **Verification:** 116 runs with no failures, 12.0M data operations and 12.4M fetches,
  checked against a reference of the exact cache state at memory latencies 1–80. 36 of
  38 planted faults were caught:
  - one missed fault is masked by design: a set stays blocked until its write-back
    issues;
  - the other affects performance only.
- **Choices where the manual is open:**
  - **Replacement is tree PLRU,** following table 2-2. Chapter 3 says "strictly LRU".
  - **dcbt fills the cache.** The 603e uses a one-block touch buffer instead (3.2.4).
  - **A write-through store to a modified line** keeps the line modified.
  - **An inhibited access to a resident line** bypasses the cache and leaves the line
    untouched (3.6.4.1, "boundedly undefined").
  - **The fill order is in 32-bit words.** Figure 3-3 orders double words.
- **The core changes needed for integration:**
  - WIMG per access: the data and fetch micro-TLBs keep none today.
  - Load lanes in `d_be`.
  - Cache operations issued as data requests.
  - HID0's cache bits as flip-flops.
  - sync waiting on `d_idle`.
  - The `i_seq` hint.

**R37 — The FPU inside the core: correct, but the spike falls to 105–115 MHz.** R29's FPU
is wired into `ppc_core`, and the reference model now executes every 603e FP instruction
through R29's MPFR-checked model. What was added:
- FPRs in MLAB, three copies of 32 × 64 bits;
- a pending-write scoreboard;
- FPSCR updates in program order;
- lfd and stfd as two word uops, high word first;
- the FP-unavailable exception (0x800);
- precise FP program exceptions;
- CR1 for Rc = 1.

**Checks:**
- 9,000 seeds, 30.1M instructions compared, 0 failures, at L = 12 over wait states 1–9
  and at L = 3, 8 and 11. That covers 1.22M FP ops, 583k FP loads, 338k FP stores and
  46,587 FP-unavailable exceptions.
- 35 of 35 planted faults caught, across the scoreboard, FPSCR order, load/store beats,
  FP unavailable, CR1, serialisation, the write port and exceptions.
- The bench found five bugs in the new RTL, all fixed.

**Findings:**
- **The divider can overtake *older* multiply-add ops,** not only later ones as R29's
  interface says. The hand-over takes the first free S4b slot, so a short special-operand
  divide jumped ahead of an older fctiw and FI came out wrong.
  - Fix: a divide starts only once every older FMA op is past S4b, and no FMA op issues
    between a divide's start and its hand-over. This is the 603e's own rule for fdiv,
    fdivs and fres (6.4.3).
  - The divider also starts a cycle after its uop leaves EX, costing one cycle per divide:
    EX's late leave decision was driving the operand enables.
- **FE ≠ 00 is precise mode,** which the 603e manual allows (4.1, 4.5.7.1, table 4-7).
  - mtfsf, mtfsfi and mtfsb1 setting FEX also trap. This follows PEM 6.4.7 and the class
    C ordering of table 6-4; PEM 3.3.6's "only computational instructions" conflicts.
  - fsqrt, fsqrts and unassigned 59/63 encodings are illegal whatever MSR[FP] is.
  - lfd/stfd crossing a page while word-aligned is not alignment-faulted (603e 4.5.6.1
    would); the alignment exception is not built yet.
- **CPI costs of the implementation:**

  | What | Cost |
  |---|---|
  | Compare | 2 EX cycles |
  | stfs | 2 EX cycles |
  | FPSCR instructions | +1 cycle |
  | Divide | +1 cycle |
  | lfs/lfd load-use | +1 cycle (no forwarding) |

  FPU results forward only while VE = ZE = 0, which is how every game runs (R30).
- **Fmax.** The standalone spike runs **104.6 / 111.9 / 106.7 MHz** over three seeds,
  at 9,504 ALM (+4,854), 8,286 registers, 7 DSP and 0 M10K. Without R33's suggested
  operand copies it runs 106.4 / 115.2 / 108.1 MHz with 357 fewer registers, so the
  copies are being removed.
  - The worst paths are the integer core's: x/y forwarding into the operand registers,
    m.result to m.cr, and the MMU replay. The FPU's area pushed them out by placement,
    as the MMU's did in R31. The only FP path near the top is the step-1 immediate of a
    double (`ex_fd0` → `op_y`).
- **Board program.** The new board program has 160 FP arithmetic ops and 118 FP loads
  and stores. Its expected values are 4,813 cycles, 1,611 instructions and signature
  093AF884.
- **A Quartus 17 trap:** two `import` lines in a module header are rejected; a single
  `import a::*, b::*;` works.

**R38 — The FPU core on the board: 1,394 of 1,394 reports identical.** This is the R37
core with the per-unit operand copies removed, built as a full MiSTer build at 100 MHz:
16,468 ALM, 123 M10K, 40 DSP, RBF md5 179562d0…. The board program has 160 FP
arithmetic ops and 118 FP loads and stores. The board gave 1,394 reports in 20 s, all
identical to the simulation: 4,813 cycles, 1,611 instructions, signature 093AF884.

So the FPU's arithmetic, FPSCR and FP load/store paths behave on silicon exactly as in
simulation, for the RTL's own choices. It does not show that those choices match a
603e.

The build is **not timing-clean**: it misses by 0.046 ns on one endpoint at the slow
100 °C corner. The path is R28's again, from the harness `i_data` register through
predecode into the fetch queue entries. The FP fields the predecoder now adds put it at
about 10 ns, so it has to come out of the fetch path before 120 MHz. A build that fails
timing does not count as verified; this run checks function only.

**R39 — frsqrte must be ≤ 36 cycles, and the 2-cycle load hit costs frames.** Both on
the same 18,900-frame runs as R34, at 120 MHz, with the unified L = 12 FMA.
- **frsqrte latency.** Frames the core drops while the modelled 603e (ref66_dual)
  keeps them:

  | frsqrte (cycles) | 24 | 36 | 48 | 60 | 72 | 84 | 97 |
  |---|---|---|---|---|---|---|---|
  | VF3 | 0 | 0 | 1 | 7 | 8 | 9 | 10 |
  | Daytona 2, frames over at all | 0 | 0 | 0 | 3 | 4 | 8 | 9 |

  Bass Fishing drops none at any latency. **36 cycles is the target and 48 the limit.**
  The frames added at 60 and above are VF3 scene cuts at 100–109% that the 603e makes
  at 95–100%.
- **The built caches' costs.** R36's 2-cycle load hit holds MEM a cycle per load
  (ld2). The I-cache charges 2 cycles per taken branch and 1 per new line (ic). With
  frsqrte at 24:

  | Option | VF3: CPI / heaviest / dropped where the 603e keeps (normal / slow memory) | Daytona 2: CPI / frames over / dropped where the 603e keeps | Bass Fishing: frames over / dropped |
  |---|---|---|---|
  | Model before (ld1) | 2.25 / 139% / 0 / 0 | 2.45 / 0 / 0 | 0 / 0 |
  | ld2 | 2.44 / 150% / 8 / 1 | 2.66 / 17 / 0 | 0 / 0 |
  | ld1 + ic | 2.40 / 146% / 9 / 1 | 2.60 / 12 / 0 | 2 / 0 |
  | ld2 + ic (as built) | 2.59 / 157% / 16 / 7 | 2.81 / 71 / 51 | 19 / 0 |

  With frsqrte at 60 as well, VF3 falls off a cliff: 434 frames over, 398 of them kept
  by the 603e. Steady attract frames move from 98–99% to 100–104%.
- **Conclusion.** With unified L = 12, the core as built has no throughput margin.
  - The load hit must not hold MEM: loads need to pipeline at one per cycle with
    2-cycle latency, which the core does not do today.
  - Even then (ld1 + ic) VF3 drops 9 frames the modelled 603e keeps.
  - The difference must come from FP latency (a single-precision fast path) or from
    fetch (redirect cost).
  - A combined table is being measured: {unified L = 12, singles 4 or 6 / doubles 12}
    × {frsqrte 36, pipelined estimate} × {ld1, ld2}, with the 603e reference also
    charged its own fetch costs.
- **Caveat.** The reference is a model, and so is the core's. A margin of a frame or two
  in either direction is inside their error; tens of frames are not.

**R40 — Timing pass on the FPU core: FP arithmetic out of predecode, spike still
104–111 MHz.** Four batches (T1–T4). T3 is kept: **104.6 / 111.3 / 111.4 MHz**, at
9,420–9,444 ALM, 7,945 registers, 7 DSP and 0 M10K. It leaves CPI and the board values
unchanged.
- **Out of the fetch predecoder**, for R38's full-build path:
  - FP arithmetic: opcodes 59 and 63 decode as illegal there, via a parameter. FP
    loads and stores decode like integer ones.
  - FP register-use bits come from a shallow decoder fed straight from `i_data`.
  - FP CR-writer flags are their own predecode bits.
  - FP-unavailable comes from the ID decoder.

  The predecoder is 140 ALM (156 with the FPU's fields, 129 before the FPU). Only the
  full build can time this path.
- **Other changes:**
  - the HASH1/HASH2 wait taken from one predecode bit;
  - CR1 updated in place from a registered select;
  - the FPSCR logic split into two instances, so EX's op is not in front of `fpscr_q`;
  - `fp_wait` formed from registered next-state bits.

  Setting the pending bit a cycle late (T4) was worse on every seed and was reverted.
- **Still limiting.** The integer x/y forward into the operand registers leads two of
  three seeds: from the queue head through the compare and a 49-way select, with
  1.0–1.5 ns routing hops. A select precomputed a cycle early needs the ID uop's
  destination a cycle early, which would grow either the decoder ahead of the compare
  or the fetch predecode. Also near the top:
  - the MMU's EX translation into `m.rp`;
  - the FMA's own S1 stage (−0.68 ns).
- **FP forwarding, measured on random programs:**
  - **lfd forwarding** is cheap for timing and saves 30 cycles in 4.57M. Real code
    pays a cycle per directly dependent use, so the random programs understate it.
  - **lfs forwarding** saves 38 but adds about 6 levels to the forward path.

  Neither is built. The 2-cycle compare costs at most one cycle per fcmpu or fcmpo
  (≤ 0.06%). It cannot drop to one cycle: compare plus FPSCR update measured 8.66 ns.
- **Checks:** the bench passes after every batch. 38 of 39 planted faults are caught;
  the miss is now an equivalent mutant, because the replay address is taken before the
  write it tested.
- **Status against D-15.** With MMU and FPU the core no longer reaches 120 MHz: about
  105–111 MHz standalone, lower in the full build. R39's frame margins were measured
  at 120 MHz, so at 110 MHz they are worse still. Unified L = 12 at about 110 MHz
  needs a review of D-15. The next measures:
  - a combined CPI table, being run;
  - the full-build Fmax of T3, being built.

**R41 — The fitter folded R28's fetch register back into the M10K.** R40's core
(T3), built in full at 100 MHz (16,501 ALM, 40 DSP), missed by 0.003 ns: 99.97 MHz.
The worst path began at `i_data`, the register R28 added between the test ROM and the
core. But the fitter had packed that register into the M10K's output register, so the
block's clock-to-out (the RAM's clock arrives about 2.3 ns late) was again in front of
predecode. The map report shows `i_q` packed into `rom_rtl_0`, with the ROM's output
unregistered. The core's own path from that register, through predecode and into the
fetch queue, is about 9.3 ns.

The harness now keeps `i_data` in the fabric: it is marked preserve, and is fed
through a mux that changes the word only while `i_valid` is low, where the core
ignores it. So R28's earlier 114.97 MHz was not a clean measure of the core either. A
register that Quartus can fold into a RAM block does not isolate the RAM's
clock-to-out. With the harness fixed, the same core closes 100 MHz with +0.628 ns (16,486 ALM,
123 M10K, 40 DSP, RBF md5 76c9986d…). On the board it gave 1,394 of 1,394 reports
identical to the simulation (4,813 cycles, 1,611 instructions, 093AF884), so this is
the first timing-clean board run of the core with MMU and FPU. Its full-design Fmax is 106.7 MHz.

**R42 — Decision grid: at 120 MHz all three titles need singles at 4, a frsqrte of 36
cycles or a pipelined estimate, and a load hit that does not hold MEM.** Same runs and
frames as R34, with the built I-cache costs on every option.

The 603e reference was corrected first. R34's dual model charged branches a dispatch
slot plus 1–2 cycles. The 603e manual (6.4.1.1, branch folding) says branches take no
dispatch slot and a taken branch's target arrives "in most cases" before any bubble;
the fetcher reads two words a cycle into a six-entry queue, with no line-crossing cost
on a hit. `ref66_dual_ic` models that and is the optimistic end; the old model is the
pessimistic end.

| 603e reference | VF3 CPI / heaviest / frames over | Daytona 2 | Bass Fishing |
|---|---|---|---|
| Old dual model | 1.30 / 123% / 1,187 | 1.47 / 108% / 20 | 1.38 / 146% / 26 |
| Branch folding (`ref66_dual_ic`) | 1.09 / 107% / 335 | 1.27 / 93% / 0 | 1.20 / 122% / 14 |

Core at 120 MHz, frames dropped that the branch-folding 603e keeps (Daytona 2: frames
over at all):

| Core option | Changes from as built | VF3 CPI / dropped | Daytona 2 CPI / over | Bass Fishing over (603e keeps) |
|---|---|---|---|---|
| As built: unified L = 12, frsqrte 97, load holds MEM | 0 | 2.82 / 1,352 | 2.97 / 334 | 18–19 (4–5) |
| Unified, frsqrte estimate, load pipelined | 2 | 2.37 / 30 | 2.58 / 8 | 0–2 (0) |
| Singles 4 / doubles 12, frsqrte 97, load pipelined | 2 | 2.10 / 6 | 2.35 / 0 | 0–2 (0) |
| Singles 4 / doubles 12, frsqrte estimate, load holds | 2 | 2.04 / 3 | 2.38 / 0 | 18–19 (4–5) |
| **Singles 4 / doubles 12, frsqrte 36, load pipelined** | 3 | **1.91 / 0** | **2.22 / 0** | 0–2 (0) |
| **Singles 4 / doubles 12, frsqrte estimate, load pipelined** | 3 | **1.84 / 0** (heaviest 99%) | **2.17 / 0** | 0–2 (0) |
| Singles 6 / doubles 12, frsqrte estimate, load pipelined | 3 | 1.96 / 0 | 2.27 / 0 | 0–2 (0) |

- **The fewest changes that pass all three titles is three:** a single-precision path
  at latency 4, frsqrte at 36 cycles or as a pipelined estimate, and loads that
  pipeline. Against the pessimistic reference it is one change fewer.
- **The core as built fails badly:** 1,356 VF3 frames and 334 Daytona 2 frames over
  budget.
- **This is D-15's original FP plan,** fast single-precision pipelined, which R29
  replaced with one unified double FMA. It is the plan the measurements now support.
- **Two limits on the table:**
  - it is measured at 120 MHz, and R40 puts the core at 105–111 MHz;
  - doubles at 12 pipelined still means a double unit, so the area question is two FMA
    datapaths or one FMA with a short single path. That is not measured.

**These are study-level decisions, left for review.**

**R43 — The geometry datapath is bit-exact but 2.5× over its area row and 1.8–2× too
slow at 50 MHz.** Built behind R35's walker, in `rtl/real3d/`:
- the four lanes, each one multiplier into one adder computing (a·b) + c, where a plain
  multiply adds −0 and so stays bit-exact;
- static microcode generated by `sim/real3d/gen_ucode.py`;
- the Model 2 divider, copied at `47edd9dc00` (THIRD_PARTY.md);
- the correctly rounded reciprocal;
- a 64-bit CORDIC unit for the correctly rounded frustum set-up, or a square-root unit
  for the cheap one.

**Checks:**
- **Against the C++ model:** every triangle field matches on all 400 VF3 frames (3.61M
  triangles) and all 400 Bass Fishing frames (1.83M), in all three builds (exact
  set-up, cheap set-up, culling). Memory latency was random at 12–20 cycles and the
  output not ready 20% of the time.
- **Units:** the reciprocal matches `1.0f/x` on 76.8M inputs, and the CORDIC its model
  and libm on 958,556.
- **Planted faults:** 13 of 13 caught.

**Measured against the budget:**

| | Measured | Budget |
|---|---|---|
| Area | about 19.9K ALM, 0–1 M10K, 9 DSP | §8.1: 6,000–8,000 ALM |
| Cycles at p99, VF3 | 1.54M (177% of a 50 MHz frame) | 100% |
| Cycles at p99, Bass Fishing | 1.76M (202%) | 100% |
| Cycles with culling, VF3 / Bass Fishing | 1.38M (159%) / 1.23M (141%) | 100% |

- **Where the area goes:**
  - controller: 9.1K ALM, about 11K registers in buffers;
  - lanes: 5.9K, including 393 ALM per multiplier that keeps subnormals, against 144
    for my Model 1 flush-to-zero unit;
  - CORDIC: 2.2K;
  - the standalone top's virtual pins: about 1.2K.
- **Fmax:** 63.6–73.3 MHz with the 50 MHz constraint, which closes. With the 100 MHz
  constraint it reaches 74.4–80.3 MHz and fails.
- **Speed.** The engine waits on its own kernels' latency 42–47% of the time. It is
  latency-bound, not operation-bound.
- **Culling** cuts VROM reads by 41% (VF3) and 48% (Bass Fishing), and loses 0.06%
  and 0.01% of MAME's triangles.
- **Memory per frame at p99:**
  - VF3: 31.7 MB/s of polygon RAM and 49.9 MB/s of VROM;
  - Bass Fishing: 93.6 MB/s of VROM.

  These include 10–15% of prefetch past each model's end.
- **R35's wrong turn explained.** MAME stores the projection's −1 and −2fn/(f−n)
  transposed, for its row-vector maths. That makes clip w about 2× depth, and halving
  the frustum half-angle cancels it.
- **No subnormal reaches the geometry in 800 frames,** so flush-to-zero units would
  give the same results on this data.

**The fixes are §8.4's techniques:**
- buffers into MLAB or M10K, read a word at a time;
- flush-to-zero FP units;
- one shared adder in the CORDIC;
- interleaving two chunks' transform and lighting to hide the 10-cycle chain latency;
- overlapping clip, assembly and projection with the kernels.

With the CPU at about 10K ALM (R40) and the rest of the design at 29K–37K (§8.1),
this geometry does not fit as built. Cutting it back to its row is now on the critical
path for fitting the core.

**R44 — Pipelined loads and the caches connected: the core runs a load a cycle, at a cost
of 3–6 MHz.**
- **Loads pipeline.** A load hit occupies MEM for one cycle, its data arrives two
  cycles after the request, and loads and stores can issue back to back. The data port
  allows up to three requests in flight: the uops in MEM and in WB, plus the one in
  the current cycle. `d_rdy` comes from a register, and answers come in order.
  - **Load-use distance is 3, not R42's 2.** A directly dependent instruction waits two
    bubbles, because EX starts only from registers.
  - GPR writes moved back from MEM exit to WB exit, undoing R27's change, behind a 4:1
    forward with a select formed from registers.
  - sc, rfi, mtmsr, isync, traps and EX-taken exceptions now wait for MEM and WB to
    empty. Otherwise a handler read a stale SRR0, SRR1 or MSR (PEM 6.1.2.4.2 allows the
    delay).
  - On the board program: 4,813 → 4,715 cycles (−2.0%). On the bench at data latency 2:
    −6.5%; at latency 1: +0.7%, because the consumer now waits for WB.
- **The D-cache accepts a request every cycle on hits:** 64 loads at 1.00 cycle each,
  and store-then-load pairs at 2.00. Its spike runs 124.2 / 128.1 / 130.9 MHz at
  776–782 ALM and 18 M10K.
- **R36's core changes are all in:**
  - WIMG per access from the BATs and micro-TLBs, with real mode as 0011;
  - load lanes in `d_be`;
  - dcbz, dcbst, dcbf, dcbi, dcbt and dcbtst as data requests;
  - icbi from MEM with EA[20–26];
  - HID0's cache bits as flip-flops;
  - sync waiting for every answer and `d_idle`;
  - `i_seq`;
  - the dcbz alignment exception on W or I pages;
  - dcbi privilege.

  `rtl/cpu/ppc_cpu.sv` puts the core and both caches behind the caches' memory port.
  The board harness gains `CACHED = 1`, with its ROM and RAM behind that port at a
  4-cycle latency.
- **Fit.** The CPU spike runs **101.6 / 105.0 / 105.4 MHz**, against R40's
  104.6 / 111.3 / 111.4, at 9,662–9,686 ALM and 7 DSP. The new leading paths are:
  - WB's wait and forward select into EX's operand capture (−1.0 to −1.5 ns);
  - EX translation's WIMG into the dcbz alignment term (−1.37 ns).

  R40's queue-head → `op_y` path remains.
- **Verification:**
  - **Core bench:** 3,000 seeds at wait states 1–9, 1,000 with 25% back-pressure and
    1,000 at data latency 2; 16.6M instructions in all.
  - **CPU subsystem bench:** 940 seeds at memory latency 1–80 (3.1M instructions). Every
    retirement is checked against the ISS; final memory is not compared, because dirty
    lines stay in the D-cache.
  - **Planted faults: 16 of 19 caught.** The misses:
    - icbi of the wrong set: there is no self-modifying code in the programs;
    - sync drain removed: equivalent in an in-order, single-master system;
    - replay kill not gated by WB's wait: probably equivalent.
- **Bugs found:**
  - **An early WB forward.** A consumer could enter EX before WB's load word had
    arrived. The old bench hid it by showing the next answer's word before `d_valid`;
    the bench now shows junk outside `d_valid`.
  - The stale SRR1 after sc.
  - A D-cache write-port conflict between a store hit and dcbz.
- **Choices where the manual is open:**
  - dcbt is a no-op in real mode, because the page is guarded (3.5, 3.7.2);
  - DSISR[22–26] for a dcbz alignment fault comes from instruction bits 6–10;
  - dcbi is translated and protected as a store; dcbst and dcbf as loads;
  - ICFI and DCFI always read 0.
- **New board values:**

  | Harness | Cycles | Instructions | Signature |
  |---|---|---|---|
  | No cache | 4,697 | 1,632 | EA97AAFC |
  | Cached, 4-cycle memory | 11,313 | 1,632 | EA97AAFC |

  The board program now enables both caches first and contains cache operations.

**R45 — The CPU with its caches on the board: identical to the simulation, at 90 MHz.**
Both of R44's forms missed 100 MHz in the full build:
- **no cache:** 95.5 MHz (−0.467 ns). The worst path ran into the debug monitor's
  signature register, from WB's load alignment, so it is the harness's trace port.
- **CACHED = 1:** 94.1 MHz (−0.629 ns). The worst path runs from EX's operands,
  through the address add and translation, into the D-cache's request registers. That
  one is real.

Rebuilt with the CPU clock at 90 MHz, both close timing. Each was run on the board,
20 s per form:

| Form | Slack at 90 MHz | Fmax | ALM | M10K | DSP | RBF md5 | Board | Cycles / instructions / signature |
|---|---|---|---|---|---|---|---|---|
| No cache | +0.855 ns | 97.5 MHz | 16,822 | 123 | 40 | e9861960… | 1,393 of 1,393 identical | 4,697 / 1,632 / EA97AAFC |
| Caches, 4-cycle memory | +0.565 ns | 94.8 MHz | 18,299 | 159 | 40 | d1f874fc… | 1,386 of 1,386 identical | 11,313 / 1,632 / EA97AAFC |

The cached program runs on silicon exactly as simulated, every cycle included:
- both caches enabled through HID0;
- cache operations;
- cold misses with critical word first and early restart;
- write-backs;
- uncached I/O pages.

In this short program the cycles are mostly cold misses (69% fetch wait). A timing pass
to bring the full build back above 100 MHz is under way.

**R46 — M3 groundwork: a rasteriser model bit-exact against MAME, and why two pixels a
clock is not enough.** The references are MAME's rasterised 3D layer and composite, plus
Supermodel grabs (finished frames, and 3D alone through a new `M3NO2D` mode), for the
400 VF3 and 400 Bass Fishing frames. They need fresh triangle lists from the same MAME
run, because MAME's undefined vertex slot (R35) gives different values from run to run.
For the composite, Supermodel's tile state is saved on the same frames.
- **MAME against Supermodel.**
  - The 3D layer reaches PSNR 13.5 dB (VF3) and 10.7 dB (Bass Fishing) at p50; colour
    where both draw, 8.3–8.6 dB.
  - Coverage disagrees on 4.47% of pixels. The causes are lighting, fog (Supermodel
    only), translucency, and MAME z-fighting on coplanar layers.
  - Tile layers sit 2 px apart in x and y.
  - Poses match frame for frame, which refutes the earlier note that they differ.
  - **So M3's "within tolerance of Supermodel" cannot be a PSNR.** It becomes a
    coverage test against Supermodel plus a bit-exact model test.
- **The C++ model, MAME form,** is bit-exact on all 800 frames in all eight texel
  formats. It excludes 347k pixels in 39 VF3 frames whose value depends on texel reads
  past a sheet's end (undefined in MAME), each tracked by a taint bit.
- **The RTL form cannot be bit-exact against MAME.** MAME steps each interpolant once
  per pixel in FP32, and that drift cannot be reproduced at several pixels a clock. The
  RTL form uses:
  - 16.16 edges;
  - FP32 set-up in MAME's order;
  - planes evaluated exactly in 40-bit block fixed point;
  - 24-bit Z and a 24-bit reciprocal.

  Against MAME it scores 47.1 dB (VF3) and 37.1 dB (Bass Fishing). Spans differ on
  0.004–0.007%, Z outcomes on 0.003% (VF3) and 0.15% (Bass Fishing). Its texture
  coordinates stay within 1/256 texel of the set-up plane on 99.8% of fetches, while
  MAME's own drift more than 4/256 off it on 36–54%. A fully fixed-point set-up was
  measured and rejected: shade differs on 27% against 3%.
- **Correction to §8.3 risk 3 and R13.** At VF3's attract p99 (2.18M Z-tested pixels,
  11.4× overdraw), share of a 50 MHz frame:

  | Scheme | VF3 p99 | Bass Fishing |
  |---|---|---|
  | 2 px/clk | 127% | — |
  | Z pre-pass | 96% | — |
  | 4 Z tests and 2 shaded px/clk | 66% | 32% |

  The plan is the last one.
- **Estimates:**
  - **M10K:** about 148 with 16-row bands (Z 20, colour double-buffered 40, a 64 KiB
    texture cache with tags 74, FIFOs 14); 118 with 8-row bands, 208 with 32-row.
  - **ALM:** 8,000–11,200, against the §8.1 row's 7,500–10,000.
  - **DSP:** 26–34.
- **Correction to open question 1, D-6 and §7.3.** Banded Z needs the polygon store off
  chip, which puts DDR3 at 180–216 MB/s in all against the 100 MB/s assumed. M0-C must
  measure DDR3 burst bandwidth; otherwise the polygon store moves to the second SDRAM.
- **First RTL: the rasteriser front,** a slope divider plus the edge walker.
  - **Exact:** on every span of the 800 frames (40.2M) and on 38.5M synthetic tie spans,
    with input gaps and output stalls.
  - **Planted faults:** 8 of 8 caught.
  - **Fit at 50 MHz:** 60.7–64.8 MHz, 2,512–2,515 ALM, 0 M10K, 0 DSP. The divider is
    two-thirds of it; sharing its normalisers should save about 1,000 ALUTs.
- **Undocumented:** MAME reads texels past the end of a sheet, and what the hardware
  does there is unknown.

**R47 — The Supermodel-form rasteriser model (D-19): fog is Bass Fishing's gap, lighting is
VF3's, and translucency may cost 140 M10K.** `sim/real3d/r3d_rast_sm.cpp` models New3D's
behaviour behind 11 switches (colour, decode, sample, alpha, translucency, depth, layer,
cull, fog, mip, priority). Each was measured against Supermodel's 3D-only grabs over the
800 frames:

| Form | VF3: PSNR all / p50 frame / coverage disagreeing | Bass Fishing |
|---|---|---|
| MAME | 8.58 / 13.50 dB / 4.47% | 10.91 / 10.67 dB / 4.47% |
| Supermodel, all switches | 8.72 / 14.16 dB / 5.13% | 17.29 / 21.82 dB / 3.13% |

- **Fog decides Bass Fishing.** Fog alone takes its p50 from 10.7 to 21.3 dB.
- **Translucency and culling** are worth about 1 dB each on Bass Fishing.
- **Sampling and mipmaps** are worth up to 0.4 dB at p50.
- **Each other switch** moves p50 by less than 0.4 dB.
- **VF3's remaining gap is lighting, which belongs to the geometry's vertex output.**
  Supermodel lights the sand floor white where MAME's light vector leaves it at ambient
  grey. So D-19's lighting rows (m2-notes rows 18–22) go to the geometry.
- **Not modelled:** Supermodel's white block-culling flashes, quad interpolation,
  micro-textures and specular.
- **Estimated cost of each feature in the RTL:**
  - fog: about 5 DSP and 160 ALM;
  - depth on 1/w: about −100 ALM;
  - culling: one flag bit from the geometry;
  - mipmaps: small logic, but trilinear doubles texel reads;
  - **translucency as Supermodel does it,** two layers with their own Z: band memory
    about 60 → 200 M10K at 16-row bands, or about 100 at 8 rows.
- **Open, for the board or the cabinet: does translucency dither?** Supermodel's own
  source names header[6] bit 17 "translucency pattern select". Its composite rule,
  where both layers present give opaque, is what two screen-door stipple patterns would
  produce. If the Pro-1000 dithers, translucency needs no extra M10K and matches
  Supermodel only on average. A close photo of VF3's translucent shadows would decide
  it.
- **Set-up sharing.** The rasteriser's triangle set-up is 25 dot operations per
  textured triangle. On its own unit it costs 1,300–1,600 ALM and 4 DSP. On the
  geometry's FP pool it costs about 150 ALM and adds 287K cycles at VF3 p99, 33% of a
  50 MHz frame. Sharing only works if the restructured geometry lands near 40–55% of a
  frame, so it is decided together with R43's area pass.
  - Flushing set-up results to zero changes no pixel: 6 subnormals in 546M operations.
  - 1/w can come from the geometry, which already computes it for projection.
- **Shared divider.** The per-triangle divisions can share one divider and normaliser.
  The per-pixel perspective reciprocal cannot, at two pixels a clock, without cutting
  slopes to 24 bits, which changes coverage.

**R48 — At the clocks the core reaches, a scalar in-order core falls about 10% short of
the 603e's throughput.** This re-runs R42's grid at 100, 105, 110 and 120 MHz, with the
built caches' miss latency scaled per clock (L = 10 / 11 / 11 / 12). It adds `ld1b`,
the load path as built (R44):
- load-use distance 3;
- 2 more cycles for a hit outside the predicted way (4.25% of VF3's load hits, 6.29%
  of Daytona 2's, 3.65% of Bass Fishing's);
- FP loads holding WB for a cycle.
- **Against the branch-folding 603e (`ref66_dual_ic`), nothing passes all three titles
  at 100, 105 or 110 MHz.** At 120 MHz three configurations pass with a 2-cycle load
  use (`ld1`), and none with the built `ld1b`.
- **The best configuration, at 100 MHz:** singles at 4, doubles at 12, a pipelined
  frsqrte estimate and `ld1`.
  - It delivers about 55 / 47 / 49 M instructions/s on VF3 / Daytona 2 / Bass Fishing,
    against the 603e model's 61 / 52 / 55 at 66 MHz.
  - VF3 still loses 602 frames the 603e keeps.
- **Against the pessimistic 603e model** (branches take a dispatch slot and a cycle),
  that same configuration passes all three titles at 100 MHz. The real chip lies
  between the two models.
- **The built third load-use cycle costs nearly as much as the MEM hold it replaced.**
  `ld1b` is 0.16–0.22 CPI above `ld1`, and within 0.03 of `ld2`. On VF3 at 110 MHz it
  turns 5 lost frames into 190.

  | Title | singles 4 + estimate: ld1 → ld1b (ld2) | unified 12 + frsqrte 97: ld1 → ld1b (ld2) |
  |---|---|---|
  | VF3, 100 MHz | 1.81 → 1.98 (2.01) | 2.60 → 2.76 (2.79) |
  | Daytona 2, 100 MHz | 2.11 → 2.31 (2.33) | 2.70 → 2.90 (2.91) |
  | Bass Fishing, 100 MHz | 2.06 → 2.24 (2.27) | 2.49 → 2.67 (2.70) |

- **The core as built** (unified 12, frsqrte 97, `ld2`) at 100 MHz: VF3 CPI 2.79, with
  8,331 frames over budget; Daytona 2 CPI 2.91, with 8,887.

**Conclusion.** D-21 and D-22 are necessary but not enough at 100 MHz. The core also
needs:
- a load-use distance of 2;
- either cheaper taken branches, since the 603e folds them and this core pays 2 cycles,
  or limited dual issue.

Which levers are worth what is being measured.

**R49 — A real-hardware video capture of VF3 as an image reference.** "Virtua Fighter 3
(Arcade, 1996) Real Arcade Hardware Capture – Full Playthrough",
https://www.youtube.com/watch?v=WFfIEQcY6aQ. It outranks both emulators for anything it
shows clearly (oracle order: hardware first).

From a frame of stage 1 (Jacky against Taka-Arashi, 28"93 on the clock):
- **The fighters' shadows look like a smooth darkening** of the sand: the floor's grain
  shows through at roughly 35–40% of the floor's brightness, with no checker or dot
  pattern visible. This leans towards blended translucency over R47's stipple reading.
  It does not settle it: the video is scaled about 3.7× from 496 × 384 and
  H.264-compressed, which can average a one-pixel stipple into a flat tone.
- **The sand floor is a lit mid tan.** R47 has Supermodel lighting a VF3 sand floor
  white and MAME leaving it ambient grey. Which is nearer the hardware needs this stage
  rendered by each emulator. Lighting gets checked against this video, not against
  either emulator alone.

A frame-accurate comparison needs frames taken from the video itself, matched to the same
stage and moment in a Supermodel run.

**R50 — The geometry rebuilt on one shared pool: 19.9K → 8.2K ALM, real time at 100 MHz,
still bit-exact.** This applies §8.4 to R43's datapath.

| Technique | Applied | ALM before → after | Source |
|---|---|---|---|
| Register files out of flops | into M10K (one write port) | 800 → 0, +8 M10K | Model 1 `7d95cdd` |
| Storage out of flops | every controller buffer into MLAB | controller 9,097 → about 2,800 logic + 1,404 MLAB; registers 14,952 → 8,712 | Model 1 `a5c905c`, `e643842`, `29a5fa0` |
| Duplicated arithmetic built once | one divide for projection, clip and set-up; one outcode set; one u/v converter | reciprocal 265 + fp_div 130 → 363 | Model 1 `86927fb` |
| Idle registers shared | assembly, clip and new-vertex registers; viewport slots | in the controller figure | Model 1 `86927fb` |
| Operators time-shared | the CORDIC on one adder and shifter | 2,212 → 966, and 0 by default (below) | Model 2 R952 |
| One pool, divide on the pool | every FP op | fp_div 130 → 0, fsqrt 117 → 0 | Model 1 `640eb4e`, `m2_fp_pool` |
| MLAB/M10K trade | microcode ROM to M10K; matrix stack in the register file's spare half | ROM 687 → 4 M10K | Model 1 `29a5fa0` |
| Flush-to-zero units | default; subnormal units kept as a parameter | fp_mul 393 → 88, fp_add 430 → 323 | Model 1 fp_mul |
| Per-build parameters | FRUSTUM_SM, SETUP_CR, CULL, SUBNORM | about −90 | Model 2 R948 |

- **Latency hidden.** Four decoupled machines (main, decoder, polygon machine and
  emitter) are coupled through memories. Main runs up to two chunks ahead, and clip and
  projection interleave with transform and lighting at kernel boundaries.
- **Fit at 10 ns:**
  - Fmax 110.1 / 110.6 / 112.3 MHz;
  - **8,170–8,193 ALM** (plus 622 holding the standalone top's pins), 13 M10K, 10 DSP;
  - that is 170–190 ALM over the 8,000 target.
- **Cycles at p99, default build** (Supermodel frustum, culling):
  - VF3: 1.05M cycles, **60.7% of a 100 MHz frame**;
  - Bass Fishing: 1.04M cycles, **59.7%**.

  The engine is dependency-bound: issue occupancy at p99 is 34–37% (20–22% of a frame),
  so a fifth lane would buy little.
- **Correction to D-3 and §8.3.** The geometry runs at **100 MHz full rate**. Its best
  structure needs 1.04–1.06M cycles at p99, which is 120% of a 50 MHz frame. §8.3
  already allowed 100 MHz with a clock enable; this goes further, to every cycle.
- **Correction: the clip divide cannot be reciprocal × multiply.** That differs from
  MAME on 27% of divides. The reciprocal is extended into a correctly rounded divide, at
  one extra DSP stage.
- **D-19 as built so far:**
  - **Culling.** Bass Fishing's p99 drops from 84.8% to 60.8% of a frame. VROM reads
    fall from 49.8 to 29.6 MB/s on VF3, and from 91.7 to 48.4 MB/s on Bass Fishing.
  - **Supermodel's frustum.** It removes the CORDIC (966 ALM) and is bit-exact to
    Supermodel's CPU float maths by construction.
  - **Not built yet:** LOD, colour-table base, transparency width, and lighting (rows 6,
    12, 17 and 18–22). Supermodel lights per fragment, with face normals for flat
    polygons, the fixed shade from the vertex byte, double-sided polygons drawn twice,
    and its own sun clamp. That touches the decoder, the transform-and-lighting kernel
    and the emitter.
- **Correction to R35: MAME's image scale is off by about 4%.** MAME's frame is 4.1%
  wider and 2.5% taller than Supermodel's on VF3's main viewport. On three Supermodel
  frames, Supermodel's frustum correlates better with Supermodel's image, and scaling
  MAME's triangles peaks near the predicted 0.96. This supports Supermodel's
  construction without proving it; R49's hardware video can decide.
- **Checks:**
  - every triangle field matches on all 800 frames in all five builds;
  - flush-to-zero confirmed by a monitor on every FP unit's inputs and outputs: 0
    subnormals in 800 frames;
  - 20 of 22 planted faults caught. The two misses are cases the frames never reach: a
    zero product's sign at an output, and reuse two chunks back.
- **Memory at p99, default build:**

  | Region | VF3 | Bass Fishing |
  |---|---|---|
  | Culling RAM | 4.6 MB/s | 3.6 MB/s |
  | Polygon RAM | 31.4 MB/s | 1.7 MB/s |
  | VROM | 29.6 MB/s | 48.4 MB/s |

  The VROM figures replace R43's.

**R51 — Culling and Supermodel's frustum measured separately. The Real3D's own documents
say Gouraud.**
- **Area and timing of each D-19 item** (seed 1, 10 ns, totals including 622 pin ALMs):

  | Build | ALM | Fmax |
  |---|---|---|
  | MAME frustum, no culling | 9,496 (CORDIC 950) | 100.9 MHz |
  | MAME frustum, culling | 9,670 | 92.6 MHz, fails |
  | Supermodel frustum, no culling | 8,694 | 112.7 MHz |
  | Supermodel frustum, culling (default) | 8,796 | 110.1 MHz |

  Culling costs +102 ALM on the default build, for R50's cycle and VROM savings.
  Supermodel's frustum saves 802–874 ALM. The MAME-and-culling build does not close
  10 ns, and is not the default.
- **Lighting is Gouraud.** Real3D's architecture overview of the R3D-100 (Nov 1996), a
  sibling of the Pro-1000, says the raster processor does Gouraud shading and that
  lighting makes faceted models look smooth (`sm3-builds/refs/`, lines 183, 249–255 and
  388–390). That is documented silicon behaviour: lighting per vertex, interpolated
  across the triangle. Supermodel lights per fragment, which is a GL convenience. So
  D-19's lighting rows are built as Supermodel's lighting formula evaluated per vertex,
  and the geometry hands the rasteriser a per-vertex shade, not normals. Per-fragment
  is measured in the model only, for comparison.

**R52 — The throughput levers, priced: at 100 MHz only a branch target buffer plus 603e-style
dual issue passes all three titles against the branch-folding 603e.** The base is
singles at 4, doubles at 12, a pipelined frsqrte estimate (D-21, D-22) and the core's
costs as built: load use 3, taken-branch redirect 2, sequential new line 1,
way-prediction miss 2, single issue. All 96 combinations of five levers were run at
100 MHz on 18,900 frames per title, with a subset at 110 MHz.

What each lever is worth on VF3 at 100 MHz (from CPI 1.98, and 848 frames lost against
`ref66_dual_ic`):

| Lever | CPI saved | Frames lost |
|---|---|---|
| Dual issue to two different 603e units (IU, LSU, FPU, SRU, BPU) | 0.17 | 32 |
| 256-entry BTB (taken-branch hit rate VF3 89.1%, Daytona 2 73.6%, Bass Fishing 85.1%) | 0.14 | 11 |
| 64-entry BTB (82.5% / 61.9% / 76.3%) | 0.13 | 14 |
| Redirect 1 cycle instead of 2 | 0.08 | 412 |
| No new-line cycle | 0.08 | 478 |
| Load use 2 | 0.07 | 641 |
| Dual issue of FP with non-FP only | 0.03 | 844 |
| Way miss 1 cycle instead of 2 | 0.01 | 848 |

- **The BTB and dual issue are the only levers that remove over-budget frames,** not
  just CPI.
- **The smallest passing sets:**
  - **Against `ref66_dual_ic` at 100 MHz:** two levers, a 256-entry BTB plus dual issue
    to different units. CPI is 1.64 / 2.01 / 1.92 on VF3 / Daytona 2 / Bass Fishing,
    with no frame lost. No single-issue set passes; even all four single-issue levers
    together lose 2 VF3 frames.
  - **Against the midpoint of the two 603e models:** dual issue plus a 1-cycle redirect
    or either BTB.
  - **At 110 MHz:** three single-issue levers pass against `ref66_dual_ic` (256 BTB, no
    new-line cycle, way miss 1).
- **Dual issue to different units is the 603e's own dispatch rule** (603e manual §6).
  Branch folding is how the 603e makes taken branches free (6.4.1.1); a BTB is this
  core's way to the same effect.
- **Area is not yet estimated for either lever.** A 256-entry direct-mapped BTB is
  small, about 2 M10K. Dual issue means a second integer issue path, two more
  register-file read ports and wider forwarding, which is a large change to `ppc_core`.
  **This is a study-level decision.**
- **Modelling note.** The load-use-2 lever here keeps the way-miss and FP-load WB costs,
  so it is 0.1 CPI smaller than R48's `ld1`.

**R53 — D-21 and D-22 built standalone: the fast single path closes at latency 6, not 4.**
Both are in `ppc_fpu_fast` (new top), not yet wired into the core.

**The fast single unit (`ppc_fpu_sfma`):**
- **Design.** R29's FMA construction at single width: a 75-bit window, both subtract
  orders, leading- and trailing-zero counts, and a tiny result rounded at a moved place.
- **Routing.** Opcode-59 add, sub, mul and the four multiply-adds go fast when every
  operand they read is a zero, or has its low 29 fraction bits clear and a biased
  exponent of 874–1150 or 7FF. Anything else goes to the double FMA.
- **Same results by construction.** On every seed, about 496k fast/double twins match
  bit for bit, with 0 route mismatches.
- **Ordering.** Fast results enter the double unit's output register, so there is one
  result port. A fast op issues only when no result is due at t + 6 or later. Results
  therefore leave in issue order, and FPSCR updates stay in program order.

**The frsqrte estimate (`ppc_fpu_rsqe`):**
- **Table.** 128 entries indexed by exponent parity and the top 6 fraction bits; a
  linear segment per entry (22-bit base, 13-bit slope) on the next 12 bits; 1 DSP; a
  logic ROM, so 0 M10K.
- **Precision.** Worst relative error over every input is 1.27e-5 (2^-16.3), against
  PEM's 1/32. The significand fits a single, so consumers stay fast.
- **Timing.** One op a cycle, latency 6.
- **Shared table.** `sim/fpu/gen_rsqe_tab.py` writes the same integers into the RTL
  and the reference model.
- **FR and FI unchanged by frsqrte.** Supermodel's source writes FR or FI for no FP
  op. That corrects R29's choice 3, which wrote them from a rounding Supermodel never
  does (D-17).
- **The divider loses its square root** (−190 ALM).

**Latency.** At 8.9 ns (112 MHz), latency 6 closes on all three seeds:

| Fit | Worst slack s1 / s2 / s3 (ns) | Note |
|---|---|---|
| L = 6 | +0.060 / +0.028 / −0.090 | seed 3's only failure is the standalone harness's own FPSCR instance; every FPU path is ≥ +0.39 |
| L = 5 | −1.044 | one seed |
| L = 4 | −3.826 | one seed |

Each adjacent pair of stages carries about 14 ns of logic. L = 5 would need a
leading-zero anticipator beside the add, and L = 4 a near/far split as well. **So R42's
"singles at 4" is not met; the build gives 6.**

**Area:**
- **Total:** `ppc_fpu_fast` is **4,240–4,243 ALM, 6 DSP, 0 M10K**: +910 ALM and +2 DSP
  over R33's unified FPU.
- **Parts:**

  | Unit | ALM |
  |---|---|
  | Double FMA | 2,665–2,673 |
  | Single unit | 852–862 |
  | Estimate pipe | 267–279 |
  | Divider | 436–441 |

**The double FMA improved along the way,** with results unchanged:
- operand normalisation counts leading zeros on the 53-bit significand, which fixes S1A
  (−0.26 → about +0.55 ns, −31 ALM) and should ease R40's FMA S1 path in the core;
- single-precision tiny tests use a clamped 9-bit subtract.

**A bug in the reference model, fixed.** The disabled-underflow path for single ops on
double operands narrowed MPFR's exponent range, which the operands also have to sit
inside. It returned garbage, for example fdivs of −2^191 by −2^341. The RTL was right.
The path now rounds to the 2^-149 or 2^-1074 grid by hand from a 600-bit value. Before
the fix it showed as 1 failure in about 4.7M checks.

**Checks:**
- fast bench: about 4.72M checks per seed over 7 seeds, 0 failures;
- unified FPU bench: counts unchanged;
- core bench: 300 seeds pass;
- planted faults: 25 of 26 caught. The one miss is an equivalent mutant: a clamp bound
  no single-range operand can reach.

**Core changes listed for integration:**
- instantiate `ppc_fpu_fast`;
- decode frsqrte as an FMA-port op;
- register the route decision;
- add fast-issue slots in the write-port vectors and a fast-issue wait;
- build the ISS with `-DPPC_FPU_RSQRTE_EST`.

**R54 — Whole-design area review: about 4,000–5,000 ALM over, so sharing and space saving
come first.** Each block's current figure, against its §8.1 row:

| Block | §8.1 row (opt / pess) | Now | Basis |
|---|---|---|---|
| CPU: core, MMU, pipelined loads | 3,500–12,000 for the whole CPU | about 6,310 (9,670 less the FPU) | R44 fit |
| CPU: FPU (D-21, D-22) | in the CPU row | 4,240 | R53 fit |
| CPU: I + D caches | in the CPU row | 1,440, 36 M10K | R44 fit |
| CPU: BTB, dual issue | in the CPU row | about 150–250 (BTB) + 1,000–2,000 (dual issue) | estimate, being measured |
| Real3D geometry and walker | 6,000 / 8,000 | 8,170 | R50 fit |
| Rasteriser | 7,500 / 10,000 | 8,000–11,200 | R46 estimate |
| Framework: `sys/` + `hps_io` | 6,025 / 6,025 | 6,025 | measured on Model 2, MiSTer options already off |
| **Sound: 68000 (fx68k)** | 1,859 / 1,859 | not built | measured on Model 2 |
| **Sound: 2 × SCSP** | 3,857 / 3,857 | not built | measured on Model 2 (1,928 each) |
| **Sound: board glue** (bank latch, i8251 link, mixing) | 357 / 600 | not built | Model 2 |
| Tile generator, 4 layers | 1,100 / 2,000 | not built | `[EST]` from Model 2 |
| SDRAM controller and arbiter | 750 / 1,500 | not built | `[EST]` from m2_sdram |
| DDR3 path outside the rasteriser | 200 / 300 | not built | Model 2 |
| CPU bus bridge (MPC105, PCI windows) | 600 / 1,000 | not built | `[EST]` from m2_cpu_bridge |
| I/O, i8251, NVRAM, RTC, 53C810 SCRIPTS | 500 / 1,500 | not built | `[EST]` |
| Real3D command, status, texture upload | 300 / 800 | not built | `[EST]` |

The optimistic sum is about 45,000–46,500 ALM against 41,910 (about 41,000 usable).

**Candidates for sharing, each to be measured before it is decided:**
1. **Doubles through the single unit.** D-15's original "double multiply iterated
   through the same multiplier" would drop most of R53's 2,670-ALM pipelined double FMA.
   Doubles run 1.5K–9K a frame (R30). R42's d16n rows put a non-pipelined double path at
   about +0.1 CPI, so its frame cost has to be weighed against about 2,000 ALM.
2. **The divider into an iterative double unit.** Today it is 436–441 ALM on its own.
3. **The rasteriser's triangle set-up on the geometry's FP pool.** The pool issues 20–22%
   of a frame at p99 (R50), and R47 priced its own unit at 1,300–1,600 ALM and 4 DSP.
4. **One divider and normaliser for every per-triangle division in the rasteriser**
   (about −1,000 ALUTs, R46).
5. **Dual issue only in its smallest form,** or not at all. At 110 MHz three cheap
   single-issue levers pass (R52), so the clock is weighed against about 1,000–2,000 ALM.
6. **Sound is about 6,100–6,300 ALM, the biggest block not yet started.**
   - **The two SCSPs time-multiplexed through one datapath** (§8.2 lever 2): about −1,900
     if the 512 clocks per sample allow it. That needs a measure of how many of those
     clocks Model 2's `m2_scsp` actually uses per sample.
   - **The SCSP's own datapath** (slot engine, DSP, envelope and LFO) read for
     time-sharing and storage out of flops, as R952 did for Model 2's converters.
   - **The 68000:** a smaller 68000 core instead of fx68k (1,859), if one is cycle-good
     enough for the sound program. Measure, don't assume.
   - D-8 lets early builds leave sound out. That is a schedule measure, not a saving.
7. **The rasteriser built to §8.4 from the start,** at or below its row.
8. **Narrower control state everywhere:** flops into MLAB, idle registers shared, one
   adder and one shifter shared across groups that are never live together (§8.4).

**The rule from here:** no block grows without an R-entry that names what it shares and
what it saved. The full-design ledger in §8.1 is re-measured after each block lands.

**R55 — D-19's geometry rows built: lighting gains up to 1.9 dB, Gouraud costs nothing
against per-fragment, and the rows cost 446 ALM.**

| Row | Change | ALM | p99 cycles | Against Supermodel |
|---|---|---|---|---|
| 22 (+19), lighting | normal × model matrix; sunFactor clamped to [0, 1]; I = min(sf·d + a, 1) | about 0 | 0 | +1.9 dB VF3, +1.2 dB Bass Fishing at p50. VF3 pixels more than 8 out: 45% → 28% |
| 18, normals | face normals for flat polygons; (2b + 1)/255 normals; a per-slot normal memory | **+252** | about 0 | no measurable change |
| 20, fixed shading | fixed shade as a T&L input | +14 | 0 | no measurable change |
| 21, double-sided | second shade carried; the inverted copy emitted | about 0 | +0.5 VF3 / +2.3 Bass Fishing | +0.2 dB |
| 17, transparency | transparency-0 polygons dropped (bit 23 is the opaque flag) | +8 | 0 | no measurable change |
| 12, colour table | walker carries the colour base; 12-bit index | +13 | 0 | VF3 all-pixel PSNR 9.55 → 9.71 dB |
| 6, LOD | model only: about 0.1% of pixels; an estimated 400–600 ALM and +9% of a VF3 frame | — | — | not built |

- **Light vector.** It is (n5, n6, −n4) in MAME's view space, which is GL eye space
  with y negated. Negating y as well would double-flip it. Supermodel's sun clamp is on
  in both games.
- **Gouraud against per-fragment** (R51): VF3 19.62 against 19.63 dB at p50, Bass
  Fishing 23.32 against 23.33. Per-vertex lighting costs nothing measurable, so the
  rasteriser takes one shade per vertex.
- **Output is now a word stream.** A record is 3 + 7n words (vertex x, y, z, w, u/w,
  v/w, shade), with the format in `sim/real3d/m2-notes.md` §13. It removed 672
  vertex flops and about 1,000 virtual pins.
- **Fit at 10 ns:**
  - 106.1 / 109.9 / 105.9 MHz;
  - 8,908–8,949 ALM including 232 pin ALMs, so **about 8,790–8,830 without them**,
    about 600 over R50 and the 8,000 target;
  - 12 M10K, 10 DSP.

  p99 cycles are 62.9% (VF3) and 63.3% (Bass Fishing) of a 100 MHz frame.
- **Checks:**
  - bit-exact against the model on all 800 frames in seven builds, plus the six one-row-off
    builds;
  - 33 of 36 planted faults caught;
  - two latent R50 bugs found and fixed: the polygon list rewritten under the emitter,
    and the translation read after the event.
- **The rasteriser's set-up on the geometry pool,** measured in RTL:

  | Case | VF3 p99 | Bass Fishing p99 |
  |---|---|---|
  | Set-up for every emitted triangle | 100.7% | 99.0% |
  | One set-up per double-sided polygon | 94.6% | 87.0% |

  That is over 85%. A facing test in the geometry plus a third lane-register read would
  bring it to about 80% / 75% (estimated). Shared, it costs about 150–300 ALM and 3 M10K,
  against 1,300–1,600 ALM and 4 DSP for its own unit.
- **Area candidates (R54):**
  - **Row 18's per-slot normal memory:** +252 ALM for no measurable image change.
  - **Row 6, LOD:** 400–600 ALM for about 0.1% of pixels.

**R56 — The CPU with caches closes 100 MHz in the full build, and runs on the board.** A
timing pass on R44's core, in seven batches:
- **The core:**
  - register compares a cycle early;
  - multiplier inputs gated, so Quartus cannot pack operand copies into the DSP;
  - MEM fields taken from registers;
  - write bits folded into early selects;
  - the MEM kill target through the second redirect;
  - a predecoded op for EX;
  - the CR kinds split into bit, field and mtcrf;
  - dcbz's W/I alignment check moved to MEM.
- **The D-cache:** request fields load whenever `d_rdy` is high, and bypass and
  reservation are formed in the lookup cycle.
- **The harness:** the monitor registers all its inputs.

**Cost.** Moving the dcbz check costs about 1.1 cycles per alignment fault: +295 cycles
in 20.87M on the bench. Nothing else moved; every UART value is unchanged.

| Full build at 100 MHz | Fmax | Slack | ALM / M10K / DSP | RBF md5 | Board, 20 s |
|---|---|---|---|---|---|
| No cache | 102.47 MHz | +0.241 ns | 16,835 / 123 / 40 | 28ca0e6c… | 1,394 of 1,394 identical (4,697 / 1,632 / EA97AAFC) |
| CACHED = 1 | 101.72 MHz | +0.169 ns | 18,319 / 159 / 40 | b6c1ae7a… | 1,387 of 1,387 identical (11,313 / 1,632 / EA97AAFC) |

- **The margin is 2%, not the planned 12%.** The standalone spike runs 103.3 / 105.2 /
  101.6 MHz, or 112.8 / 108.7 / 108.5 with its inputs registered, and swings about
  ±5 MHz per seed between batches.
- **Paths that remain:** inside the FPU (the FMA and divider, −0.44 to −0.94 ns on
  some seeds), MEM's replay hold into the fetch queue's pop, and the second redirect's
  mux.
- **Wrong turns, both reverted:** a CR reader bit through a 32:1 select (−2.08 ns), and
  an FPR one-hot from the decoder (−1.04 ns).
- **Two-cycle load use, built as a patch** (`sm3-builds/tp_lu2_patch.py`), not in the
  tree. It passes the bench, but costs **6.9 MHz** on average (110.0 → 103.2 MHz with
  registered inputs), because EX's operands then come through a mux. R52 puts its worth
  at 0.07 CPI on VF3, about 3.5%, which the 6.9 MHz (about 6.5%) outweighs. Not taken.
- **Checks:**
  - core, subsystem and cache benches pass;
  - each restructure has a Verilator equivalence check against the logic it replaced;
  - 42 of 46 planted faults caught. One miss (`fp_ok_fres`) is a generator gap: it never
    emits opcode 63 with bits [5:1] = 24.

**R57 — Framework switches and dual SDRAM measured: −1,053 ALM and −12 DSP.** These are
cached full builds of R56's RTL at 100 MHz. Framework = everything in `sys_top` outside
`emu`, plus `hps_io`.

| Build | Total ALM | M10K | DSP | Framework ALM | CPU Fmax |
|---|---|---|---|---|---|
| R56 as built, no framework switches | 18,319 | 159 | 40 | about 7,177 | 101.7 MHz |
| + `MISTER_DUAL_SDRAM` (D-26) | 17,994 | 159 | 40 | about 6,830 | 98.3 MHz |
| + `DOWNSCALE_NN`, `DISABLE_ADAPTIVE`, `DISABLE_YC`, `DISABLE_ALSA` | **17,266** | **157** | **28** | **about 6,141** | 92.8 MHz |

- **What each switch removes:**
  - **Dual SDRAM** removes only `vga_out` and `vga_scaler_out` (277 ALM).
  - **This framework version still builds `vga_osd` (about 520 ALM, about 4 M10K) and
    `csync_vga` (23) under `MISTER_DUAL_SDRAM`.** They are not only analog: MiSTer's
    direct video (native video over HDMI, used for CRTs through an HDMI-to-VGA adapter)
    takes its picture and csync from them (`sys_top.v` 1255–1259). Removing them would
    leave direct video without the OSD and without csync, so **they stay** for now.
  - **`DOWNSCALE_NN`** takes 12 DSP out of the scaler (`ascal` 1,974 → 1,846 ALM).
  - **ALSA and YC** go entirely.
  - **The audio filter is kept** (D-26): `audio_out` 871 ALM, against Model 2's 443
    with its patch.
- **Against Model 2.** Model 2's framework in its latest fit on disk (s1633, 30 Oct) is
  6,025 ALM with its audio filter removed; this core's 6,141 keeps the filter.
- **The CPU's Fmax swings with placement:** 101.7 → 98.3 → 92.8 MHz across these three
  builds, with the CPU unchanged. Every failing path in the last build is fetch
  translation into the I-cache's uncached decision: the fetch PC or redirect, then the
  IBAT compare, then WIMG, then `ppc_icache` `p_unc`. That is R56's cached-build worst path
  (+0.17 ns there), so it is real. It must come off the fetch cycle before R56's
  closure can be trusted.
- **From here,** full builds use `MISTER_DUAL_SDRAM` and the four switches, as the
  release will.

**R58 — The built FPU latency (6) and iterated doubles, priced: without dual issue the CPU
needs about 110 MHz.** Same runs as R52 (18,900 frames per title, all four 603e
references).
- **Latency 6 against 4 costs about 0.1 CPI in every row:** +0.10–0.11 on VF3, +0.08–0.09
  on Daytona 2, +0.04–0.05 on Bass Fishing. The rule that a fast op waits behind a double
  costs nothing on VF3 and about 0.01 elsewhere.
- **Single issue with the cheap levers** (BTB 256, no new-line cycle, way miss 1), the
  option D-27 leaves:

  | Clock | VF3: CPI / heaviest / lost against `ic` (midpoint) | Daytona 2: CPI / heaviest / over | Bass Fishing: lost against `ic` |
  |---|---|---|---|
  | 100 MHz | 1.85 / 126% / 7 (4) | 2.21 / 107% / 18 | 5 |
  | 110 MHz | 1.87 / 116% / 3 (0) | 2.24 / 98% / 0 | 0 |

  At 100 MHz it passes against neither the branch-folding reference nor the midpoint. At
  110 MHz it passes all three titles against the midpoint and loses 3 VF3 frames against
  `ic`. Those are the scene-cut frames of R34, where the real chip is near its own limit
  too.
- **With dual issue** (not built, D-27): BTB 256 plus dual issue passes all three at
  110 MHz against `ic`. At 100 MHz it still loses 3 VF3 and 5 Daytona 2 frames.
- **Dropping the pipelined double FMA** (about 2,000 ALM) for doubles iterated through the
  single unit costs about +0.11 CPI at 16 cycles, +0.25 at 24 and +0.40 at 32. Daytona
  2's double maths library pays most. Frames lost against `ic`, with BTB 256 plus dual
  issue:

  | Doubles | 100 MHz: VF3 / Daytona 2 / Bass Fishing | 110 MHz: VF3 / Daytona 2 / Bass Fishing |
  |---|---|---|
  | Pipelined, 12 cycles | 3 / 5 / 0 | 0 / 0 / 0 |
  | Iterated, 16 cycles | 7 / 18 / 4 | 3 / 0 / 0 |
  | Iterated, 24 cycles | 16 / 109 / 6 | 7 / 17 / 3 |
  | Iterated, 32 cycles | 22 / 429 / 8 | 13 / 54 / 5 |

  Only 16 cycles is near pipelined, and only alongside dual issue, which is not
  built. **So the pipelined double FMA stays.**
- **Conclusion for D-27.** A single-issue CPU with the cheap levers needs about
  **110 MHz in the full build** to keep the frames the 603e keeps. R56 closes 100 MHz
  by 2%, so roughly 10% of Fmax is still to find.

**R59 — Both SCSPs on one engine: bit-identical to two, −1,833 ALM, but sound is still
about 1,270 over D-27.** Model 2A's `m2_scsp` was copied at `47edd9dc` into
`rtl/sound/scsp/` (THIRD_PARTY.md). `scsp_dual` runs both chips through one engine and
one DSP, behind `SCSP_SHARED`. The two-engine build is kept for comparison.
- **The clock budget premise was wrong.** `m2_scsp` is a microcoded engine behind an
  output FIFO, not a 16-clocks-per-slot pipeline:
  - an active slot takes about 24 clocks plus fetch waits, an idle slot 1;
  - effect returns and output take about 101 clocks a sample;
  - the DSP takes 33 + 5 × LastStep clocks.

  One chip needs up to about 1,540 clocks a sample. So the shared engine runs on
  enables of a fast clock, as Model 2 does at 80 MHz. On VF3 it takes 1,462 clocks a
  sample pair on average, and it is DSP-bound (the two DSP runs take 1,433). At
  100 MHz that is 64% of the 2,268 available.
- **The interleave is a chip's whole sample at a time,** not one slot of each chip. Each
  chip's DSP runs while the other chip takes its turn.
- **Identity, proved on real streams.** MAME 0.289 played VF3 for 180 s through a Lua
  write-tap, no MAME patch: 6.1M and 5.8M register writes on the two chips (both are
  used), and 39.5M sound-RAM writes. Every output sample, slot sample, read, interrupt
  level and MIDI byte matched two separate engines:
  - **0 differences over 1,764,000 samples a chip,** at memory latencies of 6–40 and
    0–600 clocks;
  - also 0 on random streams with random DSP programs and 68000 RAM rewrites;
  - 14 of 19 planted faults caught; the 5 misses are unreachable by construction.

  The header lists six places where free-running behaviour can differ, all
  write-timing. The largest: a write can land between a chip's slots and its effect
  returns, for one sample.

| Build (both SCSPs, standalone) | ALM | M10K | DSP | Fmax |
|---|---|---|---|---|
| Two `m2_scsp` + port arbiters | 3,981–3,990 | 60 | 6 | 87.1–90.3 MHz |
| Shared engine, as first built | 2,424 | 37 | 3 | 85.4 |
| + COEF/MADRS in one M10K, FM ring in M10K | **2,144–2,154** | 39 | 3 | 86.9–89.5 |
| + shared 256-line caches | 2,144 | 35 | 3 | 89.5 |
| + FM off (VF3 never writes FM) | 2,116 | 38 | 3 | 87.2 |
| + chip 1's unused timers cut (its interrupt is not wired on Model 3) | 2,053 | 39 | 3 | 90.1 |

- **The engine misses 100 MHz** at 87–90 MHz. The paths are `m2_scsp`'s own (M10K
  output → ALU → side compare → flags → state RAM), not the sharing's. 80 MHz works:
  about 0.16% of sample pairs overrun the cycle budget and the FIFO absorbs them. An
  800 MHz VCO gives 100, 50 and 80 MHz, at the cost of a crossing to the SDRAM.
- **Where the 2,150 goes:**
  - engine core about 1,030;
  - DSP 580;
  - chip 0's common registers, timers and MIDI 235;
  - chip 1's common registers 121;
  - level 101;
  - LFO 60;
  - caches 52 each.

  The engine already shares one 27×18 multiplier and one adder across envelope, LFO,
  interpolation and mix. Getting to about 1,140 would need a narrower ALU taking two
  passes, a microcode rewrite that eats the cycle margin.
- **The 68000.** TG68K.C (LGPL-3.0, 68000 mode) is 1,808 ALM against fx68k's 2,142
  standalone, and reaches 58.9 MHz against 109. It does not aim for cycle accuracy, and
  VF3's driver polls the 0x408 monitor about 32,000 times a second per chip, the
  timing-sensitive pattern of Model 2's R885. fx68k stays.
- **Sound total:**

  | Part | ALM |
  |---|---|
  | fx68k | 1,859 |
  | SCSPs on one engine | 2,053–2,148 |
  | Glue | 357+ |
  | **Total** | **about 4,270–4,360** |

  That is against D-27's 3,000 (§8.1's old row was 6,073–6,316), so sound is about
  1,270–1,360 over and must be paid back elsewhere or the budget moved.

**R60 — Geometry area pass: 8,800 → about 7,170 ALM, and the rasteriser's set-up fits on
its pool.**

The default build:

| | R55 | Now |
|---|---|---|
| ALM, without pins | about 8,800 | **7,160–7,177** |
| ALM, with the 232 pin ALMs | — | 7,276–7,293 |
| M10K / DSP | — | 12 / 10 |
| Fmax at 10 ns | — | 111.5 / 104.1 / 104.7 MHz |
| p99 cycles, VF3 / Bass Fishing | 62.9% / 63.3% | **63.5% / 63.8%** |

The default build is still bit-exact on all 800 frames, and every build of
`test_real3d_geo3_all` passes. 44 of 50 planted faults are caught; the misses are
cases these frames never reach.

**Savings by §8.4 technique** (seed 1, before → after):

| Technique | Change | ALM |
|---|---|---|
| Storage out of flops | row 18's normal memory and face-normal path removed (D-25) | 250 → 0 |
| | vertex slots by reference: 4 memories → 1 | 80 → 20 |
| | clip lists by entry index | about −80 |
| | polygon rings, stream FIFO and walker return stack 64 → 32 deep | about −200 in all, for +0.4% of a frame |
| Duplicated arithmetic built once | one int16 converter for viewport words and u/v | about −120 |
| | one /255 product | about −60 |
| | outcodes as 3 magnitude compares (miter: 200M cases, 0 different) | about −30 |
| | cull verdict as sign + magnitude | about −30 |
| | one adder for all culling-RAM addresses | about −60 |
| Narrower arithmetic | the divide's remainder test 51 → 30 bits (\|d\| < 2^26; 20.7M divides and 72.3M reciprocals, 0 wrong) | 365 → about 300 |
| One shared pool | Supermodel's frustum as one kernel; R50's broadcast side channel removed | about −200 |

**Further trades, built as parameters and off by default.** Each one spends M10K to save
ALM:

| Parameter | ALM | M10K |
|---|---|---|
| `SIDE_M10K` | −336 | +16 |
| `OUT_M10K` | −228 | +4 |
| `BC_M10K` | −140 | +8 |
| All three | −685 | +28 |

Dropping one of the four pool lanes would save about 880 ALM and 2 M10K, for an
estimated +7–10% of a frame and a rewrite of the generator and controller.

**The rasteriser's triangle set-up on the geometry's pool** (p99, share of a 100 MHz
frame):

| Configuration | VF3 | Bass Fishing |
|---|---|---|
| No set-up | 63.5% | 63.8% |
| Set-up, two reads | 100.7% | 99.0% |
| Set-up, third read port | 93.7% | 90.3% |
| Third read port, facing test on the pool | 96.0% | 88.8% |
| **Third read port, facing test beside the pool** | **83.6%** | **79.6%** |

All rows except the first two include loading each kernel's inputs.
- **Facing test beside the pool:** this one fits under 85% on both games. Back faces
  are 37% of VF3's polygons and 25% of Bass Fishing's, and they are dropped before
  set-up.
- **Cost:** about 250–450 ALM, 1–2 DSP and 7 M10K on the geometry side, against
  1,300–1,600 ALM and 4 DSP for the rasteriser's own set-up unit. The output format is
  in `m2-notes` §14.
- **VF3's margin is only 1.4 points.** Daytona 2 has about 46% more new vertices at
  p99, so it must be measured before the rasteriser drops its own unit.

**A Quartus 17 trap.** A 2-D unpacked array (`logic [4:0] clst [2][16]`) segfaults
`quartus_map` in elaboration. `--analyze_file` passes it; the flat form works.

**R61 — BTB built and verified. Two-word fetch is the strongest single-issue lever, and
R52 overstated the new-line cost.**

**The BTB.** `rtl/cpu/ppc_btb.sv` is built and verified but not yet in the core.
- **Configurable:** 64, 128 or 256 entries, in M10K or MLAB. The tag includes MSR[IR,PR].
- **Hit rate** against the CPI model's own 256-entry BTB, branch by branch:
  - VF3 89.07%, Daytona 2 72.96%, Bass Fishing 75.14%;
  - 0 mismatches on every built policy;
  - 11 of 11 planted faults caught.
- **Two ways into fetch:**
  - **Design D** looks up at the branch's own request. A hit costs 1 cycle, and it needs
    no cache change.
  - **Design C** looks up one request ahead. A hit costs 0, but it needs a pre-read port
    in the I-cache.
- **Dynamic direction.** 72% of the bc branches that the static rule predicts not-taken
  are taken. Correct predictions as a share of taken branches:

  | | VF3 | Daytona 2 | Bass Fishing |
  |---|---|---|---|
  | D, static | 64.8% | 44.6% | 35.4% |
  | D, dynamic direction | 76.1% | 62.6% | 70.6% |
- **Size:** 64 entries keeps the hit rate, at 48 ALM and 2 M10K (or 107 ALM in MLAB).
  Fmax is far above need, 177–228 MHz.
- **Design C's pre-read port** adds 173 ALM to the I-cache (457 → 630). It takes the
  cache's standalone Fmax from 117–129 MHz down to 106–117 MHz.
- **Core side:** about 150 ALM, plus 100–150 for bclr/bcctr targets.

**R52's base charged too much for crossing a cache line.** The built I-cache's lookahead
crosses a sequential line at 0 cost. It pays +1 only when a jump lands at words 5–7.

| Line crossings per instruction | VF3 | Daytona 2 | Bass Fishing |
|---|---|---|---|
| Charged by R52's model | 0.090 | 0.108 | 0.096 |
| Paid by the built cache | 0.022 | 0.026 | 0.027 |

The real core is about 0.07 CPI better than R52's base.

**Two-word fetch is the strongest single-issue lever measured.** It costs about 350 ALM:
queue push of 2 (+152 measured), a second predecoder, and a 64-bit I-cache output path.

| At 100 MHz, single issue | Frames lost against `ref66_dual_ic` (all three titles) | CPI VF3 / Daytona 2 / Bass Fishing |
|---|---|---|
| BTB C, R52's model | 63 | — |
| BTB C, with the real 1-word fetch queue modelled | 32 | — |
| BTB C + 2-word fetch | 12 | — |
| BTB C + 2-word fetch + dynamic direction | **3** (3 / 0 / 0) | **1.726 / 2.055 / 1.986** |

At 110 MHz, single issue with BTB C 256 or 64 (no new-line cycle, way miss 1) loses 0.

**Not worth building:**
- **Way miss cut to 1 cycle:** an estimated 100–150 ALM on a known full-build path, for
  0.01 CPI and 0 frames.
- **The residual new-line fix:** a second tag copy (an estimated +2 M10K and 40 ALM),
  for 0.02–0.027 CPI.

**Dual issue, for the record (D-27, not built).** Prototype-measured, it adds 1,513 to
at least 2,287 ALM in its datapath variants. Branch as the second op is 344 ALM plus
core steering. The best variant loses 0–3 frames, where single issue with 2-word fetch
loses 3. With 2-word fetch, single issue is within a few frames of dual issue for about
a sixth of the area.

**R62 — Rasteriser at 100 MHz, first three blocks: 2,589 of 8,000 ALM, and culling must
come from the normal, not the screen winding.** Built and benched bit for bit against
the RTL-form model (now moved to D-28 half width), all closing 10 ns on three seeds:

| Block | ALM | M10K | DSP | Fmax | §8.4 |
|---|---|---|---|---|---|
| Front: slope divider and band walker | 1,595–1,600 | 0 | 0 | 105.2–114.3 MHz | The divider's three normalisers became a fixed 16-bit skip on a 2:1 select: divider 2,510 → 900 ALUTs, front 2,513 → 1,598 ALM |
| Set-up intake (D-25/R60 option (a): coefficients from the geometry) | 310–313 | 0 | 0 | 146.8–156.2 MHz | One FP→fixed converter for 21 conversions a textured triangle |
| Z span, 2 a clock | 676 | 0 | 2 | 127.8–141.3 MHz | One multiplier for both span products; DDA lanes per pixel |

**Checks:**
- **Divider:** 5.45M list triangles and 2M random, 0 wrong.
- **Walker:** 40.2M spans, plus 38.5M synthetic at 7-row bands, 0 wrong.
- **Intake:** 184M fields, 0 wrong.
- **Z span:** 675M pixels, 0 wrong.
- **Planted faults:** 16 of 16 caught. Five of them only by the random sets.

**Findings:**
- **Culling must come from the normal.**
  - The screen-space winding agrees with Supermodel's view-space normal test on only
    58–69% of single-sided triangles. Culling by winding makes VF3 worse: 13.50 → 12.19
    dB p50, and coverage disagreement 4.46% → 8.82%.
  - Culling by the normal, sent as a flag in the record, leaves the image unchanged and
    takes VF3 p99 from 66.9% to 53.6% of a frame.
  - **This bears on R60.** Its 83.6% / 79.6% used the model's normal verdict, but the
    hardware form of "facing beside the pool" was a 2-D cross product. The facing test
    has to use the normal, and R60's figure must be re-measured with it. The normal test
    on the pool measured 92.1% for VF3.
- **Depth on 1/w,** not MAME's w, is the default. It is image-neutral (VF3 13.63 against
  13.61 dB). It saves 12 of 66 set-up products and about 16 B per stored triangle, and
  gives fog a single path.
- **Translucency, three modes in the model** (p99 share of a 100 MHz frame with FIFOs;
  p50 PSNR against Supermodel):

  | Mode | VF3 frame | Bass Fishing frame | VF3 PSNR | Bass Fishing PSNR | Extra M10K |
  |---|---|---|---|---|---|
  | 0, MAME blend | 57.1% | 26.3% | 13.63 dB | 19.05 dB | 0 |
  | 1, 4×4 ordered-dither stipple | 38.5% | 26.2% | 13.45 dB | 19.05 dB | 0 |
  | 2, Supermodel's two layers | 45.8% | 26.3% | 14.12 dB | 19.13 dB | about +36 (8-row bands) |

  R49's hardware video leans to a blend.
- **8-row bands** cost nothing at the frame level, because shading binds.
- **Integer fog matches the float form** (Bass Fishing 19.13 against 19.12 dB p50).
  R47's fog-alone figure of 21.3 dB is not reproduced here: this harness gives 19.12.
- **Estimate for the whole rasteriser:** 7,000–7,150 ALM, 70–80 M10K and 28–32 DSP.
  The front's two savings, estimated at −350 ALM, measured −148 (front 1,598 → 1,450,
  registers −720), because registers share ALMs with logic as in Model 2's R952. The
  ledger now stands at 2,439 of 8,000 ALM for three blocks.
- **The geometry record must add:**
  - 1/w for untextured triangles;
  - the cull flag from the normal;
  - header[6] bits 11–15 (fog) and bit 17 (translucency layer);
  - a viewport record with fog colour, ambient, density, start and kz.

  It must also drop the w plane.

**R63 — The fast FPU in the core, on the board at 100 MHz; closure holds on one seed in
three.** D-21 and D-22 are wired into `ppc_core` as R53 listed:
- `ppc_fpu_fast`, with LS = 6;
- frsqrte on the FMA port as the estimate;
- the route decision registered beside the operands;
- fast write slots, and a fast-issue wait;
- the ISS built with `-DPPC_FPU_RSQRTE_EST`.

The wait register is held inverted (`ffbl_q`, reset 0), because `ffok_q = 0` at power-up
broke the schedule check in the board harness's first cycle.

**The I-cache uncached path (R57).** `i_wimg[2]` is registered raw and the reserved-page
compare uses the physical address. Both take effect from the first pending cycle, which
is the tag-check cycle anyway. It costs zero cycles: the cache and subsystem benches are
cycle-identical, and `p_unc` has left every failing path list.

**Generator gaps closed:**
- every unassigned 59/63 encoding is now emitted, including fres on 63, so `fp_ok_fres`
  is caught;
- FP dependency patterns added: a multiply-add result reused, and a burst of
  multiply-add, access and move.

Without the dependency patterns a route-forwarding fault went uncaught. A
multi-cycle-FSM fault also became uncaught, because the fast path leaves fewer results
in flight.

**Checks:**
- core bench: 5,000 seeds, 16.6M instructions, including back-pressure and data latency
  2;
- subsystem, cache, `test_fpu_fast` and `test_fpu_reg`: all pass;
- planted faults: 43 of 49 caught. The misses are R44's three, one generator gap
  (`pd_ctr_bclr`) and two equivalent mutants.

**The board program changed.** D-21 saves 94 cycles (2.0%) without cache and 62 (0.8%)
with it.

| Harness | Cycles | Instructions | Signature |
|---|---|---|---|
| No cache | 4,586 | 1,633 | 0CA977DF |
| Cached | 7,774 | 1,633 | 0CA977DF |

**Fit.** The CPU spike runs 100.6 / 101.1 / 106.3 MHz at 10,840–10,867 ALM and 9 DSP.
Like for like against R57's release form (17,266 / 157 / 28), the full build is +893 to
+964 ALM, +0 M10K and +2 DSP. Full builds in release form, at 100 MHz:

| Form | Seed 1 | Seed 2 | Seed 3 |
|---|---|---|---|
| No cache (16.7K ALM, 121 M10K, 30 DSP) | −0.355 ns | −0.116 ns | **+0.294 ns (103.0 MHz)** |
| Cached (18.2K ALM, 157 M10K, 30 DSP) | −0.402 ns | **+0.317 ns (103.3 MHz)** | −0.034 ns |

The failing paths are earlier core paths pushed out by the FPU's area:
- the fetch-queue pop;
- the EX address into the D-cache request;
- the harness redirect into its ROM;
- the route register;
- the single unit's first stage.

**On the board,** run on 2026-10-31 at 19:07 with the closing seeds:
- **no cache, seed 3** (RBF md5 dfe28134…): 1,395 of 1,395 reports identical to the
  simulation;
- **cached, seed 2** (RBF md5 0c960225…): 1,391 of 1,391 identical.

**100 MHz closure is not yet robust.** The CPU needs a timing pass for margin, and
R58's 110 MHz target still stands.

**R64 — M2.5: the first Model 3 image on the MiSTer, bit-exact with the bench.** The
viewer (`quartus/m3_viewer`, D-31) draws VF3 frame 7860 from a geometry dump:
- the real `r3d_geo`;
- the real rasteriser intake, front, zspan and zband;
- three temporary blocks that later work replaces.

| Temporary block | ALM | Stands in for |
|---|---|---|
| `m3v_setup`, one multiplier, adder and reciprocal | 1,831 | set-up on the geometry pool (R60) |
| `m3v_store`, unbinned DDR3 lists | 1,096 | D-20's binned polygon store |
| Gouraud shader, every texel white, fog off | about 535 | textures and fog |

Every band replays every triangle, so a frame takes 0.25–0.52 s. The set-up holds the
geometry for 6.66 M of its 7.07 M busy cycles. The 3D layer is 32 bits a pixel (Model 2's
`m2_fb_read` format) rather than D-6's 16, so the board can be compared bit for bit.
Everything runs on the primary SDRAM: the release form still has `MISTER_DUAL_SDRAM`,
with the second module's pins tri-stated.

**Bench.** Frames 1800, 7800, 7860, 7920 and 13770 are bit-exact against the RTL-form
model:
- every set-up record;
- every list count;
- 0 of 190,464 layer pixels differ;
- the DDR3 model at 20–60% busy and 2–120 cycles of latency.

Planted faults: 12 of 15 caught. One miss is equivalent at the image, because a negative
shade clamps to 0 either way. The other two (flat triangles, and a polygon-RAM read that
ends on an odd word) do not occur in frame 7860.

**Fit, release form at 100 MHz:**

| Seed | Fmax | Slack | Note |
|---|---|---|---|
| 1 | 106.7 MHz | +0.62 ns | no critical warning |
| 2 | 108.0 MHz | +0.74 ns | critical warning on another corner |
| 3 | 105.6 MHz | +0.53 ns | no critical warning |

All three seeds use 23.2K ALM, 118 M10K and 52 DSP. `r3d_geo` inside the viewer is 8,107
ALM, 20 M10K and 11 DSP. This batch was built without `AUTO_SHIFT_REGISTER_RECOGNITION
OFF`: synthesis put delay pipes in `r3d_recip`, `r3d_fp_add` and `r3d_face` into RAM shift
taps. The setting is now in the viewer's qsf. These area figures are not the study's until
a batch with the setting is fitted.

**On the board,** 2026-10-31 at 21:11, seed 1 (RBF md5 b3ee2a03…). It was loaded by an
MGL that loads both files, so no OSD input is needed. Main's screenshot is 496×384 and
matches the bench's `f007860_rtl.png` on every pixel outside the hex overlay. The
overlay reads 9,424 triangles, 0 dropped, 262,314 pixels and lists 7,281 / 2,139 / 4, as
the bench does.

**Findings:**
- **Intake tag 15** marks both the three header beats and plane 1's dy (8 + 4 + 3). A
  consumer must count header beats; reading dy as a header word gave 120 pixels the wrong
  shade.
- **SDRAM read capture.** With `rd_lat_sel` = 2 (Model 2's board value), data comes 2
  cycles after READ. `m2_sdram`'s comments on that index are out of date.
- **Geometry record.** The record changed during the work to 4 + 6n words, with viewport
  records and 1/w. The viewer follows the new form and drops the viewport records, so it
  has no fog yet.
- **Wrong turns:**
  - the first loader check windows sat in all-zero polygon RAM and missed a lane-swap
    fault; they now start at each region's first non-zero word;
  - the fit script did not rewrite `SEARCH_PATH`;
  - the shift-tap setting was missed in the first batch.
- **Traffic at 57.52 Hz, frame 7860:**
  - SDRAM: 31.9 MB/s transferred, about 25% of the usable bandwidth;
  - DDR3: 28.4 MB/s of polygon RAM, 43.8 MB/s each of layer writes and scan-out;
  - D-20 bins would bring the real-time DDR3 total to about 180 MB/s at 16 bits a pixel.

**R65 — The facing test from the normal costs about 780 ALM, and Daytona 2 cannot carry
the set-up on the pool.** R60 estimated 250–450 ALM for "port plus facing", but that
assumed a 2D cross product. The real test is a dot of the transformed vertex with the
transformed normal: 21 FP32 products and 17 sums a polygon. It is built as a dedicated
multiply-add beside the pool (`r3d_face.sv`), reusing `r3d_fp_mul` and `r3d_fp_add`:
766–792 ALM, 1 DSP, 0 M10K. The adder alone is 332–336 ALM.

**What did not work:**
- **Cheaper forms.** Measured on 4.07M polygons, every model-space shortcut is wrong:
  - Q = M·Mᵀ and r = M·t with an exact integer d: 28K–806K VF3 polygons wrong, from
    cancellation;
  - the eye position assuming a similarity: 31 wrong;
  - the eye position with the true inverse: 840 wrong.

  Nothing under about 20 bits of precision avoids an adder.
- **The pool's idle slots.** These would save the adder and multiplier, but Daytona 2's
  heaviest frames leave the engine only 24% idle.

**The facing model is now FP32 in the hardware's order:**
- V_j = ((x·T0j + y·T1j) + z·T2j) + T3j
- N_j = (n0·T0j + n1·T1j) + n2·T2j
- d = (V0·N0 + V1·N1) + V2·N2

Against the old double test it differs on 0 of VF3's 2,930,333 polygons, 0 of Bass
Fishing's 1,133,455, and 14 of Daytona 2's 2,136,440.

**Back faces are dropped in the geometry by default (`FACE_DROP=1`),** before their clip and
projection. They are 42% of single-sided polygons in all three games.

**Record format, R62's fields.** A polygon record is 4 + 6n words:
- per vertex: x, y, 1/w, 256u/w, 256v/w and shade;
- 1/w always present; window z and w dropped;
- word 3 carries the fog intensity, the translucency layer and the facing bit;
- a 6-word viewport record, marked by nv = 0, carries fog and kz.

SETUP5 becomes SETUP4 (no w plane): 63 ops, 97 cycles.

**Step 2.1.** The RTL takes a `step15` input, and the Daytona 2 model matches MAME on every
field over 400 frames of each dump set (4.17M triangles).
- A fresh Supermodel run did not reproduce the 2026-10-30 Daytona 2 dumps: all 5 frames
  checked differ, unlike VF3 and Bass Fishing.
- A new set of 400 frames is in `dumps_daytona2_r62`; the old set is kept.

**Checks:**
- RTL bit-exact on 800 VF3/Bass Fishing frames in eleven builds, and on 400 Daytona 2
  frames;
- `test_real3d_face`: 900K random polygons, none wrong;
- planted faults: 59 of 65 caught. All 15 new ones are caught; the 6 misses are R60's six.

A real bug was found: the decoder wrote a normal into a facing slot still in use. It gave
112 wrong verdicts in VF3 frame 190, all on a model's first polygon. It is fixed and
planted as FA10.

**Fit, 10 ns, three seeds:** 8,077–8,084 ALM without pins, 12 M10K, 11 DSP, 105.6–112.6 MHz.
That is +800 over R60. It is inside D-27's 8,550 but about 480 over the 7,600 target, before
the set-up port (estimated 150–300 more).

**Frame load, p99 of a 100 MHz frame over 400 frames:**

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Without the facing test | 62.5% | 62.1% | 96.9% |
| Default, no set-up | 61.3% | 56.9% | 93.9% |
| With set-up on the pool | 78.7% | 73.2% | **120.7%** |

**Why Daytona 2 is heavy.** It is bound per node and per model: 2,582 models and 3,927 nodes
a frame at p99, against VF3's 1,096 and 1,831.

| Share of the frame | % |
|---|---|
| Node culling (CULLU + CULLT, 93 latency-bound cycles a node) | 15.7 |
| Transform and light | 27.6 |
| Matrix product | 9.4 |
| Model set-up | 9.4 |

R20 sized the geometry from the vertex count alone, which was wrong for Step 2.x.

**Open:**
- Option (a), the set-up on the pool (D-28), holds for Step 1.0 but not for Daytona 2.
- The geometry alone leaves Daytona 2 6% margin at p99.
- Area options to pay back the 480 ALM:

| Option | ALM | M10K |
|---|---|---|
| `OUT_M10K` | −228 | +4 |
| `BC_M10K` | −140 | +8 |
| `SIDE_M10K` | −336 | +16 |

**R66 — Rasteriser blocks 1–6 built and exact; texture fills, not logic, now bind the
frame.** Built:
- the Z band;
- the shading interpolants with a stallable copy of the reciprocal;
- fog's depth term;
- colour with the fog blend;
- the texel path (8 formats, bilinear);
- a 32 KiB 4-way texture cache.

All are bit-exact against `render_d25`: 136–208 M pixels on VF3 + Bass Fishing frames per
block, plus 3–5 M random cases each, and the reciprocal is exhaustive over its 8,388,608
mantissas. Planted faults: 36 of 38 caught; the two cache misses are near-equivalent
mutants.

**The intake now takes R65's record by default (`SETUP4=1`).** The header goes out as 4 beats
on tag 31, so no tag is shared any more (R64's tag-15 clash is gone). The planes come in the
order 1/w, u/w, v/w, shade. The M2.5 viewer was built on the old intake, so it must move to
this form before its next build.

**Area, ALM counted without the virtual pins.** Inside the rasteriser the pins are wires;
the interpolants alone had 318 pin ALMs.

| Block | ALM | M10K | DSP | Fmax at 10 ns |
|---|---|---|---|---|
| Z band | 632–642 | 12 | 0 | 114–119 MHz |
| Interpolants and reciprocal | 1,192–1,195 | 0 | 15 | 114–122 MHz |
| Fog depth term | 139 | 0 | 1 | 132–145 MHz |
| Colour and fog blend | 130 | 0 | 13 | 172–182 MHz |
| Texel | 527–528 | 0 | 12 | 112–115 MHz |
| Texture cache, 32 KiB | 972–974 | 43 | 1 | **47–50 MHz, fails** |
| Intake, SETUP4 | 269 | 0 | 0 | 142–154 MHz |

Built so far: 5,717 ALM, 55 M10K and 44 DSP, with the first three blocks (R62) included.
The unbuilt blocks are estimated at 1,500–1,850 ALM: band buffers and blend, the D-20 store
client, and control. That projects to 7,300–7,700 of 8,000, with M10K at about 85–95
against 70–80. DSP is over plan, 44 against 28–32: Quartus 17 puts one product in every
block, even in its two-18×18 mode. Moving colour's small products to logic would save 7 DSP
for about 150 ALM.

The cache's worst path is tag read, hit, victim choice and tag write in one cycle. It needs
the lookup split into compare and allocate stages.

**Texture bandwidth, p99 share of a 100 MHz frame, level-0 sampling:**

| Case | VF3 | Bass Fishing |
|---|---|---|
| Z walk alone | 33.6% | 20.2% |
| Cache RTL, 32 KiB, harsh memory model | 132.7% | 46.6% |
| 32 KiB, ideal 64-bit port | 89.0% | 30.9% |
| 64 KiB, ideal 64-bit port | 67.1% | 30.3% |
| 32 KiB, 16-bit SDRAM | 274.8% | 99.6% |

VF3's p99 frame needs about 300 K line fills (547 MB/s) at 32 KiB and about 425 MB/s at
64 KiB. That agrees with R16/R18 at p50 and is worse at p99. A 16-bit SDRAM cannot carry
it at any cache size I can afford.

Supermodel samples mip levels (m3-notes, row MIP). The rasteriser model so far samples
level 0, which reads far more texels than the hardware for every minified texture. D-25's
LOD is the geometry's model LOD (row 6), not this. The levers are:
- the mip level, nearest-level or trilinear;
- a 64 KiB cache;
- a 64-bit texture port;
- the Z pre-pass.

The mip level's effect on fills is measured next, before a decision.

**Wrong turns:**
- The Z band had a verdict race. An old generation's verdict landed behind the sweep, and
  its stamp came round live again 64 generations later; only one synthetic seed showed
  it. Rule: a verdict is dropped once the sweep starts or when it is stale.
- The cache's beat counters were one bit short, so a full 32-beat buffer read as empty and
  the cache locked up on real frames after 169,953 pixels.
- The cache simulation first chained each lookup to the previous pixel, which hid the
  look-ahead.
- Timing splits were needed in the Z band (75–80 → 114 MHz), colour (93–96 → 172 MHz) and
  texel (92–93 → 112 MHz).
- An inline `genvar` was rejected by Quartus 17.

**R67 — Mip levels cut texture fills by a third to a half. No SDRAM form carries VF3; DDR3
needs a pipelined master. The texture cache and the band stage close at 100 MHz.** render_d25 now has Supermodel's mip levels. Before this only
`render_sm` had them, and level 0 had been the model's default. The settings are
`d25_opts::mip` (0 = level 0, 1 = nearest level, 2 = trilinear) and `lod` (0 = Supermodel's
per-pixel LOD; 1 = a hardware form, below).

The fill tool now replays the RTL's round-robin victim rather than LRU. With LRU it
reproduces R66 exactly; the RTL's rule costs 1.8% (VF3) and 7.2% (Bass Fishing) more p99
fills.

**VF3, p99 share of a 100 MHz frame over 400 frames, textured cycles:**

| Form | Cache | Fills p99 | MB/s | Ideal 64-bit | SDRAM, `m2_sdram` | DDR3 pipelined, latency 13 / 30 / 60 |
|---|---|---|---|---|---|---|
| Level 0 | 32 KiB | 302,324 | 557 | 90.7% | 390.8–406.2% | 102.2 / 117.0 / 183.1% |
| Level 0 | 64 KiB | 222,723 | 410 | 67.3% | 287.2–298.2% | 74.7 / 81.9 / 128.9% |
| Nearest | 32 KiB | 192,391 | 354 | 64.7% | 250.5–259.5% | 70.0 / 75.9 / 110.2% |
| Nearest | 64 KiB | 166,758 | 307 | 57.2% | 220.6–228.1% | 60.9 / 64.9 / 97.3% |
| Trilinear | 32 KiB | 204,853 | 377 | 86.8% | 266.2–276.6% | 93.1 / 99.7 / 128.0% |
| Trilinear | 64 KiB | 180,026 | 331 | 79.6% | 235.5–244.2% | 84.3 / 88.6 / 106.3% |

**Bass Fishing** at 32 KiB, p99:

| Form | Fills | SDRAM | DDR3, latency 30 |
|---|---|---|---|
| Level 0 | 112,590 | 132.7% | 41.1% |
| Nearest | 58,200 | 76.7% | 34.3% |
| Trilinear | 76,032 | 100.0% | 44.3% |

**The memories:**
- **SDRAM:** modelled as `m2_sdram` with one transfer at a time, 19 / 21 / 24 cycles a line for
  a row hit, a closed bank and a conflict, plus refresh. R66's 16 cycles a line was
  optimistic: level 0 at 32 KiB is 390.8%, not 274.8%.
- **DDR3:** modelled as a pipelined master on the 64-bit f2sdram port, shared with R64's
  measured clients. With D-20's 180 MB/s of other traffic at latency 30, VF3 nearest is
  80.3% (32 KiB) and 67.9% (64 KiB). With 16 fills in flight at latency 60, it is
  75.3% / 65.2%.
- **`m2_ddr3` as built,** one request at a time, gives no gain: VF3 nearest at 32 KiB is
  229.7%.

**Findings:**
- **The nearest level is the largest single lever.** VF3 p99 fills fall 36% (p50: 51%), and
  Bass Fishing's fall 48%.
- **Trilinear costs more cycles than its fills save.** 36–41% of textured pixels need a
  second quad, and VF3 p99 is 72.6% of a frame before any miss.
- **No 16-bit SDRAM form carries VF3;** the best is 220.6%. Textures need the DDR3 port with
  a pipelined master. Two things are not yet known: whether MiSTer's f2sdram bridge accepts
  more than one pending read, and the core's DDR3 latency. The only board figure is
  `m2_ddr3`'s header: 13 cycles typical, 50 worst (Model 2 R351). Both must be measured on
  the board; the result decides 8 against 16 fills in flight and the cache size.

**Image cost against Supermodel's grabs, p50 frame PSNR:**
- level 0: VF3 13.63 dB, Bass Fishing 19.05 dB;
- nearest: VF3 13.72 dB, Bass Fishing 19.17 dB;
- trilinear: VF3 13.72 dB, Bass Fishing 19.16 dB.

Nearest and trilinear are equal to 0.01 dB.

**LOD in hardware form.** The LOD needs one FP32 word per polygon from the geometry:
B = d·NP·(−cota)·kz².
- **Where the terms come from:** d is `r3d_face`'s facing dot, NP is header[5] bits 8–31 /
  4096, and cota is viewport word 3.
- **What it adds:** the record becomes 5 + 6n words. The two products fit in the face unit's
  idle slots.
- **Rasteriser cost:** 160–230 ALM, 0 DSP, 0 M10K, by estimate.
- **Against Supermodel's per-pixel form:** it changes 1.14% of VF3 pixels, at +16.7% p99 fills.
- **Not checked:** whether `r3d_face`'s d has the model's scale. A factor of 2 is a whole
  level.
- **Supermodel's exact form** would need a fifth plane, d/w, for 0.00 dB more.

**Block 7, `r3d_rast_band.sv`:**
- two 8-row band buffers by x parity, D-29's plain blend done exactly as d + ((c−d)f >> 8),
  and a flush on `m2_ddr3`'s request contract (D-20's switch);
- **fit:** 444–448 ALM without pins, 20 M10K, 2 DSP, 123.6–126.1 MHz;
- **checks:** 0 wrong on 147.3 M VF3 and 60.2 M Bass Fishing pixels, 2,000 random frames, and
  busy and gap stress runs; 11 of 11 planted faults caught.

**The texture cache closes at 100 MHz once split.** Its lookup is now four stages: read, compare, allocate, then a tag write. Compare re-reads the tags while it waits, and forwards the tag writes on the read edge and in flight. The victim is chosen for each candidate bank in parallel, and rows are updated by loops over constant ways.
- **Fit, three seeds:** 106.0–108.3 MHz at 781–790 ALM without pins, 43 M10K and 0 DSP. R66's form was 972 ALM, 1 DSP and 47–50 MHz.
- **Checks:** 0 wrong on 800 frames, with the fill sequence checked address by address, and on random sets at 32 and 64 KiB. Planted faults: 13 of 13 caught. The fill-sequence check also catches R66's two misses.
- **The first split failed:** 73.6–80.0 MHz at 1,813 ALM. Quartus 17 builds variable part-select writes (`row[vic*EB +: EB]`) as multipliers in the path. Rule: write rows with constant-index loops.
- **Not fitted:** the 64 KiB form (it is benched).

**Rasteriser built so far:** about 5,970 ALM without pins, 75 M10K and 45 DSP.

**Wrong turns:**
- Forwarding into the allocate stage was planned, then found unnecessary.
- The band block's write gate put band 0 into both buffers after reset; only the random set
  catches it, and it is now planted as G8.
- An inline `genvar` was written again and caught before analysis.

**R68 — D-32 built: the geometry is 7,532–7,568 ALM, 32 M10K, 103–106 MHz.**
`OUT_M10K` and `SIDE_M10K` are now the defaults, and nothing else had to change:
- the output buffer's read-ahead was already built for R62's record;
- the side memories are read registered either way.

**Checks:**
- **Bit-exact:** the default and set-up-load builds on 400 VF3, 400 Bass Fishing and 400 Daytona 2
  frames; every other build on VF3 and Bass Fishing.
- **Cycles:** identical to the old defaults in all ten builds.
- **Planted faults:** 64 of 70 caught, all five new read-timing faults included; the misses are
  R60's six.

**Fit, three seeds:** 7,532–7,568 ALM without pins, 32 M10K, 11 DSP, 103.1–106.3 MHz. That is
−527 ALM on average for +20 M10K. The memories gave −566, as predicted, and the controller logic
around them gave back about 35. The worst path is now the decoder's normal conversion, at
0.30–0.59 ns of slack; that logic did not change, so the fitter placed it worse. The geometry is
30–70 ALM under its 7,600 target, before the set-up port (an estimated 150–300).

**Wrong turn:** R65's first verify pass differs from these runs by a few hundred cycles at p99.
The old-default controls reproduce the new numbers exactly, so that pass ran on an earlier RTL
or bench, not on anything D-32 changed.

**R69 — DDR3 on this board, measured with one read at a time: 15 cycles typical latency, and
up to 719 MB/s per port with long bursts.** I used Robert Peip's MiSTerDDR3Test (GPL-3.0,
commit 1a11970, its release RBF md5 a93630ba…), read in `tools/ddr3test-ref`, on my MiSTer
on 2026-10-31 at 100 MHz.
- Its reader issues one read command, waits for every beat, then issues the next. So it
  measures the one-in-flight case, like Model 2's `m2_ddr3`, and not pipelined reads.
- It ran with the framework's scaler live (HDMI out), and nothing else in the core.
- The settings went in through `DDR3TEST.CFG` and the screen was read with Main's
  screenshot command: about 14 s a case, min/max over that time, average over 65,536
  transfers.

| Burst (64-bit words) | Bytes | Random 4 MiB, MB/s | Sequential, MB/s |
|---|---|---|---|
| 1 | 8 | 45.6 | 51.0 |
| 2 | 16 | 87.6 | 95.8 |
| 4 | 32 | 158.3 | — |
| 8 | 64 | 260.1 | 279.4 |
| 16 | 128 | 392.5 | 408.9 |
| 32 | 256 | 523.7 | 549.2 |
| 128 | 1,024 | 708.7 | 718.7 |

**Latency, command to first beat:**

| | Random | Sequential |
|---|---|---|
| Minimum | 12–13 cycles | 12–13 cycles |
| Average | 15.0–15.3 cycles | 13.3–13.4 cycles |
| Maximum over 14 s | 81–99 cycles | 81–99 cycles |

Bursts are interrupted part way (Burstwait non-zero), so a reader must take beats as
`DOUT_READY` gives them.

**What it shows:**
- **Throughput fits latency.** One in flight, a request costs about latency + beats + 1
  cycles. For a 32-byte line, 32 / (15.2 + 4 + 1) × 100 MHz = 158 MB/s, as measured. This is
  why Model 2 saw about 100 MB/s usable (§7.3): it used short requests one at a time.
- **The port itself streams 709–719 MB/s,** 89–90% of the 800 MB/s peak, so it is not the
  limit for R67's 307–354 MB/s.
- **Latency is better than R67 assumed.** R67 modelled 13–30 cycles; the average is 15. At
  13 cycles, R67 has VF3 nearest at 32 KiB at 70.0% of a frame.
- **Fills of 32-byte lines need several in flight.** One at a time gives 158 MB/s, under
  VF3's need. Three or more in flight cover it, if the bridge accepts them.

**At 125 MHz** (the core's clock option, set in the OSD; sequential, 128 words): 871 MB/s, 87% of the 1,000 MB/s peak. Latency there is 15 cycles at best, 17.5 typical and 98 worst. So latency is about 135–140 ns in real time at either clock, and a faster port clock helps long bursts but not short fills. 140 ns is at the best end of R67's 130–300 ns range, and about what Model 2's SDRAM controller takes for one 32-byte line (19–24 cycles).

Random reads at 125 MHz, 128 words, read off the screen on the board: 860 MB/s, against 871 sequential. With long bursts, random addressing costs almost nothing.

**Writes at 125 MHz** (set in the OSD; the core always writes one 64-bit word per command): 749 MB/s. That is 93.7 M writes a second, so the port accepted a write on 75% of cycles. The wait from command to acceptance was 0.72 cycles on average and 76 at worst. Writes are posted, with no wait for completion, and the port takes them back to back with only `BUSY` back-pressure. So the bridge does queue commands, which supports, but does not prove, several reads in flight. For the layer and store writes it means writing costs port slots, not latency.

Whether it does accept them is still unmeasured; the DDR3 probe core does that next. The
fallback, if it doesn't, is longer lines: 64-byte lines one at a time give 260 MB/s, at the
cost of more bytes per miss, which the fill tool can price.

**R70 — CPU timing pass: the spike gains 2–8 MHz, but only 3 of 6 full builds close 100 MHz.**
CPI and function are unchanged: bench cycle totals are identical across 16 regressions, and
both board-harness reports match R63 to the byte. The changes:
- predecode tables (`ppc_pdec.sv`);
- the FP route from predecode, with a stored not-single bit per FPR;
- the registered FP wait;
- register compares made early, with the holds choosing last;
- a pop split into copies, and fetch queue entries 2–3 as a ring;
- per-candidate micro-TLB and DBAT compares, the latter carry-free;
- sfma stage moves;
- the branch decision taken last;
- a redirect copy for fetch.

**Spike, 8.333 ns, seeds 1 / 2 / 3:**
- R63: 100.6 / 101.1 / 106.3 MHz;
- now: **108.3 / 108.6 / 110.1 MHz**, at 11,086–11,090 ALM (+219 to +250).

**Full builds, release form, 100 MHz** (slack, ns, seeds 1 / 2 / 3):

| Build | Seed 1 | Seed 2 | Seed 3 | RBF kept |
|---|---|---|---|---|
| No cache | **+0.147** | −0.286 | −0.512 | `tq_full_s1` (af42b94c…) |
| Cached | −0.303 | **+0.115** | **+0.075** | `tq_fullc_s2` (3b533c98…), `tq_fullc_s3` (cf2dee96…) |

R63 had 2 of 6 closing. The full builds swing about ±0.4 ns from fit to fit, more than any
single fix. Area is no cache 16.75K ALM (+27 to +50) and cached 18.07–18.14K ALM (−11 to −162),
because the predecode tables let Quartus trim the second decoder instance.

**The remaining failures are mostly at the core's edges:**
- the fetch-queue head copied into an SPR-MLAB address register, −0.51;
- register file → `op_x`, −0.29;
- the harness monitor input, −0.30;
- D-cache `wk_act` → `redir_a_q`, −0.30;
- the harness ROM address through the fetch IBAT compare, −0.1 to −0.36.

Two of these are test-harness paths that the real core does not have.

**Planted faults:** 75 of 84 caught. One generator gap is new: the bench never hits a DBAT with
BL ≠ 0. A DBAT unit test, 20M cases, catches it.

**Wrong turns:**
- `op_x`/`op_y` translation copies made the EX forward worse, so they were reverted.
- A D-cache compare split and `d_req`-last made the cached builds worse by 0.9–1.2 ns, so they
  were reverted too.
- The monitor's CRC loop was built as a 7-LUT chain until written as a flat XOR matrix.
- Spike gains often did not carry into the full builds.

**On the board,** 2026-11-01 at 00:24:
- **no cache, seed 1** (af42b94c…): 1,394 of 1,394 reports identical to R63's capture (P00 000011EA, P09 0CA977DF);
- **cached, seed 2** (3b533c98…): 1,391 of 1,391 identical (P00 00001E5E).

**R71 — A DDR3 client must not start at PLL lock: Main releases the FPGA-to-SDRAM ports only
after the core is running.** The first DDR3 probe builds hung on the board, 2026-11-01. Four
diagnostic loads of `fit_s11` (8284efe3…) showed where:

| Load | Result |
|---|---|
| Default | The 8 MiB pattern write appears to complete; the first read is taken and never answered |
| Init skipped (no writes at all) | The first read is still never answered |
| Single-beat init | Six writes are taken, then `BUSY` stays high for ever |
| Reads through `m2_ddr3`'s path | Never answered |

So the probe's burst handshake was not the cause. The port was not being serviced.

**The cause is in Main's load order** (`fpga_io.cpp` 378–400 and 480–497):
1. `do_bridge(0)` writes 0 to the SDRAM controller's `fpgaportrst` (0x5080), which holds every
   FPGA port in reset.
2. Main configures the FPGA; the core starts and its PLL locks.
3. Only then does `do_bridge(1)` release the ports.

The emu's `RESET` (sysmem `reset_out`) does not include `fpgaportrst`, so the fabric cannot see
it. A core that touches `DDRAM_*` within microseconds of lock talks to a port held in reset, and
the commands taken then wedge it.

The M2.5 viewer worked because its first DDR3 access came seconds later, after Main had loaded
files. Peip's tester waits for its hps_io reset/pause first.

**Rule:** every DDR3 client, in the probe, the viewer and the core, waits for hps_io's first
command, which Main sends after `do_bridge(1)`, plus a margin, before its first access.

**R72 — A wedged DDR3 port survives core reloads. Every probe result after the first hang
was taken on a dead port.** On 2026-11-01 at 01:47 I loaded Peip's DDR3Test again. It had read
158 MB/s at 4 words random at 23:37 the night before (R69). Now it showed 0 transfers, minimum
delay 65535 and 0 bytes/s.

`fit_s18`, gated on hps_io and on `RESET` low for 100 ms, showed:
- `RESET` low at the take;
- its read taken on the first cycle and never answered;
- four stray beats arriving before its first read, left from an earlier core's outstanding read.

**The sequence, most likely:**
1. The first probe builds (`fit_s4`/`s5`, 00:04) accessed DDR3 at PLL lock, before Main's
   `do_bridge(1)` (R71). That wedged the HPS SDRAM controller's FPGA port.
2. Main's `do_bridge(0)`/`(1)` on each later core load does not clear it.
3. Every board run since then, all four diagnostics included, ran on a wedged port. So their
   conclusions are void:
   - R71's diagnosis of the "first read never answered";
   - the "owner misroute";
   - the theory that both masters saw the take late.

**What stands:**
- R71's rule still stands as a hypothesis for how the port got wedged.
- The `RESET` and hps_io gates stay in every DDR3 client.

**What is not yet shown:** that the gated builds work, and whether a power cycle clears the
port. Both wait for a board power cycle.

**Rule:** a core that wedges the DDR3 port takes the board out of DDR3 testing until a power
cycle. After any DDR3 hang, run Peip's tester before drawing a conclusion from the next build.

**R73 — The geometry started a new model with the old model's read still waiting. Two
late-acknowledge bugs are fixed, and the geometry is now benched behind slow memories.**
The M2.5 viewer bench found it: on VF3 frame 1800, with polygon RAM data late, the geometry
emitted 7,613 of 7,621 triangles. One 16-polygon model was never decoded, and the records
after it shifted.

A per-cycle port trace from the viewer, replayed into `tb_r3d_geo`, put the cause inside the
geometry, and showed it depends only on port timing.

**Bug 1, `r3d_mstream`.** I believed the drain after `stop` covered every read of an abandoned
model. It covered only acknowledged bursts. A burst requested but not yet acknowledged survived
`stop`, and when its acknowledge came late it did one of three things:
- **Acknowledged in the cycle the decoder tested `ready_start`:** the start was ignored, and the
  geometry hung.
- **Acknowledged in the start cycle:** the new model streamed from the old model's next address.
  This is frame 1800's case.
- **Acknowledged after the start:** the old model's 8 words led the new model's FIFO.

Fix: `ready_start` also waits for no request pending.

**Bug 2, `r3d_geo` `M_END`.** `busy` fell while the stream's last prefetch of the frame was still
waiting at its port. An owner that hands the port on at `!busy` never answers it, and the next
frame hangs. The viewer kept its ports across frames, so it never saw this. Fix: `M_END` waits
for `ms_ready`.

**What triggers them is a late acknowledge (100–400 cycles), not long latency.** Every earlier
bench acknowledged in 1–3 cycles, so none could see it. The new bench, `tb_r3d_geo --hostile`
(`make test_real3d_geo_timing`), sets per-port acknowledge delay, BUSY, gaps, reads held and
output stall. A frame fails if `busy` falls with a port still open.

**Checks after both fixes:**
- **Default bench:** cycles identical on all 1,200 frames (VF3, Bass Fishing, Daytona 2).
- **Hostile profiles:** 1,200 of 1,200 frames exact under each of six, with acknowledges up to
  400 cycles, latency up to 200, BUSY up to 60%, output stall up to 90%; 6.57 M triangles each.
- **Viewer:** exact in 9 cases (1–8 reads held, BUSY 30%, latency 30–120), and on five frames at
  BUSY 40%.
- **Planted faults:** T1 and T2 re-create the two bugs.

**Planted faults:** 82 run, 75 caught, including T1 and T2. Nothing changed from R68's results. The seven misses are R60's six (N2, N4, N6, G5, G8, G11) and L8, the emitter's wait for the LOD word: that word is always ready long before the record leaves, so the wait never holds.

**Fit, 10 ns, three seeds:** 7,582 / 7,565 / 7,571 ALM without pins, 32 M10K, 11 DSP. These builds also carry the LOD word logic, off by default. Fmax by seed:

| Seed | Slow 100C | Slow −40C |
|---|---|---|
| 1 | 107.6 MHz | 103.6 MHz |
| 2 | 107.0 MHz | 106.8 MHz |
| 3 | 101.2 MHz | **97.9 MHz** |

Seed 3 misses 100 MHz at −40C. (I first wrote seed 1 here; that was a misreading of the report order.)

**Correction to R68: D-32's builds do not close at −40C.** R68 read the slow 100C corner only:

| Build | Slow 100C | Slow −40C |
|---|---|---|
| R68, `d32` seeds 1 / 2 / 3 | 103.1 / 104.0 / 106.3 MHz | **97.2 / 96.5 / 99.7 MHz** (all three miss 10 ns) |
| R65, `face3`, before D-32 | 105.6–112.6 MHz | 103.4–109.5 MHz (all three meet) |

Moving the output buffer and side memories into M10K cost the cold corner its margin. That matches R68's note that the worst path moved to the decoder's normal conversion. With the late-acknowledge terms added, two of three seeds meet both corners, which is seed variation, not a fix. The −40C path needs finding. From now on, every fit reports both slow corners.

**Wrong turns:**
- My first hostile profiles varied latency and per-port limits, but acknowledged quickly, and
  never triggered it.
- I called the fix done after the viewer passed; the sweep then found bug 2.

**R74 — The textured viewer is bit-exact in simulation and closes timing; the DDR3 probe
builds wait for the board.**

**The viewer now uses the real rasteriser blocks:**
- interpolants with the reciprocal, texel address and filter, colour, and band;
- the 32 KiB texture cache, on `m3_ddr3p`'s second pipelined port;
- texture RAM in DDR3, tiled, at word 0xC00000 + 0x100000·k.

The temporary shader, band and second Z walk are gone; removing them freed 714 ALM, 13 M10K and
11 DSP. Textures are level 0. There is no fog yet, because the bench frames carry no viewport
records.

**Checks:**
- All 5 frames match `render_d25` with textures under four slow-DDR3 profiles (BUSY up to 50%,
  latency up to 120, 16-read bridge). Frame 7860 equals `f007860_d25tex.png`.
- Planted faults: 18 of 19 caught; S2 is equivalent on these frames.

**Fit, release form, with both DDR3 gates (R71):**

| Seed | Fmax | Slack (setup) | ALM |
|---|---|---|---|
| 1 | 104.5 MHz | +0.43 ns | 26,276 |
| 2 | 101.9 MHz | +0.18 ns | 26,331 |
| 3 | 103.2 MHz | +0.31 ns | 26,308 |

All three seeds use 183 M10K and 79 DSP, with no critical warnings.

**The viewer's rasteriser in total:** 6,379–6,397 ALM, 91 M10K, 40 DSP. The large blocks:

| Block | ALM | M10K | DSP |
|---|---|---|---|
| Interpolants | about 1,150 | — | 15 |
| Texture cache | about 865 | 42 | — |
| Texel filter | about 425 | — | 12 |
| Band | 259 | 20 | 2 |
| `m3_ddr3p` | about 625 | — | — |

**DDR3 probe builds for the board, after a power cycle (R72):**
- `m3_ddr3p` with caps of 1, 2, 4, 8 and 16 reads in flight (`fit_s18`, `fit_s20`–`s23`);
- an `m2_ddr3` control (`fit_s19`);
- a variable-clock build (62.5–125 MHz, cap 32, `fit_s25`), which closes the DDR3 domain at
  125 MHz with +0.59 ns.

Each builds a register read-owner queue, and every DDR3 client has both gates.

**Timing finding:** a late port signal (`DOUT_READY`) through a 64:1 read mux missed 125 MHz by
0.39 ns. Choosing late between two registered head candidates (rq[rp], rq[rp+1]) fixed it.

**Bandwidth:** the framework's scaler (ascal) takes about 90 MB/s at 496×384 and 60 Hz, on its
own f2sdram port.

**Wrong turns:** the read-owner queue was chased three times and the f2sdram terminator once,
all on a port wedged since the first probe build (R72). The changes are kept, because each
removes a real hazard.

**R75 — Nearest-level mip sampling built, and R67's texture fills were far too pessimistic.**
D-33's nearest level is built end to end, behind parameters that default off:
- **Geometry:** `LOD_WORD` adds B to the record, which becomes 5 + 6n words.
- **Rasteriser:** `MIP` in the intake, interpolants and texel path.

**B is formed as built:**
- −cota·kz² is formed once per viewport: no product at all with Supermodel's frustum (kz = 1), and
  the idle face unit forms it with MAME's.
- B = d·(NP·C).
- `r3d_face`'s d has the model's scale: |d/dv − 1| is 5.6e-8 at p50 over 3.37 M polygons, and
  nothing sits at a factor of 2.

**Checks:**
- **Geometry, `LOD_WORD` = 1:** bit-exact on 1,200 frames, plus five other builds; +0.13% p99
  cycles.
- **Rasteriser, mip 1 / lod 1:** bit-exact against `render_d25` on every block and on the cache's
  fill sequence.
- **Planted faults:** 12 of 12 caught in the rasteriser and 9 of 10 in the geometry. The miss is
  a safety interlock that never holds.

**Area, 10 ns, three seeds, all closing:**

| Unit | ALM | M10K | DSP |
|---|---|---|---|
| Geometry LOD word | about +31 (7,597–7,616 in all) | 0 | 0 |
| Texel | +189 to +192 | 0 | 0 |
| Interpolants | +46 to +49 | 0 | 0 |
| Intake | +27 to +32 | 0 | 0 |

The rasteriser side totals +263 to +273 ALM, against the 160–230 estimated. With the LOD word on,
the geometry sits at its 7,600 target, up to 16 over.

**Shortcuts and a side fix:**
- The mip texel is exact only for size codes up to 5 and levels up to 9. That covers everything the
  geometry can send, and narrowing it saved 34 ALM.
- The face unit's adder zero-select was a slot late, and worked only because an unwritten MLAB word
  reads as zero. It is fixed in the default build too: cycles are unchanged and every frame is exact.

**The main finding: R67's fills came from MAME's lists, which draw a back-facing double-sided polygon
once, with its own normal.** Its B is then negative, so it samples level 0. The geometry, like
Supermodel, sends the inverted copy, whose B is positive, so it gets a real level.

Fills at p99 with the geometry's own B:

| | VF3, 32 / 64 KiB | Bass Fishing, 32 / 64 KiB |
|---|---|---|
| R67's figures | 192,391 / 166,758 | 58,200 / 49,545 |
| **Geometry's B** | **53,790 / 47,015** | **41,569 / 33,831** |
| DDR3, latency 30 | 56.5% / 55.7% | 31.4% / 29.7% |
| One 16-bit SDRAM, textures only | 77.5% / 71.1% | 58.3% / 50.7% |

The image does not move against Supermodel's grabs: 0.00 dB on VF3, −0.01 dB on Bass Fishing.

**Wrong turns:**
- A hand-typed log2 table was about a third wrong; it is now generated from the formula.
- A shared global matrix stack corrupted the multi-threaded fill runs; it is now `thread_local`.
- Two of my hand-computed constant-fold values were wrong. Quartus was right.

**This reopens D-33.** On these figures a 16-bit SDRAM of its own can carry VF3's textures at p99,
which R67 ruled out. DDR3 has about 44% of a frame to spare. The choice between DDR3 and the second
SDRAM, and the cache size, should be re-weighed once the DDR3 probe has board numbers (R72).

**R76 — Set-up on the geometry pool is built (D-28 a, `SETUP_OUT`), and the geometry now meets
both slow corners.** It matches `rtl_tri::setup` word for word on 6.57 M records over 1,200
frames (VF3, Bass Fishing, Daytona 2). It is also exact under the hostile timing profiles, and
the default build is unchanged cycle for cycle.

**How the inputs arrive.** There is no port load, unlike R65's model.
- A port gathers each triangle's sorted inputs into the projection queue's unused upper half of
  M10K.
- The kernel reads them through the side port. One micro-op writes both the register copy and the
  record's x/y words.
- SETUP4 is 73 ops / 110 cycles, against the model's 115; SETUP2 is 47 ops / 104 cycles.

**Frame load, p99:**

| Build | VF3 | Bass Fishing | Daytona 2 |
|---|---|---|---|
| Built (`SETUP_OUT = 1`) | 76.9% | 71.3% | 117.6% |
| R65's model | 78.7% | 73.2% | 120.7% |

Daytona 2 still needs D-10's reduced rate, or a set-up unit in the rasteriser.

**Fit, 10 ns, ALM without pins:**

| Build | ALM | M10K | DSP | Slow 100C | Slow −40C |
|---|---|---|---|---|---|
| `SETUP_OUT = 1` | 7,860–7,868 | 43 | 11 | 109.6–111.8 MHz | 104.9–110.6 MHz |
| Default | 7,505–7,526 | 32 | 11 | 108.0–116.9 MHz | 105.2–111.2 MHz |

The set-up costs +347 ALM and +11 M10K. That is 50 ALM over R60's estimate, and inside
7,600 + 300. It replaces the viewer's 1,780-ALM, 7-DSP temporary set-up.

**The −40C misses (R73) were three logic paths that the fitter placed worse after D-32.** None was
M10K-bound. Each is fixed exactly, bit- and cycle-identical:
- **The decoder's k255:** −(2b+1) = {~b, 1}, so an XOR instead of a subtract. The ambient byte
  gets its own register.
- **The pool's post-op clamp:** its flags are registered beside the raw result.
- **The facing unit's integer-to-float:** sign and magnitude are formed in the read cycle, −54 ALM.

**Planted faults:** 88 of 97 caught. The misses are R60's six, L8, and two new ones that need a
non-finite triangle or d exactly 0, which these frames do not contain.

**Wrong turns:**
- A cold-corner script named the corner wrongly; the database was gone before the paths were
  read, which cost one refit.
- A sort compared a y word straight off M10K (−0.37 ns at −40C); it is now registered first.
- A first post-op fix cost +120 ALM; it was narrowed to the paths that need it.

**R77 — M0-E done: the tile generator is exact on 389 frames, 553–560 ALM and 54 M10K. MAME and
Supermodel disagree on the stencil.**

**What I believed:**
- the tile generator would cost 1,100–2,000 ALM (§8.1);
- the palette would cost 32 M10K;
- MAME's `draw_layer` would be the reference (M0-E).

**What is now known:**
- **Area:** standalone, 10 ns, three seeds: 553–560 ALM without pins, 54 M10K, 0 DSP, at
  130.8–135.4 MHz (slow 100C) and 124.3–135.5 MHz (slow −40C).
- **The palette needs 52 M10K,** not 32. Its 524,288 bits cannot fit in 32 blocks.
- **One fetch engine serves all four layers** (299 ALM), on an `m2_sdram`-contract port, with two
  512×18 line buffers and 3 clocks a tile. It saves about 900 ALM against four engines.
- **Line timing:** at 14-clock memory latency the slowest line uses 64% of its 4,100-clock time. It
  holds without overruns up to about 22 clocks.
- **Nothing from Model 2 is copied.** Its tile chip is System 24's: a priority bit per tile,
  4 bpp only, a different row mask. Only the structure carried over: an engine fetching ahead of an
  emitter, line banks, and R906's skip of hidden columns.

**The two emulators differ at seven points** over the dumps:

| Point | MAME | Supermodel | Pixels |
|---|---|---|---|
| Stencil | map space, bit 0 leftmost, each layer masked separately | screen space, bit 15 leftmost, one layer of each pair per pixel | 1,792,811 in 29 frames |
| Line scroll | always applied | only when register bit 15 is set | 0 (unused) |
| Line-scroll word for line y | 2y | 2(y^1) | 0 |
| Tile pair straddling a stencil change | normal | uses the pair 16 px to the left | 1,692–3,445 |
| Pattern codes past 0xF8000 | wrap | read directly | 0 |
| 5→8-bit colour | pal5bit | ×255/31 | 51.6 M (±1 step) |
| Colour offset (F1180040/44) | unused | applied | 760,256 (Bass Fishing fades) |

Following D-19, the core defaults to Supermodel's behaviour, with MAME's as a compile-time option.
Supermodel's notes say the tile RAM fetches one layer of each pair per 16 pixels, which supports its
screen-space stencil. The colour offset is hardware behaviour, not a PC addition: Bass Fishing uses
it for fades, so it is built (about 40 ALM).

**Still open:** the coordinate space and the pair form. The bit order is settled by R84: bit 15 is leftmost.

**How it was established:**
- Dumps of tile RAM, palette and registers came from patched emulator copies:
  - MAME: 255 VF3 frames;
  - Supermodel: 56 VF3 and 78 Bass Fishing frames;
  - the patches are in `tools/mame-census/`.
- Literal re-implementations match each emulator's own output with 0 differing pixels.
- The RTL is exact against both builds on 714 and 765 frames. It stays exact behind hostile memory
  (acknowledges up to 800 clocks late) and recovers from overrun.
- 20 of 20 planted faults are caught. Line scroll and stencil variety needed synthetic register
  sets, because no dumped game uses them; Scud Race or Lost World dumps would test line scroll for
  real.

**Wrong turns:**
- Stock MAME Lua could not read the scroll registers, and its 64-bit write tap aborts on bit 63.
- `busy` fell before the emitter finished.
- A first FSM spent 7 clocks a tile (125 overruns).
- The palette took 60 M10K before the 2 K × 5-bit banking.

**R78 — D-30 built: BTB, two-word fetch and the 603e's branch look-ahead. 360 frames lost
becomes 15, for about 1,060 ALM; the two-word fetch is the overrun.** All three additions sit
behind parameters (`BTB`, `FETCH2`, `LA`). With all three off, R70's bench cycle totals come
back exactly.

**What was built:**
- **BTB design C**, 64 entries in 2 M10K, looked up a request ahead. Beyond R61, it is also
  looked up for fetch-side redirect targets, which takes VF3 from 13 to 4 frames lost in the
  model. Predictions are checked on arrival, and register targets are checked in EX and
  refetched from MEM. The y bit is honoured.
- **Two-word fetch:** even/odd data banks in the same 16 M10K, pairs only on the cache's fast
  path, and a second predecoder.
- **Direction:** the 603e's rule from the manual (§2.3.4.4). A bc, bclr or bcctr is resolved from
  the committed CR/CTR when nothing in flight writes them; otherwise the y bit decides
  (PEM 4.2.4.2).

**Frames lost** (CPI model `lv6_`, 18,900 frames per title at 100 MHz, against `ref66_dual_ic`):

| Step | Singles at 4 | Built fast path at 6 |
|---|---|---|
| Before D-30 | 360 | 373 |
| + BTB | 22 | 42 |
| + two-word | 17 | 37 |
| + direction | **15** (VF3 3 / Daytona 2 6 / Bass Fishing 6) | **34** |

At 110 MHz this becomes 0 / 3.

R61 predicted 3. Its model left out costs the built core pays:
- 64 entries under the y rule;
- a single-word first request after a redirect;
- refetching ops (rfi, sc, isync, mtmsr, MMU writes);
- the real line-crossing cost.

CPI as built is 1.757 / 2.100 / 2.031.

**Area, about +1,060 ALM and +2 M10K against D-30's 750:**

| Addition | Core | Cache | M10K |
|---|---|---|---|
| BTB with pre-read port | ±0 | +139 | +2 |
| Two-word fetch | about +730 | +153 | 0 |
| Direction | about +35 | 0 | 0 |

The two-word fetch is the overrun; R61 estimated about 350 for it. Full builds are now 19.2K ALM
cached and 17.2K uncached.

**Timing:**
- **Spike:** 105.8 / 108.5 / 108.0 MHz, against R70's 108.3 / 108.6 / 110.1. The set costs 1.5 MHz
  net, after R70's MLAB address patch.
- **Full builds at 100 MHz:** one uncached seed in three closes (+0.017 ns). The others fail only on
  the harness ROM's path through fetch translation, which the real core does not have. One cached
  seed in eleven closes (+0.196 ns), over four rounds, with seeds swinging ±0.5 ns.

**Planted faults:** BTB 16 of 24, two-word 17 of 19, direction 9 of 10, existing set 75 of 84 (as R70).

**On the board,** 2026-11-01 at 06:52, the round-1 and round-2 RTL:
- **no cache** (3aaaa6ac…): 1,394 of 1,394 reports identical, P00 000011DA;
- **cached** (a1fe9f93…): 1,392 of 1,392 identical, P00 00001E34.

P01 00000661 and P09 0CA977DF are unchanged. The board program is straight-line code, so the BTB
cannot show a gain there; P00's drop comes from the two-word fetch and the look-ahead.

**Wrong turns:**
- A failed check in the same cycle as a hard redirect retired the wrong PC.
- Grouping changed under the BTB keys: 4,150 failed checks before the key request's size was
  stored in the entry.
- Two-word pairs crossed a page.
- The soft allocation key was read a cycle late (+1.9% cycles).
- A CPI model option (`lv5_` cx) never applied; it was rerun as `lv6_`.

**Open:** the two-word fetch costs about 880 ALM for 2–5 frames lost, against a CPU over its D-27
budget. Whether to keep it is a budget decision.

**R79 — The viewer now runs on the geometry's own set-up and on the nearest mip level: exact, and
about 1,030–1,120 ALM smaller. Every project times one corner only.**

**The changes:**
- `r3d_geo` runs with `SETUP_OUT` and `LOD_WORD` on, and `MIP = 1` is on in the intake,
  interpolants and both texel instances.
- `m3v_setup` is deleted. The store now builds its own row word. With `LOD_WORD` the header is
  5 words, so y1 and y3 are words 6 and 10, and nw is 24 or 18.

**Checks:**
- **Both parameters together**, never benched before: the geometry is exact on 1,200 frames. The
  p99 loads are 77.0% / 71.4% / 117.7% (VF3, Bass Fishing, Daytona 2), about 0.1 point above R76.
- **All 5 frames** match `render_d25` (mip 1, lod 1) under 7 DDR3 profiles, including late
  acknowledges up to 400 cycles. 0 of 190,464 pixels differ on each.
- **Planted faults:** 27 of 28 caught. The miss is equivalent: the texel's mip form only ever reads
  u, v [20:0], so a 69-bit ring A is exact (−1 M10K, not yet built). Rule: read a block's bit needs
  from its masks, not from its port widths.

**Speed:** on frame 7860 the geometry phase drops from 65.3 to 18.7 ms. A frame takes 0.23–0.29 s,
still bound by the unbinned store's replay (D-20, block 8 in progress).

**Area:**
- **Set-up move:** −1,393 to −1,405 ALM, −7 DSP, +11 M10K, as expected.
- **Mip:** +390 to +419 ALM and +2 M10K.

**Fit, at 100 MHz:**

| Seed | ALM | M10K | DSP | Slow 100C | Slow −40C | Note |
|---|---|---|---|---|---|---|
| 1 | 25,245 | 196 | 72 | 101.6 MHz | 100.7 MHz | hold −0.012 ns at −40C |
| 2 | 25,209 | 196 | 72 | **99.9 MHz** | **98.3 MHz** | misses |
| 3 | 25,226 | 196 | 72 | 102.2 MHz | 104.7 MHz | closes at both corners; the board build (md5 2aba8ce6…) |

Seed 2's failing path is an unregistered ready chain: zband `a_y` → zspan's output enable. It needs
a skid stage.

**The finding:** `Model3.qsf`, the viewer and probe projects, and Model 2's qsf all set
`TIMEQUEST_MULTICORNER_ANALYSIS OFF`, as the MiSTer template does. The fitter then optimises and
times the slow 100C model only, and never repairs setup or hold at the −40C corner. That is how R68
missed −40C unseen, and how viewer seed 1 carries a −40C hold violation. Every fit now reports both
corners. Whether to turn multicorner analysis on for release builds is open: it means longer
compiles, and the fitter would then repair both corners.

**R80 — M0-F done: the 53C810 as Model 3 uses it is one opcode, single-stepped. No VF3 quirk; 260
ALM.**

**What I believed** (§5.2):
- VF3 writes DSP, which auto-starts SCRIPTS, and then single-steps, so the core needs a quirk.
- SCRIPTS needs block move, JUMP, CALL, RETURN and register reads and writes.

**What the captures show.** VF3, VF3 Team Battle and Bass Fishing (`bassdx`) were captured in
Supermodel and MAME: 615,501 steps, about 1.33 GB moved. All of them run one Sega driver:
- **DMODE.MAN (manual start) is set at boot,** so no DSP write ever starts SCRIPTS: 0 automatic
  starts in five captures, in both emulators.
- **Each memory move is run as one single step** (DCNTL = SSM|STD) from the interrupt routine, the
  manual's own single-step procedure (p. 5-46). The closing INT 0x98080000 is read by the CPU as
  data and never executed.
- **The only opcode ever executed is MMOV** (DCMD 0xC0). Every move is word-aligned with a length
  that is a multiple of 4.
- **The register window is the game's BAR1 write:** C0000000 for VF3, F9000000 for Bass Fishing.

**Built, `rtl/io/m3_scripts.sv`:**
- MMOV, honouring MAN, one instruction per step with DSP advanced past it;
- interrupts: SSI → DIP → IRQ, with a DSTAT read clearing them;
- INT behind a parameter;
- an illegal-instruction interrupt for everything else.

Its memory port is the CPU bus protocol with 8-word line bursts. The bursts are required: single
words take 20.3 ms of DMA in VF3's largest frame, against a 17.4 ms frame.

**Where MAME and Supermodel disagree on what the games read.** I followed the manual:

| What | MAME | Supermodel | Built (manual) |
|---|---|---|---|
| ISTAT after a step | 0x03, then SIP stuck | 0x01 | 0x01 |
| DSTAT read | 0x08, never cleared | 0x88, clears | 0x88, clears |
| Interrupt released by | ISTAT read | DSTAT read | DSTAT read |
| DCNTL.STD after a step | stays set | stays set | clears |

The interrupt goes the same way in effect in both: unmaskable by the board, on the 603e's external
interrupt.

**Checks:**
- A three-form C++ reference (manual, Supermodel, MAME) replays each emulator's captures with
  0 mismatches in 9.45 M register accesses.
- The RTL matches the manual's form on every capture chain, behind hostile memories (acknowledges up
  to 400 cycles late, BUSY up to 60%).
- 40 of 40 planted faults are caught.

**Fit:** 258–260 ALM, 0 M10K, 0 DSP, at 156–164 MHz (slow 100C) and 162–169 MHz (−40C). That
compares with D-27's 500–1,500 for I/O and SCRIPTS together.

**Area notes:** the FIFO in MLAB saved 174 ALM. A shared adder measured 16 ALM larger than three
incrementers, so it is not used.

**DMA rate:** 85.8 MB/s with line bursts. VF3's p99 frame moves in 1.97 ms. The hardware's PCI
memory-to-memory rate is estimated at about 40 MB/s, with 66 MB/s the ceiling.

**Not covered:** Lost World (C1000000) and Get Bass proper are not on disk, and Scud Race's auto-start
is untested (Step 1.5).

**Wrong turns:**
- I first planned a special case for the quirk.
- Supermodel's PCI log folds dwords, so its writes were mapped by value.
- Editing CRLF sources produced a 9,736-line patch at first; it is now 263 lines.
- Bench memory carried across capture files and gave 46 false failures.
- Two original faults hit logic that never mattered, and that logic was removed.

**R81 — Whole-core ALM projection after the night of 2026-10-31: about 41.4K with two-word fetch,
about 40.6K without.** Measured where a block exists, and estimated where marked:

| Block | ALM | Basis |
|---|---|---|
| Framework (release form, with `hps_io`) | 6,141 | R57, measured |
| CPU, cached, with BTB, look-ahead and two-word fetch | about 12,440 | R78's cached full build (19.2K) less R63's framework and harness share |
| — without two-word fetch | about 11,560 | less about 880 (R78) |
| Geometry, set-up on the pool and LOD word | about 7,700 | R79, in the viewer's fit |
| Rasteriser | about 7,900 | R79's chain at 6,680–6,710, plus the binned store and top level, estimated at 1,050–1,250 (blocks 8–9 in progress) |
| Sound: 68000, both SCSPs on one engine, glue | about 4,360 | fx68k 1,859 + R59's 2,148 + 357 |
| Everything else | about 2,900 | measured: tile generator 560 (R77), SCRIPTS 260 (R80), `m2_sdram` 331, `m3_ddr3p` about 625; estimated: bridge, I/O, NVRAM, RTC, compositor about 1,100 |
| **Total with two-word fetch** | **about 41,440** | over the 41,000 working ceiling |
| **Total without** | **about 40,560** | under it, with about 440 to spare |

Hence D-36.

**Against D-27's per-block budgets:**
- Geometry: about 850 under its 8,550.
- Rasteriser: at its 8,000.
- Everything else: about 600 under its 3,500.
- Sound: over its 3,000 by about 1,360. That was already known (R59), and FM stays (D-29).
- CPU: about 900 over its share with two-word fetch, and on budget without it.

**M10K projection:** about 380–400 of 553, with the palette at 52 (R77) and the geometry at 43.

**R82 — After a power cycle, DDR3 works with several reads in flight: 794 MB/s at 16, and both
viewers are bit-exact on the board.** Run on 2026-11-01 after I power-cycled the MiSTer.

**Peip's tester first, the R69 setting:** 158.4 MB/s, latency 12 min / 15 average / 105 max. The
port was back.

**The probe**, 32-byte random reads in an 8 MiB region, at 100 MHz, scaler live. Every case ran
with 0 pattern errors over 3–7 passes.

| Reads in flight | No background | Layer 22+22 MB/s | Layer + store, 143 MB/s |
|---|---|---|---|
| 1 (`m2_ddr3`) | 142.5 MB/s, median 17 | 131.7 | 103.7 |
| 1 (`m3_ddr3p`) | 149.7 MB/s, median 14 | 140.3 | 112.8 |
| 2 | 296.7 | 284.9 | 254.4 |
| 4 | 581.2 | 553.0 | 486.3 |
| 8 | 773.0, median 26 | 737.9 | 642.9 |
| 16 | 794.5, median 58 | 771.9 | 672.5 |
| 16, sequential | 798.1 | — | — |

- **Latency:** min 12–15 cycles in every case. The worst seen was 100–136 cycles.
- **What the probe confirmed:**
  - the bridge holds at least 16 reads (AC = 16);
  - throughput scales almost linearly to 4 in flight;
  - at 16 in flight it reaches 99% of the port's 800 MB/s peak.
- **Sharing costs little:** the frame buffer and polygon store take 7–17% of what the textures get.
- **D-33 is confirmed.** R75's VF3 p99 fill rate, about 99 MB/s (53,790 fills × 32 B × 57.5 Hz), is
  under a sixth of what 4 in flight delivers alongside the other traffic.
- **D-34:** there is no case on these numbers for moving the frame buffer or the store off DDR3.
- **R72 is confirmed:** the wedge was cleared by the power cycle, and with both gates (R71) no build
  wedged it again over eight core loads.

**Viewers on the board** (MGL load, screenshot):
- **Textured viewer, R74, seed 1** (382c2b86…): frame 7860 matches `render_d25` with textures on every
  pixel outside the overlay. This is the planks-over-water image.
- **Viewer v3, R79, seed 3** (2aba8ce6…): geometry set-up and nearest mip. It matches the bench's mip
  reference (`png_mip/f007860_rtl.png`) on every pixel outside the overlay. It differs from level 0 on
  51,299 pixels, as mip should.

**R83 — D-36 carried out: two-word fetch is off by default.** The core regression passed with
`FETCH2 = 0` and `BTB = LA = 1`, so the default changed in `ppc_core`, `ppc_icache` and
`ppc_cpu`.

**Regression:** the full R70/R78 set passes. Cycles with the real caches move very little:

| Run | Change against R78 |
|---|---|
| Subsystem | +0.08% |
| `--loops` | +0.07% |
| Core bench at wait 9 | +19% |

The core bench's +19% is not a real cost: it offers two words on 75% of cycles, which no real
cache does.

**Board harness:**
- uncached P00 000011DA, unchanged;
- cached P00 00001E4E against 00001E34 (+26 cycles of 7,732, +0.34%);
- P09 0CA977DF.

**Planted faults:**
- BTB and pre-read port: 16 of 24, the same eight misses as R78.
- Look-ahead: 8 of the 9 that still apply. `la_w1` is dead code with two-word fetch off.

**CPI model, singles at 4, 100 MHz:** 18 frames lost (VF3 3 / Daytona 2 8 / Bass Fishing 7),
against 15 with two-word fetch, so two-word fetch was worth 3 frames in 56,700.

**Spike, 8.333 ns:** 102.5 / 108.9 / 109.2 MHz at slow 100C and 102.2 / 107.8 / 109.3 MHz at
−40C, at 11,444–11,550 ALM. That is −321 to −427 ALM in the core alone, and the cache sheds about
153 more. Seed 1's worst path is EX→MEM, not fetch.

**CPI at the built fast path (L = 6):** 35 frames lost against 34. At 110 MHz it is 0 / 3, unchanged.

**Full release builds, 100 MHz, multicorner on (D-35).** CPU-clock setup slack, slow 100C / slow −40C:

| Form | Seed 1 | Seed 2 | Seed 3 | ALM |
|---|---|---|---|---|
| No cache | +0.008 / **−0.013** ns | −0.039 / −0.116 ns | −0.223 / −0.288 ns | 17,214–17,235 |
| Cached | −0.177 / −0.203 ns | −0.117 / −0.243 ns | −0.232 / −0.231 ns | 18,680–18,735 |

- The fast corners have more than 4 ns of margin, and hold is clean everywhere.
- **No build closes at every corner.** Uncached seed 1 would have counted as closing under R78's single-corner check: it misses by 13 ps at −40C.
- The worst paths move with the seed, and none is a fetch path: the harness ROM into the queue head, the DTLB MLAB read address, `op_y`, and the D-cache's `r_a`.

**Area correction:** two-word fetch was worth about 520 ALM cached (−446 to −610 against R78), not R81's 880, because the spike and cache deltas had been added. The uncached build saves nothing, since its harness ties the second word low. R81's total without two-word fetch is therefore about **40,920 ALM**, about 80 under the 41,000 ceiling, not 440.

**On the board** (2026-11-01, 11:06), both despite their small misses. My rule: small negative slack is fine for a board test, but a release must close every corner.
- **Uncached, seed 1** (29c7f0d0…): 1,395 of 1,395 reports identical, P00 000011DA.
- **Cached, seed 2** (060746af…): 1,392 of 1,392 identical, P00 00001E4E.

P09 is 0CA977DF in both.

**Open:** CPU closure at 100 MHz at every corner, for release. The misses are 0.01–0.29 ns.

**R84 — The stencil's bit order, from cabinet footage: bit 15 is the leftmost span (Supermodel).
The coordinate space and pair form are still open.**

**What I believed:** R77 left the bit order open. On VF3's portrait wipes the two emulators showed
mirror-image splits, and I took the wipe's "ghost" for a translucent layer.

**What is now known:**
- **One wipe, everywhere.** VF3's logo and portrait wipes are the same 16-step stencil wipe on pair
  B, alternating by line. In step k, even lines have `FFFF>>k` and odd lines `FFFF<<k`, while pair A
  stays `FFFF`. The ghost is every other line cleared. `f002040` and `f009360` are steps 5 and 8 of
  it, with the line parity swapped.
- **The cabinet's even lines open from the left.** The footage is YouTube kiv9COZRsys at 0:26–0:28,
  the SEGA logo wiping to the Great Wall. That is Supermodel's bit 15 leftmost; MAME's bit 0 puts the
  even lines on the right. The RTL default (`MASK_SCREEN = 1`) was already Supermodel's, so nothing
  changes.
- **The span grid matches both emulators:** spans start at x = 0 and are 32 px wide, span 15 is cut
  to 16 px, and the white closes at x = 256, not the screen centre.
- **One cabinet effect no form models:** on the lines that open from the right, the white run ends
  3–6 px before the span boundary, leaving a clear column at x ≈ 250–257 at step 8. That is about
  5 px × 192 lines a frame for 16 frames.

**How it was established:**
- Per-frame dumps of the wipe in both patched emulators: 512 frames, 0 px against each emulator's
  own output.
- The footage geometry was fitted from the stripe period (1.883 rows a line) and 62 logo edges
  (offset −1.13 rows, rms 0.19).
- A per-band correlation of each hypothesis's line pattern against the footage: Supermodel scores
  +0.44 to +0.76 in every band, and MAME the exact negative. The verdict holds for any grid offset
  from −1.95 to 0.0 rows.
- The RTL is exact on the 38 wipe frames in both builds, and behind the hostile memory.
- The comparison images (cabinet | MAME | Supermodel) are in `sm3-builds/tilegen/wipe_out/cmp/`.

**Still open:**
- Map against screen space, and the exclusive pair against a per-layer test. No dumped stencil crosses a
  visible layer with X scroll; Scud Race or Lost World would.
- The assumption that stencil word y applies to line y. Line-resolution Bass Fishing footage would
  close it.
- The early end of the white run.

**Wrong turns:**
- I read the ghost as a translucent layer.
- The 718 lit rows were first taken as 384 lines.
- A sign error in the stripe-phase fit first made the verdict look ambiguous.
- A two-parameter logo fit on one band of lines.

**R85 — Stage A boots VF3 on the board to the SEGA logo. The crash at the wipe is a CPU gap:
misaligned loads.**

**What runs on the board** (2026-11-01; system builds `sysa_s1` b155c886… and `sysb_s3` 8c848af9…, the
`vf3` MRA, `vf3.zip` md5-checked):
- The boot ROM runs, then the Japan-only warning screen, then the SEGA Presents tile screen with
  "Insert Coin(s)" and "Credit 0/2".
- **CPI on the UART:** 2.37 / 1.45 / 1.58 / 1.95 over the boot seconds, then 1.80 every second from
  6 s to 16 s, against the CPI model's 1.77 for VF3. Load-use is 36.5% of cycles, fetch waiting
  20.8%, data-memory wait 5.4%. With the Real3D stubbed this is the attract loop, not gameplay.
- **In simulation:** the warning screen's tile state equals Supermodel's byte for byte, and the video
  output matches the reference on all 190,464 pixels. The bus streams match Supermodel's through
  frame 320.
- **Area:** about 21.8–22.1K ALM in total; the system around the CPU is 3,393 ALM.
- **Timing:** seed 3 closes the CPU. Its only failing paths are in the debug CPI monitor's CRC.

**What I believed:** at 17 s the game stopped on the logo, and I first took it for a wait on the
Real3D stub.

**What is now known:** it is a data storage interrupt into the boot ROM's fatal handler. The overlay
shows vector 0x300, SRR0 0x00054B08 and MSR 0x1000.
- At 0x55134–0x55170, VF3 does `lwzx` with a base of FFFD4FD2, a word load at 2 mod 4 from a pointer
  table in fixed CROM.
- `ppc_core` forces misaligned accesses aligned, a gap left from R31. So it read 2FD2FFFD for
  FFFD2FD2.
- A later `lhau` on that pointer misses every BAT and PTE.

Under the 603e manual (§4.5.6), misaligned integer accesses within a page are done in hardware. Only
the listed cases take the alignment exception. The seed-3 build crashed the same way, so it is not
timing.

**Fix:** misaligned word and halfword loads and stores in `ppc_core`, by the manual. That is in
progress, and the crash is being reproduced in the system bench before any fit.

**R86 — CPU timing at every corner, then misaligned accesses (D-39): +350 ALM cached, and closure
lost again.**

**The timing pass.** Twenty restructures, each with a Verilator equivalence check, left aligned
code cycle-identical. They took the D-36 core from 0 of 6 closing builds to 2 of 3 uncached and 1 of 3
cached at 100 MHz with multicorner timing (round 3), at D-36's area (−20 uncached, +60 cached).

**Misaligned support** (R85), required behaviour with no parameter:
- **Split and merge:** crossing halfwords and words split in EX into two word accesses (+1 cycle
  each) and merge in WB. This covers D, X, update and byte-reversed forms, cached and uncached.
- **Alignment exception** (0x600, DSISR by form, DAR = EA, or EA+4 for lmw/stmw) for:
  - lmw, stmw, lwarx and stwcx. not word-aligned;
  - FP accesses not word-aligned;
  - page crossings per D-39.
- **Checks:**
  - all 34 regression runs pass, including 15 misaligned ones (1,012 crossings of a word and 404 of a
    page in 200 seeds, 139 of them with DR=1 inside a DBAT block);
  - 20 of 20 planted faults caught;
  - the board harness is unchanged.
- **The one change to aligned code:** FP doubles crossing a page now trap.

**Area:**

| Form | Against D-36 |
|---|---|
| Uncached `ppc_core` | +215 to +245 ALM |
| Cached `ppc_cpu` | **+350 ALM** |

On R83's arithmetic, the core is now about 250–300 ALM over 41,000.

**Timing after the misaligned support:**
- **100 MHz:** uncached 1 of 3, cached 0 of 3.
- **110 MHz:** the 9.09 ns spike runs 107.7 / 105.8 / 99.7 MHz at its worst corner.
- **The paths to 110,** with estimated fixes, are in `sim/cpu/cpu-notes.md`:
  - the interrupt → `ex_xcm` → split-hold chain (−0.94 ns);
  - EX translation into MEM control;
  - the FMA stage-1b shift count;
  - the FP wait into the queue pop;
  - the split merge;
  - I-cache data into predecode;
  - D-cache decisions into EX redirect;
  - IBAT into the I-cache pa.

  Together they are about +150–250 ALM.

**On the board** (2026-11-01, 15:23): the uncached round-6 build with misaligned support (b7368660…)
gave 1,395 of 1,395 reports identical. P00 000011DA, P01 00000661, P09 0CA977DF.

**Wrong turns:**
- One round launched from the repo root with a bad RTL path; it was killed before producing anything.
- Flat queue entries put the pop through a LUT.
- The split's byte offset from the ALU sum put a subtract into EX's hold (−1.03 ns).
- A comment beginning "// synthesis" was read by Quartus as a pragma.
- `pkill -x` killed a parallel quick check.

**R87 — Rasteriser blocks 8 and 9: the whole rasteriser is bit-exact. At level 0, texture fills
bind it through the cache's write gate.** `r3d_rast` takes the geometry's record stream (viewport
and SETUP4 records) and writes a finished layer to DDR3.

**Exact.** It matches `render_d25` (textured, fog, cull) on 400 VF3 + 400 Bass Fishing frames,
207.6 M pixels, 0 wrong. That holds on board-like ports and on hostile ones (acks up to 400 cycles,
BUSY 60%, latency to 200). The MIP = 1 build is exact on 4 frames.

**Block 8, the D-20 store.** It bins set-up records by 8-row band, as runs:
- consecutive records sit contiguous in DDR3 and are read in one burst each;
- alpha lists grow downward, so they come out newest first for free;
- records are packed, from 23 to 21 words.

At VF3 p99 that is 18,043 (record, band) pairs but only 5,710 burst reads.

| Store traffic, p99 | MB/s |
|---|---|
| VF3 | 132.0 |
| Bass Fishing | 124.3 |
| VF3, all DDR3 with a 16-bit layer and polygon RAM | 207.5 (D-20 estimated about 180) |
| VF3, all DDR3 with the 32-bit layer as built | 251.3 |

R82's board numbers carry this with room.

**Block 9** is the top level. Fog is per viewport through `r3d_rast_fogvp`. Scroll fog is a frame
input for now, because the R62 viewport record lacks the fields that its rule needs.

**Area, three seeds, without pins:** 7,719–7,774 ALM, 100 M10K, 45 DSP, against the 8,000 budget.
Blocks 8 + 9 are 1,848–1,874 ALM against R66's estimate of 1,050–1,250.

**Timing:** 100.0–102.7 MHz at 100C on all three seeds. At −40C it fails on all three (97.3–99.4 MHz),
in the front's edge product and the texel filter, which lose 6–8 MHz once assembled.

**Frame time at level 0:** VF3 p99 is **148.9%** of a 100 MHz frame. Even with 1-cycle memory the
worst frames take 139–147%. The cache writes a fill only after every earlier pixel has read, so a frame
costs about pixels + 4 × fills, against R67's overlapped model (90.7%). Two levers:
- the nearest mip level (R75 cuts fills at p99 from 302 K to 54 K);
- a write gate that waits only for pixels reading the victim line.

**Bugs found, both latent in the viewer's `m3v_raster`:**
- the 15-cycle idle window was shorter than the interpolants' latency;
- end-of-band marks overflowed the band FIFO across runs of empty bands.

**Planted faults:** 33 of 36 caught. One is equivalent, and two need frames the harness does not have.

**R88 — The rasteriser as it will run fits the frame. It closes at both slow corners, but the mip
build is about 270 ALM over budget.**

**The configuration that runs** (MIP = 1, the geometry's B, 32 KiB cache) is bit-exact against
`render_d25` on 400 VF3 + 400 Bass Fishing frames, on both port profiles, with 0 port-rule violations.

| p99 of a 100 MHz frame | VF3 | Bass Fishing |
|---|---|---|
| Board-like ports | **82.1%** | 63.3% |
| Hostile ports (the store's binning binds) | 128.8% | 102.8% |

- **Fills at p99** are 53,805 and 41,545, R75's prediction to within 0.06%.
- **DDR3 at p99, VF3:** texture 99.0 + store 132.0 + layer 43.8 = 265.4 MB/s. That is well inside R82's board figures.
- **What binds at mip** is the shading lane (one pixel a clock) and the Z band, not the cache.

**The cache's write gate by slot.** A fill now waits only for earlier pixels reading its victim's
slot, using two dual-port M10K of per-slot stamps.
- Level 0's VF3 p99 goes from 148.9% to 138.6%, not to R87's max(pixels, 4 × fills).
- The 8-fill window binds next. 16 fills would give the heaviest level-0 frame 121.7%, but under 3% at
  mip, so they were not taken.
- The gate first deadlocked the rasteriser. The band stage can hold the cache's output for thousands
  of cycles, so a 5-bit need counter wrapped; it is now 10 bits. The cache bench missed this because
  its stalls lasted one cycle, and it now has long holds (fault K15).

**−40C closure.** Nine path moves with no latency change, across seven blocks over four fit rounds:
the edge, texel, cache, Z band, store and fog unit, plus a 163-bit skid stage at the top level.
Each round showed the next layer of paths.

| Build | ALM | M10K | DSP | Slow 100C | Slow −40C |
|---|---|---|---|---|---|
| MIP = 0, three seeds | 7,832–7,850 | 101 | 45 | 104.8–107.4 MHz | 103.5–104.0 MHz |
| MIP = 1, three seeds | **8,266–8,279** | 102 | 45 | 102.0–105.2 MHz | 100.7–103.3 MHz |

Hold is clean. The MIP = 1 build is 266–279 ALM over the rasteriser's 8,000; mip costs +423 to +447 ALM,
about 300 of it in the two texel instances.

**M10K:** the span queue moved to an MLAB (−1 block). The bin table cannot drop to 3 blocks: its read
mux then misses timing.

**Wrong turns:**
- The gate's gain was first judged on the cache bench, whose memory hid both the gain and the hang.
- Moving the edge's whole compare chain to stage 0 only moved the failure.
- The −40C report had never been written: the corner is named `7_slow_1100mv_-40c`, not `n40c`.

**R89 — The stage B host blocks are built and exact against VF3's whole trace. They are over the
"everything else" budget.** These are the Real3D's command and status block (`r3d_cmd`), its memory
windows (`r3d_mem`), texture upload (`r3d_texup`), the compositor (`m3_compose`) and a seven-port DDR3
arbiter (`m3_ddr_arb`).

**Checks, against Supermodel dumps from a hooked copy whose bus trace is byte-identical to stage A's:**
- every status read over 1,500 frames at the real frame length;
- 43 draws of display-list RAM and polygon RAM;
- 42 texture sheets (8 MiB each), checked again at replay end;
- 98 culling RAM images;
- 2,000 synthetic textures of every type and depth;
- the compositor: 0 pixels differ from Supermodel's layer order (tiles below, 3D, tiles above), on five cases including DDR3 at 50% BUSY.

All identical. VF3 uploads one texture format only (type 0, 16-bit, 256×256 with mips), so the synthetic set covers the rest. Planted faults: 27 of 32 caught; two are real misses.

**What I believed:** the stage A plan doubled 8E and polygon RAM, with the two copies swapped at the flip.

**What is now known:** a plain swap breaks VF3. Polygon models are written once, so its memory differs
from Supermodel's at 1,236 of 1,500 flips. So `r3d_mem` writes both copies at once. Where the geometry
is reading the read copy, the write is deferred as an address run (at most 37 polygon and 6 8E runs a
frame) and copied over after the geometry. Each draw then sees memory exactly as at its flip.

**Texture words** are staged in a DDR3 ring and decoded between frames up to a mark set at each draw,
because frame N−1 is still drawing when frame N's words arrive.

**Area, one seed, estimated in the system:**

| Block | ALM | M10K |
|---|---|---|
| `r3d_cmd` | about 160 | 0 |
| `r3d_mem` | about 385 | 3 |
| `r3d_texup` | about 670 | 2 |
| `m3_compose` | about 175 | 5 |
| `m3_ddr_arb` | about 460 | 0 |
| **Total** | **about 1,850** | **10** |

That is against about 1,100 left of the "everything else" budget, about 750 over, to be paid back for
release (D-40). Every block closes 100 MHz at both slow corners.

**An RTL bug found by a planted fault:** `r3d_mem` deadlocked the bus when its run FIFO filled. VF3
never fills it.

**Open:**
- Scroll fog needs viewport node words that the geometry's viewport record lacks, so it is tied off
  for now.
- The `m2_sdram` write burst: 8E writes cost about 40 SDRAM clocks each, up to 20% of the port at VF3's
  peak.
- Whether `tot_clks` in the status packet matters to any game.

**R90 — The D-cache snoops 53C810 DMA (D-41): +108 ALM in the cache, about 0.4% of a frame.**
Desk-only so far: not board-tested.

**What I believed:** a snoop needs its own tag port, or must stall the CPU.

**What is built** (`ppc_dcache` `SNOOP`, brought out through `ppc_cpu` as `sn_req`, `sn_addr[31:5]`,
`sn_inv` and `sn_ack`):
- The snoop waits until the cache has nothing in flight, as the 603e retries a snoop during a fill,
  cast-out or tag write (§3.6.3, §3.6.8).
- It then holds the read port for two cycles, as the cast-out copy does, and acts through the existing
  dcbst/dcbf paths.
- **A DMA read** pushes a modified line, which stays valid and clean. **A DMA write** pushes it, then
  invalidates it.
- `sn_ack` comes after the push's last beat.
- A snooped write to the reservation's line clears the reservation (§2.3.6.1).
- With HID0[DCE] = 0, nothing is snooped (§3.2.3.2).

**Cost:** 0.68–1.34 CPU cycles per snoop, about 7,100 cycles in VF3's p99 frame (about 5,280 lines).

**Checks:**
- With `sn_req` low, the R70/R78/R86 regression is identical line for line, harness signatures included.
- D-cache bench: 96 of 96 runs at latencies 1–80 under hostile memory.
- Subsystem bench: 260 seeds, each DMA read checked against the stores made.
- Planted faults: 14 of 14 caught in the cache bench, 7 of 9 in the subsystem bench.

**Fit, seed 1:**
- `ppc_dcache` 842 → 950 ALM, `ppc_cpu` +65 ALM, M10K unchanged.
- CPU spike: 108.3 / 108.7 MHz at 9.09 ns.
- Cached harness: −0.61 / −0.53 ns with the snoop driven, against −0.30 / −0.63 ns without it. The failing
  paths are the known seed-sensitive ones, not the snoop's.

**Open:** which snoop type the MPC105 uses for a PCI read. The 603e's table 3-6 invalidates on a
non-inhibited global read; the build keeps the line valid. Both are coherent; only the later hit rate
differs.

**To finish D-41 in the system:**
1. `m3_bus` snoops every 53C810 access to work RAM, one per line, before the data moves. ROM needs none.
2. It keeps granting the CPU port while the snoop waits.
3. It orders a DMA write before the CPU's next transaction.
4. Then the PVR moves to 0x00060103.

The I-cache is not snooped (603e §3.1): code written by DMA needs icbi, as on the chip.

**R91 — VF3's crash at about 21 s is a CPU bug: an indexed load with update, when rD = rB.**
`ppc_decode` cracks lbzux/lhzux/lhaux/lwzux into two uops: step 0 loads rD, then step 1 computes
rA = rA + rB by reading rB again. When rD = rB, step 0 has already overwritten rB, so rA gets old rA plus
the loaded data instead of the EA. The manual allows rD = rB; only rA = 0 and rA = rD are invalid forms.
The FP update loads are safe for a different reason than I first wrote: they re-read rA + d / rB too, but their earlier steps write an FPR, so nothing is overwritten. Only stfdu's second word uses the held address.

**How it shows in VF3:** at 0x4D040, `lbzux r4,r6,r4` reads a copy routine's block number from a table.
The wrong r6 then sends the count read (`lbz r23,1(r6)`) to low RAM, which holds 0 instead of 3. So the
outer copy loop never ends. It copies 128 KiB block after block, one every two frames, until its source
pointer wraps past FFFFFFFF to 001E0D7C. The lwzx at 0x4D09C then misses at EA 0x00200000, which the game's
own TLB attribute table leaves unmapped, and the handler takes its DSI to the boot ROM's fatal loop.

**How it was established:**
- The board overlay (R85's latch, made correct in build 6) showed the fatal DSI at SRR0 0x4D09C, after a
  decrementer entry and a DTLB miss.
- The system bench reproduced it exactly at frame 1,158, with r21 = 001E0D7C and r23 = −99.
- The first wrong value is at frame 960.

**Wrong turns:**
- Because a decrementer interrupt was the last entry before the fatal, I first suspected interrupt state:
  rfi, the TGPR shadows, an interrupt inside a misaligned split. It only happened to land inside the
  runaway loop.
- A first overlay latch keyed on PC ranges caught ordinary code at 0x1700. A second, keyed on SRR changes,
  never fired, because the exception's own retirement record already carries the new SRRs.
- The earlier static checks of the page table and BATs were right, but beside the point.

**Why the regression missed it:** the core bench's generator evidently never made rD = rB for update
loads. Every copy before 21 s was already the wrong length, silently.

**Fix, built:** `ppc_core` `id_xuea` makes the second step of an integer update load take EX's EA through the existing forward path, with no new registers. `ppc_decode` only changes step 1's fields and comments.

**Checks:**
- The reproducer (`lbzux r4,r6,r4` and every other integer update load), cached and uncached, with a DTLB miss on step 0, passes in all four modes; the old RTL fails it at once.
- A new `--alias` generator ran 1,500 seeds (core and subsystem), all passing; the old RTL fails them in the first seed.
- The R70/R78/R86/R90 regression is identical line for line, and the harness signatures are unchanged.
- 5 of 5 planted faults are caught.
- The same logic exists as a patch for build 5's CPU (`sm3-builds/fix_lux/build5.patch`), for the stage A board build.

**Timing, one seed:** the CPU spike at 9.09 ns ran 99.1 / 99.7 MHz, against 108.3 / 108.7 before. The worst path (EX translation into `m_rpq`) does not touch the change, and the fix's first form gave 107.8 / 108.1 on the same seed. So it is probably placement, but a three-seed batch must confirm that before any release fit.

**Open, from the same review:**
- `lmw` with rA inside the loaded range re-reads rA on each step. The manual calls the form invalid, and Supermodel and MAME keep the original base. My rule is to follow the emulators where the manual leaves behaviour undefined, so `lmw` should hold its base. That needs a held register on the `imm4_q` path.
- R86's Verilator-only merge check misfires on cache ops (dcbst/dcbf/dcbt) at an EA that is not word-aligned. It should exclude them.

**R92 — Stage A's goal is met: VF3 runs its attract loop on the MiSTer.** This is build 7 (9ae84cd8…): build
6's system with build 5's CPU plus the R91 update-load fix, PVR 0x00060101, at 100 MHz. Setup is −0.29 /
−0.33 ns at the slow corners, and hold is positive everywhere. It was loaded on 2026-11-01 at about 20:15, on
my call, before the bench confirmed.

VF3 passes the SEGA logo and runs its attract loop with no fatal entry for at least 55 s. On screen are the
blue-sky backdrops with clouds, the character portraits (Kage, Lau), "Insert Coin(s)", "Credit 0/2" and the
SEGA copyright: every tile layer the attract draws. There is no 3D yet, since the Real3D is a stub.

The exception counter keeps rising through interrupts and TLB misses, and the fatal words stay zero.

**CPI on the UART:** 1.80 to 16 s, then 2.5–4.1 through the attract. The attract does more copying and
decompression, and waits on the Real3D stub.

**R93 — The first integrated build (stage A + B) misses 100 MHz by 2 ns and does not boot on the board.**
`sysb_s1` (89f65a14…), release form, seed 1:
- **Timing:** setup −2.02 / −1.79 ns at the slow corners (about 83 / 85 MHz); hold positive.
- **Area:** 41,138 of 41,910 ALM (98%), 300 M10K, 86 DSP.
- **Worst path:** the scan-out request through the DDR3 arbiter's combinational grant into `m3_ddr3p` and the
  f2sdram registers. At 98% full, every block slips about 1 ns: the CPU D-cache, the geometry, the texture
  cache and texture upload.

**Bench:** every Real3D DMA word is identical to Supermodel, and at frame 1,121 the layer drawn in the system
equals `render_d25` on all 190,464 pixels.

**On the board,** loaded anyway to see: dead from the first second (CPI 21.5), no screen.

**Next:** the arbiter's grant gets a register, texture upload's compare is pipelined, and a 90 MHz build follows.

**Area is now the binding problem.** This is without sound or real I/O. The release needs several thousand ALM
freed (R81: about 2,500–2,750 against 41,000, with sound and I/O).

**R94 — The first 3D on the board: VF3's attract fights draw on the MiSTer.** The integrated build of
stage A + B (`sysb90_s1`, 5f02194c…) runs at 90 MHz:
- setup +0.05 / −0.41 ns at the slow corners, hold positive;
- 41,346 ALM (99%), 300 M10K, 86 DSP.

The 90 MHz build follows two timing fixes after the 100 MHz build missed by 2 ns (R93): a registered owner in
the DDR3 arbiter's grant, and registered end and jump tests in texture upload.

On the board, 2026-11-01: boot, the warning screen and the SEGA logo, then **the attract fights in 3D,
textured, over the tile layers.** Aoi, Kage, Pai, Jacky, Lau and Lion fight on the beach stage, with the sky and
the character portraits, for at least 60 s with no fatal entry.

**Glitches seen:**
- a few frames show a fighter's suit flat-coloured, or a full-screen blur;
- particle sprays appear.

The fights are not yet compared with Supermodel in the bench: the fixed-CPU chain had reached frame 900. So
these glitches are untriaged.

**CPI through the fights:** 3.0–5.7. The game now waits on the real Real3D, and the CPU clock is 90 MHz.

**Area is now the binding problem:** 99% full without sound or real I/O.

| Largest blocks | ALM |
|---|---|
| `ppc_cpu` | 12,515 |
| `r3d_rast` | 8,142 |
| `r3d_geo` | 7,762 |
| `ascal` | 2,112 |
| the viewer's two read adapters, carried into the system | 963 |
| `m3_cpu_mon` (debug) | 359 |
