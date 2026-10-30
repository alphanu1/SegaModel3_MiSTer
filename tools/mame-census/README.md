# MAME census for gates M0-A1 and M0-B

A patch to MAME 0.289 (`f34f0250`) that counts what a Model 3 game does, so
the design study works from measurements instead of guesses.

- **CPU (M0-A1):** every executed PowerPC instruction by opcode class, SPR
  traffic, MSR translation state, PC region (RAM / fixed CROM / banked CROM),
  code footprint, distinct 32-byte I-cache lines per frame, single vs double
  FP, and whether each double result is exactly representable as single.
  Optional `M3IDLE` PC ranges count busy-wait loops per frame.
- **Real3D (M0-B):** highest word and distinct words written to display-list,
  culling and polygon RAM, and per frame: models, polygons (and how many come
  from VROM), new and reused vertices, polygon fetch bytes, triangles after
  clip, summed triangle area (overdraw), texture uploads; texture-sheet tiles
  written, every texel the renderer reads, distinct 4x4 blocks per frame, and a
  4-way texel cache with 32-byte lines simulated at 8, 16, 32, 64 and 128 KiB.

## Build

```
git clone --local /path/to/mame mame289-m3 && cd mame289-m3
git checkout f34f0250
git apply ../../tools/mame-census/mame0289-model3-census.patch
make SUBTARGET=m3 SOURCES=src/mame/sega/model3.cpp TOOLS=0 NOWERROR=1 -j20
```

Build under `sm3-builds/`, never in this tree.

## Run

```
M3CENSUS=$PWD/vf3 \
M3IDLE=1E928-1E930,1E97C-1E984,1E9DC-1E9E4,1E9FC-1EA04,2518-251C \
../mame289-m3/m3 vf3 -rompath /home/ben/roms/Model3 \
    -video none -sound none -nothrottle -seconds_to_run 360 -skip_gameinfo \
    -cfg_directory cfg -nvram_directory nvram -autoboot_script play.lua
```

Run from a scratch folder under `sm3-builds/`: MAME writes `cfg/` and
`nvram/` there, and `nvram/` is ROM-derived. `play.lua` coins up at frame
5200 (`M3_COIN_FRAME`), starts and mashes player 1 so the run covers gameplay.

Add `-numprocessors 1` when the texel cache matters: MAME's renderer is
multithreaded, and the cache model needs texels in drawing order. It runs at
about 30% speed that way.

Outputs: `<prefix>.txt` and `<prefix>_frames.csv` (CPU), `<prefix>_real3d.txt`
and `<prefix>_real3d.csv` (Real3D). The census callback costs about 20% of
MAME's speed; a 360 s run takes about 7 minutes.

The `M3IDLE` ranges are VF3's vblank and decrementer wait loops, found from
the hottest-PC table. Other games need their own.

## Supermodel replay (games MAME cannot run)

MAME stalls on Bass Fishing (I/O board error) and Daytona 2. Supermodel runs them,
but draws through OpenGL, so it cannot count texels. Instead Supermodel saves the
Real3D memories every N frames and MAME's renderer replays them with the census on.

Supermodel at `036af1c` (github.com/trzy/supermodel), with
`supermodel-real3d-dump.patch`; needs SDL2_net. Build with
`make -f Makefiles/Makefile.UNIX`.

```
env -u WAYLAND_DISPLAY SDL_VIDEODRIVER=x11 \
M3DUMP_DIR=$PWD/dumps_bassdx M3DUMP_EVERY=30 M3DUMP_START=1800 M3DUMP_EXIT=13800 \
xvfb-run -a -s "-screen 0 640x480x24 +extension GLX" \
    ./bin/supermodel /home/ben/roms/Model3/bassdx.zip \
    -no-throttle -no-vsync -no-sound -no-dsb -no-gpu-thread
```

Each dump is the 8C (4 MiB), 8E (1 MiB), polygon (4 MiB) and texture (8 MiB) RAM,
gzip level 1, 0.5–5 MB. Then replay one dump per MAME frame on the same set, which
supplies VROM and the stepping:

```
M3REPLAY=$PWD/dumps_bassdx M3CENSUS=$PWD/replay_bassdx \
../mame289-m3/m3 bassdx -rompath /home/ben/roms/Model3 -video none -sound none \
    -nothrottle -numprocessors 1 -seconds_to_run 120 -skip_gameinfo
```

During a replay the game's own frames are not drawn. Unknown texture types and
traversal addresses MAME would treat as fatal are counted and skipped instead.
Without input injection, Supermodel runs cover attract mode only. The RAM-footprint
lines in a replay's summary describe MAME's own game, not the dumps.

### Supermodel CPU census

The same patch counts every instruction Supermodel's PowerPC interpreter executes.
`M3CPU=<prefix>` writes `<prefix>_cpu_frames.csv` (instructions, idle, ROM-executed
per frame) and `<prefix>_cpu.txt` (hottest RAM PCs with their opcodes). `M3IDLE`
takes the same busy-wait ranges as the MAME census. Run with `-no-idle-skip`, or
Supermodel skips the wait loops and they are never counted. Supermodel renders every
frame, so its per-frame counts are the 60 Hz figures; MAME's run at VF3's 30 Hz.

Wait loops found so far: VF3 `1E928-1E930,1E97C-1E984,1E9DC-1E9E4,1E9FC-1EA04,2518-251C`;
Daytona 2 `4C754-4C760,4C73C-4C744,4C6CC-4C6D4,541C-5428`.

`M3SHOT_DIR` (with `M3SHOT_EVERY`) saves numbered framebuffer grabs. They are taken
before the 3D is drawn, so they show the 2D layers only. Supermodel's BMP header has a
wrong file size; read them with `magick -define bmp:ignore-filesize=true`.
