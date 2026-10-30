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
  clip, summed triangle area (overdraw), texture uploads.

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

Outputs: `<prefix>.txt` and `<prefix>_frames.csv` (CPU), `<prefix>_real3d.txt`
and `<prefix>_real3d.csv` (Real3D). The census callback costs about 20% of
MAME's speed; a 360 s run takes about 7 minutes.

The `M3IDLE` ranges are VF3's vblank and decrementer wait loops, found from
the hottest-PC table. Other games need their own.
