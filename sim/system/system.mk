# Sega Model 3 core for MiSTer FPGA -- the system (study D-37 stage A) benches.
#
#   make -f sim/system/system.mk test_jtag test_io test_sdr_port test_bus
#   make -f sim/system/system.mk system      # the whole system bench (needs the ROMs)
#
# Kept out of the main Makefile so the two can be edited independently.
# Traces and ROM-derived files live in sm3-builds/sys (git-ignored).

VERILATOR ?= verilator
SYS_OBJ   ?= sm3-builds/sys/obj
TRACE     ?= sm3-builds/sys/sm_bus.bin
VFLAGS    = --cc --exe --build -j 6 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC
CFLAGS_SYS = -O2 -std=c++17 -I$(PWD)/sim/system

CPU_RTL = rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_fpu_pkg.sv rtl/cpu/ppc_decode.sv rtl/cpu/ppc_alu.sv \
          rtl/cpu/ppc_muldiv.sv rtl/cpu/ppc_regfile.sv rtl/cpu/ppc_fpregfile.sv rtl/cpu/ppc_fetchq.sv \
          rtl/cpu/ppc_fpu_lzc.sv rtl/cpu/ppc_fpu_fma.sv rtl/cpu/ppc_fpu_div.sv rtl/cpu/ppc_fpu_misc.sv \
          rtl/cpu/ppc_fpu_ldst.sv rtl/cpu/ppc_fpu_fpscr.sv rtl/cpu/ppc_fpu.sv rtl/cpu/ppc_fpu_sfma.sv \
          rtl/cpu/ppc_fpu_rsqe.sv rtl/cpu/ppc_fpu_fast.sv rtl/cpu/ppc_pdec.sv rtl/cpu/ppc_pdw.sv rtl/cpu/ppc_btb.sv rtl/cpu/ppc_core.sv \
          rtl/cpu/ppc_cache_pkg.sv rtl/cpu/ppc_cache_ram.sv rtl/cpu/ppc_icache.sv rtl/cpu/ppc_dcache.sv \
          rtl/cpu/ppc_cpu.sv
# the bench's CPU: the FPU's leading-zero counter swapped for a fast,
# equivalent model (sim/system/ppc_fpu_lzc_sim.sv, tb_lzc_equiv.cpp)
CPU_SIM = $(filter-out rtl/cpu/ppc_fpu_lzc.sv,$(CPU_RTL)) sim/system/ppc_fpu_lzc_sim.sv
TG_RTL  = rtl/real3d/r3d_ram.sv rtl/video/m3_tilegen_pal.sv rtl/video/m3_tilegen_fetch.sv rtl/video/m3_tilegen.sv
# stage B: the Real3D (the geometry, the rasteriser, the host blocks, DDR3, the compositor)
R3D_SYS = rtl/real3d/r3d_ucode_pkg.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_div.sv \
          rtl/real3d/r3d_fsqrt.sv rtl/real3d/r3d_recip.sv rtl/real3d/r3d_face.sv rtl/real3d/r3d_mstream.sv \
          rtl/real3d/r3d_setup_cr.sv rtl/real3d/r3d_ucode_su.sv rtl/real3d/r3d_ucode.sv rtl/real3d/r3d_vpu.sv \
          rtl/real3d/r3d_walker.sv rtl/real3d/r3d_geo.sv \
          rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv rtl/real3d/r3d_rast_edge.sv rtl/real3d/r3d_rast_front.sv \
          rtl/real3d/r3d_rast_intake.sv rtl/real3d/r3d_rast_zspan.sv rtl/real3d/r3d_rast_zband.sv rtl/real3d/r3d_rast_recip.sv \
          rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_fogp.sv rtl/real3d/r3d_rast_texel.sv rtl/real3d/r3d_rast_tcache.sv \
          rtl/real3d/r3d_rast_colour.sv rtl/real3d/r3d_rast_band.sv rtl/real3d/r3d_rast_fogvp.sv \
          rtl/real3d/r3d_rast_store.sv rtl/real3d/r3d_rast_zq.sv rtl/real3d/r3d_rast.sv \
          rtl/real3d/r3d_cmd.sv rtl/real3d/r3d_mem.sv $(TEXUP_RTL) rtl/real3d/r3d_vrom_rd.sv \
          rtl/mem/m3_ddr_arb.sv rtl/mem/m3_ddr3p.sv rtl/video/m2_fb_read.sv rtl/video/m3_compose.sv \
          rtl/viewer/m3v_sdr_rd.sv rtl/viewer/m3v_ddr_rd.sv
# R100: BUS_RTL lets a bench build take a patched m3_bus (Daytona 2's Step 2 map,
# sm3-builds/dt2/m3_bus_step2.patch) with SYS_DEFS = +define+M3_STEP2_BUS
BUS_RTL  ?= rtl/sys/m3_bus.sv
TEXUP_RTL ?= rtl/real3d/r3d_texup.sv    # R100: with M3_STEP2_TEX, sm3-builds/dt2/handoff2/rtl/r3d_texup.sv
SYS_DEFS ?= +define+M3_STEP2_BUS=1 +define+M3_STEP2_TEX=1 +define+M3_DMAFAST=1 +define+M3_R3B=1   # (R144: as Model3.qsf)
SYS_RTL = $(R3D_SYS) rtl/dbg/m3_uart_tx.sv rtl/dbg/m3_cpu_mon.sv rtl/sys/m3_jtag.sv rtl/io/m3_93c46.sv rtl/io/m3_io.sv rtl/io/m3_cab_inputs.sv rtl/io/m3_inputs.sv rtl/sys/m3_sdr_port.sv $(BUS_RTL) \
          rtl/io/m3_scripts.sv rtl/mem/m2_sdram.sv rtl/video/m2_video_timing.sv rtl/viewer/m3v_overlay.sv \
          rtl/dbg/m3_frame_busy.sv rtl/dbg/m3_fps.sv rtl/sys/m3_xmem.sv rtl/sys/m3_5881.sv rtl/sys/m3_sec.sv rtl/sys/m3_loader.sv \
          rtl/sound/m3_scsp_dsp.sv rtl/sound/m3_sndbridge.sv rtl/sys/m3_system.sv

.PHONY: test_jtag test_jtag_fault lint_sys test_inputs test_io test_sdr_port test_bus test_bus_tg system test_xmem test_sec test_texup2

# r3d_texup's three-word texture port (R100; Step 1.5 and later, D-46) against its FIFO
# path, on 64 MiB of VROM (Daytona 2) and on 32 MiB (--v32: a Step 1.5 set). TEXUP2_RTL:
# the r3d_texup carrying STEP2_PORT (in the tree since the Step 2 merge)
TEXUP2_RTL ?= rtl/real3d/r3d_texup.sv
$(SYS_OBJ)/texup2/Vr3d_texup: $(TEXUP2_RTL) sim/system/tb_r3d_texup2.cpp
	@$(VERILATOR) $(VFLAGS) --public-flat-rw -GSTEP2_PORT=1 --top-module r3d_texup -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/texup2 $(TEXUP2_RTL) sim/system/tb_r3d_texup2.cpp >/dev/null

test_texup2: $(SYS_OBJ)/texup2/Vr3d_texup
	@echo "== r3d_texup: the three-word texture port against the FIFO"
	./$(SYS_OBJ)/texup2/Vr3d_texup 300 1
	./$(SYS_OBJ)/texup2/Vr3d_texup 300 2
	./$(SYS_OBJ)/texup2/Vr3d_texup 300 3 --v32
	./$(SYS_OBJ)/texup2/Vr3d_texup 300 4 --v32
	! TEXUP_FAULT_V64=1 ./$(SYS_OBJ)/texup2/Vr3d_texup 20 5 --v32 > /dev/null

# m3_sec (the 315-5881, study R100) against Daytona 2's Supermodel trace
SEC_TRACE ?= sm3-builds/dt2/sec_dt2.bin
$(SYS_OBJ)/sec/Vm3_sec: rtl/sys/m3_sec.sv rtl/sys/m3_5881.sv sim/system/tb_m3_sec.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_sec -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/sec rtl/sys/m3_sec.sv rtl/sys/m3_5881.sv sim/system/tb_m3_sec.cpp >/dev/null

test_sec: $(SYS_OBJ)/sec/Vm3_sec
	@echo "== m3_sec: the security board against Daytona 2's trace"
	./$(SYS_OBJ)/sec/Vm3_sec $(SEC_TRACE) 29250e16
	! ./$(SYS_OBJ)/sec/Vm3_sec $(SEC_TRACE) 29250e16 --fault > /dev/null

# m3_xmem (study R100): VROM in the extra memory, against a reference. R100's bench (one request at a time)
# runs on the MRA layout (XLIN 0), both forms; R127's (tb_m3_xmem2: requests overlapped as r3d_mstream asks
# them, 64 MiB texture reads, bursts) on all four
$(SYS_OBJ)/xmem/Vm3_xmem: rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_xmem -GXLIN=0 -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/xmem rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem.cpp >/dev/null
$(SYS_OBJ)/xmem_one/Vm3_xmem: rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_xmem -GXLIN=0 -GPIPE=0 -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/xmem_one rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem.cpp >/dev/null
XMEM2 = p1l1:-GPIPE=1:-GXLIN=1 p1l0:-GPIPE=1:-GXLIN=0 p0l1:-GPIPE=0:-GXLIN=1 p0l0:-GPIPE=0:-GXLIN=0
$(SYS_OBJ)/xmem2_%/Vm3_xmem: rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem2.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_xmem $(subst :, ,$(patsubst $*:%,%,$(filter $*:%,$(XMEM2)))) -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/xmem2_$* rtl/sys/m3_xmem.sv sim/system/tb_m3_xmem2.cpp >/dev/null

test_xmem: $(SYS_OBJ)/xmem/Vm3_xmem $(SYS_OBJ)/xmem_one/Vm3_xmem $(SYS_OBJ)/xmem2_p1l1/Vm3_xmem $(SYS_OBJ)/xmem2_p1l0/Vm3_xmem $(SYS_OBJ)/xmem2_p0l1/Vm3_xmem $(SYS_OBJ)/xmem2_p0l0/Vm3_xmem
	@echo "== m3_xmem: geometry and texture VROM reads on a pipelined port"
	./$(SYS_OBJ)/xmem/Vm3_xmem 200000 1
	./$(SYS_OBJ)/xmem/Vm3_xmem 200000 2
	./$(SYS_OBJ)/xmem_one/Vm3_xmem 200000 1
	./$(SYS_OBJ)/xmem2_p1l1/Vm3_xmem 1 300000 --xlin
	./$(SYS_OBJ)/xmem2_p1l1/Vm3_xmem 2 300000 --xlin
	./$(SYS_OBJ)/xmem2_p1l1/Vm3_xmem 3 300000 --xlin --gout 0 --seq 30 --tex 20
	./$(SYS_OBJ)/xmem2_p1l1/Vm3_xmem 4 300000 --xlin --gout 0 --lat 40 136 --ack 40
	./$(SYS_OBJ)/xmem2_p1l0/Vm3_xmem 1 300000
	./$(SYS_OBJ)/xmem2_p1l0/Vm3_xmem 2 300000 --gout 0 --lat 1 2 --ack 100
	./$(SYS_OBJ)/xmem2_p0l1/Vm3_xmem 3 300000 --xlin --gout 1
	./$(SYS_OBJ)/xmem2_p0l0/Vm3_xmem 4 300000 --gout 1
	./$(SYS_OBJ)/xmem2_p0l0/Vm3_xmem 5 300000 --gout 1

$(SYS_OBJ)/jtag/Vm3_jtag: rtl/sys/m3_jtag.sv sim/system/tb_m3_jtag.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_jtag -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/jtag rtl/sys/m3_jtag.sv sim/system/tb_m3_jtag.cpp >/dev/null

test_jtag: $(SYS_OBJ)/jtag/Vm3_jtag
	@echo "== m3_jtag against the Supermodel trace"
	./$(SYS_OBJ)/jtag/Vm3_jtag $(TRACE)

test_jtag_fault: $(SYS_OBJ)/jtag/Vm3_jtag
	@echo "== m3_jtag, one TDO expectation flipped (must FAIL)"
	! ./$(SYS_OBJ)/jtag/Vm3_jtag $(TRACE) --fault

# m3_inputs and m3_cab_inputs (the cabinet's controls by input profile) against a reference written from
# MAME's and Supermodel's ports
$(SYS_OBJ)/inputs/Vm3_inputs: rtl/io/m3_cab_inputs.sv rtl/io/m3_inputs.sv sim/system/tb_m3_inputs.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_inputs -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/inputs rtl/io/m3_cab_inputs.sv rtl/io/m3_inputs.sv sim/system/tb_m3_inputs.cpp >/dev/null

test_inputs: $(SYS_OBJ)/inputs/Vm3_inputs
	@echo "== m3_inputs: the controls by profile against MAME's and Supermodel's ports"
	./$(SYS_OBJ)/inputs/Vm3_inputs 1
	./$(SYS_OBJ)/inputs/Vm3_inputs 2

$(SYS_OBJ)/io/Vm3_io: rtl/io/m3_93c46.sv rtl/io/m3_io.sv sim/system/tb_m3_io.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_io -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/io rtl/io/m3_93c46.sv rtl/io/m3_io.sv sim/system/tb_m3_io.cpp >/dev/null

test_io: $(SYS_OBJ)/io/Vm3_io
	@echo "== m3_io (315-5649, 93C46) against the Supermodel trace"
	./$(SYS_OBJ)/io/Vm3_io $(TRACE)

# R133: the DIP bank from the MRA's <switches> (index 254), in Main's order
$(SYS_OBJ)/loader_sw/Vm3_loader: rtl/cpu/ppc_cache_pkg.sv rtl/sys/m3_loader.sv sim/system/tb_loader_sw.cpp
	@mkdir -p $(SYS_OBJ)/loader_sw
	@$(VERILATOR) $(VFLAGS) --top-module m3_loader -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/loader_sw rtl/cpu/ppc_cache_pkg.sv rtl/sys/m3_loader.sv sim/system/tb_loader_sw.cpp >/dev/null

test_loader_sw: $(SYS_OBJ)/loader_sw/Vm3_loader
	@echo "== m3_loader: the DIP bank from <switches>"
	./$(SYS_OBJ)/loader_sw/Vm3_loader

$(SYS_OBJ)/sdr/Vm3_sdr_tb: rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp rtl/cpu/ppc_cache_pkg.sv
	@$(VERILATOR) $(VFLAGS) --top-module m3_sdr_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/sdr \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp >/dev/null

test_sdr_port: $(SYS_OBJ)/sdr/Vm3_sdr_tb
	@echo "== m3_sdr_port on m2_sdram, random transactions against a reference memory"
	./$(SYS_OBJ)/sdr/Vm3_sdr_tb 200000 1

# R107: the same with the sixteen-column line read (L16) and the bank hash (BXOR), each and both
SDR_VARIANTS = l16 bx l16bx lw1 pair ra pw pw4 all rel $(R112_VARIANTS)
SDR_G_l16 = -GL16=1
SDR_G_bx = -GBXOR=1
SDR_G_l16bx = -GL16=1 -GBXOR=1
SDR_G_lw1 = -GLW1=1
SDR_G_pair = -GL16=1 -GPAIR=1
SDR_G_ra = -GL16=1 -GPAIR=1 -GRA=1
SDR_G_pw = -GLW1=1 -GPW=1
SDR_G_pw4 = -GPW=1
SDR_G_all = -GL16=1 -GBXOR=1 -GLW1=1 -GPAIR=1 -GRA=1 -GRA_CONF=1 -GPW=1
# the set Model3.sv builds (study R107)
SDR_G_rel = -GL16=1 -GBXOR=1 -GLW1=1 -GPAIR=1 -GPW=1
# R112: the miss path's clocks, each on the release set and all together (FAST is the bus's; the port
# bench builds it but has no bus); n112: all five; a112: all five with the read-ahead
R112_VARIANTS = fwd freq fdsp hitrd fast n112 a112 d112 r3p b112
SDR_G_fwd = $(SDR_G_rel) -GFWD=1
SDR_G_freq = $(SDR_G_rel) -GFREQ=1
SDR_G_fdsp = $(SDR_G_rel) -GFDSP=1
SDR_G_hitrd = $(SDR_G_rel) -GHITRD=1
SDR_G_fast = $(SDR_G_rel) -GFAST=1
SDR_G_n112 = $(SDR_G_rel) -GFWD=1 -GFREQ=1 -GFDSP=1 -GHITRD=1 -GFAST=1
SDR_G_a112 = $(SDR_G_n112) -GRA=1 -GRA_CONF=1
# d112: n112 with the posted write's bursts held 16 clocks (PW_DLY)
SDR_G_d112 = $(SDR_G_n112) -GPW_DLY=16
# r3p: the release set with the bus's Real3D post buffer (R3P)
SDR_G_r3p = $(SDR_G_rel) -GR3P=1
# b112: the set Model3.sv builds (R112's six, HITRD off)
SDR_G_b112 = $(SDR_G_rel) -GFWD=1 -GFREQ=1 -GFDSP=1 -GFAST=1 -GPW_DLY=8 -GR3P=1
$(SYS_OBJ)/sdr_%/Vm3_sdr_tb: rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp rtl/cpu/ppc_cache_pkg.sv
	@$(VERILATOR) $(VFLAGS) $(SDR_G_$*) --top-module m3_sdr_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/sdr_$* \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp >/dev/null

test_sdr_port_r107: $(foreach v,$(SDR_VARIANTS),$(SYS_OBJ)/sdr_$(v)/Vm3_sdr_tb)
	@echo "== m3_sdr_port, L16 / BXOR variants"
	./$(SYS_OBJ)/sdr_l16/Vm3_sdr_tb 200000 1
	./$(SYS_OBJ)/sdr_bx/Vm3_sdr_tb 200000 2 --bxor
	./$(SYS_OBJ)/sdr_l16bx/Vm3_sdr_tb 200000 3 --bxor
	./$(SYS_OBJ)/sdr_lw1/Vm3_sdr_tb 200000 4 --lw1
	./$(SYS_OBJ)/sdr_pair/Vm3_sdr_tb 200000 6
	./$(SYS_OBJ)/sdr_ra/Vm3_sdr_tb 200000 7 --seq
	./$(SYS_OBJ)/sdr_ra/Vm3_sdr_tb 200000 8
	./$(SYS_OBJ)/sdr_ra/Vm3_sdr_tb 200000 10 --seq --idle 40
	./$(SYS_OBJ)/sdr_ra/Vm3_sdr_tb 200000 11 --seq --idle 12
	./$(SYS_OBJ)/sdr_pw/Vm3_sdr_tb 200000 12 --lw1
	./$(SYS_OBJ)/sdr_pw/Vm3_sdr_tb 200000 13 --lw1 --idle 12
	./$(SYS_OBJ)/sdr_pw4/Vm3_sdr_tb 200000 14
	./$(SYS_OBJ)/sdr_pw/Vm3_sdr_tb 200000 15 --lw1 --nowait
	./$(SYS_OBJ)/sdr_pw4/Vm3_sdr_tb 200000 16 --nowait
	./$(SYS_OBJ)/sdr_all/Vm3_sdr_tb 200000 17 --bxor --lw1 --seq --nowait --idle 20
	./$(SYS_OBJ)/sdr/Vm3_sdr_tb 200000 18 --nowait
	./$(SYS_OBJ)/sdr_all/Vm3_sdr_tb 200000 5 --bxor --lw1
	./$(SYS_OBJ)/sdr_all/Vm3_sdr_tb 200000 9 --bxor --lw1 --seq
	./$(SYS_OBJ)/sdr_rel/Vm3_sdr_tb 200000 19 --bxor --lw1
	./$(SYS_OBJ)/sdr_rel/Vm3_sdr_tb 200000 20 --bxor --lw1 --nowait --idle 12
	./$(SYS_OBJ)/sdr_fwd/Vm3_sdr_tb 200000 21 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_freq/Vm3_sdr_tb 200000 22 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_fdsp/Vm3_sdr_tb 200000 23 --bxor --lw1 --nowait --idle 12
	./$(SYS_OBJ)/sdr_hitrd/Vm3_sdr_tb 200000 24 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_n112/Vm3_sdr_tb 200000 25 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_n112/Vm3_sdr_tb 200000 26 --bxor --lw1 --idle 12
	./$(SYS_OBJ)/sdr_n112/Vm3_sdr_tb 200000 27 --bxor --lw1
	./$(SYS_OBJ)/sdr_a112/Vm3_sdr_tb 200000 28 --bxor --lw1 --seq --nowait --idle 20
	./$(SYS_OBJ)/sdr_a112/Vm3_sdr_tb 200000 29 --bxor --lw1 --seq --idle 40
	./$(SYS_OBJ)/sdr_d112/Vm3_sdr_tb 200000 30 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_d112/Vm3_sdr_tb 200000 31 --bxor --lw1 --idle 12
	./$(SYS_OBJ)/sdr_d112/Vm3_sdr_tb 200000 32 --bxor --lw1 --nowait --idle 40
	./$(SYS_OBJ)/sdr_b112/Vm3_sdr_tb 200000 33 --bxor --lw1
	./$(SYS_OBJ)/sdr_b112/Vm3_sdr_tb 200000 34 --bxor --lw1 --nowait
	./$(SYS_OBJ)/sdr_b112/Vm3_sdr_tb 200000 35 --bxor --lw1 --idle 12
	./$(SYS_OBJ)/sdr_b112/Vm3_sdr_tb 200000 36 --bxor --lw1 --nowait --idle 40

lint_sys:
	@echo "== lint m3_system"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-TIMESCALEMOD \
	  $(SYS_DEFS) --top-module m3_system $(CPU_RTL) $(TG_RTL) $(SYS_RTL)
	@echo "  clean"

SYS_THREADS ?= 1
# SYS_X = unique: registers without a reset start random (+verilator+seed+N at run
# time), as the board's ALLOW_POWER_UP_DONT_CARE lets them; default fast (zeros)
SYS_X ?= fast
$(SYS_OBJ)/system/Vm3_system: $(CPU_SIM) $(TG_RTL) $(SYS_RTL) sim/system/tb_m3_system.cpp sim/system/sdram_dev.h
	@$(VERILATOR) $(VFLAGS) $(SYS_DEFS) --x-assign $(SYS_X) --x-initial $(SYS_X) --savable --threads $(SYS_THREADS) --top-module m3_system -CFLAGS "$(CFLAGS_SYS) -I$(PWD)/sim/tilegen" \
	  -LDFLAGS "-lz" --Mdir $(SYS_OBJ)/system $(CPU_SIM) $(TG_RTL) $(SYS_RTL) sim/system/tb_m3_system.cpp >/dev/null

system: $(SYS_OBJ)/system/Vm3_system

$(SYS_OBJ)/bus/Vm3_bus_tb: rtl/sys/m3_bus.sv rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_bus_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/bus \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv rtl/sys/m3_bus.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp >/dev/null

test_bus: $(SYS_OBJ)/bus/Vm3_bus_tb
	@echo "== m3_bus with its SDRAM target: two masters against a reference"
	./$(SYS_OBJ)/bus/Vm3_bus_tb 100000 1

# R152: the bus with the tile generator's bus timing (TGT) and the R112 options as the core builds them;
# m3_bus's TGT timing itself against Supermodel's model: sim/system/tb_m3_bus_tg.cpp (test_bus_tg)
$(SYS_OBJ)/bus_tgt/Vm3_bus_tb: rtl/sys/m3_bus.sv rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp
	@mkdir -p $(SYS_OBJ)/bus_tgt
	@$(VERILATOR) $(VFLAGS) -GTGT=1 -GFAST=1 -GR3P=1 --top-module m3_bus_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/bus_tgt \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv rtl/sys/m3_bus.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp >/dev/null
$(SYS_OBJ)/bus_tg/Vm3_bus: rtl/sys/m3_bus.sv sim/system/tb_m3_bus_tg.cpp
	@mkdir -p $(SYS_OBJ)/bus_tg
	@$(VERILATOR) --cc --exe --build -j 6 -O2 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL +define+M3_STEP2_BUS=1 \
	  --public-flat-rw --top-module m3_bus -GR3P=1 -GFAST=1 -GTGT=1 -GTG_MHZ=110 -GSTEP2=1 -CFLAGS "-O2 -std=c++17" --Mdir $(SYS_OBJ)/bus_tg \
	  rtl/cpu/ppc_cache_pkg.sv rtl/sys/m3_bus.sv sim/system/tb_m3_bus_tg.cpp >/dev/null
test_bus_tg: $(SYS_OBJ)/bus_tgt/Vm3_bus_tb $(SYS_OBJ)/bus_tg/Vm3_bus
	@echo "== m3_bus with TGT: two masters against a reference, then the timing against Supermodel's"
	./$(SYS_OBJ)/bus_tgt/Vm3_bus_tb 100000 21
	./$(SYS_OBJ)/bus_tg/Vm3_bus --cycles 4000000 --seed 1 | tail -n 3
	./$(SYS_OBJ)/bus_tg/Vm3_bus --cycles 4000000 --seed 2 --gap 4 | tail -n 3

# R107: the bus with the port's L16 / BXOR / LW1 (the SDR_VARIANTS above)
$(SYS_OBJ)/bus_%/Vm3_bus_tb: rtl/sys/m3_bus.sv rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp
	@mkdir -p $(SYS_OBJ)/bus_$*
	@$(VERILATOR) $(VFLAGS) $(SDR_G_$*) --top-module m3_bus_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/bus_$* \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv rtl/sys/m3_bus.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp >/dev/null

test_bus_r107: $(foreach v,$(SDR_VARIANTS),$(SYS_OBJ)/bus_$(v)/Vm3_bus_tb)
	@echo "== m3_bus, L16 / BXOR / LW1 variants"
	./$(SYS_OBJ)/bus_l16/Vm3_bus_tb 100000 11
	./$(SYS_OBJ)/bus_bx/Vm3_bus_tb 100000 12 --bxor
	./$(SYS_OBJ)/bus_l16bx/Vm3_bus_tb 100000 13 --bxor
	./$(SYS_OBJ)/bus_lw1/Vm3_bus_tb 100000 14
	./$(SYS_OBJ)/bus_pair/Vm3_bus_tb 100000 16
	./$(SYS_OBJ)/bus_ra/Vm3_bus_tb 100000 17
	./$(SYS_OBJ)/bus_pw/Vm3_bus_tb 100000 18
	./$(SYS_OBJ)/bus_pw4/Vm3_bus_tb 100000 19
	./$(SYS_OBJ)/bus_all/Vm3_bus_tb 100000 15 --bxor
	./$(SYS_OBJ)/bus_rel/Vm3_bus_tb 100000 20 --bxor
	./$(SYS_OBJ)/bus_fwd/Vm3_bus_tb 100000 21 --bxor
	./$(SYS_OBJ)/bus_freq/Vm3_bus_tb 100000 22 --bxor
	./$(SYS_OBJ)/bus_fdsp/Vm3_bus_tb 100000 23 --bxor
	./$(SYS_OBJ)/bus_hitrd/Vm3_bus_tb 100000 24 --bxor
	./$(SYS_OBJ)/bus_fast/Vm3_bus_tb 100000 25 --bxor
	./$(SYS_OBJ)/bus_n112/Vm3_bus_tb 100000 26 --bxor
	./$(SYS_OBJ)/bus_n112/Vm3_bus_tb 100000 27 --bxor
	./$(SYS_OBJ)/bus_a112/Vm3_bus_tb 100000 28 --bxor
	./$(SYS_OBJ)/bus_d112/Vm3_bus_tb 100000 29 --bxor
	./$(SYS_OBJ)/bus_d112/Vm3_bus_tb 100000 30 --bxor
	./$(SYS_OBJ)/bus_rel/Vm3_bus_tb 100000 31 --bxor --r3stall
	./$(SYS_OBJ)/bus_r3p/Vm3_bus_tb 100000 32 --bxor
	./$(SYS_OBJ)/bus_r3p/Vm3_bus_tb 100000 33 --bxor --r3stall
	./$(SYS_OBJ)/bus_r3p/Vm3_bus_tb 100000 34 --bxor --r3slow
	./$(SYS_OBJ)/bus_rel/Vm3_bus_tb 100000 35 --bxor --r3slow
	./$(SYS_OBJ)/bus_b112/Vm3_bus_tb 100000 36 --bxor
	./$(SYS_OBJ)/bus_b112/Vm3_bus_tb 100000 37 --bxor --r3stall
	./$(SYS_OBJ)/bus_b112/Vm3_bus_tb 100000 38 --bxor --r3slow
	./$(SYS_OBJ)/bus_b112/Vm3_bus_tb 100000 39 --bxor
