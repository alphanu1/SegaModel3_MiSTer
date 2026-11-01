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
          rtl/real3d/r3d_rast_store.sv rtl/real3d/r3d_rast.sv \
          rtl/real3d/r3d_cmd.sv rtl/real3d/r3d_mem.sv rtl/real3d/r3d_texup.sv rtl/real3d/r3d_vrom_rd.sv \
          rtl/mem/m3_ddr_arb.sv rtl/mem/m3_ddr3p.sv rtl/video/m2_fb_read.sv rtl/video/m3_compose.sv \
          rtl/viewer/m3v_sdr_rd.sv rtl/viewer/m3v_ddr_rd.sv
SYS_RTL = $(R3D_SYS) rtl/dbg/m3_uart_tx.sv rtl/dbg/m3_cpu_mon.sv rtl/sys/m3_jtag.sv rtl/io/m3_93c46.sv rtl/io/m3_io.sv rtl/sys/m3_sdr_port.sv rtl/sys/m3_bus.sv \
          rtl/io/m3_scripts.sv rtl/mem/m2_sdram.sv rtl/video/m2_video_timing.sv rtl/viewer/m3v_overlay.sv \
          rtl/sys/m3_loader.sv rtl/sys/m3_system.sv

.PHONY: test_jtag test_jtag_fault lint_sys test_io test_sdr_port test_bus system

$(SYS_OBJ)/jtag/Vm3_jtag: rtl/sys/m3_jtag.sv sim/system/tb_m3_jtag.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_jtag -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/jtag rtl/sys/m3_jtag.sv sim/system/tb_m3_jtag.cpp >/dev/null

test_jtag: $(SYS_OBJ)/jtag/Vm3_jtag
	@echo "== m3_jtag against the Supermodel trace"
	./$(SYS_OBJ)/jtag/Vm3_jtag $(TRACE)

test_jtag_fault: $(SYS_OBJ)/jtag/Vm3_jtag
	@echo "== m3_jtag, one TDO expectation flipped (must FAIL)"
	! ./$(SYS_OBJ)/jtag/Vm3_jtag $(TRACE) --fault

$(SYS_OBJ)/io/Vm3_io: rtl/io/m3_93c46.sv rtl/io/m3_io.sv sim/system/tb_m3_io.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_io -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/io rtl/io/m3_93c46.sv rtl/io/m3_io.sv sim/system/tb_m3_io.cpp >/dev/null

test_io: $(SYS_OBJ)/io/Vm3_io
	@echo "== m3_io (315-5649, 93C46) against the Supermodel trace"
	./$(SYS_OBJ)/io/Vm3_io $(TRACE)

$(SYS_OBJ)/sdr/Vm3_sdr_tb: rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp rtl/cpu/ppc_cache_pkg.sv
	@$(VERILATOR) $(VFLAGS) --top-module m3_sdr_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/sdr \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv sim/system/m3_sdr_tb.sv sim/system/tb_m3_sdr_port.cpp >/dev/null

test_sdr_port: $(SYS_OBJ)/sdr/Vm3_sdr_tb
	@echo "== m3_sdr_port on m2_sdram, random transactions against a reference memory"
	./$(SYS_OBJ)/sdr/Vm3_sdr_tb 200000 1

lint_sys:
	@echo "== lint m3_system"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-TIMESCALEMOD \
	  --top-module m3_system $(CPU_RTL) $(TG_RTL) $(SYS_RTL)
	@echo "  clean"

SYS_THREADS ?= 1
# SYS_X = unique: registers without a reset start random (+verilator+seed+N at run
# time), as the board's ALLOW_POWER_UP_DONT_CARE lets them; default fast (zeros)
SYS_X ?= fast
$(SYS_OBJ)/system/Vm3_system: $(CPU_SIM) $(TG_RTL) $(SYS_RTL) sim/system/tb_m3_system.cpp sim/system/sdram_dev.h
	@$(VERILATOR) $(VFLAGS) --x-assign $(SYS_X) --x-initial $(SYS_X) --savable --threads $(SYS_THREADS) --top-module m3_system -CFLAGS "$(CFLAGS_SYS) -I$(PWD)/sim/tilegen" \
	  -LDFLAGS "-lz" --Mdir $(SYS_OBJ)/system $(CPU_SIM) $(TG_RTL) $(SYS_RTL) sim/system/tb_m3_system.cpp >/dev/null

system: $(SYS_OBJ)/system/Vm3_system

$(SYS_OBJ)/bus/Vm3_bus_tb: rtl/sys/m3_bus.sv rtl/sys/m3_sdr_port.sv rtl/mem/m2_sdram.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp
	@$(VERILATOR) $(VFLAGS) --top-module m3_bus_tb -CFLAGS "$(CFLAGS_SYS)" --Mdir $(SYS_OBJ)/bus \
	  rtl/cpu/ppc_cache_pkg.sv rtl/mem/m2_sdram.sv rtl/sys/m3_sdr_port.sv rtl/sys/m3_bus.sv sim/system/m3_bus_tb.sv sim/system/tb_m3_bus.cpp >/dev/null

test_bus: $(SYS_OBJ)/bus/Vm3_bus_tb
	@echo "== m3_bus with its SDRAM target: two masters against a reference"
	./$(SYS_OBJ)/bus/Vm3_bus_tb 100000 1
