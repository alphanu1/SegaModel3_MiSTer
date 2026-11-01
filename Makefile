# Sega Model 3 core for MiSTer FPGA -- build, lint and test targets.
#
# Lint and simulate locally before any Quartus build: a fit is the most
# expensive way to find an error.

VERILATOR ?= verilator
QUARTUS   ?= /home/ben/intelFPGA_lite/17.0/quartus/bin

CPU_RTL = rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_fpu_pkg.sv rtl/cpu/ppc_decode.sv rtl/cpu/ppc_alu.sv \
          rtl/cpu/ppc_muldiv.sv rtl/cpu/ppc_regfile.sv rtl/cpu/ppc_fpregfile.sv rtl/cpu/ppc_fetchq.sv \
          rtl/cpu/ppc_fpu_lzc.sv rtl/cpu/ppc_fpu_fma.sv rtl/cpu/ppc_fpu_div.sv rtl/cpu/ppc_fpu_misc.sv \
          rtl/cpu/ppc_fpu_ldst.sv rtl/cpu/ppc_fpu_fpscr.sv rtl/cpu/ppc_fpu.sv rtl/cpu/ppc_fpu_sfma.sv \
          rtl/cpu/ppc_fpu_rsqe.sv rtl/cpu/ppc_fpu_fast.sv rtl/cpu/ppc_pdec.sv rtl/cpu/ppc_pdw.sv rtl/cpu/ppc_btb.sv rtl/cpu/ppc_core.sv \
          rtl/cpu/ppc_cache_pkg.sv rtl/cpu/ppc_cache_ram.sv rtl/cpu/ppc_icache.sv rtl/cpu/ppc_dcache.sv \
          rtl/cpu/ppc_cpu.sv
DBG_RTL = rtl/dbg/m3_uart_tx.sv rtl/dbg/m3_cpu_mon.sv rtl/dbg/m3_cputest.sv

# The board test program: seed and length (tb_ppc_core --emit)
CPUTEST_SEED ?= 1
CPUTEST_LEN  ?= 1500

.PHONY: all lint test syn_check clean lint_cpu test_cpu lint_cputest cputest_hex test_cputest
all: lint test

lint: lint_cpu lint_cputest
test: test_cpu test_cputest

lint_cpu:
	@echo "== lint ppc_core"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM --top-module ppc_core $(CPU_RTL)
	@echo "  clean"

lint_cputest:
	@echo "== lint m3_cputest (CPU, monitor, UART)"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM \
	  --top-module m3_cputest $(CPU_RTL) $(DBG_RTL)
	@echo "  clean"

# Random programs against the reference model, every instruction compared.
# TEST_ARGS: seeds first_seed length max_wait
TEST_ARGS ?= 200 1 1500 4
# CPU_OBJ: the bench's build folder; CPU_GFLAGS: -G overrides (the FPU's
# stage parameters, for the smaller latencies)
CPU_OBJ ?= obj_ppc_core
CPU_GFLAGS ?=
$(CPU_OBJ)/Vppc_core: $(CPU_RTL) sim/cpu/tb_ppc_core.cpp sim/cpu/ppc_iss.h sim/fpu/ppc_fpu_ref.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
	  --top-module ppc_core $(CPU_GFLAGS) -Isim/cpu -CFLAGS "-O2 -std=c++17 -DPPC_FPU_RSQRTE_EST -I$(PWD)/sim/cpu -I$(PWD)/sim/fpu" \
	  -LDFLAGS "-lmpfr -lgmp" --Mdir $(CPU_OBJ) $(CPU_RTL) sim/cpu/tb_ppc_core.cpp >/dev/null

test_cpu: $(CPU_OBJ)/Vppc_core
	@echo "== test ppc_core against the reference model"
	./$(CPU_OBJ)/Vppc_core $(TEST_ARGS)

# the CPU subsystem (core and caches, rtl/cpu/ppc_cpu.sv) behind the caches'
# memory agent; max_wait is the first-word latency's upper bound
SUB_ARGS ?= 200 1 1500 80
SUB_OBJ ?= obj_ppc_cpu
.PHONY: test_cpu_sub
$(SUB_OBJ)/Vppc_cpu: $(CPU_RTL) sim/cpu/tb_ppc_core.cpp sim/cpu/ppc_iss.h sim/fpu/ppc_fpu_ref.h sim/cache/cache_mem.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
	  --top-module ppc_cpu -CFLAGS "-O2 -std=c++17 -DPPC_CPU -DPPC_FPU_RSQRTE_EST -I$(PWD)/sim/cpu -I$(PWD)/sim/fpu -I$(PWD)/sim/cache" \
	  -LDFLAGS "-lmpfr -lgmp" --Mdir $(SUB_OBJ) $(CPU_RTL) sim/cpu/tb_ppc_core.cpp >/dev/null

test_cpu_sub: $(SUB_OBJ)/Vppc_cpu
	@echo "== test ppc_cpu (core and caches) against the reference model"
	./$(SUB_OBJ)/Vppc_cpu $(SUB_ARGS)

# The board test's program, checked against the model, as ROM images.
cputest_hex: $(CPU_OBJ)/Vppc_core
	@mkdir -p build/cputest
	./$(CPU_OBJ)/Vppc_core --emit $(CPUTEST_SEED) $(CPUTEST_LEN) build/cputest

# The board test system in simulation: prints exactly what the board's UART must.
obj_cputest/Vm3_cputest: $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp cputest_hex
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY \
	  --top-module m3_cputest -GDIVISOR=8 \
	  -GCODE_HEX='"$(PWD)/build/cputest/cputest_code.hex"' -GDATA_HEX='"$(PWD)/build/cputest/cputest_data.hex"' \
	  --Mdir obj_cputest $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp >/dev/null

test_cputest: obj_cputest/Vm3_cputest
	@echo "== board test system in simulation (expected UART output)"
	./obj_cputest/Vm3_cputest 2 | tee build/cputest/expected.txt

# the same with CACHED = 1: the core with its caches, the ROM and RAM behind
# the caches' memory port
.PHONY: test_cputest_cached
obj_cputest_c/Vm3_cputest: $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp cputest_hex
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY \
	  --top-module m3_cputest -GDIVISOR=8 -GCACHED=1 \
	  -GCODE_HEX='"$(PWD)/build/cputest/cputest_code.hex"' -GDATA_HEX='"$(PWD)/build/cputest/cputest_data.hex"' \
	  --Mdir obj_cputest_c $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp >/dev/null

test_cputest_cached: obj_cputest_c/Vm3_cputest
	@echo "== board test system with caches in simulation (expected UART output)"
	./obj_cputest_c/Vm3_cputest 2 | tee build/cputest/expected_cached.txt

# Quartus 17 analysis of the CPU: the dialect Verilator does not enforce.
syn_check:
	@echo "== quartus analysis and elaboration, ppc_core"
	@cd quartus/cpu_spike && $(QUARTUS)/quartus_map --analysis_and_elaboration cpu_spike > /dev/null 2>&1; \
	  grep -E "^Error|Info: Quartus Prime Analysis & Elaboration was successful" output_files/cpu_spike.map.rpt 2>/dev/null | head -5; \
	  grep -h "^Error" quartus/cpu_spike/*.log 2>/dev/null | head -5; true

clean:
	rm -rf obj_*

# ---- FPU (rtl/cpu/ppc_fpu_*.sv): lint, the reference model, the unit bench
FPU_RTL = rtl/cpu/ppc_fpu_pkg.sv rtl/cpu/ppc_fpu_lzc.sv rtl/cpu/ppc_fpu_fma.sv rtl/cpu/ppc_fpu_div.sv \
          rtl/cpu/ppc_fpu_misc.sv rtl/cpu/ppc_fpu_ldst.sv rtl/cpu/ppc_fpu_fpscr.sv rtl/cpu/ppc_fpu.sv
FPU_OBJ ?= sm3-builds/fpu_obj
# FPU_ARGS: pipeline_ops comb_ops seed
FPU_ARGS ?= 2000000 2000000 1

.PHONY: lint_fpu test_fpu test_fpu_ref
lint_fpu:
	@echo "== lint ppc_fpu"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module ppc_fpu $(FPU_RTL)
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module fpu_tb_top $(FPU_RTL) sim/fpu/fpu_tb_top.sv
	@echo "  clean"

$(FPU_OBJ)/Vfpu_tb_top: $(FPU_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp sim/fpu/ppc_fpu_ref.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
	  --top-module fpu_tb_top -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/fpu" -LDFLAGS "-lmpfr -lgmp" \
	  --Mdir $(FPU_OBJ) $(FPU_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp >/dev/null

test_fpu: $(FPU_OBJ)/Vfpu_tb_top
	@echo "== FPU units against the reference model"
	./$(FPU_OBJ)/Vfpu_tb_top $(FPU_ARGS)

# The reference model against the host's IEEE arithmetic
$(FPU_OBJ)/test_ref_host: sim/fpu/test_ref_host.cpp sim/fpu/ppc_fpu_ref.h
	@mkdir -p $(FPU_OBJ)
	@g++ -O2 -std=c++17 -frounding-math -Isim/fpu sim/fpu/test_ref_host.cpp -o $@ -lmpfr -lgmp

test_fpu_ref: $(FPU_OBJ)/test_ref_host
	@echo "== FPU reference model against host IEEE arithmetic"
	./$(FPU_OBJ)/test_ref_host 2000000 1

# The FPU regression: the directed set and random seeds, at the default
# stage parameters (L = 12) with the small unit two cycles (PIPE = 1, as
# 120 MHz needs). FPU_REG_OPS: pipeline_ops comb_ops per seed.
FPU_REG_SEEDS ?= 1 2
FPU_REG_OPS   ?= 3000000 1000000
.PHONY: test_fpu_reg
$(FPU_OBJ)_pipe/Vfpu_tb_top: $(FPU_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp sim/fpu/ppc_fpu_ref.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
	  --top-module fpu_tb_top -GMISC_PIPE=1 -CFLAGS "-O2 -std=c++17 -DMISC_PIPE -I$(PWD)/sim/fpu" -LDFLAGS "-lmpfr -lgmp" \
	  --Mdir $(FPU_OBJ)_pipe $(FPU_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp >/dev/null

test_fpu_reg: $(FPU_OBJ)_pipe/Vfpu_tb_top
	@echo "== FPU regression: directed + seeds $(FPU_REG_SEEDS), L = 12, small unit PIPE = 1"
	@for s in $(FPU_REG_SEEDS); do ./$(FPU_OBJ)_pipe/Vfpu_tb_top $(FPU_REG_OPS) $$s | grep -E "^directed|^checked|^FAIL" | head -4; done

# ---- FPU with the fast single-precision port (ppc_fpu_fast, study D-21 /
# D-22): the single unit and the frsqrte estimate beside the double unit.
# The bench routes every op with the RTL's fpu_fast_route, issues fast ops on
# f_fast_ok, checks f-port results arrive in issue order, runs every fast
# single op a second time on the double unit (the two results must be
# identical) and checks each frsqrte estimate against 1/sqrt with MPFR.
# FPU_FAST_G: the single unit's stage boundaries (-GSQ1=0 ... -GSQ5=0 each
# take a cycle off LS = 6); give each setting its own FPU_FAST_OBJ.
FPU_FAST_RTL   = $(FPU_RTL) rtl/cpu/ppc_fpu_sfma.sv rtl/cpu/ppc_fpu_rsqe.sv rtl/cpu/ppc_fpu_fast.sv
FPU_FAST_G     ?=
FPU_FAST_OBJ   ?= $(FPU_OBJ)_fast
FPU_FAST_SEEDS ?= 1 2
FPU_FAST_OPS   ?= 3000000 1000000
.PHONY: lint_fpu_fast test_fpu_fast fpu_rsqe_tab
lint_fpu_fast:
	@echo "== lint ppc_fpu_fast"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module ppc_fpu_fast $(FPU_FAST_G) $(FPU_FAST_RTL)
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module fpu_tb_top -GFAST=1 $(FPU_FAST_G) \
	  $(FPU_FAST_RTL) sim/fpu/fpu_tb_top.sv
	@echo "  clean"

$(FPU_FAST_OBJ)/Vfpu_tb_top: $(FPU_FAST_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp sim/fpu/ppc_fpu_ref.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME --assert \
	  --top-module fpu_tb_top -GMISC_PIPE=1 -GFAST=1 $(FPU_FAST_G) \
	  -CFLAGS "-O2 -std=c++17 -DMISC_PIPE -DFPU_FAST -DPPC_FPU_RSQRTE_EST -I$(PWD)/sim/fpu" -LDFLAGS "-lmpfr -lgmp" \
	  --Mdir $(FPU_FAST_OBJ) $(FPU_FAST_RTL) sim/fpu/fpu_tb_top.sv sim/fpu/tb_fpu.cpp >/dev/null

test_fpu_fast: $(FPU_FAST_OBJ)/Vfpu_tb_top
	@echo "== FPU fast port: directed + seeds $(FPU_FAST_SEEDS) $(FPU_FAST_G)"
	@for s in $(FPU_FAST_SEEDS); do ./$(FPU_FAST_OBJ)/Vfpu_tb_top $(FPU_FAST_OPS) $$s | \
	  grep -E "^directed|^checked|^FAIL|^fast port|frsqrte estimates" | head -8; done

# the frsqrte estimate's table, written into the RTL and the model
fpu_rsqe_tab:
	python3 sim/fpu/gen_rsqe_tab.py rtl/cpu/ppc_fpu_rsqe.sv sim/fpu/ppc_fpu_ref.h

# ---- caches (rtl/cpu/ppc_icache.sv, ppc_dcache.sv): lint and the benches.
# PPC_CACHE_SIM turns on the RAM model's same-edge read corruption and the
# protocol assertions. CACHE_ARGS: ops seed lat_lo lat_hi
CACHE_RTL = rtl/cpu/ppc_cache_pkg.sv rtl/cpu/ppc_cache_ram.sv
CACHE_OBJ ?= sm3-builds/cache_obj
CACHE_SEEDS ?= 1 2 3 4 5 6 7 8
CACHE_OPS ?= 200000
CACHE_VFLAGS = --cc --exe --build -j 8 -O3 --assert -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME +define+PPC_CACHE_SIM

.PHONY: lint_cache test_dcache test_icache test_cache
lint_cache:
	@echo "== lint ppc_dcache, ppc_icache"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM --top-module ppc_dcache $(CACHE_RTL) rtl/cpu/ppc_dcache.sv
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM --top-module ppc_icache $(CACHE_RTL) rtl/cpu/ppc_icache.sv
	@echo "  clean"

$(CACHE_OBJ)/d/Vppc_dcache: $(CACHE_RTL) rtl/cpu/ppc_dcache.sv sim/cache/tb_dcache.cpp sim/cache/cache_mem.h
	@mkdir -p $(CACHE_OBJ)/d
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_dcache -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" \
	  --Mdir $(CACHE_OBJ)/d $(CACHE_RTL) rtl/cpu/ppc_dcache.sv sim/cache/tb_dcache.cpp >/dev/null

$(CACHE_OBJ)/i/Vppc_icache: $(CACHE_RTL) rtl/cpu/ppc_icache.sv sim/cache/tb_icache.cpp sim/cache/cache_mem.h
	@mkdir -p $(CACHE_OBJ)/i
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_icache -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" \
	  --Mdir $(CACHE_OBJ)/i $(CACHE_RTL) rtl/cpu/ppc_icache.sv sim/cache/tb_icache.cpp >/dev/null

test_dcache: $(CACHE_OBJ)/d/Vppc_dcache
	@echo "== ppc_dcache against the reference, seeds $(CACHE_SEEDS)"
	@for s in $(CACHE_SEEDS); do ./$(CACHE_OBJ)/d/Vppc_dcache $(CACHE_OPS) $$s || exit 1; done

test_icache: $(CACHE_OBJ)/i/Vppc_icache
	@echo "== ppc_icache against the reference, seeds $(CACHE_SEEDS)"
	@for s in $(CACHE_SEEDS); do ./$(CACHE_OBJ)/i/Vppc_icache $(CACHE_OPS) $$s || exit 1; done

test_cache: lint_cache test_dcache test_icache

# Planted faults, each in a copy under sm3-builds/cache_fault/ (sim/cache/faults.py)
.PHONY: test_cache_faults
test_cache_faults:
	@python3 sim/cache/faults.py

# ---- Real3D geometry (M2): the C++ reference model and its diff against MAME
# Golden triangle lists come from MAME with tools/mame-census/mame0289-real3d-tris.patch
# (M3REPLAY + M3TRIS + M3TRIS_VROM); dumps and VROM images stay under sm3-builds/.
R3D_OBJ   ?= sm3-builds/r3d/obj
R3D_GAME  ?= vf3
R3D_ARITH ?= rtl
R3D_DUMPS ?= sm3-builds/census/dumps_$(R3D_GAME)
R3D_VROM  ?= sm3-builds/r3d/vrom_$(R3D_GAME).bin
R3D_GOLD  ?= sm3-builds/r3d/tris_$(R3D_GAME)
R3D_SRC    = sim/real3d/r3d_ref.cpp sim/real3d/r3d_geo.cpp
R3D_HDR    = sim/real3d/r3d_geo.h sim/real3d/r3d_fp.h sim/real3d/r3d_trilist.h

.PHONY: real3d_model test_real3d_fp test_real3d
$(R3D_OBJ)/r3d_ref: $(R3D_SRC) $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@g++ -O2 -g -std=c++17 -Wall -Wextra -Isim/real3d -o $@ $(R3D_SRC) -lz

real3d_model: $(R3D_OBJ)/r3d_ref

# the bit-level reciprocal against 1.0f / x on every mantissa
test_real3d_fp: $(R3D_OBJ)/r3d_ref
	@echo "== Real3D reciprocal, every mantissa"
	@./$(R3D_OBJ)/r3d_ref --selftest

# R3D_GAME=vf3|bassdx, R3D_ARITH=host|rtl
test_real3d: test_real3d_fp
	@echo "== Real3D model against MAME's triangle lists, $(R3D_GAME), $(R3D_ARITH) arithmetic"
	@./$(R3D_OBJ)/r3d_ref --dumps $(R3D_DUMPS) --vrom $(R3D_VROM) --golden $(R3D_GOLD) --arith $(R3D_ARITH)

# ---- Real3D display-list walker (rtl/real3d) against the model's event trace
R3D_RTL = rtl/real3d/r3d_walker.sv
R3D_LAT ?= 8
R3D_STALL ?= 20

.PHONY: lint_real3d test_real3d_walker
lint_real3d:
	@echo "== lint r3d_walker"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module r3d_walker $(R3D_RTL)
	@echo "  clean"

$(R3D_OBJ)/walker/Vr3d_walker: $(R3D_RTL) sim/real3d/tb_r3d_walker.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME --top-module r3d_walker \
	  -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(R3D_OBJ)/walker $(R3D_RTL) $(PWD)/sim/real3d/tb_r3d_walker.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null

test_real3d_walker: lint_real3d $(R3D_OBJ)/walker/Vr3d_walker
	@echo "== r3d_walker against the model, $(R3D_GAME), latency $(R3D_LAT), stall $(R3D_STALL)%"
	@./$(R3D_OBJ)/walker/Vr3d_walker --dumps $(R3D_DUMPS) --lat $(R3D_LAT) --stall $(R3D_STALL)

# ---- Real3D geometry datapath (rtl/real3d/r3d_geo.sv and below)
# Micro-code: python3 sim/real3d/gen_ucode.py rewrites r3d_ucode.sv, r3d_ucode_pkg.sv, r3d_ucode.h.
R3D_GEO_RTL = rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_fp_div.sv \
              rtl/real3d/r3d_recip.sv rtl/real3d/r3d_fsqrt.sv rtl/real3d/r3d_setup_cr.sv rtl/real3d/r3d_ucode_pkg.sv \
              rtl/real3d/r3d_ucode.sv rtl/real3d/r3d_vpu.sv rtl/real3d/r3d_walker.sv rtl/real3d/r3d_mstream.sv \
              rtl/real3d/r3d_geo.sv
# R3D_SETUP: cr = correctly rounded set-up (SETUP_CR=1), alg = algebraic (SETUP_CR=0)
# R3D_CULL: 1 = Supermodel-style node culling
R3D_SETUP ?= cr
R3D_CULL  ?= 0
R3D_VFLAGS = --cc --exe --build -j 16 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-WIDTHTRUNC

.PHONY: real3d_ucode lint_real3d_geo test_real3d_units test_real3d_geo
real3d_ucode:
	@python3 sim/real3d/gen_ucode.py

lint_real3d_geo:
	@echo "== lint r3d_geo"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-WIDTHTRUNC --top-module r3d_geo $(R3D_GEO_RTL)
	@echo "  clean"

# FP units, reciprocal (every mantissa), square root, CORDIC set-up, engine kernels
test_real3d_units:
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module tb_r3d_fp_top -CFLAGS "-O2 -std=c++17" --Mdir $(R3D_OBJ)/fp \
	  rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_recip.sv sim/real3d/tb_r3d_fp_top.sv $(PWD)/sim/real3d/tb_r3d_fp.cpp >/dev/null
	@./$(R3D_OBJ)/fp/Vtb_r3d_fp_top 20000000 1
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_fsqrt -CFLAGS "-O2" --Mdir $(R3D_OBJ)/sqrt rtl/real3d/r3d_fsqrt.sv $(PWD)/sim/real3d/tb_r3d_sqrt.cpp >/dev/null
	@./$(R3D_OBJ)/sqrt/Vr3d_fsqrt
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_setup_cr -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" --Mdir $(R3D_OBJ)/cr rtl/real3d/r3d_setup_cr.sv $(PWD)/sim/real3d/tb_r3d_cr.cpp >/dev/null
	@./$(R3D_OBJ)/cr/Vr3d_setup_cr 1000000
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_vpu -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" --Mdir $(R3D_OBJ)/vpu \
	  rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_recip.sv rtl/real3d/r3d_ucode.sv rtl/real3d/r3d_vpu.sv \
	  $(PWD)/sim/real3d/tb_r3d_vpu.cpp >/dev/null
	@./$(R3D_OBJ)/vpu/Vr3d_vpu 300

# the whole geometry against the reference model, triangle by triangle:
# R3D_GAME=vf3|bassdx, R3D_SETUP=cr|alg, R3D_CULL=0|1
$(R3D_OBJ)/geo_$(R3D_SETUP)_$(R3D_CULL)/Vr3d_geo: $(R3D_GEO_RTL) sim/real3d/tb_r3d_geo.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GSETUP_CR=$(if $(filter cr,$(R3D_SETUP)),1,0) -GCULL=$(R3D_CULL) \
	  -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" --Mdir $(R3D_OBJ)/geo_$(R3D_SETUP)_$(R3D_CULL) \
	  $(R3D_GEO_RTL) $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null

test_real3d_geo: lint_real3d_geo $(R3D_OBJ)/geo_$(R3D_SETUP)_$(R3D_CULL)/Vr3d_geo
	@echo "== r3d_geo against the model, $(R3D_GAME), set-up $(R3D_SETUP), cull $(R3D_CULL)"
	@./$(R3D_OBJ)/geo_$(R3D_SETUP)_$(R3D_CULL)/Vr3d_geo --dumps $(R3D_DUMPS) --vrom $(R3D_VROM) --setup $(R3D_SETUP) \
	  $(if $(filter 1,$(R3D_CULL)),--cull,)

# ---- Real3D geometry, the shared-pool datapath (sim/real3d/m2-notes.md section 9)
# Unit benches: FP units with and without subnormals, the divide / reciprocal,
# the one-adder CORDIC set-up against its integer model, the engine kernels.
# (test_real3d_units above predates the micro-code package the engine imports.)
.PHONY: test_real3d_pool test_real3d_geo_all test_real3d_faults
test_real3d_pool:
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module tb_r3d_fp_top -CFLAGS "-O2 -std=c++17" --Mdir $(R3D_OBJ)/fp \
	  rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_recip.sv sim/real3d/tb_r3d_fp_top.sv $(PWD)/sim/real3d/tb_r3d_fp.cpp >/dev/null
	@./$(R3D_OBJ)/fp/Vtb_r3d_fp_top 20000000 1
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_setup_cr -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" --Mdir $(R3D_OBJ)/cr rtl/real3d/r3d_setup_cr.sv $(PWD)/sim/real3d/tb_r3d_cr.cpp >/dev/null
	@./$(R3D_OBJ)/cr/Vr3d_setup_cr 1000000
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_fsqrt -CFLAGS "-O2" --Mdir $(R3D_OBJ)/sqrt rtl/real3d/r3d_fsqrt.sv $(PWD)/sim/real3d/tb_r3d_sqrt.cpp >/dev/null
	@./$(R3D_OBJ)/sqrt/Vr3d_fsqrt
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_vpu -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" --Mdir $(R3D_OBJ)/vpu \
	  rtl/real3d/r3d_ucode_pkg.sv rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_recip.sv \
	  rtl/real3d/r3d_ucode.sv rtl/real3d/r3d_vpu.sv $(PWD)/sim/real3d/tb_r3d_vpu.cpp >/dev/null
	@./$(R3D_OBJ)/vpu/Vr3d_vpu 300

# all three builds (exact set-up, cheap set-up, culling) on both games, 400 frames each
test_real3d_geo_all:
	@for g in vf3 bassdx; do for b in "cr 0" "alg 0" "cr 1"; do set -- $$b; \
	  $(MAKE) --no-print-directory test_real3d_geo R3D_GAME=$$g R3D_SETUP=$$1 R3D_CULL=$$2 || exit 1; done; done

# planted faults, each in a copy under sm3-builds/r3d/faults2/
test_real3d_faults:
	@python3 sim/real3d/faults.py F1_stack_pop F2_vertex_reuse F3_clip_order F4_recip_round F5_viewport_xy \
	  F6_pass_label F7_colour_table F8_shading_lit F9_shade_clamp F10_tex_x F11_viewport_wh F12_tls_bank F13_cordic_round \
	  N1_chunk_order N2_buffer_hazard N3_pbuf_tag N4_ftz_sign N5_vs_window N6_reuse_window N7_div_lane \
	  N8_sm_frustum N9_cull_verdict

# ---- Real3D rasteriser model (sim/real3d/r3d_rast*.cpp, sim/real3d/m3-notes.md)
# Inputs: MAME's triangle lists and frame images from one replay run
# (tools/mame-census/mame0289-real3d-frames.patch, M3TRIS + M3FRAMES), the
# Supermodel dumps for texture RAM, and optionally Supermodel's frame grabs
# (supermodel-real3d-frames.patch). Everything ROM-derived stays in sm3-builds/.
RAST_OBJ   ?= sm3-builds/rast/obj
RAST_DIR   ?= sm3-builds/rast
RAST_ARITH ?= mame
RAST_SRC    = sim/real3d/r3d_rast_ref.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_rast_sm.cpp sim/real3d/r3d_geo.cpp
RAST_HDR    = sim/real3d/r3d_rast.h sim/real3d/r3d_trilist.h sim/real3d/r3d_geo_lod.h sim/real3d/r3d_geo.h

.PHONY: test_real3d_rast test_real3d_rast_all compare_real3d_sm
$(RAST_OBJ)/r3d_rast_ref: $(RAST_SRC) $(RAST_HDR)
	@mkdir -p $(RAST_OBJ)
	@g++ -O2 -g -std=c++17 -Wall -Wextra -ffp-contract=off -Isim/real3d -o $@ $(RAST_SRC) -lz -lpthread

# R3D_GAME=vf3|bassdx, RAST_ARITH=mame|rtl
test_real3d_rast: $(RAST_OBJ)/r3d_rast_ref
	@echo "== Real3D rasteriser model ($(RAST_ARITH) arithmetic) against MAME's 3D layer, $(R3D_GAME)"
	@./$(RAST_OBJ)/r3d_rast_ref --tris $(RAST_DIR)/tris_$(R3D_GAME) --dumps $(R3D_DUMPS) --mame $(RAST_DIR)/mame_$(R3D_GAME) \
	  --arith $(RAST_ARITH) --threads 16

test_real3d_rast_all:
	@for g in vf3 bassdx; do for a in mame rtl; do \
	  $(MAKE) --no-print-directory test_real3d_rast R3D_GAME=$$g RAST_ARITH=$$a || exit 1; done; done

# MAME's frame images against Supermodel's grabs (3D alone and finished frames)
compare_real3d_sm: $(RAST_OBJ)/r3d_rast_ref
	@./$(RAST_OBJ)/r3d_rast_ref --compare-only --mame $(RAST_DIR)/mame_$(R3D_GAME) \
	  --sm $(RAST_DIR)/sm_$(R3D_GAME)_3d/grab --smfull $(RAST_DIR)/sm_$(R3D_GAME)_full/grab --threads 16

# ---- Real3D rasteriser RTL: the 50 MHz front's targets are gone; the front was
# re-timed for 100 MHz (D-28). See test_real3d_rast100_front, _intake, _zspan.

# the geometry as built by default (study D-19): Supermodel's frustum and node
# culling. R3D_SETUP2=sm|cr|alg selects the frustum (cr, alg: MAME's), R3D_CULL2=1|0
R3D_SETUP2 ?= sm
R3D_CULL2  ?= 1
.PHONY: test_real3d_geo2
$(R3D_OBJ)/geo2_$(R3D_SETUP2)_$(R3D_CULL2)/Vr3d_geo: $(R3D_GEO_RTL) sim/real3d/tb_r3d_geo.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GFRUSTUM_SM=$(if $(filter sm,$(R3D_SETUP2)),1,0) -GSETUP_CR=$(if $(filter alg,$(R3D_SETUP2)),0,1) \
	  -GCULL=$(R3D_CULL2) -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" --Mdir $(R3D_OBJ)/geo2_$(R3D_SETUP2)_$(R3D_CULL2) \
	  $(R3D_GEO_RTL) $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
test_real3d_geo2: lint_real3d_geo $(R3D_OBJ)/geo2_$(R3D_SETUP2)_$(R3D_CULL2)/Vr3d_geo
	@echo "== r3d_geo against the model, $(R3D_GAME), frustum $(R3D_SETUP2), cull $(R3D_CULL2)"
	@./$(R3D_OBJ)/geo2_$(R3D_SETUP2)_$(R3D_CULL2)/Vr3d_geo --dumps $(R3D_DUMPS) --vrom $(R3D_VROM) --setup $(R3D_SETUP2) \
	  $(if $(filter 1,$(R3D_CULL2)),--cull,)

# ---- Real3D geometry with the D-19 rows (sim/real3d/m2-notes.md section 11)
# R3D_ROWS=sm (default: Supermodel's rows 12, 17, 18, 20, 21, 22) or mame (every
# SM_* parameter 0); the bench's model takes the same rows (--rows).
R3D_ROWS ?= sm
R3D_ROWP  = $(if $(filter mame,$(R3D_ROWS)),-GSM_FLAT=0 -GSM_FIXED=0 -GSM_DOUBLE=0 -GSM_LIGHT=0 -GSM_TRANS=0 -GSM_CTAB=0,)
.PHONY: test_real3d_geo3 test_real3d_geo3_all
$(R3D_OBJ)/geo3_$(R3D_SETUP2)_$(R3D_CULL2)_$(R3D_ROWS)/Vr3d_geo: $(R3D_GEO_RTL) sim/real3d/tb_r3d_geo.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GFRUSTUM_SM=$(if $(filter sm,$(R3D_SETUP2)),1,0) -GSETUP_CR=$(if $(filter alg,$(R3D_SETUP2)),0,1) \
	  -GCULL=$(R3D_CULL2) $(R3D_ROWP) -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(R3D_OBJ)/geo3_$(R3D_SETUP2)_$(R3D_CULL2)_$(R3D_ROWS) $(R3D_GEO_RTL) $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
test_real3d_geo3: lint_real3d_geo $(R3D_OBJ)/geo3_$(R3D_SETUP2)_$(R3D_CULL2)_$(R3D_ROWS)/Vr3d_geo
	@echo "== r3d_geo against the model, $(R3D_GAME), frustum $(R3D_SETUP2), cull $(R3D_CULL2), rows $(R3D_ROWS)"
	@./$(R3D_OBJ)/geo3_$(R3D_SETUP2)_$(R3D_CULL2)_$(R3D_ROWS)/Vr3d_geo --dumps $(R3D_DUMPS) --vrom $(R3D_VROM) --setup $(R3D_SETUP2) \
	  $(if $(filter 1,$(R3D_CULL2)),--cull,) --rows $(R3D_ROWS)

# every build on both games: Supermodel's frustum with and without culling,
# MAME's correctly rounded frustum with and without, MAME's algebraic one; the
# default build with MAME's rows; and MAME throughout (cr, no culling, MAME rows)
test_real3d_geo3_all:
	@for g in vf3 bassdx; do for b in "sm 1 sm" "sm 0 sm" "cr 1 sm" "cr 0 sm" "alg 0 sm" "sm 1 mame" "cr 0 mame"; do set -- $$b; \
	  $(MAKE) --no-print-directory test_real3d_geo3 R3D_GAME=$$g R3D_SETUP2=$$1 R3D_CULL2=$$2 R3D_ROWS=$$3 || exit 1; done; done

# the lighting rows through the rasteriser's Supermodel form against
# Supermodel's 3D-only grabs: MAME's lighting, Supermodel's per vertex, per
# fragment (sim/real3d/r3d_light_cmp.cpp; light_cmp.py sums the lines)
.PHONY: compare_real3d_light
$(R3D_OBJ)/r3d_light_cmp: sim/real3d/r3d_light_cmp.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR) $(RAST_SRC) $(RAST_HDR)
	@mkdir -p $(R3D_OBJ)
	@g++ -O2 -g -std=c++17 -Wall -Wextra -ffp-contract=off -Isim/real3d -o $@ sim/real3d/r3d_light_cmp.cpp sim/real3d/r3d_geo.cpp \
	  sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_rast_sm.cpp -lz
compare_real3d_light: $(R3D_OBJ)/r3d_light_cmp
	@./$(R3D_OBJ)/r3d_light_cmp --dumps $(R3D_DUMPS) --vrom $(R3D_VROM) --sm $(RAST_DIR)/sm_$(R3D_GAME)_3d/grab > $(R3D_OBJ)/light_$(R3D_GAME).txt
	@python3 sim/real3d/light_cmp.py $(R3D_OBJ)/light_$(R3D_GAME).txt

# planted faults with the D-19 rows and the word stream (sim/real3d/faults.py)
.PHONY: test_real3d_faults2
test_real3d_faults2:
	@python3 sim/real3d/faults.py F1_stack_pop F2_vertex_reuse F3_clip_order F4_recip_round F5_viewport_xy \
	  F6_pass_label F7_colour_table F8_shading_lit F9_intensity_limit F10_tex_x F11_viewport_wh F12_tls_bank F13_cordic_round \
	  N1_chunk_order N2_buffer_hazard N3_pbuf_tag N4_ftz_sign N5_vs_window N6_reuse_window N7_div_lane \
	  N8_sm_frustum N9_cull_verdict D1_face_normal D2_double_second D3_sun_clamp D4_colour_base D5_fixed_sign \
	  D6_trans_drop D7_light_z D8_normal_matrix D9_reuse_key D10_copy_shade D11_list_hazard D12_trans_latch \
	  D13_const_load D14_div_ring

# ---- branch target buffer (rtl/cpu/ppc_btb.sv): lint, random bench, trace bench.
# BTB_CFG: N XW STYLE. BTB_TRACE: a census branch trace (tools/mame-census
# M3BRTRACE), BTB_R52: the CPI model's lv_ BTB hits over its window.
BTB_CFG   ?= 256 0 0
BTB_OBJ   ?= sm3-builds/btb_obj
BTB_SEEDS ?= 1 2 3 4 5 6 7 8
BTB_TRACE ?=
BTB_R52   ?= -1
btb_n  = $(word 1,$(BTB_CFG))
btb_xw = $(word 2,$(BTB_CFG))
btb_st = $(word 3,$(BTB_CFG))
BTB_BIN = $(BTB_OBJ)/n$(btb_n)_x$(btb_xw)_s$(btb_st)/Vppc_btb
.PHONY: lint_btb test_btb
lint_btb:
	@echo "== lint ppc_btb"
	@for g in "-GN=256" "-GN=128 -GXW=22" "-GN=64 -GSTYLE=1"; do \
	  $(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL $$g rtl/cpu/ppc_btb.sv || exit 1; done
	@echo "  clean"
$(BTB_BIN): rtl/cpu/ppc_btb.sv sim/btb/tb_btb.cpp sim/btb/btb_model.h
	@mkdir -p $(dir $(BTB_BIN))
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal --top-module ppc_btb -GN=$(btb_n) -GXW=$(btb_xw) -GSTYLE=$(btb_st) \
	  -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/btb -DBTB_N=$(btb_n) -DBTB_XW=$(btb_xw)" \
	  --Mdir $(dir $(BTB_BIN)) rtl/cpu/ppc_btb.sv sim/btb/tb_btb.cpp >/dev/null
test_btb: $(BTB_BIN)
	@echo "== ppc_btb N=$(btb_n) XW=$(btb_xw) STYLE=$(btb_st) against the model"
	@for s in $(BTB_SEEDS); do ./$(BTB_BIN) random $$s 200000 || exit 1; done
	@if [ -n "$(BTB_TRACE)" ]; then ./$(BTB_BIN) trace $(BTB_TRACE) $(BTB_R52) || exit 1; fi

# ---- dual-issue prototypes (rtl/cpu/ppc_dual_proto.sv, ppc_pair_proto.sv,
# ppc_regfile2w.sv; study D-24): the datapath against in-order execution in
# five configurations, and the queue / pairing front in four.
DUAL_OBJ ?= sm3-builds/dual_obj
DUAL_CFGS = "0 2 1 0 1 0" "1 2 1 0 1 0" "1 4 1 1 1 0" "1 2 0 0 0 1" "1 2 1 0 0 1"
PAIR_CFGS = "4 0" "1 0" "2 0" "1 1"
.PHONY: test_dual
test_dual:
	@echo "== dual-issue prototypes"
	@for c in $(DUAL_CFGS); do set -- $$c; o=$(DUAL_OBJ)/d$$1b$$2w$$3l$$4c$$5p$$6; mkdir -p $$o; \
	  $(VERILATOR) --cc --exe --build -j 8 -O2 -Wno-fatal --top-module ppc_dual_proto -GDUAL=$$1 -GBOPS=$$2 -GBW2=$$3 \
	    -GBLD=$$4 -GBCR=$$5 -GPMUX=$$6 -CFLAGS "-O2 -std=c++17 -DDUAL=$$1 -DBW2=$$3 -DPMUX=$$6" --Mdir $$o \
	    rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_alu.sv rtl/cpu/ppc_regfile2w.sv rtl/cpu/ppc_dual_proto.sv sim/dual/tb_dual.cpp >/dev/null || exit 1; \
	  for s in 1 2 3; do ./$$o/Vppc_dual_proto $$s 100000 || exit 1; done; done
	@for c in $(PAIR_CFGS); do set -- $$c; o=$(DUAL_OBJ)/pp$$1f$$2; mkdir -p $$o; \
	  $(VERILATOR) --cc --exe --build -j 8 -O2 -Wno-fatal --top-module ppc_pair_proto -GVAR=$$1 -GFETCH2=$$2 \
	    -CFLAGS "-O2 -std=c++17 -DFETCH2=$$2 -DVAR4=$$([ $$1 = 4 ] && echo 1 || echo 0)" --Mdir $$o \
	    rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_fpu_pkg.sv rtl/cpu/ppc_decode.sv rtl/cpu/ppc_pair_proto.sv sim/dual/tb_pair.cpp >/dev/null || exit 1; \
	  for s in 1 2 3; do ./$$o/Vppc_pair_proto $$s 200000 || exit 1; done; done

# ---- sound: the SCSP (rtl/sound/scsp, from Model 2 at 47edd9dc; THIRD_PARTY.md)
# Benches against sim/sound/scsp_ref.h (MAME 0.289 scsp.cpp, ported).
# SCSP_CAP=<capture dir> for the replays (regs.bin / ramw.bin / out.bin);
# captures and objects live in sm3-builds/ (ROM-derived data stays out of git).
SCSP_DIR  := rtl/sound/scsp
SCSP_OBJ  ?= sm3-builds/scsp_obj
SCSP_VFL  := -Wall -Wno-DECLFILENAME -I$(SCSP_DIR) -Isim/sound -CFLAGS "-O2 -std=c++17 -I$(CURDIR)/sim/sound"
SCSP_RTL  := $(SCSP_DIR)/m2_scsp.sv $(SCSP_DIR)/m2_scsp_regs.sv $(SCSP_DIR)/m2_scsp_level.sv $(SCSP_DIR)/m2_scsp_cache.sv $(SCSP_DIR)/m2_scsp_dsp.sv
SCSP_HEX  := $(SCSP_DIR)/scsp_lvl.hex $(SCSP_DIR)/scsp_eg.hex $(SCSP_DIR)/scsp_lfoscale.hex $(SCSP_DIR)/scsp_rate.hex \
             $(SCSP_DIR)/scsp_ucf.hex $(SCSP_DIR)/scsp_uce.hex $(SCSP_DIR)/scsp_lfonoise.hex
SCSP_WNO  := -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY

.PHONY: scsp_ucode test_m2_scsp test_m2_scsp_dsp test_m2_scsp_level
# the slot engine's microcode: scsp_uasm.py writes the two ROMs and the layout include
scsp_ucode:
	python3 sim/sound/scsp_uasm.py $(SCSP_DIR)

test_m2_scsp_level: $(SCSP_OBJ)/level/Vm2_scsp_level
	@echo "== m2_scsp_level (the level word against MAME's table, every encoding)"
	@cp -f $(SCSP_HEX) $(SCSP_OBJ)/level/ && cd $(SCSP_OBJ)/level && ./Vm2_scsp_level
$(SCSP_OBJ)/level/Vm2_scsp_level: $(SCSP_DIR)/m2_scsp_level.sv sim/sound/tb_m2_scsp_level.cpp sim/sound/scsp_ref.h
	@mkdir -p $(dir $@)
	$(VERILATOR) --cc --exe --build -j 8 $(SCSP_VFL) --top-module m2_scsp_level --Mdir $(dir $@) \
	  $(SCSP_DIR)/m2_scsp_level.sv sim/sound/tb_m2_scsp_level.cpp >/dev/null

# one SCSP replaying a capture, sample for sample, against scsp_ref.h
test_m2_scsp: $(SCSP_OBJ)/top/Vm2_scsp
	@echo "== m2_scsp (a register stream replayed, sample for sample)"
	@cp -f $(SCSP_HEX) $(SCSP_OBJ)/top/ && cd $(SCSP_OBJ)/top && SCSP_CAP=$(abspath $(SCSP_CAP)) ./Vm2_scsp
$(SCSP_OBJ)/top/Vm2_scsp: $(SCSP_RTL) $(SCSP_DIR)/scsp_tables.svh $(SCSP_DIR)/scsp_lfonoise.svh $(SCSP_DIR)/scsp_uc.svh sim/sound/tb_m2_scsp.cpp sim/sound/scsp_ref.h
	@mkdir -p $(dir $@)
	$(VERILATOR) --cc --exe --build -j 8 $(SCSP_VFL) $(SCSP_WNO) --top-module m2_scsp --Mdir $(dir $@) \
	  $(SCSP_RTL) sim/sound/tb_m2_scsp.cpp >/dev/null

# the DSP alone: a capture's programs (SCSP_CAP) or DSP_RANDOM=<programs>
test_m2_scsp_dsp: $(SCSP_OBJ)/dsp/Vm2_scsp_dsp
	@echo "== m2_scsp_dsp (EFREG and its writes against MAME's, each sample)"
	@cd $(SCSP_OBJ)/dsp && SCSP_CAP=$(abspath $(SCSP_CAP)) ./Vm2_scsp_dsp
$(SCSP_OBJ)/dsp/Vm2_scsp_dsp: $(SCSP_DIR)/m2_scsp_dsp.sv $(SCSP_DIR)/m2_scsp_cache.sv sim/sound/tb_m2_scsp_dsp.cpp sim/sound/scsp_ref.h
	@mkdir -p $(dir $@)
	$(VERILATOR) --cc --exe --build -j 8 $(SCSP_VFL) $(SCSP_WNO) --top-module m2_scsp_dsp --Mdir $(dir $@) \
	  $(SCSP_DIR)/m2_scsp_dsp.sv $(SCSP_DIR)/m2_scsp_cache.sv sim/sound/tb_m2_scsp_dsp.cpp >/dev/null

# synthetic captures (FM, PCM16, noise, SSCTL, MOBUF and reads): scsp_randcap <dir> <seed> <samples> [noise]
$(SCSP_OBJ)/randcap: sim/sound/scsp_randcap.cpp sim/sound/scsp_ref.h
	@mkdir -p $(dir $@)
	g++ -O2 -std=c++17 -Isim/sound $< -o $@

# ---- Model 3's two SCSPs on one engine (rtl/sound/scsp/scsp_dual.sv)
# tb_scsp_dual: the dual against two m2_scsp on the same streams, every output
# sample compared (STEP); FREE=1 measures the clocks a sample pair takes.
# SCSP_CAP=<dir with c0/ c1/>, SCSP_N, LAT_LO / LAT_HI, SCSP_SEED, MHZ.
SCSP_DUAL_RTL := $(SCSP_RTL) $(SCSP_DIR)/scsp_dual.sv $(SCSP_DIR)/scsp_dual_dsp.sv
SCSP_DUAL_HEX := $(SCSP_HEX) $(SCSP_DIR)/scsp_d_ucf.hex $(SCSP_DIR)/scsp_d_uce.hex
.PHONY: scsp_ucode_dual test_scsp_dual
scsp_ucode_dual:
	python3 sim/sound/scsp_uasm.py $(SCSP_DIR) dual
test_scsp_dual: $(SCSP_OBJ)/dual/Vscsp_dual_harness
	@echo "== scsp_dual against two m2_scsp ($(SCSP_CAP))"
	@cp -f $(SCSP_DUAL_HEX) $(SCSP_OBJ)/dual/ && cd $(SCSP_OBJ)/dual && SCSP_CAP=$(abspath $(SCSP_CAP)) ./Vscsp_dual_harness
$(SCSP_OBJ)/dual/Vscsp_dual_harness: $(SCSP_DUAL_RTL) $(SCSP_DIR)/scsp_uc.svh $(SCSP_DIR)/scsp_d_uc.svh sim/sound/scsp_dual_harness.sv sim/sound/tb_scsp_dual.cpp
	@mkdir -p $(dir $@)
	$(VERILATOR) --cc --exe --build -j 8 -O3 $(SCSP_VFL) $(SCSP_WNO) -Wno-REDEFMACRO --top-module scsp_dual_harness --Mdir $(dir $@) \
	  $(SCSP_DUAL_RTL) sim/sound/scsp_dual_harness.sv sim/sound/tb_scsp_dual.cpp >/dev/null
# the clock budget: FREE=1 with where each engine's clocks go (PROF)
.PHONY: test_scsp_dual_budget
test_scsp_dual_budget: $(SCSP_OBJ)/prof/Vscsp_dual_harness
	@echo "== scsp_dual clock budget ($(SCSP_CAP))"
	@cp -f $(SCSP_DUAL_HEX) $(SCSP_OBJ)/prof/ && cd $(SCSP_OBJ)/prof && FREE=1 SCSP_CAP=$(abspath $(SCSP_CAP)) ./Vscsp_dual_harness
$(SCSP_OBJ)/prof/Vscsp_dual_harness: $(SCSP_DUAL_RTL) $(SCSP_DIR)/scsp_uc.svh $(SCSP_DIR)/scsp_d_uc.svh sim/sound/scsp_dual_harness.sv sim/sound/tb_scsp_dual.cpp
	@mkdir -p $(dir $@)
	$(VERILATOR) --cc --exe --build -j 8 -O3 --public-flat-rw -Wno-fatal -Wno-lint -Wno-style -I$(SCSP_DIR) -CFLAGS "-O2 -std=c++17 -DPROF" \
	  --top-module scsp_dual_harness --Mdir $(dir $@) $(SCSP_DUAL_RTL) sim/sound/scsp_dual_harness.sv sim/sound/tb_scsp_dual.cpp >/dev/null

# ---- Real3D geometry, the area pass (sim/real3d/m2-notes.md section 14):
# every planted fault in sim/real3d/faults.py, the area pass's G faults with them
.PHONY: test_real3d_faults3
test_real3d_faults3:
	@python3 sim/real3d/faults.py $$(python3 -c "import importlib.util as u; s = u.spec_from_file_location('f', 'sim/real3d/faults.py'); \
	  m = u.module_from_spec(s); s.loader.exec_module(m); print(' '.join(m.FAULTS))")

# ---- Real3D rasteriser at 100 MHz (D-28): the front with banded walks and the
# shared-lane divider (rtl/real3d/r3d_div_lane.sv), against the model
RAST100_FRONT_RTL = rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv rtl/real3d/r3d_rast_edge.sv rtl/real3d/r3d_rast_front.sv
RAST_BAND ?= 16
.PHONY: lint_real3d_rast100 test_real3d_rast100_front
lint_real3d_rast100:
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME --top-module r3d_rast_front $(RAST100_FRONT_RTL)
$(RAST_OBJ)/edge100/Vr3d_rast_edge: rtl/real3d/r3d_rast_edge.sv sim/real3d/tb_r3d_rast_edge.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_edge -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/edge100 rtl/real3d/r3d_rast_edge.sv $(PWD)/sim/real3d/tb_r3d_rast_edge.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
$(RAST_OBJ)/div100/Vr3d_rast_div: rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv sim/real3d/tb_r3d_rast_div.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_div -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/div100 rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv $(PWD)/sim/real3d/tb_r3d_rast_div.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
$(RAST_OBJ)/front100/Vr3d_rast_front: $(RAST100_FRONT_RTL) sim/real3d/tb_r3d_rast_edge.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_front -CFLAGS "-O2 -std=c++17 -ffp-contract=off -DFRONT -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/front100 $(RAST100_FRONT_RTL) $(PWD)/sim/real3d/tb_r3d_rast_edge.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
test_real3d_rast100_front: lint_real3d_rast100 $(RAST_OBJ)/edge100/Vr3d_rast_edge $(RAST_OBJ)/div100/Vr3d_rast_div $(RAST_OBJ)/front100/Vr3d_rast_front
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/div100/Vr3d_rast_div --tris $(RAST_DIR)/tris_$$g || exit 1; done
	@./$(RAST_OBJ)/div100/Vr3d_rast_div --random 2000000
	@for b in $(RAST_BAND) 0; do for g in vf3 bassdx; do ./$(RAST_OBJ)/edge100/Vr3d_rast_edge --tris $(RAST_DIR)/tris_$$g --gap 20 --stall 20 --band $$b || exit 1; done; done
	@./$(RAST_OBJ)/edge100/Vr3d_rast_edge --synthetic 200000 --gap 20 --stall 20 --band 7
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/front100/Vr3d_rast_front --tris $(RAST_DIR)/tris_$$g --gap 20 --stall 20 --band $(RAST_BAND) || exit 1; done
	@./$(RAST_OBJ)/front100/Vr3d_rast_front --synthetic 200000 --gap 20 --stall 20 --band $(RAST_BAND)

# the set-up intake (option a: the geometry's set-up records, m2-notes.md section 14)
$(RAST_OBJ)/intake/Vr3d_rast_intake: rtl/real3d/r3d_rast_intake.sv sim/real3d/tb_r3d_rast_intake.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_intake -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/intake rtl/real3d/r3d_rast_intake.sv $(PWD)/sim/real3d/tb_r3d_rast_intake.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_intake
test_real3d_rast100_intake: $(RAST_OBJ)/intake/Vr3d_rast_intake
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_intake.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/intake/Vr3d_rast_intake --tris $(RAST_DIR)/tris_$$g --oow || exit 1; done
	@./$(RAST_OBJ)/intake/Vr3d_rast_intake --random 2000000

# ---- the I-cache's pre-read port (ppc_icache PRE = 1, D-30 design C), in the
# protocol the core's BTB uses: sim/btb/tb_icache_pre.cpp
.PHONY: test_icache_pre
test_icache_pre:
	@mkdir -p $(BTB_OBJ)/icpre
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_icache -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" --Mdir $(BTB_OBJ)/icpre \
	  $(CACHE_RTL) rtl/cpu/ppc_icache.sv sim/btb/tb_icache_pre.cpp >/dev/null
	@for s in 1 2 3 4; do ./$(BTB_OBJ)/icpre/Vppc_icache $$s 200000 || exit 1; done

# ---- I-cache pre-read prototype (rtl/cpu/ppc_icache_pr.sv, the R52 "redirect 1"
# lever): directed timing of the built cache, the prototype's own bench, and the
# existing I-cache bench run on the prototype with the new inputs tied off.
.PHONY: test_icache_pr
test_icache_pr:
	@mkdir -p $(BTB_OBJ)/iclv $(BTB_OBJ)/icpr $(BTB_OBJ)/icpr_reg
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_icache -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" --Mdir $(BTB_OBJ)/iclv \
	  $(CACHE_RTL) rtl/cpu/ppc_icache.sv sim/btb/tb_icache_lv.cpp >/dev/null
	./$(BTB_OBJ)/iclv/Vppc_icache
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_icache_pr -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" --Mdir $(BTB_OBJ)/icpr \
	  $(CACHE_RTL) rtl/cpu/ppc_icache_pr.sv sim/btb/tb_icache_pr.cpp >/dev/null
	@for s in 1 2 3; do ./$(BTB_OBJ)/icpr/Vppc_icache_pr $$s 200000 || exit 1; done
	@sed -e 's/Vppc_icache/Vppc_icache_pr/g' -e 's/t->icbi_req = 0;/t->icbi_req = 0; t->i_pv = 0; t->i_pre = 0;/' \
	  sim/cache/tb_icache.cpp > $(BTB_OBJ)/icpr_reg/tb_icache_pr_reg.cpp
	@$(VERILATOR) $(CACHE_VFLAGS) --top-module ppc_icache_pr -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cache" --Mdir $(BTB_OBJ)/icpr_reg \
	  $(CACHE_RTL) rtl/cpu/ppc_icache_pr.sv $(BTB_OBJ)/icpr_reg/tb_icache_pr_reg.cpp >/dev/null
	@for s in $(CACHE_SEEDS); do ./$(BTB_OBJ)/icpr_reg/Vppc_icache_pr $(CACHE_OPS) $$s | tail -1 || exit 1; done

# the depth plane along a span, two pixels a clock
$(RAST_OBJ)/zspan/Vr3d_rast_zspan: rtl/real3d/r3d_rast_zspan.sv sim/real3d/tb_r3d_rast_zspan.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_zspan -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/zspan rtl/real3d/r3d_rast_zspan.sv $(PWD)/sim/real3d/tb_r3d_rast_zspan.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_zspan
test_real3d_rast100_zspan: $(RAST_OBJ)/zspan/Vr3d_rast_zspan
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_zspan.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/zspan/Vr3d_rast_zspan --tris $(RAST_DIR)/tris_$$g --oow || exit 1; done
	@./$(RAST_OBJ)/zspan/Vr3d_rast_zspan --tris $(RAST_DIR)/tris_vf3 --frames 100
	@./$(RAST_OBJ)/zspan/Vr3d_rast_zspan --random 3000000

# ---- Real3D rasteriser at 100 MHz: the Z band and the Z walk's control
# (rtl/real3d/r3d_rast_zband.sv), against render_d25 band by band; RAM contents
# random at power-up. RAST_FRAMES limits the frames per game.
RAST_FRAMES ?= 400
ZBAND_VFLAGS = $(R3D_VFLAGS) --x-initial unique --x-assign unique
$(RAST_OBJ)/zband/Vr3d_rast_zband: rtl/real3d/r3d_rast_zband.sv sim/real3d/tb_r3d_rast_zband.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(ZBAND_VFLAGS) --top-module r3d_rast_zband -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz -pthread" \
	  --Mdir $(RAST_OBJ)/zband rtl/real3d/r3d_rast_zband.sv $(PWD)/sim/real3d/tb_r3d_rast_zband.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
$(RAST_OBJ)/zband1/Vr3d_rast_zband: rtl/real3d/r3d_rast_zband.sv sim/real3d/tb_r3d_rast_zband.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(ZBAND_VFLAGS) -GTRANS=1 --top-module r3d_rast_zband -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz -pthread" \
	  --Mdir $(RAST_OBJ)/zband1 rtl/real3d/r3d_rast_zband.sv $(PWD)/sim/real3d/tb_r3d_rast_zband.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_zband
test_real3d_rast100_zband: $(RAST_OBJ)/zband/Vr3d_rast_zband $(RAST_OBJ)/zband1/Vr3d_rast_zband
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_zband.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/zband/Vr3d_rast_zband --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g \
	  --frames $(RAST_FRAMES) +verilator+rand+reset+2 || exit 1; done
	@for s in 1 2 3 4 5 6 7 8; do ./$(RAST_OBJ)/zband/Vr3d_rast_zband --synthetic 20000 --seed $$s +verilator+rand+reset+2 || exit 1; done
	@./$(RAST_OBJ)/zband/Vr3d_rast_zband --synthetic 20000 --gap 0 --stall 0 --delay 0 +verilator+rand+reset+2
	@./$(RAST_OBJ)/zband/Vr3d_rast_zband --synthetic 20000 --gap 0 --stall 0 --delay 200 +verilator+rand+reset+2
	@for s in 1 2 3; do ./$(RAST_OBJ)/zband1/Vr3d_rast_zband --synthetic 20000 --trans 1 --seed $$s +verilator+rand+reset+2 || exit 1; done
	@./$(RAST_OBJ)/zband1/Vr3d_rast_zband --tris $(RAST_DIR)/tris_vf3 --dumps sm3-builds/census/dumps_vf3 --frames 40 --trans 1 +verilator+rand+reset+2

# the shading lane's interpolants: shade, u, v and the perspective reciprocal
RAST100_INTERP_RTL = rtl/real3d/r3d_rast_recip.sv rtl/real3d/r3d_rast_interp.sv
$(RAST_OBJ)/interp/Vr3d_rast_interp: $(RAST100_INTERP_RTL) sim/real3d/tb_r3d_rast_interp.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_interp -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz -pthread" \
	  --Mdir $(RAST_OBJ)/interp $(RAST100_INTERP_RTL) $(PWD)/sim/real3d/tb_r3d_rast_interp.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_interp
test_real3d_rast100_interp: $(RAST_OBJ)/interp/Vr3d_rast_interp
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME --top-module r3d_rast_interp $(RAST100_INTERP_RTL)
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/interp/Vr3d_rast_interp --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) || exit 1; done
	@./$(RAST_OBJ)/interp/Vr3d_rast_interp --random 3000000

# ---- Real3D geometry: the facing test beside the pool and the R62 record
# (sim/real3d/m2-notes.md section 15). r3d_face.sv joins the geometry's files.
R3D_GEO_RTL += rtl/real3d/r3d_face.sv

# fog's depth term P beside the texel fetch
$(RAST_OBJ)/fogp/Vr3d_rast_fogp: rtl/real3d/r3d_rast_fogp.sv sim/real3d/tb_r3d_rast_fogp.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_fogp -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/fogp rtl/real3d/r3d_rast_fogp.sv $(PWD)/sim/real3d/tb_r3d_rast_fogp.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_fogp
test_real3d_rast100_fogp: $(RAST_OBJ)/fogp/Vr3d_rast_fogp
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_fogp.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/fogp/Vr3d_rast_fogp --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) || exit 1; done
	@./$(RAST_OBJ)/fogp/Vr3d_rast_fogp --random 3000000

# the colour stage: draw kinds, colour modulation, shade, fog's lerp
$(RAST_OBJ)/colour/Vr3d_rast_colour: rtl/real3d/r3d_rast_colour.sv sim/real3d/tb_r3d_rast_colour.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_colour -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/colour rtl/real3d/r3d_rast_colour.sv $(PWD)/sim/real3d/tb_r3d_rast_colour.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_colour
test_real3d_rast100_colour: $(RAST_OBJ)/colour/Vr3d_rast_colour
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_colour.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/colour/Vr3d_rast_colour --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames 100 || exit 1; done
	@./$(RAST_OBJ)/colour/Vr3d_rast_colour --random 3000000

# r3d_face alone against the model's FP32 test: random matrices and polygons, three seeds
.PHONY: test_real3d_face
$(R3D_OBJ)/face/Vr3d_face: rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv sim/real3d/tb_r3d_face.cpp sim/real3d/r3d_fp.h
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_face -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d" --Mdir $(R3D_OBJ)/face \
	  rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv $(PWD)/sim/real3d/tb_r3d_face.cpp >/dev/null
test_real3d_face: $(R3D_OBJ)/face/Vr3d_face
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM --top-module r3d_face \
	  rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv
	@for s in 1 2 3; do ./$(R3D_OBJ)/face/Vr3d_face 300000 $$s || exit 1; done

# the same with the LOD word (LOD = 1, study D-33): B = d x (NP x C) checked
# in order, C changed while idle; KZ2 1.0 (Supermodel's frustum) and MAME's
.PHONY: test_real3d_face_lod
$(R3D_OBJ)/face_lod%/Vr3d_face: rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv sim/real3d/tb_r3d_face.cpp sim/real3d/r3d_fp.h
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_face -GLOD=1 -GKZ2=32\'h$(if $(filter 1,$*),3f800000,408000a8) \
	  -CFLAGS "-O2 -std=c++17 -DFACE_LOD -DFACE_KZ2=0x$(if $(filter 1,$*),3f800000,408000a8)u -I$(PWD)/sim/real3d" --Mdir $(dir $@) \
	  rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv $(PWD)/sim/real3d/tb_r3d_face.cpp >/dev/null
test_real3d_face_lod: $(R3D_OBJ)/face_lod1/Vr3d_face $(R3D_OBJ)/face_lod2/Vr3d_face
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM --top-module r3d_face -GLOD=1 \
	  rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_face.sv
	@for k in 1 2; do for s in 1 2 3; do ./$(R3D_OBJ)/face_lod$$k/Vr3d_face 300000 $$s || exit 1; done; done

# MAME's rows have no facing test
R3D_ROWP = $(if $(filter mame,$(R3D_ROWS)),-GSM_FLAT=0 -GSM_FIXED=0 -GSM_DOUBLE=0 -GSM_LIGHT=0 -GSM_TRANS=0 -GSM_CTAB=0 -GSM_CULL=0,)

# the geometry with the facing test and the R62 record (default: Supermodel's
# frustum, culling, every row). R3D_KEEP=1 is FACE_DROP = 0 (every record sent,
# with its cull flag); R3D_GAME=daytona2 runs the Step 2.1 set (step15).
R3D_KEEP   ?= 0
# R3D_LOD=1: LOD_WORD = 1, the 5 + 6 n record with the LOD word (study D-33)
R3D_LOD    ?= 0
R3D_G4DUMPS = $(if $(filter daytona2,$(R3D_GAME)),sm3-builds/census/dumps_daytona2_r62,$(R3D_DUMPS))
R3D_G4ARGS  = $(if $(filter daytona2,$(R3D_GAME)),--step 21,) $(if $(filter 1,$(R3D_KEEP)),--face-keep,) $(if $(filter 1,$(R3D_LOD)),--lod,)
R3D_G4OBJ   = $(R3D_OBJ)/geo4_$(R3D_SETUP2)_$(R3D_CULL2)_$(R3D_ROWS)_$(R3D_KEEP)$(if $(filter 1,$(R3D_LOD)),_lod,)
.PHONY: test_real3d_geo4 test_real3d_geo4_all test_real3d_faults4 test_real3d_setup4
$(R3D_G4OBJ)/Vr3d_geo: $(R3D_GEO_RTL) sim/real3d/tb_r3d_geo.cpp sim/real3d/r3d_geo.cpp $(R3D_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GFRUSTUM_SM=$(if $(filter sm,$(R3D_SETUP2)),1,0) -GSETUP_CR=$(if $(filter alg,$(R3D_SETUP2)),0,1) \
	  -GCULL=$(R3D_CULL2) $(R3D_ROWP) -GFACE_DROP=$(if $(filter 1,$(R3D_KEEP)),0,1) -GLOD_WORD=$(R3D_LOD) \
	  -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d $(if $(filter mame,$(R3D_ROWS)),,-DFACE_MON)" -LDFLAGS "-lz" \
	  --Mdir $(R3D_G4OBJ) $(R3D_GEO_RTL) $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
test_real3d_geo4: lint_real3d_geo $(R3D_G4OBJ)/Vr3d_geo
	@echo "== r3d_geo (facing test, R62 record) against the model, $(R3D_GAME), frustum $(R3D_SETUP2), cull $(R3D_CULL2), rows $(R3D_ROWS), keep $(R3D_KEEP)"
	@./$(R3D_G4OBJ)/Vr3d_geo --dumps $(R3D_G4DUMPS) --vrom $(R3D_VROM) --setup $(R3D_SETUP2) \
	  $(if $(filter 1,$(R3D_CULL2)),--cull,) --rows $(R3D_ROWS) $(R3D_G4ARGS)

# every build on both games (test_real3d_geo3_all's seven), FACE_DROP = 0, and Daytona 2's default
test_real3d_geo4_all:
	@for g in vf3 bassdx; do for b in "sm 1 sm 0" "sm 0 sm 0" "cr 1 sm 0" "cr 0 sm 0" "alg 0 sm 0" "sm 1 mame 0" "cr 0 mame 0" "sm 1 sm 1"; do set -- $$b; \
	  $(MAKE) --no-print-directory test_real3d_geo4 R3D_GAME=$$g R3D_SETUP2=$$1 R3D_CULL2=$$2 R3D_ROWS=$$3 R3D_KEEP=$$4 || exit 1; done; done
	@$(MAKE) --no-print-directory test_real3d_geo4 R3D_GAME=daytona2

# every planted fault (sim/real3d/faults.py), the facing test's FA faults with them
test_real3d_faults4:
	@R3D_D2DUMPS=daytona2_r62 python3 sim/real3d/faults.py $$(python3 -c "import importlib.util as u; s = u.spec_from_file_location('f', 'sim/real3d/faults.py'); \
	  m = u.module_from_spec(s); s.loader.exec_module(m); print(' '.join(m.FAULTS))")

# the default build behind slow, irregular memories (tb_r3d_geo --hostile, m2-notes
# section 15): late acknowledges, BUSY, gaps in the words, one or two reads held,
# output stalls, a seed each; R3D_TFRAMES frames of each game from R3D_TFIRST
R3D_TFRAMES ?= 20
R3D_TFIRST  ?= 190
R3D_TPROFILES = "--hostile 7 --ack 50 400 --lat 2 20 --maxrd 1 --stall 0 --seed 1" \
                "--hostile 7 --ack 1 200 --lat 1 200 --busy 60 --gap 30 --maxrd 2 --stall 90 --seed 2" \
                "--hostile 7 --ack 1 20 --lat 30 200 --busy 60 --gap 50 --maxrd 1 --stall 50 --seed 3" \
                "--hostile 2 --ack 1 400 --lat 1 120 --busy 30 --stall 20 --seed 4" \
                "--hostile 5 --ack 1 300 --lat 1 200 --busy 40 --gap 20 --maxrd 1 --stall 70 --seed 5" \
                "--hostile 7 --ack 1 2 --lat 1 2 --stall 90 --seed 6"
.PHONY: test_real3d_geo_timing
test_real3d_geo_timing: $(R3D_OBJ)/geo4_sm_1_sm_0/Vr3d_geo
	@for p in $(R3D_TPROFILES); do for g in vf3 bassdx daytona2; do \
	  d=$(R3D_DUMPS); x=; if [ $$g = daytona2 ]; then d=sm3-builds/census/dumps_daytona2_r62; x="--step 21"; else d=sm3-builds/census/dumps_$$g; fi; \
	  echo "== $$g $$p"; R3D_MAXCYC=20000000 ./$(R3D_OBJ)/geo4_sm_1_sm_0/Vr3d_geo --dumps $$d --vrom sm3-builds/r3d/vrom_$$g.bin --setup sm --cull --rows sm $$x \
	    --first $(R3D_TFIRST) --frames $(R3D_TFRAMES) $$p | grep '^r3d_geo:' ; done; done

# the rasteriser's set-up on the pool (m2-notes sections 14, 15): the ROM with
# the set-up kernels and port C is generated into a copy of the tree, then the
# default build with SETUP_LOAD, SETUP_3R and SETUP_PORT on, R3D_GAME's 400 frames
R3D_SU4 = sm3-builds/r3d/su4
test_real3d_setup4:
	@rm -rf $(R3D_SU4); mkdir -p $(R3D_SU4)/sim/real3d $(R3D_SU4)/rtl
	@cp sim/real3d/gen_ucode.py $(R3D_SU4)/sim/real3d/ && cp -r rtl/real3d $(R3D_SU4)/rtl/real3d
	@python3 $(R3D_SU4)/sim/real3d/gen_ucode.py --setup-kernels --3r 2>/dev/null
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GSETUP_LOAD=1 -GSETUP_3R=1 -GSETUP_PORT=1 \
	  -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/real3d -DFACE_MON" -LDFLAGS "-lz" --Mdir $(R3D_SU4)/obj \
	  $(subst rtl/real3d,$(R3D_SU4)/rtl/real3d,$(R3D_GEO_RTL)) $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
	@./$(R3D_SU4)/obj/Vr3d_geo --dumps $(R3D_G4DUMPS) --vrom $(R3D_VROM) --setup sm --cull $(R3D_G4ARGS)

# the set-up on the pool (SETUP_OUT, study D-28 option (a); m2-notes section 15): the
# geometry sends set-up records, checked word by word against rtl_tri::setup
# (r3d_rast_rtl.cpp), so the bench links the rasteriser model; R3D_GAME's 400 frames
R3D_SUOBJ = $(R3D_OBJ)/geo_su
.PHONY: test_real3d_setup_out test_real3d_geo_timing_su
$(R3D_SUOBJ)/Vr3d_geo: $(R3D_GEO_RTL) rtl/real3d/r3d_ucode_su.sv sim/real3d/tb_r3d_geo.cpp sim/real3d/r3d_geo.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(R3D_HDR) $(RAST_HDR)
	@mkdir -p $(R3D_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_geo -GSETUP_OUT=1 \
	  -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d -DFACE_MON -DGEO_SETUP_OUT" -LDFLAGS "-lz" --Mdir $(R3D_SUOBJ) \
	  $(R3D_GEO_RTL) rtl/real3d/r3d_ucode_su.sv $(PWD)/sim/real3d/tb_r3d_geo.cpp $(PWD)/sim/real3d/r3d_geo.cpp \
	  $(PWD)/sim/real3d/r3d_rast.cpp $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
test_real3d_setup_out: $(R3D_SUOBJ)/Vr3d_geo
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-UNUSEDPARAM -Wno-WIDTHTRUNC --top-module r3d_geo -GSETUP_OUT=1 $(R3D_GEO_RTL) rtl/real3d/r3d_ucode_su.sv
	@echo "== r3d_geo SETUP_OUT against rtl_tri::setup, $(R3D_GAME)"
	@./$(R3D_SUOBJ)/Vr3d_geo --dumps $(R3D_G4DUMPS) --vrom $(R3D_VROM) --setup sm --cull $(R3D_G4ARGS) --setup-out
# the hostile profiles of test_real3d_geo_timing on the SETUP_OUT build
test_real3d_geo_timing_su: $(R3D_SUOBJ)/Vr3d_geo
	@for p in $(R3D_TPROFILES); do for g in vf3 bassdx daytona2; do \
	  d=$(R3D_DUMPS); x=; if [ $$g = daytona2 ]; then d=sm3-builds/census/dumps_daytona2_r62; x="--step 21"; else d=sm3-builds/census/dumps_$$g; fi; \
	  echo "== $$g $$p"; R3D_MAXCYC=20000000 ./$(R3D_SUOBJ)/Vr3d_geo --dumps $$d --vrom sm3-builds/r3d/vrom_$$g.bin --setup sm --cull --rows sm $$x \
	    --first $(R3D_TFIRST) --frames $(R3D_TFRAMES) $$p --setup-out | grep '^r3d_geo:' ; done; done

# the bilinear texel: addressing, the eight formats, the filter (ideal 4-bank port)
$(RAST_OBJ)/texel/Vr3d_rast_texel: rtl/real3d/r3d_rast_texel.sv sim/real3d/tb_r3d_rast_texel.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_texel -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/texel rtl/real3d/r3d_rast_texel.sv $(PWD)/sim/real3d/tb_r3d_rast_texel.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_texel
test_real3d_rast100_texel: $(RAST_OBJ)/texel/Vr3d_rast_texel
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_texel.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/texel/Vr3d_rast_texel --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) || exit 1; done
	@./$(RAST_OBJ)/texel/Vr3d_rast_texel --random 3000000

# the texel, the interpolants and the intake with mip levels (MIP = 1, study
# D-33): render_d25 with mip = 1, lod = 1 on every pixel, and random sets
$(RAST_OBJ)/texelm/Vr3d_rast_texel: rtl/real3d/r3d_rast_texel.sv sim/real3d/tb_r3d_rast_texel.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_texel -GMIP=1 -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(dir $@) rtl/real3d/r3d_rast_texel.sv $(PWD)/sim/real3d/tb_r3d_rast_texel.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
$(RAST_OBJ)/interpm/Vr3d_rast_interp: rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_recip.sv sim/real3d/tb_r3d_rast_interp.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_interp -GMIP=1 -CFLAGS "-O2 -std=c++17 -ffp-contract=off -DINTERP_MIP -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(dir $@) rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_recip.sv $(PWD)/sim/real3d/tb_r3d_rast_interp.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
$(RAST_OBJ)/intakem/Vr3d_rast_intake: rtl/real3d/r3d_rast_intake.sv sim/real3d/tb_r3d_rast_intake.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_intake -GMIP=1 -CFLAGS "-O2 -std=c++17 -ffp-contract=off -DINTAKE_MIP -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(dir $@) rtl/real3d/r3d_rast_intake.sv $(PWD)/sim/real3d/tb_r3d_rast_intake.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_mip
test_real3d_rast100_mip: $(RAST_OBJ)/texelm/Vr3d_rast_texel $(RAST_OBJ)/interpm/Vr3d_rast_interp $(RAST_OBJ)/intakem/Vr3d_rast_intake
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -GMIP=1 rtl/real3d/r3d_rast_texel.sv
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -GMIP=1 --top-module r3d_rast_interp rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_recip.sv
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -GMIP=1 rtl/real3d/r3d_rast_intake.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/texelm/Vr3d_rast_texel --mip --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) || exit 1; done
	@./$(RAST_OBJ)/texelm/Vr3d_rast_texel --mip --random 3000000
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/interpm/Vr3d_rast_interp --mip --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) || exit 1; done
	@./$(RAST_OBJ)/interpm/Vr3d_rast_interp --mip --random 3000000
	@./$(RAST_OBJ)/intakem/Vr3d_rast_intake --mip --tris $(RAST_DIR)/tris_vf3 --frames 40 --oow
	@./$(RAST_OBJ)/intakem/Vr3d_rast_intake --mip --random 500000

# texture cache shapes, measured in the RTL's pixel order
$(RAST_OBJ)/r3d_tcache_sim: sim/real3d/r3d_tcache_sim.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_geo.cpp $(RAST_HDR) $(R3D_HDR)
	@mkdir -p $(dir $@)
	@$(CXX) -O2 -std=c++17 -ffp-contract=off -Isim/real3d -o $@ sim/real3d/r3d_tcache_sim.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_geo.cpp -lz -pthread
.PHONY: measure_real3d_tcache
measure_real3d_tcache: $(RAST_OBJ)/r3d_tcache_sim
	@for g in vf3 bassdx; do echo "== $$g"; ./$(RAST_OBJ)/r3d_tcache_sim --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES); done

# ---- M2.5 viewer (rtl/viewer, study D-31): the whole viewer against the models.
# The load files are ROM-derived and live in sm3-builds/viewer only:
#   python3 tools/viewer/m3v_pack.py vrom --zip /home/ben/roms/Model3/vf3.zip --out sm3-builds/viewer/vf3.v3v
#   python3 tools/viewer/m3v_pack.py frames --out sm3-builds/viewer/vf3.v3f <dumps>
VIEW_OBJ   ?= sm3-builds/viewer/obj
VIEW_DUMPS ?= sm3-builds/census/dumps_vf3/f007860.bin.gz
VIEW_ARGS  ?=
VIEW_RTL = rtl/real3d/r3d_ram.sv rtl/real3d/r3d_fp_mul.sv rtl/real3d/r3d_fp_add.sv rtl/real3d/r3d_fp_div.sv \
           rtl/real3d/r3d_recip.sv rtl/real3d/r3d_fsqrt.sv rtl/real3d/r3d_setup_cr.sv rtl/real3d/r3d_ucode_pkg.sv \
           rtl/real3d/r3d_ucode.sv rtl/real3d/r3d_vpu.sv rtl/real3d/r3d_walker.sv rtl/real3d/r3d_mstream.sv \
           rtl/real3d/r3d_face.sv rtl/real3d/r3d_ucode_su.sv rtl/real3d/r3d_geo.sv \
           rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv rtl/real3d/r3d_rast_edge.sv rtl/real3d/r3d_rast_front.sv \
           rtl/real3d/r3d_rast_intake.sv rtl/real3d/r3d_rast_zspan.sv rtl/real3d/r3d_rast_zband.sv \
           rtl/real3d/r3d_rast_recip.sv rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_texel.sv \
           rtl/real3d/r3d_rast_tcache.sv rtl/real3d/r3d_rast_colour.sv rtl/real3d/r3d_rast_band.sv \
           rtl/mem/m2_sdram.sv rtl/mem/m3_ddr3p.sv rtl/video/m2_fb_read.sv rtl/video/m2_video_timing.sv \
           rtl/viewer/m3v_probe.sv rtl/viewer/m3v_store.sv rtl/viewer/m3v_raster.sv rtl/viewer/m3v_sdr_rd.sv \
           rtl/viewer/m3v_ddr_rd.sv rtl/viewer/m3v_ddr_arb.sv rtl/viewer/m3v_loader.sv rtl/viewer/m3v_overlay.sv \
           rtl/viewer/m3v_core.sv
.PHONY: lint_viewer test_viewer
lint_viewer:
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-PINCONNECTEMPTY \
	  -Wno-SYNCASYNCNET -Wno-WIDTHEXPAND --top-module m3v_core $(VIEW_RTL) 2>&1 | grep -v 'rtl/real3d\|rtl/mem\|rtl/video' || true
$(VIEW_OBJ)/Vm3v_core: $(VIEW_RTL) sim/viewer/tb_m3v.cpp sim/real3d/r3d_geo.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(R3D_HDR) $(RAST_HDR)
	@mkdir -p $(VIEW_OBJ)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module m3v_core \
	  -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" --Mdir $(VIEW_OBJ) \
	  $(VIEW_RTL) $(PWD)/sim/viewer/tb_m3v.cpp $(PWD)/sim/real3d/r3d_geo.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
test_viewer: $(VIEW_OBJ)/Vm3v_core
	@./$(VIEW_OBJ)/Vm3v_core --vrom sm3-builds/viewer/vf3.v3v $(foreach d,$(VIEW_DUMPS),--dump $(d)) \
	  --out sm3-builds/viewer/png --video $(VIEW_ARGS)

# the texture cache against the texture RAM, behind a DDR3-like memory model
$(RAST_OBJ)/tcache/Vr3d_rast_tcache: rtl/real3d/r3d_rast_tcache.sv sim/real3d/tb_r3d_rast_tcache.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_rast_tcache -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/tcache rtl/real3d/r3d_rast_tcache.sv $(PWD)/sim/real3d/tb_r3d_rast_tcache.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
# the cache at 64 KiB (SB = 9), the same benches
$(RAST_OBJ)/tcache64/Vr3d_rast_tcache: rtl/real3d/r3d_rast_tcache.sv sim/real3d/tb_r3d_rast_tcache.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_rast_tcache -GSB=9 -CFLAGS "-O2 -std=c++17 -ffp-contract=off -DTC_SB=9 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/tcache64 rtl/real3d/r3d_rast_tcache.sv $(PWD)/sim/real3d/tb_r3d_rast_tcache.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_tcache64
test_real3d_rast100_tcache64: $(RAST_OBJ)/tcache64/Vr3d_rast_tcache
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/tcache64/Vr3d_rast_tcache --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames 100 +verilator+rand+reset+2 || exit 1; done
	@./$(RAST_OBJ)/tcache64/Vr3d_rast_tcache --random 2000000 +verilator+rand+reset+2

# block 7: band buffers, the blend and the layer write (rtl/real3d/r3d_rast_band.sv), against
# render_d25's image on every pixel; a memory with m2_ddr3's contract and random BUSY
$(RAST_OBJ)/band/Vr3d_rast_band: rtl/real3d/r3d_ram.sv rtl/real3d/r3d_rast_band.sv sim/real3d/tb_r3d_rast_band.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_rast_band -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/band rtl/real3d/r3d_ram.sv rtl/real3d/r3d_rast_band.sv $(PWD)/sim/real3d/tb_r3d_rast_band.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_band
test_real3d_rast100_band: $(RAST_OBJ)/band/Vr3d_rast_band
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME --top-module r3d_rast_band rtl/real3d/r3d_ram.sv rtl/real3d/r3d_rast_band.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/band/Vr3d_rast_band --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames $(RAST_FRAMES) +verilator+rand+reset+2 || exit 1; done
	@./$(RAST_OBJ)/band/Vr3d_rast_band --random 2000 +verilator+rand+reset+2
	@./$(RAST_OBJ)/band/Vr3d_rast_band --random 300 --gap 50 --busy 80 +verilator+rand+reset+2

.PHONY: test_real3d_rast100_tcache
test_real3d_rast100_tcache: $(RAST_OBJ)/tcache/Vr3d_rast_tcache
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_tcache.sv
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/tcache/Vr3d_rast_tcache --tris $(RAST_DIR)/tris_$$g --dumps sm3-builds/census/dumps_$$g --frames 100 +verilator+rand+reset+2 || exit 1; done
	@./$(RAST_OBJ)/tcache/Vr3d_rast_tcache --random 2000000 +verilator+rand+reset+2
	@./$(RAST_OBJ)/tcache/Vr3d_rast_tcache --random 500000 --lat 2 4 --gap 0 --stall 0 +verilator+rand+reset+2

# the intake on R62's record (SETUP4 = 0): a 3-word header, five parameters textured
$(RAST_OBJ)/intake62/Vr3d_rast_intake: rtl/real3d/r3d_rast_intake.sv sim/real3d/tb_r3d_rast_intake.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) -GSETUP4=0 --top-module r3d_rast_intake -CFLAGS "-O2 -std=c++17 -ffp-contract=off -DSETUP5 -I$(PWD)/sim/real3d" -LDFLAGS "-lz" \
	  --Mdir $(RAST_OBJ)/intake62 rtl/real3d/r3d_rast_intake.sv $(PWD)/sim/real3d/tb_r3d_rast_intake.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp >/dev/null
.PHONY: test_real3d_rast100_intake62
test_real3d_rast100_intake62: $(RAST_OBJ)/intake62/Vr3d_rast_intake
	@./$(RAST_OBJ)/intake62/Vr3d_rast_intake --tris $(RAST_DIR)/tris_vf3 --frames 40 --oow
	@./$(RAST_OBJ)/intake62/Vr3d_rast_intake --random 500000

# ---- the whole rasteriser (blocks 8 and 9: r3d_rast_store, r3d_rast) against
# render_d25 on 400 + 400 frames, behind DDR3 ports with their contracts
# enforced; sim/real3d/m3-notes.md section 12. RAST_JOBS processes of
# RAST_CHUNK frames each; logs in sm3-builds/rast/w_<game>_<profile>.log.
RAST_TOP_RTL = rtl/real3d/r3d_div_lane.sv rtl/real3d/r3d_rast_div.sv rtl/real3d/r3d_rast_edge.sv rtl/real3d/r3d_rast_front.sv \
  rtl/real3d/r3d_rast_intake.sv rtl/real3d/r3d_rast_zspan.sv rtl/real3d/r3d_rast_zband.sv rtl/real3d/r3d_rast_recip.sv \
  rtl/real3d/r3d_rast_interp.sv rtl/real3d/r3d_rast_fogp.sv rtl/real3d/r3d_rast_texel.sv rtl/real3d/r3d_rast_tcache.sv \
  rtl/real3d/r3d_rast_colour.sv rtl/real3d/r3d_ram.sv rtl/real3d/r3d_rast_band.sv rtl/real3d/r3d_rast_fogvp.sv \
  rtl/real3d/r3d_rast_store.sv rtl/real3d/r3d_rast.sv
RAST_JOBS  ?= 12
RAST_CHUNK ?= 25
# the port profiles: p0 board-like; p1 hostile acknowledges (up to 400 cycles
# on 30% of store and layer requests, 2% of texture lines), BUSY 60%, latency
# 20-200, beat gaps 30%, 8 lines in flight, input gaps 50%
RAST_P0 =
RAST_P1 = --ack 1 400 30 --tack 1 400 2 --busy 60 --lat 20 200 --gap 30 --maxrd 8 --igap 50
$(RAST_OBJ)/rast/Vr3d_rast: $(RAST_TOP_RTL) sim/real3d/tb_r3d_rast.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_geo.cpp $(RAST_HDR) $(R3D_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_rast -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" \
	  -LDFLAGS "-lz -pthread" --Mdir $(RAST_OBJ)/rast $(RAST_TOP_RTL) $(PWD)/sim/real3d/tb_r3d_rast.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
# the configuration that runs (study R75, R79): MIP = 1, B the geometry's LOD word
$(RAST_OBJ)/rastm/Vr3d_rast: $(RAST_TOP_RTL) sim/real3d/tb_r3d_rast.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp sim/real3d/r3d_geo.cpp $(RAST_HDR) $(R3D_HDR)
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_rast -GMIP=1 -CFLAGS "-O2 -std=c++17 -ffp-contract=off -I$(PWD)/sim/real3d" \
	  -LDFLAGS "-lz -pthread" --Mdir $(RAST_OBJ)/rastm $(RAST_TOP_RTL) $(PWD)/sim/real3d/tb_r3d_rast.cpp $(PWD)/sim/real3d/r3d_rast.cpp \
	  $(PWD)/sim/real3d/r3d_rast_rtl.cpp $(PWD)/sim/real3d/r3d_geo.cpp >/dev/null
$(RAST_OBJ)/fogvp/Vr3d_rast_fogvp: rtl/real3d/r3d_rast_fogvp.sv sim/real3d/tb_r3d_rast_fogvp.cpp
	@mkdir -p $(dir $@)
	@$(VERILATOR) $(R3D_VFLAGS) --top-module r3d_rast_fogvp -CFLAGS "-O2 -std=c++17" --Mdir $(RAST_OBJ)/fogvp \
	  rtl/real3d/r3d_rast_fogvp.sv $(PWD)/sim/real3d/tb_r3d_rast_fogvp.cpp >/dev/null
$(RAST_OBJ)/r3d_store_census: sim/real3d/r3d_store_census.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp $(RAST_HDR)
	@mkdir -p $(dir $@)
	@g++ -O2 -std=c++17 -ffp-contract=off -Isim/real3d -o $@ sim/real3d/r3d_store_census.cpp sim/real3d/r3d_rast.cpp sim/real3d/r3d_rast_rtl.cpp -lz -lpthread
.PHONY: lint_real3d_rast_top test_real3d_rast_top test_real3d_rast_top_mip measure_real3d_store
lint_real3d_rast_top:
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME rtl/real3d/r3d_rast_store.sv
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL rtl/real3d/r3d_rast_fogvp.sv
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY --top-module r3d_rast $(RAST_TOP_RTL)
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY --top-module r3d_rast -GMIP=1 $(RAST_TOP_RTL)
test_real3d_rast_top: lint_real3d_rast_top $(RAST_OBJ)/rast/Vr3d_rast $(RAST_OBJ)/fogvp/Vr3d_rast_fogvp
	@for s in 1 2 3; do ./$(RAST_OBJ)/fogvp/Vr3d_rast_fogvp 1000000 $$s || exit 1; done
	@for g in vf3 bassdx; do echo "== r3d_rast, $$g, profile p0"; \
	  sim/real3d/run_rast.sh $(PWD)/$(RAST_OBJ)/rast/Vr3d_rast $$g $(RAST_CHUNK) $(RAST_JOBS) $(PWD)/sm3-builds/rast/w_$${g}_p0.log $(RAST_P0) || exit 1; \
	  echo "== r3d_rast, $$g, profile p1"; \
	  sim/real3d/run_rast.sh $(PWD)/$(RAST_OBJ)/rast/Vr3d_rast $$g $(RAST_CHUNK) $(RAST_JOBS) $(PWD)/sm3-builds/rast/w_$${g}_p1.log $(RAST_P1) || exit 1; done
# MIP = 1 with the geometry's B (render_d25 mip 1, lod 1, d25_opts::lodb); logs w_<game>_m<profile>.log
test_real3d_rast_top_mip: lint_real3d_rast_top $(RAST_OBJ)/rastm/Vr3d_rast
	@for g in vf3 bassdx; do echo "== r3d_rast MIP, $$g, profile p0"; \
	  sim/real3d/run_rast.sh $(PWD)/$(RAST_OBJ)/rastm/Vr3d_rast $$g $(RAST_CHUNK) $(RAST_JOBS) $(PWD)/sm3-builds/rast/w_$${g}_mp0.log --mip --geo-lod $(PWD)/sm3-builds/r3d/vrom_$$g.bin $(RAST_P0) || exit 1; \
	  echo "== r3d_rast MIP, $$g, profile p1"; \
	  sim/real3d/run_rast.sh $(PWD)/$(RAST_OBJ)/rastm/Vr3d_rast $$g $(RAST_CHUNK) $(RAST_JOBS) $(PWD)/sm3-builds/rast/w_$${g}_mp1.log --mip --geo-lod $(PWD)/sm3-builds/r3d/vrom_$$g.bin $(RAST_P1) || exit 1; done
measure_real3d_store: $(RAST_OBJ)/r3d_store_census
	@for g in vf3 bassdx; do ./$(RAST_OBJ)/r3d_store_census --tris $(RAST_DIR)/tris_$$g; done

# ---------------------------------------------------------------- tile generator (M0-E)
# Dumps in sm3-builds/tilegen (tools/mame-census/README.md, "Tile generator dumps").
# test_tilegen_ref: the reference renderers against MAME's draw_layer and Supermodel's DrawLine.
# test_tilegen: m3_tilegen in both configurations against the reference, quick and hostile memories.
TG_DIR  = sm3-builds/tilegen
TG_RTL  = rtl/real3d/r3d_ram.sv rtl/video/m3_tilegen_pal.sv rtl/video/m3_tilegen_fetch.sv rtl/video/m3_tilegen.sv
TG_MAME = -GMASK_SCREEN=0 -GSCROLL_SM=0 -GRS_SWAP=0 -GSTALE=0 -GCODE_WRAP=1 -GEXPAND_SM=0 -GOFFSET=0
TG_VFLAGS = --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-UNUSEDSIGNAL --top-module m3_tilegen
$(TG_DIR)/bin/tg_tool: sim/tilegen/tg_tool.cpp sim/tilegen/tg_ref.h
	@mkdir -p $(dir $@)
	@g++ -O2 -std=c++17 -Wall -Wextra -o $@ sim/tilegen/tg_tool.cpp
$(TG_DIR)/obj_sm/Vm3_tilegen: $(TG_RTL) sim/tilegen/tb_tilegen.cpp sim/tilegen/tg_ref.h
	@$(VERILATOR) $(TG_VFLAGS) -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/tilegen" --Mdir $(TG_DIR)/obj_sm $(TG_RTL) sim/tilegen/tb_tilegen.cpp >/dev/null
$(TG_DIR)/obj_mame/Vm3_tilegen: $(TG_RTL) sim/tilegen/tb_tilegen.cpp sim/tilegen/tg_ref.h
	@$(VERILATOR) $(TG_VFLAGS) $(TG_MAME) -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/tilegen" --Mdir $(TG_DIR)/obj_mame $(TG_RTL) sim/tilegen/tb_tilegen.cpp >/dev/null
.PHONY: lint_tilegen test_tilegen_ref test_tilegen faults_tilegen
lint_tilegen:
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-WIDTHEXPAND -Wno-UNUSEDSIGNAL --top-module m3_tilegen $(TG_RTL)
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-WIDTHEXPAND -Wno-UNUSEDSIGNAL --top-module m3_tilegen $(TG_MAME) $(TG_RTL)
test_tilegen_ref: $(TG_DIR)/bin/tg_tool
	@cd $(TG_DIR) && ./bin/tg_tool check mame_vf3 sm_vf3 sm_bassdx | tail -n 1
test_tilegen: lint_tilegen $(TG_DIR)/obj_sm/Vm3_tilegen $(TG_DIR)/obj_mame/Vm3_tilegen
	@cd $(TG_DIR) && ./obj_sm/Vm3_tilegen --cfg sm --mutate 2 sm_vf3 sm_bassdx | tail -n 3
	@cd $(TG_DIR) && ./obj_mame/Vm3_tilegen --cfg mame --mutate 2 mame_vf3 | tail -n 3
	@cd $(TG_DIR) && ./obj_sm/Vm3_tilegen --cfg sm --lat 1 400 --busy 30 --max 10 sm_vf3 sm_bassdx | tail -n 3
	@cd $(TG_DIR) && ./obj_sm/Vm3_tilegen --cfg sm --realtime --lat 24 28 --max 10 sm_vf3 sm_bassdx | tail -n 3
faults_tilegen:
	@python3 sim/tilegen/faults.py

# ---------------------------------------------------------------- 53C810 SCRIPTS (M0-F)
# Captures and vectors in sm3-builds/scripts_cap (tools/mame-census/README.md, "53C810 capture").
# test_scripts_ref: the reference replays each capture exactly in its emulator's form.
# test_scripts: m3_scripts against the HW-form reference: every capture behind a quick memory,
#   samples behind hostile ones, synthetic programs, and each build variant.
# faults_scripts: planted faults.
SC_DIR = sm3-builds/scripts_cap
SC_RTL = rtl/cpu/ppc_cache_pkg.sv rtl/io/m3_scripts.sv
SC_SM  = vec/sm_vf3.vec vec/sm_bassdx.vec vec/sm_vf3tb.vec
SC_MAME = vec/mame_vf3.vec vec/mame_vf3_play.vec
SC_HDR = sim/scripts/tb_m3_scripts.cpp sim/scripts/m3s_ref.h sim/scripts/m3s_vec.h
SC_VFLAGS = --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-UNUSEDSIGNAL --top-module m3_scripts
$(SC_DIR)/bin/tb_ref: sim/scripts/tb_ref.cpp sim/scripts/m3s_ref.h sim/scripts/m3s_vec.h
	@mkdir -p $(dir $@)
	@g++ -O2 -std=c++17 -Wall -Wextra -o $@ sim/scripts/tb_ref.cpp
define SC_BUILD
$(SC_DIR)/obj_$(1)/Vm3_scripts: $(SC_RTL) $(SC_HDR)
	@$(VERILATOR) $(SC_VFLAGS) $(2) -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/scripts" --Mdir $(SC_DIR)/obj_$(1) $(SC_RTL) sim/scripts/tb_m3_scripts.cpp >/dev/null
endef
$(eval $(call SC_BUILD,l1,))
$(eval $(call SC_BUILD,l0,-GLINE=0))
$(eval $(call SC_BUILD,rdall,-GRD_ALL=1))
$(eval $(call SC_BUILD,noint,-GINT_OP=0))
$(eval $(call SC_BUILD,off,-GENABLE=0))
.PHONY: scripts_vec lint_scripts test_scripts_ref test_scripts faults_scripts
scripts_vec:
	@cd $(SC_DIR) && mkdir -p vec && for g in vf3 bassdx vf3tb; do python3 $(PWD)/sim/scripts/cap2vec.py sm sm_$$g.log vec/sm_$$g.vec; done
	@cd $(SC_DIR) && for g in vf3 vf3_play; do python3 $(PWD)/sim/scripts/cap2vec.py mame mame_$$g.log vec/mame_$$g.vec; done
lint_scripts:
	@for g in "" -GLINE=0 -GRD_ALL=1 -GINT_OP=0 -GENABLE=0; do \
	  $(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module m3_scripts $$g $(SC_RTL) || exit 1; done
test_scripts_ref: $(SC_DIR)/bin/tb_ref
	@cd $(SC_DIR) && ./bin/tb_ref --form sm $(SC_SM) && ./bin/tb_ref --form mame $(SC_MAME)
	@cd $(SC_DIR) && ./bin/tb_ref --form hw $(SC_SM) $(SC_MAME) | grep -v "^  read\|^vec" ; true
test_scripts: lint_scripts $(SC_DIR)/obj_l1/Vm3_scripts $(SC_DIR)/obj_l0/Vm3_scripts $(SC_DIR)/obj_rdall/Vm3_scripts $(SC_DIR)/obj_noint/Vm3_scripts $(SC_DIR)/obj_off/Vm3_scripts
	@cd $(SC_DIR) && ./obj_l1/Vm3_scripts --ack 0 2 --synth 2000 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_l1/Vm3_scripts --seed 11 --ack 0 400 --busy 30 --every 40 --synth 1000 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_l1/Vm3_scripts --seed 12 --ack 100 400 --busy 60 --gap 50 --every 60 --synth 500 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_l1/Vm3_scripts --seed 13 --ack 0 3 --gap 90 --every 20 --synth 1000 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_l1/Vm3_scripts --seed 14 --ack 0 400 --busy 30 --poll --every 40 --synth 1000 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_l0/Vm3_scripts --seed 7 --ack 0 400 --busy 30 --every 40 --synth 1000 $(SC_SM) $(SC_MAME) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_rdall/Vm3_scripts --rd_all --seed 5 --ack 0 400 --busy 30 --synth 1500 | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_noint/Vm3_scripts --no_int --seed 6 --ack 0 400 --busy 30 --every 40 --synth 1500 $(SC_SM) | grep -v "^  moved" | tail -n 3
	@cd $(SC_DIR) && ./obj_off/Vm3_scripts --off | grep -v "^  moved" | tail -n 3
faults_scripts:
	@python3 sim/scripts/faults.py

# ==== D-37 stage B: the Real3D fed live (sim/system/notes.md, Stage B) ====
SB_OBJ    ?= sm3-builds/sb/obj
SB_ORACLE ?= sm3-builds/sb_oracle
SB_TRACE  ?= sm3-builds/sys/sm_bus.bin
SB_SDRAM  ?= sm3-builds/sys/vf3_sdram.bin
SB_HOST_RTL = rtl/real3d/r3d_cmd.sv rtl/real3d/r3d_mem.sv rtl/real3d/r3d_texup.sv rtl/mem/m3_ddr_arb.sv \
              rtl/mem/m2_sdram.sv rtl/mem/m3_ddr3p.sv sim/real3d/r3d_host_tb.sv
SB_LIST   = $(shell cat $(SB_ORACLE)/list.txt 2>/dev/null)
.PHONY: lint_sb test_sb_host
lint_sb:
	@for m in r3d_cmd r3d_mem r3d_texup m3_ddr_arb; do \
	  $(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module $$m rtl/real3d/r3d_cmd.sv rtl/real3d/r3d_mem.sv rtl/real3d/r3d_texup.sv rtl/mem/m3_ddr_arb.sv || exit 1; done
$(SB_OBJ)/host/Vr3d_host_tb: $(SB_HOST_RTL) sim/real3d/tb_r3d_host.cpp sim/system/sdram_dev.h
	@mkdir -p $(SB_OBJ)/host
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module r3d_host_tb -CFLAGS "-O2 -std=c++17" \
	  --Mdir $(SB_OBJ)/host $(SB_HOST_RTL) $(PWD)/sim/real3d/tb_r3d_host.cpp >/dev/null
SB_HOST = ./$(SB_OBJ)/host/Vr3d_host_tb --trace $(SB_TRACE) --sdram $(SB_SDRAM) --dump $(SB_ORACLE)/dump --quiet +verilator+rand+reset+2
# test_sb_host: synthetic uploads of every type and depth; the trace with the
# geometry overlapping the next frame's writes; the whole trace at real frame
# length (about 13 minutes)
test_sb_host: lint_sb $(SB_OBJ)/host/Vr3d_host_tb
	@$(SB_HOST) --list 1 --to 300 --synth-tex 2000 | grep -v " 0 texels differ" | tail -n 3
	@$(SB_HOST) --list $(SB_LIST) --to 1500 --geo 150000 400000 --rast 20000 150000 --minchk 20 | tail -n 3
	@$(SB_HOST) --list $(SB_LIST) --fmin 1738525 --geo 350000 1560000 --rast 350000 1200000 --minchk 40 | tail -n 3
SB_COMP_RTL = rtl/video/m3_compose.sv rtl/video/m2_fb_read.sv rtl/video/m2_video_timing.sv rtl/mem/m3_ddr3p.sv sim/video/m3_compose_tb.sv
$(SB_OBJ)/comp/Vm3_compose_tb: $(SB_COMP_RTL) sim/video/tb_m3_compose.cpp
	@mkdir -p $(SB_OBJ)/comp
	@$(VERILATOR) $(R3D_VFLAGS) --x-initial unique --x-assign unique --top-module m3_compose_tb -CFLAGS "-O2 -std=c++17" \
	  --Mdir $(SB_OBJ)/comp $(SB_COMP_RTL) $(PWD)/sim/video/tb_m3_compose.cpp >/dev/null
.PHONY: test_sb_compose
test_sb_compose: $(SB_OBJ)/comp/Vm3_compose_tb
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY -Wno-SYNCASYNCNET --top-module m3_compose rtl/video/m3_compose.sv rtl/video/m2_fb_read.sv
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f001399.surf --layer $(SB_ORACLE)/layer_1400.bin --grab $(SB_ORACLE)/grab_1400.rgb --buf 1 --ppm $(SB_OBJ)/comp/f1400.ppm +verilator+rand+reset+2
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f001499.surf --layer $(SB_ORACLE)/layer_1500.bin --grab $(SB_ORACLE)/grab_1500.rgb --buf 2 --busy 50 --lat 13 120 +verilator+rand+reset+2
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f000999.surf --layer $(SB_ORACLE)/layer_1000.bin --grab $(SB_ORACLE)/grab_1000.rgb --show 0 --buf 3 +verilator+rand+reset+2
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f001399.surf --synth 7 --buf 0 --busy 40 --lat 8 100 +verilator+rand+reset+2
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f001374.surf --synth 9 --buf 3 +verilator+rand+reset+2
	@./$(SB_OBJ)/comp/Vm3_compose_tb --surf $(SB_ORACLE)/tg/f001399.surf --synth 5 --show 0 --buf 1 +verilator+rand+reset+2
.PHONY: faults_sb
faults_sb:
	@python3 sim/real3d/faults_sb.py
