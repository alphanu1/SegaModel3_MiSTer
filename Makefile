# Sega Model 3 core for MiSTer FPGA -- build, lint and test targets.
#
# Lint and simulate locally before any Quartus build: a fit is the most
# expensive way to find an error.

VERILATOR ?= verilator
QUARTUS   ?= /home/ben/intelFPGA_lite/17.0/quartus/bin

CPU_RTL = rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_decode.sv rtl/cpu/ppc_alu.sv \
          rtl/cpu/ppc_muldiv.sv rtl/cpu/ppc_regfile.sv rtl/cpu/ppc_core.sv

.PHONY: all lint test syn_check clean lint_cpu test_cpu
all: lint test

lint: lint_cpu
test: test_cpu

lint_cpu:
	@echo "== lint ppc_core"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module ppc_core $(CPU_RTL)
	@echo "  clean"

# Random programs against the reference model, every instruction compared.
# TEST_ARGS: seeds first_seed length max_wait
TEST_ARGS ?= 200 1 1500 4
obj_ppc_core/Vppc_core: $(CPU_RTL) sim/cpu/tb_ppc_core.cpp sim/cpu/ppc_iss.h
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
	  --top-module ppc_core -Isim/cpu -CFLAGS "-O2 -std=c++17 -I$(PWD)/sim/cpu" \
	  --Mdir obj_ppc_core $(CPU_RTL) sim/cpu/tb_ppc_core.cpp >/dev/null

test_cpu: obj_ppc_core/Vppc_core
	@echo "== test ppc_core against the reference model"
	./obj_ppc_core/Vppc_core $(TEST_ARGS)

# Quartus 17 analysis of the CPU: the dialect Verilator does not enforce.
syn_check:
	@echo "== quartus analysis and elaboration, ppc_core"
	@cd quartus/cpu_spike && $(QUARTUS)/quartus_map --analysis_and_elaboration cpu_spike > /dev/null 2>&1; \
	  grep -E "^Error|Info: Quartus Prime Analysis & Elaboration was successful" output_files/cpu_spike.map.rpt 2>/dev/null | head -5; \
	  grep -h "^Error" quartus/cpu_spike/*.log 2>/dev/null | head -5; true

clean:
	rm -rf obj_*
