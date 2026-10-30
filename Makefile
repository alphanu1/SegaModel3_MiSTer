# Sega Model 3 core for MiSTer FPGA -- build, lint and test targets.
#
# Lint and simulate locally before any Quartus build: a fit is the most
# expensive way to find an error.

VERILATOR ?= verilator
QUARTUS   ?= /home/ben/intelFPGA_lite/17.0/quartus/bin

CPU_RTL = rtl/cpu/ppc_pkg.sv rtl/cpu/ppc_decode.sv rtl/cpu/ppc_alu.sv \
          rtl/cpu/ppc_muldiv.sv rtl/cpu/ppc_regfile.sv rtl/cpu/ppc_core.sv
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
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL --top-module ppc_core $(CPU_RTL)
	@echo "  clean"

lint_cputest:
	@echo "== lint m3_cputest (CPU, monitor, UART)"
	@$(VERILATOR) --lint-only -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-PINCONNECTEMPTY \
	  --top-module m3_cputest $(CPU_RTL) $(DBG_RTL)
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

# The board test's program, checked against the model, as ROM images.
cputest_hex: obj_ppc_core/Vppc_core
	@mkdir -p build/cputest
	./obj_ppc_core/Vppc_core --emit $(CPUTEST_SEED) $(CPUTEST_LEN) build/cputest

# The board test system in simulation: prints exactly what the board's UART must.
obj_cputest/Vm3_cputest: $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp cputest_hex
	@$(VERILATOR) --cc --exe --build -j 8 -O3 -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY \
	  --top-module m3_cputest -GDIVISOR=8 \
	  -GCODE_HEX='"$(PWD)/build/cputest/cputest_code.hex"' -GDATA_HEX='"$(PWD)/build/cputest/cputest_data.hex"' \
	  --Mdir obj_cputest $(CPU_RTL) $(DBG_RTL) sim/cpu/tb_cputest.cpp >/dev/null

test_cputest: obj_cputest/Vm3_cputest
	@echo "== board test system in simulation (expected UART output)"
	./obj_cputest/Vm3_cputest 2 | tee build/cputest/expected.txt

# Quartus 17 analysis of the CPU: the dialect Verilator does not enforce.
syn_check:
	@echo "== quartus analysis and elaboration, ppc_core"
	@cd quartus/cpu_spike && $(QUARTUS)/quartus_map --analysis_and_elaboration cpu_spike > /dev/null 2>&1; \
	  grep -E "^Error|Info: Quartus Prime Analysis & Elaboration was successful" output_files/cpu_spike.map.rpt 2>/dev/null | head -5; \
	  grep -h "^Error" quartus/cpu_spike/*.log 2>/dev/null | head -5; true

clean:
	rm -rf obj_*
