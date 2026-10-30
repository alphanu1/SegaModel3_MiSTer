# 100 MHz CPU clock (study D-3). Virtual pins carry no I/O timing.
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty
