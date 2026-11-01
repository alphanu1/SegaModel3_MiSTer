# m3_scripts, standalone: 100 MHz, the CPU and memory clock (D-28)
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty
