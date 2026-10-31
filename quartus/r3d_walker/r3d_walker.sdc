# Real3D clock, D-3: 50 MHz
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty
