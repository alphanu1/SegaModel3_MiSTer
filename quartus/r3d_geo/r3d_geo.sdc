# Real3D clock: 100 MHz, timing closed at 10 ns (sim/real3d/m2-notes.md section 9)
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty
