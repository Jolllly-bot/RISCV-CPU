create_clock -period 10 -name clk -waveform {0 5} [get_ports cpu_clk]

set_property HD.CLK_SRC BUFGCTRL_X0Y1 [get_ports cpu_clk]

