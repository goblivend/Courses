vlib work
vcom -93 mux41.vhd
vcom -93 mux41_tb.vhd
vsim mux41_tb
view signals
add wave *
run -all
