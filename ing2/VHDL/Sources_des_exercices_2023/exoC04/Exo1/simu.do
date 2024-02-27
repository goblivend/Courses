vlib work
vcom -93 mux81.vhd
vcom -93 mux81_tb.vhd
vsim mux81_tb
view signals
add wave *
run -all
