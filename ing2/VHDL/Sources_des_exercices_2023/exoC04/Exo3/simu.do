vlib work
vcom -93 minmax.vhd
vcom -93 minmax_tb.vhd
vsim minmax_tb
view signals
add wave *
run -all
