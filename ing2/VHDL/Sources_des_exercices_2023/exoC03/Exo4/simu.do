vlib work
vcom -93 addc1.vhd
vcom -93 addc4.vhd
vcom -93 addc4_tb.vhd
vsim addc4_tb
view signals
add wave *
run -all
