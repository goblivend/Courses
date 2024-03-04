vlib work
vcom -93 serialor.vhd
vcom -93 serialor_tb.vhd
vsim serialor_tb(TB2)
view signals
add wave *
run -all
