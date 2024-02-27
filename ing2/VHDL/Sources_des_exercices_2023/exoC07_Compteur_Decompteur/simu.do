vlib work

vcom -93 counter.vhd
vcom -93 counter_tb.vhd

vsim counter_tb(Bench)

view signals
add wave *

run -all