vlib work

vcom -93 ../src/ram.vhd
vcom -93 ram_tb.vhd

vsim ram_tb

view signals

add wave *

run -all
