vlib work

vcom -2008 ../src/ram.vhd
vcom -2008 ram_tb.vhd

vsim ram_tb

view signals

add wave *

run -all
