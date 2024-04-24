vlib work

vcom -93 ../src/ram.vhd
vcom -93 ../src/alu.vhd
vcom -93 ram_alu_tb.vhd

vsim ram_alu_tb

view signals

add wave *

run -all
