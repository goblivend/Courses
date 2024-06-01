vlib work

vcom -2008 ../src/ram.vhd
vcom -2008 ../src/alu.vhd
vcom -2008 ram_alu_tb.vhd

vsim ram_alu_tb

view signals

add wave -radix hexadecimal *

run -all
