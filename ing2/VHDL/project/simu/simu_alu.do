vlib work

vcom -2008 ../src/alu.vhd
vcom -2008 alu_tb.vhd

vsim alu_tb

view signals

add wave -radix hexadecimal *

run -all
