vlib work

vcom -93 ../src/alu.vhd
vcom -93 alu_tb.vhd

vsim alu_tb

view signals

add wave *

run -all
