vlib work

vcom -2008 ../src/mux21.vhd
vcom -2008 mux21_tb.vhd

vsim mux21_tb

view signals

add wave -radix hexadecimal *

run -all
