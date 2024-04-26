vlib work

vcom -93 ../src/mux21.vhd
vcom -93 mux21_tb.vhd

vsim mux21_tb

view signals

add wave *

run -all
