vlib work

vcom -93 ../src/alu.vhd
vcom -93 ../src/exten.vhd
vcom -93 ../src/mem.vhd
vcom -93 ../src/mux21.vhd
vcom -93 ../src/ram.vhd
vcom -93 ../src/treatment.vhd
vcom -93 treatment_tb.vhd

vsim treatment_tb

view signals

add wave *

run -all
