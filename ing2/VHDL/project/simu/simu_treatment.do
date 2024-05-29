vlib work

vcom -2008 ../src/alu.vhd
vcom -2008 ../src/exten.vhd
vcom -2008 ../src/mem.vhd
vcom -2008 ../src/mux21.vhd
vcom -2008 ../src/ram.vhd
vcom -2008 ../src/treatment.vhd
vcom -2008 treatment_tb.vhd

vsim treatment_tb

view signals

add wave -radix hexadecimal *
add wave -radix hexadecimal /treatment_tb/proc/reg_bench
add wave -radix hexadecimal /treatment_tb/proc/mem_bench

run -all
