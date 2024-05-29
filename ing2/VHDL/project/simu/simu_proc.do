vlib work

vcom -2008 ../src/exten.vhd
vcom -2008 ../src/mux21.vhd
vcom -2008 ../src/alu.vhd
vcom -2008 ../src/reg.vhd
vcom -2008 ../src/instr_mem.vhd
vcom -2008 ../src/instrs.vhd
vcom -2008 ../src/decoder.vhd
vcom -2008 ../src/ram.vhd
vcom -2008 ../src/mem.vhd
vcom -2008 ../src/treatment.vhd
vcom -2008 ../src/proc.vhd
vcom -2008 proc_tb.vhd

vsim proc_tb

view signals

add wave -radix hexadecimal *
add wave -radix hexadecimal /proc_tb/procs/ttt

run -all
