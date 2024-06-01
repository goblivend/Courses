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
#add wave -radix hexadecimal {sim:/proc_tb/uut/decoder1/instr}
add wave -radix hexadecimal {sim:/proc_tb/uut/instrs1/pc}
#add wave -radix hexadecimal {sim:/proc_tb/uut/instrs1/offset}
#add wave -radix hexadecimal {sim:/proc_tb/uut/instrs1/offset_out}
add wave                    {sim:/proc_tb/uut/decoder1/curr}
add wave                    {sim:/proc_tb/uut/decoder1/cond}
#add wave                    {sim:/proc_tb/uut/decoder1/RegAff}
#add wave -radix hexadecimal {sim:/proc_tb/uut/decoder1/Imm8}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/exten_out}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/sel_register}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/alu1/b}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/rb}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/b}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/ra}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/a}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/rw}
#add wave -radix hexadecimal {sim:/proc_tb/uut/rd}
#add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/w}
add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/register1/Banc}
add wave -radix hexadecimal {sim:/proc_tb/uut/treatment1/mem1/Banc}


run -all
