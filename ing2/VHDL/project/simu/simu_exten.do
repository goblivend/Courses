vlib work

vcom -93 ../src/exten.vhd
vcom -93 exten_tb.vhd

vsim exten_tb

view signals

add wave *

run -all
