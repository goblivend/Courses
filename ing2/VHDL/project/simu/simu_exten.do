vlib work

vcom -2008 ../src/exten.vhd
vcom -2008 exten_tb.vhd

vsim exten_tb

view signals

add wave *

run -all
