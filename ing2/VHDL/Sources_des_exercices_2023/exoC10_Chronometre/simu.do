vlib work

vcom -93 fsm.vhd
vcom -93 fsmtb.vhd

vsim fsmtb

view signals
add wave *

run -all
