vlib work

vcom -93 ramchip.vhd
vcom -93 board.vhd
vcom -93 board_tb.vhd

vsim board_tb(Bench)

view signals
add wave *

run -all
