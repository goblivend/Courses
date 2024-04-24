vlib work

vcom -93 ramchip.vhd
vcom -93 ramchip_tb.vhd

vsim ramchip_tb(Bench)

view signals
add wave *

run -all
