
vlib work

vcom -93 decodeur.vhd
vcom -93 decodeur_tb.vhd

vsim decodeur_tb(bench)

add wave *

run -all