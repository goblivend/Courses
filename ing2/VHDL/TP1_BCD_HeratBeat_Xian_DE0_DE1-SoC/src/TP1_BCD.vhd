

LIBRARY ieee;
USE ieee.std_logic_1164.all; 



ENTITY TP1_BCD is
	PORT
	(
		CLOCK_50 	:  IN  STD_LOGIC;
		KEY			 	:  IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		SW 				:  IN  STD_LOGIC_VECTOR(9 DOWNTO 0);
		HEX0 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX1 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX2 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX3 			:  OUT  STD_LOGIC_VECTOR(0 TO 6)
	);
END entity;

ARCHITECTURE RTL OF TP1_BCD IS 

	signal 	rst,clk, pol  : std_logic;
	signal Tick1ms: std_logic;
	signal UNITIES, TENS, HUNDREDS, THOUSNDS: STD_LOGIC_VECTOR(3 downto 0);

BEGIN 
	rst <= not KEY(0);
	clk <= CLOCK_50; 
	pol <= SW(9);

	inst: entity work.FDIV port map(CLK => clk, RST => rst, Tick1ms => Tick1ms);
	inst3: entity work.BCD_COUNTER port map(CLK => clk, RST => rst, Tick1ms => Tick1ms, SW => SW(7 downto 0), UNITIES => UNITIES, TENS => TENS, HUNDREDS => HUNDREDS, THOUSNDS => THOUSNDS);

	inst4: entity work.SEVEN_SEG port map(DATA => UNITIES, Pol => pol, Segout => HEX0);
	inst5: entity work.SEVEN_SEG port map(DATA => TENS, Pol => pol, Segout => HEX1);
	inst6: entity work.SEVEN_SEG port map(DATA => HUNDREDS, Pol => pol, Segout => HEX2);
	inst7: entity work.SEVEN_SEG port map(DATA => THOUSNDS, Pol => pol, Segout => HEX3);
END architecture;