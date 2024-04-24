

LIBRARY ieee;
USE ieee.std_logic_1164.all; 



ENTITY TP1_Chenillard is
	PORT
	(
		CLOCK_50 	:  IN  STD_LOGIC;
		KEY			 	:  IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		SW 				:  IN  STD_LOGIC_VECTOR(9 DOWNTO 0);
		LEDR 			:  OUT  STD_LOGIC_VECTOR(9 downto 0)
	);
END entity;

ARCHITECTURE RTL OF TP1_Chenillard IS 

	signal 	rst,clk : std_logic;
	signal Tick10ms: std_logic;

BEGIN 

rst <= not KEY(0);
clk <= CLOCK_50; 

inst: entity work.FDIV port map(CLK => clk, RST => rst, Tick10ms => Tick10ms);
inst1: entity work.CHENILLARD port map (CLK => clk, RST => rst, Tick10ms => Tick10ms, LED => LEDR);

END architecture;