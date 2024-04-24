

LIBRARY ieee;
USE ieee.std_logic_1164.all; 



ENTITY TP1_HeartBeat is
	PORT
	(
		CLOCK_50 	:  IN  STD_LOGIC;
		KEY			 	:  IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		SW 				:  IN  STD_LOGIC_VECTOR(9 DOWNTO 0);
		LEDR 			:  OUT  STD_LOGIC_VECTOR(9 downto 0)
	);
END entity;

ARCHITECTURE RTL OF TP1_HeartBeat IS 

	signal 	rst,clk : std_logic;
	signal Tick10us: std_logic;

BEGIN 

rst <= not KEY(0);
clk <= CLOCK_50; 
LEDR(9 downto 1) <= SW(9 downto 1);

inst: entity work.FDIV port map(CLK => clk, RST => rst, Tick10us => Tick10us);
inst1: entity work.HEARTBEAT port map (CLK => clk, RST => rst, TICK10US => Tick10us, LED => LEDR(0));

END architecture;