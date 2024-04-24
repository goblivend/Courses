

LIBRARY ieee;
USE ieee.std_logic_1164.all; 



ENTITY TP1_pingpong is
	PORT
	(
		CLOCK_50 	:  IN  STD_LOGIC;
		KEY			 	:  IN  STD_LOGIC_VECTOR(1 DOWNTO 0);
		SW 				:  IN  STD_LOGIC_VECTOR(9 DOWNTO 0);
		LEDR 			:  OUT  STD_LOGIC_VECTOR(9 downto 0);
        HEX0 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX1 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX2 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX3 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
        HEX4 			:  OUT  STD_LOGIC_VECTOR(0 TO 6);
		HEX5 			:  OUT  STD_LOGIC_VECTOR(0 TO 6)
	);
END entity;

ARCHITECTURE RTL OF TP1_pingpong IS 

	signal rst,clk : std_logic;
	signal Tick10ms: std_logic;
    signal POINTAUO, POINTATO, POINTBUO, POINTBTO, POINTM1O, POINTM2O: STD_LOGIC_VECTOR(4 downto 0);

BEGIN 

rst <= SW(0);
clk <= CLOCK_50;

fdiv: entity work.FDIV port map(CLK => clk, RST => rst, Tick10ms => Tick10ms);


pingpong: entity work.PINGPONG port map (
        CLK => clk,
        RST => rst,
        KEYA => KEY(0),
        KEYB => KEY(1),
        Tick10ms => Tick10ms,
        LED => LEDR,
        POINTAU => POINTAUO,
        POINTAT => POINTATO,
        POINTBU => POINTBUO,
        POINTBT => POINTBTO,
        POINTM1 => POINTM1O,
        POINTM2 => POINTM2O
    );

pointAU: entity work.SEVEN_SEG_P port map(DATA => POINTAUO, Segout => HEX0);
pointAT: entity work.SEVEN_SEG_P port map(DATA => POINTATO, Segout => HEX1);
pointBU: entity work.SEVEN_SEG_P port map(DATA => POINTBUO, Segout => HEX4);
pointBT: entity work.SEVEN_SEG_P port map(DATA => POINTBTO, Segout => HEX5);
pointM1: entity work.SEVEN_SEG_P port map(DATA => POINTM1O, Segout => HEX2);
pointM2: entity work.SEVEN_SEG_P port map(DATA => POINTM2O, Segout => HEX3);

END architecture;