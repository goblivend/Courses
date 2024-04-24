-- Squelette pour l'exercice sur les machines � �tats

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity FSM is
	port (	Clk :				in 	STD_LOGIC;
			nRst: 				in 	STD_LOGIC;
			Start_Stop, Clear:	in 	STD_LOGIC;
			Cnt_En, Cnt_Rst: 	out 	STD_LOGIC);
end entity;

architecture RTL of FSm is
	type State_Type is (Zero, Start, Running, Stp, Stopped, Reset);
	signal State: State_Type;
begin
	process (Clk, nRst)
	begin
		if nRst = '0' then
			Cnt_en <= '0';
			Cnt_Rst <= '0';
			State <= Zero;
		elsif rising_edge(Clk) then
			case State is
				when Zero =>
					if Start_Stop = '1' then
						State <= Start;
						Cnt_En <= '1';
					end if;
				when Start =>
					if Start_Stop = '0' then
						State <= Running;
					end if;
				when Running =>
					if Start_Stop = '1' then
						State <= Stp;
						Cnt_en <= '0';
					end if;
				when Stp =>
					if Start_Stop = '0' then
						State <= Stopped;
					end if;
				when Stopped =>
					if Start_Stop = '1' then
						State <= Start;
						Cnt_en <= '1';
					elsif Clear = '1' then
						State <= Reset;
						Cnt_rst <= '1';
					end if;
				when Reset =>
					if Clear = '0' then
						State <= Zero;
						Cnt_rst <= '0';
					end if;
			end case;
		end if;
	end process;


end architecture;
