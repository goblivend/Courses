LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity MUX81 IS
	port(
		I 	: in std_logic_vector(7 downto 0);
		Y 	: out std_logic;
		SEL: in std_logic_vector(2 downto 0));
END MUX81 ;


architecture RTL of MUX81 is
begin
process(I, SEL)
begin
	case sel is
		when "000" => y <= I(0);
		when "001" => y <= I(1);
		when "010" => y <= I(2);
		when "011" => y <= I(3);
		when "100" => y <= I(4);
		when "101" => y <= I(5);
		when "110" => y <= I(6);
		when "111" => y <= I(7);
		when others => y <= '-';
	end case;
	end process;
end architecture;
