LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity MUX41 IS
	port (
	I		: in std_logic_vector(3 downto 0);
    SEL	 	: in std_logic_vector(1 downto 0);
    Y  		: out std_logic);
END entity ;


architecture BEHAVIOUR of MUX41 is
begin
    with SEL select
    Y <= I(0) when "00",
         I(1) when "01",
         I(2) when "10",
         I(3) when others;


end architecture;
