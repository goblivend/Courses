LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity ADDC4 is
port (
			I1, I2: in std_logic_vector(3 downto 0);
			CIN : in std_logic;
			SUM  : out std_logic_vector(3 downto 0);
			COUT : out  std_logic );
end entity ADDC4;



architecture STRUCT of ADDC4 is
	signal COUT1, COUT2, COUT3 : std_logic;
begin
	A1: entity work.addc1 port map (A => I1(0), B => I2(0), Cin => CIN,   Sum => SUM(0), Cout => COUT1);
	A2: entity work.addc1 port map (A => I1(1), B => I2(1), Cin => COUT1, Sum => SUM(1), Cout => COUT2);
	A3: entity work.addc1 port map (A => I1(2), B => I2(2), Cin => COUT2, Sum => SUM(2), Cout => COUT3);
	A4: entity work.addc1 port map (A => I1(3), B => I2(3), Cin => COUT3, Sum => SUM(3), Cout => COUT);


end architecture STRUCT;
