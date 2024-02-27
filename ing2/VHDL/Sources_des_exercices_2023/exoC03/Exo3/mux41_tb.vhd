LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity MUX41_TB IS

END MUX41_TB ;


architecture BENCH of MUX41_TB is


	signal I : std_logic_vector(3 downto 0);
	signal SEL :  std_logic_vector(1 downto 0);
	signal Y :  std_logic;
	
begin
	process
	begin
		SEL <= "00";
		for j in 0 to 15 loop
			I <= std_logic_vector(to_unsigned(j,4));
			assert Y=I(0) report "Error on I(0)" severity warning; 
			wait for 10 ns;
		end loop;
		
		SEL <= "01";
		for j in 0 to 15 loop
			I <= std_logic_vector(to_unsigned(j,4));
			assert Y=I(1) report "Error on I(1)" severity warning;
			wait for 10 ns;
		end loop;
		
		SEL <= "10";
		for j in 0 to 15 loop
			I <= std_logic_vector(to_unsigned(j,4));
			assert Y=I(2) report "Error on I(2)" severity warning;
			wait for 10 ns;
		end loop;
		
		SEL <= "11";
		for j in 0 to 15 loop
			I <= std_logic_vector(to_unsigned(j,4));
			assert Y=I(3) report "Error on I(3)" severity warning;
			wait for 10 ns;
		end loop;
		
		
		report "End of test. Verify that no error was reported.";
		wait;
		
	end process;
	
UUT : entity work.MUX41
port map ( I      => I,
           SEL      => SEL,
           Y  => Y);
		
end BENCH;