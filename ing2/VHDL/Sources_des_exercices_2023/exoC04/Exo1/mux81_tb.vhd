LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity MUX81_TB IS

END MUX81_TB ;


architecture BENCH of MUX81_TB is


	signal I : std_logic_vector(7 downto 0);
	signal SEL :  std_logic_vector(2 downto 0);
	signal Y :  std_logic;
	signal OK: BOOLEAN;
	
begin
	process
	begin
		OK <= TRUE;
		SEL <= "000";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(0) report "Error on I(0)" severity warning; 
			if Y /= I(0) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "001";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(1) report "Error on I(1)" severity warning;
			if Y /= I(1) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "010";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(2) report "Error on I(2)" severity warning;
			if Y /= I(2) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "011";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(3) report "Error on I(3)" severity warning;
			if Y /= I(3) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "100";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(4) report "Error on I(4)" severity warning; 
			if Y /= I(4) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "101";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(5) report "Error on I(5)" severity warning; 
			if Y /= I(5) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "110";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(6) report "Error on I(6)" severity warning; 
			if Y /= I(6) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		SEL <= "111";
		for j in 0 to 255 loop
			I <= std_logic_vector(to_unsigned(j,8));
			assert Y=I(7) report "Error on I(7)" severity warning; 
			if Y /= I(7) then
      			OK <= FALSE;
    		end if;
			wait for 10 ns;
		end loop;
		
		report "End of test. Verify that no error was reported.";
		wait;
		
	end process;
	
UUT : entity work.MUX81
port map ( I      => I,
           SEL      => SEL,
           Y  => Y);
		
end BENCH;