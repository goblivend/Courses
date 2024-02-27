LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;


entity ADDC4_TB IS

END ADDC4_TB ;


architecture BENCH of ADDC4_TB is


	signal I1, I2 : std_logic_vector(3 downto 0);
	signal CIN    :  std_logic;
	signal SUM    :  std_logic_vector(3 downto 0);
--	signal verifsum, verifboucle    :  unsigned(4 downto 0);
	signal COUT   :  std_logic;
	signal somme  :  integer;
	
begin
	process
	begin
		-- si on veut tester uniquement deux cas et vérifier que la somme
		-- et la retenue sont bonnes
		--CIN <= '1';
		--I1<="0110";
		--I2<="1011";
		--assert SUM="0010" report "Error on SUM" severity warning;
		--assert COUT='1' report "Error on COUT" severity warning;
		--wait for 50 ns;
		
		--CIN <= '0';
		--I1<="0111";
		--I2<="0011";
		--assert SUM="1010" report "Error on SUM" severity warning;
		--assert COUT='0' report "Error on COUT" severity warning;
		--wait for 50 ns;
		
		
		--pour tester si les sorties sont bonnes avec tous les cas pour les deux entrées
				
		CIN <= '0';
		for j in 0 to 15 loop
			I1 <= std_logic_vector(to_unsigned(j,4));
			for k in 0 to 15 loop
				I2 <= std_logic_vector(to_unsigned(k,4));
				somme<=j+k;
				wait for 10 ns;
				assert unsigned(COUT&SUM)=to_unsigned(somme,5) report "Error on SUM " severity warning; 

			end loop;
		end loop;
		
		CIN <= '1';
		for j in 0 to 15 loop
			I1 <= std_logic_vector(to_unsigned(j,4));
			for k in 0 to 15 loop
				I2 <= std_logic_vector(to_unsigned(k,4));
				somme<=j+k+1;
				wait for 10 ns;
				assert unsigned(COUT&SUM)=to_unsigned(somme,5) report "Error on SUM " severity warning; 
			end loop;
		end loop;
			
		report "End of test. Verify that no error was reported.";
		wait;
		
	end process;
	
UUT : entity work.ADDC4
port map ( I1=> I1, I2=> I2, CIN=>CIN, 
           SUM => SUM, COUT=> COUT);
		
end BENCH;