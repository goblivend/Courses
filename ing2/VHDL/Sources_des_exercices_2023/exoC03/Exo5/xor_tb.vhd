LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY XOR_TB IS
END ENTITY XOR_TB;

--
ARCHITECTURE BENCH OF XOR_TB IS
  signal A,B,S1 : STD_LOGIC;
BEGIN
  process
    begin
      A <= '0';
      B <= '0';
      wait for 50 NS;
      A <= '0';
      B <= '1';
      wait for 50 NS;
      A <= '1';
      B <= '0';
      wait for 50 NS;
      A <= '1';
      B <= '1';
      wait for 50 NS;
      wait;
  end process;
  
  UUT1: entity WORK.XOR2(struct) port map (A=> A, B => B, S=>S1 );    
END ARCHITECTURE BENCH;

