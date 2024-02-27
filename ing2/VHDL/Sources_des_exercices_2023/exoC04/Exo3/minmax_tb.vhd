----------- Squelette du Banc de Test pour l'exercice MinMax -------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity MinMax_TB is
 port( OK: out boolean:=TRUE);
end entity MinMax_TB;

architecture BENCH of MinMax_TB is

  
  signal Tmin0max1: std_logic;
  signal TX,TY,TZ : std_logic_vector(3 downto 0);
 
begin
  
  stimulus:process
  begin
        
    
  wait;
  end process;

  
end architecture BENCH;
