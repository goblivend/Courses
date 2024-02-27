-- Squelette de l'exercice serialor

library IEEE;
use IEEE.Std_logic_1164.all;

entity SerialOR is
  port (Clock,rst, D	: in Std_logic;
        Q		 : out Std_logic);
end entity serialOR;

architecture RTL of SerialOR is
--déclaration des signaux
begin

--instructions concurrente

  process(clock,rst) 
  begin

    --instructions séquentielle
    
  end process;
  
end architecture RTL;

