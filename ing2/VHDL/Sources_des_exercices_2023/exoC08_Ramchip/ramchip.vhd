-- Squelette pour l'exercice RamChip

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity RamChip is
  port (Address: in Std_logic_vector(3 downto 0);
        Data: inout Std_logic_vector(7 downto 0);
        nCS, nWE, nOE: in Std_logic);
end entity;

architecture Behaviour of RamChip is
   
begin
  

end architecture;
