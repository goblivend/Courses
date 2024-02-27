-- Squelette pour l'exercice Compteur
library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.Numeric_std.all;

entity Counter is
  port (Clock, Reset, Enable, Load, UpDn: in Std_logic;
        Data: in Std_logic_vector(7 downto 0);
        Q:   out Std_logic_vector(7 downto 0));
end entity Counter;

-- Première partie
architecture RTL of Counter is
begin

end architecture;

-- Deuxième partie
architecture RTL2 of Counter is
begin

end architecture;

-- Troisième partie
architecture RingCounter of Counter is
begin

end architecture;