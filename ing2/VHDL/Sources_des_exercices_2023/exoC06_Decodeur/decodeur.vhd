-- Squelette pour l'exercice Decodeur

library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.numeric_std.all;

entity decodeur is
  port (I : in  Std_logic_vector(5 downto 0);
        EN: in  Std_logic;
        Y : out Std_logic_vector(63 downto 0));
end entity;

architecture rtl of decodeur is
begin
  
end architecture rtl;
