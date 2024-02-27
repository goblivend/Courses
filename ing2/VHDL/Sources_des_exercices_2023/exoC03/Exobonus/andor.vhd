library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity ANDOR is
    port (
        A,B,C : in std_logic;
        F : out std_logic);
end entity;

Architecture dataflow of ANDOR is
Begin
    F <= (A and B) or C;
End architecture;
