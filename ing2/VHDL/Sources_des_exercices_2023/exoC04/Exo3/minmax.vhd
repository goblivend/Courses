----------- Squelette de l'exercice MinMAx --------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity MinMax is
  port (Min0Max1: in  STD_LOGIC;
        X, Y    : in  STD_LOGIC_VECTOR (3 downto 0);
        Z       : out STD_LOGIC_VECTOR (3 downto 0));
end entity MinMax;

architecture RTL of MinMax is
begin
  process(Min0Max1, X, Y)
  begin
    if Min0Max1 = '0' then
      if X < Y then
        Z <= X;
      else
        Z <= Y;
      end if;
    else
      if X > Y then
        Z <= X;
      else
        Z <= Y;
      end if;
    end if;
  end process;
end architecture RTL;
