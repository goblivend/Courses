-- Squelette de l'exercice serialor

library IEEE;
use IEEE.Std_logic_1164.all;

entity SerialOR is
  port (Clock,rst, D	: in Std_logic;
        Q		 : out Std_logic);
end entity;

architecture RTL of SerialOR is
--d�claration des signaux
  signal tmp1, tmp2, tmp3, tmp4 : std_logic;
begin

--instructions concurrente
  Q <= tmp1 or tmp2 or tmp3 or tmp4;

  process(clock,rst)
  begin
    if rst = '1' then
      tmp1 <= '0'; tmp2 <= '0'; tmp3 <= '0'; tmp4 <= '0';
    elsif rising_edge(clock) then
      tmp1 <= D;
      tmp2 <= tmp1;
      tmp3 <= tmp2;
      tmp4 <= tmp3;
    end if;
    --instructions s�quentielle

  end process;

end architecture RTL;


architecture RTL2 of SerialOR is
--d�claration des signaux
  signal tmp1, tmp2, tmp3, tmp4 : std_logic;
begin

--instructions concurrente
  Q <= tmp1 or tmp2 or tmp3 or tmp4;

  process(clock,rst)
  begin
    if rst = '1' then
      tmp1 <= '0'; tmp2 <= '0'; tmp3 <= '0'; tmp4 <= '0';
    elsif rising_edge(clock) then
      tmp1 <= D and not(tmp4);
      tmp2 <= tmp1;
      tmp3 <= tmp2;
      tmp4 <= tmp3;
    end if;
    --instructions s�quentielle

  end process;

end architecture RTL2;
