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
  minmax: entity work.minmax port map (Min0Max1 => TMin0max1, X => TX, Y => ty, Z => TZ);

  stimulus:process
  begin
    OK <= TRUE;

    Tmin0max1 <= '0';
    TX <= "0000";
    TY <= "0000";
    wait for 5 ns;
    if TZ /= "0000" then
      OK <= False;
    end if;

    TX <= "0001";
    TY <= "0000";
    wait for 5 ns;
    if TZ /= "0000" then
      OK <= False;
    end if;

    TX <= "0001";
    TY <= "0010";
    wait for 5 ns;
    if TZ /= "0001" then
      OK <= False;
    end if;

    TX <= "0101";
    TY <= "0010";
    wait for 5 ns;
    if TZ /= "0010" then
      OK <= False;
    end if;

    TX <= "0101";
    TY <= "1010";
    wait for 5 ns;
    if TZ /= "0101" then
      OK <= False;
    end if;

    Tmin0max1 <= '1';

    TX <= "0000";
    TY <= "0000";
    wait for 5 ns;
    if TZ /= "0000" then
      OK <= False;
    end if;

    TX <= "0001";
    TY <= "0000";
    wait for 5 ns;
    if TZ /= "0001" then
      OK <= False;
    end if;

    TX <= "0001";
    TY <= "0010";
    wait for 5 ns;
    if TZ /= "0010" then
      OK <= False;
    end if;

    TX <= "0101";
    TY <= "0010";
    wait for 5 ns;
    if TZ /= "0101" then
      OK <= False;
    end if;

    TX <= "0101";
    TY <= "1010";
    wait for 5 ns;
    if TZ /= "1010" then
      OK <= False;
    end if;

  wait;
  end process;


end architecture BENCH;
