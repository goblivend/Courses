-- Banc de Test pour l'exercice décodeur

entity Decodeur_tb is
port( OK: out BOOLEAN := TRUE);
end entity;

library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.Numeric_std.all;

architecture BENCH of Decodeur_tb is

  
  signal I : Std_logic_vector(5 downto 0);
  signal EN: Std_logic;
  signal Y : Std_logic_vector(63 downto 0);

begin

  Stimulus: process
    constant Zero: Std_logic_vector := 
                   Std_logic_vector(To_unsigned(0, 64));
    variable Reg : Std_logic_vector(63 downto 0) := 
                   Std_logic_vector(To_unsigned(1, 64));
  begin
    EN <= '0';
    for J in 0 to 63 loop
      I <= Std_logic_vector(To_unsigned(J, 6));
      wait for 10 NS;
      if Y /= Zero then
        OK <= FALSE;
      end if;
    end loop;

    EN <= '1';
    for J in 0 to 63 loop
      I <= Std_logic_vector(To_unsigned(J, 6));
      wait for 10 NS;
      if Y /= Reg then
        OK <= FALSE;
      end if;
      Reg := Reg(62 downto 0) & '0';
    end loop;

    wait;
  end process;

  D: entity work.decodeur port map (I => I, EN => EN, Y => Y);

end architecture;

