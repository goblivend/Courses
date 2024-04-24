-- Squelette pour l'exercice carte m�moire

library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.numeric_std.all;

entity Board is
  port (Address: in Std_logic_vector(7 downto 0);
        Data: inout Std_logic_vector(15 downto 0);
        nWE, nOE: in Std_logic);
end entity Board;

architecture Struct of Board is
  signal cs : Std_logic_vector(15 downto 0);
begin
  process(Address(7 downto 4))
  begin
    cs <= (others => '1');
    cs(to_integer(unsigned(Address(7 downto 4)))) <= '0';
  end process;

  Chips : for i in 0 to 15 generate
    Chip : entity work.RamChip generic map (WordSize => 16, AddressSize => 4)
      port map (Address => Address(3 downto 0),
                Data => Data,
                nWE => nWE,
                nOE => nOE,
                nCS => cs(i));
  end generate;

end architecture Struct;
