-- Squelette pour l'exercice carte m�moire

library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.numeric_std.all;

entity Board is
  generic (WordSize: integer := 16; AddressSize: integer := 8; ChipAddressSize: integer := 4);
  port (Address: in Std_logic_vector(AddressSize-1 downto 0);
        Data: inout Std_logic_vector(WordSize-1 downto 0);
        nWE, nOE: in Std_logic);
end entity Board;

architecture Struct of Board is
  signal cs : Std_logic_vector(2**ChipAddressSize-1 downto 0);
begin
  process(Address(AddressSize-1 downto AddressSize-ChipAddressSize))
  begin
    cs <= (others => '1');
    cs(to_integer(unsigned(Address(AddressSize-1 downto AddressSize-ChipAddressSize)))) <= '0';
  end process;

  Chips : for i in 0 to 2**ChipAddressSize-1 generate
    Chip : entity work.RamChip generic map (WordSize => WordSize, AddressSize => AddressSize-ChipAddressSize)
      port map (Address => Address(ChipAddressSize-1 downto 0),
                Data => Data,
                nWE => nWE,
                nOE => nOE,
                nCS => cs(i));
  end generate;

end architecture Struct;
