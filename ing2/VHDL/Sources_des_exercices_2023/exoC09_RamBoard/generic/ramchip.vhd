-- Squelette pour l'exercice RamChip

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity RamChip is
generic(AddressSize: positive := 4; WordSize: positive := 8);
  port (Address: in Std_logic_vector(AddressSize-1 downto 0);
        Data: inout Std_logic_vector(WordSize-1 downto 0);
        nCS, nWE, nOE: in Std_logic);
end entity;

architecture Behaviour of RamChip is
  type RamType is array (0 to 2**Address'LENGTH-1) of Std_logic_vector(Data'RANGE);
  signal RAM : RamType;
begin
  process (nCS, nWE, nOE, Address, Data )
  begin
    data <= (others => 'Z');
    if nCS = '0' then
      if nWE = '0' then
        RAM(to_integer(unsigned(Address))) <= Data;
      elsif nOE = '0' then
        Data <= RAM(to_integer(unsigned(Address)));
      end if;
    end if;
  end process;
end architecture;
