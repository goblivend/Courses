library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity exten is
    generic (
        n : integer
        );
    port (
        e : in std_logic_vector(n-1 downto 0) := (others=>'0');
        s : out std_logic_vector(31 downto 0) := (others=>'0')
    );
end entity exten;

architecture rtl of exten is
    signal tmp : std_logic_vector(31 downto 0) := (others=>'0');
begin
    process (e) is
    begin
        for i in 0 to 31 loop
            if i < e'length then
                tmp(i) <= e(i);
            else
                tmp(i) <= e(e'length-1);
            end if;
        end loop ; -- extend
    end process ; -- extend
    s <= tmp;
end rtl ; -- arch
