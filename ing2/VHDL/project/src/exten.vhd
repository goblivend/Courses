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
    process (e, tmp) is
        -- variable s : string(1 to 32) := "                                ";
        -- variable se : string(1 to n) := "        ";
    begin
        for i in 0 to 31 loop
            if i < e'length then
                -- s := s(2 to 32) & std_logic'image(e(i))(2);
                -- se := se(2 to n) & std_logic'image(e(i))(2);
                tmp(i) <= e(i);
            else
                -- s := s(2 to 32) & std_logic'image(e(e'length-1))(2);
                tmp(i) <= e(e'length-1);
            end if;
        end loop ; -- extend
        -- report "tmp = " & integer'image(to_integer(signed(tmp))) & ", e = " & integer'image(to_integer(signed(e))) & ", s = " & s & ", se = " & se;
    end process ; -- extend
    s <= tmp;

end rtl ; -- arch
