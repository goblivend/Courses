library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mux21 is
    generic (
        n : integer := 32
        );
    port (
        com : std_logic := '0';
        a, b : in std_logic_vector(n-1 downto 0) := (others=>'0');
        s : out std_logic_vector(n-1 downto 0) := (others=>'0')
        );
end entity mux21;

architecture rtl of mux21 is
begin
    s <= a when com = '0' else b;
end rtl ; -- arch
