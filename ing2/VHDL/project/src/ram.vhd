library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ram is
    port (
        clk : in std_logic := '0';
        rst : in std_logic := '0';
        ra, rb, rw : in std_logic_vector(3 downto 0) := (others=>'0');
        we : in std_logic := '0';
        w : in std_logic_vector(31 downto 0) := (others=>'0');
        a, b : out std_logic_vector(31 downto 0) := (others=>'0')
        );
end entity ram;

architecture rtl of ram is
    -- Declaration Type Tableau Memoire
    type table is array(15 downto 0) of std_logic_vector(31 downto 0);
    -- Fonction d'Initialisation du Banc de Registres
    function init_banc return table is
        variable result : table;
    begin
        for i in 14 downto 0 loop
            result(i) := (others=>'0');
        end loop;
        result(15) := X"00000000";
        return result;
    end init_banc;
        -- Déclaration et Initialisation du Banc de Registres 16x32 bits
    signal Banc: table:=init_banc;
begin
    process(clk, rst)
    begin
        if (rst = '1') then
            banc <= init_banc;
        elsif rising_edge(clk) then
            if (we = '1') then
                banc(to_integer(unsigned(rw))) <= w;
            end if;
        end if ;
    end process ; -- identifier
    a <= banc(to_integer(unsigned(ra)));
    b <= banc(to_integer(unsigned(rb)));
end rtl ; -- arch
