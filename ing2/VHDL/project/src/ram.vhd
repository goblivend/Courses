library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ram is
    port (
        clk : in std_logic;
        reset : in std_logic;
        ra, rb, rw : in std_logic_vector(3 downto 0);
        we : in std_logic;
        w : in std_logic_vector(31 downto 0);
        a, b : out std_logic_vector(31 downto 0)
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
        result(15) := X"00000030";
        return result;
    end init_banc;
        -- Déclaration et Initialisation du Banc de Registres 16x32 bits
    signal Banc: table:=init_banc;
begin
    process(clk, reset)
    begin
        if (reset = '1') then
            banc <= init_banc;
        elsif rising_edge(clk) then
            report "Clock Rising Edge" severity note;
            if (we = '1') then
                banc(to_integer(unsigned(rw))) <= w;
                report "Ecriture dans le registre " & integer'image(to_integer(unsigned(rw))) severity note;
            end if;
        end if ;
    end process ; -- identifier
    a <= banc(to_integer(unsigned(ra)));
    b <= banc(to_integer(unsigned(rb)));
end rtl ; -- arch
