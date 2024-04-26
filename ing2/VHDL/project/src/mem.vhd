library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem is
    port (
        clk : in std_logic;
        reset : in std_logic;
        addr : in std_logic_vector(5 downto 0);
        datain : in std_logic_vector(31 downto 0);
        dataout : out std_logic_vector(31 downto 0);
        we : in std_logic
    );
end entity mem;

architecture rtl of mem is
    -- Declaration Type Tableau Memoire
    type table is array(63 downto 0) of std_logic_vector(31 downto 0);
    -- Fonction d'Initialisation du Banc de Registres
    function init_banc return table is
        variable result : table;
    begin
        for i in 63 downto 0 loop
            result(i) := (others=>'0');
        end loop;
        return result;
    end init_banc;
        -- Déclaration et Initialisation du Banc de Memoire 64x32 bits
    signal Banc: table:=init_banc;
begin
    process(clk, reset)
    begin
        if (reset = '1') then
            banc <= init_banc;
        elsif rising_edge(clk) then
            if (we = '1') then
                banc(to_integer(unsigned(addr))) <= datain;
                report "Ecriture dans la memoire " & integer'image(to_integer(unsigned(addr))) & " = " & integer'image(to_integer(unsigned(datain))) severity note;
            end if;
        end if ;
    end process ; -- identifier
    dataout <= banc(to_integer(unsigned(addr)));
end rtl ; -- arch
