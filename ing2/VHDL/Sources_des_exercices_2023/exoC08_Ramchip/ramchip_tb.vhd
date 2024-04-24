-- Squelette pour l'exercice RamChip

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;


entity RamChip_tb is
end entity;

architecture Bench of RamChip_tb is
    signal nCS, nOE, nWE : std_logic := '1';
    signal address : std_logic_vector(3 downto 0);
    signal data : std_logic_vector(7 downto 0);
begin


    UUUT : entity work.RamChip
        port map(
            nWE => nWE,
            nCS => nCS,
            nOE => nOE,
            address => address,
            data => data
        );

-- Process for writing and reading data
    process
    begin

        nWE <= '1';
        nCS <= '1';
        nOE <= '1';
        address <= "0000";
        data <= (others => 'Z');

        -- Write on 0
        -- Setup
        wait for 5 ns;
        data <= "01010101"; -- x"55"
        address <= "0000";
        nCS <= '0';

        wait for 5 ns;
        nWE <= '0';

        -- Hold
        wait for 5 ns;
        nWE <= '1';

        wait for 5 ns;
        nCS <= '1';
        data <= (others => 'Z');
        address <= "1001";


        -- Write on 9
        -- Setup
        wait for 5 ns;
        data <= "11110000"; -- x"F0"
        address <= "1001";
        nCS <= '0';

        wait for 5 ns;
        nWE <= '0';

        -- Hold
        wait for 5 ns;
        nWE <= '1';

        wait for 5 ns;
        nCS <= '1';
        data <= (others => 'Z');


        -- Read on 0
        wait for 5 ns;
        address <= "0000";
        nCS <= '0';

        wait for 5 ns;
        nOE <= '0';

        wait for 5 ns;
        assert data = "01010101" report "Error on read 0" severity error;
        nOE <= '1';

        wait for 5 ns;
        nCS <= '1';



        -- Read on 9
        wait for 5 ns;
        address <= "1001";
        nCS <= '0';

        wait for 5 ns;
        nOE <= '0';

        wait for 5 ns;
        assert data = "11110000" report "Error on read 9" severity error;
        nOE <= '1';

        wait for 5 ns;
        nCS <= '1';

        wait;
    end process;





end architecture;
