library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MUX21_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of MUX21_TB is
    signal s, a, b : std_logic_vector(31 downto 0) := (others => '0');
    signal com : std_logic := '0';
begin
    process
    begin
        ok <= TRUE;
        wait for 1 NS;

        -- -- -- -- -- -- --

        a <= x"FFFFFFFF";
        b <= x"00000000";
        com <= '0';

        wait for 1 NS;

        if s /= x"FFFFFFFF" then
            ok <= FALSE;
            report "Test 1 failed" severity error;
        end if;

        wait for 5 NS;

        -- -- -- -- -- -- --

        a <= x"00000000";
        b <= x"FFFFFFFF";
        com <= '1';

        wait for 1 NS;

        if s /= x"FFFFFFFF" then
            ok <= FALSE;
            report "Test 2 failed" severity error;
        end if;

        wait for 5 NS;
        wait;
    end process;

    UUT: entity work.mux21(RTL)  port map (
        s => s,
        a => a,
        b => b,
        com => com
    );
end architecture;
