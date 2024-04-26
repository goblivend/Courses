library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity EXTEN_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of EXTEN_TB is
    signal s1, s2 : std_logic_vector(31 downto 0) := (others => '0');
    signal e1 : std_logic_vector(7 downto 0) := (others => '0');
    signal e2 : std_logic_vector(15 downto 0) := (others => '0');
begin
    process
    begin
        ok <= TRUE;
        wait for 1 NS;

        -- -- -- -- -- -- --

        e1 <= x"FF";
        wait for 1 NS;

        if s1 /= x"FFFFFFFF" then
            ok <= FALSE;
            report "Test 1 : extending 8 bits negative to 32 failed";
        end if;

        -- -- -- -- -- -- --

        e2 <= x"FFFF";
        wait for 1 NS;

        if s2 /= x"FFFFFFFF" then
            ok <= FALSE;
            report "Test 2 : extending 16 bits negative to 32 failed";
        end if;

        -- -- -- -- -- -- --

        e1 <= x"00";
        wait for 1 NS;

        if s1 /= x"00000000" then
            ok <= FALSE;
            report "Test 3 : extending 8 bits positive to 32 failed";
        end if;

        -- -- -- -- -- -- --

        e2 <= x"0000";
        wait for 1 NS;

        if s2 /= x"00000000" then
            ok <= FALSE;
            report "Test 4 : extending 16 bits positive to 32 failed";
        end if;

        -- -- -- -- -- -- --

        e1 <= x"7F";
        wait for 1 NS;

        if s1 /= x"0000007F" then
            ok <= FALSE;
            report "Test 5 : extending 8 bits positive to 32 failed";
        end if;

        -- -- -- -- -- -- --

        e2 <= x"7FFF";
        wait for 1 NS;

        if s2 /= x"00007FFF" then
            ok <= FALSE;
            report "Test 6 : extending 16 bits positive to 32 failed";
        end if;

        wait for 5 NS;
        wait;
    end process;

    UUT1: entity work.exten(RTL) generic map ( n => 8 ) port map (
        s => s1,
        e => e1
    );

    UUT2: entity work.exten(RTL) generic map ( n => 16 ) port map (
        s => s2,
        e => e2
    );
end architecture;
