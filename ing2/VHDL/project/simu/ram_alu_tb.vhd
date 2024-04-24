library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RAM_ALU_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of RAM_ALU_TB is
    signal ra, rb, rw : std_logic_vector(3 downto 0) := (others => '0');
    signal w, rama, ramb : std_logic_vector(31 downto 0) := (others => '0');
    signal clk, rst, we : std_logic := '0';
    signal op : std_logic_vector(2 downto 0) := "000";
    signal alua, alub, result : std_logic_vector(31 downto 0) := (others => '0');
    signal n, z, c, v : std_logic := '0';
begin
    process
        variable va, vb : signed(31 downto 0);
    begin
        ok <= TRUE;

        rst <= '1';
        wait for 5 NS;
        rst <= '0';
        wait for 5 NS;

-- - R(1) = R(15)
-- - R(1) = R(1) + R(15)
-- - R(2) = R(1) + R(15)
-- - R(3) = R(1) - R(15)
-- - R(5) = R(7) - R(15)

        -- -- -- -- -- -- --


        op <= "011";
        ra <= X"F";
        rb <= X"1";
        rw <= X"1";
        wait for 1 NS;
        we <= '1';
        w <= rama;
        report "R(1) = R(15)" severity NOTE;
        report "rama = " & integer'image(to_integer(signed(rama))) severity NOTE;
        report "w = " & integer'image(to_integer(signed(w))) severity NOTE;
        clk <= '1';

        wait for 5 NS;

        if rama /= X"00000030" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(1) = R(15) failed" severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(1) = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --

        ra <= X"F";
        rb <= X"1";
        rw <= X"1";

        op <= "000";
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;
        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;

        if rama /= X"00000030" or ramb /= X"00000060" then
            OK <= FALSE;
        end if;

        clk <= '0';

        wait for 5 NS;




        wait for 5 NS;
        wait;
    end process;

    UUT1: entity work.RAM(RTL)  port map (
        clk => clk,
        reset => rst,
        ra => ra,
        rb => rb,
        rw => rw,
        we => we,
        w => w,
        a => rama,
        b => ramb
    );

    UUT2: entity work.ALU(RTL)  port map (
        op => op,
        a => alua,
        b => alub,
        s => result,
        n => n,
        z => z,
        c => c,
        v => v
    );
end architecture;
