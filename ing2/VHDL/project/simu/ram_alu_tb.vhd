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


        -- -- -- -- -- -- --
        -- - R(1) = R(15)

        op <= "011";
        ra <= X"F";
        rb <= X"1";
        rw <= X"1";
        wait for 1 NS;
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;

        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;
        we <= '0';

        if rama /= X"00000030" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(1)  = R(15) failed" severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(1)  = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --
        -- - R(1) = R(1) + R(15)

        ra <= X"1";
        rb <= X"F";
        rw <= X"1";
        wait for 1 NS;

        op <= "000";
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;
        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;
        we <= '0';

        if rama /= X"00000060" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(1)  = R(1) + R(15) failed" severity ERROR;
            report "R(1)  = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --
        -- - R(2) = R(1) + R(15)

        ra <= X"1";
        rb <= X"F";
        rw <= X"2";
        wait for 1 NS;

        op <= "000";
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;
        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;
        we <= '0';

        if rama /= X"00000060" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(2)  = R(1) + R(15) failed" severity ERROR;
            report "R(1)  = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        ra <= X"2";
        wait for 1 NS;
        if rama /= X"00000090" then
            OK <= FALSE;
            report "R(2) = R(1) + R(15) failed" severity ERROR;
            report "R(2) = " & integer'image(to_integer(signed(rama))) severity ERROR;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --
        -- - R(3) = R(1) - R(15)

        ra <= X"1";
        rb <= X"F";
        rw <= X"3";
        wait for 1 NS;

        op <= "010";
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;
        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;
        we <= '0';

        if rama /= X"00000060" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(3)  = R(1) - R(15) failed" severity ERROR;
            report "R(1)  = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        ra <= X"3";
        wait for 1 NS;
        if rama /= X"00000030" then
            OK <= FALSE;
            report "R(3) = R(1) - R(15) failed" severity ERROR;
            report "R(3) = " & integer'image(to_integer(signed(rama))) severity ERROR;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --
        -- - R(5) = R(7) - R(15)


        ra <= X"7";
        rb <= X"F";
        rw <= X"5";

        wait for 1 NS;

        op <= "010";
        alua <= rama;
        alub <= ramb;
        wait for 1 NS;
        we <= '1';
        w <= result;

        clk <= '1';

        wait for 5 NS;
        we <= '0';

        if rama /= X"00000000" or ramb /= X"00000030" then
            OK <= FALSE;
            report "R(5)  = R(7) - R(15) failed" severity ERROR;
            report "R(7)  = " & integer'image(to_integer(signed(rama))) severity ERROR;
            report "R(15) = " & integer'image(to_integer(signed(ramb))) severity ERROR;
        end if;

        ra <= X"5";
        wait for 1 NS;
        if rama /= X"FFFFFFD0" then
            report integer'image(to_integer(signed(rama)));
            OK <= FALSE;
            report "R(5) = R(7) - R(15) failed" severity ERROR;
            report "R(5) = " & integer'image(to_integer(signed(rama))) severity ERROR;
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
