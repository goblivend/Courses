library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RAM_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of RAM_TB is
    signal ra, rb, rw : std_logic_vector(3 downto 0) := (others => '0');
    signal w, a, b : std_logic_vector(31 downto 0) := (others => '0');
    signal clk, rst, we : std_logic := '0';
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

        clk <= '1';

        wait for 5 NS;

        if a /= X"00000000" or b /= X"00000000" then
            OK <= FALSE;
        end if;

        clk <= '0';

        wait for 5 NS;

        -- -- -- -- -- -- --

        ra <= X"F";
        rb <= X"0";

        clk <= '1';

        wait for 5 NS;

        if a /= X"00000030" or b /= X"00000000" then
            OK <= FALSE;
        end if;

        clk <= '0';
        wait for 5 NS;

        -- -- -- -- -- -- --

        we <= '1';
        w <= X"00000042";

        ra <= X"0";
        rb <= X"F";

        clk <= '1';

        wait for 5 NS;

        if a /= X"00000042" or b /= X"00000030" then
            OK <= FALSE;
        end if;

        clk <= '0';
        wait for 5 NS;

        -- -- -- -- -- -- --

        we <= '0';
        w <= std_logic_vector(signed(a) + signed(b));

        ra <= X"0";
        rb <= X"F";

        clk <= '1';

        wait for 5 NS;

        if a /= X"00000042" or b /= X"00000030" then
            OK <= FALSE;
            report "a = " & integer'image(to_integer(signed(a))) severity error;
            report "b = " & integer'image(to_integer(signed(b))) severity error;
        end if;

        clk <= '0';
        wait for 5 NS;

        -- -- -- -- -- -- --

        we <= '1';
        w <= std_logic_vector(signed(a) + signed(b));

        ra <= X"0";
        rb <= X"F";

        clk <= '1';

        wait for 5 NS;

        if a /= X"00000072" or b /= X"00000030" then
            OK <= FALSE;
            report "a = " & integer'image(to_integer(signed(a))) severity error;
            report "b = " & integer'image(to_integer(signed(b))) severity error;
        end if;

        clk <= '0';
        wait for 5 NS;



        wait for 5 NS;
        wait;
    end process;

    UUT: entity work.RAM(RTL)  port map (
        clk => clk,
        reset => rst,
        ra => ra,
        rb => rb,
        rw => rw,
        we => we,
        w => w,
        a => a,
        b => b
    );
end architecture;
