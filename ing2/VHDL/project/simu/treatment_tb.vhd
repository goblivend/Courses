library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity TREATMENT_TB is
end;

architecture TB of TREATMENT_TB is
    signal clk, reset, we_register, we_mem, sel_register, sel_mem, n, c, v ,z: std_logic := '0';
    signal op: std_logic_vector(2 downto 0) := (others => '0');
    signal add_reg_a, add_reg_b, add_reg_w : std_logic_vector(3 downto 0) := (others => '0');
    signal imm: std_logic_vector(7 downto 0) := (others => '0');
begin
    process
    begin
        clk <= '0';
        reset <= '1';
        wait for 1 NS;
        reset <= '0';

        clk <= '1';
        wait for 1 NS;
        report "Reset done, clock High";
        -- -- -- -- -- -- --

        add_reg_a <= x"0";
        add_reg_b <= x"F";
        add_reg_w <= x"0";
        sel_register <= '0';
        op <= "000";
        sel_mem <= '0';

        we_mem <= '0';
        we_register <= '1';

        wait for 5 NS;
        clk <= '0';
        report "About to do first test reg[0] = reg[0] + reg[F]";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done first test reg[0] = reg[0] + reg[F] = 30"; -- [0] = 30

        -- -- -- -- -- -- --

        add_reg_a <= x"0";
        add_reg_b <= x"0";
        add_reg_w <= x"1";
        sel_register <= '1';
        op <= "000";
        imm <= x"12";
        sel_mem <= '0';

        we_mem <= '0';
        we_register <= '1';

        wait for 4 NS;
        clk <= '0';
        report "About to do second test reg[1] = reg[0] + x'12' = 42";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done second test reg[1] = reg[0] + x'12' = 42"; -- [1] = 42

        -- -- -- -- -- -- --

        add_reg_a <= x"1";
        add_reg_b <= x"0";
        add_reg_w <= x"2";
        sel_register <= '0';
        op <= "010";
        sel_mem <= '0';

        we_mem <= '0';
        we_register <= '1';

        wait for 4 NS;
        clk <= '0';
        report "About to do third test reg[2] = reg[1] - reg[0] = 12";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done third test reg[2] = reg[1] - reg[0] = 12"; -- [2] = 12

        -- -- -- -- -- -- --

        add_reg_a <= x"2";
        add_reg_b <= x"0";
        add_reg_w <= x"3";
        sel_register <= '0';
        op <= "010";
        imm <= x"D0";
        sel_mem <= '0';

        we_mem <= '0';
        we_register <= '1';

        wait for 4 NS;
        clk <= '0';
        report "About to do fourth test reg[3] = reg[2] - x'D0' = 42";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done fourth test reg[3] = reg[2] - x'D0' = 42"; -- [3] = 42

        -- -- -- -- -- -- --

        add_reg_a <= x"3";
        add_reg_b <= x"0";
        add_reg_w <= x"4";
        sel_register <= '1';
        op <= "011";
        sel_mem <= '0';

        we_mem <= '0';
        we_register <= '1';

        wait for 4 NS;
        clk <= '0';
        report "About to do fifth test reg[4] = reg[3] = 42";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done fifth test reg[4] = reg[3] = 42"; -- [4] = 42

        -- -- -- -- -- -- --

        add_reg_a <= x"2";
        add_reg_b <= x"4";
        add_reg_w <= x"F";
        sel_register <= '0';
        op <= "011";
        sel_mem <= '0';

        we_mem <= '1';
        we_register <= '0';

        wait for 4 NS;
        clk <= '0';
        report "About to do sixth test mem[reg[2]] = reg[4] => mem[33] = 42";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done sixth test mem[reg[2]] = reg[4] = 42"; -- mem[33] = 42

        -- -- -- -- -- -- --

        add_reg_a <= x"0";
        add_reg_b <= x"2";
        add_reg_w <= x"6";
        sel_register <= '0';
        op <= "001";
        sel_mem <= '1';

        we_mem <= '0';
        we_register <= '1';

        wait for 4 NS;
        clk <= '0';
        report "About to do seventh test reg[6] = mem[reg[2]] => reg[6] = 42";
        wait for 5 NS;
        clk <= '1';
        wait for 1 NS;
        report "Done seventh test reg[6] = mem[reg[2]] = 42"; -- reg[6] = 42

        -- -- -- -- -- -- --




        wait for 3 NS;
        wait;
    end process;

    uut : entity work.TREATMENT
        port map (
            clk => clk,
            reset => reset,
            we_register => we_register,
            we_mem => we_mem,
            sel_register => sel_register,
            sel_mem => sel_mem,
            op => op,
            n => n,
            z => z,
            c => c,
            v => v,
            add_reg_a => add_reg_a,
            add_reg_b => add_reg_b,
            add_reg_w => add_reg_w,
            exten_in => imm
        );
end architecture;
