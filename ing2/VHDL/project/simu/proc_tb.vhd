library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity PROC_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of PROC_TB is
    signal reg_aff : std_logic_vector(31 downto 0) := (others => '0');
    signal clk, rst : std_logic := '0';
begin

    procs : process
        alias ttt : std_logic_vector(31 downto 0) is <<signal.uut.treatment1.register_a : std_logic_vector(31 downto 0)>>;
    begin
        ok <= TRUE;

        wait for 10 ns;
        rst <= '1';
        wait for 10 ns;
        rst <= '0';
        wait for 10 ns;

        for i in 0 to 8 loop
            clk <= '1';
            wait for 10 ns;
            clk <= '0';
            wait for 10 ns;
        end loop;

        wait;
    end process;

    UUT: entity work.PROC(RTL)  port map (
        clk => clk,
        rst => rst,
        reg_aff => reg_aff
    );
end architecture;
