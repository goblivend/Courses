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
        alias pc : std_logic_vector(31 downto 0) is <<signal.uut.instrs1.pc : std_logic_vector(31 downto 0)>>;
        type reg_table is array(15 downto 0) of std_logic_vector(31 downto 0);
        type data_table is array(63 downto 0) of std_logic_vector(31 downto 0);
        alias reg : reg_table is <<signal.uut.treatment1.register1.Banc : reg_table>>;
        alias data : data_table is <<signal.uut.treatment1.mem1.Banc : data_table>>;
        type instructions is (NOP, MOV, ADDi, ADDr, CMP, LDR, STR, BAL, BLT);
        alias op : instructions is <<signal.uut.decoder1.curr : instructions>>;

    begin
        ok <= TRUE;

        wait for 5 ns;
        rst <= '1';
        wait for 5 ns;
        rst <= '0';
        wait for 10 ns;

        for i in 0 to 52 loop -- 52 instructions
            clk <= '1';
            wait for 5 ns;
            clk <= '0';
            wait for 5 ns;
        end loop;


        assert pc = x"00000008" report "Error in test: pc/= x8 : => " & integer'image(to_integer(signed(pc))) severity error;
        assert op = BAL report "Error in test: op/= BAL : => " & instructions'image(op) severity error;
        assert data(to_integer(unsigned(reg(1)))) = x"0000000A" report "Error in test: data(1) /= xA : => " & integer'image(to_integer(unsigned(data(to_integer(unsigned(reg(1)))))) ) severity error;

        wait;
    end process;

    UUT: entity work.PROC(RTL)  port map (
        clk => clk,
        rst => rst,
        reg_aff => reg_aff
    );
end architecture;
