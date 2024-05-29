library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity treatment is
    port (
        clk : in std_logic := '0';
        rst : in std_logic := '0';
        we_register : in std_logic := '0';
        we_mem : in std_logic := '0';
        sel_register : in std_logic := '0';
        sel_mem : in std_logic := '0';
        op : in std_logic_vector(2 downto 0) := (others => '0');
        add_reg_w, add_reg_a, add_reg_b : in std_logic_vector(3 downto 0) := (others => '0');
        exten_in : in std_logic_vector(7 downto 0) := (others => '0');
        n, z, c, v: out std_logic := '0';
        reg_aff: out std_logic_vector(31 downto 0) := (others => '0')
    );
end entity treatment;

architecture rtl of treatment is
    signal register_a, register_b, mux_register, alu_out, mux_alu_mem, exten_out, dataout_mem : std_logic_vector(31 downto 0) := (others => '0');
begin
    reg_aff <= register_b;
    mux1 : entity work.mux21
        port map (
            a => register_b,
            b => exten_out,
            com => sel_register,
            s => mux_register
    );

    mux2 : entity work.mux21
        port map (
            a => alu_out,
            b => dataout_mem,
            com => sel_mem,
            s => mux_alu_mem
    );

    register1 : entity work.ram
        port map (
            clk => clk,
            rst => rst,
            we => we_register,
            rw => add_reg_w,
            ra => add_reg_a,
            rb => add_reg_b,
            w => mux_alu_mem,
            a => register_a,
            b => register_b
    );

    alu1 : entity work.alu
        port map (
            op => op,
            a => register_a,
            b => mux_register,
            s => alu_out,
            n => n,
            z => z,
            c => c,
            v => v
    );

    mem1 : entity work.mem
        port map (
            clk => clk,
            rst => rst,
            addr => alu_out(5 downto 0),
            datain => register_b,
            dataout => dataout_mem,
            we => we_mem
            );

    exten1 : entity work.exten
        generic map (
            n => 8
        )
        port map (
            e => exten_in,
            s => exten_out
    );


end architecture rtl ; -- arch
