library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity proc is
  port (
    clk, rst : in std_logic ;
    reg_aff : out std_logic_vector(31 downto 0)
  );
end proc ;

architecture rtl of proc is
    signal psr_regs, psr_regs_out, instr : std_logic_vector(31 downto 0) := (others => '0');
    signal nPCSel, RegWr, AluSrc, PSREn, MemWr, WrSrc, RegSel, RegAff : std_logic := '0';
    signal ALUCtr : std_logic_vector(2 downto 0)      := (others => '0');
    signal Rd, Rn, Rm, mux_rb : std_logic_vector(3 downto 0)  := (others => '0');
    signal imm8 : std_logic_vector(7 downto 0)        := (others => '0');
    signal imm24 : std_logic_vector(23 downto 0)      := (others => '0');
    signal curr_reg_aff : std_logic_vector(31 downto 0) := (others => '0');
    signal tmp_reg_aff : std_logic_vector(31 downto 0)  := (others => '0');
begin
    mux21Rb: entity work.mux21 generic map (
        n => 4
    ) port map (
        com => RegSel,
        a => rm,
        b => rd,
        s => mux_rb
    ) ;

    psr_reg : entity work.reg(rtl) port map (
        clk => clk,
        rst => rst,
        we => PSREn,
        pc_in => psr_regs,
        pc_out => psr_regs_out
    ) ;

    treatment1 : entity work.treatment(rtl) port map (
        clk => clk,
        rst => rst,
        we_register => RegWr,
        we_mem => MemWr,
        sel_register => ALUSrc,
        sel_mem => WrSrc,
        op => ALUCtr,
        add_reg_w => rd,
        add_reg_a => rn,
        add_reg_b => mux_rb,
        exten_in => imm8,
        n => psr_regs(31),
        c => psr_regs(30),
        z => psr_regs(29),
        v => psr_regs(28),
        reg_aff => tmp_reg_aff
    ) ;

    instrs1 : entity work.instrs port map (
        clk => clk,
        rst => rst,
        nPCSel => nPCSel,
        instr => instr,
        offset => imm24
    ) ;

    decoder1 : entity work.decoder port map (
        instr => instr,
        psr_regs => psr_regs_out,
        nPCSel => nPCSel,
        RegWr => RegWr,
        ALUSrc => ALUSrc,
        PSREn => PSREn,
        MemWr => MemWr,
        WrSrc => WrSrc,
        RegSel => RegSel,
        RegAff => RegAff,
        ALUCtr => ALUCtr,
        Rd => Rd,
        Rn => Rn,
        Rm => Rm,
        Imm8 => Imm8,
        Imm24 => Imm24
    ) ;

    curr_reg_aff <= tmp_reg_aff when RegAff = '1' else curr_reg_aff;
    reg_aff <= curr_reg_aff;
end architecture ; -- rtl
