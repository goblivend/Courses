library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder is
  port (
    Instr, psr_regs : in std_logic_vector(31 downto 0) := (others => '0');
    nPCSel, RegWr, AluSrc, PSREn, MemWr, WrSrc, RegSel, RegAff : out std_logic := '0';
    ALUCtr : out std_logic_vector(2 downto 0) := (others => '0');

    Rd, Rn, Rm : out Std_logic_vector(3 downto 0) := (others => '0');
    Imm8: out Std_logic_vector(7 downto 0):= (others => '0');
    Imm24: out Std_logic_vector(23 downto 0):= (others => '0')
  ) ;
end decoder ;

architecture rtl of decoder is
  type instructions is (MOV, ADDi, ADDr, CMP, LDR, STR, BAL, BLT);
  signal curr: instructions;
begin
  process(Instr)
    begin
    if instr(27 downto 26) = "00" then
      if instr(24 downto 21) = "0100" then
        if instr(25) = '1' then
          curr <= ADDi;
        else
          curr <= ADDr;
        end if;
      elsif instr(24 downto 21) = "1010" then
        curr <= CMP;
      elsif instr(24 downto 21) = "1101" then
        curr <= MOV;
      end if;
      Imm8 <= Instr(7 downto 0);
      rn <= Instr(19 downto 16);
      rd <= Instr(15 downto 12);
      rm <= Instr(3 downto 0);
    elsif instr(27 downto 26) = "01" then
      if instr(25) = '1' then
        curr <= LDR;
      else
        curr <= STR;
      end if;
    elsif instr(27 downto 25) = "101" then
      if instr(31 downto 28) = "1011" then
        curr <= BLT;
      else
        curr <= BAL;
      end if;
      Imm24 <= Instr(23 downto 0);
    end if;
  end process;

  process(curr)
  begin
    case curr is
      when ADDi =>
        nPCsel <= '0' ;
        RegWr  <= '1' ;
        ALUSrc <= '1' ;
        ALUCtr <= "000";
        PSREn  <= '1' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when ADDr =>
        nPCsel <= '0';
        RegWr  <= '1';
        ALUSrc <= '0' ;
        ALUCtr <= "000";
        PSREn  <= '1' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when BAL =>
        nPCsel <= '1' ;
        RegWr  <= '0' ;
        ALUSrc <= '0' ;
        ALUCtr <= "---";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when BLT =>
        nPCsel <= '1' ;
        RegWr  <= '0' ;
        ALUSrc <= '0' ;
        ALUCtr <= "---";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when CMP =>
        nPCsel <= '0' ;
        RegWr  <= '0' ;
        ALUSrc <= '0' ;
        ALUCtr <= "010";
        PSREn  <= '1' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ; -- ??????????????
        RegAff <= '0' ;
      when LDR =>
        nPCsel <= '0' ;
        RegWr  <= '1' ;
        ALUSrc <= '1' ;
        ALUCtr <= "000";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '1' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when MOV =>
        nPCsel <= '0';
        RegWr  <= '1';
        ALUSrc <= '0';
        ALUCtr <= "001";
        PSREn  <= '0';
        MemWr  <= '0';
        WrSrc  <= '0';
        RegSel <= '0';
        RegAff <= '0';
      when STR =>
        nPCsel <= '0';
        RegWr  <= '0';
        ALUSrc <= '1';
        ALUCtr <= "000";
        PSREn  <= '0';
        MemWr  <= '1';
        WrSrc  <= '0';
        RegSel <= '0';
        RegAff <= '1';


    end case;
  end process;

end architecture ; -- rtl
