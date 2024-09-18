library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity decoder is
  port (
    Instr  : in std_logic_vector(31 downto 0);
    psr_regs : in std_logic_vector(31 downto 0) := (others => '0');
    nPCSel, RegWr, AluSrc, PSREn, MemWr, WrSrc, RegSel, RegAff : out std_logic := '0';
    ALUCtr : out std_logic_vector(2 downto 0) := (others => '0');

    Rd, Rn, Rm : out Std_logic_vector(3 downto 0) := (others => '0');
    Imm8: out Std_logic_vector(7 downto 0):= (others => '0');
    Imm24: out Std_logic_vector(23 downto 0):= (others => '0')
  ) ;
end decoder ;

architecture rtl of decoder is
  type instructions is (NOP, MOV, ADDi, ADDr, CMP, LDR, STR, BAL, BLT);
  signal curr: instructions := NOP;
  type condition is (EQ, NE, CS, CC, MI, PL, VS, VC, HI, LS, GE, LT, GT, LE, AL);
  signal cond: condition := AL;
  signal cond_bin: std_logic_vector(3 downto 0) := "0000";
begin
  Imm8 <= Instr(7 downto 0);
  rn <= Instr(19 downto 16);
  rd <= Instr(15 downto 12);
  rm <= Instr(3 downto 0);

  process(Instr)
  begin
    cond_bin <= Instr(31 downto 28);
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
      else
        report "Unknown instruction line 35";
      end if;
    elsif instr(27 downto 26) = "01" then
      if instr(20) = '1' then
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
    else
      report "Unknown instruction line 55";
    end if;
  end process;

  process(curr, psr_regs)
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
        ALUCtr <= "000";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when BLT =>
        nPCsel <= psr_regs(31);
        RegWr  <= '0' ;
        ALUSrc <= '1' ;
        ALUCtr <= "000";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when CMP =>
        nPCsel <= '0' ;
        RegWr  <= '0' ;
        ALUSrc <= '1' ;
        ALUCtr <= "010";
        PSREn  <= '1' ;
        MemWr  <= '0' ;
        WrSrc  <= '0' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when LDR =>
        nPCsel <= '0' ;
        RegWr  <= '1' ;
        ALUSrc <= '1' ;
        ALUCtr <= "011";
        PSREn  <= '0' ;
        MemWr  <= '0' ;
        WrSrc  <= '1' ;
        RegSel <= '0' ;
        RegAff <= '0' ;
      when MOV =>
        nPCsel <= '0';
        RegWr  <= '1';
        ALUSrc <= '1';
        ALUCtr <= "001";
        PSREn  <= '0';
        MemWr  <= '0';
        WrSrc  <= '0';
        RegSel <= '0';
        RegAff <= '0';
      when STR =>
        nPCsel <= '0';
        RegWr  <= '1';
        ALUSrc <= '1';
        ALUCtr <= "011";
        PSREn  <= '0';
        MemWr  <= '1';
        WrSrc  <= '1';
        RegSel <= '1';
        RegAff <= '1';
      when NOP =>
        nPCsel <= '0';
        RegWr  <= '0';
        ALUSrc <= '0';
        ALUCtr <= "000";
        PSREn  <= '0';
        MemWr  <= '0';
        WrSrc  <= '0';
        RegSel <= '0';
        RegAff <= '0';
      when others =>
        report "Unknown instruction line";

    end case;
  end process;

  with cond_bin select
    cond <= EQ when "0000",
            NE when "0001",
            CS when "0010",
            CC when "0011",
            MI when "0100",
            PL when "0101",
            VS when "0110",
            VC when "0111",
            HI when "1000",
            LS when "1001",
            GE when "1010",
            LT when "1011",
            GT when "1100",
            LE when "1101",
            AL when "1110",
            AL when others;


end architecture ; -- rtl
