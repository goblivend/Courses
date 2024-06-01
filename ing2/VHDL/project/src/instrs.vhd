library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity instrs is
  port (
    clk, rst : in std_logic := '0';
    nPCsel : in std_logic := '0';
    offset : in std_logic_vector(23 downto 0) := (others => '0');
    instr : out std_logic_vector(31 downto 0)
  );
end instrs ;

architecture rlt of instrs is
    signal pc, offset_out : std_logic_vector(31 downto 0) := (others => '0');
begin
    process(clk, rst)
        variable v_pc : integer := 42;
    begin
        if rst = '1' then
            pc <= (others => '0');
        elsif rising_edge(clk) then
            if signed(pc) >= 63 then
                pc <= (others => '0');
            elsif nPCsel = '1' then
                pc <= std_logic_vector(signed(pc) + to_signed(1, 32) + signed(offset_out));
            else
                pc <= std_logic_vector(signed(pc) + to_signed(1, 32));
            end if;
        end if;
    end process;

    exten : entity work.exten generic map (
            n => 24
        )
        port map (
            e => offset,
            s => offset_out
    ) ;

    instr_mem : entity work.instr_mem port map (
        PC => pc,
        instruction => instr
    ) ;

end architecture ; -- rtl
