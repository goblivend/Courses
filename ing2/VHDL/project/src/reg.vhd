library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reg is
  port (
    clk, rst, we : in std_logic := '0';
    pc_in : in std_logic_vector(31 downto 0) := (others => '0');
    pc_out : out std_logic_vector(31 downto 0) := (others => '0')
  ) ;
end reg ;

architecture rtl of reg is
begin
    process(clk, rst)
    begin
        if rst = '1' then
            pc_out <= (others => '0') ;
        elsif rising_edge(clk) then
            if we = '1' then
                pc_out <= pc_in ;
            end if ;
        end if ;
    end process ;
end architecture ; -- rtl
