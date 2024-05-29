library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity treatment is
    port (
        clk : in std_logic;
        rst : in std_logic;
        aff_out1, aff_out2, aff_out3, aff_out4 : out std_logic_vector(1 to 7)
    );
end entity treatment;

architecture rtl of treatment is
    signal reg_aff : std_logic_vector(31 downto 0);
begin
    aff1 : entity work.seven_seg
        port map (
            data => reg_aff(3 downto 0)
            pol => '1',
            Segout => aff_out1
        );
    aff2 : entity work.seven_seg
        port map (
            data => reg_aff(7 downto 4)
            pol => '1',
            Segout => aff_out2
        );
    aff3 : entity work.seven_seg
        port map (
            data => reg_aff(11 downto 8)
            pol => '1',
            Segout => aff_out3
        );
    aff4 : entity work.seven_seg
        port map (
            data => reg_aff(15 downto 12)
            pol => '1',
            Segout => aff_out4
        );

    proc : entity work.proc
        port map (
            clk => clk,
            rst => rst,
            reg_aff => reg_aff
        );

end architecture rtl ; -- arch
