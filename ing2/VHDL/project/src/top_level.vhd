library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ARM7TDMMI is
    port (
        CLOCK_50 : in std_logic;
        KEY : in std_logic_vector(1 downto 0);
        SW : in std_logic_vector(9 downto 0);
        HEX0, HEX1, HEX2, HEX3 : out std_logic_vector(0 to 6)
    );
end entity ARM7TDMMI;

architecture rtl of ARM7TDMMI is
    signal reg_aff : std_logic_vector(31 downto 0);
    signal rst, pol : std_logic;
    signal cnt : integer;
begin
    rst <= not key(0);
	pol <= SW(9);

    aff1 : entity work.seven_seg
        port map (
            data => reg_aff(3 downto 0),
            pol => pol,
            Segout => HEX0
        );
    aff2 : entity work.seven_seg
        port map (
            data => reg_aff(7 downto 4),
            pol => pol,
            Segout => HEX1
        );
    aff3 : entity work.seven_seg
        port map (
            data => reg_aff(11 downto 8),
            pol => pol,
            Segout => HEX2
        );
    aff4 : entity work.seven_seg
        port map (
            data => reg_aff(15 downto 12),
            pol => pol,
            Segout => HEX3
        );

    proc : entity work.proc
        port map (
            clk => CLOCK_50,
            rst => rst,
            reg_aff => reg_aff
        );

end architecture rtl ; -- arch
