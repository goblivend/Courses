library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity alu is
    port(
        op: in std_logic_vector(2 downto 0);
        a, b: in std_logic_vector(31 downto 0);
        s: out std_logic_vector(31 downto 0);
        n, z, c, v: out std_logic
    );
end entity alu;

architecture rtl of alu is
    signal res_vect : std_logic_vector(32 downto 0) := (others => '0');
    signal res : signed(32 downto 0) := to_signed(0, 33);
begin
    process(a, b, op)
    begin
        case op is
            when "000" =>
                res <= signed('0' & a) + signed('0' & b);
                s <= std_logic_vector(signed(a) + signed(b));
                if a(31) = b(31) and a(31) /= res(32) then
                    v <= '1';
                    report "Overflow : " & std_logic'image(a(31)) & ", " & std_logic'image(b(31)) & ", " & std_logic'image(res(31)) severity warning;
                else
                    v <= '0';
                end if;
            when "001" =>
                res <= signed('0' & b);
                s <= b;
                v <= '0';
            when "010" =>
                res <= signed('0' & a) - signed('0' & b);
                s <= std_logic_vector(signed(a) - signed(b));
                if a(31) /= b(31) and a(31) /= res(32) then
                    v <= '1';
                    report "Overflow : " & integer'image(to_integer(signed('0' & a))) & ", " & integer'image(to_integer(signed('0' & b))) & ", " & integer'image(to_integer(signed(res))) severity warning;
                else
                    v <= '0';
                end if;
            when "011" =>
                    res <= signed('0' & a);
                    s <= a;
                    v <= '0';
            when others =>
                report "Unknown OPCODE" & std_logic'image(op(2)) & std_logic'image(op(1)) & std_logic'image(op(0)) severity error;
                res <= (others => 'X');
            end case ;
    end process;
    res_vect <= std_logic_vector(res);
    c <= res_vect(32);
    z <= '1' when res_vect(31 downto 0) = std_logic_vector(to_signed(0, 32)) else '0';
    n <= res_vect(31);
    -- s <= res_vect(31 downto 0);

end architecture rtl;
