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
    signal res_vect : std_logic_vector(32 downto 0);
begin
    process(a, b, op)
        variable a2, b2: std_logic_vector(32 downto 0);
    begin
        a2 := signed("0" & a);
        b2 := signed("0" & b);
        case op is
            when "000" =>
                res <= a2 + b2;
                if a(31) = b(31) and a(31) /= res(32) then
                    v <= '1';
                    report "Overflow : " & integer'image(to_integer(a2)) & ", " & integer'image(to_integer(b2)) & ", " & integer'image(to_integer(signed(res))) severity warning;
                else
                    v <= '0';
                end if;
            when "001" =>
                    res <= b2;
                    v <= '0';
            when "010" =>
                res <= a2 - b2;
                if a(31) /= b(31) and a(31) /= res(32) then
                    v <= '1';
                    report "Overflow : " & integer'image(to_integer(a2)) & ", " & integer'image(to_integer(b2)) & ", " & integer'image(to_integer(signed(res))) severity warning;
                else
                    v <= '0';
                end if;
            when "011" =>
                    res <= a2;
                    v <= '0';
            when others =>
                report "Unknown OPCODE" & std_logic'image(op(2)) & std_logic'image(op(1)) & std_logic'image(op(2)) severity error;
                res <= (others => 'X');
        end case;

        for i in 0 to 32 loop
            s2 := std_logic'image(res(i))(2) & s2(1 to 64);
        end loop ;
        report "Result : " & s2 severity note;
        s2 := (others => ' ');
    end process;
    res_vect <= std_logic_vector(res);
    c <= res_vect(32);
    z <= '1' when res_vect(31 downto 0) = std_logic_vector(to_signed(0, 32)) else '0';
    n <= res_vect(31);
    s <= res_vect(31 downto 0);
end architecture rtl;
