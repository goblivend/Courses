library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ALU_TB is
    port(OK: out BOOLEAN);
end;

architecture TB of ALU_TB is
    signal op : std_logic_vector(2 downto 0) := "000";
    signal a, b, result : std_logic_vector(31 downto 0) := (others => '0');
    signal n, z, c, v : std_logic := '0';
begin
    process
        variable va, vb : signed(31 downto 0);
    begin
        op <= "000";
        ok <= TRUE;

        -- -- -- -- -- -- --

        va := to_signed(0, 32);
        vb := to_signed(0, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(0, 32) or n /= '0' or z /= '1' or c /= '0' or v /= '0'  then
            OK <= FALSE;
        end if;

        -- -- -- -- -- -- --

        va := to_signed(5, 32);
        vb := to_signed(0, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(5, 32) or n /= '0' or z /= '0' or c /= '0' or v /= '0'  then
            OK <= FALSE;
        end if;

        -- -- -- -- -- -- --

        va := to_signed(-5, 32);
        vb := to_signed(0, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(-5, 32)  or n /= '1' or z /= '0' or c /= '0' or v /= '0' then
            OK <= FALSE;
        end if;

        -- -- -- -- -- -- --

        va := to_signed(-15, 32);
        vb := to_signed(15, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(0, 32) or n /= '0' or z /= '1' or c /= '1' or v /= '0' then
            OK <= FALSE;
        end if;

        -- -- -- -- -- -- --

        va := to_signed(integer'high, 32);
        vb := to_signed(1, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(integer'low, 32) or n /= '1' or z /= '0' or c /= '0' or v /= '1' then
            OK <= FALSE;
            report "found" & integer'image(to_integer(signed(result))) & " " & integer'image(integer'low);
        end if;


        -- -- -- -- -- -- --

        op <= "010";
        va := to_signed(18, 32);
        vb := to_signed(-48, 32);
        a <= std_logic_vector(va);
        b <= std_logic_vector(vb);

        wait for 5 NS;

        if signed(result) /= to_signed(66, 32) or n /= '0' or z /= '0' or c /= '1' or v /= '0' then
            OK <= FALSE;
            report "found" & integer'image(to_integer(signed(result))) ;
        end if;




        wait for 5 NS;
        wait;
    end process;

    UUT: entity work.ALU(RTL)  port map (
        op => op,
        a => a,
        b => b,
        s => result,
        n => n,
        z => z,
        c => c,
        v => v
        );

end architecture;
