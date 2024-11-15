-- SevenSeg.vhd

-- Notes :
--  * We don't ask for an hexadecimal decoder, only 0..9
--  * outputs are active high if Pol='1'
--    else active low (Pol='0')
--  * Order is : Segout(1)=Seg_A, ... Segout(7)=Seg_G
--
--  * Display Layout :
--
--       A=Seg(1)
--      -----
--    F|     |B=Seg(2)
--     |  G  |
--      -----
--     |     |C=Seg(3)
--    E|     |
--      -----
--        D=Seg(4)


library IEEE;
  use IEEE.std_logic_1164.all;
  use IEEE.numeric_std.all;

Entity seven_seg is
  port ( Data   : in  std_logic_vector(3 downto 0) := "0000";      -- Expected within 0 .. F
         Pol    : in  std_logic := '0';                            -- '0' if active LOW
         Segout : out std_logic_vector(1 to 7) := (others => '0')  -- Segments A, B, C, D, E, F, G
  );
end entity seven_seg;

Architecture COMB of seven_seg is
begin
  process(Data, Pol)
  begin
    if Pol = '1' then
      case Data is
        when "0000" => Segout <= "1111110";
        when "0001" => Segout <= "0110000";
        when "0010" => Segout <= "1101101";
        when "0011" => Segout <= "1111001";
        when "0100" => Segout <= "0110011";
        when "0101" => Segout <= "1011011";
        when "0110" => Segout <= "1011111";
        when "0111" => Segout <= "1110000";
        when "1000" => Segout <= "1111111";
        when "1001" => Segout <= "1111011";

        when "1010" => Segout <= "1110111";
        when "1011" => Segout <= "0011111";
        when "1100" => Segout <= "0001101";
        when "1101" => Segout <= "0111101";
        when "1110" => Segout <= "1001111";
        when "1111" => Segout <= "1000111";

      end case;
    else
      case Data is
        when "0000" => Segout <= "0000001";
        when "0001" => Segout <= "1001111";
        when "0010" => Segout <= "0010010";
        when "0011" => Segout <= "0000110";
        when "0100" => Segout <= "1001100";
        when "0101" => Segout <= "0100100";
        when "0110" => Segout <= "0100000";
        when "0111" => Segout <= "0001111";
        when "1000" => Segout <= "0000000";
        when "1001" => Segout <= "0000100";

        when "1010" => Segout <= "0001000";
        when "1011" => Segout <= "1100000";
        when "1100" => Segout <= "1110010";
        when "1101" => Segout <= "1000010";
        when "1110" => Segout <= "0110000";
        when "1111" => Segout <= "0111000";
      end case;
    end if;
  end process;
end architecture COMB;
