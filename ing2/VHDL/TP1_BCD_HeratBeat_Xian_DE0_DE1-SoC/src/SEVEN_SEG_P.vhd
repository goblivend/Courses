library IEEE;
  use IEEE.std_logic_1164.all;
  use IEEE.numeric_std.all;

-- ------------------------------
    Entity SEVEN_SEG_P is
-- ------------------------------
  port ( Data   : in  std_logic_vector(4 downto 0); -- Expected within 0 .. 9
         Segout : out std_logic_vector(1 to 7) );   -- Segments A, B, C, D, E, F, G
end entity SEVEN_SEG_P;

-- -----------------------------------------------
    Architecture COMB of SEVEN_SEG_P is
-- ------------------------------------------------

begin
  process(Data)
  begin
    case Data is
      when "00000" => Segout <= "0000001";
      when "00001" => Segout <= "1001111";
      when "00010" => Segout <= "0010010";
      when "00011" => Segout <= "0000110";
      when "00100" => Segout <= "1001100";
      when "00101" => Segout <= "0100100";
      when "00110" => Segout <= "0100000";
      when "00111" => Segout <= "0001111";
      when "01000" => Segout <= "0000000";
      when "01001" => Segout <= "0000100";
      when "01010" => Segout <= "1111110"; -- -
      when "01011" => Segout <= "0011000"; -- P
      when "01100" => Segout <= "0001000"; -- A
      when "01101" => Segout <= "0000000"; -- B
      when "01110" => Segout <= "0100011"; -- W
      when "01111" => Segout <= "0000001"; -- O
      when "10000" => Segout <= "0001001"; -- N
      when others => Segout <= "1111111"; --  
    end case;
  end process;
end architecture COMB;

