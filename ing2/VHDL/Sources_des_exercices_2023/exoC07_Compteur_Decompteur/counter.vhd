-- Squelette pour l'exercice Compteur
library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.Numeric_std.all;

entity Counter is
  port (Clock, Reset, Enable, Load, UpDn: in Std_logic;
        Data: in Std_logic_vector(7 downto 0);
        Q:   out Std_logic_vector(7 downto 0));
end entity Counter;

-- Premi�re partie
architecture RTL of Counter is
  signal tmp: unsigned(7 downto 0);
begin
  process(Clock, Reset)
  begin
    if Reset = '1' then
      tmp <= (others =>'0');
    elsif rising_edge(Clock) then
      if Enable = '1' then
        if load = '1' then
          tmp <= unsigned(data);
        elsif UpDn = '1' then
          tmp <= tmp + 1;
        else
          tmp <= tmp - 1;
        end if;

      end if;

    end if;
  end process;
  Q <= std_logic_vector(tmp);
end architecture;

-- Deuxi�me partie
architecture RTL2 of Counter is
 signal tmp: unsigned(7 downto 0);
begin
  process(Clock, Reset)
  begin
    if rising_edge(Clock) then
      if Reset = '1' then
        tmp <= (others =>'0');
      elsif Enable = '1' then
        if load = '1' then
          tmp <= unsigned(data);
        elsif UpDn = '1' then
          tmp <= tmp + 1;
        else
          tmp <= tmp - 1;
        end if;

      end if;

    end if;
  end process;
  Q <= std_logic_vector(tmp);
end architecture;

-- Troisi�me partie
architecture RingCounter of Counter is
  signal cnt : unsigned(7 downto 0);
begin
  process(clock, reset)
  begin
    if reset = '1' then
      cnt <= "00000001";
    elsif rising_edge(clock) then
      if enable = '1' then
        if load = '1' then
          cnt <= unsigned(data);
        else
          if UpDn = '1' then
            if cnt = "10000000" then
              cnt <= "00000001";
            else
              cnt <= cnt srl 1;
            end if;
          else
            if cnt = "00000001" then
              cnt <= "10000000";
            else
              cnt <= cnt sll 1;
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
