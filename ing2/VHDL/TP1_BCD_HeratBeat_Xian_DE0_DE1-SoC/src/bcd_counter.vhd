-- BCD_COUNTER.vhd
-- ---------------------------------------
--   Squelette du compteur BCD
-- ---------------------------------------
--
-- Compter de 0000 � 9999 et repartir de 0000
-- Le comptage doit se faire a chaque N ms,
-- pour cela, on utilise le signal TICK1MS


library IEEE;
  use IEEE.std_logic_1164.ALL;
  use IEEE.numeric_std.ALL;


-- ---------------------------------------
-- entity BCD_COUNTER is
-- ---------------------------------------
-- generic ( N  : positive := 100 );  -- facteur de division de Tick1ms
--   port ( CLK        : in  std_logic;
--          RST        : in  std_logic;
--          TICK1MS    : in  std_logic;
--          UNITIES    : out std_logic_vector(3 downto 0);
--          TENS       : out std_logic_vector(3 downto 0);
--          HUNDREDS   : out std_logic_vector(3 downto 0);
--          THOUSNDS   : out std_logic_vector(3 downto 0));
-- end entity BCD_COUNTER;
--
-- ---------------------------------------
-- architecture RTL of BCD_COUNTER is
-- ---------------------------------------
--   subtype subint is integer range 0 to 10;
--   subtype tick is integer range 0 to N + 1;
--   signal tickCount: integer := 0;
--   signal unit: subint := 0;
--   signal ten: subint := 0;
--   signal hundred: subint := 0;
--   signal thousand: subint := 0;
-- begin
-- process(RST,CLK)
-- begin
--   if RST = '1' then
--     tickCount <= 0;
--     unit <= 0;
--     ten <= 0;
--     hundred <= 0;
--     thousand <= 0;
--   elsif rising_edge(CLK) then
--     if Tick1ms = '1' then
--       tickCount <= tickCount + 1;
--       if tickCount = N-1 then
--         tickCount <= 0;
--         unit <= unit + 1;
--         if unit = 9 then
--           unit <= 0;
--           ten <= ten + 1;
--           if ten = 9 then
--             ten <= 0;
--             hundred <= hundred + 1;
--             if hundred = 9 then
--               hundred <= 0;
--               thousand <= thousand + 1;
--               if thousand = 9 then
--                 unit <= 0;
--                 ten <= 0;
--                 hundred <= 0;
--                 thousand <= 0;
--               end if;
--             end if;
--           end if;
--         end if;
--       end if;
--     end if;
--   end if;
-- end process;
--   UNITIES <= STD_LOGIC_VECTOR(TO_UNSIGNED(unit,4));
--   TENS <= STD_LOGIC_VECTOR(TO_UNSIGNED(ten,4));
--   HUNDREDS <= STD_LOGIC_VECTOR(TO_UNSIGNED(hundred,4));
--   THOUSNDS <= STD_LOGIC_VECTOR(TO_UNSIGNED(thousand,4));
-- end architecture RTL;


-- Improvement
---------------------------------------
entity BCD_COUNTER is
  ---------------------------------------
    port ( CLK        : in  std_logic;
           RST        : in  std_logic;
           TICK1MS    : in  std_logic;
           SW         : in std_logic_vector(7 downto 0);
           UNITIES    : out std_logic_vector(3 downto 0);
           TENS       : out std_logic_vector(3 downto 0);
           HUNDREDS   : out std_logic_vector(3 downto 0);
           THOUSNDS   : out std_logic_vector(3 downto 0));
  end entity BCD_COUNTER;

  ---------------------------------------
  architecture RTL of BCD_COUNTER is
  ---------------------------------------
    signal s_units, s_tens, s_hundreds, s_thousands : natural := 0;
    signal s_ms : natural := 0;
  begin
  process(RST,CLK)
  begin
    if RST = '1' then
      s_units <= 0;
      s_tens <= 0;
      s_hundreds <= 0;
      s_thousands <= 0;
      s_ms <= 1;
    elsif rising_edge(CLK) then
      if Tick1ms = '1' then
        s_ms <= s_ms + 1;
        if s_ms >= UNSIGNED(SW) then
          s_ms <= 0;
          s_units <= s_units + 1;
          if s_units = 9 then
            s_units <= 0;
            s_tens <= s_tens + 1;
            if s_tens = 9 then
              s_tens <= 0;
              s_hundreds <= s_hundreds + 1;
              if s_hundreds = 9 then
                s_hundreds <= 0;
                s_thousands <= s_thousands + 1;
                if s_thousands = 9 then
                  s_thousands <= 0;
                end if;
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;
  UNITIES <= std_logic_vector(to_unsigned(s_units,4));
  TENS <= std_logic_vector(to_unsigned(s_tens,4));
  HUNDREDS <= std_logic_vector(to_unsigned(s_hundreds,4));
  THOUSNDS <= std_logic_vector(to_unsigned(s_thousands,4));
  end architecture RTL;
