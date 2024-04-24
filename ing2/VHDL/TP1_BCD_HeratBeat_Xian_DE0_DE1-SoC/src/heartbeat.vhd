
library IEEE;
  use IEEE.std_logic_1164.all;
  use IEEE.numeric_std.all;

-- -----------------------------------
    Entity HEARTBEAT is
-- -----------------------------------
  port ( CLK      : in  std_logic;    -- 60 MHz in  case of Tornado
         RST      : in  std_logic;    -- Asynchronous Reset active high
         TICK10US : in  std_logic;    -- Tick 10 us (see FDIV)
         LED      : out std_logic );  -- Output LED
end entity HEARTBEAT;


-- -----------------------------------
    Architecture RTL of HEARTBEAT is
-- -----------------------------------
      signal count512: integer range 0 to 512 := 0;
      signal count256: integer range 0 to 256 := 0;
      signal status: std_logic := '0';
begin

process (CLK, RST)
begin
  if RST = '1' then
    count512 <= 0;    
    count256 <= 0;
  elsif rising_edge(CLK) then
    if TICK10US = '1' then
      count512 <= count512 + 1;
      if count512 >= 511 then
        count512 <= 0;
        if status = '0' then
          count256 <= count256 + 1;
          if count256 >= 255 then
            status <= '1';
          end if;
        else
          count256 <= count256 - 1;
          if count256 <= 1 then
            status <= '0';
          end if;
        end if;
      end if;
    end if;
  end if;
end process;

LED <= '0' when count256 < count512 else '1'; 

end architecture RTL;
