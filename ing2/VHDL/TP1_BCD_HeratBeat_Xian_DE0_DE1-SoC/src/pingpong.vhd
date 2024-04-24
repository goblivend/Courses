library IEEE;
  use IEEE.std_logic_1164.ALL;
  use IEEE.numeric_std.ALL;

---------------------------------------
entity PINGPONG is
---------------------------------------
  port (  CLK         : in  std_logic;
          RST         : in  std_logic;
          KEYA        : in  std_logic;
          KEYB        : in  std_logic;
          TICK10MS    : in  std_logic;
          POINTAU     : out std_logic_vector(4 downto 0);
          POINTAT     : out std_logic_vector(4 downto 0);
          POINTBU     : out std_logic_vector(4 downto 0);
          POINTBT     : out std_logic_vector(4 downto 0);
          POINTM1     : out std_logic_vector(4 downto 0);
          POINTM2     : out std_logic_vector(4 downto 0);
          LED         : out std_logic_vector(9 downto 0));
end entity PINGPONG;

---------------------------------------
architecture RTL of PINGPONG is
---------------------------------------
  signal tickCount: integer range 0 to 61 := 0;
  signal LEDvalue: std_logic_vector(9 downto 0) := "0000000001";
  signal LEDpos: integer range 0 to 10 := 0;
  signal direction: std_logic := '0';

  signal vpointAU: integer range 0 to 18 := 0;
  signal vpointAT: integer range 0 to 18 := 0;
  signal vpointBU: integer range 0 to 18 := 0;
  signal vpointBT: integer range 0 to 18 := 0;
  signal vpointM1: integer range 0 to 18 := 10;
  signal vpointM2: integer range 0 to 18 := 10;

  signal speed: integer range 0 to 11 := 10;
  signal services: integer range 0 to 11 := 1;
  signal status: integer range 0 to 3 := 0;
  signal blink: std_logic := '0';
begin
  process(CLK)
    variable updateService: std_logic := '0';
  begin
    if RST = '1' then
      tickCount <= 0;
      LEDvalue <= "0000000001";
      LEDpos <= 0;
      direction <= '0';
      vpointAU <= 0;
      vpointAT <= 0;
      vpointBU <= 0;
      vpointBT <= 0;
      vpointBU <= 0;
      vpointM1 <= 10;
      vpointM2 <= 10;
      speed <= 10;
      services <= 1;
      updateService := '0';
      status <= 0;
      blink <= '0';
    elsif rising_edge(CLK) then
      if Tick10ms = '1' then
        tickCount <= tickCount + 1;
        if tickCount = speed and status = 0 then -- wait for x ms
          tickCount <= 0;

          if direction = '0' then
            LEDpos <= LEDpos + 1;
          else
            LEDpos <= LEDpos - 1;
          end if;

          -- update LED
          if direction = '0' then
            LEDvalue <= LEDvalue(8 downto 0) & LEDvalue(9);
          else
            LEDvalue <= LEDvalue(0) & LEDvalue(9 downto 1);
          end if;

          if LEDpos = 9 and direction = '0' then
            -- handle PB input
            if KEYB = '0' then
              speed <= speed - 1;
              if speed = 1 then
                speed <= 1;
              end if;
              direction <= '1';
              LEDvalue <= "1000000000";
              LEDpos <= 9;
            else
              -- PA score update
              vpointAU <= vpointAU + 1;
              if vpointAU = 9 then
                vpointAU <= 0;
                vpointAT <= vpointAT + 1;
              end if;
              -- if vpointAT = 2 and vpointAU = 0 then
              --   status <= 1;
              -- end if;
              -- PA service
              updateService := '1';
            end if;
          elsif LEDpos = 0 and direction = '1' then
            -- handle PA input
            if KEYA = '0' then
              speed <= speed - 1;
              if speed = 1 then
                speed <= 1;
              end if;
              direction <= '0';
              LEDvalue <= "0000000001";
              LEDpos <= 0;
            else
              -- PB score update
              vpointBU <= vpointBU + 1;
              if vpointBU = 9 then
                vpointBU <= 0;
                vpointBT <= vpointBT + 1;
              end if;
              -- if vpointBT = 2 and vpointBU = 0 then
              --   status <= 2;
              -- end if;
              -- PB service
              updateService := '1';
            end if;
          end if;

          -- handle services
          if updateService = '1' then
            updateService := '0';
            services <= services + 1;
            if services < 5 then
              direction <= '0';
              LEDvalue <= "0000000001";
              LEDpos <= 0;
            else
              direction <= '1';
              LEDvalue <= "1000000000";
              LEDpos <= 9;
            end if;
            if services = 9 then
              services <= 0;
            end if;
            speed <= 10;
          end if;
        -- elsif tickCount = 60 then
        --   tickCount <= 0;
        --   if blink = '0' then
        --     blink <= '1';
        --     LEDvalue <= "0000000000";
        --     vpointAU <= 17;
        --     vpointAT <= 17;
        --     vpointBU <= 17;
        --     vpointBT <= 17;
        --     vpointM1 <= 17;
        --     vpointM2 <= 17;
        --   else
        --     blink <= '0';
        --     LEDvalue <= "1111111111";
        --     vpointBT <= 11;
        --     if status = 1 then
        --       vpointBU <= 12;
        --     else
        --       vpointBU <= 13;
        --     end if;
        --     vpointAT <= 15;
        --     vpointAU <= 16;
        --     vpointM2 <= 17;
        --     vpointM1 <= 14;
        --   end if;
        end if;
      end if;
    end if;
  end process;

  LED <= LEDvalue;
  POINTAU <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointAU,5));
  POINTAT <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointAT,5));
  POINTBU <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointBU,5));
  POINTBT <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointBT,5));
  POINTM1 <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointM1,5));
  POINTM2 <= STD_LOGIC_VECTOR(TO_UNSIGNED(vpointM2,5));
end architecture RTL;
