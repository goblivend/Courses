library IEEE;
  use IEEE.std_logic_1164.ALL;
  use IEEE.numeric_std.ALL;

---------------------------------------
entity CHENILLARD is
  ---------------------------------------
    port ( CLK        : in  std_logic;
           RST        : in  std_logic;
           TICK10MS    : in  std_logic;
           LED          : out std_logic_vector(9 downto 0));
  end entity CHENILLARD;
  
  ---------------------------------------
  architecture RTL of CHENILLARD is
  ---------------------------------------
    subtype tick is integer range 0 to 11;
    signal tickCount: integer := 0;
    signal LEDtmp: std_logic_vector(9 downto 0) := "0000000001";
    signal count: integer range 0 to 10 := 0;
  begin
  process(RST,CLK)
    variable status: std_logic := '0';
  begin
    if RST = '1' then
      tickCount <= 0;
      status := '0';
      count <= 0;
      LEDtmp <= "0000000001";
    elsif rising_edge(CLK) then
      if Tick10ms = '1' then 
        tickCount <= tickCount + 1;
        if tickCount >= 10 then
          tickCount <= 0;
          count <= count + 1;
          if status = '1' then
            LEDtmp <= LEDtmp(0) & LEDtmp(9 downto 1);
          else
            LEDtmp <= LEDtmp(8 downto 0) & LEDtmp(9);
          end if;
          if count > 7 then
            status := not status;
            count <= 0;
          end if;
        end if;
      end if;
    end if;
  end process;
    LED <= LEDtmp;
  end architecture RTL;
  