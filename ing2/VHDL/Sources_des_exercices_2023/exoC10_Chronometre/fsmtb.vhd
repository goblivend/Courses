-- Banc de test pourl'exercice sur les machines à état

entity FSMTB is
	port( OK: out BOOLEAN := TRUE);
end entity;

library IEEE;
use IEEE.STD_LOGIC_1164.all;

architecture A1 of FSMTB is

  

  signal clk, nRst: STD_LOGIC := '0';
	signal Buttons : std_logic_vector(1 downto 0);
	signal Start_Stop : std_logic;
	signal Clear : std_logic;
  signal cnt_en, cnt_rst: STD_LOGIC;

  

begin

  process
  begin
    while Now < 200 NS loop
      clk <= '0';
      wait for 5 NS;
      clk <= '1';
      wait for 5 NS;
    end loop;
    wait;
  end process;

  process
  begin
    nRst <= '0'; -- Zero
    Buttons <= "00";
    wait for 3 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    nRst <= '1'; 
    for I in 1 to 17 loop
      wait for 1 NS;
      if cnt_en /= '0' or cnt_rst /= '0' then
        OK <= FALSE;
      end if;
    end loop;
    Buttons <= "10"; -- Start
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Running
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "10"; -- Stop
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Stopped
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "01"; -- Reset
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '1' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Zero
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '1' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "10"; -- Start (from Zero)
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Running
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "10"; -- Stop
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Stopped
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "10"; -- Start (from Stopped)
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Running
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "10"; -- Stop
    wait for 1 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Stopped
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "01"; -- Reset
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '1' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    Buttons <= "00"; -- Zero
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '1' then
      OK <= FALSE;
    end if;
    wait for 5 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    
    -- Test asynchronous reset (gives an error if reset is synchronous)

    wait for 5 ns;
    Buttons <= "10"; -- Start
    wait for 1 NS;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 4 NS;
    if cnt_en /= '1' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    wait for 3 ns;
    nrst <= '0';
    wait for 1 ns;
    if cnt_en /= '0' or cnt_rst /= '0' then
      OK <= FALSE;
    end if;
    nrst <= '1';
    wait for 1 ns;
   
    wait;
  end process;
	Start_Stop <=Buttons(1);
	Clear <= Buttons(0);
  UUT: entity work.FSM port map (	clk =>clk,
  												nrst => nrst, 
  												Start_Stop =>Start_Stop,
  												Clear => Clear,
  												Cnt_En=>cnt_en, 
  												Cnt_Rst=>cnt_rst);

end;

