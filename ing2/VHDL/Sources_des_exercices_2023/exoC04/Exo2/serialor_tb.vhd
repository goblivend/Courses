-- Test bench for Serial OR Exercise

-- This file contains two architectures...

--   architecture TB1 for the first part of the exercise
--   architecture TB2 for the If You Have Time

library IEEE;
use IEEE.Std_logic_1164.all;

entity SerialOR_TB is
port(OK: out BOOLEAN);
end;

architecture TB1 of SerialOR_TB is

  -- For first part of exercise
  
  signal Clock, D, Q, rst: Std_logic; 

begin

rst <= '1' , '0' after 1 ns;
  process
  begin
    D <= '0';
    OK <= TRUE;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    wait;
  end process;

  UUT: entity work.SerialOR(RTL)  port map (Clock => clock,rst => rst, D=>D, Q=>Q);

end;




architecture TB2 of SerialOR_TB is

  -- For "If You Have Time" part
  signal Clock,rst, D, Q: Std_logic;

begin

	rst <= '1' , '0' after 1 ns;

  process
  begin
    D <= '0';
    OK <= TRUE;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    D <= '1';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    D <= '0';

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '1' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    Clock <= '0';
    wait for 5 NS;
    Clock <= '1';
    wait for 5 NS;

    if Q /= '0' then
      OK <= FALSE;
    end if;

    wait;
  end process;


UUT: entity work.SerialOR(RTL2)  port map (Clock=> clock, rst => rst,D=> D, Q=> Q);
end;


