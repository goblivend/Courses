-- Test Bench pour l'exercice carte mémoire (non générique)


entity Board_TB is
end entity Board_TB;

library IEEE;
use IEEE.Std_logic_1164.all;
use IEEE.Numeric_std.all;

architecture Bench of Board_TB is

 
  signal AddrBus: Std_logic_vector(7 downto 0) := (others => '0');
  signal DataBus: Std_logic_vector(15 downto 0);
  signal nOE, nWE: Std_logic;

begin

  process

    variable AddrTemp: Std_logic_vector(7 downto 0);
    variable DataTemp: Std_logic_vector(15 downto 0);

  begin
    nWE <= '1';
    nOE <= '1';
    AddrBus <= (others => '0');
    DataBus <= (others => 'Z');
    wait for 10 NS;

-- Test Read/Write conflict

    nWE <= '0';
    nOE <= '0';
    wait for 10 NS;
    nWE <= '1';
    nOE <= '1';
    wait for 10 NS;


--  Write into all the Ram contents by driving the data bus

    for i in 0 to 2**8-1 loop
      AddrTemp := Std_logic_vector(To_unsigned(I, 8));
      DataTemp := "00000000" & AddrTemp;
      AddrBus <= AddrTemp;
      wait for 5 NS;
      DataBus <= DataTemp;
      wait for 5 NS;
      nWE <= '0';
      wait for 20 NS;
      nWE <= '1';
      wait for 20 NS;
    end loop;

--  Turn off driver to the data bus and read out new Ram contents

    DataBus <= (others => 'Z');
    wait for 50 NS;
    for i in 0 to 2**8-1 loop
      AddrTemp := Std_logic_vector(To_unsigned(I, 8));
      DataTemp := "00000000" & AddrTemp;
      AddrBus <= AddrTemp;
      wait for 10 NS;
      nOE <= '0';
      wait for 10 NS;
      assert DataBus = DataTemp
        report "Failed to read back correct data from Ram Board";
      wait for 10 NS;
      nOE <= '1';
      wait for 20 NS;
    end loop;
    assert FALSE report "Finished checking output" severity NOTE; 
    wait;
  end process;

  G1: entity work.Board port map (AddrBus, DataBus, nWE, nOE);

end; 

