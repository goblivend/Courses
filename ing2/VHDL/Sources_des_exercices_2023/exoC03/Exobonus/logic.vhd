library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity LOGIC is
    port (
        J,K,L,M,N : in std_logic;
        P,Q : out std_logic);
end entity;

Architecture STRUCT of LOGIC is
    signal P2 : std_logic;
begin
    ANDOR1: entity work.ANDOR(dataflow) port map (A => J, B => K, C => L, F => P2);
    ANDOR2: entity work.ANDOR(dataflow) port map (A => P2, B => M, C => N, F => Q);
    P <= P2;
end architecture;
