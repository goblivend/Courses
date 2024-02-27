LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

entity mux21 is
    port (SEL, A, B : in std_logic;
          Y : out std_logic);
end entity;

architecture dataflow of mux21 is
begin
    Y <= ((not SEL) and A) or (SEL and B);
end architecture;
