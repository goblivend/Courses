LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;

entity addc1 is
    port (A, B, Cin : in std_logic;
        Sum, Cout : out std_logic);
end entity;

architecture dataflow of addc1 is
begin
    Sum <= A xor B xor Cin;
    Cout <= (A and B) or (Cin and A) or (Cin and B);
end architecture;
