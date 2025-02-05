library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity instr_mem is
    port(
        PC: in std_logic_vector (31 downto 0) := (others=>'0');
        Instruction: out std_logic_vector (31 downto 0) := x"E3A01020"--(others=>'0')
    );
end entity;

architecture RTL of instr_mem is
    type RAM64x32 is array (0 to 63) of std_logic_vector (31 downto 0);
    function init_mem return RAM64x32 is
        variable result : RAM64x32;
    begin
        for i in 63 downto 0 loop
            result (i):=(others=>'0');
        end loop;               -- PC        -- INSTRUCTION    -- COMMENTAIRE
        result (0):=x"E3A01020";-- 0x0 _main -- MOV R1,#0x20   -- R1 = 0x20
        result (1):=x"E3A02000";-- 0x1       -- MOV R2,#0x00   -- R2 = 0
        result (2):=x"E6110000";-- 0x2 _loop -- LDR R0,0(R1)   -- R0 = DATAMEM[R1]
        result (3):=x"E0822000";-- 0x3       -- ADD R2,R2,R0   -- R2 = R2 + R0
        result (4):=x"E2811001";-- 0x4       -- ADD R1,R1,#1   -- R1 = R1 + 1
        result (5):=x"E351002A";-- 0x5       -- CMP R1,0x2A    -- Flag = R1-0x2A,si R1 <= 0x2A
        result (6):=x"BAFFFFFB";-- 0x6       -- BLT loop       -- PC =PC+1+(-5) si N = 1
        result (7):=x"E6012000";-- 0x7       -- STR R2,0(R1)   -- DATAMEM[R1] = R2
        result (8):=x"EAFFFFF7";-- 0x8       -- BAL main       -- PC=PC+1+(-9)
        return result;
    end init_mem;
    signal mem: RAM64x32 := init_mem;
begin
    Instruction <= mem(to_integer(unsigned (PC)));
end architecture;

--

-- E3A01020 -- MOV R1,#0x20   -- R1 = 0x20
-- E    3    A    0    1    0    2    0
-- 1110 0011 1010 0000 0001 0000 0010 0000
-- ^    ^    ^    ^    ^    ^    ^    ^  ^
-- 31   27   23   19   15   11   7    3  0

-- E3A02000 -- MOV R2,#0x00   -- R2 = 0
-- E    3    A    0    2    0    0    0
-- 1110 0011 1010 0000 0010 0000 0000 0000
-- ^    ^    ^    ^    ^    ^    ^    ^  ^
-- 31   27   23   19   15   11   7    3  0

-- E6110000 -- LDR R0,0(R1)   -- R0 = DATAMEM[R1]
-- E    6    1    1    0    0    0    0
-- 1110 0110 0001 0001 0000 0000 0000 0000
-- ^    ^    ^    ^    ^    ^    ^    ^  ^
-- 31   27   23   19   15   11   7    3  0

-- E351002A -- CMP R1,0x2A    -- Flag = R1-0x2A,si R1 <= 0x2A
-- E    3    5    1    0    0    2    A
-- 1110 0011 0101 0001 0000 0000 0010 1010
-- ^    ^    ^    ^    ^    ^    ^    ^  ^
-- 31   27   23   19   15   11   7    3  0

-- E6012000 -- STR R2,0(R1)   -- DATAMEM[R1] = R2
-- E    6    0    1    2    0    0    0
-- 1110 0110 0000 0001 0010 0000 0000 0000
-- ^    ^    ^    ^    ^    ^    ^    ^  ^
-- 31   27   23   19   15   11   7    3  0
--                 Rn   Rd
