library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity UART_TX is
    port (
        Clk   : in std_logic;
        Reset : in std_logic;
        Go    : in std_logic;
        Data  : in std_logic_vector(7 downto 0);
        Tick  : in std_logic;
        Tx_Busy : out std_logic;
        Tx    : out std_logic
    );
end entity;

architecture RTL of UART_TX is
    signal reg : std_logic_vector(9 downto 0);
    signal i : integer;
    type StateType is (E0, E1, E2, E3, E4);
    signal State : StateType;
begin

    process (Clk, Reset)
    begin
        if Reset = '1'then
            State <= E0;
            Tx <= '1';
            Tx_Busy <= '0';
            i <= 0;
            reg <= (others => '0');
        elsif rising_edge(Clk) then
            case State is
                when E0 =>
                    if Go = '1' then
                        State <= E1;
                        reg <= '1' & Data & '0';
                        Tx_Busy <= '1';
                        i <= 0;
                    end if;
                when E1 =>
                    if Tick = '1' then
                        State <= E2;
                        Tx <= reg(i);
                        i <= i + 1;
                    end if;
                when E2 =>
                    if Tick = '1' then
                        State <= E3;
                    end if;
                when E3 =>
                    if i > 9 then
                        State <= E4;
                        Tx_Busy <= '0';
                        i <= 0;
                    else
                        State <= E2;
                        Tx <= reg(i);
                        i <= i + 1;
                    end if;
                when E4 =>
                    State <= E0;
                when others =>
                    State <= E0;
            end case;
        end if;
    end process;
end architecture;
