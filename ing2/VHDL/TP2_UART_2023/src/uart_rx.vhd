library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity UART_RX is
    port (
        Clk          : in std_logic;
        Reset        : in std_logic;
        Rx           : in std_logic;
        Tick_halfbit : in std_logic;
        Clear_fdiv   : out std_logic;
        Err          : out std_logic;
        DAV          : out std_logic;
        Data         : out std_logic_vector(7 downto 0)
    );
end UART_RX;

architecture RTL of UART_RX is
    signal reg       : std_logic_vector(7 downto 0);
    signal Rxr, Rxrr : std_logic;
    signal i : integer 0 to 15;
    type StateType is (E0, E1, E1bis, E2, E3, E4, E5, E6, E7, E8, Erreur);
    signal State     : StateType;
begin
    process (Clk, Reset)
        if Reset = '1' then
            State <= E0;
            clear_fdiv <= '0';
            i <= 0;
            reg <= (others => '0');
            data <= (others => '0');
            err <= '0';
            dav <= '0';
        elsif rising_edge(Clk) then
            Rxr <= Rx;
            Rxrr <= Rxr;
            case State is
                when E0 =>
                    if Rxrr = '0' then
                        State <= E1;
                        clear_fdiv <= '1';
                    end if;
                when E1 =>
                    State <= E1bis;
                    clear_fdiv <= '0';
                when E1bis =>
                    if Tick_halfbit = '1' then
                        State <= E2;
                    end if;
                when E2 =>
                    if Rxrr = '1' then
                        Erreur <= '1';
                        State <= Err;
                    else
                        State <= E3;
                    end if;
                when E3 =>
                    if Tick_halfbit = '1' then
                        State <= E4;
                    end if;
                when E4 =>
                    if i > 7 then
                        State <= E6;
                    elsif Tick_halfbit = '1' then
                        State <= E5;
                        reg(i) <= Rxrr;
                        i <= i + 1;
                    end if;
                when E5 =>
                    if Tick_halfbit = '1' then
                        State <= E4;
                    end if;
                when E6 =>
                    if Tick_halfbit = '1' then
                        State <= E7;
                    end if;
                when E7 =>
                    if Rxrr = '0' then
                        State <= Erreur;
                        Err <= '1';
                    else
                        State <= E8;
                        DAV <= '1';
                        data <= reg;
                    end if;
                when E8 =>
                    i <= 0;
                    reg <= (others => '0');
                    err <= '0';
                    dav <= '0';
                    State <= E0;
                    clear_fdiv <= '0';
                when Erreur =>
                    i <= 0;
                    reg <= (others => '0');
                    err <= '0';
                    dav <= '0';
                    State <= E0;
                    clear_fdiv <= '0';
            end case;
        end if;

    end process;

end RTL;
