package body Singleton_Stack is
    type Stack_Array is array (1 .. Max_Size) of Integer;
    Stack : Stack_Array;
    Idx   : Integer := 0;

    procedure Push (V : Integer) is
    begin
        if Idx < Max_Size then
            Idx         := Idx + 1;
            Stack (Idx) := V;
        else
            null; -- Overflow
        end if;
    end Push;

    procedure Pop (V : out Integer) is
    begin
        if Idx > 0 then
            V   := Stack (Idx);
            Idx := Idx - 1;
        else
            null; -- no more elements
        end if;
    end Pop;
end Singleton_Stack;
