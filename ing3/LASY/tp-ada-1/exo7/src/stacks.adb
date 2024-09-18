package body Stacks is

    procedure Push (S : in out Stack; V : Integer) is
    begin
        if S.Idx < Max_Size then
            S.Idx          := S.Idx + 1;
            S.Data (S.Idx) := V;
        else
            null; -- stack full
        end if;
    end Push;

    procedure Pop (S : in out Stack; V : out Integer) is
    begin
        if S.Idx > 0 then
            V     := S.Data (S.Idx);
            S.Idx := S.Idx - 1;
        else
            null; -- no more element
        end if;
    end Pop;

end Stacks;
