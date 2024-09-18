package Stacks is
    Max_Size : constant Integer := 512;

    type Stack is private;

    procedure Push (S : in out Stack; V : Integer);
    procedure Pop (S : in out Stack; V : out Integer);

private
    type Stack_Array is array (1 .. Max_Size) of Integer;

    type Stack is record
        Data : Stack_Array;
        Idx  : Integer := 0;
    end record;
end Stacks;
