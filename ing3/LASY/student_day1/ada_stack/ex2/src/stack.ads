package Stack is
   Max : constant Natural := 2;

   type Element_T is new Natural;
   type Stack_T is private;

   function Is_Empty (S : Stack_T) return Boolean;
   function Is_Full (S : Stack_T) return Boolean;

   procedure Reset (S : out Stack_T);
   function Pop (S : in out Stack_T) return Element_T
   with Pre => not Is_Empty (S);
   procedure Push (S : in out Stack_T; E : Element_T)
   with Pre => not Is_Full (S);
private
   type Element_Array_T is array (Positive range 1 .. Max) of Element_T;
   type Stack_T is record
      Content : Element_Array_T;
      Length  : Natural := 0;
   end record;
   function Is_Empty (S : Stack_T) return Boolean
   is (S.Length = 0);
   function Is_Full (S : Stack_T) return Boolean
   is (S.Length = Max);
end Stack;
