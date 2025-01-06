package Stack is
   Max : constant Natural := 2;

   type Element_T is new Natural;
   type Stack_T is private;
   type Element_Array_T is array (Positive range 1 .. Max) of Element_T;

   function Is_Empty (S : Stack_T) return Boolean;
   function Is_Full (S : Stack_T) return Boolean;
   function Size (S : Stack_T) return Natural;
   function Get_Model (S : Stack_T) return Element_Array_T;

   procedure Reset (S : out Stack_T)
   with Post => Is_Empty (S) and then Size (S) = 0;
   function Pop (S : in out Stack_T) return Element_T
   with
     Pre => not Is_Empty (S),
     Post =>
       not Is_Full (S)
       and then Size (S) = Size (S'Old) - 1
       and then Pop'Result = Get_Model (S'Old) (Size (S'Old));
   procedure Push (S : in out Stack_T; E : Element_T)
   with
     Pre => not Is_Full (S),
     Post => not Is_Empty (S) and then Size (S) = Size (S'Old) + 1;
private
   type Stack_T is record
      Content : Element_Array_T;
      Length  : Natural := 0;
   end record;
   function Is_Empty (S : Stack_T) return Boolean
   is (S.Length = 0);
   function Is_Full (S : Stack_T) return Boolean
   is (S.Length = Max);
   function Size (S : Stack_T) return Natural
   is (S.Length);
   function Get_Model (S : Stack_T) return Element_Array_T
   is (S.Content);
end Stack;
