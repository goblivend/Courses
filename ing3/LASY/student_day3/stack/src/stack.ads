package Stack
  with SPARK_Mode => On
is
   Max : constant Natural := 100;
   subtype Stack_Range is Positive range 1 .. Max;
   subtype Length_Type is Natural range 0 .. Max;

   type Element_T is new Natural;
   type Stack_T is private;

   type Model is array (Positive range <>) of Element_T
   with Dynamic_Predicate => Model'Length <= Max;

   function Get_Model (S : Stack_T) return Model
   with Ghost;

   function Size (S : Stack_T) return Length_Type;
   function Is_Full (S : Stack_T) return Boolean
   is (Size (S) = Max);
   function Is_Empty (S : Stack_T) return Boolean
   is (Size (S) = 0);

   procedure Reset (S : out Stack_T)
   with Post => Is_Empty (S);

   procedure Pop (S : in out Stack_T; E : out Element_T)
   with Pre => not Is_Empty (S), Post => Get_Model (S) & E = Get_Model (S)'Old;

   procedure Push (S : in out Stack_T; E : Element_T)
   with Pre => not Is_Full (S), Post => Get_Model (S) = Get_Model (S)'Old & E;


private

   type Element_Array is array (Stack_Range) of Element_T;
   type Stack_T is record
      Content : Element_Array;
      Length  : Length_Type := 0;
   end record;

   function Size (S : Stack_T) return Length_Type
   is (S.Length);

   function Get_Model (S : Stack_T) return Model
   is (Model (S.Content (1 .. S.Length)));
end Stack;
