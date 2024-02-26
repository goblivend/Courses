--GNAT 4.9.3

with Ada.Text_IO; use Ada.Text_IO;

procedure Tp3q2 is
   -- Suppose that we have a term a*(b+c).
   -- The goal of this function is to compute a*b + a*c  and to
   -- return the computed result.
   generic
      -- The Generic type
      type Element(<>) is private;

      -- Require a multiplication for this type
      with function "*"(U, V: Element) return Element is <>;

      -- Require an addition for this type
      -- FIXME-begin
      with function "+"(U, V: Element) return Element is <>;
      -- FIXME-end

   function Generic_Distribute(a,b,c : in Element) return Element;

   -- The implementation of the generic function
   function Generic_Distribute(a,b,c : in Element) return Element is
   begin
      -- FIXME-begin
      return a*b + a*c;
      -- FIXME-end
   end Generic_Distribute;

   -- Instanciate the function for  integers
   function distribute_integer is
      new Generic_Distribute(Integer,      -- The type to use
                             "*" => "*",   -- The multiplicative function
                             -- Specify the addition function
                             -- FIXME-begin
                               "+" => "+"
                               -- FIXME-end
                            );

   -- Implement necessary function in order to provide an implementation
   -- of the generic function for String. Note that a(i) references the i-th
   -- element for the string "a", a'Length its length, and a'Range helps for
   -- iterations
   -- FIXME-begin
   function "*" (U, V: String) return String is
      Result : String (1 .. 2 * U'Length * V'Length);
      Index  : Natural := 1;
   begin
      for I in U'Range loop
         for J in V'Range loop
            Result (Index) := U (I) ;
            Result (Index + 1) := V (J);
            Index := Index + 2;
         end loop;
      end loop;
      return Result;
   end "*";

   function "+" (U, V: String) return String is
      Result : String (1 .. U'Length + V'Length);
      Index  : Natural := 1;
   begin
      for I in U'Range loop
         Result (Index) := U (I) ;
         Index := Index + 1;
      end loop;
      for J in V'Range loop
         Result (Index) := V (J);
         Index := Index + 1;
      end loop;
      return Result;
   end "+";

   -- FIXME-end

   -- Provide an implementation of Generic_Distribe for string
   -- where "*" is the cartesian product of two string, i.e.
   -- "ab"*"ac" would result in "aaacbabc".
   function distribute_string is
      new Generic_Distribute(
                             -- FIXME-begin
                               String,
                                 "*" => "*",
                                 "+" => "+"
                               -- FIXME-end
                            );

begin
   Put_Line(Integer'Image(distribute_integer(2,3,4)));
   Put_Line(distribute_string("az","b","c"));
end Tp3q2;
