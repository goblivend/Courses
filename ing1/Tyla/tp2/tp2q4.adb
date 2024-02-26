-- Compilation with "gnat make  -gnata  prepost.adb"
-- If -gnata is ommitted, pre and post conditions will not
-- be checked

with Ada.Text_IO; use Ada.Text_IO;
with Ada.Integer_Text_IO; use Ada.Integer_Text_IO;

procedure Tp2q4 is

   -- FIXME-begin
   function add(A, B : in out INTEGER) return INTEGER
   is
      res : INTEGER;
   begin
      res := A + B;
      A := 2;
      B := 3;
      return res;
   end add;
   -- FIXME-end


   function Sum_Of_Numbers(A, B : in out INTEGER) return INTEGER
     with
     -- Precondition must ensure that A should be negative, B positive
     -- Postcondition must ensure that the expected result is A+B

     -- *Warning*
     -- You must play with pre/post conditions so that:
     --     (1) the return value will be correctly checked
     --     (2) BUT the values of A and B are modifified by the pre/post-condition


     -- FIXME-begin
     Pre => (A < 0 and B > 0),
     Post => (Sum_Of_Numbers'Result = add(A, B))
     -- FIXME-end
   is
   begin
      -- Changing A+B to A*B must trigger an error
      return A + B;
   end Sum_Of_Numbers;

   A, B : INTEGER;
begin
   A := -1;
   B := 2;
   Put(Sum_Of_Numbers(A,B));
   Put(A);
   Put(B);
end Tp2q4;
