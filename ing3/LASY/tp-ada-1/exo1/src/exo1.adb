with Ada.Text_IO; use Ada.Text_IO;

procedure Exo1 is
   function Invert (S : String) return String is
      Result : String (S'Range);
   begin
      for I in S'Range loop
         Result (S'Length - I + 1) := S (I);
      end loop;
      return Result;
   end Invert;
begin
   Put_Line (Invert ("Hello, World!"));
end Exo1;
