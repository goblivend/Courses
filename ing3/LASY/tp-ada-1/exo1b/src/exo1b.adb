with Ada.Text_IO; use Ada.Text_IO;

procedure Exo1b is
   procedure Invert (S : in out String)
   is
      Result : String (1 .. S'Length);
   begin
      for I in S'Range loop
         Result (S'Length - I + 1) := S (I);
      end loop;
      S := Result;
   end Invert;
   Str : String := "Hello, World!";
begin
   Invert (Str);
   Put_Line (Str);
end Exo1b;
