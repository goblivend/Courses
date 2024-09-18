with Ada.Text_IO;     use Ada.Text_IO;
with Singleton_Stack; use Singleton_Stack;

procedure Exo6 is
   N : Integer;
begin
   Push (0);
   Push (1);
   Push (2);
   Push (3);
   Pop (N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (N);
   Put_Line ("N = " & Integer'Image (N));

end Exo6;
