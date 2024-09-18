with Ada.Text_IO; use Ada.Text_IO;
with Stacks;      use Stacks;

procedure Exo7 is
   N  : Integer;
   S1 : Stack;
   S2 : Stack;
begin
   Push (S1, 0);
   Push (S2, 1);
   Push (S1, 2);
   Push (S2, 3);
   Pop (S2, N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (S1, N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (S2, N);
   Put_Line ("N = " & Integer'Image (N));
   Pop (S1, N);
   Put_Line ("N = " & Integer'Image (N));
end Exo7;
