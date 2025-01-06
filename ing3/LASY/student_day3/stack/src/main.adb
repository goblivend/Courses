with Stack; use Stack;
with Ada.Text_IO;

procedure Main is

   E1, E2 : Element_T;
   S      : Stack_T;
begin
   Reset (S);
   Push (S, 1);
   --   Push (S, 2);
   Pop (S, E1); -- Equals 2 ?
   Pop (S, E2); -- Equals 1 ?
   Ada.Text_IO.Put_Line (Integer'Image (Integer (E1)));
   Ada.Text_IO.Put_Line (Integer'Image (Integer (E2)));
end Main;
