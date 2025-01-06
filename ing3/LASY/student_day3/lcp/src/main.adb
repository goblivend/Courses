with Longest_Common_Prefix;
with Ada.Text_IO;

procedure Main
is
   L : Natural;
begin
   Longest_Common_Prefix.Data := (3, 4, 5, 0, 9, 4, 5, 0, 9, others => 0);
   L := Longest_Common_Prefix.LCP (2, 6);
   Ada.Text_IO.Put_Line ("LCP = " & Integer'Image(L));
end Main;
