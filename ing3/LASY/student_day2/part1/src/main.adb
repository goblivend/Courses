with Ada.Text_IO;
with Geometry; use Geometry;

procedure Main is
   C : Coordinates := (1.0, 2.0);
   V : Vector := Vector (C);
begin
   Ada.Text_IO.Put_Line ("Coord norm:" & Squared_Norm (C)'Img);

   Ada.Text_IO.Put_Line ("Vector norm:" & Squared_Norm (V)'Img);
end Main;
