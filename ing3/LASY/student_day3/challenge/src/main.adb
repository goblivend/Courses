with Test; use Test;
with Ada.Text_IO;

procedure Main is
   I : Integer := -2000;
begin
   I := Prove_The_Absence_Of_Run_Time_Errors(I);
   Ada.Text_IO.Put_Line(I'Img);
end Main;
