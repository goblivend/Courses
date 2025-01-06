with Ada.Text_IO;
with Ordered_Structures; use Ordered_Structures;

procedure Main is
   OList : Ordered_Structure'Class := New_Ordered_List;
   OBST  : Ordered_Structure'Class := New_BST;
begin
   OList.Insert (2);
   OList.Insert (1);
   OList.Insert (3);
   OList.Delete (2);

   for E of OList.Get_Elements loop
      Ada.Text_IO.Put_Line ("List element:" & E'Img);
   end loop;

   OBST.Insert (3);
   OBST.Insert (2);
   OBST.Insert (1);
   OBST.Delete (3);

   for E of OBST.Get_Elements loop
      Ada.Text_IO.Put_Line ("BST element:" & E'Img);
   end loop;
end Main;
