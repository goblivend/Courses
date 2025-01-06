pragma Ada_2022;

with Ada.Text_IO;
with Isqrt; use Isqrt;
with Max; use Max;

procedure main is
   R : Natural;
   A : constant Our_Array := [1, 2, 8, 4, 0, 12, 5, others => 0];
   Sorted_A : constant Our_Array := [1, 5, 10, 20, 32, 42, others => 100];
   Sorted_B : constant Our_Array := [2, 8, 15, 20, 25, 43, others => 99];
   M : Merged_Array;
   I_Max : constant Index_Range := Arrays_Max (A);
begin
   R := Find_Int_Sqrt (8);
   Ada.Text_IO.Put_Line (R'Img);

   R := Find_Int_Sqrt (9);
   Ada.Text_IO.Put_Line (R'Img);

   Ada.Text_IO.Put_Line (A (I_Max)'Img);

   Merge (Sorted_A, Sorted_B, M);

   Ada.Text_IO.Put_Line (M'Img);

end main;
