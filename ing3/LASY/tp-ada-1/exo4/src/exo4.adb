with Ada.Text_IO; use Ada.Text_IO;

procedure Exo4 is
   function To_Roman (I : Integer) return String is
      Roman : String (1 .. 15) := (others => '.');
      N     : Integer          := I;
      Idx   : Integer          := 1;

      procedure Add (C : Character) is
      begin
         Roman (Idx) := C;
         Idx         := Idx + 1;
      end Add;

      procedure AddN (C : Character; N : Integer) is
      begin
         for I in 1 .. N loop
            Add (C);
         end loop;
      end AddN;

   begin
      AddN ('M', N / 1_000);
      N := N mod 1_000;

      if N >= 900 then
         Add ('C');
         Add ('M');
         N := N - 900;
      end if;

      if N >= 500 then
         Add ('D');
         N := N - 500;
      end if;

      if N >= 400 then
         Add ('C');
         Add ('D');
         N := N - 400;
      end if;

      AddN ('C', N / 100);
      N := N mod 100;

      if N >= 90 then
         Add ('X');
         Add ('C');
         N := N - 90;
      end if;

      if N >= 50 then
         Add ('L');
         N := N - 50;
      end if;

      if N >= 40 then
         Add ('X');
         Add ('L');
         N := N - 40;
      end if;

      AddN ('X', N / 10);
      N := N mod 10;

      if N >= 9 then
         Add ('I');
         Add ('X');
         N := N - 9;
      end if;

      if N >= 5 then
         Add ('V');
         N := N - 5;
      end if;

      if N >= 4 then
         Add ('I');
         Add ('V');
         N := N - 4;
      end if;

      AddN ('I', N);

      -- Return substring of Roman
      return Roman (1 .. Idx - 1);
   end To_Roman;
begin
   Put_Line (To_Roman (1));
   Put_Line (To_Roman (3));
   Put_Line (To_Roman (4));
   Put_Line (To_Roman (5));
   Put_Line (To_Roman (9));
   Put_Line (To_Roman (10));
   Put_Line (To_Roman (49));
   Put_Line (To_Roman (50));
   Put_Line (To_Roman (95));
   Put_Line (To_Roman (100));
   Put_Line (To_Roman (200));
   Put_Line (To_Roman (450));
   Put_Line (To_Roman (550));
   Put_Line (To_Roman (980));
   Put_Line (To_Roman (1_500));
   Put_Line (To_Roman (2_024));
end Exo4;
