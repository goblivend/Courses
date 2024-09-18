with Ada.Text_IO; use Ada.Text_IO;

procedure Exo2 is
   function to_String (N : Integer) return String is
      I        : Integer          := N;
      Idx      : Integer          := 16;
      S        : String (1 .. 16) := (others => '0');
      Was_Zero : Boolean          := False;
   begin
      if I = 0 then
         Idx := 15;
      elsif I < 0 then
         Was_Zero := True;
         I        := -I;
      end if;
      while I /= 0 loop
         S (Idx) := Character'Val (I mod 10 + Character'Pos ('0'));
         I       := I / 10;
         Idx     := Idx - 1;
      end loop;
      if Was_Zero then
         S (Idx) := '-';
      else
         Idx := Idx + 1;
      end if;
      return S (Idx .. S'Length);
   end to_String;
begin
   Put_Line (to_String (-1_234_567));
   Put_Line (to_String (-123));
   Put_Line (to_String (0));
   Put_Line (to_String (123));
   Put_Line (to_String (1_234_567));
end Exo2;
