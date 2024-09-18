with Ada.Text_IO; use Ada.Text_IO;

procedure Exo3 is
   function To_Integer (S : String) return Integer is
      I        : Integer := 0;
      Was_Zero : Boolean := False;
      Idx      : Integer := 1;
   begin
      if S (Idx) = '-' then
         Was_Zero := True;
         Idx      := Idx + 1;
      end if;

      while Idx <= S'Length loop
         if S (Idx) /= '_' then
            I := I * 10 + (Character'Pos (S (Idx)) - Character'Pos ('0'));
         end if;
         Idx := Idx + 1;
      end loop;

      if Was_Zero then
         I := -I;
      end if;
      return I;
   end To_Integer;
begin
   Put_Line (To_Integer ("-1_234_567")'Image);
   Put_Line (To_Integer ("-123")'Image);
   Put_Line (To_Integer ("0")'Image);
   Put_Line (To_Integer ("123")'Image);
   Put_Line (To_Integer ("1_234_567")'Image);
end Exo3;
