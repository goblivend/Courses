package body Test with SPARK_Mode => On
is
   function Prove_The_Absence_Of_Run_Time_Errors ( Input : Integer) return Integer is
      X, Y, K : Integer;
   begin
      K := Input / 100;
      X := 2;
      Y := K + 5;
      while X < 10 loop
         X := X + 1;
         Y := Y + 3;
      end loop;

      if (3 * K + 100) > 43 then
         Y := Y + 1;
         X := X / (X - Y);
      end if;
      return X;
   end Prove_The_Absence_Of_Run_Time_Errors;
end Test;
