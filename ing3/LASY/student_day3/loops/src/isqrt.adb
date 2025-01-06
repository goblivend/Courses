package body Isqrt
  with SPARK_Mode
is

   function Find_Int_Sqrt (N : in Natural) return Natural is
      Lower, Upper, Middle : Natural;
      Maximum_Root         : constant Natural :=
        46341; -- assumes Natural'Last = 2**31-1.
   begin
      Lower := 0;

      if N >= Maximum_Root then
         Upper := Maximum_Root;
      else
         Upper := N + 1;
      end if;

      loop
         --  Add a pragma Loop_Invariant and a pragma Loop_Variant here.
         pragma
           Loop_Invariant
             (Lower < Upper
                and then Upper <= Maximum_Root
                and then Lower * Lower <= N
                and then Upper * Upper > N);
         exit when Lower + 1 = Upper;

         Middle := Lower + (Upper - Lower) / 2;
         if Middle * Middle > N then
            Upper := Middle;
         else
            Lower := Middle;
         end if;
      end loop;

      pragma Assert (Lower * Lower <= N and (Lower + 1) * (Lower + 1) > N);

      return Lower;
   end Find_Int_Sqrt;

end Isqrt;
