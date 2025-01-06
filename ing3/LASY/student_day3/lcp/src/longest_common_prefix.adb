package body Longest_Common_Prefix
  with SPARK_Mode => On
is
   function LCP (X, Y : Positive) return Natural is
      L : Natural;
   begin
      L := 0;
      while X + L <= Data'Last
        and then Y + L <= Data'Last
        and then Data (X + L) = Data (Y + L)
      loop
         L := L + 1;
         pragma
           Loop_Invariant
             (X + L - 1 in Data'Range
                and then Y + L - 1 in Data'Range
                and then (for all I in 0 .. L - 1
                          => Data (X + I) = Data (Y + I)));

         pragma Loop_Variant (Increases => L);
      end loop;
      pragma
        Assert
          ((L = 0
            or else (X + L - 1 in Data'Range and then Y + L - 1 in Data'Range))
             and then (L = 0
                       or else (for all I in 0 .. L - 1
                                => Data (X + I) = Data (Y + I)))
             and then (X + L > Data'Last
                       or else Y + L > Data'Last
                       or else Data (X + L) /= Data (Y + L)));
      return L;
   end LCP;

end Longest_Common_Prefix;
