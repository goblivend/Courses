package Longest_Common_Prefix
  with SPARK_Mode => On, Initializes => Data
is
   type List_T is array (Positive range <>) of Integer;
   Data : List_T (1 .. 1000) := (others => 0);

   function LCP (X, Y : Positive) return Natural
   with
     Pre => X in Data'Range and Y in Data'Range,
     Post =>
       ((LCP'Result = 0
         or else (X + LCP'Result - 1 in Data'Range
                  and then Y + LCP'Result - 1 in Data'Range))
        and then (for all I in 0 .. LCP'Result - 1
                  => Data (X + I) = Data (Y + I))
        and then (X + LCP'Result > Data'Last
                  or else Y + LCP'Result > Data'Last
                  or else Data (X + LCP'Result) /= Data (Y + LCP'Result))),
     Contract_Cases =>
       (Data (X) /= Data (Y) => LCP'Result = 0,
        X = Y                => LCP'Result = Data'Last - X + 1,
        others               => LCP'Result > 0);

end Longest_Common_Prefix;
