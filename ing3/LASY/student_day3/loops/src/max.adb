package body Max
  with SPARK_Mode
is

   function Arrays_Max (A : in Our_Array) return Index_Range is
      X : Index_Range := Index_Range'First;
      Y : Index_Range := Index_Range'Last;
   begin
      while X /= Y loop
         --  Write a suitable Loop_Invariant to help prove the postcondition
         pragma
           Loop_Invariant
             (X in Index_Range
                and then Y in Index_Range
                and then X < Y
                and then (for all N in Index_Range'First .. X
                          => (A (X) >= A (N) or else A (Y) >= A (N)))
                and then (for all N in Y .. Index_Range'Last
                          => (A (X) >= A (N) or else A (Y) >= A (N))));
         --  Write a suitable Loop_Variant to help prove loop termination

         if A (X) <= A (Y) then
            X := X + 1;
         else
            Y := Y - 1;
         end if;
      end loop;
      pragma Assert ((for all N in Index_Range => A (X) >= A (N)));
      return X;
   end Arrays_Max;

   procedure Merge (A, B : Our_Array; Res : in out Merged_Array) is
      X : Extended_Range := A'First;
      Y : Extended_Range := B'First;
   begin
      for K in Merged_Range loop

         if Y not in B'Range or else (X in A'Range and then A (X) <= B (Y))
         then
            Res (K) := A (X);
            X := X + 1;
         else
            Res (K) := B (Y);
            Y := Y + 1;
         end if;

         --  Write a suitable Loop_Invariant to help prove the postcondition

         pragma
           Loop_Invariant
             (K
                = (if X in A'Range then X - 1 else A'Last)
                  + (if Y in B'Range then Y - 1 else B'Last));

         pragma
           Loop_Invariant
             ((for all N in Res'First .. K
               => (for all M in Res'First .. K
                   => (if N < M then Res (N) <= Res (M)))));

         pragma Loop_Invariant (X not in A'Range or else Res (K) <= A (X));
         pragma Loop_Invariant (Y not in B'Range or else Res (K) <= B (Y));

      end loop;
   end Merge;

end Max;
