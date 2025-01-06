package Isqrt with SPARK_Mode
is
  -- The function below calculates the integer square root of a
  -- natural number.

   function Find_Int_Sqrt (N : in Natural) return Natural
   with
     Post => Find_Int_Sqrt'Result * Find_Int_Sqrt'Result <= N and
       (Find_Int_Sqrt'Result + 1) * (Find_Int_Sqrt'Result + 1) > N;

end Isqrt;
