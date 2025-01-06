package Max with SPARK_Mode Is

   Min_Table_Size : constant := 1;
   Max_Table_Size : constant := 100;
   Min_Content    : constant := -200;
   Max_Content    : constant := 200;

   subtype Extended_Range is Positive range Min_Table_Size .. Max_Table_Size + 1;
   subtype Index_Range    is Positive range Min_Table_Size .. Max_Table_Size;
   type Content_Range     is range Min_Content .. Max_Content;
   type Base_Array        is array (Positive range <>) of Content_Range;
   subtype Our_Array      is Base_Array (Min_Table_Size .. Max_Table_Size);

   function Arrays_Max (A : in Our_Array) return Index_Range
   with Post => (for all N in Index_Range => A (Arrays_Max'Result) >= A (N));
   --  Add a loop invariant and variant in Arrays_Max to make it prove

   function Is_Sorted (A : Base_Array) return Boolean is
     (for all N in A'Range =>
        (for all M in A'Range => (if N < M then A (N) <= A (M))));

   subtype Merged_Range is Positive range Min_Table_Size .. Max_Table_Size * 2;
   subtype Merged_Array is Base_Array (Min_Table_Size .. Max_Table_Size * 2);

   procedure Merge (A, B : Our_Array; Res : in out Merged_Array) with
     Pre  => Is_Sorted (A) and Is_Sorted (B),
     Post => Is_Sorted (Res);
   --  Add a loop invariant in Merge to make it prove

end Max;
