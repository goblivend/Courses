package body Stack
  with SPARK_Mode => On
is
   procedure Reset (S : out Stack_T) is
   begin
      S.Content := (others => 0);
      S.Length := 0;
   end Reset;

   procedure Pop (S : in out Stack_T; E : out Element_T) is
      Idx : constant Stack_Range := S.Length;
   begin
      S.Length := S.Length - 1;
      E := S.Content (Idx);
   end Pop;

   procedure Push (S : in out Stack_T; E : Element_T) is
      Idx : Stack_Range;
   begin
      S.Length := S.Length + 1;
      Idx := S.Length;
      S.Content (Idx) := E;
   end Push;
end Stack;
