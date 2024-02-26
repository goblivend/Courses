--GNAT 4.9.3

with Ada.Text_IO, Ada.Command_Line;

procedure TP1Q1 is

   -- Variables that can be accessed by all procedures
   -- and functions in this scope.
   X: Positive := 10;
   Res: Integer := 0;
   BB : array (1..2) of Integer := (1, 2);
   Idx : Integer := 1;

   procedure Fib (P :
                      -- FIXME-begin
                      in
                      -- FIXME-end
                      Integer;
                  result :
                      -- FIXME-begin
                      out
                      -- FIXME-end
                      Integer) is
       fib1: Positive := 1;
       fib2: Positive := 1;
   begin
       if P > 2 then
          Fib(P-1, fib1);
          Fib(P-2, fib2);
          result := fib1+fib2;
       else
          result := 1;
       end if;
   end Fib;

   procedure Blackbox(x:
                      -- FIXME-begin
                      in out
                      -- FIXME-end
                      Integer ) is
   begin
       BB(1) := 6;
       Idx   := 2;
       x     := x+3;
   end;

begin

   Ada.Text_IO.Put("Fibonacci(" & Integer'Image(X) & " ) = ");
   Fib(X, Res);
   Ada.Text_IO.Put_Line(Integer'Image(Res));
   Ada.Text_IO.Put_Line("---");

   Ada.Text_IO.Put_Line(Integer'Image(BB(1)));
   Ada.Text_IO.Put_Line(Integer'Image(BB(2)));
   Ada.Text_IO.Put_Line(Integer'Image(Idx));
   Blackbox(BB(Idx));
   Ada.Text_IO.Put_Line(Integer'Image(BB(1)));
   Ada.Text_IO.Put_Line(Integer'Image(BB(2)));
   Ada.Text_IO.Put_Line(Integer'Image(Idx));

end TP1Q1;
