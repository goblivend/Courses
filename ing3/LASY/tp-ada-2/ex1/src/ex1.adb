with Ada.Text_IO; use Ada.Text_IO;
with Expr_Eval;   use Expr_Eval;

procedure Ex1 is
   pragma Assertion_Policy (Check);
   E : Expr_Access :=
     new Expr'
       (Kind => Bin_Op,
        L    => new Expr'(Kind => Literal, Val => 12),
        R    => new Expr'(Kind => Literal, Val => 15),
        Op   => Add);
begin
   pragma Assert (Eval (E.all) = 27);

   E := E + new Expr'(Kind => Literal, Val => 3);
   pragma Assert (Eval (E.all) = 30);

   E := E - new Expr'(Kind => Literal, Val => 5);
   pragma Assert (Eval (E.all) = 25);

   E := E * new Expr'(Kind => Literal, Val => 2);
   pragma Assert (Eval (E.all) = 50);

   E := E or E (12);
   pragma Assert (Eval (E.all) = 42);
end Ex1;
