--  with Ada.Text_IO; use Ada.Text_IO;
with Expr_Eval; use Expr_Eval;

procedure Ex2 is
   E : Expr_Access := new Bin_Op'(Op => Add, L => new Literal'(Val => 12), R => new Literal'(Val => 15));
begin
   pragma Assert (Eval (E.all) = 27);

   E := E + new Literal'(Val => 3);
   pragma Assert (Eval (E.all) = 30);

   E := E - new Literal'(Val => 5);
   pragma Assert (Eval (E.all) = 25);

   E := E * new Literal'(Val => 2);
   pragma Assert (Eval (E.all) = 50);

   E := new If_Expr'(Cond => new Bin_Op'(Op => Logic_And, L => new Literal'(Val => 1), R => new Literal'(Val => 1)),
                     Then_Expr => new Literal'(Val => 1),
                     Else_Expr => new Literal'(Val => 0));
   pragma Assert (Eval (E.all) = 1);

end Ex2;
