package body Expr_Eval is

   overriding function Eval (B : Bin_Op) return Integer is
   begin
      declare
         L : constant Integer := Eval (B.L.all);
         R : constant Integer := Eval (B.R.all);
      begin
         case B.Op is
            when Add =>
               return L + R;
            when Sub =>
               return L - R;
            when Mul =>
               return L * R;
            when Div =>
               return L / R;
            when Logic_Or =>
               if L /= 0 or else R /= 0 then
                  return 1;
               else
                  return 0;
               end if;
            when Logic_And =>
               if L /= 0 and then R /= 0 then
                  return 1;
               else
                  return 0;
               end if;
         end case;
      end;
   end Eval;

   overriding function Eval (I : If_Expr) return Integer is
   begin
      if Eval (I.Cond.all) /= 0 then
         return Eval (I.Then_Expr.all);
      else
         return Eval (I.Else_Expr.all);
      end if;
   end Eval;

   overriding function Eval (L : Literal) return Integer is
   begin
      return L.Val;
   end Eval;

   function "+" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Add, L => L, R => R);
   end "+";

   function "-" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Sub, L => L, R => R);
   end "-";

   function "/" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Div, L => L, R => R);
   end "/";

   function "*" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Mul, L => L, R => R);
   end "*";

   function "or" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Logic_Or, L => L, R => R);
   end "or";

   function "and" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Bin_Op'(Op => Logic_And, L => L, R => R);
   end "and";

   function E (Val : Integer) return Expr_Access is
   begin
      return new Literal'(Val => Val);
   end E;

   function Iff (Cond, Then_Expr, Else_Expr : Expr_Access) return Expr_Access is
   begin
      return new If_Expr'(Cond => Cond, Then_Expr => Then_Expr, Else_Expr => Else_Expr);
   end Iff;



end Expr_Eval;
