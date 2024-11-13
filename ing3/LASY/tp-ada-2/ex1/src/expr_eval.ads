package Expr_Eval is

    type Expr;

    type Expr_Kind is (Bin_Op, Literal, If_Expr);
    type Op_Kind is (Add, Sub, Mul, Div, Logic_And, Logic_Or);
    type Expr_Access is access Expr;

    type Expr (Kind : Expr_Kind) is record
      case Kind is
         when Bin_Op =>
            L, R : Expr_Access;
            Op   : Op_Kind;
         when If_Expr =>
            Cond, Then_Expr, Else_Expr : Expr_Access;
         when Literal =>
            Val : Integer;
      end case;
    end record;

    function "+" (L, R : Expr_Access) return Expr_Access;
    function "-" (L, R : Expr_Access) return Expr_Access;
    function "/" (L, R : Expr_Access) return Expr_Access;
    function "*" (L, R : Expr_Access) return Expr_Access;
    function "or" (L, R : Expr_Access) return Expr_Access;
    function "and" (L, R : Expr_Access) return Expr_Access;
    function E (Val : Integer) return Expr_Access;
    function Iff (Cond, Then_Expr, Else_Expr : Expr_Access) return Expr_Access;

    function Eval (E: Expr) return Integer;



end Expr_Eval;
