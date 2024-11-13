package Expr_Eval is

   type Expr;

   type Op_Kind is (Add, Sub, Mul, Div, Logic_And, Logic_Or);
   type Expr_Access is access Expr'Class;

   type Expr is abstract tagged null record;
   function Eval (E: Expr) return Integer is abstract;

   type Bin_Op is new Expr with record
      L, R : Expr_Access;
      Op   : Op_Kind;
   end record;
   overriding function Eval (B : Bin_Op) return Integer;

   type If_Expr is new Expr with record
      Cond, Then_Expr, Else_Expr : Expr_Access;
   end record;
   overriding function Eval (I : If_Expr) return Integer;

   type Literal is new Expr with record
      Val : Integer;
   end record;
   overriding function Eval (L : Literal) return Integer;

   function "+" (L, R : Expr_Access) return Expr_Access;
   function "-" (L, R : Expr_Access) return Expr_Access;
   function "/" (L, R : Expr_Access) return Expr_Access;
   function "*" (L, R : Expr_Access) return Expr_Access;
   function "or" (L, R : Expr_Access) return Expr_Access;
   function "and" (L, R : Expr_Access) return Expr_Access;
   function E (Val : Integer) return Expr_Access;
   function Iff (Cond, Then_Expr, Else_Expr : Expr_Access) return Expr_Access;

end Expr_Eval;
