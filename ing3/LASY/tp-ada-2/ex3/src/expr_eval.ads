package Expr_Eval is

   type Expr;

   type Op_Kind is (Add, Sub, Mul, Div, Logic_And, Logic_Or);
   type Expr_Access is access Expr'Class;

   subtype Var_Name is String (1 .. 2);



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


   type Let is new Expr with record
      Bind        : Var_Name;
      Bound_Value : Expr_Access;
      Target_Expr : Expr_Access;
   end record;
   overriding function Eval (L : Let) return Integer;

   type Ref is new Expr with record
      Ref : Var_Name;
   end record;
   overriding function Eval (R : Ref) return Integer;


   function "+" (L, R : Expr_Access) return Expr_Access;
   function "-" (L, R : Expr_Access) return Expr_Access;
   function "/" (L, R : Expr_Access) return Expr_Access;
   function "*" (L, R : Expr_Access) return Expr_Access;
   function "or" (L, R : Expr_Access) return Expr_Access;
   function "and" (L, R : Expr_Access) return Expr_Access;
   function E (Val : Integer) return Expr_Access;
   function Iff (Cond, Then_Expr, Else_Expr : Expr_Access) return Expr_Access;

end Expr_Eval;
