package body Expr_Eval is

   function "+" (L, R : Expr_Access) return Expr_Access is
   begin
      return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Add);
   end "+";

    function "-" (L, R : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Sub);
    end "-";

    function "/" (L, R : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Div);
    end "/";

    function "*" (L, R : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Mul);
    end "*";

    function "or" (L, R : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Logic_Or);
    end "or";

    function "and" (L, R : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => Bin_Op, L => L, R => R, Op => Logic_And);
    end "and";

    function E (Val : Integer) return Expr_Access is
    begin
        return new Expr'(Kind => Literal, Val => Val);
    end E;

    function Iff (Cond, Then_Expr, Else_Expr : Expr_Access) return Expr_Access is
    begin
        return new Expr'(Kind => If_Expr, Cond => Cond, Then_Expr => Then_Expr, Else_Expr => Else_Expr);
    end Iff;

    function Eval (E: Expr) return Integer is
    begin
        case E.Kind is
            when Literal =>
                return E.Val;
            when Bin_Op =>
                declare
                    L : constant Integer := Eval (E.L.all);
                    R : constant Integer := Eval (E.R.all);
                begin
                    case E.Op is
                        when Add =>
                            return L + R;
                        when Sub =>
                            return L - R;
                        when Mul =>
                            return L * R;
                        when Div =>
                            return L / R;
                        when Logic_Or =>
                            if L /= 0 then
                                return 1;
                             else
                                return 0;
                            end if;
                        when Logic_And =>
                            if L /= 0 and R /= 0 then
                                return 1;
                            else
                                return 0;
                            end if;
                    end case;
                end;
            when If_Expr =>
                declare
                    Cond : constant Integer := Eval (E.Cond.all);
                begin
                    if Cond /= 0 then
                        return Eval (E.Then_Expr.all);
                    else
                        return Eval (E.Else_Expr.all);
                    end if;
                end;
        end case;
    end Eval;

end Expr_Eval;
