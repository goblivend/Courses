use std::collections::HashMap;

#[derive(Debug)]
enum Operator {
    Plus,
    Minus,
    Divide,
    Multiply,
}

#[derive(Debug, PartialEq, Clone)]
enum ExprResult {
    Bool(bool),
    Int(i32),
}

trait Expr {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult;
}

struct IntLiteral {
    value: i32,
}

impl Expr for IntLiteral {
    fn eval(&self, _vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        ExprResult::Int(self.value)
    }
}

struct BoolLiteral {
    value: bool,
}

impl Expr for BoolLiteral {
    fn eval(&self, _vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        ExprResult::Bool(self.value)
    }
}

struct BinOp {
    l: Box<dyn Expr>,
    op: Operator,
    r: Box<dyn Expr>,
}

impl Expr for BinOp {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let ExprResult::Int(l) = self.l.eval(vars) else {
            panic!("BinOp l: Expected integer");
        };
        let ExprResult::Int(r) = self.r.eval(vars) else {
            panic!("BinOp r: Expected integer");
        };
        match self.op {
            Operator::Plus => ExprResult::Int(l + r),
            Operator::Minus => ExprResult::Int(l - r),
            Operator::Divide => ExprResult::Int(l / r),
            Operator::Multiply => ExprResult::Int(l * r),
        }
    }
}

struct IfExpr {
    cond: Box<dyn Expr>,
    true_branch: Box<dyn Expr>,
    false_branch: Box<dyn Expr>,
}

impl Expr for IfExpr {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let ExprResult::Bool(res) = self.cond.eval(vars) else {
            panic!("If cond: Expected boolean");
        };
        if res {
            self.true_branch.eval(vars)
        } else {
            self.false_branch.eval(vars)
        }
    }
}

struct Let {
    name: String,
    value: Box<dyn Expr>,
    body: Box<dyn Expr>,
}

impl Expr for Let {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let value = self.value.eval(vars);
        vars.insert(self.name.clone(), value);
        self.body.eval(vars)
    }
}

struct Ref {
    name: String,
}

impl Expr for Ref {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        vars.get(&self.name).unwrap().clone()
    }
}

struct Print {
    expr: Box<dyn Expr>,
}

impl Expr for Print {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let res = self.expr.eval(vars);
        println!("{:?}", res);
        res
    }
}

fn eval(expr: &dyn Expr) -> ExprResult {
    let mut vars = HashMap::new();
    expr.eval(&mut vars)
}

#[cfg(test)]
mod tests {
    use crate::ex5::eval;
    use crate::ex5::BinOp;
    use crate::ex5::BoolLiteral;
    use crate::ex5::ExprResult;
    use crate::ex5::IfExpr;
    use crate::ex5::IntLiteral;
    use crate::ex5::Let;
    use crate::ex5::Operator;
    use crate::ex5::Print;
    use crate::ex5::Ref;

    #[test]
    fn test_literal() {
        let e = IntLiteral { value: 15 };
        assert_eq!(eval(&e), ExprResult::Int(15));
    }

    #[test]
    fn test_plus() {
        let e = BinOp {
            l: Box::new(IntLiteral { value: 15 }),
            op: Operator::Plus,
            r: Box::new(IntLiteral { value: 12 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(27));
    }

    #[test]
    fn test_minus() {
        let e = BinOp {
            l: Box::new(IntLiteral { value: 15 }),
            op: Operator::Minus,
            r: Box::new(IntLiteral { value: 12 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(3));
    }

    #[test]
    fn test_div() {
        let e = BinOp {
            l: Box::new(IntLiteral { value: 15 }),
            op: Operator::Divide,
            r: Box::new(IntLiteral { value: 3 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(5));
    }

    #[test]
    fn test_mul() {
        let e = BinOp {
            l: Box::new(IntLiteral { value: 15 }),
            op: Operator::Multiply,
            r: Box::new(IntLiteral { value: 3 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(45));
    }

    #[test]
    fn test_if() {
        let e = IfExpr {
            cond: Box::new(BoolLiteral { value: true }),
            true_branch: Box::new(IntLiteral { value: 15 }),
            false_branch: Box::new(IntLiteral { value: 12 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(15));
    }

    #[test]
    fn test_let() {
        let e = Let {
            name: "x".to_string(),
            value: Box::new(IntLiteral { value: 15 }),
            body: Box::new(Ref {
                name: "x".to_string(),
            }),
        };
        assert_eq!(eval(&e), ExprResult::Int(15));
    }

    #[test]
    fn test_let_advanced() {
        let e = Let {
            name: "x".to_string(),
            value: Box::new(IntLiteral { value: 15 }),
            body: Box::new(Let {
                name: "y".to_string(),
                value: Box::new(IntLiteral { value: 12 }),
                body: Box::new(BinOp {
                    l: Box::new(Ref {
                        name: "x".to_string(),
                    }),
                    op: Operator::Plus,
                    r: Box::new(Ref {
                        name: "y".to_string(),
                    }),
                }),
            }),
        };
        assert_eq!(eval(&e), ExprResult::Int(27));
    }

    #[test]
    fn test_print() {
        let e = Print {
            expr: Box::new(IntLiteral { value: 15 }),
        };
        assert_eq!(eval(&e), ExprResult::Int(15));
    }
}
