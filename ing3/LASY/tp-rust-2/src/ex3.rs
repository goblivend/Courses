use std::collections::HashMap;

#[derive(Debug, PartialEq)]
enum Operator {
    Plus,
    Minus,
    Divide,
    Multiply,
    And,
    Or,
}

#[derive(Debug, PartialEq, Clone)]
enum ExprResult {
    Bool(bool),
    Int(i32),
}

#[derive(Debug)]
enum Expr {
    BinOp {
        l: Box<Expr>,
        op: Operator,
        r: Box<Expr>,
    },
    IfExpr {
        cond: Box<Expr>,
        true_branch: Box<Expr>,
        false_branch: Box<Expr>,
    },
    Literal(ExprResult),
    Let {
        name: String,
        value: Box<Expr>,
        body: Box<Expr>,
    },
    Ref(String),
}

impl Expr {
    fn eval_impl(expr: Expr, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        match expr {
            Expr::Literal(n) => n,
            Expr::BinOp { l, op, r } => {
                if op == Operator::And {
                    let ExprResult::Bool(l) = Self::eval_impl(*l, vars) else {
                        panic!("Invalid operation: BinOp with non-boolean operands")
                    };
                    if !l {
                        return ExprResult::Bool(false);
                    }
                    let ExprResult::Bool(r) = Self::eval_impl(*r, vars) else {
                        panic!("Invalid operation: BinOp with non-boolean operands")
                    };
                    return ExprResult::Bool(r);
                } else if op == Operator::Or {
                    let ExprResult::Bool(l) = Self::eval_impl(*l, vars) else {
                        panic!("Invalid operation: BinOp with non-boolean operands")
                    };
                    if l {
                        return ExprResult::Bool(true);
                    }
                    let ExprResult::Bool(r) = Self::eval_impl(*r, vars) else {
                        panic!("Invalid operation: BinOp with non-boolean operands")
                    };
                    return ExprResult::Bool(r);
                }
                let ExprResult::Int(l) = Self::eval_impl(*l, vars) else {
                    panic!("Invalid operation: BinOp with non-integer operands")
                };
                let ExprResult::Int(r) = Self::eval_impl(*r, vars) else {
                    panic!("Invalid operation: BinOp with non-integer operands")
                };
                match op {
                    Operator::Plus => ExprResult::Int(l + r),
                    Operator::Minus => ExprResult::Int(l - r),
                    Operator::Divide => ExprResult::Int(l / r),
                    Operator::Multiply => ExprResult::Int(l * r),
                    _ => panic!("Invalid operation: BinOp with non-arithmetic operator"),
                }
            }
            Expr::IfExpr {
                cond,
                true_branch,
                false_branch,
            } => {
                let ExprResult::Bool(res) = Self::eval_impl(*cond, vars) else {
                    panic!("Invalid operation: IfExpr with non-boolean condition")
                };
                if res {
                    Self::eval_impl(*true_branch, vars)
                } else {
                    Self::eval_impl(*false_branch, vars)
                }
            }
            Expr::Let { name, value, body } => {
                let value = Self::eval_impl(*value, vars);
                vars.insert(name, value.clone());
                Self::eval_impl(*body, vars)
            }
            Expr::Ref(name) => vars.get(&name).unwrap().clone(),
        }
    }

    fn eval(expr: Expr) -> ExprResult {
        let mut vars = HashMap::<String, ExprResult>::new();

        Self::eval_impl(expr, &mut vars) // Implement eval_impl
    }
}

#[cfg(test)]
mod tests {
    use crate::ex3::Expr;
    use crate::ex3::ExprResult;
    use crate::ex3::Operator;

    #[test]
    fn test_literal() {
        let e = Expr::Literal(ExprResult::Int(15));
        assert_eq!(Expr::eval(e), ExprResult::Int(15));
    }

    #[test]
    fn test_plus() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Int(15))),
            op: Operator::Plus,
            r: Box::new(Expr::Literal(ExprResult::Int(12))),
        };
        assert_eq!(Expr::eval(e), ExprResult::Int(27));
    }

    #[test]
    fn test_minus() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Int(15))),
            op: Operator::Minus,
            r: Box::new(Expr::Literal(ExprResult::Int(12))),
        };
        assert_eq!(Expr::eval(e), ExprResult::Int(3));
    }

    #[test]
    fn test_div() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Int(15))),
            op: Operator::Divide,
            r: Box::new(Expr::Literal(ExprResult::Int(3))),
        };
        assert_eq!(Expr::eval(e), ExprResult::Int(5));
    }

    #[test]
    fn test_mul() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Int(15))),
            op: Operator::Multiply,
            r: Box::new(Expr::Literal(ExprResult::Int(3))),
        };
        assert_eq!(Expr::eval(e), ExprResult::Int(45));
    }

    #[test]
    fn test_if_0() {
        let e = Expr::IfExpr {
            cond: Box::new(Expr::Literal(ExprResult::Bool(false))),
            true_branch: Box::new(Expr::Literal(ExprResult::Int(1))),
            false_branch: Box::new(Expr::Literal(ExprResult::Int(0))),
        };

        assert_eq!(Expr::eval(e), ExprResult::Int(0));
    }

    #[test]
    fn test_if_1() {
        let e = Expr::IfExpr {
            cond: Box::new(Expr::Literal(ExprResult::Bool(true))),
            true_branch: Box::new(Expr::Literal(ExprResult::Int(1))),
            false_branch: Box::new(Expr::Literal(ExprResult::Int(0))),
        };

        assert_eq!(Expr::eval(e), ExprResult::Int(1));
    }

    #[test]
    fn test_let_simple() {
        let e = Expr::Let {
            name: "x".to_string(),
            value: Box::new(Expr::Literal(ExprResult::Int(1))),
            body: Box::new(Expr::Ref("x".to_string())),
        };

        assert_eq!(Expr::eval(e), ExprResult::Int(1));
    }

    #[test]
    fn test_let_advanced() {
        let e = Expr::Let {
            name: "x".to_string(),
            value: Box::new(Expr::Literal(ExprResult::Int(1))),
            body: Box::new(Expr::BinOp {
                l: Box::new(Expr::Ref("x".to_string())),
                op: Operator::Plus,
                r: Box::new(Expr::Literal(ExprResult::Int(2))),
            }),
        };

        assert_eq!(Expr::eval(e), ExprResult::Int(3));
    }

    #[test]
    fn test_and() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Bool(true))),
            op: Operator::And,
            r: Box::new(Expr::Literal(ExprResult::Bool(false))),
        };

        assert_eq!(Expr::eval(e), ExprResult::Bool(false));
    }

    #[test]
    fn test_or() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(ExprResult::Bool(true))),
            op: Operator::Or,
            r: Box::new(Expr::Literal(ExprResult::Bool(false))),
        };

        assert_eq!(Expr::eval(e), ExprResult::Bool(true));
    }
}
