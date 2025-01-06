use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone)]
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

use lexpr::{self, Value};

fn parse(value: &Value) -> Expr {
    match value {
        Value::Number(n) => Expr::Literal(ExprResult::Int(n.as_i64().unwrap() as i32)),
        Value::Bool(b) => Expr::Literal(ExprResult::Bool(*b)),
        Value::Symbol(s) => Expr::Ref(s.to_string()),
        Value::Cons(l) => {
            let head = l.car();

            let Value::Cons(head2) = l.cdr() else {
                panic!("Expected a cons cell")
            };

            let Value::Cons(head3) = head2.cdr() else {
                panic!("Expected a cons cell")
            };

            match head {
                Value::Symbol(s) => match &**s {
                    // Dereference Box<str> to &str
                    "+" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::Plus,
                        r: Box::new(parse(head3.car())),
                    },
                    "-" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::Minus,
                        r: Box::new(parse(head3.car())),
                    },
                    "*" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::Multiply,
                        r: Box::new(parse(head3.car())),
                    },
                    "/" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::Divide,
                        r: Box::new(parse(head3.car())),
                    },
                    "and" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::And,
                        r: Box::new(parse(head3.car())),
                    },
                    "or" => Expr::BinOp {
                        l: Box::new(parse(head2.car())),
                        op: Operator::Or,
                        r: Box::new(parse(head3.car())),
                    },
                    "if" => {
                        let Value::Cons(head4) = head3.cdr() else {
                            panic!("Expected a cons cell")
                        };
                        Expr::IfExpr {
                            cond: Box::new(parse(head2.car())),
                            true_branch: Box::new(parse(head3.car())),
                            false_branch: Box::new(parse(head4.car())),
                        }
                    }
                    "let" => {
                        let name = head2.car().as_symbol().unwrap().to_string();
                        let value = Box::new(parse(head3.car()));
                        let Value::Cons(head4) = head3.cdr() else {
                            panic!("Expected a cons cell")
                        };
                        let body = Box::new(parse(head4.car()));
                        Expr::Let { name, value, body }
                    }
                    _ => panic!("Unknown operator"),
                },
                _ => panic!("Expected a symbol"),
            }
        }
        e => panic!("Unexpected value type {:?}", e),
    }
}

#[cfg(test)]
mod tests {
    use crate::ex4::parse;
    use crate::ex4::Expr;
    use crate::ex4::ExprResult;

    #[test]
    fn test_literal_parser() {
        let e = parse(&lexpr::from_str("42").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(42));
    }

    #[test]
    fn test_plus_parser() {
        let e = parse(&lexpr::from_str("(+ 15 12)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(27));
    }

    #[test]
    fn test_plus_advanced_parser() {
        let e = parse(&lexpr::from_str("(+ (+ 1 1) 12)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(14));
    }

    #[test]
    fn test_minus_parser() {
        let e = parse(&lexpr::from_str("(- 15 12)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(3));
    }

    #[test]
    fn test_div_parser() {
        let e = parse(&lexpr::from_str("(/ 15 3)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(5));
    }

    #[test]
    fn test_if_parser() {
        let e = parse(&lexpr::from_str("(if #t 42 0)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(42));
    }

    #[test]
    fn test_let_parser() {
        let e = parse(&lexpr::from_str("(let x 42 x)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(42));
    }

    #[test]
    fn test_advanced_let_parser() {
        let e = parse(&lexpr::from_str("(let x 42 (+ x 1))").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Int(43));
    }

    #[test]
    fn test_and_parser() {
        let e = parse(&lexpr::from_str("(and #t #t)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Bool(true));
    }

    #[test]
    fn test_or_parser() {
        let e = parse(&lexpr::from_str("(or #f #t)").unwrap());
        assert_eq!(Expr::eval(e), ExprResult::Bool(true));
    }
}
