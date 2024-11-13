#[derive(Debug)]
enum Operator {
    Plus,
    Minus,
    Divide,
    Multiply,
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
    Literal(i32),
}

impl Expr {
    fn eval(&self) -> i32 {
        match &self {
            Expr::Literal(n) => *n,
            Expr::BinOp { l, op, r } => match *op {
                Operator::Plus => l.eval() + r.eval(),
                Operator::Minus => l.eval() - r.eval(),
                Operator::Divide => l.eval() / r.eval(),
                Operator::Multiply => l.eval() * r.eval(),
            },
            &Expr::IfExpr {
                cond,
                true_branch,
                false_branch,
            } => {
                if cond.eval() != 0 {
                    true_branch.eval()
                } else {
                    false_branch.eval()
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::exo8::Expr;
    use crate::exo8::Operator;

    #[test]
    fn test_plus() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(15)),
            op: Operator::Plus,
            r: Box::new(Expr::Literal(12)),
        };
        assert_eq!(e.eval(), 27);
    }

    #[test]
    fn test_minus() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(15)),
            op: Operator::Minus,
            r: Box::new(Expr::Literal(12)),
        };
        assert_eq!(e.eval(), 3);
    }

    #[test]
    fn test_div() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(15)),
            op: Operator::Divide,
            r: Box::new(Expr::Literal(3)),
        };
        assert_eq!(e.eval(), 5);
    }

    #[test]
    fn test_mul() {
        let e = Expr::BinOp {
            l: Box::new(Expr::Literal(15)),
            op: Operator::Multiply,
            r: Box::new(Expr::Literal(3)),
        };
        assert_eq!(e.eval(), 45);
    }

    #[test]
    fn test_if_0() {
        let e = Expr::IfExpr {
            cond: Box::new(Expr::Literal(0)),
            true_branch: Box::new(Expr::Literal(1)),
            false_branch: Box::new(Expr::Literal(0)),
        };

        assert_eq!(e.eval(), 0);
    }

    #[test]
    fn test_if_1() {
        let e = Expr::IfExpr {
            cond: Box::new(Expr::Literal(1)),
            true_branch: Box::new(Expr::Literal(1)),
            false_branch: Box::new(Expr::Literal(0)),
        };

        assert_eq!(e.eval(), 1);
    }
}
