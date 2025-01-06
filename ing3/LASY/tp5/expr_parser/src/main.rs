mod ex6;

use ex6::eval;
use ex6::BinOp;
use ex6::BoolLiteral;
use ex6::IfExpr;
use ex6::IntLiteral;
use ex6::Let;
use ex6::Operator;
use ex6::Print;
use ex6::Ref;

#[macro_export]
macro_rules! parse {
    (true) => {
        BoolLiteral { value: true }
    };

    (false) => {
        BoolLiteral { value: false }
    };

    ($l:literal) => {
        IntLiteral { value: $l }
    };

    ($s:ident) => {
        Ref {
            name: stringify!($s).to_string(),
        }
    };

    ((+ $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::Plus,
            r: Box::new(parse!($r)),
        }
    };

    ((- $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::Minus,
            r: Box::new(parse!($r)),
        }
    };

    ((* $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::Multiply,
            r: Box::new(parse!($r)),
        }
    };

    ((/ $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::Divide,
            r: Box::new(parse!($r)),
        }
    };

    ((and $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::And,
            r: Box::new(parse!($r)),
        }
    };

    ((or $l:tt $r:tt)) => {
        BinOp {
            l: Box::new(parse!($l)),
            op: Operator::Or,
            r: Box::new(parse!($r)),
        }
    };

    ((print $expr:tt)) => {
        Print {
            expr: Box::new(parse!($expr)),
        }
    };

    ((if $cond:tt $true:tt $false:tt)) => {
        IfExpr {
            cond: Box::new(parse!($cond)),
            true_branch: Box::new(parse!($true)),
            false_branch: Box::new(parse!($false)),
        }
    };

    ((let $name:ident $value:tt $body:tt)) => {
        Let {
            name: stringify!($name).to_string(),
            value: Box::new(parse!($value)),
            body: Box::new(parse!($body)),
        }
    };
}

fn main() {
    println!("Hello, world!");

    let res = parse!((* (+ 1 22) 3));
    println!("{:?}", eval(&res));
}

// tests
#[cfg(test)]
mod tests {
    use super::*;
    use crate::ex6::ExprResult;

    #[test]
    fn test_parse_simple_literal_int() {
        let res = parse!(1);
        assert_eq!(eval(&res), ExprResult::Int(1));
    }

    #[test]
    fn test_parse_simple_literal_bool() {
        let res = parse!(true);
        assert_eq!(eval(&res), ExprResult::Bool(true));
    }

    #[test]
    fn test_parse_simple_addition() {
        let res = parse!((+ 1 2));
        assert_eq!(eval(&res), ExprResult::Int(3));
    }

    #[test]
    fn test_parse_simple_subtraction() {
        let res = parse!((- 1 2));
        assert_eq!(eval(&res), ExprResult::Int(-1));
    }

    #[test]
    fn test_parse_simple_multiplication() {
        let res = parse!((* 2 3));
        assert_eq!(eval(&res), ExprResult::Int(6));
    }

    #[test]
    fn test_parse_simple_division() {
        let res = parse!((/ 6 2));
        assert_eq!(eval(&res), ExprResult::Int(3));
    }

    #[test]
    fn test_parse_nested_expr() {
        let res = parse!((* (+ 1 22) 3));
        assert_eq!(eval(&res), ExprResult::Int(69));
    }

    #[test]
    fn test_parse_print() {
        let res = parse!((print 1));
        assert_eq!(eval(&res), ExprResult::Int(1));
    }

    #[test]
    fn test_parse_if() {
        let res = parse!((if true 1 2));
        assert_eq!(eval(&res), ExprResult::Int(1));
    }

    #[test]
    fn test_parse_let() {
        let res = parse!((let x 1 x));
        assert_eq!(eval(&res), ExprResult::Int(1));
    }

    #[test]
    fn test_parse_let_nested() {
        let res = parse!((let x 1 (+ x 2)));
        assert_eq!(eval(&res), ExprResult::Int(3));
    }

    #[test]
    fn test_and() {
        let res = parse!((and true true));
        assert_eq!(eval(&res), ExprResult::Bool(true));
    }

    #[test]
    fn test_or() {
        let res = parse!((or false true));
        assert_eq!(eval(&res), ExprResult::Bool(true));
    }
}
