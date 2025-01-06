use std::collections::HashMap;

#[derive(Debug, PartialEq, Clone)]
pub enum Operator {
    Plus,
    Minus,
    Divide,
    Multiply,
    And,
    Or,
}

#[derive(Debug, PartialEq, Clone)]
pub enum ExprResult {
    Bool(bool),
    Int(i32),
}

#[derive(Debug, PartialEq, Clone)]
pub enum ExprType {
    Bool,
    Int,
}

pub trait Expr {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult;
    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str>;
}

pub struct IntLiteral {
    pub value: i32,
}

impl Expr for IntLiteral {
    fn eval(&self, _vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        ExprResult::Int(self.value)
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        Ok(ExprType::Int)
    }
}

pub struct BoolLiteral {
    pub value: bool,
}

impl Expr for BoolLiteral {
    fn eval(&self, _vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        ExprResult::Bool(self.value)
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        Ok(ExprType::Bool)
    }
}

pub struct BinOp {
    pub l: Box<dyn Expr>,
    pub op: Operator,
    pub r: Box<dyn Expr>,
}

impl Expr for BinOp {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        if self.op == Operator::And {
            let ExprResult::Bool(l) = self.l.eval(vars) else {
                panic!("BinOp l: Expected boolean");
            };
            if !l {
                return ExprResult::Bool(false);
            }
            let ExprResult::Bool(r) = self.r.eval(vars) else {
                panic!("BinOp r: Expected boolean");
            };
            return ExprResult::Bool(r);
        } else if self.op == Operator::Or {
            let ExprResult::Bool(l) = self.l.eval(vars) else {
                panic!("BinOp l: Expected boolean");
            };
            if l {
                return ExprResult::Bool(true);
            }
            let ExprResult::Bool(r) = self.r.eval(vars) else {
                panic!("BinOp r: Expected boolean");
            };
            return ExprResult::Bool(r);
        }
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
            _ => panic!("BinOp: Invalid operator"),
        }
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        let l_type = self.l.check_types(vars);
        let r_type = self.r.check_types(vars);
        if let Err(e) = l_type {
            return Err(e);
        }

        if let Err(e) = r_type {
            return Err(e);
        }

        if self.op == Operator::And || self.op == Operator::Or {
            if l_type == Ok(ExprType::Bool) && r_type == Ok(ExprType::Bool) {
                return Ok(ExprType::Bool);
            } else {
                return Err("BinOp: Expected Bool types");
            }
        } else {
            if r_type == Ok(ExprType::Int) && l_type == Ok(ExprType::Int) {
                l_type
            } else {
                Err("BinOp: Expected Int types")
            }
        }
    }
}

pub struct IfExpr {
    pub cond: Box<dyn Expr>,
    pub true_branch: Box<dyn Expr>,
    pub false_branch: Box<dyn Expr>,
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

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        let cond_type = self.cond.check_types(vars);
        if let Err(e) = cond_type {
            return Err(e);
        }

        let true_type = self.true_branch.check_types(vars);
        if let Err(e) = true_type {
            return Err(e);
        }

        let false_type = self.false_branch.check_types(vars);
        if let Err(e) = false_type {
            return Err(e);
        }

        if cond_type == Ok(ExprType::Bool) && true_type == false_type {
            true_type
        } else {
            Err("IfExpr: Expected Bool type for condition and same type for both branches")
        }
    }
}

pub struct Let {
    pub name: String,
    pub value: Box<dyn Expr>,
    pub body: Box<dyn Expr>,
}

impl Expr for Let {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let value = self.value.eval(vars);
        vars.insert(self.name.clone(), value);
        self.body.eval(vars)
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        match self.value.check_types(vars) {
            Ok(value_type) => {
                vars.insert(self.name.clone(), value_type);
                self.body.check_types(vars)
            }
            Err(e) => Err(e),
        }
    }
}

pub struct Ref {
    pub name: String,
}

impl Expr for Ref {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        vars.get(&self.name).unwrap().clone()
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        match vars.get(&self.name) {
            Some(t) => Ok(t.clone()),
            None => Err("Ref: Unknown variable"),
        }
    }
}

pub struct Print {
    pub expr: Box<dyn Expr>,
}

impl Expr for Print {
    fn eval(&self, vars: &mut HashMap<String, ExprResult>) -> ExprResult {
        let res = self.expr.eval(vars);
        println!("{:?}", res);
        res
    }

    fn check_types(&self, vars: &mut HashMap<String, ExprType>) -> Result<ExprType, &'static str> {
        self.expr.check_types(vars)
    }
}

pub fn eval(expr: &dyn Expr) -> ExprResult {
    let mut vars = HashMap::new();
    expr.eval(&mut vars)
}

pub fn check_types(expr: &dyn Expr) -> Result<ExprType, &'static str> {
    let mut vars = HashMap::new();
    expr.check_types(&mut vars)
}

#[cfg(test)]
mod tests {
    use crate::ex6::check_types;
    use crate::ex6::BinOp;
    use crate::ex6::BoolLiteral;
    use crate::ex6::ExprType;
    use crate::ex6::IfExpr;
    use crate::ex6::IntLiteral;
    use crate::ex6::Let;
    use crate::ex6::Operator;
    use crate::ex6::Print;
    use crate::ex6::Ref;

    #[test]
    fn test_literal() {
        let expr = IntLiteral { value: 42 };
        assert_eq!(check_types(&expr), Ok(ExprType::Int));
    }

    #[test]
    fn test_binop() {
        let expr = BinOp {
            l: Box::new(IntLiteral { value: 42 }),
            op: Operator::Plus,
            r: Box::new(IntLiteral { value: 42 }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Int));
    }

    #[test]
    fn test_if() {
        let expr = IfExpr {
            cond: Box::new(BoolLiteral { value: true }),
            true_branch: Box::new(IntLiteral { value: 42 }),
            false_branch: Box::new(IntLiteral { value: 42 }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Int));
    }

    #[test]
    fn test_let() {
        let expr = Let {
            name: "x".to_string(),
            value: Box::new(IntLiteral { value: 42 }),
            body: Box::new(Ref {
                name: "x".to_string(),
            }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Int));
    }

    #[test]
    fn test_unknown_ref() {
        let expr = Ref {
            name: "x".to_string(),
        };
        assert_eq!(check_types(&expr), Err("Ref: Unknown variable"));
    }

    #[test]
    fn test_print() {
        let expr = Print {
            expr: Box::new(IntLiteral { value: 42 }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Int));
    }

    #[test]
    fn test_print_ref() {
        let expr = Print {
            expr: Box::new(Ref {
                name: "x".to_string(),
            }),
        };
        assert_eq!(check_types(&expr), Err("Ref: Unknown variable"));
    }

    #[test]
    fn test_fail_binop() {
        let expr = BinOp {
            l: Box::new(IntLiteral { value: 42 }),
            op: Operator::Plus,
            r: Box::new(BoolLiteral { value: true }),
        };
        assert_eq!(check_types(&expr), Err("BinOp: Expected Int types"));
    }

    #[test]
    fn test_and_valid() {
        let expr = BinOp {
            l: Box::new(BoolLiteral { value: true }),
            op: Operator::And,
            r: Box::new(BoolLiteral { value: true }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Bool));
    }

    #[test]
    fn test_and_invalid() {
        let expr = BinOp {
            l: Box::new(BoolLiteral { value: true }),
            op: Operator::And,
            r: Box::new(IntLiteral { value: 42 }),
        };
        assert_eq!(check_types(&expr), Err("BinOp: Expected Bool types"));
    }

    #[test]
    fn test_or_valid() {
        let expr = BinOp {
            l: Box::new(BoolLiteral { value: true }),
            op: Operator::Or,
            r: Box::new(BoolLiteral { value: true }),
        };
        assert_eq!(check_types(&expr), Ok(ExprType::Bool));
    }

    #[test]
    fn test_or_invalid() {
        let expr = BinOp {
            l: Box::new(BoolLiteral { value: true }),
            op: Operator::Or,
            r: Box::new(IntLiteral { value: 42 }),
        };
        assert_eq!(check_types(&expr), Err("BinOp: Expected Bool types"));
    }
}
