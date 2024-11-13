#[derive(Debug)]
pub struct Stack {
    arr: Vec<i32>,
}

impl Stack {
    fn new() -> Self {
        Stack { arr: vec![] }
    }

    fn pop(&mut self) -> Option<i32> {
        self.arr.pop()
    }

    fn push(&mut self, value: i32) -> bool {
        self.arr.push(value);
        true
    }

    fn peek(&mut self) -> Option<i32> {
        if self.arr.is_empty() {
            None
        } else {
            Some(self.arr[self.arr.len() - 1])
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::exo6::Stack;

    #[test]
    fn test() {
        let mut s: Stack = Stack::new();
        assert_eq!(s.peek(), None);

        assert_eq!(s.pop(), None);
        assert_eq!(s.push(5), true);
        assert_eq!(s.peek(), Some(5));
        assert_eq!(s.pop(), Some(5));
        assert_eq!(s.peek(), None)
    }
}
