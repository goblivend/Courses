#[derive(Debug)]
pub struct Stack {
    arr: [i32; 512],
    size: i32,
}

impl Stack {
    fn new() -> Self {
        Stack {
            arr: [0; 512],
            size: 0,
        }
    }

    fn pop(&mut self) -> Option<i32> {
        if self.size == 0 {
            return Option::None;
        }
        self.size -= 1;
        Some(self.arr[self.size as usize])
    }

    fn push(&mut self, value: i32) -> bool {
        if self.size == 512 {
            return false;
        }
        self.arr[self.size as usize] = value;
        self.size += 1;
        true
    }

    fn peek(&mut self) -> Option<i32> {
        if self.size == 0 {
            return Option::None;
        }
        Some(self.arr[(self.size - 1) as usize])
    }
}

#[cfg(test)]
mod tests {
    use crate::exo5::Stack;

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
