#[derive(Debug)]
struct SortedList<T: Ord> {
    data: Vec<T>,
    size: usize,
}

impl<T: Ord> SortedList<T> {
    pub fn new() -> SortedList<T> {
        SortedList {
            data: vec![],
            size: 0,
        }
    }

    pub fn add(&mut self, value: T) {
        let mut i = 0;
        while i < self.size && self.data[i] < value {
            i += 1;
        }
        self.data.insert(i, value);
        self.size += 1;
    }
}

#[cfg(test)]
mod tests {
    use crate::ex1::SortedList;

    #[test]
    fn test_one_element() {
        let mut list = SortedList::new();
        list.add(1);
        assert_eq!(list.data, vec![1]);
    }

    #[test]
    fn test_two_elements() {
        let mut list = SortedList::new();
        list.add(2);
        list.add(1);
        assert_eq!(list.data, vec![1, 2]);
    }

    #[test]
    fn test_many_elements() {
        let mut list = SortedList::new();
        list.add(3);
        list.add(1);
        list.add(2);
        list.add(4);
        list.add(5);
        assert_eq!(list.data, vec![1, 2, 3, 4, 5]);
    }
}
