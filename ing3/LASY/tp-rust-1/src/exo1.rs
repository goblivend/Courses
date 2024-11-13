pub fn invert(s: &String) -> String {
    s.chars().rev().collect()
}


#[cfg(test)]
mod tests {
    use crate::exo1::invert;

    #[test]
    fn test_invert() {
        assert_eq!(invert(&"".to_string()), "");
        assert_eq!(invert(&"Hello".to_string()), "olleH");
        assert_eq!(invert(&"Hello World!".to_string()), "!dlroW olleH");
    }
}
