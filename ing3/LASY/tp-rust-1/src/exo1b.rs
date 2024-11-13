pub fn invert(s: &mut String) {
    *s = s.chars().rev().collect();
}

#[cfg(test)]
mod tests {
    use crate::exo1b::invert;

    fn test_invert(s1: &str, s2 : &str) {
        let mut s: String = s1.to_string();
        invert(&mut s);
        assert_eq!(s, s2);

    }

    #[test]
    fn tests() {
        test_invert("", "");
        test_invert("ABC", "CBA");
        test_invert("Hello World!", "!dlroW olleH");
    }
}
