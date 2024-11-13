pub fn int_to_string(i: i32) -> String {
    let mut s: String = "".to_string();
    let mut neg = false;
    let mut n = i;

    if n < 0 {
        neg = true;
        n = -n;
    } else if n == 0 {
        s.insert(0, '0');
    }

    while n > 0 {
        s.insert(
            0,
            char::from_u32(u32::try_from('0').unwrap() + u32::try_from(n % 10).unwrap()).unwrap(),
        );
        n /= 10;
    }

    if neg {
        s.insert(0, '-');
    }
    return s;
}

#[cfg(test)]
mod tests {
    use crate::exo2::int_to_string;

    #[test]
    fn test_to_string_neg() {
        assert_eq!(int_to_string(-1920), "-1920");
        assert_eq!(int_to_string(-144), "-144");
        assert_eq!(int_to_string(-42), "-42");
        assert_eq!(int_to_string(-10), "-10");
        assert_eq!(int_to_string(-1), "-1");
    }

    #[test]
    fn test_to_string_zero() {
        assert_eq!(int_to_string(0), "0");
    }

    #[test]
    fn test_to_string_pos() {
        assert_eq!(int_to_string(1), "1");
        assert_eq!(int_to_string(10), "10");
        assert_eq!(int_to_string(42), "42");
        assert_eq!(int_to_string(144), "144");
        assert_eq!(int_to_string(1920), "1920");
    }
}
