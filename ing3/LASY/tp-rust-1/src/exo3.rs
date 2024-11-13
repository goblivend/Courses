fn string_to_int(s: String) -> i32 {
    let mut i: i32 = 0;
    let mut idx = 0;
    let mut sign: i32 = 1;

    if s.starts_with("-") {
        idx += 1;
        sign = -1;
    }

    while idx < s.len().try_into().unwrap() {
        i = i * 10 + i32::try_from(s.chars().nth(idx).unwrap().to_digit(10).unwrap()).unwrap();
        idx += 1;
    }

    i * sign
}

#[cfg(test)]
mod tests {
    use crate::exo3::string_to_int;

    #[test]
    fn test_to_int_neg() {
        assert_eq!(string_to_int("-1920".to_string()), -1920);
        assert_eq!(string_to_int("-144".to_string()), -144);
        assert_eq!(string_to_int("-10".to_string()), -10);
        assert_eq!(string_to_int("-1".to_string()), -1);
    }

    #[test]
    fn test_to_int_zero() {
        assert_eq!(string_to_int("0".to_string()), 0);
    }

    #[test]
    fn test_to_int_pos() {
        assert_eq!(string_to_int("1".to_string()), 1);
        assert_eq!(string_to_int("10".to_string()), 10);
        assert_eq!(string_to_int("144".to_string()), 144);
        assert_eq!(string_to_int("1920".to_string()), 1920);
    }
}
