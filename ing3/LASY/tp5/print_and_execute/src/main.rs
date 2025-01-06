#[macro_export]
macro_rules! print_and_execute {
    ($($x:expr),*) => {
        $(
            println!("{} evals to {}", stringify!($x), $x);
        )*
    };
}

fn main() {
    print_and_execute!(3, 34, "hello", {
        let x = 34;
        x
    });
}
