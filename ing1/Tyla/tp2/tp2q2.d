// TP2Q2

import std.stdio;
import std.math;

long square_root(long x)
in {
    // Ensure that x is positive

    // FIXME-begin
    assert(x >= 0, "argument must be positive");
    // FIXME-end
}
out (result) {
    // Ensure that result^2 <= x < (result+1)^2
    // Otherwise raise "not a valid square root"

    // FIXME-begin
    assert(result * result <= x && x < (result + 1) * (result + 1), "not a valid square root");
    // FIXME-end
}
body {
    // FIXME-begin
    return cast(long) sqrt(cast(real) x);
    // FIXME-end
}

void main()
{
   writeln(square_root(42)); // OK displays 6
    // square_root(-42); // Fails with error "argument must be positive"
}
