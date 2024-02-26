import java.util.*;
import java.lang.*;

class Bouncer{
    // FIXME-begin
    static void check_not_null(Integer a, String name){
        if(a == null){
            throw new IllegalArgumentException(name + " is null");
        }
    }

    static void check_negative(Integer a, String name){
        if(a >= 0){
            throw new IllegalArgumentException(name + " is positive");
        }
    }

    static void check_positive(Integer a, String name){
        if(a <= 0){
            throw new IllegalArgumentException(name + " is negative");
        }
    }
    // FIXME-end
}

class Tp2q5
{
    public static Integer product_of_two_negatives(Integer a, Integer b){
        Bouncer.check_not_null(a, "a");
        Bouncer.check_not_null(b, "b");
        Bouncer.check_negative(a, "a");
        Bouncer.check_negative(b, "b");

        Integer res = a * b;

        Bouncer.check_positive(res, "res");
        return  res;
    }

    public static void main(String args[])
    {
        System.out.println(product_of_two_negatives(-1,-2));
        // System.out.println(product_of_two_negatives(-1,2)); //KO java.lang.IllegalArgumentException: b is positive
        // System.out.println(product_of_two_negatives(1,-2)); //KO java.lang.IllegalArgumentException: a is positive
        // System.out.println(product_of_two_negatives(null,-2)); //KO java.lang.IllegalArgumentException: a is null
        // System.out.println(product_of_two_negatives(-1,null)); //KO java.lang.IllegalArgumentException: b is null
    }
}
