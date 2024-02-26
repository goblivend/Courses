import java.util.*;
import java.lang.*;

// FIXME-begin
interface IsACat
{
    public abstract void meow();
}
// FIXME-end

class Tiger
    // FIXME-begin
    implements IsACat
    // FIXME-end
{
    public void meow(){
        System.out.println("Grrrr");
    }
}

class Kitty
    // FIXME-begin
    implements IsACat
    // FIXME-end
{
    public void meow(){
        System.out.println("Meow!");
    }
}

class Spider
    // FIXME-begin
    // FIXME-end
{
    public void meow(){
        System.out.println("Try to meow!");
    }
}


class CatManagement
{
    // FIXME-begin
    public static <T extends IsACat> void make_it_meow(T cat){
        cat.meow();
    }
    // FIXME-end
}

public class Tp3q1
{

    public static void main(String args[])
    {
        CatManagement.make_it_meow(new Tiger());
        CatManagement.make_it_meow(new Kitty());
        // CatManagement.make_it_meow(new Spider()); error inferred type does not conform to upper bound(s)
    }
}
