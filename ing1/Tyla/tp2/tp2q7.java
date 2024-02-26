import java.util.*;
import java.lang.*;
import java.util.ArrayList;
import java.util.List;

class A {
}

class B extends A {
}

class C extends B {
}

class Tp2q7
{
    // FIXME-begin
    public static List<Class<?>> listInheritance(Class<?> c){
        List<Class<?>> list = new ArrayList<Class<?>>();
        list.add(c);
        while(c.getSuperclass() != null){
            list.add(c.getSuperclass());
            c = c.getSuperclass();
        }
        return list;
    }
    // FIXME-end

    public static void main(String args[])
    {
        for (Class<?> c : listInheritance(C.class))
            System.out.println(c.getName());

        System.out.println("---");

        for (Class<?> c : listInheritance(B.class))
            System.out.println(c.getName());
        System.out.println("---");
    }
}
