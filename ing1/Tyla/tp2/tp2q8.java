import java.lang.annotation.*;
import java.lang.reflect.*;
import java.util.*;
import java.lang.*;
import java.util.ArrayList;
import java.util.List;


@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
@interface MyTaggedMethod {
    public String value() default "";
}

class A{
    @MyTaggedMethod(value="first tag")
    public void foo(){}

    // Not tagged !
    public void bar(){}

    @MyTaggedMethod(value="second tag")
    public void foobar(){}
}

class Tp2q8
{
    // FIXME-begin
    public static List<Method> findMyTaggedMethods(Class<?> c){
        List<Method> list = new ArrayList<Method>();
        for(Method m : c.getDeclaredMethods()){
            if(m.isAnnotationPresent(MyTaggedMethod.class)){
                list.add(m);
            }
        }
        return list;
    }
    // FIXME-end

    public static void main(String args[])
    {
        List<Method> list = findMyTaggedMethods(A.class);
        for (Method m : list)
            System.out.println(m.getName());
    }
}
