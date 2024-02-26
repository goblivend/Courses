class Foo{

    // FIXME-begin
    private Foo(int p1, int p2, int p3, int p4) {
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.p4 = p4;
    }


    static class FooBuilder {
        int p1 = 1;
        int p2 = 2;
        int p3 = 3;
        int p4 = 4;

        public FooBuilder withP1(int n) {
            p1 = n;
            return this;
        }

        public FooBuilder withP2(int n) {
            p2 = n;
            return this;
        }

        public FooBuilder withP3(int n) {
            p3 = n;
            return this;
        }

        public FooBuilder withP4(int n) {
            p4 = n;
            return this;
        }
        public Foo build() {
            return new Foo(p1, p2, p3, p4);
        }
    };
    public static FooBuilder builder() {
        return new FooBuilder();
    }
    // FIXME-end

    public int getP1(){
        return p1;
    }
    public int getP2(){
        return p2;
    }
    public int getP3(){
        return p3;
    }
    public int getP4(){
        return p4;
    }

    private int p1;
    private int p2;
    private int p3;
    private int p4;
}
public class Main{

     public static void main(String []args){
        {
            Foo f = Foo.builder()
                       .build();
            System.out.println(f.getP1());
            System.out.println(f.getP2());
            System.out.println(f.getP3());
            System.out.println(f.getP4());
            System.out.println("----");
        }
        {
            Foo f = Foo.builder()
                       .withP4(8)
                       .withP1(3)
                       .build();
            System.out.println(f.getP1());
            System.out.println(f.getP2());
            System.out.println(f.getP3());
            System.out.println(f.getP4());
        }
     }
}
