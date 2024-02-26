#include <iostream>
#include <functional>
#include <memory>

template <typename T>
class lazy_ctor
{
  // FIXME-begin
public:
    std::function<T()> ctor;
    T stored = nullptr;

    lazy_ctor(std::function<T()> t)
        : ctor{t}
    {}

    T operator->() {
        if (!stored)
            stored = ctor();
        return stored;
    }
    auto& operator*() {
        if (!stored)
            stored = ctor();
        return *stored;
    }
  // FIXME-end
};


class foo
{
public:
  foo(int i) : i_(i)
  {
    std::cout << "foo::ctor(" << i_ << ") may be costly" << std::endl;
  };

  foo(foo&& f) =default;

  foo(const foo& f) = delete;
  foo(foo& f) = delete;

  ~foo(){
    std::cout << "foo::dtor(" << i_ << ")" << std::endl;
  };

  bool doit() const
  {
    std::cout << "foo::doit(" << i_ << ")" << std::endl;
    return true;
  };

private:
  int i_;
};


int main()
{
  lazy_ctor<std::shared_ptr<foo>> lazy_foo_1 =
    { [] () {return  std::make_unique<foo>(1) ;} };
  lazy_ctor<std::shared_ptr<foo>> lazy_foo_2 =
    { [] () {return  std::make_unique<foo>(2) ;} };

  std::cout << "    Not contructed yet!" << std::endl;
  if (lazy_foo_1->doit() || lazy_foo_2->doit())
    std::cout << "   2 should not have been constructed\n";

  std::cout << "--\n";
  lazy_ctor<std::shared_ptr<foo>> lazy_foo_3 =
    { [] () {return  std::make_unique<foo>(3) ;} };

  std::cout << "    Not contructed yet!" << std::endl;
  lazy_foo_3->doit();
  std::cout << "--\n";
  (*lazy_foo_3).doit();
}
