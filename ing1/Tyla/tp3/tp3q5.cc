#include <iostream>

// Simple class defining both hello_world and bye_world
class HelloBye
{
public:
  int hello_world() const { return 0; }
  int bye_world() const { return 0; }
};

// Simple class defining only hello_world
class Hello
{
public:
  int hello_world() const { return 0; }
};

// Simple class defining only bye_world
class Bye
{
public:
  int bye_world() const { return 0; }
};

// Simple class defining nothing
class Misc {};

// The goal of this class is to use SFINAE to check wether the
// template argument has both hello_world and bye_world methods
template <typename T>
class has_hellobye
{
private:
  // First define two types using "using". Since templates works
  // on types, one of this types will represent TRUE while the
  // other will represent FALSE.
  // Notice that these two types must have different size. Have
  // a look to true_type and false_type

  // FIXME-begin
  using TRUE = std::true_type;
  using FALSE = std::false_type;
  // FIXME-end

  // Define here a templated method test that will return the
  // FALSE type. This method is variadic and will represent the
  // default behavior, i.e. the type has not both hello_world
  // and bye_world method.

  // FIXME-begin
  template <typename U> static FALSE test(...) { return FALSE{}; }
  // FIXME-end

  // Now we must define the case where both hello_world and bye_world
  // are available.
  // To do that define a template method test. This method takes
  // an integer as parameter and return TRUE *only if* the given
  // template has both hello_world and bye_world.
  //
  // Notice: use decltype and declval to achive that
  // Warning: check if the method is const.
  // FIXME-begin
  template <typename U>
  static auto test(int) -> decltype(std::declval<const U>().hello_world(),
                                    std::declval<const U>().bye_world(),
                                    TRUE{})
  {
      return TRUE{};
  }
  // FIXME-end
public:

  static constexpr bool value =
    std::is_same<decltype(test<T>(0)),
                 // Put here the TRUE type you have defined.
                 // FIXME-begin
                  TRUE
                 // FIXME-end
                 >::value;
};


int main(int argc, char *argv[])
{
  std::cout << has_hellobye<HelloBye>::value << std::endl;
  std::cout << has_hellobye<Hello>::value << std::endl;
  std::cout << has_hellobye<Bye>::value << std::endl;
  std::cout << has_hellobye<Misc>::value << std::endl;
  return 0;
}
