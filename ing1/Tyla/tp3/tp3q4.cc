#include <iostream>

// adder takes three templates:
//      - type of the first parameter
//      - type of the second parameter
//      - type of the result
// FIXME-begin
template <typename T1, typename T2, typename T3>
class adder
{
public:
  T3 compute(T1 a, T2 b)
  {
    return a + b;
  }
};
// FIXME-end

class matrix2D
{
public:
  matrix2D(int tl, int tr, int bl, int br)
  {
    array[0][0] = tl;
    array[0][1] = tr;
    array[1][0] = bl;
    array[1][1] = br;
  }

  // FIXME-begin
  matrix2D operator+(const matrix2D &m)
  {
    matrix2D result(0, 0, 0, 0);
    result.array[0][0] = array[0][0] + m.array[0][0];
    result.array[0][1] = array[0][1] + m.array[0][1];
    result.array[1][0] = array[1][0] + m.array[1][0];
    result.array[1][1] = array[1][1] + m.array[1][1];
    return result;
  }
  // FIXME-end

public:
  int array[2][2];
};

// operator<< for matrix2D
// FIXME-begin
std::ostream& operator<<(std::ostream &os, const matrix2D &m)
{
  os << m.array[0][0] << " " << m.array[0][1] << std::endl;
  os << m.array[1][0] << " " << m.array[1][1] << std::endl;
  return os;
}
// FIXME-end

int main()
{
  // Work for ints
  adder<int, int, int> a1;
  std::cout << a1.compute(1, 2) << std::endl;

  // Or on whatever type with a "+"
  adder<matrix2D, matrix2D, matrix2D> a2;
  matrix2D m1(1, 3, 1, 0);
  matrix2D m2(0, 0, 7, 5);
  std::cout << a2.compute(m1, m2) << std::endl;
}
