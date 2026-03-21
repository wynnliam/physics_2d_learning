// Liam Wynn, 2/2/2025, 2D Physics Engine

/*
  TODO: Add nicer build system. For now:
  g++ -g *.cpp

  To run:

  ./a.out
*/


#include <iostream>
#include "./vecdef.h"
#include "./util.h"

using namespace std;

int main() {
  vec2def a = { 1, 2 };
  float beta = 0.785398f;
  vec2def c = vec2_rotate(a, beta);

  print_vec2def(a);
  print_vec2def(c);

  return 0;
}
