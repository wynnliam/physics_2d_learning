// Liam Wynn, 2/2/2025, 2D Physics Engine

/*
  TODO: Add nicer build system. For now:
  g++ -g *.cpp

  To run:

  ./a.out
*/


#include <iostream>
#include "./vectdef.h"
#include "./util.h"

using namespace std;

int main() {
  vec3def a(4.0f, -2.0f, 0.0f);
  vec3def b(-3.0f, 1.0f, 0.0f);
  vec3def c1 = vec3_cross_product(a, b);
  vec3def c2 = vec3_cross_product(b, a);

  print_vec3def(a);
  print_vec3def(b);
  print_vec3def(c1);
  print_vec3def(c2);

  return 0;
}
