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
  vec3def a(2, 1, 3);
  vec3def b = vec3_scale(a, 2);

  print_vec3def(a);
  print_vec3def(b);

  return 0;
}
