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
  vec2def v(2, 1);
  vec2def v1 = vec2_perp(v);

  print_vec2def(v);
  print_vec2def(v1);

  return 0;
}
