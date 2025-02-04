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
  vec2def v(10.0f, 40.0f);
  float scalar = 4.25f;
  vec2def s = vec2_scale(v, scalar);

  cout << "v: ";
  print_vec2def(v);

  cout << "4.25 * v = ";
  print_vec2def(s);

  return 0;
}
