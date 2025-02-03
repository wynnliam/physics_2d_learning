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
  vec2def v(1.0f, 2.0f);
  float v_mag;

  v_mag = vec2_magnitude(v);

  cout << "v: ";
  print_vec2def(v);
  cout << "mag(v) = " << v_mag << endl;

  return 0;
}
