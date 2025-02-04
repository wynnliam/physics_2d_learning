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
  vec2def m(3.0f, 4.0f);
  vec2def r = vec2_add(v, m);
  vec2def s = vec2_sub(v, m);
  float v_mag;

  v_mag = vec2_magnitude(v);

  cout << "v: ";
  print_vec2def(v);
  cout << "mag(v) = " << v_mag << endl;

  cout << "m: ";
  print_vec2def(m);

  cout << "v + m: ";
  print_vec2def(r);

  cout << "v - m: ";
  print_vec2def(s);

  return 0;
}
