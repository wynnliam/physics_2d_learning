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
  vec3def a(0.0f, 0.0f, 0.0f);
  vec3def b(2, 1, 3);
  vec3def c = vec3_norm(b);

  print_vec3def(vec3_norm(a));
  print_vec3def(c);

  cout << vec3_magnitude(c) << endl;

  return 0;
}
