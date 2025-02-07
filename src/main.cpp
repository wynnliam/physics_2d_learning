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
  vec2def a(1.0f, 2.0f);
  vec2def b(3.0f, 4.0f);
  float dot_prod = vec2_dot(a, b);

  cout << "Dot prod is: " << dot_prod << endl;

  return 0;
}
