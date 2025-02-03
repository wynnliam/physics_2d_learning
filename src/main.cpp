// Liam Wynn, 2/2/2025: 2D Physics Engine

/*
  TODO: Add nicer build system. For now:
  g++ -g *.cpp

  To run:

  ./a.out
*/

#include <iostream>

#include "./vectdef.h"

using namespace std;

int main() {
  vect2d v(1.0f, 2.0f);
  vect3d m(0.67f, 10.54f, 3.72f);

  cout << "Vector: [" << v.x << ", " << v.y << "]" << endl;
  cout << "Vector: [" << m.x << ", " << m.y << ", " << m.z << "]" << endl;

  return 0;
}
