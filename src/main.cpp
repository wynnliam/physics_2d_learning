// Liam Wynn, 2/2/2025: 2D Physics Engine

/*
  TODO: Add nicer build system. For now:
  g++ -g *.cpp

  To run:

  ./a.out
*/

#include <iostream>

#include "./vect2d.h"

using namespace std;

int main() {
  vect2d v(1.0f, 2.0f);

  cout << "Vector: [" << v.x << ", " << v.y << "]" << endl;

  return 0;
}
