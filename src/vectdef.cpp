// Liam Wynn, 2/2/2025, 2D Physics Engine

#include "vectdef.h"
#include <cmath>

float vec2_magnitude(const vec2def& v) {
  double x_sqrd;
  double y_sqrd;
  float magnitude;

  x_sqrd = v.x * v.x;
  y_sqrd = v.y * v.y;

  magnitude = (float)sqrt(x_sqrd + y_sqrd);

  return magnitude;
}

