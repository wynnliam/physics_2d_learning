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

vec2def vec2_add(const vec2def& a, const vec2def& b) {
  vec2def c(a.x + b.x, a.y + b.y);

  return c;
}

vec2def vec2_sub(const vec2def& a, const vec2def& b) {
  vec2def c(a.x - b.x, a.y - b.y);

  return c;
}
