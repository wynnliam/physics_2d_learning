// Liam Wynn, 2/2/2025, 2D Physics Engine

#include "vectdef.h"
#include <cmath>

//
// VEC2 IMPL
//

bool vec2_eq(const vec2def& a, const vec2def& b) {
  return a.x == b.x && a.y == b.y;
}

float vec2_magnitude(const vec2def& v) {
  double x_sqrd;
  double y_sqrd;
  float magnitude;

  x_sqrd = v.x * v.x;
  y_sqrd = v.y * v.y;

  magnitude = (float)sqrt(x_sqrd + y_sqrd);

  return magnitude;
}

float vec2_dot(const vec2def& a, const vec2def& b) {
  float x_prod;
  float y_prod;
  float dot_prod;

  x_prod = a.x * b.x;
  y_prod = a.y * b.y;
  dot_prod = x_prod + y_prod;

  return dot_prod;
}

vec2def vec2_perp(const vec2def& v) {
  vec2def result(v.y, -v.x);

  return result;
}

vec2def vec2_add(const vec2def& a, const vec2def& b) {
  vec2def c(a.x + b.x, a.y + b.y);

  return c;
}

vec2def vec2_sub(const vec2def& a, const vec2def& b) {
  vec2def c(a.x - b.x, a.y - b.y);

  return c;
}

vec2def vec2_scale(const vec2def& v, const float scalar) {
  vec2def scaled_v(v.x * scalar, v.y * scalar);

  return scaled_v;
}

//
// VEC3 IMPL
//

vec3def vec3_cross_product(const vec3def& a, const vec3def& b) {
  vec3def result(0.0f, 0.0f, 0.0f);

  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;

  return result;
}
