// Liam Wynn, 3/2/2025, 2D Physics Engine

#include "./vec3def.h"
#include <cmath>

vec3def vec3_cross_product(const vec3def& a, const vec3def& b) {
  vec3def result(0.0f, 0.0f, 0.0f);

  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;

  return result;
}

