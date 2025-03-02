// Liam Wynn, 3/2/2025, 2D Physics Engine

#include "./vec3def.h"
#include <cmath>

bool vec3_eq(const vec3def& a, const vec3def& b) {
  return a.x == b.x &&
         a.y == b.y &&
         a.z == b.z;
}

float vec3_magnitude(const vec3def& v) {
  double x_sqrd;
  double y_sqrd;
  double z_sqrd;
  float magnitude;

  x_sqrd = v.x * v.x;
  y_sqrd = v.y * v.y;
  z_sqrd = v.z * v.z;

  magnitude = (float)sqrt(x_sqrd + y_sqrd + z_sqrd);

  return magnitude;
}

float vec3_dot(const vec3def& a, const vec3def& b) {
  float x_prod;
  float y_prod;
  float z_prod;
  float dot_prod;

  x_prod = a.x * b.x;
  y_prod = a.y * b.y;
  z_prod = a.z * b.z;
  dot_prod = x_prod + y_prod + z_prod;

  return dot_prod;
}

vec3def vec3_add(const vec3def& a, const vec3def& b) {
  vec3def c(a.x + b.x, a.y + b.y, a.z + b.z);

  return c;
}

vec3def vec3_sub(const vec3def& a, const vec3def& b) {
  vec3def c(a.x - b.x, a.y - b.y, a.z - b.z);

  return c;
}

vec3def vec3_scale(const vec3def& v, const float scalar) {
  vec3def scaled_v(
    v.x * scalar,
    v.y * scalar,
    v.z * scalar
  );

  return scaled_v;
}

vec3def vec3_cross_product(const vec3def& a, const vec3def& b) {
  vec3def result(0.0f, 0.0f, 0.0f);

  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;

  return result;
}

