// Liam Wynn, 3/2/2025, 2D Physics Engine

/*
  Here I define a 3D vector. There isn't much to say - a 3D vector has 3
  floats, I know there are countless libraries out there that do this.
  Please don't take this to be a replacement for Box2D or Havoc or anything.
  I am making zero attempt to make this fast or slick or anything. This is
  purely for my own personal edification.
*/

#ifndef VEC3DEF
#define VEC3DEF

//
// VEC3DEF
//

struct vec3def {
  float x;
  float y;
  float z;

  vec3def(const float x, const float y, const float z) : x(x), y(y), z(z) { }

  ~vec3def() = default;
};

//
// Returns true if the components of a == components of b.
//

bool vec3_eq(const vec3def& a, const vec3def& b);

//
// Calculates the magnitude of v. Uses the Pythagorean formula to solve
// for the magnitude.
//

float vec3_magnitude(const vec3def& v);

//
// Computes a noramlized version of v called v1. The magnitude of v1 is
// 1. Note if the magnitude of v is 0, then we return a zero vector.
//

vec3def vec3_norm(const vec3def& v);

//
// Calculates the dot product of a and b (also known as the scalar product).
//

float vec3_dot(const vec3def& a, const vec3def& b);

//
// Calculates a new vec3 c = a + b.
//

vec3def vec3_add(const vec3def& a, const vec3def& b);

//
// Calculates a new vec3 c = a - b.
//

vec3def vec3_sub(const vec3def& a, const vec3def& b);

//
// Scales each component of v by multiplying with scalar.
//

vec3def vec3_scale(const vec3def& v, const float scalar);

//
// Implements the cross product of a and b. NOTE: This is thus far the only routine
// that the vec3 has that the vec2 does not.
//

vec3def vec3_cross_product(const vec3def& a, const vec3def& b);

#endif
