// Liam Wynn, 2/2/2025, 2D Physics Engine

/*
  Here I define a 2D vector. There isn't much to say - a 2D vector has 2
  floats, I know there are countless libraries out there that do this.
  Please don't take this to be a replacement for Box2D or Havoc or anything.
  I am making zero attempt to make this fast or slick or anything. This is
  purely for my own personal edification.
*/

#ifndef VEC2DEF
#define VEC2DEF

//
// VEC2DEF
//

struct vec2def {
  float x;
  float y;

  vec2def(const float x, const float y) : x(x), y(y) { }

  ~vec2def() = default;
};

//
// Returns true if the components of a == components of b.
//

bool vec2_eq(const vec2def& a, const vec2def& b);

//
// Calculates the magnitude of v. Uses the Pythagorean formula to solve
// for the magnitude.
//

float vec2_magnitude(const vec2def& v);

//
// Computes a noramlized version of v called v1. The magnitude of v1 is
// 1. Note if the magnitude of v is 0, then we return a zero vector.
//

vec2def vec2_norm(const vec2def& v);

//
// Calculates the dot product of a and b (also known as the scalar product).
//

float vec2_dot(const vec2def& a, const vec2def& b);

//
// Defines the clockwise perpendicular product of a given vector v.
// That is, it returns the vector v1 that is v rotated 90 degrees
// clockwise. (The above statement assumes the y-axis grows upwards.
// In some contexts, the screen renders downwards. Keep that in mind
// when you look at this function. This is about the closest we can
// get to a "cross" product of a 2D vector.
//

vec2def vec2_perp(const vec2def& v);

//
// Calculates a new vec2 c = a + b.
//

vec2def vec2_add(const vec2def& a, const vec2def& b);

//
// Calculates a new vec2 c = a - b.
//

vec2def vec2_sub(const vec2def& a, const vec2def& b);

//
// Scales each component of v by multiplying with scalar.
//

vec2def vec2_scale(const vec2def& v, const float scalar);

#endif
