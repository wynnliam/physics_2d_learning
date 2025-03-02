// Liam Wynn, 2/2/2025, 2D Physics Engine

/*
  Here I define basic 2D and 3D vectors. For now, there isn't much to say -
  a 2D vector has 2 floats, and a 3D vector has 3 floats. I know there are
  countless libraries out there that do this. Please don't take this to be
  a replacement for Box2D or Havoc or anything. I am making zero attempt to
  make this fast or slick or anything. This is purely for my own personal
  edification.
*/

#ifndef VECTDEF
#define VECTDEF

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
// Returns true if a.x == b.x and a.y == b.y
//

bool vec2_eq(const vec2def& a, const vec2def& b);

//
// Calculates the magnitude of v. Uses the Pythagorean formula to solve
// mag = sqrt(x^2 + y^2).
//

float vec2_magnitude(const vec2def& v);

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

//
// VEC3DEF
//

// TODO: Not sure if we will do 3D implementations of operations. For now,
// we will have this definition, but I may just leave it as is.
struct vec3def {
  float x;
  float y;
  float z;

  vec3def(const float x, const float y, const float z) : x(x), y(y), z(z) { }

  ~vec3def() = default;
};

//
// Implements the cross product of a and b. NOTE: This is thus far the only routine
// that the vec3 has that the vec2 does not.
//

vec3def vec3_cross_product(const vec3def& a, const vec3def& b);

#endif
