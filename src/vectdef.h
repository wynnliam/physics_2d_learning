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
// Calculates the magnitude of v. Uses the Pythagorean formula to solve
// mag = sqrt(x^2 + y^2).
//

float vec2_magnitude(const vec2def& v);

//
// Calculates a new vec2 c = a + b.
//

vec2def vec2_add(const vec2def& a, const vec2def& b);

//
// Calculates a new vec2 c = a - b.
//

vec2def vec2_sub(const vec2def& a, const vec2def& b);

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

#endif
