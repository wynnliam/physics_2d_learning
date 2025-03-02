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
