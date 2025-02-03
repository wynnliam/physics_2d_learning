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

struct vect2d {
  float x;
  float y;

  vect2d(const float x, const float y) : x(x), y(y) { }

  ~vect2d() = default;
};

struct vect3d {
  float x;
  float y;
  float z;

  vect3d(const float x, const float y, const float z) : x(x), y(y), z(z) { }

  ~vect3d() = default;
};

#endif
