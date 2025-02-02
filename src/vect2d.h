// Liam Wynn, 2/2/2025: 2D Physics Engine

#ifndef VECTDEF
#define VECTDEF

struct vect2d {
  float x;
  float y;

  vect2d(const float x, const float y) : x(x), y(y) { }

  ~vect2d() = default;
};

#endif
