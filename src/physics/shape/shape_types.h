// Liam Wynn, 3-30-2026, 2D Physics Engine

/*
  Here we define the possible rigid-body shape types we will implement in this
  engine.
*/

#ifndef SHAPE_TYPES
#define SHAPE_TYPES

#include <cstdint>
#include "../../graphics.h"
#include "../vec2def.h"
#include <vector>

enum class shape_type {
  CIRCLE,
  BOX,
  POLYGON
};

struct circledef {
  float radius;
};

struct boxdef {
  float width;
  float height;
};

struct polydef {
  std::vector<vec2def> vertices;
};

shape_type get_shape_type(const circledef& c);
shape_type get_shape_type(const boxdef& b);
shape_type get_shape_type(const polydef& p);

float get_moment_of_inertia(const circledef& c);
float get_moment_of_inertia(const boxdef& b);
float get_moment_of_inertia(const polydef& p);

void draw_shape(
  graphics& gr,
  const circledef& c,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

void draw_shape(
  graphics& gr,
  const boxdef& b,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

void draw_shape(
  graphics& gr,
  const polydef& p,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

#endif

