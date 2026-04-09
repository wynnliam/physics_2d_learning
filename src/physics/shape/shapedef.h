// Liam Wynn, 3-31-2026, 2D Physics Engine

/*
  Here we define the public-facing API to handle shapes.
*/

#ifndef SHAPE
#define SHAPE

#include "./shape_types.h"
#include <variant>

using shapedef = std::variant<
  circledef,
  boxdef,
  polydef
>;

shape_type get_shape_type(const shapedef& s);

float shape_get_moment_of_inertia(const shapedef& s);

void shape_transform(shapedef& s, const vec2def& pos, const float angle);

void draw_shape(
  graphics& gr,
  const shapedef& s,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

#endif
