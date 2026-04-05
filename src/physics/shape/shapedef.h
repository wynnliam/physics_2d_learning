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

void draw_shape(
  graphics& gr,
  const shapedef& s,
  const int x,
  const int y,
  const uint32_t color
);

#endif
