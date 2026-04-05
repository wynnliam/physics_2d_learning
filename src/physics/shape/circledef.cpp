// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

shape_type get_shape_type(const circledef& c) {
  return shape_type::CIRCLE;
}

void draw_shape(
  graphics& gr,
  const circledef& c,
  const int x,
  const int y,
  const uint32_t color
) {
  graphics_draw_fill_circle(gr, x, y, c.radius, color);
}
