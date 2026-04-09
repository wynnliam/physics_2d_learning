// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

void shape_init(circledef& c) {
  // Nothing to do
}

shape_type get_shape_type(const circledef& c) {
  return shape_type::CIRCLE;
}

float get_moment_of_inertia(const circledef& c) {
  // N.B this needs to be multiplied by rigid body mass.
  return 0.5f * (c.radius * c.radius);
}

void transform_shape(circledef& c, const vec2def& pos, const float angle) {

  //
  // Nothing to do here.
  //

}

void draw_shape(
  graphics& gr,
  const circledef& c,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {
  graphics_draw_circle(gr, x, y, c.radius, angle, color);
}
