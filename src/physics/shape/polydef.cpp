// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

void shape_init(polydef& p) {
}

shape_type get_shape_type(const polydef& p) {
  return shape_type::POLYGON;
}

float get_moment_of_inertia(const polydef& p) {
  return 3.0f;
}

void transform_shape(polydef& p, const vec2def& pos, const float angle) {

  //
  // TODO: Iterate over all verts and transform.
  //

}

void draw_shape(
  graphics& gr,
  const polydef& p,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {

  //
  // TODO: Finish me!
  //

}

