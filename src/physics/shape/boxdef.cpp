// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

void shape_init(boxdef& b) {

  //
  // Set each vertex around the origin (0, 0).
  // N.B. up is -y.
  //

  b.verts[BOX_TL].x = -(b.width / 2);
  b.verts[BOX_TL].y = -(b.height / 2);

  b.verts[BOX_TR].x =  (b.width / 2);
  b.verts[BOX_TR].y = -(b.height / 2);

  b.verts[BOX_BR].x =  (b.width / 2);
  b.verts[BOX_BR].y =  (b.height / 2);

  b.verts[BOX_BL].x = -(b.width / 2);
  b.verts[BOX_BL].y =  (b.height / 2);
}

shape_type get_shape_type(const boxdef& b) {
  return shape_type::BOX;
}

float get_moment_of_inertia(const boxdef& b) {
  // 0.083333 is 1/12. N.B. this needs to be multiplied by rigid body mass.
  return 0.083333f * (b.width * b.width * b.height * b.height);
}

void draw_shape(
  graphics& gr,
  const boxdef& b,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {

  graphics_draw_polygon(
    gr,
    x,
    y,
    4,
    b.verts,
    color
  );
}
