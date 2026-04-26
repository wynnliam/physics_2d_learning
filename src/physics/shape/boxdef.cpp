// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

void shape_init(boxdef& b) {
  size_t i;

  //
  // Set each vertex around the origin (0, 0).
  // N.B. up is -y.
  //

  b.local_verts[BOX_TL].x = -(b.width / 2);
  b.local_verts[BOX_TL].y = -(b.height / 2);

  b.local_verts[BOX_TR].x =  (b.width / 2);
  b.local_verts[BOX_TR].y = -(b.height / 2);

  b.local_verts[BOX_BR].x =  (b.width / 2);
  b.local_verts[BOX_BR].y =  (b.height / 2);

  b.local_verts[BOX_BL].x = -(b.width / 2);
  b.local_verts[BOX_BL].y =  (b.height / 2);

  for (i = 0; i < 4; i++) {
    b.world_verts[i] = b.local_verts[i];
  }
}

shape_type get_shape_type(const boxdef& b) {
  return shape_type::BOX;
}

float get_moment_of_inertia(const boxdef& b) {
  // 0.083333 is 1/12. N.B. this needs to be multiplied by rigid body mass.
  return 0.083333f * (b.width * b.width + b.height * b.height);
}

void transform_shape(boxdef& b, const vec2def& pos, const float angle) {
  size_t i;

  //
  // For each vertex, rotate about the origin first, and then translate.
  //

  for (i = 0; i < 4; i++) {
    b.world_verts[i] = vec2_rotate(b.local_verts[i], angle);
    b.world_verts[i] = vec2_add(b.world_verts[i], pos);
  }
}

void draw_shape(
  graphics& gr,
  const boxdef& b,
  SDL_Texture* texture,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {
  if (texture == NULL) {
    graphics_draw_polygon(
      gr,
      x,
      y,
      4,
      b.world_verts,
      color
    );
  } else {
    graphics_draw_texture(
      gr,
      texture,
      x,
      y,
      b.width,
      b.height,
      angle
    );
  }
}
