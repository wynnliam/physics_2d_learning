// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shape_types.h"

void shape_init(polydef& p) {
  size_t i;
  size_t num_verts;

  num_verts = p.local_vertices.size();
  p.world_vertices.resize(num_verts);

  for (i = 0; i < num_verts; i++) {
    p.world_vertices[i] = p.local_vertices[i];
  }
}

shape_type get_shape_type(const polydef& p) {
  return shape_type::POLYGON;
}

float get_moment_of_inertia(const polydef& p) {
  return 5000.0f;
}

void transform_shape(polydef& p, const vec2def& pos, const float angle) {
  size_t i;
  size_t num_verts;

  num_verts = p.world_vertices.size();

  //
  // For each vertex, rotate about the origin first, and then translate.
  //

  for (i = 0; i < num_verts; i++) {
    p.world_vertices[i] = vec2_rotate(p.local_vertices[i], angle);
    p.world_vertices[i] = vec2_add(p.world_vertices[i], pos);
  }

}

void draw_shape(
  graphics& gr,
  const polydef& p,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {
  graphics_draw_polygon(
    gr,
    x,
    y,
    p.world_vertices.size(),
    p.world_vertices.data(),
    color
  );
}

