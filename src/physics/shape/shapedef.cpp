// Liam Wynn, 3-31-2026, 2D Physics Engine

#include "./shapedef.h"

shape_type get_shape_type(const shapedef& s) {
  return std::visit([](const auto& arg){ return get_shape_type(arg); }, s);
}

float shape_get_moment_of_inertia(const shapedef& s) {
  return std::visit(
    [](const auto& arg) { return get_moment_of_inertia(arg); },
    s
  );
}

void shape_transform(shapedef& s, const vec2def& pos, const float angle) {
  std::visit(
    [&pos, angle](auto& arg) { transform_shape(arg, pos, angle); },
    s
  );
}

void draw_shape(
  graphics& gr,
  const shapedef& s,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
) {
  std::visit(
    [&gr, x, y, angle, color](const auto& arg) { 
      draw_shape(gr, arg, x, y, angle, color); 
    },
    s
  );
}
