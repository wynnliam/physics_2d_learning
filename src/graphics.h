// Liam Wynn, 3-24-2026, 2D Physics Engine

#ifndef GRAPHICS
#define GRAPHICS

#include "./physics/vec2def.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <vector>

struct graphics {
  uint32_t window_w;
  uint32_t window_h;
  SDL_Window* window;
  SDL_Renderer* renderer;
};

bool graphics_open_window(graphics& g);

void graphics_clear_screen(graphics& g, const uint32_t color);

void graphics_draw_circle(
  graphics& g,
  const int x,
  const int y,
  const int radius,
  const float angle,
  const uint32_t color
);

void graphics_draw_fill_circle(
  graphics& g,
  const int x,
  const int y,
  const int radius,
  const uint32_t color
);

void graphics_draw_fill_rect(
  graphics& g,
  const int x,
  const int y,
  const int w,
  const int h,
  const uint32_t color
);

void graphics_draw_polygon(
  graphics& g,
  const int x,
  const int y,
  const size_t num_verts,
  const vec2def* verts,
  const uint32_t color
);

void graphics_draw_line(
  graphics& g,
  const int x0,
  const int y0,
  const int x1,
  const int y1,
  const uint32_t color
);

void graphics_draw_texture(
  graphics& g,
  SDL_Texture* texture,
  const int x,
  const int y,
  const int width,
  const int height,
  const float rotation
);

void graphics_draw_frame(graphics& g);

void graphics_close_window(graphics& g);

#endif
