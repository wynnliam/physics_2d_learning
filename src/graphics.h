// Liam Wynn, 3-24-2026, 2D Physics Engine

#ifndef GRAPHICS
#define GRAPHICS

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

void graphics_draw_fill_circle(
  graphics& g,
  const int x,
  const int y,
  const int radius,
  const uint32_t color
);

void graphics_draw_frame(graphics& g);

void graphics_close_window(graphics& g);

#endif
