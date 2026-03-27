// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./graphics.h"
#include <iostream>

using namespace std;

bool graphics_open_window(graphics& g) {
  SDL_DisplayMode display_mode;
  int result;

  result = SDL_Init(SDL_INIT_EVERYTHING);
  if (result != 0) {
    cerr << "error initializing SDL" << endl;
    return false;
  }

  SDL_GetCurrentDisplayMode(0, &display_mode);
  g.window_w = display_mode.w;
  g.window_h = display_mode.h;

  g.window = SDL_CreateWindow(
    NULL,
    0,
    0,
    g.window_w,
    g.window_h,
    SDL_WINDOW_BORDERLESS
  );

  if (g.window == NULL) {
    cerr << "error creating SDL window" << endl;
    return false;
  }

  g.renderer = SDL_CreateRenderer(
    g.window,
    -1,
    SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
  );

  if (g.renderer == NULL) {
    cerr << "error creating SDL renderer" << endl;
    return false;
  }

  return true;
}

void graphics_clear_screen(graphics& g, const uint32_t color) {
  SDL_SetRenderDrawColor(g.renderer, color >> 16, color >> 8, color, 255);
  SDL_RenderClear(g.renderer);
}

void graphics_draw_fill_circle(
  graphics& g,
  const int x,
  const int y,
  const int radius,
  const uint32_t color
) {
  filledCircleColor(g.renderer, x, y, radius, color);
}

void graphics_draw_fill_rect(
  graphics& g,
  const int x,
  const int y,
  const int w,
  const int h,
  const uint32_t color
) {
  boxColor(
    g.renderer,
    x - w / 2.0,
    y - h / 2.0,
    x + w / 2.0,
    y + h / 2.0,
    color
  );
}

void graphics_draw_frame(graphics& g) {
  SDL_RenderPresent(g.renderer);
}

void graphics_close_window(graphics& g) {
  SDL_DestroyRenderer(g.renderer);
  SDL_DestroyWindow(g.window);
  SDL_Quit();
}

