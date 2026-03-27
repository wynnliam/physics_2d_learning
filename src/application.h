// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Implements the SDL game loop
*/

#ifndef APPLICATION
#define APPLICATION

#include "./graphics.h"
#include "./physics/particle.h"
#include <vector>

struct application {
  graphics gr;
  bool running;
  int time_prev_frame;

  std::vector<particle*> particles;
  vec2def push_force;
  SDL_Rect fluid;
};

void app_setup(application& app);

bool app_is_running(application& app);

void app_input(application& app);

void app_update(application& app);

void app_draw(application& app);

void app_destroy(application& app);

#endif
