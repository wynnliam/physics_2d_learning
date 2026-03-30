// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Implements the SDL game loop
*/

#ifndef APPLICATION
#define APPLICATION

#include "./graphics.h"
#include "./physics/body.h"
#include <vector>

struct chain_link {
  std::vector<size_t> links;
};

struct application {
  graphics gr;
  bool running;
  int time_prev_frame;

  vec2def mouse_cursor;
  bool left_mouse_button_down;

  std::vector<body*> bodys;

  vec2def push_force;

  SDL_Rect fluid;

  // chain.size() == bodys.size().
  std::vector<chain_link> chain;

  float spring_k;
  float spring_rest_length;
};

void app_setup(application& app);

bool app_is_running(application& app);

void app_input(application& app);

void app_update(application& app);

void app_draw(application& app);

void app_destroy(application& app);

#endif
