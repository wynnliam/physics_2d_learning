// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"
#include "./physics/constants.h"
#include "./physics/force.h"
#include <algorithm>
#include <iostream>

using namespace std;

void app_setup(application& app) {
  body* a;
  body* b;
  circledef circle;
  constraint* constr_ab;

  app.running = graphics_open_window(app.gr);
  app.time_prev_frame = SDL_GetTicks();

  world_init(
    app.w,
    // Gravity
    10.0f,
    // Drag TODO: Should be per-body
    0.00f,
    // Friction TODO: Should be per-body
    //4.0f * PIXELS_PER_METERS
    0.00f
  );

  circle.radius = 30.0f;
  a = new body;
  body_init(
    *a,
    circle,
    app.gr.window_w / 2,
    app.gr.window_h / 2,
    0.0f,
    0.0f,
    0.0f
  );

  circle.radius = 20.0f;
  b = new body;
  body_init(
    *b,
    circle,
    a->position.x - 100.0f,
    a->position.y,
    1.0f,
    0.0f,
    0.0f
  );

  world_add_body(app.w, a);
  world_add_body(app.w, b);

  constr_ab = new constraint;
  constraint_init_joint(
    *constr_ab,
    a,
    b,
    a->position
  );

  world_add_constraint(app.w, constr_ab);
}

bool app_is_running(application& app) {
  return app.running;
}

void app_input(application& app) {
  SDL_Event event;

  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_QUIT: {
        app.running = false;
        break;
      }

      case SDL_KEYDOWN: {
        if (event.key.keysym.sym == SDLK_ESCAPE) {
          app.running = false;
        }

        break;
      }
    }
  }
}

void app_update(application& app) {
  float delta_time;
  int frame_delta;
  vec2def gravity;
  int time_to_wait;

  //
  // If we got through our last frame quicker than MILLISECONDS_PER_FRAME, then
  // rest for the difference. This keeps us at FPS.
  //

  frame_delta = SDL_GetTicks() - app.time_prev_frame;
  time_to_wait = MILLISECONDS_PER_FRAME - (frame_delta);

  if (time_to_wait > 0) {
    SDL_Delay(time_to_wait);
  }

  frame_delta = SDL_GetTicks() - app.time_prev_frame;
  delta_time = (frame_delta) / 1000.0f;

  //
  // Clamp the delta time in case we do something like debug or anything to
  // interrupt it.
  //

  if (delta_time > 0.016f) {
    delta_time = 0.016f;
  }

  app.time_prev_frame = SDL_GetTicks();

  world_update(app.w, delta_time);

}

void app_draw(application& app) {
  uint32_t body_color;
  size_t i;
  size_t num_bodies;

  graphics_clear_screen(app.gr, 0xFF056263);

  num_bodies = app.w.bodies.size();
  for (i = 0; i < num_bodies; i++) {
    body_color = 0xFFFFFFFF;

    draw_shape(
      app.gr,
      app.w.bodies[i]->shape,
      app.w.bodies[i]->texture,
      app.w.bodies[i]->position.x,
      app.w.bodies[i]->position.y,
      app.w.bodies[i]->rotation,
      body_color
    );
  }

  graphics_draw_frame(app.gr);
}

void app_destroy(application& app) {
  world_cleanup(app.w);
  graphics_close_window(app.gr);
}
