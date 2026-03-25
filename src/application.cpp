// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"
#include "./physics/constants.h"

void app_setup(application& app) {
  app.running = graphics_open_window(app.gr);

  app.time_prev_frame = SDL_GetTicks();

  app.p = new particle;
  particle_init(*(app.p), 50, 100, 1.0f);
  app.p->radius = 50;
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

  app.p->acceleration = vec2def(2.73 * PIXELS_PER_METERS, 9.81 * PIXELS_PER_METERS);

  //
  // Integrate the acceleration and velocity to find the new position.
  //

  app.p->velocity = vec2_add(
    vec2_scale(app.p->acceleration, delta_time),
    app.p->velocity
  );

  app.p->position = vec2_add(
    vec2_scale(app.p->velocity, delta_time),
    app.p->position
  );

  //
  // Clamp the particle's position to be inside the bounds of the screen.
  //

  if (app.p->position.x - app.p->radius <= 0) {
    app.p->position.x = app.p->radius;
    app.p->velocity.x *= -1.0f;
  }

  if (app.p->position.x + app.p->radius > app.gr.window_w) {
    app.p->position.x = app.gr.window_w - app.p->radius;
    app.p->velocity.x *= -1.0f;
  }

  if (app.p->position.y - app.p->radius <= 0) {
    app.p->position.y = app.p->radius;
    app.p->velocity.y *= -1.0f;
  }

  if (app.p->position.y + app.p->radius > app.gr.window_h / 2) {
    app.p->position.y = app.gr.window_h / 2 - app.p->radius;
    app.p->velocity.y *= -1.0f;
  }
}

void app_draw(application& app) {
  graphics_clear_screen(app.gr, 0xFF056263);

  graphics_draw_fill_circle(
    app.gr,
    app.p->position.x,
    app.p->position.y,
    app.p->radius,
    0xFFFFFFFF
  );

  graphics_draw_frame(app.gr);
}

void app_destroy(application& app) {
  delete app.p;

  graphics_close_window(app.gr);
}
