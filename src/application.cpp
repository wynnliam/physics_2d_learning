// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"

void app_setup(application& app) {
  app.running = graphics_open_window(app.gr);
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
}

void app_draw(application& app) {
  graphics_clear_screen(app.gr, 0xFF056263);
  graphics_draw_fill_circle(app.gr, 200, 200, 40, 0xFFFFFFFF);
  graphics_draw_frame(app.gr);
}

void app_destroy(application& app) {
  graphics_close_window(app.gr);
}
