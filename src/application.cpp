// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"
#include "./physics/constants.h"
#include "./physics/force.h"

void app_setup(application& app) {
  app.running = graphics_open_window(app.gr);
  app.time_prev_frame = SDL_GetTicks();

  app.push_force = vec2def(0.0f, 0.0f);

  app.fluid.x = 0;
  app.fluid.y = app.gr.window_h / 2;
  app.fluid.w = app.gr.window_w;
  app.fluid.h = app.gr.window_h / 2;
}

bool app_is_running(application& app) {
  return app.running;
}

void app_input(application& app) {
  SDL_Event event;
  particle* p;
  int x;
  int y;

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

        if (event.key.keysym.sym == SDLK_UP) {
          app.push_force.y = -50 * PIXELS_PER_METERS;
        }

        if (event.key.keysym.sym == SDLK_DOWN) {
          app.push_force.y = 50 * PIXELS_PER_METERS;
        }

        if (event.key.keysym.sym == SDLK_RIGHT) {
          app.push_force.x = 50 * PIXELS_PER_METERS;
        }

        if (event.key.keysym.sym == SDLK_LEFT) {
          app.push_force.x = -50 * PIXELS_PER_METERS;
        }

        break;
      }

      case SDL_KEYUP: {
        if (event.key.keysym.sym == SDLK_UP) {
          app.push_force.y = 0;
        }

        if (event.key.keysym.sym == SDLK_DOWN) {
          app.push_force.y = 0;
        }

        if (event.key.keysym.sym == SDLK_RIGHT) {
          app.push_force.x = 0;
        }

        if (event.key.keysym.sym == SDLK_LEFT) {
          app.push_force.x = 0;
        }

        break;
      }

      case SDL_MOUSEBUTTONDOWN: {
        if (event.button.button == SDL_BUTTON_LEFT) {
          SDL_GetMouseState(&x, &y);

          p = new particle;
          particle_init(*p, x, y, 1.0f);
          p->radius = 5;
          app.particles.push_back(p);

          break;
        }
      }
    }
  }
}

void app_update(application& app) {
  float bottom;
  float bound;
  float delta_time;
  vec2def drag;
  vec2def force_wind;
  vec2def force_weight;
  vec2def friction;
  vec2def gravity;
  int frame_delta;
  size_t i;
  float next_radius;
  size_t num_particles;
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
  num_particles = app.particles.size();

  force_wind = vec2def(2.73 * PIXELS_PER_METERS, 0.0f);
  gravity = vec2def(0.0f, 9.81 * PIXELS_PER_METERS);

  //
  // Apply each force to the particles.
  //

  for (i = 0; i < num_particles; i++) {

    //
    // Apply a wind force to each particle.
    //

    //particle_add_force(*(app.particles[i]), force_wind);

    //
    // Apply the push force.
    //

    particle_add_force(*(app.particles[i]), app.push_force);

    //
    // Apply friction.
    //

    friction = generate_friction_force(
      *(app.particles[i]),
      10.0f * PIXELS_PER_METERS
    );

    particle_add_force(*(app.particles[i]), friction);

    //
    // Apply the weight force to each particle.
    //

    //force_weight = vec2_scale(gravity, app.particles[i]->mass);
    //particle_add_force(*(app.particles[i]), force_weight);

    //
    // Apply the drag force to each particle if the particle is inside the
    // liquid.
    //

    //if (app.particles[i]->position.y >= app.fluid.y) {
    //  drag = generate_drag_force(*(app.particles[i]), 0.01f);
    //  particle_add_force(*(app.particles[i]), drag);
    //}
  }

  //
  // Now that we've our forces, perform the integration step to calculate the
  // position of each particle.
  //

  for (i = 0; i < num_particles; i++) {
    particle_integrate(*(app.particles[i]), delta_time);
  }

  //
  // Clamp the particle's position to be inside the bounds of the screen.
  //

  // Define a bottom that isn't quite the bottom of the screen. I find on my
  // machine the bottom of the screen is not visible so particles go missing.
  bottom = app.gr.window_h * 0.9f;

  for (i = 0; i < num_particles; i++) {
    next_radius = app.particles[i]->radius;

    bound = app.particles[i]->position.x - next_radius;
    if (bound <= 0) {
      app.particles[i]->position.x = next_radius;
      app.particles[i]->velocity.x *= -1.0f;
    }

    bound = app.particles[i]->position.x + next_radius;
    if (bound> app.gr.window_w) {
      app.particles[i]->position.x = app.gr.window_w - next_radius;
      app.particles[i]->velocity.x *= -1.0f;
    }

    bound = app.particles[i]->position.y - next_radius;
    if (bound <= 0) {
      app.particles[i]->position.y = next_radius;
      app.particles[i]->velocity.y *= -1.0f;
    }

    bound = app.particles[i]->position.y + next_radius;
    if (bound > bottom) {
      app.particles[i]->position.y = bottom - next_radius;
      app.particles[i]->velocity.y *= -1.0f;
    }
  }
}

void app_draw(application& app) {
  size_t i;
  size_t num_particles;

  graphics_clear_screen(app.gr, 0xFF056263);

  graphics_draw_fill_rect(
    app.gr,
    app.fluid.x + app.fluid.w / 2,
    app.fluid.y + app.fluid.h / 2,
    app.fluid.w,
    app.fluid.h,
    0xFF6E3713
  );

  num_particles = app.particles.size();

  for (i = 0; i < num_particles; i++) {
    graphics_draw_fill_circle(
      app.gr,
      app.particles[i]->position.x,
      app.particles[i]->position.y,
      app.particles[i]->radius,
      0xFFFFFFFF
    );
  }

  graphics_draw_frame(app.gr);
}

void app_destroy(application& app) {
  size_t i;
  size_t num_particles;

  num_particles = app.particles.size();
  for (i = 0; i < num_particles; i++) {
    delete app.particles[i];
  }

  graphics_close_window(app.gr);
}
