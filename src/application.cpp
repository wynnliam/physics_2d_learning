// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"
#include "./physics/constants.h"
#include "./physics/force.h"
#include <algorithm>

using namespace std;

static void insert_chain_link(
  vector<size_t>& chain,
  const size_t to
);

void app_setup(application& app) {
  app.running = graphics_open_window(app.gr);
  app.time_prev_frame = SDL_GetTicks();

  app.chain.resize(4);
  app.spring_k = 300.0f;
  app.spring_rest_length = 80;

  app.bodies.resize(app.chain.size());

  app.bodies[0] = new body;
  body_init(*(app.bodies[0]), 50, 100, 2.0f);
  app.bodies[0]->radius = 4;
  insert_chain_link(app.chain[0].links, 1);
  insert_chain_link(app.chain[0].links, 2);
  insert_chain_link(app.chain[0].links, 3);
  
  app.bodies[1] = new body;
  body_init(*(app.bodies[1]), 100, 100, 2.0f);
  app.bodies[1]->radius = 4;
  insert_chain_link(app.chain[1].links, 0);
  insert_chain_link(app.chain[1].links, 2);
  insert_chain_link(app.chain[1].links, 3);
  
  app.bodies[2] = new body;
  body_init(*(app.bodies[2]), 50, 200, 2.0f);
  app.bodies[2]->radius = 4;
  insert_chain_link(app.chain[2].links, 0);
  insert_chain_link(app.chain[2].links, 1);
  insert_chain_link(app.chain[2].links, 3);
  
  app.bodies[3] = new body;
  body_init(*(app.bodies[3]), 100, 200, 2.0f);
  app.bodies[3]->radius = 4;
  insert_chain_link(app.chain[3].links, 0);
  insert_chain_link(app.chain[3].links, 1);
  insert_chain_link(app.chain[3].links, 2);
  
  //for (i = 0; i < app.bodies.size(); i++) {
  //  app.bodies[i] = new body;
  //  body_init(
  //    *(app.bodies[i]),
  //    app.gr.window_w / 2,
  //    i * app.spring_rest_length + 50,
  //    2.0f
  //  );
  //  app.bodies[i]->radius = 4;

  //  if (i > 0) {
  //    insert_chain_link(app.chain[i].links, i - 1);
  //    insert_chain_link(app.chain[i - 1].links, i);
  //  }
  //}

  app.push_force = vec2def(0.0f, 0.0f);

  app.fluid.x = 0;
  app.fluid.y = app.gr.window_h / 2;
  app.fluid.w = app.gr.window_w;
  app.fluid.h = app.gr.window_h / 2;

  app.left_mouse_button_down = false;
}

bool app_is_running(application& app) {
  return app.running;
}

void app_input(application& app) {
  SDL_Event event;
  vec2def impulse_dir;
  float impulse_mag;
  vec2def mouse_to_p0;
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

      case SDL_MOUSEMOTION: {
        app.mouse_cursor.x = event.motion.x;
        app.mouse_cursor.y = event.motion.y;

        break;
      }

      case SDL_MOUSEBUTTONDOWN: {
        if (event.button.button == SDL_BUTTON_LEFT) {
          app.left_mouse_button_down = true;
          SDL_GetMouseState(&x, &y);
          app.mouse_cursor.x = x;
          app.mouse_cursor.y = y;
        }

        break;
      }

      case SDL_MOUSEBUTTONUP: {
        if (app.left_mouse_button_down) {
          if (event.button.button == SDL_BUTTON_LEFT) {
            app.left_mouse_button_down = false;

            mouse_to_p0 = vec2_sub(
              app.bodies[0]->position,
              app.mouse_cursor
            );
            impulse_dir = vec2_norm(mouse_to_p0);
            impulse_mag = vec2_magnitude(mouse_to_p0) * 5.0f;

            app.bodies[0]->velocity = vec2_scale(impulse_dir, impulse_mag);
          }
        }

        break;
      }
    }
  }
}

void app_update(application& app) {
  vec2def attraction;
  float bottom;
  float bound;
  float delta_time;
  vec2def force_drag;
  vec2def force_spring;
  vec2def force_weight;
  vec2def force_friction;
  vec2def gravity;
  int frame_delta;
  size_t i;
  size_t j;
  size_t l;
  float next_radius;
  size_t num_links;
  size_t num_bodies;
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
  num_bodies = app.bodies.size();

  gravity = vec2def(0.0f, 9.81 * PIXELS_PER_METERS);

  //
  // Apply each force to the bodies.
  //

  for (i = 0; i < num_bodies; i++) {

    //
    // Apply the push force.
    //

    body_add_force(*(app.bodies[i]), app.push_force);

    //
    // Apply friction.
    //

    //force_friction = generate_friction_force(
    //  *(app.bodies[i]),
    //  5.0f * PIXELS_PER_METERS
    //);

    //body_add_force(*(app.bodies[i]), force_friction);

    num_links = app.chain[i].links.size();
    for (j = 0; j < num_links; j++) {
      l = app.chain[i].links[j];

      force_spring = generate_spring_force(
        *(app.bodies[i]),
        *(app.bodies[l]),
        app.spring_rest_length,
        app.spring_k
      );

      body_add_force(*(app.bodies[i]), force_spring);
    }

    //
    // Apply the weight force to each body.
    //

    force_weight = vec2_scale(gravity, app.bodies[i]->mass);
    body_add_force(*(app.bodies[i]), force_weight);

    //
    // Apply a drag force.
    //

    force_drag = generate_drag_force(*(app.bodies[i]), 0.01f);
    body_add_force(*(app.bodies[i]), force_drag);

    //
    // Apply the drag force to each body if the body is inside the
    // liquid.
    //

    //if (app.bodies[i]->position.y >= app.fluid.y) {
    //  force_drag = generate_drag_force(*(app.bodies[i]), 0.01f);
    //  body_add_force(*(app.bodies[i]), force_drag);
    //}
  }

  //
  // Now that we've our forces, perform the integration step to calculate the
  // position of each body.
  //

  for (i = 0; i < num_bodies; i++) {
    body_integrate(*(app.bodies[i]), delta_time);
  }

  //
  // Clamp the body's position to be inside the bounds of the screen.
  //

  // Define a bottom that isn't quite the bottom of the screen. I find on my
  // machine the bottom of the screen is not visible so bodies go missing.
  bottom = app.gr.window_h * 0.9f;

  for (i = 0; i < num_bodies; i++) {
    next_radius = app.bodies[i]->radius;

    bound = app.bodies[i]->position.x - next_radius;
    if (bound <= 0) {
      app.bodies[i]->position.x = next_radius;
      app.bodies[i]->velocity.x *= -1.0f;
    }

    bound = app.bodies[i]->position.x + next_radius;
    if (bound> app.gr.window_w) {
      app.bodies[i]->position.x = app.gr.window_w - next_radius;
      app.bodies[i]->velocity.x *= -1.0f;
    }

    bound = app.bodies[i]->position.y - next_radius;
    if (bound <= 0) {
      app.bodies[i]->position.y = next_radius;
      app.bodies[i]->velocity.y *= -1.0f;
    }

    bound = app.bodies[i]->position.y + next_radius;
    if (bound > bottom) {
      app.bodies[i]->position.y = bottom - next_radius;
      app.bodies[i]->velocity.y *= -1.0f;
    }
  }
}

void app_draw(application& app) {
  size_t i;
  size_t j;
  size_t l;
  size_t num_bodies;
  size_t num_links;

  graphics_clear_screen(app.gr, 0xFF056263);

  if (app.left_mouse_button_down) {
    graphics_draw_line(
      app.gr,
      app.bodies[0]->position.x,
      app.bodies[0]->position.y,
      app.mouse_cursor.x,
      app.mouse_cursor.y,
      0xFF0000FF
    );
  }

  //graphics_draw_fill_rect(
  //  app.gr,
  //  app.fluid.x + app.fluid.w / 2,
  //  app.fluid.y + app.fluid.h / 2,
  //  app.fluid.w,
  //  app.fluid.h,
  //  0xFF6E3713
  //);

  //
  // Draw the spring and anchor.
  //

  num_bodies = app.bodies.size();

  for (i = 0; i < num_bodies; i++) {
    num_links = app.chain[i].links.size();

    for (j = 0; j < num_links; j++) {
      l = app.chain[i].links[j];

      graphics_draw_line(
        app.gr,
        app.bodies[i]->position.x,
        app.bodies[i]->position.y,
        app.bodies[l]->position.x,
        app.bodies[l]->position.y,
        0xFF0000FF
      );
    }

    graphics_draw_fill_circle(
      app.gr,
      app.bodies[i]->position.x,
      app.bodies[i]->position.y,
      app.bodies[i]->radius,
      0xFFFFFFFF
    );
  }

  graphics_draw_frame(app.gr);
}

void app_destroy(application& app) {
  size_t i;
  size_t num_bodies;

  num_bodies = app.bodies.size();
  for (i = 0; i < num_bodies; i++) {
    delete app.bodies[i];
  }

  graphics_close_window(app.gr);
}

/* UTILITY ROUTINES */
void insert_chain_link(
  vector<size_t>& chain,
  const size_t to
) {
  auto f = std::find(chain.begin(), chain.end(), to);

  if (f == chain.end()) {
    chain.push_back(to);
  }
}
