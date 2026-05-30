// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./application.h"
#include "./physics/constants.h"
#include "./physics/force.h"
#include <algorithm>
#include <iostream>

using namespace std;

void app_setup(application& app) {
  vec2def anchor;
  boxdef box;
  circledef circle;

  app.running = graphics_open_window(app.gr);
  app.time_prev_frame = SDL_GetTicks();

  world_init(
    app.w,
    // Gravity
    9.8f,
    // Drag TODO: Should be per-body
    0.00f,
    // Friction TODO: Should be per-body
    //4.0f * PIXELS_PER_METERS
    0.00f
  );

  //
  // Add all of the ragdoll components.
  //

  /*circle.radius = 5.0f;
  shape_init(circle);
  body* bob = new body;
  body_init(
    *bob,
    circle,
    app.gr.window_w / 2,
    app.gr.window_h / 2 - 200,
    0.0f,
    0.0f,
    0.0f
  );

  circle.radius = 25.0f;
  shape_init(circle);
  body* head = new body;
  body_init(
    *head,
    circle,
    bob->position.x,
    bob->position.y + 70,
    5.0f,
    1.0f,
    0.7f
  );

  box.width = 50;
  box.height = 100;
  shape_init(box);
  body* torso = new body;
  body_init(
    *torso,
    box,
    head->position.x,
    head->position.y + 80,
    3.0f,
    1.0f,
    0.7f
  );

  box.width = 15;
  box.height = 70;
  shape_init(box);
  body* left_arm = new body;
  body_init(
    *left_arm,
    box,
    torso->position.x - 32,
    torso->position.y - 10,
    1.0f,
    1.0f,
    0.7f
  );

  body* right_arm = new body;
  body_init(
    *right_arm,
    box,
    torso->position.x + 32,
    torso->position.y - 10,
    1.0f,
    1.0f,
    0.7f
  );

  box.width = 20;
  box.height = 90;
  shape_init(box);
  body* left_leg = new body;
  body_init(
    *left_leg,
    box,
    torso->position.x - 20,
    torso->position.y + 97,
    1.0f,
    1.0f,
    0.7f
  );

  body* right_leg = new body;
  body_init(
    *right_leg,
    box,
    torso->position.x + 20,
    torso->position.y + 97,
    1.0f,
    1.0f,
    0.7f
  );

  body_set_texture(*bob, app.gr, "./assets/ragdoll/bob.png");
  body_set_texture(*head, app.gr, "./assets/ragdoll/head.png");
  body_set_texture(*torso, app.gr, "./assets/ragdoll/torso.png");
  body_set_texture(*left_arm, app.gr, "./assets/ragdoll/leftArm.png");
  body_set_texture(*right_arm, app.gr, "./assets/ragdoll/rightArm.png");
  body_set_texture(*left_leg, app.gr, "./assets/ragdoll/leftLeg.png");
  body_set_texture(*right_leg, app.gr, "./assets/ragdoll/rightLeg.png");

  world_add_body(app.w, bob);
  world_add_body(app.w, head);
  world_add_body(app.w, torso);
  world_add_body(app.w, left_arm);
  world_add_body(app.w, right_arm);
  world_add_body(app.w, left_leg);
  world_add_body(app.w, right_leg);

  //
  // Now add all of the ragdoll constraints
  //

  constraint* string = new constraint;
  constraint_init_joint(
    *string,
    bob,
    head,
    bob->position
  );

  anchor = vec2_add(head->position, vec2def(0, 25));
  constraint* neck = new constraint;
  constraint_init_joint(
    *neck,
    head,
    torso,
    anchor
  );

  anchor = vec2_add(torso->position, vec2def(-28, -45));
  constraint* left_shoulder = new constraint;
  constraint_init_joint(
    *left_shoulder,
    torso,
    left_arm,
    anchor
  );

  anchor = vec2_add(torso->position, vec2def(28, -45));
  constraint* right_shoulder = new constraint;
  constraint_init_joint(
    *right_shoulder,
    torso,
    right_arm,
    anchor
  );

  anchor = vec2_add(torso->position, vec2def(-20, 50));
  constraint* left_hip = new constraint;
  constraint_init_joint(
    *left_hip,
    torso,
    left_leg,
    anchor
  );

  anchor = vec2_add(torso->position, vec2def(20, 50));
  constraint* right_hip = new constraint;
  constraint_init_joint(
    *right_hip,
    torso,
    right_leg,
    anchor
  );

  world_add_constraint(app.w, string);
  world_add_constraint(app.w, neck);
  world_add_constraint(app.w, left_shoulder);
  world_add_constraint(app.w, right_shoulder);
  world_add_constraint(app.w, left_hip);
  world_add_constraint(app.w, right_hip);*/
  
  //
  // Now add the floor and walls to contain all the components.
  //

  box.width = app.gr.window_w - 50;
  box.height = 50;
  shape_init(box);
  body* floor = new body;
  body_init(
    *floor,
    box,
    app.gr.window_w / 2,
    app.gr.window_h - 50,
    0.0f,
    0.7f,
    0.0f
  );

  box.width = 50;
  box.height = app.gr.window_h - 100;
  shape_init(box);
  body* left_wall = new body;
  body_init(
    *left_wall,
    box,
    50,
    app.gr.window_h / 2 - 25,
    0.0f,
    0.2f,
    0.0f
  );

  body* right_wall = new body;
  body_init(
    *right_wall,
    box,
    app.gr.window_w  - 50,
    app.gr.window_h / 2 - 25,
    0.0f,
    0.2f,
    0.0f
  );

  world_add_body(app.w, floor);
  world_add_body(app.w, left_wall);
  world_add_body(app.w, right_wall);
}

bool app_is_running(application& app) {
  return app.running;
}

void app_input(application& app) {
  body* basketball;
  body* bob;
  vec2def bob_vel;
  boxdef box;
  circledef circle;
  body* crate;
  SDL_Event event;
  vec2def mouse_pos;
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

        break;
      }

      case SDL_MOUSEBUTTONDOWN: {
        SDL_GetMouseState(&x, &y);

        if (event.button.button == SDL_BUTTON_LEFT) {
          circle.radius = 64;
          shape_init(circle);
          basketball = new body;
          body_init(
            *basketball,
            circle,
            x,
            y,
            1.0f,
            0.7f,
            0.1f
          );
          body_set_texture(*basketball, app.gr, "./assets/basketball.png");
          world_add_body(app.w, basketball);
        }

        if (event.button.button == SDL_BUTTON_RIGHT) {
          box.width = 140;
          box.height = 140;
          shape_init(box);
          crate = new body;
          body_init(
            *crate,
            box,
            x,
            y,
            1.0f,
            0.2f,
            0.1f
          );
          body_set_texture(*crate, app.gr, "./assets/crate.png");
          world_add_body(app.w, crate);
        }

        break;
      }

      case SDL_MOUSEMOTION: {
        //SDL_GetMouseState(&x, &y);
        //bob = app.w.bodies[0];
        //bob->position.x = x;
        //bob->position.y = y;
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
