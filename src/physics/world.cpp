// Liam Wynn, 4-28-2026, 2D Physics Engine

#include "./world.h"
#include "./collision.h"
#include "./constants.h"
#include "./force.h"

using namespace std;

/*struct world {
  std::vector<body*> bodies;
  std::vector<vec2def> forces;
  std::vector<float> torques;
  float gravity;
};*/

void world_init(
  world& w,
  const float gravity,
  const float drag,
  const float friction
) {
  w.gravity = gravity;
  // TODO: Im pretty sure these are per-body actually
  w.drag = drag;
  w.friction = friction;
}

void world_add_body(world& w, body* b) {
  w.bodies.push_back(b);
}

void world_add_force(world& w, const vec2def& force) {
  w.forces.push_back(force);
}

void world_add_torque(world& w, const float torque) {
  w.torques.push_back(torque);
}

void world_update(world& w, const float delta_time) {
  vec2def force_drag;
  vec2def force_friction;
  vec2def force_weight;
  vec2def g_vec;
  size_t i;
  size_t j;
  size_t num_bodies;
  size_t num_forces;
  size_t num_torques;

  num_bodies = w.bodies.size();
  num_forces = w.forces.size();
  num_torques = w.torques.size();

  //
  // Apply the forces.
  //

  g_vec = vec2def(0.0f, 9.81 * PIXELS_PER_METERS);

  for (i = 0; i < num_bodies; i++) {
    // Apply gravity.
    force_weight = vec2_scale(g_vec, w.bodies[i]->mass);
    body_add_force(*(w.bodies[i]), force_weight);

    // Apply friction
    force_friction = generate_friction_force(
      *(w.bodies[i]),
      w.friction
    );
    body_add_force(*(w.bodies[i]), force_friction);

    // Apply drag.
    force_drag = generate_drag_force(*(w.bodies[i]), w.drag);
    body_add_force(*(w.bodies[i]), force_drag);

    for (j = 0; j < num_forces; j++) {
      body_add_force(*(w.bodies[i]), w.forces[j]);
    }
  }

  //
  // Apply torque values.
  //

  for (i = 0; i < num_bodies; i++) {
    for (j = 0; j < num_torques; j++) {
      body_add_torque(*(w.bodies[i]), w.torques[j]);
    }
  }

  //
  // Now that we've our forces, perform the integration step to calculate the
  // position of each body.
  //

  for (i = 0; i < num_bodies; i++) {
    body_update(*(w.bodies[i]), delta_time);
  }

  //
  // Handle collisions.
  //

  for (i = 0; i < 100; i++) {
    world_check_collisions(w);
  }
}

void world_check_collisions(world& w) {
  collision_contact contact;
  size_t i;
  size_t j;
  size_t num_bodies;

  num_bodies = w.bodies.size();
  
  for (i = 0; i < num_bodies; i++) {
    for (j = i + 1; j < num_bodies; j++) {
      if (is_colliding(w.bodies[i], w.bodies[j], contact)) {
        collision_solve_by_impulse(contact);
      }
    }
  }
}

void world_cleanup(world& w) {
  size_t i;
  size_t num_bodies;

  num_bodies = w.bodies.size();
  for (i = 0; i < num_bodies; i++) {
    body_cleanup(*(w.bodies[i]));
    delete w.bodies[i];
  }
}
