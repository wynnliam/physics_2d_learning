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

void world_init(world& w, const float gravity) {
  w.gravity = gravity;
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
  vec2def force_weight;
  vec2def g_vec;
  size_t i;
  size_t j;
  size_t num_bodies;
  size_t num_forces;

  num_bodies = w.bodies.size();
  num_forces = w.forces.size();

  //
  // Apply the forces.
  //

  g_vec = vec2def(0.0f, 9.81 * PIXELS_PER_METERS);

  for (i = 0; i < num_bodies; i++) {
    // Apply gravity.
    force_weight = vec2_scale(g_vec, w.bodies[i]->mass);
    body_add_force(*(w.bodies[i]), force_weight);

    for (j = 0; j < num_forces; j++) {
      body_add_force(*(w.bodies[i]), w.forces[j]);
    }
  }

  //
  // Now that we've our forces, perform the integration step to calculate the
  // position of each body.
  //

  for (i = 0; i < num_bodies; i++) {
    body_update(*(w.bodies[i]), delta_time);
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
  // TODO: Finish me!
}
