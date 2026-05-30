// Liam Wynn, 4-28-2026, 2D Physics Engine

#include "./world.h"
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

void world_add_constraint(world& w, constraint* c) {
  w.constraints.push_back(c);
}

void world_add_force(world& w, const vec2def& force) {
  w.forces.push_back(force);
}

void world_add_torque(world& w, const float torque) {
  w.torques.push_back(torque);
}

void world_update(world& w, const float delta_time) {
  collision_contact contact;
  vec2def force_drag;
  vec2def force_friction;
  vec2def force_weight;
  vec2def g_vec;
  size_t i;
  size_t j;
  constraint* next_constraint;
  size_t num_bodies;
  size_t num_constraints;
  size_t num_forces;
  size_t num_penetrations;
  size_t num_torques;
  vector<constraint*> penetrations;

  num_bodies = w.bodies.size();
  num_constraints = w.constraints.size();
  num_forces = w.forces.size();
  num_torques = w.torques.size();

  //
  // Apply the forces and torques.
  //

  g_vec = vec2def(0.0f, w.gravity * PIXELS_PER_METERS);

  for (i = 0; i < num_bodies; i++) {
    // Apply gravity.
    force_weight = vec2_scale(g_vec, w.bodies[i]->mass);
    body_add_force(*(w.bodies[i]), force_weight);

    for (j = 0; j < num_forces; j++) {
      body_add_force(*(w.bodies[i]), w.forces[j]);
    }

    for (j = 0; j < num_torques; j++) {
      body_add_torque(*(w.bodies[i]), w.torques[j]);
    }
  }

  //
  // Now that we've our forces, perform integration on the forces to get our
  // velocities.
  //

  for (i = 0; i < num_bodies; i++) {
    body_integrate_forces(*(w.bodies[i]), delta_time);
  }

  //
  // Handle collisions.
  //

  for (i = 0; i < num_bodies; i++) {
    for (j = i + 1; j < num_bodies; j++) {
      if (is_colliding(w.bodies[i], w.bodies[j], contact)) {
        //collision_solve_by_impulse(contact);
        w.debug_contact.push_back(contact);

        next_constraint = new constraint;
        constraint_init_penetration(
          *next_constraint,
          contact.a,
          contact.b,
          contact.start,
          contact.end,
          contact.normal
        );

        penetrations.push_back(next_constraint);
      }
    }
  }

  num_penetrations = penetrations.size();

  //
  // Solve all constraints. Apply warm starting
  //

  for (i = 0; i < num_constraints; i++) {
    constraint_presolve(*(w.constraints[i]), delta_time);
  }

  for (i = 0; i < num_penetrations; i++) {
    constraint_presolve(*(penetrations[i]), delta_time);
  }

  for (j = 0; j < 5; j++) {
    for (i = 0; i < num_constraints; i++) {
      constraint_solve(*(w.constraints[i]));
    }

    for (i = 0; i < num_penetrations; i++) {
      constraint_solve(*(penetrations[i]));
    }
  }

  for (i = 0; i < num_constraints; i++) {
    constraint_postsolve(*(w.constraints[i]));
  }

  for (i = 0; i < num_penetrations; i++) {
    constraint_postsolve(*(penetrations[i]));
  }

  //
  // Now update our positions using our velocities.
  //

  for (i = 0; i < num_bodies; i++) {
    body_integrate_velocities(*(w.bodies[i]), delta_time);
  }

  //
  // Need to clean up the penetration constraints.
  //

  for (i = 0; i < num_penetrations; i++) {
    constraint_cleanup(*(penetrations[i]));
  }
}

void world_cleanup(world& w) {
  size_t i;
  size_t num_bodies;
  size_t num_constraints;

  num_bodies = w.bodies.size();
  for (i = 0; i < num_bodies; i++) {
    body_cleanup(*(w.bodies[i]));
    delete w.bodies[i];
  }

  num_constraints = w.constraints.size();
  for (i = 0; i < num_constraints; i++) {
    constraint_cleanup(*(w.constraints[i]));
    delete w.constraints[i];
  }
}
