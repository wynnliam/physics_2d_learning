// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./particle.h"

void particle_init(
  particle& p,
  const float x,
  const float y,
  const float mass
) {
  p.position = vec2def(x, y);

  if (mass == 0.0f) {
    p.mass = 1.0f;
    p.inv_mass = 1.0f;
  } else {
    p.mass = mass;
    p.inv_mass = 1.0f / mass;
  }

  particle_clear_forces(p);
}

void particle_add_force(particle& p, const vec2def& force) {
  p.sum_force = vec2_add(force, p.sum_force);
}

void particle_integrate(particle& p, const float delta_time) {

  //
  // First, calculate the final acceleration of the particle for this frame. We
  // apply the law acceleration = force / mass to get the acceleration, and then
  // we perform a step of integration.
  //

  p.acceleration = vec2_scale(p.sum_force, p.inv_mass);

  //
  // Perform two steps of Euler Integration to find the position. First, we
  // integrate the acceleration to compute the velocity. Then, we integrate the
  // velocity to find the position.
  //

  p.velocity = vec2_add(vec2_scale(p.acceleration, delta_time), p.velocity);
  p.position = vec2_add(vec2_scale(p.velocity, delta_time), p.position);

  //
  // After integrating we need to clear the forces since they only apply for
  // this frame.
  //

  particle_clear_forces(p);
}

void particle_clear_forces(particle& p) {
  p.sum_force = vec2def(0.0f, 0.0f);
}
