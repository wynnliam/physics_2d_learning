// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Represents the simplest physical object in our engine. A particle is a simple
  point-mass object without shape.
*/

#ifndef PARTICLE
#define PARTICLE

#include "./vec2def.h"

struct particle {
  vec2def position;
  vec2def velocity;
  vec2def acceleration;
  float mass;
  float inv_mass;
  vec2def sum_force;
  int radius;
};

void particle_init(particle& p, const float x, const float y, const float mass);

void particle_add_force(particle& p, const vec2def& force);

void particle_integrate(particle& p, const float delta_time);

void particle_clear_forces(particle& p);

#endif

