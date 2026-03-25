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
  int radius;
};

void particle_init(particle& p, const float x, const float y, const float mass);

#endif

