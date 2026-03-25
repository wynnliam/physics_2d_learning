// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./particle.h"

void particle_init(
  particle& p,
  const float x,
  const float y,
  const float mass
) {
  p.position = vec2def(x, y);
  p.mass = mass;
}

