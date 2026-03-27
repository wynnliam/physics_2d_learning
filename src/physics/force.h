// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Impelements common physical forces.
*/

#ifndef FORCE
#define FORCE

#include "./particle.h"

vec2def generate_drag_force(const particle& p, const float k);

vec2def generate_friction_force(const particle& p, const float k);

#endif

