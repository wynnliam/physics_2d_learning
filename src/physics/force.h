// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Impelements common physical forces.
*/

#ifndef FORCE
#define FORCE

#include "./body.h"

vec2def generate_drag_force(const body& p, const float k);

vec2def generate_friction_force(const body& p, const float k);

// N.B. generates attractional force FROM a TO b. But this can be negated to get
// FOMR b to a.
vec2def generate_gravitational_force(
  const body& a,
  const body& b,
  const float G,
  const float min_dist,
  const float max_dist
);

vec2def generate_spring_force(
  const body& p,
  const vec2def anchor,
  const float rest_length,
  const float k
);

vec2def generate_spring_force(
  const body& a,
  const body& b,
  const float rest_length,
  const float k
);

#endif

