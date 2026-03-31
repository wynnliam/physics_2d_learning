// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Represents the simplest physical object in our engine. A body is a simple
  point-mass object without shape.
*/

#ifndef BODY
#define BODY

#include "./vec2def.h"

struct body {
  vec2def position;
  vec2def velocity;
  vec2def acceleration;
  float mass;
  float inv_mass;
  vec2def sum_force;
  int radius;
};

void body_init(body& p, const float x, const float y, const float mass);

void body_add_force(body& p, const vec2def& force);

void body_integrate(body& p, const float delta_time);

void body_clear_forces(body& p);

#endif

