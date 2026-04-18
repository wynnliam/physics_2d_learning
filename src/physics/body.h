// Liam Wynn, 3-24-2026, 2D Physics Engine

/*
  Represents the simplest physical object in our engine. A body is a simple
  point-mass object without shape.
*/

#ifndef BODY
#define BODY

#include "./vec2def.h"
#include "./shape/shapedef.h"

struct body {
  // TODO: Debug code!!!
  bool collides;

  // Linear motion, linear force, mass.
  vec2def position;
  vec2def velocity;
  vec2def acceleration;

  // Angular motion.
  // N.B. rotation is in radians.
  float rotation;
  float angular_velocity;
  float angular_acceleration;

  // Forces and torque.
  vec2def sum_force;
  float sum_torque;

  // Mass and Moment of Inertia.
  float mass;
  float inv_mass;
  // N.B. in a lot of engines, they use I.
  float inertia;
  float inv_inertia;

  float restitution;

  shapedef shape;
};

void body_init(
  body& p,
  const shapedef& shape,
  const float x,
  const float y,
  const float mass,
  const float restitution
);

void body_add_force(body& p, const vec2def& force);

void body_add_torque(body& p, const float torque);

void body_apply_impulse(body& p, const vec2def impulse);

void body_update(body& p, const float delta_time);

void body_integrate_linear(body& p, const float delta_time);

void body_integrate_angular(body& p, const float delta_time);

void body_clear_forces(body& p);

void body_clear_torque(body& p);

bool body_is_static(const body& p);

#endif

