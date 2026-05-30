// Liam Wynn, 5-2-2026, 2D Physics Engine

#ifndef CONSTRAINT
#define CONSTRAINT

#include "./body.h"
#include "./matrix.h"

//
// N.B. Historically we called the JOINT constraint DISTANCE. The principle is
// the same for the most part: we cannot be more than a distance away from a
// specific point.
//

enum class constraint_type {
  JOINT,
  PENETRATION
};

struct constraint {
  constraint_type type;
  body* a;
  body* b;
  vec2def a_point;
  vec2def b_point;
  matrix jacobian;
  float bias;
  vecndef cached_lambda;
  // ONLY USED IF CONSTRAINT IS A PENETRATION
  // Penetration of the collision in a's local space. TODO: Consider keeping the
  // World space version too since we use it and don't recalculate.
  vec2def normal;
  // Coefficient of friction between a and b.
  float friction;
};

void constraint_init_joint(
  constraint& c,
  body* a,
  body* b,
  const vec2def& anchor_point
);

void constraint_init_penetration(
  constraint& c,
  body* a,
  body* b,
  // TODO: Should we pass in the contact data itself?
  const vec2def& a_collision_point,
  const vec2def& b_collision_point,
  const vec2def& collision_normal
);

void constraint_cleanup(constraint& c);

matrix constraint_get_inv_mat(const constraint& c);

vecndef constraint_get_velocities(const constraint& c);

void constraint_presolve(constraint& c, const float dt);

void constraint_solve(constraint& c);

void constraint_postsolve(constraint& c);

#endif

