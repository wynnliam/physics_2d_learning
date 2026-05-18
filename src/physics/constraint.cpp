// Liam Wynn, 5-12-2026, 2D Physics Engine

#include "constraint.h"

/* CONSTRAINT SOLVE ROUTINES */

static void solve_as_joint(constraint& c);

static void solve_as_penetration(constraint& c);

/* MAIN API IMPL */
void constraint_init_joint(
  constraint& c,
  body* a,
  body* b,
  const vec2def anchor_point
) {
  c = {};

  c.type = constraint_type::JOINT;
  c.a = a;
  c.b = b;

  c.a_point = body_world_space_to_local_space(*(c.a), anchor_point);
  c.b_point = body_world_space_to_local_space(*(c.b), anchor_point);

  matrix_init(c.jacobian, 1, 6);
}

matrix constraint_get_inv_mat(const constraint& c) {
  matrix result;

  //
  // Return a 6x6 matrix with all the inverse mass and inverse I of bodies "a"
  // and "b". The final format will be as follows:
  // [ 1/ma 0    0    0    0    0    ]
  // [ 0    1/ma 0    0    0    0    ]
  // [ 0    0    1/Ia 0    0    0    ]
  // [ 0    0    0    1/mb 0    0    ]
  // [ 0    0    0    0    1/mb 0    ]
  // [ 0    0    0    0    0    1/Ib ]
  //

  matrix_init(result, 6, 6);
  matrix_zero(result);

  result.rows[0].data[0] = c.a->inv_mass;
  result.rows[1].data[1] = c.a->inv_mass;
  result.rows[2].data[2] = c.a->inv_inertia;

  result.rows[3].data[3] = c.b->inv_mass;
  result.rows[4].data[4] = c.b->inv_mass;
  result.rows[5].data[5] = c.b->inv_inertia;

  return result;
}

vecndef constraint_get_velocities(const constraint& c) {
  vecndef result;

  //
  // Return a vector with 6 entries as follows:
  // [ a.vel_x       ]
  // [ a.vel_y       ]
  // [ a.angular_vel ]
  // [ b.vel_x       ]
  // [ b.vel_y       ]
  // [ b.angular_vel ]
  //

  vecn_init(result, 6);
  vecn_zero(result);

  result.data[0] = c.a->velocity.x;
  result.data[1] = c.a->velocity.y;
  result.data[2] = c.a->angular_velocity;

  result.data[3] = c.b->velocity.x;
  result.data[4] = c.b->velocity.y;
  result.data[5] = c.b->angular_velocity;

  return result;
}

void constraint_solve(constraint& c) {
  switch (c.type) {
    case constraint_type::JOINT: {
      solve_as_joint(c);
      break;
    }

    case constraint_type::PENETRATION: {
      solve_as_penetration(c);
      break;
    }

    default: {
      break;
    }
  }
}

/* CONSTRAINT SOLVE ROUTINES IMPL */

void solve_as_joint(constraint& c) {
  // TODO
}

void solve_as_penetration(constraint& c) {
  // TODO
}
