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

  //
  // Steps to solve
  // 1. Load the Jacobian
  // 2. V = GetVelocities
  // 3. invM = GetInverseMassMatrix
  // 4. Compute lambda --> impulse to apply to objects A and B
  // 5. Apply lambda impulse to A and B
  //

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
  matrix inv_m;
  vec2def j1;
  float j2;
  vec2def j3;
  float j4;
  vec2def pa;
  vec2def pa_minus_pb;
  vec2def pb;
  vec2def pb_minus_pa;
  vec2def ra;
  vec2def rb;
  vecndef v;

  //
  // Compute where the anchor point is now in world space. Note that we need to
  // see the same point in world space relative to both a and b. The constraint
  // is solved when pa == pb.
  //

  pa = body_local_space_to_world_space(*(c.a), c.a_point);
  pb = body_local_space_to_world_space(*(c.b), c.b_point);

  //
  // Load the jacobian. I will not explain why we use this formula, because I
  // straight up forgot how we got it. We first compute some values that we
  // reuse a lot here.
  //

  pa_minus_pb = vec2_sub(pa, pb);
  ra = vec2_sub(pa, c.a->position);
  pb_minus_pa = vec2_sub(pb, pa);
  rb = vec2_sub(pb, c.b->position);

  j1 = vec2_scale(pa_minus_pb, 2.0f);
  j2 = 2.0f * vec2_cross(ra, pa_minus_pb);
  j3 = vec2_scale(pb_minus_pa, 2.0f);
  j4 = 2.0f * vec2_cross(rb, pb_minus_pa);

  c.jacobian.rows[0].data[0] = j1.x;
  c.jacobian.rows[0].data[1] = j1.y;
  c.jacobian.rows[0].data[2] = j2;
  c.jacobian.rows[0].data[3] = j3.x;
  c.jacobian.rows[0].data[4] = j3.y;
  c.jacobian.rows[0].data[5] = j4;

  //
  // Next we get the velocities vector and inverse mass matrix, which will then
  // be used to calculate the impulses which we need to apply to a and b to
  // solve the constraints.
  //

  v = constraint_get_velocities(c);
  inv_m = constraint_get_inv_mat(c);

  // ...

  vecn_cleanup(v);
  matrix_cleanup(inv_m);
}

void solve_as_penetration(constraint& c) {
  // TODO
}
