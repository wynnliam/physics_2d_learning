// Liam Wynn, 5-12-2026, 2D Physics Engine

#include "constraint.h"

using namespace std;

/* CONSTRAINT SOLVE ROUTINES */

static void warm_start(constraint& c, const matrix& jt, const vecndef& l);

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

  c.bias = 0.0f;

  vecn_init(c.cached_lambda, 1);
  vecn_zero(c.cached_lambda);
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

void constraint_presolve(constraint& c, const float dt) {
  float beta;
  float err;
  vec2def j1;
  float j2;
  vec2def j3;
  float j4;
  matrix jacobian_transposed;
  vec2def pa;
  vec2def pa_minus_pb;
  vec2def pb;
  vec2def pb_minus_pa;
  vec2def ra;
  vec2def rb;

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

  matrix_zero(c.jacobian);
  c.jacobian.rows[0].data[0] = j1.x;
  c.jacobian.rows[0].data[1] = j1.y;
  c.jacobian.rows[0].data[2] = j2;
  c.jacobian.rows[0].data[3] = j3.x;
  c.jacobian.rows[0].data[4] = j3.y;
  c.jacobian.rows[0].data[5] = j4;

  //
  // TODO: We shouldn't keep alloc and freeing jacobian_transposed. We do it
  // again in solve_as_joint. So I think what we need to do is make it a member
  // var, update transpose to take a boolean that says "if true we alloc,
  // otherwise we assume the values are already alloced and we just copy"
  //

  jacobian_transposed = matrix_transpose(c.jacobian);

  warm_start(c, jacobian_transposed, c.cached_lambda);

  //
  // Calculate the bias factor (Baumgarte Stabilization). TODO: I am assuming
  // that err is the *only* thing dependent on constraint type. When we add
  // penetration constraints I will re-examine this.
  //

  switch (c.type) {
    case constraint_type::JOINT: {
      err = std::max(0.0f, vec2_dot(pb_minus_pa, pb_minus_pa) - 0.01f);
      break;
    }

    default: {
      err = 0.0f;
      break;
    }
  }

  beta = 0.2f;
  c.bias = (beta / dt) * err;

  matrix_cleanup(jacobian_transposed);
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

void constraint_postsolve(constraint& c) {
}

/* CONSTRAINT SOLVE ROUTINES IMPL */

void warm_start(constraint& c, const matrix& jt, const vecndef& l) {
  vecndef impulses;
  vec2def impulse_linear_a;
  vec2def impulse_linear_b;

  impulses = matrix_vecn_mul(jt, l).value();

  impulse_linear_a.x = impulses.data[0];
  impulse_linear_a.y = impulses.data[1];
  body_apply_impulse_linear(*(c.a), impulse_linear_a);
  body_apply_impulse_angular(*(c.a), impulses.data[2]);

  impulse_linear_b.x = impulses.data[3];
  impulse_linear_b.y = impulses.data[4];
  body_apply_impulse_linear(*(c.b), impulse_linear_b);
  body_apply_impulse_angular(*(c.b), impulses.data[5]);

  vecn_cleanup(impulses);
}

void solve_as_joint(constraint& c) {
  matrix inv_m;
  matrix jacobian_transposed;
  vecndef lambda;
  vecndef lambda_numerator;
  matrix lambda_denominator;
  matrix lambda_denominator1;
  vecndef v;

  //
  // Next we get all of the items needed to compute the lambda. This includes:
  // velocities vector, inverse mass matrix, and the transposed jacobian. These
  // will then be used to calculate the impulses which we need to apply to a and
  // b to solve the constraints.
  //

  v = constraint_get_velocities(c);
  inv_m = constraint_get_inv_mat(c);
  jacobian_transposed = matrix_transpose(c.jacobian);

  //
  // Now we compute lambda. N.B. we should check that the optionals actually
  // produce values. But I am being a lazy here.
  //

  lambda_numerator = matrix_vecn_mul(c.jacobian, v).value();
  vecn_scale(lambda_numerator, -1.0f);
  // We subtract (not add) because of inversion in last step. Also, the
  // lambda_numerator will be a single value. Hence we just do [0].
  lambda_numerator.data[0] -= c.bias;

  lambda_denominator1 = matrix_mat_mul(c.jacobian, inv_m).value();
  lambda_denominator = matrix_mat_mul(
    lambda_denominator1,
    jacobian_transposed
  ).value();

  lambda = matrix_solve_gauss_seidel(lambda_denominator, lambda_numerator);
  vecn_add(c.cached_lambda, lambda);

  //
  // Compute the final impulses with direction + magnitude and apply to the
  // bodies. This code is the same as the warm_start code, so we call it here
  // with this jacobian_transposed + lambda.
  //

  warm_start(c, jacobian_transposed, lambda);

  //
  // Clean up allocated vectors and matrices.
  // TODO: Seems like a terrible use of space and allocation.
  //

  vecn_cleanup(v);
  matrix_cleanup(inv_m);
  matrix_cleanup(jacobian_transposed);
  vecn_cleanup(lambda);
  vecn_cleanup(lambda_numerator);
  matrix_cleanup(lambda_denominator1);
  matrix_cleanup(lambda_denominator);
}

void solve_as_penetration(constraint& c) {
  // TODO
}
