// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <variant>

/* SHAPE COLLISION ROUTINES */

static bool shape_collision(
  const circledef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  collision_contact& contact
);

static bool shape_collision(
  const boxdef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  collision_contact& contact
);

static bool shape_collision(
  const polydef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  collision_contact& contact
);

// Generic not-yet-impl routine.
template<typename A, typename B>
static bool shape_collision(
  const A&,
  const B&,
  body*,
  body*,
  collision_contact&
);

// General purpose collision handler for two polygons.
static bool poly_collision(
  const vec2def* a_verts,
  const size_t a_vert_count,
  body* body_a,
  const vec2def* b_verts,
  const size_t b_vert_count,
  body* body_b,
  collision_contact& contact
); 

/* MAIN API IMPL */

bool is_colliding(body* a, body* b, collision_contact& contact) {
  return std::visit(
    [&](const auto& s1, const auto& s2) {
      return shape_collision(s1, s2, a, b, contact);
    },
    a->shape,
    b->shape
  );
}

void collision_solve_by_projection(collision_contact& contact) {
  float delta_a;
  float delta_b;
  float mass_a;
  float mass_b;
  float mass_sum;

  if (body_is_static(*(contact.a)) && body_is_static(*(contact.b))) {
    return;
  }

  mass_a = contact.a->inv_mass;
  mass_b = contact.b->inv_mass;
  mass_sum = mass_a + mass_b;

  delta_a = contact.depth / mass_sum * mass_a;
  delta_b = contact.depth / mass_sum * mass_b;

  //
  // Use the deltas to move the positions.
  //

  contact.a->position = vec2_sub(
    contact.a->position,
    vec2_scale(contact.normal, delta_a)
  );

  contact.b->position = vec2_add(
    contact.b->position,
    vec2_scale(contact.normal, delta_b)
  );
}

void collision_solve_by_impulse(collision_contact& contact) {
  body* a;
  body* b;
  float e;
  vec2def impulse;
  vec2def impulse_dir;
  float impulse_mag;
  vec2def vel_relative;
  float vrdn;

  //
  // First, separate the two objects via the projection method.
  //

  collision_solve_by_projection(contact);

  a = contact.a;
  b = contact.b;

  //
  // Nothing to do if both a and b are static.
  //

  if (body_is_static(*a) && body_is_static(*b)) {
    return;
  }

  //
  // Choose elasticity to apply. N.B. there are many different ways we could
  // handle this. I suspect one method is to have have an e for both a and b.
  //

  e = std::min(a->restitution, b->restitution);

  //
  // Now compute the relative velocity of between a and b.
  //

  vel_relative = vec2_sub(a->velocity, b->velocity);

  //
  // Now calculate the relative velocity along the normal. N.B. we do Velocity
  // Relative Dot Normal.
  //

  vrdn = vec2_dot(vel_relative, contact.normal);

  //
  // Now compute the impulse.
  //

  impulse_mag = -(1 + e) * vrdn / (a->inv_mass + b->inv_mass);
  impulse_dir = contact.normal;
  impulse = vec2_scale(impulse_dir, impulse_mag);

  //
  // Lastly, apply the impulses.
  //

  body_apply_impulse(*a, impulse);
  body_apply_impulse(*b, vec2_scale(impulse, -1));
}

/* SHAPE COLLISION ROUTINE IMPL */

bool shape_collision(
  const circledef& a,
  const circledef& b,
  body* a_body,
  body* b_body,
  collision_contact& contact
) {
  vec2def ab;
  vec2def apos;
  vec2def bpos;
  float dist;
  float rad_sum;
  bool result;

  apos = a_body->position;
  bpos = b_body->position;

  ab = vec2_sub(bpos, apos);
  dist = vec2_mag_squared(ab);
  rad_sum = a.radius + b.radius;

  result = dist <= (rad_sum * rad_sum);

  if (result == false) {
    return false;
  }

  contact.a = a_body;
  contact.b = b_body;
  contact.normal = vec2_norm(ab);
  contact.start = vec2_sub(bpos, vec2_scale(contact.normal, b.radius));
  contact.end = vec2_add(apos, vec2_scale(contact.normal, a.radius));
  contact.depth = vec2_magnitude(vec2_sub(contact.end, contact.start));

  return true;
}

bool shape_collision(
  const boxdef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  collision_contact& contact
) {
  return poly_collision(
    a.world_verts,
    4,
    body_a,
    b.world_verts,
    4,
    body_b,
    contact
  );
}

bool shape_collision(
  const polydef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  collision_contact& contact
) {
  return poly_collision(
    a.world_vertices.data(),
    a.world_vertices.size(),
    body_a,
    b.world_vertices.data(),
    b.world_vertices.size(),
    body_b,
    contact
  );
}

bool poly_collision(
  const vec2def* a_verts,
  const size_t a_vert_count,
  body* body_a,
  const vec2def* b_verts,
  const size_t b_vert_count,
  body* body_b,
  collision_contact& contact
) {
  return false;
}

template<typename A, typename B>
bool shape_collision(
  const A&,
  const B&,
  body*,
  body*,
  collision_contact&
) {
  return false;
}
