// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <variant>

//
// TODO: Replace vec2def args with body ptr
//

/* SHAPE COLLISION ROUTINES */

static bool shape_collision(
  const circledef& a,
  const circledef& b,
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

  // Depth * (m1 / (m1 + m2))
void collision_solve_by_projection(collision_contact& contact) {
  float delta_a;
  float delta_b;
  float frac_a;
  float frac_b;
  float mass_a;
  float mass_b;
  float mass_sum;

  mass_a = contact.a->mass;
  mass_b = contact.b->mass;
  mass_sum = mass_a + mass_b;

  // TODO: What do if here?
  if (mass_sum == 0.0f) {
    return;
  }

  frac_a = mass_b / (mass_sum);
  frac_b = mass_a / (mass_sum);

  delta_a = contact.depth * frac_a;
  delta_b = contact.depth * frac_b;

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
