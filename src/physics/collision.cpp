// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <limits>
#include <variant>

// When  computing the min separation, we return this collection of data here.
struct separation_info {
  // The amount of separation.
  float amount;
  // The axis or edge for which the separation was found.
  vec2def axis;
  // The point of the polygon that penetrated to produce the separation.
  vec2def point;
};

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

// Finds the minimum separation for two polygons
static void find_min_separation(
  const vec2def* a_verts,
  const size_t a_vert_count,
  const vec2def* b_verts,
  const size_t b_vert_count,
  separation_info& result
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
  float f;
  vec2def impulse;
  vec2def impulse_norm;
  vec2def impulse_norm_dir;
  vec2def impulse_tang;
  vec2def impulse_tang_dir;
  float impulse_norm_mag;
  float impulse_norm_mag_denom;
  float impulse_tang_mag;
  float impulse_tang_mag_denom;
  vec2def omega_cross_ra;
  vec2def omega_cross_rb;
  vec2def ra;
  float ra_cross_norm;
  float ra_cross_tang;
  vec2def rb;
  float rb_cross_norm;
  float rb_cross_tang;
  vec2def tangent;
  vec2def vel_relative;
  vec2def vala;
  vec2def vbla;
  float vrdn;
  float vrdt;

  a = contact.a;
  b = contact.b;

  //
  // Nothing to do if both a and b are static.
  //

  if (body_is_static(*a) && body_is_static(*b)) {
    return;
  }

  //
  // First, separate the two objects via the projection method.
  //

  collision_solve_by_projection(contact);

  //
  // Choose elasticity and friction to apply. N.B. there are many different ways
  // we could handle this. I suspect one method is to have have an e for both a
  // and b.
  //

  e = std::min(a->restitution, b->restitution);
  f = std::min(a->friction, b->friction);

  //
  // Compute the vector from the center of mass to the point of collision.
  //

  ra = vec2_sub(contact.end, a->position);
  rb = vec2_sub(contact.start, b->position);

  //
  // Compute the angular velocity crossed with r. N.B. this result is a
  // simplificiation of promoting the angular velocity and the r vectors to 3D
  // like so: omega = (0, 0, body->angular_vel), r1 = (rx, ry, 0). Then taking
  // the cross product.
  //

  omega_cross_ra = vec2_scale(vec2def(-ra.y, ra.x), a->angular_velocity);
  omega_cross_rb = vec2_scale(vec2def(-rb.y, rb.x), b->angular_velocity);

  //
  // Compute the sum of the linear *and* angular velocities for both bodies.
  //

  vala = vec2_add(a->velocity, omega_cross_ra);
  vbla = vec2_add(b->velocity, omega_cross_rb);

  //
  // Now compute the relative velocity of between a and b.
  //

  vel_relative = vec2_sub(vala, vbla);

  //
  // Compute the impulse along the normal.
  //

  ra_cross_norm = vec2_cross(ra, contact.normal);
  ra_cross_norm *= ra_cross_norm;
  ra_cross_norm *= a->inv_inertia;

  rb_cross_norm = vec2_cross(rb, contact.normal);
  rb_cross_norm *= rb_cross_norm;
  rb_cross_norm *= b->inv_inertia;

  vrdn = vec2_dot(vel_relative, contact.normal);

  impulse_norm_mag_denom =
    a->inv_mass + b->inv_mass + ra_cross_norm + rb_cross_norm;
  impulse_norm_mag = -(1 + e) * vrdn / impulse_norm_mag_denom;

  impulse_norm_dir = contact.normal;
  impulse_norm = vec2_scale(impulse_norm_dir, impulse_norm_mag);

  //
  // Compute the impulse along the tangent.
  //

  tangent = vec2_perp(contact.normal);

  ra_cross_tang = vec2_cross(ra, tangent);
  ra_cross_tang *= ra_cross_tang;
  ra_cross_tang *= a->inv_inertia;

  rb_cross_tang = vec2_cross(rb, tangent);
  rb_cross_tang *= rb_cross_tang;
  rb_cross_tang *= b->inv_inertia;

  vrdt = vec2_dot(vel_relative, tangent);

  impulse_tang_mag_denom =
    a->inv_mass + b->inv_mass + ra_cross_tang + rb_cross_tang;
  impulse_tang_mag = f * -(1 + e) * vrdt / impulse_tang_mag_denom;

  impulse_tang_dir = tangent;
  impulse_tang = vec2_scale(impulse_tang_dir, impulse_tang_mag);

  //
  // Now compute the impulse.
  //

  impulse = vec2_add(impulse_norm, impulse_tang);

  //
  // Lastly, apply the impulses.
  //

  body_apply_impulse(*a, impulse, ra);
  body_apply_impulse(*b, vec2_scale(impulse, -1), rb);
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
  separation_info sep_ab;
  separation_info sep_ba;

  find_min_separation(a_verts, a_vert_count, b_verts, b_vert_count, sep_ab);
  if (sep_ab.amount >= 0.0f) {
    return false;
  }

  find_min_separation(b_verts, b_vert_count, a_verts, a_vert_count, sep_ba);
  if (sep_ba.amount >= 0.0f) {
    return false;
  }

  //
  // We have a collision, so populate the contact information. N.B. the ab and
  // ba separations wont neccessarily be the same. Consider if the face of a
  // hits the corner of b, or the corner of a hits the face of b. The rule is:
  // face to corner is always what we want.
  //

  contact.a = body_a;
  contact.b = body_b;

  if (sep_ab.amount > sep_ba.amount) {
    contact.depth = -sep_ab.amount;
    contact.normal = vec2_perp(sep_ab.axis);
    contact.start = sep_ab.point;
    contact.end = vec2_add(
      sep_ab.point,
      vec2_scale(contact.normal, contact.depth)
    );
  } else {
    contact.depth = -sep_ba.amount;
    contact.normal = vec2_scale(vec2_perp(sep_ba.axis), -1.0f);
    contact.start = vec2_sub(
      sep_ba.point,
      vec2_scale(contact.normal, contact.depth)
    );
    contact.end = sep_ba.point;
  }


  return true;
}

void find_min_separation(
  const vec2def* a_verts,
  const size_t a_vert_count,
  const vec2def* b_verts,
  const size_t b_vert_count,
  separation_info& result
) {
  vec2def edge_a;
  vec2def edge_norm_a;
  float proj_vb_ea;
  size_t next_point;
  float next_separation;
  size_t va;
  size_t va1;
  vec2def va_to_vb;
  size_t vb;

  //
  // Defining "best" separation. For a given edge, consider its normal. If we
  // drew it on paper, it would be an arrow pointing away from the edge and
  // perpendicular to it. Note, we imply edges have a direction - indeed! The
  // direction is from the origin to the destination, if we walked the edge in
  // clockwise order.
  //
  // So the normal can be thought of as "the way the edge is facing" in a sense.
  // If a point is in front of the edge, then there is a gap between that point
  // and the edge - and hence separation. So oppositely, if a point is "behind"
  // this edge, then there is penetration.
  //
  // The name of the game is thus: for each edge of A, we are looking for the
  // least possible amount of separation. That is, suppose that vb is the vert
  // of b that is the closest to an edge of a. How much separation is that?
  // That's the value we want for that edge of a. Now for each of these
  // separation values of the edges of a, which one is the "worst" meaning it
  // provides the most separation between a and b? If this value is > 0, then
  // that means we have a gap between a and b, and hence they are not colliding.
  //

  result.amount = std::numeric_limits<float>::lowest();

  for (va = 0; va < a_vert_count; va++) {
    // Get the edge from va to va1.
    va1 = (va + 1) % a_vert_count;
    edge_a = vec2_sub(a_verts[va1], a_verts[va]);
    edge_norm_a = vec2_perp(edge_a);

    // Calculate the minimum separation between the verts of b and this edge of
    // a.
    next_separation = std::numeric_limits<float>::max();
    for (vb = 0; vb < b_vert_count; vb++) {
      // Project vb onto edge_a.
      va_to_vb = vec2_sub(b_verts[vb], a_verts[va]);
      proj_vb_ea = vec2_dot(va_to_vb, edge_norm_a);

      if (proj_vb_ea < next_separation) {
        next_separation = proj_vb_ea;
        next_point = vb;
      }
    }

    // Now decide the most separation amongst the separations of each edge.
    if (next_separation > result.amount) {
      result.amount = next_separation;
      result.axis = edge_a;
      result.point = b_verts[next_point];
    }
  }
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
