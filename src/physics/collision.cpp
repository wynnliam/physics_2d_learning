// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <algorithm>
#include <limits>
#include <variant>

using namespace std;

// When computing the min separation, we return this collection of data here.
struct separation_info {
  // The amount of separation.
  float amount;
  // The reference edge for which the separation was found.
  size_t ref_edge;
  // The point of the polygon that penetrated to produce the separation.
  vec2def support_point;
};

/* SHAPE COLLISION ROUTINES */

static bool shape_collision(
  const circledef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

static bool shape_collision(
  const boxdef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

static bool shape_collision(
  const polydef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

static bool shape_collision(
  const boxdef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

static bool shape_collision(
  const polydef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

bool shape_collision(
  const circledef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

bool shape_collision(
  const polydef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

bool shape_collision(
  const circledef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

bool shape_collision(
  const boxdef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
);

// Generic not-yet-impl routine.
template<typename A, typename B>
static bool shape_collision(
  const A&,
  const B&,
  body*,
  body*,
  vector<collision_contact>&
);

// General purpose collision handler for two polygons.
static bool poly_collision(
  const vec2def* a_verts,
  const size_t a_vert_count,
  body* body_a,
  const vec2def* b_verts,
  const size_t b_vert_count,
  body* body_b,
  vector<collision_contact>& contact
);

// Finds the minimum separation for two polygons
static void find_min_separation(
  const vec2def* a_verts,
  const size_t a_vert_count,
  const vec2def* b_verts,
  const size_t b_vert_count,
  separation_info& result
);

static vec2def edge_at(
  const vec2def* verts,
  const size_t num_verts,
  const size_t index
);

static size_t find_incident_edge_index(
  const vec2def* incident_shape,
  const size_t incident_vert_count,
  const vec2def& ref_normal
);

static size_t clip_segment_to_line(
  const vec2def* shape,
  const size_t vert_count,
  const vector<vec2def>& contact_points,
  vector<vec2def>& clipped_points,
  const vec2def& c0,
  const vec2def& c1
);

static bool poly_circle_collision(
  const vec2def* poly_verts,
  const size_t poly_vert_count,
  body* poly_body,
  const circledef& circle,
  body* circle_body,
  vector<collision_contact>& contact
);

/* MAIN API IMPL */

bool is_colliding(body* a, body* b, vector<collision_contact>& contacts) {
  return std::visit(
    [&](const auto& s1, const auto& s2) {
      return shape_collision(s1, s2, a, b, contacts);
    },
    a->shape,
    b->shape
  );

  return false;
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

  shape_transform(contact.a->shape, contact.a->position, contact.a->rotation);
  shape_transform(contact.b->shape, contact.b->position, contact.b->rotation);
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

  body_apply_impulse_at_point(*a, impulse, ra);
  body_apply_impulse_at_point(*b, vec2_scale(impulse, -1), rb);
}

/* SHAPE COLLISION ROUTINE IMPL */

bool shape_collision(
  const circledef& a,
  const circledef& b,
  body* a_body,
  body* b_body,
  vector<collision_contact>& contact
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

  contact.resize(1);

  contact[0].a = a_body;
  contact[0].b = b_body;
  contact[0].normal = vec2_norm(ab);
  contact[0].start = vec2_sub(bpos, vec2_scale(contact[0].normal, b.radius));
  contact[0].end = vec2_add(apos, vec2_scale(contact[0].normal, a.radius));
  contact[0].depth = vec2_magnitude(vec2_sub(contact[0].end, contact[0].start));

  return true;
}

bool shape_collision(
  const boxdef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
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
  vector<collision_contact>& contact
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

bool shape_collision(
  const boxdef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_collision(
    a.world_verts,
    4,
    body_a,
    b.world_vertices.data(),
    b.world_vertices.size(),
    body_b,
    contact
  );
}

bool shape_collision(
  const polydef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_collision(
    a.world_vertices.data(),
    a.world_vertices.size(),
    body_a,
    b.world_verts,
    4,
    body_b,
    contact
  );
}

bool shape_collision(
  const circledef& a,
  const polydef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_circle_collision(
    b.world_vertices.data(),
    b.world_vertices.size(),
    body_b,
    a,
    body_a,
    contact
  );
}

bool shape_collision(
  const polydef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_circle_collision(
    a.world_vertices.data(),
    a.world_vertices.size(),
    body_a,
    b,
    body_b,
    contact
  );
}

bool shape_collision(
  const circledef& a,
  const boxdef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_circle_collision(
    b.world_verts,
    4,
    body_b,
    a,
    body_a,
    contact
  );
}

bool shape_collision(
  const boxdef& a,
  const circledef& b,
  body* body_a,
  body* body_b,
  vector<collision_contact>& contact
) {
  return poly_circle_collision(
    a.world_verts,
    4,
    body_a,
    b,
    body_b,
    contact
  );
}

template<typename A, typename B>
bool shape_collision(
  const A&,
  const B&,
  body*,
  body*,
  vector<collision_contact>&
) {
  return false;
}

bool poly_collision(
  const vec2def* a_verts,
  const size_t a_vert_count,
  body* body_a,
  const vec2def* b_verts,
  const size_t b_vert_count,
  body* body_b,
  vector<collision_contact>& contact
) {
  vec2def c0;
  vec2def c1;
  vector<vec2def> contact_points;
  vector<vec2def> clipped_points;
  size_t i;
  size_t incident_edge_index;
  size_t incident_edge_index_next;
  collision_contact next_contact;
  size_t num_clipped;
  vec2def ref_edge;
  vec2def ref_edge_norm;
  float separation;
  separation_info sep_ab;
  separation_info sep_ba;
  const vec2def* shape_inc;
  size_t shape_inc_vert_count;
  const vec2def* shape_ref;
  size_t shape_ref_edge_index;
  size_t shape_ref_vert_count;
  vec2def vclip;
  vec2def vclip_minus_vref;
  vec2def vi0;
  vec2def vi1;
  vec2def vref;

  find_min_separation(a_verts, a_vert_count, b_verts, b_vert_count, sep_ab);
  if (sep_ab.amount >= 0.0f) {
    return false;
  }

  find_min_separation(b_verts, b_vert_count, a_verts, a_vert_count, sep_ba);
  if (sep_ba.amount >= 0.0f) {
    return false;
  }

  //
  // Decide who is the reference shape and who is the incident shape. From here
  // on out, we reason exclusively in terms of reference and incident shape.
  //

  if (sep_ab.amount > sep_ba.amount) {
    shape_ref = a_verts;
    shape_ref_vert_count = a_vert_count;
    shape_ref_edge_index = sep_ab.ref_edge;

    shape_inc = b_verts;
    shape_inc_vert_count = b_vert_count;
  } else {
    shape_ref = b_verts;
    shape_ref_vert_count = b_vert_count;
    shape_ref_edge_index = sep_ba.ref_edge;

    shape_inc = a_verts;
    shape_inc_vert_count = a_vert_count;
  }

  ref_edge = edge_at(shape_ref, shape_ref_vert_count, shape_ref_edge_index);
  ref_edge_norm = vec2_perp(ref_edge);

  //
  // Find the incident edge
  //

  incident_edge_index = find_incident_edge_index(
    shape_inc,
    shape_inc_vert_count,
    ref_edge_norm
  );

  //
  // Peform clipping.
  //

  incident_edge_index_next = (incident_edge_index + 1) % shape_inc_vert_count;
  vi0 = shape_inc[incident_edge_index];
  vi1 = shape_inc[incident_edge_index_next];

  contact_points = { vi0, vi1 };
  clipped_points = { vi0, vi1 };

  for (i = 0; i < shape_ref_vert_count; i++) {
    if (i == shape_ref_edge_index) {
      continue;
    }

    c0 = shape_ref[i];
    c1 = shape_ref[(i + 1) % shape_ref_vert_count];

    num_clipped = clip_segment_to_line(
      shape_ref,
      shape_ref_vert_count,
      contact_points,
      clipped_points,
      c0,
      c1
    );

    if (num_clipped < 2) {
      break;
    }

    contact_points = clipped_points;
  }

  vref = shape_ref[shape_ref_edge_index];

  for (i = 0; i < clipped_points.size(); i++) {
    vclip = clipped_points[i];
    vclip_minus_vref = vec2_sub(vclip, vref);

    separation = vec2_dot(vclip_minus_vref, ref_edge_norm);
    if (separation <= 0.0f) {
      next_contact.a = body_a;
      next_contact.b = body_b;
      next_contact.normal = ref_edge_norm;
      next_contact.start = vclip;
      next_contact.end =
        vec2_add(vclip, vec2_scale(ref_edge_norm, -separation));
      if (sep_ba.amount >= sep_ab.amount) {
        std::swap(next_contact.start, next_contact.end);
        next_contact.normal = vec2_scale(next_contact.normal, -1.0f);
      }
      next_contact.depth = 0.0f; // TODO: ???
      contact.push_back(next_contact);
    }
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
      result.ref_edge = va;
      result.support_point = b_verts[next_point];
    }
  }
}

vec2def edge_at(
  const vec2def* verts,
  const size_t num_verts,
  const size_t index
) {
  vec2def from;
  vec2def result;
  vec2def to;

  from = verts[index];
  to = verts[(index + 1) % num_verts];

  result = vec2_sub(to, from);

  return result;
}

size_t find_incident_edge_index(
  const vec2def* incident_shape,
  const size_t incident_vert_count,
  const vec2def& ref_normal
) {
  vec2def curr_edge;
  vec2def curr_edge_norm;
  float curr_proj;
  size_t i;
  size_t index_incident_edge;
  float min_proj;

  min_proj = std::numeric_limits<float>::max();
  index_incident_edge = 0;

  for (i = 0; i < incident_vert_count; i++) {
    curr_edge = edge_at(incident_shape, incident_vert_count, i);
    curr_edge_norm = vec2_perp(curr_edge);
    curr_proj = vec2_dot(curr_edge_norm, ref_normal);

    if (curr_proj < min_proj) {
      min_proj = curr_proj;
      index_incident_edge = i;
    }
  }

  return index_incident_edge;
}

size_t clip_segment_to_line(
  const vec2def* shape,
  const size_t vert_count,
  const vector<vec2def>& contact_points,
  vector<vec2def>& clipped_points,
  const vec2def& c0,
  const vec2def& c1
) {
  vec2def contact;
  float dist_0;
  float dist_1;
  vec2def norm;
  size_t result;
  float t;
  float total_dist;

  result = 0;

  norm = vec2_sub(c1, c0);
  norm = vec2_norm(norm);
  dist_0 = vec2_cross(vec2_sub(contact_points[0], c0), norm);
  dist_1 = vec2_cross(vec2_sub(contact_points[1], c0), norm);

  if (dist_0 <= 0.0f) {
    clipped_points[result] = contact_points[0];
    result++;
  }

  if (dist_1 <= 0.0f) {
    clipped_points[result] = contact_points[1];
    result++;
  }

  if (dist_0 * dist_1 < 0.0f) {
    total_dist = dist_0 - dist_1;
    t = dist_0 / total_dist;

    contact = vec2_add(
      contact_points[0],
      vec2_scale(
        vec2_sub(contact_points[1], contact_points[0]),
        t
      )
    );

    clipped_points[result] = contact;
    result++;
  }

  return result;
}

bool poly_circle_collision(
  const vec2def* poly_verts,
  const size_t poly_vert_count,
  body* poly_body,
  const circledef& circle,
  body* circle_body,
  vector<collision_contact>& contact
) {
  float best_proj;
  size_t best_v;
  size_t best_v1;
  vec2def circle_center;
  vec2def closest_edge;
  vec2def closest_point_on_edge;
  float dist;
  vec2def edge_origin_to_circle;
  vec2def next_edge;
  vec2def next_edge_norm;
  float next_proj;
  float t;
  size_t v;
  size_t v1;

  circle_center = circle_body->position;
  best_proj = std::numeric_limits<float>::lowest();

  //
  // First we need to find the edge with the most separation between it and the
  // circle. Note that most is not just absolute distance from the edge. The
  // normal of the edge points the direction of "positive". So the "most"
  // separation is the most in the direction of the edge's normal.
  //

  for (v = 0; v < poly_vert_count; v++) {
    v1 = (v + 1) % poly_vert_count;

    next_edge = vec2_sub(poly_verts[v1], poly_verts[v]);
    next_edge_norm = vec2_perp(next_edge);

    edge_origin_to_circle = vec2_sub(circle_center, poly_verts[v]);
    next_proj = vec2_dot(edge_origin_to_circle, next_edge_norm);

    if (next_proj > best_proj) {
      best_proj = next_proj;
      best_v = v;
      best_v1 = v1;
    }
  }

  closest_edge = vec2_sub(poly_verts[best_v1], poly_verts[best_v]);
  edge_origin_to_circle = vec2_sub(circle_center, poly_verts[best_v]);
  
  t = vec2_dot(edge_origin_to_circle, closest_edge);
  t = t / vec2_dot(closest_edge, closest_edge);
  t = std::clamp(t, 0.0f, 1.0f);

  closest_point_on_edge = vec2_add(
    poly_verts[best_v],
    vec2_scale(closest_edge, t)
  );

  //
  // The circle is inside the polygon, so return true.
  //

  if (best_proj < 0) {
    contact.resize(1);

    contact[0].a = poly_body;
    contact[0].b = circle_body;
    contact[0].depth = circle.radius - best_proj;
    contact[0].normal = vec2_perp(closest_edge);
    contact[0].start = vec2_sub(
      circle_center,
      vec2_scale(contact[0].normal, circle.radius)
    );
    contact[0].end = vec2_add(
      contact[0].start,
      vec2_scale(contact[0].normal, contact[0].depth)
    );

    return true;
  }

  dist = vec2_mag_squared(vec2_sub(circle_center, closest_point_on_edge));

  if (dist < (circle.radius * circle.radius)) {
    contact.resize(1);

    contact[0].a = poly_body;
    contact[0].b = circle_body;
    contact[0].depth = circle.radius - sqrt(dist);
    contact[0].normal =
      vec2_norm(vec2_sub(circle_center, closest_point_on_edge));
    contact[0].start = vec2_sub(
      circle_center,
      vec2_scale(contact[0].normal, circle.radius)
    );
    contact[0].end = closest_point_on_edge;

    return true;
  }

  return false;
}
