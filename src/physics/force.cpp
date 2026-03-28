// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./force.h"
#include <algorithm>

vec2def generate_drag_force(const particle& p, const float k) {
  vec2def drag_dir;
  float drag_mag;
  vec2def result(0, 0);
  float v_mag;

  //
  // Drag = 0.5 * p * Kd * A * mag_sqrd(v) * -norm(v)
  //
  // We simplify this by noting 0.5, p, Kd, and A are all constants. Thus, we
  // compute:
  //
  // Drag = k * mag_sqrd(v) * -norm(v)
  //

  v_mag = vec2_mag_squared(p.velocity);

  if (v_mag == 0.0f) {
    return result;
  }

  drag_dir = vec2_norm(vec2_scale(p.velocity, -1.0f));
  drag_mag = k * v_mag;
  result = vec2_scale(drag_dir, drag_mag);

  return result;
}

vec2def generate_friction_force(const particle& p, const float k) {
  vec2def result(0, 0);
  vec2def v_dir;

  //
  // Normally, friction is defined as:
  // Friction = u * mag(force_normal) * -normal(v)
  //
  // Note that u and mag(force_normal) can be simplified to k, and hence, we do
  // Friction = k * -normal(v)
  //

  v_dir = vec2_norm(p.velocity);
  v_dir = vec2_scale(v_dir, -1.0f);

  result = vec2_scale(v_dir, k);

  return result;
}

vec2def generate_gravitational_force(
  const particle& a,
  const particle& b,
  const float G,
  const float min_dist,
  const float max_dist
) {
  vec2def attraction_dir;
  float attraction_mag;
  vec2def d;
  float dms;
  vec2def result;

  d = vec2_sub(b.position, a.position);
  dms = vec2_mag_squared(d);

  dms = std::clamp(dms, min_dist, max_dist);

  attraction_dir = vec2_norm(d);
  attraction_mag = G * (a.mass * b.mass) / dms;

  result = vec2_scale(attraction_dir, attraction_mag);

  return result;
}

vec2def generate_spring_force(
  const particle& p,
  const vec2def anchor,
  const float rest_length,
  const float k
) {
  vec2def anchor_to_p;
  float displacement;
  vec2def result;
  vec2def spring_dir;
  float spring_mag;

  //
  // Hooke's Law: Spring Force = -k * displacement_len
  //

  anchor_to_p = vec2_sub(p.position, anchor);
  displacement = vec2_magnitude(anchor_to_p) - rest_length;

  spring_dir = vec2_norm(anchor_to_p);
  spring_mag = -k * displacement;

  result = vec2_scale(spring_dir, spring_mag);

  return result;
}

vec2def generate_spring_force(
  const particle& a,
  const particle& b,
  const float rest_length,
  const float k
) {
  vec2def b_to_a;
  float ba_dist;
  float displacement;
  vec2def result;
  vec2def spring_dir;
  float spring_mag;

  b_to_a = vec2_sub(a.position, b.position);
  ba_dist = vec2_magnitude(b_to_a);
  displacement = ba_dist - rest_length;

  spring_dir = vec2_norm(b_to_a);
  spring_mag = -k * displacement;

  result = vec2_scale(spring_dir, spring_mag);

  return result;
}
