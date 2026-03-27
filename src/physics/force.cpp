// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./force.h"

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
