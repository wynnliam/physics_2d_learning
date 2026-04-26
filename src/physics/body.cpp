// Liam Wynn, 3-24-2026, 2D Physics Engine

#include "./body.h"
// TODO: Should not be here!
#include <SDL2/SDL_image.h>

void body_init(
  body& p,
  const shapedef& shape,
  const float x,
  const float y,
  const float mass,
  const float restitution,
  const float friction
) {
  p.position = vec2def(x, y);

  p.rotation = 0.0f;
  p.angular_velocity = 0.0f;
  p.angular_acceleration = 0.0f;

  p.shape = shape;

  p.mass = mass;
  if (p.mass != 0.0f) {
    p.inv_mass = 1.0f / p.mass;
  } else {
    p.inv_mass = 0.0f;
  }

  p.inertia = shape_get_moment_of_inertia(shape) * p.mass;
  if (p.inertia != 0.0f) {
    p.inv_inertia = 1.0f / p.inertia;
  } else {
    p.inv_inertia = 0.0f;
  }

  p.restitution = restitution;
  p.friction = friction;

  body_clear_forces(p);
  body_clear_torque(p);

  p.texture = NULL;
}

void body_set_texture(body& p, graphics& gr, const char* texture_path) {
  SDL_Surface* surface;

  surface = IMG_Load(texture_path);

  if (surface == NULL) {
    return;
  }

  p.texture = SDL_CreateTextureFromSurface(gr.renderer, surface);
  SDL_FreeSurface(surface);
}

void body_add_force(body& p, const vec2def& force) {
  p.sum_force = vec2_add(force, p.sum_force);
}

void body_add_torque(body& p, const float torque) {
  p.sum_torque += torque;
}

void body_apply_linear_impulse(body& p, const vec2def& impulse) {
  if (body_is_static(p)) {
    return;
  }

  p.velocity = vec2_add(p.velocity, vec2_scale(impulse, p.inv_mass));
}

void body_apply_impulse(body& p, const vec2def& impulse, const vec2def& r) {
  if (body_is_static(p)) {
    return;
  }

  p.velocity = vec2_add(p.velocity, vec2_scale(impulse, p.inv_mass));
  p.angular_velocity += vec2_cross(r, impulse) * p.inv_inertia;
}

void body_update(body& p, const float delta_time) {
  body_integrate_linear(p, delta_time);
  body_integrate_angular(p, delta_time);
  shape_transform(p.shape, p.position, p.rotation);
}

void body_integrate_linear(body& p, const float delta_time) {
  if (body_is_static(p)) {
    return;
  }

  //
  // First, calculate the final acceleration of the body for this frame. We
  // apply the law acceleration = force / mass to get the acceleration, and then
  // we perform a step of integration.
  //

  p.acceleration = vec2_scale(p.sum_force, p.inv_mass);

  //
  // Perform two steps of Euler Integration to find the position. First, we
  // integrate the acceleration to compute the velocity. Then, we integrate the
  // velocity to find the position.
  //

  p.velocity = vec2_add(vec2_scale(p.acceleration, delta_time), p.velocity);
  p.position = vec2_add(vec2_scale(p.velocity, delta_time), p.position);

  //
  // After integrating we need to clear the forces since they only apply for
  // this frame.
  //

  body_clear_forces(p);
}

void body_integrate_angular(body& p, const float delta_time) {
  if (body_is_static(p)) {
    return;
  }

  //
  // Compute the angular accelertaion using the formula a = I / t. N.B. this is
  // a simplification of the torque formula t = F * d * sin(B). F is the force,
  // d is the distance from the center of mass that the force hits, and B is
  // the angle of the force.
  //

  p.angular_acceleration = p.sum_torque * p.inv_inertia;

  //
  // Perform two steps of Euler Integration to find the rotation. First we
  // integrate the angular acceleration to find the velocity; and then we
  // integrate the angular velocity to find the rotation.
  //

  p.angular_velocity += p.angular_acceleration * delta_time;
  p.rotation += p.angular_velocity * delta_time;

  body_clear_torque(p);
}

void body_clear_forces(body& p) {
  p.sum_force = vec2def(0.0f, 0.0f);
}

void body_clear_torque(body& p) {
  p.sum_torque = 0.0f;
}

bool body_is_static(const body& p) {
  const float epsilon = 0.005;
  return fabs(p.inv_mass - 0.0f) < epsilon;
}
