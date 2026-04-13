// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <variant>

/* SHAPE COLLISION ROUTINES */

static bool shape_collision(
  const circledef& a,
  const circledef& b,
  const vec2def& apos,
  const vec2def& bpos
);

// Generic not-yet-impl routine.
template<typename A, typename B>
static bool shape_collision(const A&, const B&, const vec2def&, const vec2def&);

/* MAIN API IMPL */

bool is_colliding(const body* a, const body* b) {
  return std::visit(
    [&](const auto& s1, const auto& s2) {
      return shape_collision(s1, s2, a->position, b->position);
    },
    a->shape,
    b->shape
  );
}

/* SHAPE COLLISION ROUTINE IMPL */

bool shape_collision(
  const circledef& a,
  const circledef& b,
  const vec2def& apos,
  const vec2def& bpos
) {
  vec2def ab;
  float dist;
  float rad_sum;
  bool result;

  ab = vec2_sub(bpos, apos);
  dist = vec2_mag_squared(ab);
  rad_sum = a.radius + b.radius;

  result = dist <= (rad_sum * rad_sum);

  return result;
}

template<typename A, typename B>
bool shape_collision(const A&, const B&, const vec2def&, const vec2def&) {
  return false;
}
