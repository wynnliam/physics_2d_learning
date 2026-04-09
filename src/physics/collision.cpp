// Liam Wynn, 4-9-2026, 2D Physics Engine

#include "./collision.h"
#include <variant>

/* SHAPE COLLISION ROUTINES */

static bool shape_collision(const circledef& a, const circledef& b);

// Generic not-yet-impl routine.
template<typename A, typename B>
static bool shape_collision(const A&, const B&);

/* MAIN API IMPL */

bool is_colliding(const body* a, const body* b) {
  return std::visit(
    [](const auto& s1, const auto& s2) { return shape_collision(s1, s2); },
    a->shape,
    b->shape
  );
}

/* SHAPE COLLISION ROUTINE IMPL */

bool shape_collision(const circledef& a, const circledef& b) {
  return false;
}

template<typename A, typename B>
static bool shape_collision(const A&, const B&) {
  return false;
}
