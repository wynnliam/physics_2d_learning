// Liam Wynn, 4-9-2026, 2D Physics Engine

/*
  Here we handle all logic for detecting and resolving collisions.
*/

#ifndef COLLISION
#define COLLISION

#include "./body.h"
#include "./shape/shapedef.h"

struct collision_contact {
  const body* a;
  const body* b;

  // Contact points *from* a *to* b.
  vec2def start;
  vec2def end;

  vec2def normal;
  float depth;
};

bool is_colliding(const body* a, const body* b, collision_contact& contact);

#endif
