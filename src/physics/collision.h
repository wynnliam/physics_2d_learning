// Liam Wynn, 4-9-2026, 2D Physics Engine

/*
  Here we handle all logic for detecting and resolving collisions.
*/

#ifndef COLLISION
#define COLLISION

#include "./body.h"
#include "./shape/shapedef.h"

bool is_colliding(const body* a, const body* b);

#endif
