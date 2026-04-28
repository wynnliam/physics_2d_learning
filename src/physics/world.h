// Liam Wynn, 4-28-2026, 2D Physics Engine

/*
  Defines a container for all physics objects. It's primary purpose is to
  abstract and contain all physics stuff into a single place.
*/

#ifndef WORLD
#define WORLD

#include "./body.h"
#include "./vec2def.h"
#include <vector>

struct world {
  std::vector<body*> bodies;
  std::vector<vec2def> forces;
  std::vector<float> torques;
  float gravity;
};

void world_init(world& w, const float gravity);

void world_add_body(world& w, body* b);

void world_add_force(world& w, const vec2def& force);

void world_add_torque(world& w, const float torque);

void world_update(world& w, const float delta_time);

void world_check_collisions(world& w);

void world_cleanup(world& w);

#endif
