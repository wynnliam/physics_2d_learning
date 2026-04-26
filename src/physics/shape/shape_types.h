// Liam Wynn, 3-30-2026, 2D Physics Engine

/*
  Here we define the possible rigid-body shape types we will implement in this
  engine.

  For future reference, it's not clear to me that boxdef is really all that
  useful here. If I were to keep it, I would define it as a strictly AABB box
  to distinguish it from polydef. But even then, SAT would handle all the
  things we'd want from it.
*/

#ifndef SHAPE_TYPES
#define SHAPE_TYPES

#include <cstdint>
#include "../../graphics.h"
#include "../vec2def.h"
#include <vector>

#define BOX_TL  0
#define BOX_TR  1
#define BOX_BR  2
#define BOX_BL  3

enum class shape_type {
  CIRCLE,
  BOX,
  POLYGON
};

struct circledef {
  float radius;
};

struct boxdef {
  float width;
  float height;

  vec2def local_verts[4];
  vec2def world_verts[4];
};

struct polydef {

  //
  // N.B. all vertices (local and world) must be defined in Clock-Wise Ordering.
  //

  std::vector<vec2def> local_vertices;

  std::vector<vec2def> world_vertices;
};

void shape_init(circledef& c);
void shape_init(boxdef& b);
void shape_init(polydef& p);

shape_type get_shape_type(const circledef& c);
shape_type get_shape_type(const boxdef& b);
shape_type get_shape_type(const polydef& p);

float get_moment_of_inertia(const circledef& c);
float get_moment_of_inertia(const boxdef& b);
float get_moment_of_inertia(const polydef& p);

void transform_shape(circledef& c, const vec2def& pos, const float angle);
void transform_shape(boxdef& b, const vec2def& pos, const float angle);
void transform_shape(polydef& p, const vec2def& pos, const float angle);

void draw_shape(
  graphics& gr,
  const circledef& c,
  SDL_Texture* tex,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

void draw_shape(
  graphics& gr,
  const boxdef& b,
  SDL_Texture* tex,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

void draw_shape(
  graphics& gr,
  const polydef& p,
  SDL_Texture* tex,
  const int x,
  const int y,
  const float angle,
  const uint32_t color
);

#endif

