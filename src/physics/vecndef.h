// Liam Wynn, 5-2-2026, 2D Physics Engine

/*
  Defines a general N-Vector, which we will use in constraint satisfaction.
  While we could replace the vec3 and vec2 with a vecN, these two are special
  enough in the code to warrant their own classes.
*/

#ifndef VECN
#define VECN

#include <stddef.h>

struct vecndef {
  size_t n;
  float* data;
};

//
// N.B. WE ASSUME ALL VECTOR ARGS GIVEN HAVE EQUAL N. THE ONUS IS ON *YOU* TO
// ENSURE THIS HOLDS!!!
//

void vecn_init(vecndef& v, const size_t n);

void vecn_cleanup(vecndef& v);

// dest += src
void vecn_add(vecndef& dest, const vecndef& src);

// dest -= src
void vecn_sub(vecndef& dest, const vecndef& src);

// dest *= s
void vecn_scale(vecndef& v, const float s);

float vecn_dot(const vecndef& a, const vecndef& b);

void vecn_zero(vecndef& v);

#endif
