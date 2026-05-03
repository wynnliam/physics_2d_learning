// Liam Wynn, 5-2-2026, 2D Physics Engine

#ifndef CONSTRAINT
#define CONSTRAINT

#include "./body.h"

struct constraint {
  body* a;
  body* b;

  //mat_mn get_inv_m();
  //vec_n vec;
};

void constraint_solve(constraint& c);

#endif

