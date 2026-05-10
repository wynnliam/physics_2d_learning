// Liam Wynn, 5-5-2026, 2D Physics Engine

#ifndef MATRIX
#define MATRIX

#include "./vecndef.h"
#include <optional>

struct matrix {
  // Num rows
  size_t m;
  // Num cols
  size_t n;

  // Each vecn is N dimension. There will be M of these vecndefs.
  vecndef* rows;
};

void matrix_init(matrix& mat, const size_t m, const size_t n);

void matrix_copy(matrix& dest, const matrix& src, const bool do_cleanup);

void matrix_cleanup(matrix& mat);

void matrix_zero(matrix& mat);

matrix matrix_transpose(const matrix& mat);

std::optional<matrix> matrix_mat_mul(const matrix& a, const matrix& b);

std::optional<vecndef> matrix_vecn_mul(const matrix& mat, const vecndef& v);

#endif

