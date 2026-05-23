// Liam Wynn, 5-5-2026, 2D Physics Engine

#include "./matrix.h"

using namespace std;

void matrix_init(matrix& mat, const size_t m, const size_t n) {
  size_t i;

  mat.m = m;
  mat.n = n;

  mat.rows = new vecndef[mat.m];

  for (i = 0; i < mat.m; i++) {
    vecn_init(mat.rows[i], mat.n);   
  }
}

void matrix_copy(matrix& dest, const matrix& src, const bool do_cleanup) {
  size_t i;

  if (do_cleanup) {
    matrix_cleanup(dest);
  }

  dest.m = src.m;
  dest.n = src.n;
  dest.rows = new vecndef[dest.m];

  //
  // If we do cleanup at the matrix level, we will clean up the vecndefs. So we
  // do not want to try and clean them up again. Otherwise, we do want to clean
  // them up.
  //

  for (i = 0; i < dest.m; i++) {
    vecn_copy(dest.rows[i], src.rows[i], !do_cleanup);
  }
}

void matrix_cleanup(matrix& mat) {
  size_t i;

  for (i = 0; i < mat.m; i++) {
    vecn_cleanup(mat.rows[i]);
  }

  delete[] mat.rows;

  mat.rows = NULL;
  mat.m = 0;
  mat.n = 0;
}

void matrix_zero(matrix& mat) {
  size_t i;

  for (i = 0; i < mat.m; i++) {
    vecn_zero(mat.rows[i]);
  }
}

matrix matrix_transpose(const matrix& mat) {
  size_t i;
  size_t j;
  size_t m;
  size_t n;
  matrix result;

  //
  // Capture the m and n of mat. Once we know that m and n are talking about the
  // source mat's m and n, the semantics of transposing are nicely anchored.
  //

  m = mat.m;
  n = mat.n;

  matrix_init(result, n, m);

  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      result.rows[j].data[i] = mat.rows[i].data[j];
    }
  }

  return result;
}

optional<matrix> matrix_mat_mul(const matrix& a, const matrix& b) {
  size_t i;
  size_t j;
  matrix b_transposed;
  size_t num_cols;
  size_t num_rows;
  matrix result;

  if (b.m != a.n) {
    return {};
  }

  num_rows = a.m;
  num_cols = b.n;
  matrix_init(result, num_rows, num_cols);

  //
  // Transpose b so we can compute each element with vecn_dot.
  //

  b_transposed = matrix_transpose(b);

  for (i = 0; i < num_rows; i++) {
    for (j = 0; j < num_cols; j++) {
      result.rows[i].data[j] = vecn_dot(a.rows[i], b_transposed.rows[j]);
    }
  }

  //
  // Now clean up b_transposed since we allocated memory for it.
  //

  matrix_cleanup(b_transposed);

  return result;
}

optional<vecndef> matrix_vecn_mul(const matrix& mat, const vecndef& v) {
  size_t i;
  size_t num_entries;
  vecndef result;

  if (mat.n != v.n) {
    return {};
  }

  num_entries = mat.m;
  vecn_init(result, num_entries);

  for (i = 0; i < num_entries; i++) {
    result.data[i] = vecn_dot(mat.rows[i], v);
  }

  return result;
}

vecndef matrix_solve_gauss_seidel(const matrix& A, const vecndef& b) {
  float adi;
  float adx;
  size_t i;
  size_t n;
  vecndef result;
  size_t curr_iter;

  n = b.n;

  vecn_init(result, n);
  vecn_zero(result);

  for (curr_iter = 0; curr_iter < n; curr_iter++) {
    for (i = 0; i < n; i++) {
      adi = A.rows[i].data[i];
      if (adi != 0.0f) {
        adx = vecn_dot(A.rows[i], result);
        result.data[i] += (b.data[i] - adx) / adi;
      }
    }
  }

  return result;
}
