// Liam Wynn, 5-3-2026, 2D Physics Engine

#include "./vecndef.h"

void vecn_init(vecndef& v, const size_t n) {
  v.n = n;
  v.data = new float[v.n];
  vecn_zero(v);
}

void vecn_cleanup(vecndef& v) {
  delete[] v.data;
  v.data = NULL;
}

void vecn_add(vecndef& dest, const vecndef& src) {
  size_t i;
  size_t n;

  n = dest.n;

  for (i = 0; i < n; i++) {
    dest.data[i] += src.data[i];
  }
}

void vecn_sub(vecndef& dest, const vecndef& src) {
  size_t i;
  size_t n;

  n = dest.n;

  for (i = 0; i < n; i++) {
    dest.data[i] -= src.data[i];
  }
}

void vecn_scale(vecndef& v, const float s) {
  size_t i;
  size_t n;

  n = v.n;

  for (i = 0; i < n; i++) {
    v.data[i] *= s;
  }
}

float vecn_dot(const vecndef& a, const vecndef& b) {
  size_t i;
  float result;

  result = 0.0f;

  for (i = 0; i < a.n; i++) {
    result += a.data[i] * b.data[i];
  }

  return result;
}

void vecn_zero(vecndef& v) {
  size_t i;

  for (i = 0; i < v.n; i++) {
    v.data[i] = 0.0f;
  }
}
