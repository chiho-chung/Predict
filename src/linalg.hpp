#pragma once

#include <cmath>

// Minimal fixed-size dense linear algebra for the target filters. Doubles, not
// floats: the 8x8 covariance is inverted and factorised every update and float
// loses positive-definiteness too easily there.
namespace la {

using Scalar = double;

template <int R, int C>
struct Mat {
  Scalar a[R][C]{};

  Scalar& operator()(int i, int j) { return a[i][j]; }
  Scalar operator()(int i, int j) const { return a[i][j]; }
};

template <int N>
using Vec = Mat<N, 1>;

template <int N>
Mat<N, N> identity() {
  Mat<N, N> m;
  for (int i = 0; i < N; ++i) m(i, i) = 1.0;
  return m;
}

template <int R, int C, int K>
Mat<R, K> operator*(const Mat<R, C>& x, const Mat<C, K>& y) {
  Mat<R, K> out;
  for (int i = 0; i < R; ++i) {
    for (int k = 0; k < K; ++k) {
      Scalar s = 0;
      for (int j = 0; j < C; ++j) s += x(i, j) * y(j, k);
      out(i, k) = s;
    }
  }
  return out;
}

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& x, const Mat<R, C>& y) {
  Mat<R, C> out;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j) out(i, j) = x(i, j) + y(i, j);
  return out;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& x, const Mat<R, C>& y) {
  Mat<R, C> out;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j) out(i, j) = x(i, j) - y(i, j);
  return out;
}

template <int R, int C>
Mat<R, C> operator*(Scalar s, const Mat<R, C>& x) {
  Mat<R, C> out;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j) out(i, j) = s * x(i, j);
  return out;
}

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& x) {
  Mat<C, R> out;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j) out(j, i) = x(i, j);
  return out;
}

// Force exact symmetry; repeated Kalman updates drift otherwise.
template <int N>
Mat<N, N> symmetrize(const Mat<N, N>& x) {
  Mat<N, N> out;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) out(i, j) = 0.5 * (x(i, j) + x(j, i));
  return out;
}

// Gauss-Jordan with partial pivoting. Returns false if singular.
template <int N>
bool inverse(const Mat<N, N>& in, Mat<N, N>& out) {
  Scalar a[N][2 * N]{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) a[i][j] = in(i, j);
    a[i][N + i] = 1.0;
  }

  for (int col = 0; col < N; ++col) {
    int piv = col;
    for (int r = col + 1; r < N; ++r) {
      if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
    }
    if (std::fabs(a[piv][col]) < 1e-300) return false;
    if (piv != col) {
      for (int j = 0; j < 2 * N; ++j) {
        const Scalar t = a[col][j];
        a[col][j] = a[piv][j];
        a[piv][j] = t;
      }
    }
    const Scalar inv = 1.0 / a[col][col];
    for (int j = 0; j < 2 * N; ++j) a[col][j] *= inv;
    for (int r = 0; r < N; ++r) {
      if (r == col) continue;
      const Scalar f = a[r][col];
      if (f == 0.0) continue;
      for (int j = 0; j < 2 * N; ++j) a[r][j] -= f * a[col][j];
    }
  }

  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) out(i, j) = a[i][N + j];
  return true;
}

// Lower-triangular Cholesky factor, used for UKF sigma points.
template <int N>
bool cholesky(const Mat<N, N>& in, Mat<N, N>& L) {
  L = Mat<N, N>{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j <= i; ++j) {
      Scalar s = in(i, j);
      for (int k = 0; k < j; ++k) s -= L(i, k) * L(j, k);
      if (i == j) {
        if (s <= 1e-12) return false;
        L(i, j) = std::sqrt(s);
      } else {
        L(i, j) = s / L(j, j);
      }
    }
  }
  return true;
}

template <int N>
Scalar quad_form(const Vec<N>& r, const Mat<N, N>& inv) {
  Scalar s = 0;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) s += r(i, 0) * inv(i, j) * r(j, 0);
  return s;
}

template <int N>
Scalar determinant_from_chol(const Mat<N, N>& L) {
  Scalar d = 1.0;
  for (int i = 0; i < N; ++i) d *= L(i, i);
  return d * d;
}

}  // namespace la
