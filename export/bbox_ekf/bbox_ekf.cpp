#include "bbox_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace bbox_ekf {
namespace {

constexpr int N = BBoxEkf::N;
constexpr int M = 4;
constexpr double kMinDepth = 0.35;
constexpr double kMinSize = 0.03;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kPi = 3.14159265358979323846;

template <int R, int C>
struct Mat {
  double a[R][C]{};
  double& operator()(int i, int j) { return a[i][j]; }
  double operator()(int i, int j) const { return a[i][j]; }
};
template <int D>
using Vec = Mat<D, 1>;

template <int D>
Mat<D, D> identity() {
  Mat<D, D> m;
  for (int i = 0; i < D; ++i) m(i, i) = 1.0;
  return m;
}

template <int R, int C, int K>
Mat<R, K> operator*(const Mat<R, C>& x, const Mat<C, K>& y) {
  Mat<R, K> out;
  for (int i = 0; i < R; ++i) {
    for (int k = 0; k < K; ++k) {
      double s = 0;
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
Mat<C, R> transpose(const Mat<R, C>& x) {
  Mat<C, R> out;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j) out(j, i) = x(i, j);
  return out;
}

template <int D>
Mat<D, D> symmetrize(const Mat<D, D>& x) {
  Mat<D, D> out;
  for (int i = 0; i < D; ++i)
    for (int j = 0; j < D; ++j) out(i, j) = 0.5 * (x(i, j) + x(j, i));
  return out;
}

template <int D>
bool inverse(const Mat<D, D>& in, Mat<D, D>& out) {
  double a[D][2 * D]{};
  for (int i = 0; i < D; ++i) {
    for (int j = 0; j < D; ++j) a[i][j] = in(i, j);
    a[i][D + i] = 1.0;
  }
  for (int col = 0; col < D; ++col) {
    int piv = col;
    for (int r = col + 1; r < D; ++r) {
      if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
    }
    if (std::fabs(a[piv][col]) < 1e-300) return false;
    if (piv != col) {
      for (int j = 0; j < 2 * D; ++j) {
        const double t = a[col][j];
        a[col][j] = a[piv][j];
        a[piv][j] = t;
      }
    }
    const double inv = 1.0 / a[col][col];
    for (int j = 0; j < 2 * D; ++j) a[col][j] *= inv;
    for (int r = 0; r < D; ++r) {
      if (r == col) continue;
      const double f = a[r][col];
      if (f == 0.0) continue;
      for (int j = 0; j < 2 * D; ++j) a[r][j] -= f * a[col][j];
    }
  }
  for (int i = 0; i < D; ++i)
    for (int j = 0; j < D; ++j) out(i, j) = a[i][D + j];
  return true;
}

template <int D>
double quad_form(const Vec<D>& r, const Mat<D, D>& inv) {
  double s = 0;
  for (int i = 0; i < D; ++i)
    for (int j = 0; j < D; ++j) s += r(i, 0) * inv(i, j) * r(j, 0);
  return s;
}

struct Intr {
  double fx, fy, cu, cv;
};

Intr intr_of(const Camera& cam) {
  Intr p;
  p.fx = cam.fx;
  p.fy = cam.fy;
  p.cu = cam.width * 0.5;
  p.cv = cam.height * 0.5;
  return p;
}

double comp(const Vec3& v, int i) {
  return (i == 0) ? v.x : (i == 1) ? v.y : v.z;
}

double wrap_pi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

Vec3 los_dir(float heading_deg, float attack_deg) {
  const float h = heading_deg * static_cast<float>(kDegToRad);
  const float a = attack_deg * static_cast<float>(kDegToRad);
  const float ca = std::cos(a);
  return Vec3{std::sin(h) * ca, std::cos(h) * ca, std::sin(a)}.normalized();
}

void los_from_p(double px, double py, double pz, double& hdg, double& att,
                double& range) {
  range = std::sqrt(px * px + py * py + pz * pz);
  if (range < kMinDepth) range = kMinDepth;
  const double horiz = std::sqrt(px * px + py * py);
  hdg = std::atan2(px, py);
  att = std::atan2(pz, std::max(horiz, 1e-9));
}

bool use_size_bias(const Config& cfg) { return cfg.meas_corr >= 0.05f; }
bool use_los_bias(const Config& cfg) { return cfg.los_bias_sigma_deg >= 0.01f; }
bool use_angle_bias(const Config& cfg) {
  return use_los_bias(cfg) || use_size_bias(cfg);
}

bool finite_state(const Vec<N>& x) {
  for (int i = 0; i < N; ++i) {
    if (!std::isfinite(x(i, 0))) return false;
  }
  return true;
}

// z = [heading_rad, attack_rad, width_px, height_px]
Vec<M> h_of(const Vec<N>& x, const Intr& cam, bool los_bias, bool size_bias) {
  double hdg = 0, att = 0, range = kMinDepth;
  los_from_p(x(0, 0), x(1, 0), x(2, 0), hdg, att, range);
  const double sw = std::max(kMinSize, x(6, 0));
  const double sh = std::max(kMinSize, x(7, 0));
  Vec<M> z;
  z(0, 0) = hdg;
  z(1, 0) = att;
  z(2, 0) = cam.fx * sw / range;
  z(3, 0) = cam.fy * sh / range;
  if (los_bias) {
    z(0, 0) += x(8, 0);
    z(1, 0) += x(9, 0);
  }
  if (size_bias) {
    z(2, 0) += x(10, 0);
    z(3, 0) += x(11, 0);
  }
  return z;
}

Mat<M, N> H_of(const Vec<N>& x, const Intr& cam, bool los_bias, bool size_bias) {
  const double px = x(0, 0), py = x(1, 0), pz = x(2, 0);
  double range = std::sqrt(px * px + py * py + pz * pz);
  if (range < kMinDepth) range = kMinDepth;
  const double r2 = range * range;
  const double horiz2 = std::max(px * px + py * py, 1e-8 * r2);
  const double horiz = std::sqrt(horiz2);
  const double sw = std::max(kMinSize, x(6, 0));
  const double sh = std::max(kMinSize, x(7, 0));
  const double inv_r3 = 1.0 / (range * r2);

  Mat<M, N> H;
  H(0, 0) = py / horiz2;
  H(0, 1) = -px / horiz2;
  H(1, 0) = -pz * px / (r2 * horiz);
  H(1, 1) = -pz * py / (r2 * horiz);
  H(1, 2) = horiz / r2;
  H(2, 0) = -cam.fx * sw * px * inv_r3;
  H(2, 1) = -cam.fx * sw * py * inv_r3;
  H(2, 2) = -cam.fx * sw * pz * inv_r3;
  H(3, 0) = -cam.fy * sh * px * inv_r3;
  H(3, 1) = -cam.fy * sh * py * inv_r3;
  H(3, 2) = -cam.fy * sh * pz * inv_r3;
  H(2, 6) = cam.fx / range;
  H(3, 7) = cam.fy / range;
  if (los_bias) {
    H(0, 8) = 1.0;
    H(1, 9) = 1.0;
  }
  if (size_bias) {
    H(2, 10) = 1.0;
    H(3, 11) = 1.0;
  }
  return H;
}

void pack(const double x[N], const double P[N][N], Vec<N>& xv, Mat<N, N>& Pv) {
  for (int i = 0; i < N; ++i) {
    xv(i, 0) = x[i];
    for (int j = 0; j < N; ++j) Pv(i, j) = P[i][j];
  }
}

void unpack(const Vec<N>& xv, const Mat<N, N>& Pv, double x[N],
            double P[N][N]) {
  for (int i = 0; i < N; ++i) {
    x[i] = xv(i, 0);
    for (int j = 0; j < N; ++j) P[i][j] = Pv(i, j);
  }
}

void predict_linear(Vec<N>& x, Mat<N, N>& P, double dt, const Vec3& own_vel,
                    const Config& cfg, double fx) {
  if (dt <= 0) return;
  for (int i = 0; i < 3; ++i) {
    x(i, 0) += dt * (x(3 + i, 0) - comp(own_vel, i));
  }
  Mat<N, N> F = identity<N>();
  for (int i = 0; i < 3; ++i) F(i, 3 + i) = dt;
  P = symmetrize<N>(F * P * transpose(F));

  const double sa2 = static_cast<double>(cfg.sigma_accel) * cfg.sigma_accel;
  const double sov = cfg.sigma_own_vel;
  const double q_pp = sa2 * dt * dt * dt / 3.0 + sov * sov * dt * dt;
  const double q_pv = sa2 * dt * dt / 2.0;
  const double q_vv = sa2 * dt;
  for (int i = 0; i < 3; ++i) {
    P(i, i) += q_pp;
    P(i, 3 + i) += q_pv;
    P(3 + i, i) += q_pv;
    P(3 + i, 3 + i) += q_vv;
  }
  const double qs = static_cast<double>(cfg.size_walk) * cfg.size_walk * dt;
  P(6, 6) += qs;
  P(7, 7) += qs;

  if (use_los_bias(cfg)) {
    const double walk = static_cast<double>(cfg.los_bias_walk_deg) * kDegToRad;
    const double q = walk * walk * dt;
    P(8, 8) += q;
    P(9, 9) += q;
  }

  if (use_size_bias(cfg)) {
    const double tau = std::max(0.02, static_cast<double>(cfg.meas_corr_tau_s));
    const double corr =
        std::min(0.98, std::max(0.01, static_cast<double>(cfg.meas_corr)));
    const double rho = std::exp(dt * std::log(corr) / tau);
    const double sig_px =
        std::max(0.2, static_cast<double>(cfg.meas_bias_sigma_px));
    const double sig_ang = sig_px / std::max(1.0, fx);
    const int i0 = use_los_bias(cfg) ? 10 : 8;
    for (int i = i0; i < N; ++i) {
      x(i, 0) *= rho;
      for (int j = 0; j < i0; ++j) {
        P(i, j) *= rho;
        P(j, i) *= rho;
      }
    }
    for (int i = i0; i < N; ++i) {
      for (int j = i0; j < N; ++j) P(i, j) *= rho * rho;
    }
    if (!use_los_bias(cfg)) {
      P(8, 8) += (1.0 - rho * rho) * sig_ang * sig_ang;
      P(9, 9) += (1.0 - rho * rho) * sig_ang * sig_ang;
    }
    P(10, 10) += (1.0 - rho * rho) * sig_px * sig_px;
    P(11, 11) += (1.0 - rho * rho) * sig_px * sig_px;
  }
}

void init_state(const Meas& m, const Config& cfg, Vec<N>& x, Mat<N, N>& P) {
  const Intr cam = intr_of(m.cam);
  const Vec3 dir = los_dir(m.heading_deg - m.heading_bias_deg,
                           m.attack_deg - m.attack_bias_deg);
  const double w_px = std::max(2.0, static_cast<double>(m.width_px));
  const double h_px = std::max(2.0, static_cast<double>(m.height_px));
  const double range =
      std::min(std::max(cam.fx * cfg.size_prior_m / w_px, 1.0), 400.0);

  x = Vec<N>{};
  x(0, 0) = dir.x * range;
  x(1, 0) = dir.y * range;
  x(2, 0) = dir.z * range;
  x(6, 0) = cfg.size_prior_m;
  x(7, 0) = std::max(kMinSize, cfg.size_prior_m * h_px / w_px);

  P = Mat<N, N>{};
  Vec3 e1 = dir.cross(Vec3{0, 0, 1});
  if (e1.length() < 1e-3f) e1 = dir.cross(Vec3{1, 0, 0});
  e1 = e1.normalized();
  const Vec3 e2 = dir.cross(e1).normalized();
  const double sig_perp = range * std::max(0.5f, cfg.sigma_px_center) / cam.fx;
  const double sig_along = 0.5 * range;
  const double vp = sig_perp * sig_perp;
  const double va = sig_along * sig_along;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      P(i, j) = va * comp(dir, i) * comp(dir, j) +
                vp * (comp(e1, i) * comp(e1, j) + comp(e2, i) * comp(e2, j));
    }
  }
  constexpr double kSigmaVel0 = 12.0;
  for (int i = 3; i < 6; ++i) P(i, i) = kSigmaVel0 * kSigmaVel0;
  const double ss = cfg.size_prior_sigma_m;
  P(6, 6) = ss * ss;
  P(7, 7) = ss * ss;
  if (use_los_bias(cfg)) {
    const double sa = static_cast<double>(cfg.los_bias_sigma_deg) * kDegToRad;
    P(8, 8) = sa * sa;
    P(9, 9) = sa * sa;
  } else if (use_size_bias(cfg)) {
    const double sb = std::max(0.2, static_cast<double>(cfg.meas_bias_sigma_px));
    const double sa = sb / std::max(1.0, cam.fx);
    P(8, 8) = sa * sa;
    P(9, 9) = sa * sa;
  }
  if (use_size_bias(cfg)) {
    const double sb = std::max(0.2, static_cast<double>(cfg.meas_bias_sigma_px));
    P(10, 10) = sb * sb;
    P(11, 11) = sb * sb;
  }
}

// 1 = accepted, 0 = gated, -1 = diverged
int ekf_update(Vec<N>& x, Mat<N, N>& P, const Meas& m, const Config& cfg) {
  const Intr cam = intr_of(m.cam);
  const bool ang_b = use_angle_bias(cfg);
  const bool size_b = use_size_bias(cfg);
  const bool los_b = use_los_bias(cfg);
  Vec<M> z;
  z(0, 0) = static_cast<double>(m.heading_deg - m.heading_bias_deg) * kDegToRad;
  z(1, 0) = static_cast<double>(m.attack_deg - m.attack_bias_deg) * kDegToRad;
  z(2, 0) = std::max(2.0, static_cast<double>(m.width_px));
  z(3, 0) = std::max(2.0, static_cast<double>(m.height_px));

  double sc = std::max(0.2f, cfg.sigma_px_center);
  double ss = std::max(0.2f, cfg.sigma_px_size);
  if (size_b) {
    const double leftover = 1.0 - static_cast<double>(cfg.meas_corr);
    ss = std::max(0.4, ss * leftover);
    if (!los_b) sc = std::max(0.4, sc * leftover);
  }
  const double sig_ang = sc / std::max(1.0, cam.fx);
  Mat<M, M> R;
  R(0, 0) = sig_ang * sig_ang;
  R(1, 1) = sig_ang * sig_ang;
  R(2, 2) = ss * ss;
  R(3, 3) = ss * ss;

  const Mat<M, N> H = H_of(x, cam, ang_b, size_b);
  const Vec<M> zhat = h_of(x, cam, ang_b, size_b);
  const Mat<N, M> C = P * transpose(H);
  const Mat<M, M> S = H * C + R;

  Mat<M, M> Sinv;
  if (!inverse<M>(S, Sinv)) return 0;
  Vec<M> r = z - zhat;
  r(0, 0) = wrap_pi(r(0, 0));
  r(1, 0) = wrap_pi(r(1, 0));
  const double d2 = quad_form<M>(r, Sinv);
  if (!std::isfinite(d2) || d2 > cfg.gate_chi2) return 0;

  const Mat<N, M> K = C * Sinv;
  const Vec<N> x_new = x + K * r;
  if (!finite_state(x_new)) return -1;
  x = x_new;
  P = symmetrize<N>(P - K * transpose(C));
  x(6, 0) = std::max(kMinSize, x(6, 0));
  x(7, 0) = std::max(kMinSize, x(7, 0));
  return 1;
}

Estimate pack_estimate(const Vec<N>& x, const Mat<N, N>& P, const Camera& cam) {
  Estimate out;
  const Intr ic = intr_of(cam);
  const Vec<M> z = h_of(x, ic, false, false);
  const double w = std::max(2.0, z(2, 0));
  const double h = std::max(2.0, z(3, 0));
  out.box.u0 = static_cast<float>(ic.cu - 0.5 * w);
  out.box.u1 = static_cast<float>(ic.cu + 0.5 * w);
  out.box.v0 = static_cast<float>(ic.cv - 0.5 * h);
  out.box.v1 = static_cast<float>(ic.cv + 0.5 * h);
  out.pos_rel = Vec3{static_cast<float>(x(0, 0)), static_cast<float>(x(1, 0)),
                     static_cast<float>(x(2, 0))};
  out.vel_world = Vec3{static_cast<float>(x(3, 0)), static_cast<float>(x(4, 0)),
                       static_cast<float>(x(5, 0))};
  out.range_m = out.pos_rel.length();
  out.speed_mps = out.vel_world.length();
  out.size_w_m = static_cast<float>(x(6, 0));
  out.size_h_m = static_cast<float>(x(7, 0));

  const Vec3 dir = out.pos_rel.normalized();
  double var = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) var += comp(dir, i) * P(i, j) * comp(dir, j);
  out.range_sigma_m = static_cast<float>(std::sqrt(std::max(0.0, var)));

  const float horiz = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  out.heading_deg = std::atan2(dir.x, dir.y) * static_cast<float>(kRadToDeg);
  out.attack_deg =
      std::atan2(dir.z, std::max(horiz, 1e-6f)) * static_cast<float>(kRadToDeg);
  out.heading_bias_deg = static_cast<float>(x(8, 0) * kRadToDeg);
  out.attack_bias_deg = static_cast<float>(x(9, 0) * kRadToDeg);
  out.valid = finite_state(x);
  return out;
}

}  // namespace

void Camera::set_zoom(float z, float fov_wide) {
  if (z < 0.25f) z = 0.25f;
  if (z > 16.0f) z = 16.0f;
  zoom = z;
  fov_wide_deg = fov_wide;
  fov_deg = fov_wide / z;
  if (fov_deg < 8.0f) fov_deg = 8.0f;
  if (fov_deg > 120.0f) fov_deg = 120.0f;
  fx = (width * 0.5f) / std::tan(fov_deg * 0.5f * static_cast<float>(kPi) / 180.0f);
  fy = fx;
}

Camera Camera::from_fov(int width, int height, float fov_wide_deg, float zoom) {
  Camera cam;
  cam.width = width;
  cam.height = height;
  cam.set_zoom(zoom, fov_wide_deg);
  return cam;
}

Camera Camera::from_fx(int width, int height, float fx, float fy) {
  Camera cam;
  cam.width = width;
  cam.height = height;
  cam.fx = fx;
  cam.fy = (fy > 0.0f) ? fy : fx;
  cam.fov_deg = 2.0f * std::atan((width * 0.5f) / std::max(1.0f, fx)) *
                180.0f / static_cast<float>(kPi);
  cam.fov_wide_deg = cam.fov_deg;
  cam.zoom = 1.0f;
  return cam;
}

BBoxEkf::BBoxEkf(Config cfg) : cfg_(cfg) { reset(); }

void BBoxEkf::set_config(const Config& cfg) { cfg_ = cfg; }

void BBoxEkf::reset() {
  for (int i = 0; i < N; ++i) {
    x_[i] = 0;
    for (int j = 0; j < N; ++j) P_[i][j] = 0;
  }
  have_ = false;
  filt_valid_ = false;
  t_ = 0;
  updates_ = 0;
  rejects_ = 0;
}

bool BBoxEkf::push(const Meas& m) {
  Vec<N> x;
  Mat<N, N> P;
  pack(x_, P_, x, P);

  if (!have_ || !filt_valid_) {
    init_state(m, cfg_, x, P);
    unpack(x, P, x_, P_);
    have_ = true;
    filt_valid_ = true;
    t_ = m.t;
    ++updates_;
    return true;
  }

  double dt = m.t - t_;
  if (dt < 0.0) {
    ++rejects_;
    return false;
  }
  dt = std::min(dt, 1.0);
  predict_linear(x, P, dt, m.own_vel, cfg_, m.cam.fx);
  const int rc = ekf_update(x, P, m, cfg_);
  if (rc < 0) {
    init_state(m, cfg_, x, P);
    filt_valid_ = true;
  } else if (rc == 0) {
    ++rejects_;
  }
  unpack(x, P, x_, P_);
  t_ = m.t;
  ++updates_;
  return rc != 0;
}

Estimate BBoxEkf::predict(const Camera& cam_query, const Vec3& own_vel,
                          double t_query) const {
  Estimate out;
  if (!have_ || !filt_valid_) return out;
  Vec<N> x;
  Mat<N, N> P;
  pack(x_, P_, x, P);
  if (!finite_state(x)) return out;
  double dt = t_query - t_;
  dt = std::min(std::max(dt, 0.0), 3.0);
  predict_linear(x, P, dt, own_vel, cfg_, cam_query.fx);
  if (!finite_state(x)) return out;
  return pack_estimate(x, P, cam_query);
}

}  // namespace bbox_ekf
