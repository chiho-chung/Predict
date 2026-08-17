#include "estimator/predictor.hpp"

#include <algorithm>
#include <cmath>

void BBoxPredictor::reset() {
  have_box_ = false;
  have_vel_ = false;
  box_ = {};
  t_box_ = 0;
  vel_ = {};
  size_rate_ = {};
}

void BBoxPredictor::push(const BBox& box, float t) {
  if (have_box_) {
    const float dt = t - t_box_;
    if (dt > 1e-4f) {
      const Vec2 c_new = box.center();
      const Vec2 c_old = box_.center();
      const Vec2 raw_vel = (c_new - c_old) * (1.0f / dt);
      const Vec2 raw_size{(box.width() - box_.width()) / dt,
                          (box.height() - box_.height()) / dt};

      // Detector noise gets amplified by 1/dt, so smooth the derivative.
      const float a = std::max(0.0f, std::min(0.95f, smooth_));
      if (have_vel_) {
        vel_ = vel_ * a + raw_vel * (1.0f - a);
        size_rate_ = size_rate_ * a + raw_size * (1.0f - a);
      } else {
        vel_ = raw_vel;
        size_rate_ = raw_size;
        have_vel_ = true;
      }
    }
  }

  box_ = box;
  t_box_ = t;
  have_box_ = true;
}

BBox BBoxPredictor::at(float t_now, float lead_s) const {
  if (!have_box_) return {};

  // Total extrapolation = detector dead time already elapsed + requested lead.
  const float total = age(t_now) + lead_s;

  const Vec2 c = box_.center();
  const float cu = c.x + vel_.x * total;
  const float cv = c.y + vel_.y * total;
  const float w = std::max(2.0f, box_.width() + size_rate_.x * total);
  const float h = std::max(2.0f, box_.height() + size_rate_.y * total);

  BBox out;
  out.u0 = cu - 0.5f * w;
  out.u1 = cu + 0.5f * w;
  out.v0 = cv - 0.5f * h;
  out.v1 = cv + 0.5f * h;
  return out;
}

const char* estimator_name(EstimatorType t) {
  switch (t) {
    case EstimatorType::CvPixel: return "CV-pixel";
    case EstimatorType::Ekf: return "EKF";
    case EstimatorType::Ukf: return "UKF";
    case EstimatorType::ImmEkf: return "IMM-EKF";
    case EstimatorType::ImmUkf: return "IMM-UKF";
    case EstimatorType::EkfLos: return "EKF-LOS";
    case EstimatorType::UkfLos: return "UKF-LOS";
    case EstimatorType::ImmEkfLos: return "IMM-EKF-LOS";
    case EstimatorType::ImmUkfLos: return "IMM-UKF-LOS";
    default: return "?";
  }
}

bool estimator_uses_filter(EstimatorType t) {
  return t != EstimatorType::CvPixel && t != EstimatorType::kCount;
}

bool estimator_uses_bbox(EstimatorType t) {
  return t == EstimatorType::Ekf || t == EstimatorType::Ukf ||
         t == EstimatorType::ImmEkf || t == EstimatorType::ImmUkf;
}

bool estimator_is_imm(EstimatorType t) {
  return t == EstimatorType::ImmEkf || t == EstimatorType::ImmUkf ||
         t == EstimatorType::ImmEkfLos || t == EstimatorType::ImmUkfLos;
}

bool estimator_is_unscented(EstimatorType t) {
  return t == EstimatorType::Ukf || t == EstimatorType::ImmUkf ||
         t == EstimatorType::UkfLos || t == EstimatorType::ImmUkfLos;
}

namespace {

using la::Mat;
using la::Vec;

constexpr int N = TargetFilter::N;
constexpr int M = TargetFilter::M;

// Julier unscented transform. kappa = 3 keeps every weight positive at n = 12,
// which guarantees a PSD covariance; the usual alpha/beta scaling puts a large
// negative weight on the centre point and loses that.
constexpr double kKappa = 3.0;

constexpr double kMinDepth = 0.35;  // stay off the projection singularity
constexpr double kMinSize = 0.03;

struct ProjModel {
  Vec3 right, up, fwd;
  double fx, fy, cu, cv;
};

ProjModel proj_of(const CameraFrame& cam) {
  ProjModel p;
  p.right = cam.right;
  p.up = cam.up;
  p.fwd = cam.forward;
  p.fx = cam.fx;
  p.fy = cam.fy;
  p.cu = cam.width * 0.5;
  p.cv = cam.height * 0.5;
  return p;
}

double axis_dot(const Vec3& a, double x, double y, double z) {
  return a.x * x + a.y * y + a.z * z;
}

double vec_comp(const Vec3& v, int i) {
  return (i == 0) ? v.x : (i == 1) ? v.y : v.z;
}

bool use_meas_bias(const TrackerConfig& cfg) { return cfg.meas_corr >= 0.05f; }

// z = [u, v, width_px, height_px]. Bias is detector error, so it is added only
// when comparing to a measurement, never when reprojecting a world estimate.
Vec<M> h_of(const Vec<N>& x, const ProjModel& pm, bool with_bias) {
  const double px = x(0, 0), py = x(1, 0), pz = x(2, 0);
  double zc = axis_dot(pm.fwd, px, py, pz);
  if (zc < kMinDepth) zc = kMinDepth;
  const double xc = axis_dot(pm.right, px, py, pz);
  const double yc = axis_dot(pm.up, px, py, pz);
  const double sw = std::max(kMinSize, x(6, 0));
  const double sh = std::max(kMinSize, x(7, 0));

  Vec<M> z;
  z(0, 0) = pm.cu + pm.fx * xc / zc;
  z(1, 0) = pm.cv - pm.fy * yc / zc;
  z(2, 0) = pm.fx * sw / zc;
  z(3, 0) = pm.fy * sh / zc;
  if (with_bias) {
    for (int i = 0; i < M; ++i) z(i, 0) += x(8 + i, 0);
  }
  return z;
}

Mat<M, N> H_of(const Vec<N>& x, const ProjModel& pm, bool with_bias) {
  const double px = x(0, 0), py = x(1, 0), pz = x(2, 0);
  double zc = axis_dot(pm.fwd, px, py, pz);
  if (zc < kMinDepth) zc = kMinDepth;
  const double xc = axis_dot(pm.right, px, py, pz);
  const double yc = axis_dot(pm.up, px, py, pz);
  const double sw = std::max(kMinSize, x(6, 0));
  const double sh = std::max(kMinSize, x(7, 0));
  const double inv = 1.0 / zc;

  Mat<M, N> H;
  for (int i = 0; i < 3; ++i) {
    const double a = vec_comp(pm.right, i);
    const double b = vec_comp(pm.up, i);
    const double c = vec_comp(pm.fwd, i);
    H(0, i) = pm.fx * inv * (a - xc * inv * c);
    H(1, i) = -pm.fy * inv * (b - yc * inv * c);
    H(2, i) = -pm.fx * sw * inv * inv * c;
    H(3, i) = -pm.fy * sh * inv * inv * c;
  }
  H(2, 6) = pm.fx * inv;
  H(3, 7) = pm.fy * inv;
  if (with_bias) {
    for (int i = 0; i < M; ++i) H(i, 8 + i) = 1.0;
  }
  return H;
}

Vec<M> meas_of(const BBox& b) {
  Vec<M> z;
  const Vec2 c = b.center();
  z(0, 0) = c.x;
  z(1, 0) = c.y;
  z(2, 0) = b.width();
  z(3, 0) = b.height();
  return z;
}

Mat<M, M> R_of(const TrackerConfig& cfg, bool with_bias) {
  Mat<M, M> R;
  double sc = std::max(0.2f, cfg.sigma_px_center);
  double ss = std::max(0.2f, cfg.sigma_px_size);
  // Colored part lives in the bias states; R is only the white leftover.
  if (with_bias) {
    const double leftover = 1.0 - static_cast<double>(cfg.meas_corr);
    sc = std::max(0.4, sc * leftover);
    ss = std::max(0.4, ss * leftover);
  }
  R(0, 0) = sc * sc;
  R(1, 1) = sc * sc;
  R(2, 2) = ss * ss;
  R(3, 3) = ss * ss;
  return R;
}

bool state_is_finite(const Vec<N>& x) {
  for (int i = 0; i < N; ++i) {
    if (!std::isfinite(x(i, 0))) return false;
  }
  return true;
}

// Relative position integrates (target velocity - own velocity); target
// velocity and extent hold. Linear in the state, so this is exact.
void predict_linear(Vec<N>& x, Mat<N, N>& P, double dt, const Vec3& own_vel,
                    const TrackerConfig& cfg, double sigma_accel) {
  if (dt <= 0) return;

  for (int i = 0; i < 3; ++i) {
    x(i, 0) += dt * (x(3 + i, 0) - vec_comp(own_vel, i));
  }

  Mat<N, N> F = la::identity<N>();
  for (int i = 0; i < 3; ++i) F(i, 3 + i) = dt;
  P = la::symmetrize<N>(F * P * la::transpose(F));

  const double sa2 = sigma_accel * sigma_accel;
  const double sov = cfg.sigma_own_vel;
  // Standard constant-velocity Q, plus own-velocity measurement error, which
  // corrupts relative position directly because it drives the dynamics.
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

  if (use_meas_bias(cfg)) {
    const double tau = std::max(0.02, static_cast<double>(cfg.meas_corr_tau_s));
    const double corr = std::min(0.98, std::max(0.01, static_cast<double>(cfg.meas_corr)));
    const double rho = std::exp(dt * std::log(corr) / tau);
    const double sig = std::max(0.2, static_cast<double>(cfg.meas_bias_sigma_px));
    const double qss = sig * sig;
    for (int i = 8; i < N; ++i) {
      x(i, 0) *= rho;
      for (int j = 0; j < 8; ++j) {
        P(i, j) *= rho;
        P(j, i) *= rho;
      }
    }
    for (int i = 8; i < N; ++i) {
      for (int j = 8; j < N; ++j) P(i, j) *= rho * rho;
      P(i, i) += (1.0 - rho * rho) * qss;
    }
  }
}

double gauss_likelihood(const Vec<M>& r, const Mat<M, M>& S, double d2) {
  Mat<M, M> L;
  if (!la::cholesky<M>(S, L)) return 0.0;
  const double det = la::determinant_from_chol<M>(L);
  if (!(det > 0.0)) return 0.0;
  constexpr double kTwoPiM = 39.478417604357434;  // (2*pi)^2 for M = 4
  (void)r;
  return std::exp(-0.5 * d2) / std::sqrt(kTwoPiM * det);
}

double gauss_likelihood2(const Vec<2>& r, const Mat<2, 2>& S, double d2) {
  Mat<2, 2> L;
  if (!la::cholesky<2>(S, L)) return 0.0;
  const double det = la::determinant_from_chol<2>(L);
  if (!(det > 0.0)) return 0.0;
  constexpr double kTwoPiM = 6.283185307179586;  // (2*pi)^1 for M = 2
  (void)r;
  return std::exp(-0.5 * d2) / std::sqrt(kTwoPiM * det);
}

// Apply the Kalman update in either 4-D (centre + size) or 2-D (LOS / centre).
template <int MD>
double apply_meas_update(Vec<N>& x, Mat<N, N>& P, const Vec<MD>& z,
                         const Vec<MD>& zhat, const Mat<N, MD>& C,
                         const Mat<MD, MD>& S, float gate) {
  Mat<MD, MD> Sinv;
  if (!la::inverse<MD>(S, Sinv)) return 0.0;

  const Vec<MD> r = z - zhat;
  const double d2 = la::quad_form<MD>(r, Sinv);
  if (!std::isfinite(d2) || d2 > gate) return 0.0;

  const Mat<N, MD> K = C * Sinv;
  const Vec<N> x_new = x + K * r;
  if (!state_is_finite(x_new)) {
    return -1.0;  // caller marks invalid
  }
  x = x_new;
  P = la::symmetrize<N>(P - K * la::transpose(C));
  x(6, 0) = std::max(kMinSize, x(6, 0));
  x(7, 0) = std::max(kMinSize, x(7, 0));

  if constexpr (MD == 2) return gauss_likelihood2(r, S, d2);
  else return gauss_likelihood(r, S, d2);
}

}  // namespace

void TargetFilter::init(const TrackerMeas& m, const TrackerConfig& cfg) {
  const ProjModel pm = proj_of(m.cam);
  const Vec2 c = m.box.center();
  const Vec3 dir = unproject_dir(m.cam, c.x, c.y);
  const bool bbox = estimator_uses_bbox(cfg.type);

  // Bbox filters convert angular size + the extent prior into a range.
  // LOS-only filters never read width/height, so range starts from a prior
  // and is tightened later by own-motion parallax.
  const double w_px = std::max(2.0, static_cast<double>(m.box.width()));
  const double h_px = std::max(2.0, static_cast<double>(m.box.height()));
  const double cos_off = std::max(0.2, static_cast<double>(dir.dot(m.cam.forward)));
  double range;
  if (bbox) {
    const double depth = pm.fx * cfg.size_prior_m / w_px;
    range = std::min(std::max(depth / cos_off, 1.0), 400.0);
  } else {
    range = std::min(std::max(static_cast<double>(cfg.los_range_prior_m), 1.0),
                     400.0);
  }

  x = Vec<N>{};
  x(0, 0) = dir.x * range;
  x(1, 0) = dir.y * range;
  x(2, 0) = dir.z * range;
  x(6, 0) = cfg.size_prior_m;
  x(7, 0) = bbox ? std::max(kMinSize, cfg.size_prior_m * h_px / w_px)
                 : cfg.size_prior_m;

  // Position covariance is anisotropic on purpose: pixel noise pins the two
  // directions across the line of sight, while range rests on the extent prior
  // and stays loose until own-motion parallax tightens it.
  P = Mat<N, N>{};
  Vec3 e1 = dir.cross(Vec3{0, 0, 1});
  if (e1.length() < 1e-3f) e1 = dir.cross(Vec3{1, 0, 0});
  e1 = e1.normalized();
  const Vec3 e2 = dir.cross(e1).normalized();

  const double sig_perp = range * std::max(0.5f, cfg.sigma_px_center) / pm.fx;
  // Without angular size, range is much less certain at acquisition.
  const double sig_along = bbox ? 0.5 * range : std::max(6.0, 0.7 * range);
  const double vp = sig_perp * sig_perp;
  const double va = sig_along * sig_along;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      P(i, j) = va * vec_comp(dir, i) * vec_comp(dir, j) +
                vp * (vec_comp(e1, i) * vec_comp(e1, j) +
                      vec_comp(e2, i) * vec_comp(e2, j));
    }
  }

  constexpr double kSigmaVel0 = 12.0;  // target velocity unknown at acquisition
  for (int i = 3; i < 6; ++i) P(i, i) = kSigmaVel0 * kSigmaVel0;
  const double ss = cfg.size_prior_sigma_m;
  P(6, 6) = ss * ss;
  P(7, 7) = ss * ss;

  if (use_meas_bias(cfg)) {
    const double sb = std::max(0.2, static_cast<double>(cfg.meas_bias_sigma_px));
    for (int i = 8; i < N; ++i) {
      x(i, 0) = 0.0;
      P(i, i) = sb * sb;
    }
  }

  valid = true;
}

void TargetFilter::predict(double dt, const Vec3& own_vel,
                           const TrackerConfig& cfg, double sigma_accel) {
  if (!valid) return;
  predict_linear(x, P, dt, own_vel, cfg, sigma_accel);
}

double TargetFilter::update(const TrackerMeas& m, const TrackerConfig& cfg,
                            bool unscented) {
  if (!valid) return 0.0;

  const ProjModel pm = proj_of(m.cam);
  const Vec<M> z = meas_of(m.box);
  // Bias-in-the-measurement helps the EKF. The UKF, given the same states,
  // attributes size residuals to the bias and range gets worse — so it keeps
  // the geometry-only measurement and the original R.
  const bool bias = use_meas_bias(cfg) && !unscented;
  const Mat<M, M> R = R_of(cfg, bias);

  Vec<M> zhat;
  Mat<M, M> S;
  Mat<N, M> C;  // state/measurement cross-covariance

  bool did_unscented = false;
  if (unscented) {
    // UT over the 8 geometry states only. Spreading sigma points through the
    // bias dimensions changes √(n+κ) and was measured to hurt pointing.
    constexpr int NG = 8;
    Mat<NG, NG> Pg;
    for (int i = 0; i < NG; ++i)
      for (int j = 0; j < NG; ++j) Pg(i, j) = P(i, j);
    Mat<NG, NG> L;
    if (la::cholesky<NG>(Pg, L)) {
      const double n_plus = NG + kKappa;
      const double gamma = std::sqrt(n_plus);
      constexpr int kPts = 2 * NG + 1;

      Vec<N> xs[kPts];
      double wts[kPts];
      xs[0] = x;
      wts[0] = kKappa / n_plus;
      for (int j = 0; j < NG; ++j) {
        xs[1 + j] = x;
        xs[1 + NG + j] = x;
        for (int i = 0; i < NG; ++i) {
          const double d = gamma * L(i, j);
          xs[1 + j](i, 0) += d;
          xs[1 + NG + j](i, 0) -= d;
        }
        wts[1 + j] = 1.0 / (2.0 * n_plus);
        wts[1 + NG + j] = wts[1 + j];
      }

      Vec<M> zs[kPts];
      Vec<M> zm{};
      for (int p = 0; p < kPts; ++p) {
        zs[p] = h_of(xs[p], pm, false);
        for (int i = 0; i < M; ++i) zm(i, 0) += wts[p] * zs[p](i, 0);
      }

      Mat<M, M> Sz{};
      Mat<N, M> Cz{};
      for (int p = 0; p < kPts; ++p) {
        double dz[M], dx[NG];
        for (int i = 0; i < M; ++i) dz[i] = zs[p](i, 0) - zm(i, 0);
        for (int i = 0; i < NG; ++i) dx[i] = xs[p](i, 0) - x(i, 0);
        for (int i = 0; i < M; ++i) {
          for (int j = 0; j < M; ++j) Sz(i, j) += wts[p] * dz[i] * dz[j];
          for (int j = 0; j < NG; ++j) Cz(j, i) += wts[p] * dx[j] * dz[i];
        }
      }

      zhat = zm;
      S = Sz + R;
      C = Cz;
      did_unscented = true;
    }
  }

  if (!did_unscented) {
    // Also the fallback when P will not factorise.
    const Mat<M, N> H = H_of(x, pm, bias);
    zhat = h_of(x, pm, bias);
    C = P * la::transpose(H);
    S = H * C + R;
  }

  const bool bbox = estimator_uses_bbox(cfg.type);
  const float gate = bbox ? cfg.gate_chi2 : std::min(cfg.gate_chi2, 25.0f);
  double lik = 0.0;
  if (bbox) {
    lik = apply_meas_update<M>(x, P, z, zhat, C, S, gate);
  } else {
    // Same capture, but only the box centre / LOS. Width and height are
    // ignored so this is a fair A/B against the bbox filters.
    Vec<2> z2, zhat2;
    Mat<2, 2> S2;
    Mat<N, 2> C2;
    for (int i = 0; i < 2; ++i) {
      z2(i, 0) = z(i, 0);
      zhat2(i, 0) = zhat(i, 0);
      for (int j = 0; j < 2; ++j) S2(i, j) = S(i, j);
      for (int j = 0; j < N; ++j) C2(j, i) = C(j, i);
    }
    lik = apply_meas_update<2>(x, P, z2, zhat2, C2, S2, gate);
  }
  if (lik < 0.0) {
    valid = false;
    return 0.0;
  }
  return lik;
}

void Tracker::reset() {
  px_.reset();
  for (int i = 0; i < kMaxModels; ++i) {
    models_[i] = TargetFilter{};
    weight_[i] = 0.0;
    model_prob_[i] = 0.0;
  }
  n_models_ = estimator_is_imm(cfg_.type) ? kMaxModels : 1;
  have_track_ = false;
  t_filter_ = 0;
  updates_ = 0;
  rejects_ = 0;
  last_box_ = BBox{};
}

void Tracker::set_config(const TrackerConfig& cfg) {
  const int want_models = estimator_is_imm(cfg.type) ? kMaxModels : 1;
  const bool restart = (want_models != n_models_) ||
                       (estimator_uses_filter(cfg.type) !=
                        estimator_uses_filter(cfg_.type)) ||
                       (estimator_uses_bbox(cfg.type) !=
                        estimator_uses_bbox(cfg_.type));
  cfg_ = cfg;
  px_.set_smoothing(cfg_.vel_smooth);
  if (restart) {
    reset();
  } else {
    n_models_ = want_models;
  }
}

bool Tracker::ready() const {
  if (!have_track_) return false;
  if (!estimator_uses_filter(cfg_.type)) return px_.ready();
  for (int i = 0; i < n_models_; ++i) {
    if (models_[i].valid) return true;
  }
  return false;
}

float Tracker::age(float t_now) const {
  return have_track_ ? (t_now - t_filter_) : 0.0f;
}

void Tracker::push(const TrackerMeas& m) {
  // dt is stamp-to-stamp, not 1/detect_hz. That is what undoes irregular
  // frame intervals and pipeline delay once the query uses t_now - t_stamp.
  px_.push(m.box, m.t);  // baseline stays fed so it can be switched to freely
  last_box_ = m.box;

  if (!estimator_uses_filter(cfg_.type)) {
    have_track_ = true;
    t_filter_ = m.t;
    ++updates_;
    return;
  }

  if (!have_track_) {
    for (int i = 0; i < n_models_; ++i) {
      models_[i].init(m, cfg_);
      weight_[i] = 1.0 / n_models_;
      model_prob_[i] = static_cast<float>(weight_[i]);
    }
    have_track_ = true;
    t_filter_ = m.t;
    ++updates_;
    return;
  }

  double dt = static_cast<double>(m.t) - static_cast<double>(t_filter_);
  if (dt < 0.0) return;             // out-of-sequence, drop it
  dt = std::min(dt, 1.0);           // after a long dropout, don't coast forever

  const bool unscented = estimator_is_unscented(cfg_.type);

  if (n_models_ == 1) {
    models_[0].predict(dt, m.own_vel, cfg_, cfg_.sigma_accel);
    const double lik = models_[0].update(m, cfg_, unscented);
    if (lik <= 0.0) ++rejects_;
    if (!models_[0].valid) {
      models_[0].init(m, cfg_);  // diverged; re-acquire
    }
    weight_[0] = 1.0;
    model_prob_[0] = 1.0f;
    t_filter_ = m.t;
    ++updates_;
    return;
  }

  // IMM: mix, then predict and update each hypothesis, then reweight by
  // measurement likelihood.
  const int n = n_models_;
  const double stay = std::min(0.995, std::max(0.5, (double)cfg_.imm_stay_prob));
  const double leave = (1.0 - stay) / (n - 1);

  double cbar[kMaxModels]{};
  for (int j = 0; j < n; ++j) {
    double s = 0;
    for (int i = 0; i < n; ++i) {
      s += ((i == j) ? stay : leave) * weight_[i];
    }
    cbar[j] = s;
  }

  TargetFilter mixed[kMaxModels];
  for (int j = 0; j < n; ++j) {
    Vec<N> xm{};
    double mu[kMaxModels]{};
    for (int i = 0; i < n; ++i) {
      mu[i] = (((i == j) ? stay : leave) * weight_[i]) /
              std::max(1e-12, cbar[j]);
      for (int k = 0; k < N; ++k) xm(k, 0) += mu[i] * models_[i].x(k, 0);
    }
    Mat<N, N> Pm{};
    for (int i = 0; i < n; ++i) {
      Vec<N> d;
      for (int k = 0; k < N; ++k) d(k, 0) = models_[i].x(k, 0) - xm(k, 0);
      const Mat<N, N> spread = d * la::transpose(d);
      for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
          Pm(r, c) += mu[i] * (models_[i].P(r, c) + spread(r, c));
        }
      }
    }
    mixed[j].x = xm;
    mixed[j].P = la::symmetrize<N>(Pm);
    mixed[j].valid = true;
  }
  for (int j = 0; j < n; ++j) models_[j] = mixed[j];

  double lik[kMaxModels]{};
  int gated = 0;
  for (int j = 0; j < n; ++j) {
    models_[j].predict(dt, m.own_vel, cfg_, cfg_.imm_sigma_accel[j]);
    lik[j] = models_[j].update(m, cfg_, unscented);
    if (lik[j] <= 0.0) ++gated;
  }
  if (gated == n) ++rejects_;

  double sum = 0;
  for (int j = 0; j < n; ++j) {
    weight_[j] = cbar[j] * std::max(lik[j], 1e-30);
    sum += weight_[j];
  }
  if (!(sum > 0.0)) {
    for (int j = 0; j < n; ++j) weight_[j] = 1.0 / n;
  } else {
    for (int j = 0; j < n; ++j) weight_[j] /= sum;
  }
  for (int j = 0; j < n; ++j) {
    model_prob_[j] = static_cast<float>(weight_[j]);
  }

  t_filter_ = m.t;
  ++updates_;
}

void Tracker::mix_states(Vec<N>& x_out, Mat<N, N>& P_out) const {
  x_out = Vec<N>{};
  P_out = Mat<N, N>{};

  double wsum = 0;
  for (int i = 0; i < n_models_; ++i) {
    if (!models_[i].valid) continue;
    wsum += weight_[i];
  }
  if (!(wsum > 0.0)) {
    x_out = models_[0].x;
    P_out = models_[0].P;
    return;
  }

  for (int i = 0; i < n_models_; ++i) {
    if (!models_[i].valid) continue;
    const double w = weight_[i] / wsum;
    for (int k = 0; k < N; ++k) x_out(k, 0) += w * models_[i].x(k, 0);
  }
  for (int i = 0; i < n_models_; ++i) {
    if (!models_[i].valid) continue;
    const double w = weight_[i] / wsum;
    Vec<N> d;
    for (int k = 0; k < N; ++k) d(k, 0) = models_[i].x(k, 0) - x_out(k, 0);
    const Mat<N, N> spread = d * la::transpose(d);
    for (int r = 0; r < N; ++r) {
      for (int c = 0; c < N; ++c) {
        P_out(r, c) += w * (models_[i].P(r, c) + spread(r, c));
      }
    }
  }
}

TrackEstimate Tracker::at(const CameraFrame& cam_query, const Vec3& own_vel,
                          float t_now, float lead_s) const {
  TrackEstimate out;
  if (!have_track_) return out;

  if (!estimator_uses_filter(cfg_.type)) {
    if (!px_.ready()) return out;
    out.box = px_.at(t_now, lead_s);
    out.valid = true;
    return out;
  }

  Vec<N> x;
  Mat<N, N> P;
  mix_states(x, P);
  if (!state_is_finite(x)) return out;

  // Coast from the last frame TIMESTAMP to t_now (delay removed) or to
  // t_now+H (future LOS). The interval is whatever the stamps say.
  double dt = static_cast<double>(t_now) + lead_s - static_cast<double>(t_filter_);
  dt = std::min(std::max(dt, 0.0), 3.0);
  // Only the covariance depends on which sigma is used here; the mean is
  // identical for every model because the dynamics are linear and shared.
  predict_linear(x, P, dt, own_vel, cfg_, cfg_.sigma_accel);
  if (!state_is_finite(x)) return out;

  const ProjModel pm = proj_of(cam_query);
  const Vec<M> z = h_of(x, pm, false);
  const double w = std::max(2.0, z(2, 0));
  const double h = std::max(2.0, z(3, 0));
  out.box.u0 = static_cast<float>(z(0, 0) - 0.5 * w);
  out.box.u1 = static_cast<float>(z(0, 0) + 0.5 * w);
  out.box.v0 = static_cast<float>(z(1, 0) - 0.5 * h);
  out.box.v1 = static_cast<float>(z(1, 0) + 0.5 * h);

  out.pos_rel = Vec3{static_cast<float>(x(0, 0)), static_cast<float>(x(1, 0)),
                     static_cast<float>(x(2, 0))};
  out.vel_world = Vec3{static_cast<float>(x(3, 0)), static_cast<float>(x(4, 0)),
                       static_cast<float>(x(5, 0))};
  out.range_m = out.pos_rel.length();
  out.speed_mps = out.vel_world.length();
  out.size_w_m = static_cast<float>(x(6, 0));
  out.size_h_m = static_cast<float>(x(7, 0));

  // Range uncertainty is the position covariance projected on the line of sight.
  const Vec3 dir = out.pos_rel.normalized();
  double var = 0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      var += vec_comp(dir, i) * P(i, j) * vec_comp(dir, j);
    }
  }
  out.range_sigma_m = static_cast<float>(std::sqrt(std::max(0.0, var)));

  out.model_count = n_models_;
  for (int i = 0; i < n_models_ && i < 3; ++i) out.model_prob[i] = model_prob_[i];

  out.valid = true;
  return out;
}
