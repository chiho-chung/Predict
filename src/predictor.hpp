#pragma once

#include "drone_model.hpp"
#include "linalg.hpp"
#include "vec.hpp"

// Image-plane bounding box predictor.
//
// Constant velocity on box center and size, in pixels. Kept as the baseline to
// A/B the world-frame filters against: it needs no camera pose and no own
// velocity, but it cannot estimate range or size and it has no notion of the
// geometry that actually moves the box.
class BBoxPredictor {
 public:
  void reset();
  void set_smoothing(float ema) { smooth_ = ema; }

  // New measurement from the detector at time t.
  void push(const BBox& box, float t);

  bool ready() const { return have_box_; }
  bool has_velocity() const { return have_vel_; }

  // Seconds since the last measurement.
  float age(float t_now) const { return have_box_ ? (t_now - t_box_) : 0.0f; }

  // Extrapolated box at t_now + lead_s.
  BBox at(float t_now, float lead_s) const;

  Vec2 velocity_px() const { return vel_; }
  Vec2 size_rate_px() const { return size_rate_; }

 private:
  bool have_box_ = false;
  bool have_vel_ = false;
  BBox box_{};
  float t_box_ = 0;
  Vec2 vel_{};        // center px/s
  Vec2 size_rate_{};  // width,height px/s
  float smooth_ = 0.5f;
};

const char* estimator_name(EstimatorType t);
bool estimator_uses_filter(EstimatorType t);
bool estimator_uses_bbox(EstimatorType t);
bool estimator_is_imm(EstimatorType t);
bool estimator_is_unscented(EstimatorType t);

// A detection, tagged with everything needed to interpret it: the camera pose it
// was captured with, own velocity, and the frame TIMESTAMP. Predictors never
// assume a fixed 1/Hz; they predict from the last stamp to this stamp, then
// from this stamp to t_now (delay removed) or t_now+H (future LOS).
struct TrackerMeas {
  BBox box;
  CameraFrame cam;
  Vec3 own_vel{};
  float t = 0;          // reported capture timestamp (what the filter uses)
  float t_true = 0;     // true capture time (sim / logs only)
  float latency_s = 0;  // true pipeline delay of this packet
};

// One manoeuvre hypothesis.
//
// State (12): [ p_rel(3) | v_target(3) | width_m | height_m | bias_px(4) ]
//
// Relative position, because own velocity is measurable but own position is
// not. Own velocity therefore enters as a known input and the dynamics stay
// linear; the camera projection is the only nonlinear part, so EKF and UKF
// differ solely in how the update is done.
//
// The last four states are a Gauss-Markov model of the detector's EMA jitter
// (u, v, width, height in pixels). Pulling that correlation into the state
// leaves a white innovation, which is what R, the gate and the IMM likelihood
// all assume. They are measurement bias, not geometry: reprojection ignores
// them.
class TargetFilter {
 public:
  static constexpr int N = 12;
  static constexpr int M = 4;

  void init(const TrackerMeas& m, const TrackerConfig& cfg);
  void predict(double dt, const Vec3& own_vel, const TrackerConfig& cfg,
               double sigma_accel);

  // Returns the Gaussian likelihood of the measurement, or 0 when gated out.
  double update(const TrackerMeas& m, const TrackerConfig& cfg,
                bool unscented);

  la::Vec<N> x{};
  la::Mat<N, N> P{};
  bool valid = false;
};

// Facade over the baseline predictor, bbox EKF/UKF/IMM, and LOS-only twins.
class Tracker {
 public:
  static constexpr int kMaxModels = 3;

  void reset();
  void set_config(const TrackerConfig& cfg);
  const TrackerConfig& config() const { return cfg_; }

  void push(const TrackerMeas& m);

  bool ready() const;
  float age(float t_now) const;
  int update_count() const { return updates_; }
  int reject_count() const { return rejects_; }

  // Estimate at t_now + lead_s, reprojected through cam_query. own_vel is
  // needed because the state is relative to a camera that keeps moving.
  TrackEstimate at(const CameraFrame& cam_query, const Vec3& own_vel,
                   float t_now, float lead_s) const;

  Vec2 velocity_px() const { return px_.velocity_px(); }
  const float* model_probs() const { return model_prob_; }
  int model_count() const { return n_models_; }

 private:
  void mix_states(la::Vec<TargetFilter::N>& x_out,
                  la::Mat<TargetFilter::N, TargetFilter::N>& P_out) const;

  TrackerConfig cfg_{};
  BBoxPredictor px_;  // always fed, so the baseline stays available

  TargetFilter models_[kMaxModels];
  double weight_[kMaxModels]{};
  float model_prob_[kMaxModels]{};
  int n_models_ = 1;

  bool have_track_ = false;
  float t_filter_ = 0;
  int updates_ = 0;
  int rejects_ = 0;
  BBox last_box_{};
};
