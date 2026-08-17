#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

struct Vec3 {
  float x = 0, y = 0, z = 0;

  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3& operator+=(const Vec3& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  float length() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const {
    const float len = length();
    if (len < 1e-6f) return {0, 0, 0};
    return (*this) * (1.0f / len);
  }
  Vec3 cross(const Vec3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

struct Vec2 {
  float x = 0, y = 0;
  Vec2() = default;
  Vec2(float x_, float y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }
  float length() const { return std::sqrt(x * x + y * y); }
};

struct BBox {
  float u0 = 0, v0 = 0, u1 = 0, v1 = 0;  // pixel coords, inclusive-ish

  float width() const { return u1 - u0; }
  float height() const { return v1 - v0; }
  Vec2 center() const { return {(u0 + u1) * 0.5f, (v0 + v1) * 0.5f}; }
};

struct CameraConfig {
  int width = 640;
  int height = 480;
  float fov_wide_deg = 70.0f;  // FOV at zoom = 1
  float zoom = 1.0f;           // 1 = wide, 2 = half FOV, fx doubles
  float min_zoom = 1.0f;
  float max_zoom = 4.0f;
  float fov_deg = 70.0f;       // current horizontal FOV (wide / zoom)
  float near_z = 0.5f;
  float far_z = 200.0f;

  void set_zoom(float z) {
    if (z < min_zoom) z = min_zoom;
    if (z > max_zoom) z = max_zoom;
    zoom = z;
    fov_deg = fov_wide_deg / std::max(zoom, 0.1f);
    if (fov_deg < 8.0f) fov_deg = 8.0f;
    if (fov_deg > 120.0f) fov_deg = 120.0f;
  }
};

// Optional auto zoom for the bench: a deterministic sine so every estimator
// sees the same fx(t).
struct ZoomConfig {
  bool auto_cycle = false;
  float min_zoom = 1.0f;
  float max_zoom = 3.0f;
  float period_s = 8.0f;
};

struct GimbalState {
  float yaw = 0;    // rad, world yaw of look direction
  float pitch = 0;  // rad, negative = look down
};

struct DroneState {
  Vec3 pos{0, -25, 12};
  Vec3 vel{0, 0, 0};
  float yaw = 0;  // body heading
  float max_speed = 18.0f;
  float accel = 25.0f;
};

struct TargetState {
  Vec3 pos{8.0f, 8.0f, 6.0f};  // fly high enough to be chaseable
  Vec3 vel{4.0f, 0, 0};
  float yaw = 0;  // body heading (same mesh as chaser)
};

// A single well-tuned filter beats an IMM on a smoothly turning target. The
// jinking target is what an IMM is actually for: abrupt switches between quiet
// flight and hard turns, which no single process noise can cover.
enum class TargetManeuver { Smooth = 0, Jink, kCount };

struct TargetConfig {
  TargetManeuver maneuver = TargetManeuver::Smooth;
};

struct Detection {
  BBox box;
  bool visible = false;
  float depth = 0;  // camera-forward distance
};

struct BBoxJitterConfig {
  bool enabled = true;
  float center_px = 4.0f;  // +/- pixels on bbox centre (u,v)
  float size_px = 4.0f;    // +/- pixels on width and height (not a percent)
  float smooth = 0.85f;    // 0=white noise, ~0.85=slow jitter
};

struct PredictConfig {
  float horizon_s = 0.5f;   // how far past "now" to predict
  float vel_smooth = 0.5f;  // EMA on measured px velocity (0=raw, 1=frozen)
  bool enabled = true;
};

// Detector runs slow (camera + bbox), predictor runs fast (control loop).
struct RatesConfig {
  float sim_hz = 100.0f;         // physics + predictor
  float detect_hz = 10.0f;       // nominal camera rate; real interval jitters
  float detect_latency_s = 0.05f;  // mean capture -> available
};

// Real cameras are not a metronome. Each frame still carries a timestamp, and
// every predictor uses that stamp (not 1/Hz) to undo delay and to look ahead.
struct TimingConfig {
  bool enabled = true;
  // Inter-frame period = (1/Hz) * (1 + U[-1,1] * period_jitter_frac), clipped.
  float period_jitter_frac = 0.20f;
  // Pipeline delay = latency + U[-1,1] * latency_jitter_s, floored at 0.
  float latency_jitter_s = 0.010f;
  // Reported stamp = true capture + U[-1,1] * stamp_jitter_s (clock noise).
  float stamp_jitter_s = 0.002f;
};

enum class EstimatorType {
  CvPixel = 0,  // image-space constant velocity (baseline)
  ExportEkf,    // export BBoxEkf: world LOS + box size (default)
  ExportImmEkf, // export BBoxImmEkf: 3-hypothesis IMM of that EKF
  Ekf,          // origin pose-based EKF (comparison)
  Ukf,
  ImmEkf,       // origin IMM-EKF (comparison)
  ImmUkf,
  EkfLos,       // same data, LOS / box-centre only — no width/height
  UkfLos,
  ImmEkfLos,
  ImmUkfLos,
  kCount
};

struct TrackerConfig {
  EstimatorType type = EstimatorType::ExportEkf;

  // Detector noise, 1-sigma. This describes what the detector does, which a
  // designer can characterise; it is not read from the simulator's truth.
  float sigma_px_center = 4.0f;  // 1-sigma on centre, pixels (typical 3–5)
  float sigma_px_size = 4.0f;    // 1-sigma on width/height, pixels — not a %

  // Own velocity is measured (NED), so its error drives relative position.
  float sigma_own_vel = 0.15f;

  // Angular size gives width/range, never width alone, so range and extent are
  // coupled. Own-motion parallax separates them, but only while manoeuvring;
  // this prior is what keeps range usable in between.
  //
  // The prior is on the extent that SPANS THE BOX, not on the 30 cm envelope: a
  // square quad seen from a random yaw spans 0.30*(|cos|+|sin|), which averages
  // ~0.38 m. Centring the prior on 0.30 biases range low by the same fraction.
  float size_prior_m = 0.36f;
  float size_prior_sigma_m = 0.12f;
  float size_walk = 0.01f;  // m/sqrt(s) random walk on the extent states

  float gate_chi2 = 80.0f;  // reject detections beyond this Mahalanobis^2
  float vel_smooth = 0.5f;  // CvPixel baseline only

  // Process noise, m/s^2. 8.0 is well above the target's real ~2-7 m/s^2: at a
  // 10 Hz detector the extra responsiveness beats the extra variance, and the
  // sweep in tools/rate_check.cpp is flat from 8 to 12.
  float sigma_accel = 8.0f;
  // IMM bank spanning quiet / moderate / hard manoeuvring.
  float imm_sigma_accel[3] = {3.0f, 8.0f, 20.0f};
  float imm_stay_prob = 0.93f;

  // Detector jitter is an EMA, not white. The filter models that as a
  // first-order Gauss-Markov bias on the four pixel measurements so the
  // leftover innovation is white — which is what R and the gate assume.
  // 0 disables the bias states. 0.6 matches the simulator's default EMA.
  float meas_corr = 0.6f;
  float meas_corr_tau_s = 0.10f;     // interval the coefficient is quoted over
  float meas_bias_sigma_px = 2.0f;   // steady-state bias 1-sigma, pixels

  // LOS-only filters cannot read angular size, so range starts from this prior
  // and is tightened only by own-motion parallax.
  float los_range_prior_m = 12.0f;
};

struct SpawnConfig {
  Vec3 drone_pos{0, -12, 10};
  Vec3 target_pos{6.0f, 6.0f, 6.0f};
  Vec3 target_vel{5.0f, 2.0f, 0.0f};
  uint32_t rng_seed = 0xC0FFEEu;
};

// Filter output, reprojected into a requested camera.
struct TrackEstimate {
  bool valid = false;
  BBox box;
  Vec3 pos_rel{};    // target minus camera, world axes
  Vec3 vel_world{};  // target velocity, world axes
  float range_m = 0;
  float range_sigma_m = 0;
  float size_w_m = 0;
  float size_h_m = 0;
  float speed_mps = 0;
  int model_count = 0;
  float model_prob[3]{};
};

// The gimbal is a rate-limited servo steered by the tracker's pixel error, not
// by the target's true position. Without this the box would sit pinned to the
// image center and there would be nothing for a predictor to extrapolate.
struct GimbalConfig {
  bool ideal = false;          // true: snap straight to truth every step
  float max_rate_dps = 70.0f;  // slew limit, deg/s
  float kp = 3.0f;             // commanded rate = kp * angular error
};

struct SimConfig {
  CameraConfig camera;
  ZoomConfig zoom;
  PredictConfig predict;
  BBoxJitterConfig jitter;
  RatesConfig rates;
  TimingConfig timing;
  GimbalConfig gimbal;
  TrackerConfig tracker;
  TargetConfig target;
  SpawnConfig spawn;
  bool chase_enabled = true;

  float sim_dt() const { return 1.0f / rates.sim_hz; }
  float detect_dt() const { return 1.0f / rates.detect_hz; }
};

// Line-of-sight direction expressed as the two angles a guidance loop uses.
struct LosAngles {
  float heading_deg = 0;  // azimuth of LOS in the world frame
  float attack_deg = 0;   // elevation of LOS, negative = looking down
  bool valid = false;
};

// The same LOS derived through five different signal paths, so the cost of
// detector delay and jitter is directly visible.
struct LosSet {
  LosAngles origin;     // true box, current camera pose
  LosAngles delayed;    // clean box, held at detector rate
  LosAngles jittered;   // measured (jittered) box, held at detector rate
  LosAngles estimate;   // predictor at now, delay removed
  LosAngles predicted;  // predictor at now + horizon
};

struct SimSnapshot {
  DroneState drone;
  TargetState target;
  GimbalState gimbal;

  Detection detection;     // last measured bbox, held between detector ticks
  Detection detection_gt;  // true mesh-fit bbox at current sim time
  Detection estimate_now;  // predictor at t=now (detector latency removed)
  Detection predicted;     // predictor at t=now+horizon
  Vec3 predicted_world;    // target world pos after horizon (true const-vel)

  float detection_age = 0;   // s since last detector tick
  int detect_count = 0;
  Vec2 box_vel_px{};         // predictor image-plane velocity estimate
  float held_err_px = 0;     // stale measured box vs truth
  float est_err_px = 0;      // delay-removed estimate vs truth
  float track_err_px = 0;    // truth box vs image center (gimbal pointing error)
  float gimbal_rate_dps = 0; // current slew rate
  LosSet los;

  TrackEstimate track_now;   // filter at t=now
  TrackEstimate track_pred;  // filter at t=now+horizon
  float true_range_m = 0;
  float range_err_m = 0;     // |estimated range - true range|
  float size_err_m = 0;      // |estimated width - true width|
  int tracker_updates = 0;
  int tracker_rejects = 0;
  float time = 0;

  // Last delivered frame timing (true capture, reported stamp, this packet).
  float last_period_s = 0;
  float last_latency_s = 0;
  float meas_stamp_s = 0;
  float stamp_err_s = 0;
  float zoom = 1;
  float fov_deg = 70;
  float fx = 1;
};
