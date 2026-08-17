#pragma once

#include <cmath>

// Standalone bbox EKF for another project.
//
// Copy this folder (bbox_ekf.hpp + bbox_ekf.cpp). No other Predict files needed.
// C++17, no third-party deps.
//
// What you feed it each detection:
//   - LOS: heading_deg (azimuth) and attack_deg (elevation / pitch)
//   - box size: width_px, height_px
//   - camera INTRINSICS at CAPTURE (fx / FOV / zoom / image size)
//     — not gimbal yaw/pitch. Attitude is already in the LOS.
//   - own velocity in the world / NED frame at capture
//   - frame TIMESTAMP (seconds). Do not assume a fixed 1/Hz.
//
// What it infers:
//   - relative target position, target velocity, range, target extent
//   - delay-removed LOS at t_now, or future LOS at t_now + H
//   - slow heading/attack bias and range bias (if enabled), removed from
//     the reported LOS / range
//   - fractional detector size bias (EMA jitter as ±% of box)
//
// State: [p_rel(3) | v_target(3) | width_m | height_m | los_bias(2) |
//         size_bias_frac(2) | range_bias]
// Dynamics are linear (own vel is a known input). Measurement is
// [heading, attack, width_px, height_px]. Angular size uses
//   width_px = fx * width_m / (range + range_bias) * (1 + size_bias_frac)
// so a first-catch box error can sit in range_bias instead of p_rel,
// and percent jitter is estimated rather than treated as a fixed pixel offset.

namespace bbox_ekf {

struct Vec3 {
  float x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
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

struct BBox {
  float u0 = 0, v0 = 0, u1 = 0, v1 = 0;
  float width() const { return u1 - u0; }
  float height() const { return v1 - v0; }
  float cu() const { return 0.5f * (u0 + u1); }
  float cv() const { return 0.5f * (v0 + v1); }
};

// Intrinsics only — used to turn box size (px) into range.
//   range ≈ fx * size_m / width_px
// Do not pass gimbal yaw/pitch. World LOS is heading_deg / attack_deg.
struct Camera {
  float fx = 1;
  float fy = 1;
  int width = 640;
  int height = 480;
  float fov_wide_deg = 70;  // FOV at zoom = 1
  float zoom = 1;           // 1 = wide, 2 = half FOV (fx doubles)
  float fov_deg = 70;       // current FOV = wide / zoom

  // Image size + horizontal FOV at zoom 1 + this frame's zoom.
  static Camera from_fov(int width, int height, float fov_wide_deg,
                         float zoom = 1.0f);

  // Already-calibrated focal length (pixels). fy = 0 means fy = fx.
  static Camera from_fx(int width, int height, float fx, float fy = 0.0f);

  // Change zoom on an existing camera (updates fov / fx / fy).
  void set_zoom(float zoom, float fov_wide_deg);
};

struct Config {
  // LOS is already world heading/attack — there is no bbox-centre pixel
  // measurement. ~0.70 deg is 4 px at fx ≈ 343 (sim 640x480, 70° FOV).
  float sigma_los_deg = 0.70f;
  float sigma_size_frac = 0.10f;  // 1-sigma on width/height, fraction of box
  float sigma_own_vel = 0.15f;   // own-velocity measurement 1-sigma, m/s
  float size_prior_m = 0.36f;    // metres of target that span the box
  float size_prior_sigma_m = 0.12f;
  float size_walk = 0.01f;       // m/sqrt(s) random walk on extent
  float gate_chi2 = 80.0f;       // reject if Mahalanobis^2 exceeds this
  float sigma_accel = 8.0f;      // process noise, m/s^2
  // Detector jitter is an EMA on box size as a fraction. 0 disables the
  // size-bias states (raw R on width/height).
  float meas_corr = 0.6f;
  float meas_corr_tau_s = 0.10f;
  float meas_bias_sigma_frac = 0.10f;  // GM size-bias 1-sigma, fraction
  // Slow heading/attack bias (gimbal / sensor offset). Estimated and removed
  // from the reported LOS. Set sigma to 0 if you do not want it estimated.
  float los_bias_sigma_deg = 0.0f;  // initial 1-sigma; 0 = do not estimate
  float los_bias_walk_deg = 0.02f;  // deg/sqrt(s); 0 = constant bias
  // Additive range bias (metres). Same pattern as LOS: estimate is off by
  // default — size and range are coupled on a quiet track. A first-catch box
  // error often looks like a constant offset:
  //   width_px = fx * size / (range + bias)
  // Set sigma to ~1 to estimate. Known offset: Meas::range_bias_m.
  float range_bias_sigma_m = 0.0f;  // initial 1-sigma; 0 = do not estimate
  float range_bias_walk_m = 0.0f;   // m/sqrt(s); 0 = freeze as constant
};

struct Meas {
  // LOS in the world / NED frame (same convention as Estimate).
  float heading_deg = 0;  // azimuth
  float attack_deg = 0;   // elevation / pitch, negative = look down
  // Known/calib LOS offset (deg), if you have it. Subtracted before the update.
  // Leave 0 if unknown — the filter estimates a residual when
  // Config::los_bias_sigma_deg > 0.
  float heading_bias_deg = 0;
  float attack_bias_deg = 0;
  // Known/calib range offset (metres). Angular size is interpreted as
  //   width_px = fx * size / (range + range_bias_m [+ estimated])
  // Leave 0 if unknown — the filter estimates a residual when
  // Config::range_bias_sigma_m > 0.
  float range_bias_m = 0;
  // Box size in pixels (angular size). Not corners.
  float width_px = 0;
  float height_px = 0;

  Camera cam;           // intrinsics + zoom at capture (not attitude)
  Vec3 own_vel{};       // world / NED, at capture
  double t = 0;         // frame timestamp (seconds). Required.
};

struct Estimate {
  bool valid = false;
  BBox box;             // predicted size, placed at image centre
  Vec3 pos_rel{};       // target minus camera, world axes
  Vec3 vel_world{};     // target velocity, world axes
  float range_m = 0;
  float range_sigma_m = 0;
  float size_w_m = 0;
  float size_h_m = 0;
  float speed_mps = 0;
  float heading_deg = 0;  // LOS azimuth, bias removed (from p_rel)
  float attack_deg = 0;   // LOS elevation, bias removed
  float heading_bias_deg = 0;  // estimated residual heading bias
  float attack_bias_deg = 0;   // estimated residual attack bias
  float range_bias_m = 0;      // estimated residual range bias (metres)
};

class BBoxEkf {
 public:
  static constexpr int N = 13;

  explicit BBoxEkf(Config cfg = {});

  void set_config(const Config& cfg);
  const Config& config() const { return cfg_; }

  void reset();

  // New detection. Predicts from the last stamp to m.t, then updates.
  // Returns false if the packet is out-of-sequence or gated out.
  bool push(const Meas& m);

  bool ready() const { return have_ && filt_valid_; }
  double last_stamp() const { return t_; }
  int update_count() const { return updates_; }
  int reject_count() const { return rejects_; }

  // Coast from the last stamp to t_query. cam_query is only for fx / zoom
  // (pixel size of the predicted box). LOS comes from p_rel, not attitude.
  //   delay removed:  t_query = t_now
  //   future LOS:     t_query = t_now + horizon
  // own_vel is the velocity at t_query (the camera keeps moving).
  Estimate predict(const Camera& cam_query, const Vec3& own_vel,
                   double t_query) const;

 private:
  Config cfg_{};
  double x_[N]{};
  double P_[N][N]{};
  bool have_ = false;
  bool filt_valid_ = false;
  double t_ = 0;
  int updates_ = 0;
  int rejects_ = 0;
  double last_range_bias_known_ = 0;
};

}  // namespace bbox_ekf
