#pragma once

#include <cmath>

// Standalone bbox IMM-EKF for another project.
//
// Copy this folder (bbox_imm_ekf.hpp + bbox_imm_ekf.cpp). No other Predict
// files needed. C++17, no third-party deps.
//
// Same inputs, measurement model, and bias knobs as export/bbox_ekf:
//   - LOS: heading_deg / attack_deg (world)
//   - box size: width_px, height_px
//   - camera INTRINSICS at capture (fx / FOV / zoom) — not gimbal attitude
//   - own velocity, frame TIMESTAMP
//
// Three manoeuvre hypotheses (quiet / moderate / hard accel) share the same
// 13-state EKF; mixing uses the Gaussian likelihood of the box measurement.

namespace bbox_imm_ekf {

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

struct Camera {
  float fx = 1;
  float fy = 1;
  int width = 640;
  int height = 480;
  float fov_wide_deg = 70;
  float zoom = 1;
  float fov_deg = 70;

  static Camera from_fov(int width, int height, float fov_wide_deg,
                         float zoom = 1.0f);
  static Camera from_fx(int width, int height, float fx, float fy = 0.0f);
  void set_zoom(float zoom, float fov_wide_deg);
};

struct Config {
  float sigma_los_deg = 0.70f;
  float sigma_size_frac = 0.10f;
  float sigma_own_vel = 0.15f;
  float size_prior_m = 0.36f;
  float size_prior_sigma_m = 0.12f;
  float size_walk = 0.01f;
  float gate_chi2 = 80.0f;
  // Used only when coasting the mixed estimate in predict().
  float sigma_accel = 8.0f;
  float meas_corr = 0.6f;
  float meas_corr_tau_s = 0.10f;
  float meas_bias_sigma_frac = 0.10f;
  float los_bias_sigma_deg = 0.0f;
  float los_bias_walk_deg = 0.02f;
  float range_bias_sigma_m = 0.0f;
  float range_bias_walk_m = 0.0f;
  // Quiet / moderate / hard manoeuvre bank (m/s^2). Same as the sim IMM.
  float imm_sigma_accel[3] = {3.0f, 8.0f, 20.0f};
  float imm_stay_prob = 0.93f;
};

struct Meas {
  float heading_deg = 0;
  float attack_deg = 0;
  float heading_bias_deg = 0;
  float attack_bias_deg = 0;
  float range_bias_m = 0;
  float width_px = 0;
  float height_px = 0;
  Camera cam;
  Vec3 own_vel{};
  double t = 0;
};

struct Estimate {
  bool valid = false;
  BBox box;
  Vec3 pos_rel{};
  Vec3 vel_world{};
  float range_m = 0;
  float range_sigma_m = 0;
  float size_w_m = 0;
  float size_h_m = 0;
  float speed_mps = 0;
  float heading_deg = 0;
  float attack_deg = 0;
  float heading_bias_deg = 0;
  float attack_bias_deg = 0;
  float range_bias_m = 0;
  int model_count = 3;
  float model_prob[3]{};
};

class BBoxImmEkf {
 public:
  static constexpr int N = 13;
  static constexpr int kModels = 3;

  explicit BBoxImmEkf(Config cfg = {});

  void set_config(const Config& cfg);
  const Config& config() const { return cfg_; }

  void reset();
  bool push(const Meas& m);

  bool ready() const { return have_ && filt_valid_; }
  double last_stamp() const { return t_; }
  int update_count() const { return updates_; }
  int reject_count() const { return rejects_; }
  const float* model_probs() const { return model_prob_; }
  int model_count() const { return kModels; }

  Estimate predict(const Camera& cam_query, const Vec3& own_vel,
                   double t_query) const;

 private:
  Config cfg_{};
  double x_[kModels][N]{};
  double P_[kModels][N][N]{};
  bool valid_[kModels]{};
  double weight_[kModels]{};
  float model_prob_[kModels]{};
  bool have_ = false;
  bool filt_valid_ = false;
  double t_ = 0;
  int updates_ = 0;
  int rejects_ = 0;
  double last_range_bias_known_ = 0;
};

}  // namespace bbox_imm_ekf
