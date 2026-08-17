#pragma once

#include "sim/drone_model.hpp"
#include "estimator/predictor.hpp"
#include "sim/types.hpp"

#include <cstdint>
#include <vector>

class Simulation {
 public:
  explicit Simulation(SimConfig cfg = {});

  void reset();
  void step(float dt);  // call at cfg.rates.sim_hz

  void set_target_velocity(const Vec3& v);
  void nudge_target(const Vec3& delta);
  void set_predict_horizon(float seconds);
  void toggle_chase();
  void toggle_predict();
  void toggle_jitter();
  void adjust_jitter_center(float delta_px);
  void adjust_jitter_size(float delta_frac);
  void adjust_detect_hz(float delta_hz);
  void adjust_detect_latency(float delta_s);
  void toggle_timing_jitter();
  void adjust_zoom(float delta);
  void set_zoom(float zoom);
  void toggle_ideal_gimbal();
  void cycle_estimator(int delta);
  void set_estimator(EstimatorType t);
  void cycle_target_maneuver();

  const SimSnapshot& snapshot() const { return snap_; }
  const SimConfig& config() const { return cfg_; }

  // Last detector packet handed to the tracker this step (auto-bench tape).
  bool meas_delivered_this_step() const { return meas_delivered_; }
  const TrackerMeas& last_push() const { return last_push_; }

 private:
  void update_chase(float dt);
  void update_target(float dt);
  void update_zoom();
  void update_gimbal(float dt);
  void point_gimbal_at_target();
  void capture_detection(const Detection& truth);
  void deliver_detections();
  float next_frame_period();
  float sample_latency();
  float sample_stamp(float t_true);
  void update_los(const Detection& truth, const Detection& est,
                  const Detection& pred, const TrackEstimate& est_t,
                  const TrackEstimate& pred_t);
  Detection project_target(const Vec3& target_pos) const;
  Detection apply_bbox_jitter(const Detection& clean);
  Vec3 predict_target_world(float horizon_s) const;
  Vec3 camera_position() const;
  Vec3 predicted_own_disp(float horizon_s, const TrackEstimate& est) const;
  CameraFrame current_camera() const;

  SimConfig cfg_;
  DroneState drone_;
  TargetState target_;
  GimbalState gimbal_;
  float time_ = 0;
  SimSnapshot snap_{};

  // A detection captured at t becomes available at t + detect_latency_s. It
  // carries the camera pose and own velocity from the capture instant, which is
  // what lets the estimator undo the latency instead of just smoothing it.
  struct PendingDetection {
    TrackerMeas meas;
    Detection det;
    BBox clean_box;
    bool clean_valid = false;
    float deliver_at = 0;
  };

  // Detector timing (slow loop)
  Tracker tracker_;
  std::vector<PendingDetection> pending_;
  float next_capture_at_ = 0;
  float last_period_s_ = 0.1f;
  float last_latency_s_ = 0;
  float last_stamp_err_s_ = 0;
  float last_meas_time_ = 0;  // true capture time of the newest delivered frame
  int capture_count_ = 0;
  int detect_count_ = 0;
  Detection last_meas_{};
  TrackerMeas last_push_{};
  bool meas_delivered_ = false;

  // Camera pose and clean box as of the last detector tick. A stale pixel must
  // be back-projected with the pose that produced it, not the current one.
  CameraFrame meas_cam_{};
  BBox meas_clean_box_{};
  bool meas_clean_valid_ = false;

  // Gimbal servo state. The aim error is one step old, which is what a real
  // control loop sees.
  Vec2 aim_err_px_{};
  bool have_aim_ = false;
  float gimbal_rate_dps_ = 0;

  // Correlated jitter state, advanced once per detector tick
  float jit_du_ = 0;
  float jit_dv_ = 0;
  float jit_dw_ = 0;
  float jit_dh_ = 0;
  uint32_t rng_ = 0xC0FFEEu;

  static constexpr int kLosHist = 400;
  LosAngles origin_hist_[kLosHist]{};
  LosAngles pred_hist_[kLosHist]{};
  int los_hist_i_ = 0;
  int los_hist_n_ = 0;

  // Camera position at the last detection stamp (for IMU-accurate own_disp).
  Vec3 filter_cam_pos_{};
  bool have_filter_cam_pos_ = false;
};
