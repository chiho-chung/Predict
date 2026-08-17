#include "sim/sim.hpp"
#include "sim/drone_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {

float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

float rand_uniform(uint32_t& state) {
  // xorshift32
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return (state & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
}

float rand_signed(uint32_t& state) { return rand_uniform(state) * 2.0f - 1.0f; }

}  // namespace

Simulation::Simulation(SimConfig cfg) : cfg_(std::move(cfg)) { reset(); }

void Simulation::reset() {
  drone_ = DroneState{};
  drone_.pos = cfg_.spawn.drone_pos;
  target_ = TargetState{};
  target_.pos = cfg_.spawn.target_pos;
  target_.vel = cfg_.spawn.target_vel;
  target_.yaw = std::atan2(target_.vel.x, target_.vel.y);
  rng_ = cfg_.spawn.rng_seed;
  gimbal_ = GimbalState{};
  time_ = 0;
  if (cfg_.camera.fov_wide_deg < 8.0f) cfg_.camera.fov_wide_deg = 70.0f;
  cfg_.camera.set_zoom(cfg_.zoom.auto_cycle ? cfg_.zoom.min_zoom
                                            : cfg_.camera.zoom);
  // Hand off an already-locked track, otherwise the servo has no measurement to
  // acquire from.
  point_gimbal_at_target();
  aim_err_px_ = {};
  have_aim_ = false;
  gimbal_rate_dps_ = 0;

  tracker_.set_config(cfg_.tracker);
  tracker_.reset();
  pending_.clear();
  next_capture_at_ = 0;
  last_period_s_ = cfg_.detect_dt();
  last_latency_s_ = cfg_.rates.detect_latency_s;
  last_stamp_err_s_ = 0;
  last_meas_time_ = 0;
  capture_count_ = 0;
  detect_count_ = 0;
  last_meas_ = Detection{};
  last_push_ = TrackerMeas{};
  meas_delivered_ = false;
  meas_clean_valid_ = false;
  meas_cam_ = CameraFrame{};
  meas_clean_box_ = BBox{};

  jit_du_ = jit_dv_ = 0;
  jit_dw_ = jit_dh_ = 0;
  step(0.0f);
}

void Simulation::set_target_velocity(const Vec3& v) { target_.vel = v; }

void Simulation::nudge_target(const Vec3& delta) { target_.pos += delta; }

void Simulation::set_predict_horizon(float seconds) {
  cfg_.predict.horizon_s = clampf(seconds, 0.0f, 3.0f);
}

void Simulation::toggle_chase() { cfg_.chase_enabled = !cfg_.chase_enabled; }

void Simulation::toggle_predict() { cfg_.predict.enabled = !cfg_.predict.enabled; }

void Simulation::toggle_jitter() { cfg_.jitter.enabled = !cfg_.jitter.enabled; }

void Simulation::adjust_jitter_center(float delta_px) {
  cfg_.jitter.center_px = clampf(cfg_.jitter.center_px + delta_px, 0.0f, 40.0f);
}

void Simulation::adjust_jitter_size(float delta_frac) {
  cfg_.jitter.size_frac = clampf(cfg_.jitter.size_frac + delta_frac, 0.0f, 0.50f);
}

void Simulation::adjust_detect_hz(float delta_hz) {
  cfg_.rates.detect_hz = clampf(cfg_.rates.detect_hz + delta_hz, 1.0f, 100.0f);
}

void Simulation::adjust_detect_latency(float delta_s) {
  cfg_.rates.detect_latency_s =
      clampf(cfg_.rates.detect_latency_s + delta_s, 0.0f, 0.5f);
}

void Simulation::toggle_timing_jitter() {
  cfg_.timing.enabled = !cfg_.timing.enabled;
}

void Simulation::set_zoom(float zoom) { cfg_.camera.set_zoom(zoom); }

void Simulation::adjust_zoom(float delta) {
  set_zoom(cfg_.camera.zoom + delta);
}

void Simulation::update_zoom() {
  if (!cfg_.zoom.auto_cycle) return;
  const float per = std::max(1.0f, cfg_.zoom.period_s);
  constexpr float kPi = 3.14159265358979323846f;
  const float s = 0.5f + 0.5f * std::sin(2.0f * kPi * time_ / per);
  const float z =
      cfg_.zoom.min_zoom + (cfg_.zoom.max_zoom - cfg_.zoom.min_zoom) * s;
  cfg_.camera.set_zoom(z);
}

void Simulation::toggle_ideal_gimbal() {
  cfg_.gimbal.ideal = !cfg_.gimbal.ideal;
}

void Simulation::cycle_target_maneuver() {
  const int n = static_cast<int>(TargetManeuver::kCount);
  const int idx = (static_cast<int>(cfg_.target.maneuver) + 1) % n;
  cfg_.target.maneuver = static_cast<TargetManeuver>(idx);
}

void Simulation::cycle_estimator(int delta) {
  const int n = static_cast<int>(EstimatorType::kCount);
  int idx = static_cast<int>(cfg_.tracker.type) + delta;
  idx = ((idx % n) + n) % n;
  set_estimator(static_cast<EstimatorType>(idx));
}

void Simulation::set_estimator(EstimatorType t) {
  cfg_.tracker.type = t;
  tracker_.set_config(cfg_.tracker);
}

Vec3 Simulation::camera_position() const {
  // Gimbal camera under the chaser body (plate bottom)
  return drone_.pos + Vec3{0, 0, -0.04f};
}

void Simulation::update_chase(float dt) {
  if (!cfg_.chase_enabled || dt <= 0) return;

  const float lead = cfg_.predict.enabled ? cfg_.predict.horizon_s * 0.5f : 0.0f;
  const Vec3 aim = target_.pos + target_.vel * lead;
  const Vec3 standoff = (drone_.pos - aim).normalized() * 8.0f;
  const Vec3 want_pos = aim + Vec3{standoff.x, standoff.y, 0} + Vec3{0, 0, 3.0f};
  const Vec3 to_aim = want_pos - drone_.pos;
  const Vec3 desired_dir = to_aim.normalized();

  const float dist = (target_.pos - drone_.pos).length();
  float speed_scale = 1.0f;
  if (dist < 6.0f) speed_scale = dist / 6.0f;
  if (dist > 25.0f) speed_scale = 1.25f;

  const Vec3 desired_vel = desired_dir * (drone_.max_speed * speed_scale);
  Vec3 accel = (desired_vel - drone_.vel);
  const float a_len = accel.length();
  if (a_len > drone_.accel) {
    accel = accel * (drone_.accel / a_len);
  }
  drone_.vel += accel * dt;

  const float spd = drone_.vel.length();
  if (spd > drone_.max_speed) {
    drone_.vel = drone_.vel * (drone_.max_speed / spd);
  }

  drone_.pos += drone_.vel * dt;
  drone_.pos.z = clampf(drone_.pos.z, 3.0f, 40.0f);

  if (drone_.vel.length() > 0.2f) {
    drone_.yaw = std::atan2(drone_.vel.x, drone_.vel.y);
  }
}

void Simulation::update_target(float dt) {
  if (dt <= 0) return;

  float turn = 0.35f * std::sin(time_ * 0.4f);
  if (cfg_.target.maneuver == TargetManeuver::Jink) {
    // Hard alternating turns: ~7 m/s^2 lateral, switching sign every 1.5 s.
    constexpr float kPeriod = 3.0f;
    const float phase = std::fmod(time_, kPeriod);
    turn = (phase < kPeriod * 0.5f) ? 1.2f : -1.2f;
  }
  const float c = std::cos(turn * dt);
  const float s = std::sin(turn * dt);
  const float vx = target_.vel.x * c - target_.vel.y * s;
  const float vy = target_.vel.x * s + target_.vel.y * c;
  target_.vel.x = vx;
  target_.vel.y = vy;

  Vec2 h{target_.vel.x, target_.vel.y};
  const float hs = h.length();
  if (hs > 1e-3f) {
    const float want = 6.0f;
    target_.vel.x = h.x / hs * want;
    target_.vel.y = h.y / hs * want;
  }

  target_.pos += target_.vel * dt;
  target_.pos.z = 6.0f;
  if (target_.vel.length() > 0.2f) {
    target_.yaw = std::atan2(target_.vel.x, target_.vel.y);
  }

  constexpr float bound = 60.0f;
  if (target_.pos.x > bound || target_.pos.x < -bound) target_.vel.x *= -1.0f;
  if (target_.pos.y > bound || target_.pos.y < -bound) target_.vel.y *= -1.0f;
  target_.pos.x = clampf(target_.pos.x, -bound, bound);
  target_.pos.y = clampf(target_.pos.y, -bound, bound);
}

void Simulation::point_gimbal_at_target() {
  const Vec3 cam = camera_position();
  const Vec3 to = target_.pos - cam;
  const float horiz = std::sqrt(to.x * to.x + to.y * to.y);
  gimbal_.yaw = std::atan2(to.x, to.y);
  gimbal_.pitch =
      clampf(std::atan2(to.z, std::max(horiz, 1e-4f)), -1.2f, 0.6f);
}

void Simulation::update_gimbal(float dt) {
  if (cfg_.gimbal.ideal) {
    point_gimbal_at_target();
    gimbal_rate_dps_ = 0;
    return;
  }
  if (dt <= 0 || !have_aim_) return;

  // Pixel error from the tracker -> angular error -> rate command.
  constexpr float kPi = 3.14159265358979323846f;
  const float fx =
      (cfg_.camera.width * 0.5f) / std::tan(cfg_.camera.fov_deg * 0.5f * kPi / 180.0f);
  const float yaw_err = std::atan(aim_err_px_.x / fx);
  const float pitch_err = std::atan(-aim_err_px_.y / fx);

  const float max_rate = cfg_.gimbal.max_rate_dps * kPi / 180.0f;
  float yaw_rate = clampf(cfg_.gimbal.kp * yaw_err, -max_rate, max_rate);
  float pitch_rate = clampf(cfg_.gimbal.kp * pitch_err, -max_rate, max_rate);

  // Yaw axis is compressed near steep pitch angles.
  const float cp = std::max(0.3f, std::cos(gimbal_.pitch));
  gimbal_.yaw += (yaw_rate / cp) * dt;
  gimbal_.pitch = clampf(gimbal_.pitch + pitch_rate * dt, -1.2f, 0.6f);

  gimbal_rate_dps_ =
      std::sqrt(yaw_rate * yaw_rate + pitch_rate * pitch_rate) * 180.0f / kPi;
}

CameraFrame Simulation::current_camera() const {
  return make_camera_frame(camera_position(), gimbal_.yaw, gimbal_.pitch,
                           cfg_.camera);
}

Detection Simulation::project_target(const Vec3& target_pos) const {
  const CameraFrame cam = current_camera();
  DroneModel::Pose pose;
  pose.pos = target_pos;
  pose.yaw = target_.yaw;
  return fit_bbox_from_model(cam, pose);
}

Detection Simulation::apply_bbox_jitter(const Detection& clean) {
  Detection d = clean;
  if (!d.visible || !cfg_.jitter.enabled) return d;

  const float a = clampf(cfg_.jitter.smooth, 0.0f, 0.99f);
  const float center_amp = cfg_.jitter.center_px;
  const float size_amp = cfg_.jitter.size_frac;  // fraction of current box

  const float n_du = rand_signed(rng_) * center_amp;
  const float n_dv = rand_signed(rng_) * center_amp;
  const float n_fw = rand_signed(rng_) * size_amp;
  const float n_fh = rand_signed(rng_) * size_amp;

  jit_du_ = a * jit_du_ + (1.0f - a) * n_du;
  jit_dv_ = a * jit_dv_ + (1.0f - a) * n_dv;
  jit_dw_ = a * jit_dw_ + (1.0f - a) * n_fw;
  jit_dh_ = a * jit_dh_ + (1.0f - a) * n_fh;

  const Vec2 c = clean.box.center();
  const float fw = clampf(jit_dw_, -0.90f, 0.90f);
  const float fh = clampf(jit_dh_, -0.90f, 0.90f);
  const float w = std::max(2.0f, clean.box.width() * (1.0f + fw));
  const float h = std::max(2.0f, clean.box.height() * (1.0f + fh));
  const float cu = c.x + jit_du_;
  const float cv = c.y + jit_dv_;

  d.box.u0 = cu - 0.5f * w;
  d.box.u1 = cu + 0.5f * w;
  d.box.v0 = cv - 0.5f * h;
  d.box.v1 = cv + 0.5f * h;
  d.depth = clean.depth;
  d.visible = true;
  return d;
}

float Simulation::next_frame_period() {
  const float nom = cfg_.detect_dt();
  if (!cfg_.timing.enabled || cfg_.timing.period_jitter_frac <= 0.0f) {
    return nom;
  }
  const float j = clampf(cfg_.timing.period_jitter_frac, 0.0f, 0.9f);
  return clampf(nom * (1.0f + j * rand_signed(rng_)), nom * 0.4f, nom * 2.5f);
}

float Simulation::sample_latency() {
  float lat = cfg_.rates.detect_latency_s;
  if (cfg_.timing.enabled && cfg_.timing.latency_jitter_s > 0.0f) {
    lat += cfg_.timing.latency_jitter_s * rand_signed(rng_);
  }
  return std::max(0.0f, lat);
}

float Simulation::sample_stamp(float t_true) {
  if (!cfg_.timing.enabled || cfg_.timing.stamp_jitter_s <= 0.0f) {
    return t_true;
  }
  return t_true + cfg_.timing.stamp_jitter_s * rand_signed(rng_);
}

void Simulation::capture_detection(const Detection& truth) {
  const Detection meas = apply_bbox_jitter(truth);
  if (!meas.visible) {
    tracker_.reset();
    pending_.clear();
    last_meas_ = Detection{};
    meas_clean_valid_ = false;
    jit_du_ = jit_dv_ = 0;
    jit_dw_ = jit_dh_ = 0;
    return;
  }

  const float lat = sample_latency();
  const float stamp = sample_stamp(time_);

  PendingDetection p;
  p.meas.box = meas.box;
  p.meas.cam = current_camera();
  p.meas.own_vel = drone_.vel;  // own velocity in the nav frame, as measured
  p.meas.t = stamp;             // reported timestamp — what every predictor uses
  p.meas.t_true = time_;
  p.meas.latency_s = lat;
  p.det = meas;
  p.clean_box = truth.box;
  p.clean_valid = truth.visible;
  p.deliver_at = time_ + lat;
  pending_.push_back(p);

  last_latency_s_ = lat;
  last_stamp_err_s_ = stamp - time_;
}

void Simulation::deliver_detections() {
  // Latency jitter can reorder packets. Hand ready frames to the tracker in
  // timestamp order so a late-but-older stamp is not applied after a newer one.
  std::vector<size_t> ready;
  for (size_t i = 0; i < pending_.size(); ++i) {
    if (pending_[i].deliver_at <= time_ + 1e-6f) ready.push_back(i);
  }
  if (ready.empty()) return;

  std::sort(ready.begin(), ready.end(), [&](size_t a, size_t b) {
    return pending_[a].meas.t < pending_[b].meas.t;
  });

  for (size_t idx : ready) {
    PendingDetection& p = pending_[idx];
    tracker_.push(p.meas);
    last_push_ = p.meas;
    meas_delivered_ = true;
    last_meas_ = p.det;
    last_meas_time_ = p.meas.t_true;
    meas_cam_ = p.meas.cam;
    meas_clean_box_ = p.clean_box;
    meas_clean_valid_ = p.clean_valid;
    ++detect_count_;
  }

  std::vector<char> drop(pending_.size(), 0);
  for (size_t idx : ready) drop[idx] = 1;
  std::vector<PendingDetection> keep;
  keep.reserve(pending_.size() - ready.size());
  for (size_t i = 0; i < pending_.size(); ++i) {
    if (!drop[i]) keep.push_back(pending_[i]);
  }
  pending_.swap(keep);
}

void Simulation::update_los(const Detection& truth, const Detection& est,
                            const Detection& pred) {
  snap_.los = LosSet{};
  const CameraFrame cam_now = current_camera();

  // All five paths start from a box center so they differ only by delay,
  // jitter and prediction rather than by how the angle is derived.
  if (truth.visible) {
    snap_.los.origin = los_from_box(cam_now, truth.box);
  }
  if (meas_clean_valid_) {
    snap_.los.delayed = los_from_box(meas_cam_, meas_clean_box_);
  }
  if (last_meas_.visible) {
    snap_.los.jittered = los_from_box(meas_cam_, last_meas_.box);
  }
  // Predictor output describes the current image, so it uses the current pose.
  // For the +H trace the future pose is unknown; the current one is used.
  if (est.visible) {
    snap_.los.estimate = los_from_box(cam_now, est.box);
  }
  if (pred.visible) {
    snap_.los.predicted = los_from_box(cam_now, pred.box);
  }
}

Vec3 Simulation::predict_target_world(float horizon_s) const {
  return target_.pos + target_.vel * horizon_s;
}

void Simulation::step(float dt) {
  meas_delivered_ = false;
  update_zoom();
  update_target(dt);
  update_chase(dt);
  update_gimbal(dt);
  time_ += dt;

  // Truth every step, used for display and error metrics only.
  const Detection truth = project_target(target_.pos);

  // Slow loop: capture on an irregular schedule (nominal detect_hz ± jitter).
  // Each frame is stamped; the estimator uses that stamp, not 1/Hz.
  if (capture_count_ == 0 || time_ + 1e-6f >= next_capture_at_) {
    capture_detection(truth);
    last_period_s_ = next_frame_period();
    next_capture_at_ = time_ + last_period_s_;
    ++capture_count_;
  }
  deliver_detections();

  // Fast loop: the estimator is queried every step. Because it carries the
  // state from the capture timestamp, this removes both the detector dead time
  // and the pipeline latency, and can look ahead by the horizon.
  const CameraFrame cam_now = current_camera();
  TrackEstimate est_t{};
  TrackEstimate pred_t{};
  if (tracker_.ready() && cfg_.predict.enabled) {
    est_t = tracker_.at(cam_now, drone_.vel, time_, 0.0f);
    pred_t = tracker_.at(cam_now, drone_.vel, time_, cfg_.predict.horizon_s);
  }

  Detection est{};
  Detection pred{};
  if (est_t.valid) {
    est.box = est_t.box;
    est.visible = true;
  } else if (last_meas_.visible) {
    est = last_meas_;  // prediction off, or no track yet: hold the measurement
  }
  if (pred_t.valid) {
    pred.box = pred_t.box;
    pred.visible = true;
  } else if (last_meas_.visible) {
    pred = last_meas_;
  }

  update_los(truth, est, pred);

  snap_.drone = drone_;
  snap_.target = target_;
  snap_.gimbal = gimbal_;
  snap_.detection = last_meas_;
  snap_.detection_gt = truth;
  snap_.estimate_now = est;
  snap_.predicted = pred;
  snap_.predicted_world = predict_target_world(cfg_.predict.horizon_s);
  snap_.detection_age = tracker_.ready() ? (time_ - last_meas_time_) : 0.0f;
  snap_.detect_count = detect_count_;
  snap_.box_vel_px = tracker_.velocity_px();
  snap_.gimbal_rate_dps = gimbal_rate_dps_;

  snap_.track_now = est_t;
  snap_.track_pred = pred_t;
  snap_.true_range_m = (target_.pos - camera_position()).length();
  snap_.range_err_m =
      est_t.valid ? std::fabs(est_t.range_m - snap_.true_range_m) : 0.0f;
  // The filter's extent state is "metres of target that span the box", which
  // varies with viewing aspect, so compare against that same quantity rather
  // than the 30 cm envelope.
  if (est_t.valid && truth.visible && truth.depth > 0.1f) {
    const float true_w_m = truth.box.width() * truth.depth / cam_now.fx;
    snap_.size_err_m = std::fabs(est_t.size_w_m - true_w_m);
  } else {
    snap_.size_err_m = 0;
  }
  snap_.tracker_updates = tracker_.update_count();
  snap_.tracker_rejects = tracker_.reject_count();
  snap_.time = time_;
  snap_.last_period_s = last_period_s_;
  snap_.last_latency_s = last_latency_s_;
  snap_.meas_stamp_s = last_push_.t;
  snap_.stamp_err_s = last_stamp_err_s_;
  snap_.zoom = cfg_.camera.zoom;
  snap_.fov_deg = cfg_.camera.fov_deg;
  snap_.fx = cam_now.fx;

  const Vec2 image_center{cfg_.camera.width * 0.5f, cfg_.camera.height * 0.5f};
  if (truth.visible && last_meas_.visible) {
    const Vec2 t = truth.box.center();
    snap_.held_err_px = (last_meas_.box.center() - t).length();
    snap_.est_err_px = est.visible ? (est.box.center() - t).length() : 0.0f;
    snap_.track_err_px = (t - image_center).length();
  } else {
    snap_.held_err_px = 0;
    snap_.est_err_px = 0;
    snap_.track_err_px = 0;
  }

  // Feed the gimbal servo for the next step. Using the predictor output here is
  // what lets a good predictor keep the target centered despite the 10 Hz
  // detector; with prediction off it can only chase the stale measurement.
  const Detection& aim = (cfg_.predict.enabled && est.visible) ? est : last_meas_;
  if (aim.visible) {
    aim_err_px_ = aim.box.center() - image_center;
    have_aim_ = true;
  } else {
    have_aim_ = false;
    gimbal_rate_dps_ = 0;
  }
}
