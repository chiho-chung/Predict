#include "bbox_ekf.hpp"
#include "sim.hpp"

#include <cmath>
#include <cstdio>

// Headless: feed the export EKF only world LOS + box size + fx (no yaw/pitch).
// Same detections as the sim EKF (ideal gimbal). Score delay-removed LOS/range.

namespace {

struct Acc {
  double sum_sq = 0;
  double sum = 0;
  int n = 0;
  void add(double v) {
    if (!std::isfinite(v)) return;
    sum_sq += v * v;
    sum += v;
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
  double mean() const { return n ? sum / n : 0.0; }
};

double wrap180(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

SimConfig base() {
  SimConfig cfg;
  cfg.camera.width = 480;
  cfg.camera.height = 360;
  cfg.camera.fov_wide_deg = 70.0f;
  cfg.camera.set_zoom(1.0f);
  cfg.rates.sim_hz = 100.0f;
  cfg.rates.detect_hz = 10.0f;
  cfg.rates.detect_latency_s = 0.05f;
  cfg.predict.enabled = true;
  cfg.predict.horizon_s = 0.5f;
  cfg.jitter.enabled = true;
  cfg.jitter.center_px = 4.0f;
  cfg.jitter.size_px = 4.0f;
  cfg.jitter.smooth = 0.6f;
  cfg.timing.enabled = true;
  cfg.gimbal.ideal = true;
  cfg.chase_enabled = true;
  cfg.tracker.type = EstimatorType::Ekf;
  return cfg;
}

bbox_ekf::Meas to_export(const TrackerMeas& m) {
  const LosAngles los = los_from_box(m.cam, m.box);
  bbox_ekf::Meas out;
  out.heading_deg = los.heading_deg;
  out.attack_deg = los.attack_deg;
  out.width_px = m.box.width();
  out.height_px = m.box.height();
  out.cam = bbox_ekf::Camera::from_fx(m.cam.width, m.cam.height, m.cam.fx,
                                      m.cam.fy);
  out.own_vel = {m.own_vel.x, m.own_vel.y, m.own_vel.z};
  out.t = m.t;
  return out;
}

void run(const char* tag, TargetManeuver man, bool zoom_cycle, int steps) {
  SimConfig cfg = base();
  cfg.target.maneuver = man;
  cfg.zoom.auto_cycle = zoom_cycle;
  cfg.zoom.min_zoom = 1.0f;
  cfg.zoom.max_zoom = 3.0f;
  cfg.zoom.period_s = 8.0f;

  Simulation sim(cfg);
  bbox_ekf::BBoxEkf ekf;

  Acc exp_h, exp_a, exp_r, exp_s, sim_h, sim_a, sim_r, sim_s;
  int pushes = 0, rejects = 0;

  for (int i = 0; i < steps; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      if (!ekf.push(to_export(sim.last_push()))) ++rejects;
      ++pushes;
    }

    const auto& s = sim.snapshot();
    if (s.time < 2.0f || !s.los.origin.valid) continue;

    const bbox_ekf::Camera cam_q = bbox_ekf::Camera::from_fx(
        cfg.camera.width, cfg.camera.height, s.fx, s.fx);
    const auto now =
        ekf.predict(cam_q, {s.drone.vel.x, s.drone.vel.y, s.drone.vel.z},
                    s.time);
    const float true_w_m =
        (s.detection_gt.visible && s.detection_gt.depth > 0.1f)
            ? s.detection_gt.box.width() * s.detection_gt.depth / s.fx
            : 0.0f;
    if (now.valid) {
      exp_h.add(wrap180(now.heading_deg - s.los.origin.heading_deg));
      exp_a.add(wrap180(now.attack_deg - s.los.origin.attack_deg));
      exp_r.add(now.range_m - s.true_range_m);
      if (true_w_m > 0.01f) exp_s.add(now.size_w_m - true_w_m);
    }
    if (s.los.estimate.valid) {
      sim_h.add(wrap180(s.los.estimate.heading_deg - s.los.origin.heading_deg));
      sim_a.add(wrap180(s.los.estimate.attack_deg - s.los.origin.attack_deg));
    }
    if (s.track_now.valid && s.track_now.range_m > 0.1f) {
      sim_r.add(s.range_err_m);
      if (true_w_m > 0.01f) sim_s.add(s.track_now.size_w_m - true_w_m);
    }
  }

  std::printf(
      "%-10s  export  hdg=%.3f  att=%.3f  rng=%.2f  size=%.3f  push=%d  rej=%d\n",
      tag, exp_h.rms(), exp_a.rms(), exp_r.rms(), exp_s.rms(), pushes, rejects);
  std::printf("%-10s  simEKF  hdg=%.3f  att=%.3f  rng=%.2f  size=%.3f  rej=%d\n",
              tag, sim_h.rms(), sim_a.rms(), sim_r.rms(), sim_s.rms(),
              sim.snapshot().tracker_rejects);
}

}  // namespace

void run_bias(const char* tag, float inj_h, float inj_a, bool pass_known,
              float est_sigma_deg) {
  SimConfig cfg = base();
  Simulation sim(cfg);
  bbox_ekf::Config ekf_cfg;
  ekf_cfg.los_bias_sigma_deg = est_sigma_deg;
  bbox_ekf::BBoxEkf ekf(ekf_cfg);

  Acc exp_h, exp_a, bh, ba;
  int rejects = 0;

  for (int i = 0; i < 2000; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      bbox_ekf::Meas m = to_export(sim.last_push());
      m.heading_deg += inj_h;
      m.attack_deg += inj_a;
      if (pass_known) {
        m.heading_bias_deg = inj_h;
        m.attack_bias_deg = inj_a;
      }
      if (!ekf.push(m)) ++rejects;
    }
    const auto& s = sim.snapshot();
    if (s.time < 8.0f || !s.los.origin.valid) continue;
    const bbox_ekf::Camera cam_q = bbox_ekf::Camera::from_fx(
        cfg.camera.width, cfg.camera.height, s.fx, s.fx);
    const auto now =
        ekf.predict(cam_q, {s.drone.vel.x, s.drone.vel.y, s.drone.vel.z},
                    s.time);
    if (!now.valid) continue;
    exp_h.add(wrap180(now.heading_deg - s.los.origin.heading_deg));
    exp_a.add(wrap180(now.attack_deg - s.los.origin.attack_deg));
    bh.add(now.heading_bias_deg);
    ba.add(now.attack_bias_deg);
  }

  std::printf(
      "%-14s  los hdg=%.3f att=%.3f  bias est h=%.2f (inj %.2f)  a=%.2f "
      "(inj %.2f)  rej=%d\n",
      tag, exp_h.rms(), exp_a.rms(), bh.mean(), inj_h, ba.mean(), inj_a,
      rejects);
}

int main() {
  std::printf("export BBoxEkf  (LOS + box + fx, no yaw/pitch)\n");
  std::printf("20s  10Hz  4px jitter  timing on  ideal gimbal\n\n");
  run("smooth", TargetManeuver::Smooth, false, 2000);
  run("jink", TargetManeuver::Jink, false, 2000);
  run("zoom1-3x", TargetManeuver::Smooth, true, 2000);
  std::printf("\nLOS bias  inject +1.5 heading / -0.8 attack  (score t>8s)\n");
  run_bias("raw (no corr)", 1.5f, -0.8f, false, 0.0f);
  run_bias("estimate", 1.5f, -0.8f, false, 2.0f);
  run_bias("known calib", 1.5f, -0.8f, true, 0.0f);
  return 0;
}
