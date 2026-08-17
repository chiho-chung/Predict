#include "bbox_imm_ekf.hpp"
#include "sim/sim.hpp"

#include <cmath>
#include <cstdio>

// Headless: export IMM-EKF from sim detections (LOS + box + fx). Same cases as export_ekf_check.

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

bbox_imm_ekf::Meas to_export(const TrackerMeas& m) {
  const LosAngles los = los_from_box(m.cam, m.box);
  bbox_imm_ekf::Meas out;
  out.heading_deg = los.heading_deg;
  out.attack_deg = los.attack_deg;
  out.width_px = m.box.width();
  out.height_px = m.box.height();
  out.cam = bbox_imm_ekf::Camera::from_fx(m.cam.width, m.cam.height, m.cam.fx,
                                      m.cam.fy);
  out.own_vel = {m.own_vel.x, m.own_vel.y, m.own_vel.z};
  out.t = m.t;
  return out;
}

void distort_box_range(bbox_imm_ekf::Meas& m, float inj_m) {
  if (std::fabs(inj_m) < 1e-6f) return;
  const float w = std::max(2.0f, m.width_px);
  const float r0 = std::max(1.0f, m.cam.fx * 0.36f / w);
  const float s = r0 / std::max(0.5f, r0 + inj_m);
  m.width_px *= s;
  m.height_px *= s;
}

void run(const char* tag, TargetManeuver man, bool zoom_cycle, int steps,
         const bbox_imm_ekf::Config& ekf_cfg, float size_px = 4.0f) {
  SimConfig cfg = base();
  cfg.target.maneuver = man;
  cfg.jitter.size_px = size_px;
  cfg.zoom.auto_cycle = zoom_cycle;
  cfg.zoom.min_zoom = 1.0f;
  cfg.zoom.max_zoom = 3.0f;
  cfg.zoom.period_s = 8.0f;

  Simulation sim(cfg);
  bbox_imm_ekf::BBoxImmEkf ekf(ekf_cfg);

  Acc exp_h, exp_r, exp_s, rb, sim_r;
  int rejects = 0;

  for (int i = 0; i < steps; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      if (!ekf.push(to_export(sim.last_push()))) ++rejects;
    }
    const auto& s = sim.snapshot();
    if (s.time < 2.0f || !s.los.origin.valid) continue;
    const bbox_imm_ekf::Camera cam_q = bbox_imm_ekf::Camera::from_fx(
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
      exp_r.add(now.range_m - s.true_range_m);
      rb.add(now.range_bias_m);
      if (true_w_m > 0.01f) exp_s.add(now.size_w_m - true_w_m);
    }
    if (s.track_now.valid && s.track_now.range_m > 0.1f) sim_r.add(s.range_err_m);
  }

  std::printf(
      "%-14s  rng_rms=%.2f  rng_bias=%+.2f  size=%.3f  hdg=%.3f  "
      "b_hat=%+.2f  rej=%d  (simEKF rng_rms=%.2f)\n",
      tag, exp_r.rms(), exp_r.mean(), exp_s.rms(), exp_h.rms(), rb.mean(),
      rejects, sim_r.rms());
}

void run_first_catch(const char* tag, float inj_m, float est_sigma_m) {
  SimConfig cfg = base();
  Simulation sim(cfg);
  bbox_imm_ekf::Config ekf_cfg;
  ekf_cfg.range_bias_sigma_m = est_sigma_m;
  bbox_imm_ekf::BBoxImmEkf ekf(ekf_cfg);

  Acc early, late, rb;
  int rejects = 0;
  bool first = true;

  for (int i = 0; i < 2000; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      bbox_imm_ekf::Meas m = to_export(sim.last_push());
      if (first) {
        distort_box_range(m, inj_m);
        first = false;
      }
      if (!ekf.push(m)) ++rejects;
    }
    const auto& s = sim.snapshot();
    if (!s.los.origin.valid) continue;
    const bbox_imm_ekf::Camera cam_q = bbox_imm_ekf::Camera::from_fx(
        cfg.camera.width, cfg.camera.height, s.fx, s.fx);
    const auto now =
        ekf.predict(cam_q, {s.drone.vel.x, s.drone.vel.y, s.drone.vel.z},
                    s.time);
    if (!now.valid) continue;
    rb.add(now.range_bias_m);
    const double err = now.range_m - s.true_range_m;
    if (s.time >= 2.0f && s.time < 8.0f) early.add(err);
    if (s.time >= 8.0f) late.add(err);
  }

  std::printf(
      "%-14s  t2-8 rms=%.2f bias=%+.2f  t>8 rms=%.2f bias=%+.2f  "
      "b_hat=%+.2f (inj first %+0.1f)  rej=%d\n",
      tag, early.rms(), early.mean(), late.rms(), late.mean(), rb.mean(),
      inj_m, rejects);
}

void run_range_bias(const char* tag, float inj_m, bool pass_known,
                    float est_sigma_m) {
  SimConfig cfg = base();
  Simulation sim(cfg);
  bbox_imm_ekf::Config ekf_cfg;
  ekf_cfg.range_bias_sigma_m = est_sigma_m;
  bbox_imm_ekf::BBoxImmEkf ekf(ekf_cfg);

  Acc exp_r, rb;
  int rejects = 0;

  for (int i = 0; i < 2000; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      bbox_imm_ekf::Meas m = to_export(sim.last_push());
      distort_box_range(m, inj_m);
      if (pass_known) m.range_bias_m = inj_m;
      if (!ekf.push(m)) ++rejects;
    }
    const auto& s = sim.snapshot();
    if (s.time < 8.0f || !s.los.origin.valid) continue;
    const bbox_imm_ekf::Camera cam_q = bbox_imm_ekf::Camera::from_fx(
        cfg.camera.width, cfg.camera.height, s.fx, s.fx);
    const auto now =
        ekf.predict(cam_q, {s.drone.vel.x, s.drone.vel.y, s.drone.vel.z},
                    s.time);
    if (!now.valid) continue;
    exp_r.add(now.range_m - s.true_range_m);
    rb.add(now.range_bias_m);
  }

  std::printf(
      "%-14s  rng_rms=%.2f  rng_bias=%+.2f  b_hat=%+.2f (inj %+0.1f)  rej=%d\n",
      tag, exp_r.rms(), exp_r.mean(), rb.mean(), inj_m, rejects);
}

void run_los_bias(const char* tag, float inj_h, float inj_a, bool pass_known,
                 float est_sigma_deg) {
  SimConfig cfg = base();
  Simulation sim(cfg);
  bbox_imm_ekf::Config ekf_cfg;
  ekf_cfg.los_bias_sigma_deg = est_sigma_deg;
  ekf_cfg.range_bias_sigma_m = 0.0f;
  bbox_imm_ekf::BBoxImmEkf ekf(ekf_cfg);

  Acc exp_h, exp_a, bh, ba;
  int rejects = 0;

  for (int i = 0; i < 2000; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) {
      bbox_imm_ekf::Meas m = to_export(sim.last_push());
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
    const bbox_imm_ekf::Camera cam_q = bbox_imm_ekf::Camera::from_fx(
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

}  // namespace

void run_wired_vs_standalone() {
  SimConfig cfg = base();
  cfg.tracker.type = EstimatorType::ExportImmEkf;
  Simulation sim(cfg);
  bbox_imm_ekf::BBoxImmEkf ekf;

  Acc delta, sim_r, std_r;
  for (int i = 0; i < 2000; ++i) {
    sim.step(0.01f);
    if (sim.meas_delivered_this_step()) ekf.push(to_export(sim.last_push()));
    const auto& s = sim.snapshot();
    if (s.time < 2.0f || !s.los.origin.valid) continue;
    const bbox_imm_ekf::Camera cam_q = bbox_imm_ekf::Camera::from_fx(
        cfg.camera.width, cfg.camera.height, s.fx, s.fx);
    const auto now =
        ekf.predict(cam_q, {s.drone.vel.x, s.drone.vel.y, s.drone.vel.z},
                    s.time);
    if (now.valid) std_r.add(now.range_m - s.true_range_m);
    if (s.track_now.valid && s.track_now.range_m > 0.1f) {
      sim_r.add(s.track_now.range_m - s.true_range_m);
      if (now.valid) delta.add(now.range_m - s.track_now.range_m);
    }
  }
  std::printf(
      "wired vs standalone  sim_rng=%.2f  std_rng=%.2f  |d|_rms=%.4f m\n\n",
      sim_r.rms(), std_r.rms(), delta.rms());
}

int main() {
  bbox_imm_ekf::Config off;
  off.range_bias_sigma_m = 0.0f;
  bbox_imm_ekf::Config on;
  on.range_bias_sigma_m = 1.0f;
  on.range_bias_walk_m = 0.0f;

  std::printf("export BBoxImmEkf  distance-focused  20s  10Hz  timing on\n\n");
  std::printf("--- sim Export-IMM vs standalone (same tape) ---\n");
  run_wired_vs_standalone();
  std::printf("--- range estimate OFF (default) vs ON (sigma=1, walk=0) ---\n");
  run("smooth/off", TargetManeuver::Smooth, false, 2000, off);
  run("smooth/on", TargetManeuver::Smooth, false, 2000, on);
  run("jink/off", TargetManeuver::Jink, false, 2000, off);
  run("jink/on", TargetManeuver::Jink, false, 2000, on);
  run("zoom/off", TargetManeuver::Smooth, true, 2000, off);
  run("zoom/on", TargetManeuver::Smooth, true, 2000, on);

  std::printf("\n--- first-catch stress  size jitter 8 px ---\n");
  run("sz8/off", TargetManeuver::Smooth, false, 2000, off, 8.0f);
  run("sz8/on", TargetManeuver::Smooth, false, 2000, on, 8.0f);

  std::printf("\n--- first box only +2 m, later boxes clean ---\n");
  run_first_catch("raw", 2.0f, 0.0f);
  run_first_catch("estimate", 2.0f, 1.0f);

  std::printf("\n--- inject +2 m every box (score t>8s) ---\n");
  run_range_bias("raw", 2.0f, false, 0.0f);
  run_range_bias("estimate", 2.0f, false, 1.0f);
  run_range_bias("known calib", 2.0f, true, 0.0f);
  run_range_bias("known+est", 2.0f, true, 1.0f);

  std::printf("\nLOS bias  inject +1.5 heading / -0.8 attack  (score t>8s)\n");
  run_los_bias("raw (no corr)", 1.5f, -0.8f, false, 0.0f);
  run_los_bias("estimate", 1.5f, -0.8f, false, 2.0f);
  run_los_bias("known calib", 1.5f, -0.8f, true, 0.0f);
  return 0;
}
