#include "sim/sim.hpp"

#include <cmath>
#include <cstdio>

// Headless: EKF / IMM-EKF with a 1x–3x zoom cycle vs the same scene at 1x.
// Proves the filter uses per-frame fx (box size scales with zoom).

namespace {

struct Acc {
  double sum_sq = 0;
  int n = 0;
  void add(double v) {
    if (!std::isfinite(v)) return;
    sum_sq += v * v;
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
};

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
  return cfg;
}

void run(const char* tag, EstimatorType type, bool zoom_cycle) {
  SimConfig cfg = base();
  cfg.tracker.type = type;
  cfg.zoom.auto_cycle = zoom_cycle;
  cfg.zoom.min_zoom = 1.0f;
  cfg.zoom.max_zoom = 3.0f;
  cfg.zoom.period_s = 8.0f;

  Simulation sim(cfg);
  Acc px, hdg, att, rng;
  float zoom_lo = 99, zoom_hi = 0;
  const int steps = 2000;  // 20 s
  for (int i = 0; i < steps; ++i) {
    sim.step(0.01f);
    const auto& s = sim.snapshot();
    if (s.time < 2.0f) continue;
    px.add(s.est_err_px);
    if (s.los.origin.valid && s.los.estimate.valid) {
      float dh = s.los.estimate.heading_deg - s.los.origin.heading_deg;
      float da = s.los.estimate.attack_deg - s.los.origin.attack_deg;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      hdg.add(dh);
      att.add(da);
    }
    if (s.track_now.valid && s.track_now.range_m > 0.1f) rng.add(s.range_err_m);
    if (s.zoom < zoom_lo) zoom_lo = s.zoom;
    if (s.zoom > zoom_hi) zoom_hi = s.zoom;
  }
  std::printf("%-10s  %-11s  zoom %.2f-%.2f  px=%.2f  hdg=%.3f  att=%.3f  rng=%.2f\n",
              tag, estimator_name(type), zoom_lo, zoom_hi, px.rms(), hdg.rms(),
              att.rms(), rng.rms());
}

}  // namespace

int main() {
  std::printf("zoom check  20s  10Hz  timing on  ideal gimbal\n");
  run("fixed 1x", EstimatorType::ExportEkf, false);
  run("cycle 1-3x", EstimatorType::ExportEkf, true);
  run("fixed 1x", EstimatorType::ExportImmEkf, false);
  run("cycle 1-3x", EstimatorType::ExportImmEkf, true);
  run("fixed 1x", EstimatorType::Ekf, false);
  run("cycle 1-3x", EstimatorType::Ekf, true);
  run("fixed 1x", EstimatorType::ImmEkf, false);
  run("cycle 1-3x", EstimatorType::ImmEkf, true);
  return 0;
}
