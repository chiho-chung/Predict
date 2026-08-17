#include "estimator/predictor.hpp"
#include "sim/sim.hpp"

#include <cmath>
#include <cstdio>

// Headless: +H LOS vs origin at t+H (the same scoring as the plot window).

int g_fail = 0;

void expect(const char* what, bool ok) {
  if (ok) return;
  std::printf("FAIL %s\n", what);
  g_fail = 1;
}

namespace {

struct Acc {
  double sum_sq = 0;
  double peak = 0;
  int n = 0;
  void add(double v) {
    if (!std::isfinite(v)) return;
    sum_sq += v * v;
    if (std::fabs(v) > peak) peak = std::fabs(v);
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
};

SimConfig base(bool servo, bool chase) {
  SimConfig cfg;
  cfg.camera.width = 640;
  cfg.camera.height = 480;
  cfg.camera.fov_wide_deg = 70.0f;
  cfg.camera.set_zoom(1.0f);
  cfg.rates.sim_hz = 100.0f;
  cfg.rates.detect_hz = 10.0f;
  cfg.rates.detect_latency_s = 0.05f;
  cfg.predict.enabled = true;
  cfg.predict.horizon_s = 0.5f;
  cfg.jitter.enabled = true;
  cfg.jitter.center_px = 4.0f;
  cfg.jitter.size_frac = 0.10f;
  cfg.jitter.smooth = 0.6f;
  cfg.timing.enabled = true;
  cfg.gimbal.ideal = !servo;
  cfg.chase_enabled = chase;
  return cfg;
}

void run(const char* tag, EstimatorType type, TargetManeuver man, bool servo,
         bool chase, int steps) {
  SimConfig cfg = base(servo, chase);
  cfg.tracker.type = type;
  cfg.target.maneuver = man;
  Simulation sim(cfg);

  Acc now_h, now_a, hdg, att, rng;
  for (int i = 0; i < steps; ++i) {
    sim.step(0.01f);
    const auto& s = sim.snapshot();
    if (s.time < 2.0f) continue;
    if (s.los.origin.valid && s.los.estimate.valid) {
      float dh = s.los.estimate.heading_deg - s.los.origin.heading_deg;
      float da = s.los.estimate.attack_deg - s.los.origin.attack_deg;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      now_h.add(dh);
      now_a.add(da);
    }
    if (s.pred_err_valid) {
      hdg.add(s.pred_err_hdg);
      att.add(s.pred_err_att);
    }
    if (s.track_now.valid && s.track_now.range_m > 0.1f) {
      rng.add(s.track_now.range_m - s.true_range_m);
    }
  }

  std::printf(
      "%-12s %-6s %-5s %-5s  now_h=%.3f  +H_h=%.3f peak=%.2f  "
      "+H_a=%.3f peak=%.2f  rng=%.2f\n",
      tag, man == TargetManeuver::Jink ? "jink" : "smooth",
      servo ? "servo" : "ideal", chase ? "chase" : "hold", now_h.rms(),
      hdg.rms(), hdg.peak, att.rms(), att.peak, rng.rms());

  if (type == EstimatorType::CvPixel) return;
  const bool jink = man == TargetManeuver::Jink;
  expect("now heading RMS", now_h.rms() < 1.5);
  if (!chase) {
    expect("+H hold heading peak", hdg.peak < 6.0);
    expect("+H hold attack peak", att.peak < 4.0);
  } else if (!jink) {
    expect("+H chase-smooth heading RMS", hdg.rms() < 8.0);
    expect("+H chase-smooth heading peak", hdg.peak < 20.0);
    expect("+H chase-smooth attack peak", att.peak < 20.0);
  } else {
    // 0.5 s CV cannot track a 7 m/s^2 jink; just fail a through-origin flip.
    expect("+H chase-jink heading peak", hdg.peak < 50.0);
    expect("+H chase-jink attack peak", att.peak < 30.0);
  }
}

}  // namespace

int main() {
  std::printf(
      "+H scored vs origin at t+H  (8 s, 10 Hz, 50 ms, size +/-10%%, H=0.5 s)\n");
  const EstimatorType types[] = {
      EstimatorType::CvPixel, EstimatorType::ExportEkf,
      EstimatorType::ExportImmEkf, EstimatorType::Ekf, EstimatorType::ImmEkf};
  std::printf("\n--- chase ON, gimbal servo (live default) ---\n");
  for (EstimatorType t : types) {
    run(estimator_name(t), t, TargetManeuver::Smooth, true, true, 800);
    run(estimator_name(t), t, TargetManeuver::Jink, true, true, 800);
  }
  std::printf("\n--- chase OFF, gimbal servo ---\n");
  for (EstimatorType t : {EstimatorType::ExportEkf, EstimatorType::Ekf,
                          EstimatorType::ImmEkf}) {
    run(estimator_name(t), t, TargetManeuver::Smooth, true, false, 800);
    run(estimator_name(t), t, TargetManeuver::Jink, true, false, 800);
  }
  return g_fail;
}
