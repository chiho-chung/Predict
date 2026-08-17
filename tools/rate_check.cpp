#include "csv_log.hpp"
#include "sim.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

SimConfig base_cfg() {
  SimConfig cfg;
  cfg.camera.width = 480;
  cfg.camera.height = 360;
  cfg.rates.sim_hz = 100.0f;
  cfg.rates.detect_hz = 10.0f;
  cfg.rates.detect_latency_s = 0.05f;
  cfg.predict.horizon_s = 0.5f;
  cfg.predict.enabled = true;
  cfg.jitter.enabled = true;
  return cfg;
}

struct Acc {
  double sum = 0;
  double peak = 0;
  int n = 0;
  void add(double v) {
    sum += v * v;
    if (std::fabs(v) > peak) peak = std::fabs(v);
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum / n) : 0.0; }
};

// Heading is an angle: differences must be wrapped to +/-180 or a crossing of
// the branch cut shows up as a 360 deg spike.
double diff_deg(double a, double b) {
  double d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

struct Result {
  const char* name = "";
  bool has_range = false;
  double est_px = 0;
  double held_px = 0;
  double track_px = 0;
  double los_head = 0;
  double los_att = 0;
  double pred_head = 0;
  double range_rms = 0;
  double range_peak = 0;
  double range_bias = 0;
  double size_rms = 0;
  double speed_rms = 0;
  double mean_prob[3] = {0, 0, 0};
  int models = 0;
  int rejects = 0;
};

Result run(EstimatorType type, float horizon_s, int steps,
           TargetManeuver maneuver, const TrackerConfig* tune = nullptr,
           bool jitter = true, float detect_hz = 10.0f,
           const char* exp_tag = nullptr) {
  SimConfig cfg = base_cfg();
  if (tune) cfg.tracker = *tune;
  cfg.tracker.type = type;
  cfg.predict.horizon_s = horizon_s;
  cfg.target.maneuver = maneuver;
  cfg.jitter.enabled = jitter;
  cfg.rates.detect_hz = detect_hz;

  char label[96];
  if (exp_tag && exp_tag[0]) {
    std::snprintf(label, sizeof(label), "%s", exp_tag);
  } else {
    std::snprintf(label, sizeof(label), "%s_%s_H%.2f_hz%.0f_c%.2f%s",
                  estimator_name(type),
                  maneuver == TargetManeuver::Jink ? "jink" : "smooth",
                  horizon_s, detect_hz, cfg.tracker.meas_corr,
                  jitter ? "" : "_nojitter");
  }
  ExperimentLog exp;
  exp.begin("check", label, cfg);

  Simulation sim(cfg);
  const float dt = cfg.sim_dt();
  const int lead_steps = static_cast<int>(horizon_s / dt + 0.5f);

  Result r;
  r.name = estimator_name(type);

  Acc est_px, held_px, track_px, los_h, los_a, pred_h, rng, sz, spd;
  double range_bias_sum = 0;
  double prob_sum[3] = {0, 0, 0};
  int range_n = 0;
  std::vector<LosAngles> origin_hist, pred_hist;

  // Skip the first second: the filter is still acquiring range from the prior.
  const int warmup = 100;

  for (int i = 0; i < steps; ++i) {
    sim.step(dt);
    const auto& s = sim.snapshot();
    exp.sample(cfg, s);
    origin_hist.push_back(s.los.origin);
    pred_hist.push_back(s.los.predicted);
    if (i < warmup) continue;

    if (s.detection.visible && s.detection_gt.visible) {
      est_px.add(s.est_err_px);
      held_px.add(s.held_err_px);
      track_px.add(s.track_err_px);
    }
    if (s.los.origin.valid && s.los.estimate.valid) {
      los_h.add(diff_deg(s.los.estimate.heading_deg, s.los.origin.heading_deg));
      los_a.add(diff_deg(s.los.estimate.attack_deg, s.los.origin.attack_deg));
    }
    // CV-pixel works purely in the image and reports no range, so scoring its
    // range would just measure the true range.
    if (s.track_now.valid && s.track_now.range_m > 0) {
      const double err = s.track_now.range_m - s.true_range_m;
      rng.add(err);
      range_bias_sum += err;
      ++range_n;
      sz.add(s.size_err_m);
      const float true_speed = s.target.vel.length();
      spd.add(s.track_now.speed_mps - true_speed);
      for (int k = 0; k < s.track_now.model_count && k < 3; ++k) {
        prob_sum[k] += s.track_now.model_prob[k];
      }
      r.models = s.track_now.model_count;
    }
  }

  for (size_t i = 0; i + lead_steps < pred_hist.size(); ++i) {
    if (static_cast<int>(i) < warmup) continue;
    const LosAngles& p = pred_hist[i];
    const LosAngles& future = origin_hist[i + lead_steps];
    if (!p.valid || !future.valid) continue;
    pred_h.add(diff_deg(p.heading_deg, future.heading_deg));
  }

  r.est_px = est_px.rms();
  r.held_px = held_px.rms();
  r.track_px = track_px.rms();
  r.los_head = los_h.rms();
  r.los_att = los_a.rms();
  r.pred_head = pred_h.rms();
  r.has_range = range_n > 0;
  r.range_rms = rng.rms();
  r.range_peak = rng.peak;
  r.range_bias = range_n ? range_bias_sum / range_n : 0.0;
  r.size_rms = sz.rms();
  r.speed_rms = spd.rms();
  if (range_n) {
    for (int k = 0; k < 3; ++k) r.mean_prob[k] = prob_sum[k] / range_n;
  }
  r.rejects = sim.snapshot().tracker_rejects;
  exp.end();
  return r;
}

void print_table(const char* title, float horizon_s, int steps,
                 TargetManeuver maneuver) {
  std::printf("\n=== %s target, horizon %.2f s, %.0f s run ===\n", title,
              horizon_s, steps / 100.0);
  std::printf("%-9s %7s %7s %7s %7s %8s %8s %7s %7s %5s  %s\n", "estimator",
              "estPx", "heldPx", "losHdg", "losAtt", "rngRMS", "rngPeak",
              "rngBias", "sizeRMS", "rej", "model mix");
  const EstimatorType types[] = {EstimatorType::CvPixel, EstimatorType::Ekf,
                                 EstimatorType::Ukf, EstimatorType::ImmEkf,
                                 EstimatorType::ImmUkf};
  for (EstimatorType t : types) {
    const Result r = run(t, horizon_s, steps, maneuver);
    std::printf("%-9s %7.2f %7.2f %7.3f %7.3f", r.name, r.est_px, r.held_px,
                r.los_head, r.los_att);
    if (r.has_range) {
      std::printf(" %8.2f %8.2f %7.2f %7.3f %5d", r.range_rms, r.range_peak,
                  r.range_bias, r.size_rms, r.rejects);
    } else {
      std::printf(" %8s %8s %7s %7s %5d", "-", "-", "-", "-", r.rejects);
    }
    if (r.models > 1) {
      std::printf("  quiet %.2f / mod %.2f / hard %.2f", r.mean_prob[0],
                  r.mean_prob[1], r.mean_prob[2]);
    }
    std::printf("\n");
  }
}

// Comparing a tuned single filter against an untuned IMM would be unfair, so
// sweep the process noise of both before drawing any conclusion.
void sweep_single(const char* label, TargetManeuver maneuver) {
  std::printf("\n=== EKF process-noise sweep, %s target ===\n", label);
  std::printf("%-9s %8s %8s %8s\n", "sigma_a", "estPx", "losHdg", "rngRMS");
  for (float sa : {1.0f, 2.0f, 3.0f, 5.0f, 8.0f, 12.0f, 20.0f}) {
    TrackerConfig tc;
    tc.sigma_accel = sa;
    const Result r = run(EstimatorType::Ekf, 0.5f, 3000, maneuver, &tc);
    std::printf("%-9.1f %8.2f %8.3f %8.2f\n", sa, r.est_px, r.los_head,
                r.range_rms);
  }
}

void sweep_imm(const char* label, TargetManeuver maneuver, bool jitter) {
  std::printf("\n=== IMM model-set sweep, %s target, jitter %s ===\n", label,
              jitter ? "ON" : "OFF");
  std::printf("%-10s %8s %8s %8s  %s\n", "imm set", "estPx", "losHdg", "rngRMS",
              "mean model mix");
  const float sets[4][3] = {{0.6f, 4.0f, 14.0f},
                            {1.5f, 5.0f, 12.0f},
                            {3.0f, 6.0f, 12.0f},
                            {5.0f, 10.0f, 20.0f}};
  for (const auto& s : sets) {
    TrackerConfig tc;
    tc.imm_sigma_accel[0] = s[0];
    tc.imm_sigma_accel[1] = s[1];
    tc.imm_sigma_accel[2] = s[2];
    const Result r =
        run(EstimatorType::ImmEkf, 0.5f, 3000, maneuver, &tc, jitter);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f/%.0f/%.0f", s[0], s[1], s[2]);
    std::printf("%-10s %8.2f %8.3f %8.2f  %.2f / %.2f / %.2f\n", buf, r.est_px,
                r.los_head, r.range_rms, r.mean_prob[0], r.mean_prob[1],
                r.mean_prob[2]);
  }
}

// Does the mode mix actually respond to what the target is doing? If the mix is
// the same for a smooth and a jinking target then the bank is not discriminating
// and the IMM is only paying the cost of mixing.
void validate_imm() {
  std::printf("\n=== IMM discrimination check (mix must differ by target) ===\n");
  std::printf(
      "bank forced to 1.5/5/12: the shipped default is deliberately\n"
      "responsive, and a quiet model that already absorbs the manoeuvre\n"
      "cannot be told apart from a manoeuvre model.\n");
  std::printf("%-28s %-22s %-22s\n", "condition", "smooth mix", "jinking mix");

  struct Case {
    const char* label;
    float sigma_px;
    bool jitter;
    float detect_hz;
  };
  const Case cases[] = {
      {"as configured (3 px, 10 Hz)", 3.0f, true, 10.0f},
      {"clean detector (0.5 px)", 0.5f, false, 10.0f},
      {"clean + slow (0.5 px, 4 Hz)", 0.5f, false, 4.0f},
      {"clean + fast (0.5 px, 25 Hz)", 0.5f, false, 25.0f},
  };

  for (const Case& c : cases) {
    TrackerConfig tc;
    tc.sigma_px_center = c.sigma_px;
    tc.sigma_px_size = c.sigma_px;
    tc.imm_sigma_accel[0] = 1.5f;
    tc.imm_sigma_accel[1] = 5.0f;
    tc.imm_sigma_accel[2] = 12.0f;
    const Result s = run(EstimatorType::ImmEkf, 0.5f, 3000,
                         TargetManeuver::Smooth, &tc, c.jitter, c.detect_hz);
    const Result j = run(EstimatorType::ImmEkf, 0.5f, 3000,
                         TargetManeuver::Jink, &tc, c.jitter, c.detect_hz);
    std::printf("%-28s %.2f / %.2f / %.2f      %.2f / %.2f / %.2f\n", c.label,
                s.mean_prob[0], s.mean_prob[1], s.mean_prob[2], j.mean_prob[0],
                j.mean_prob[1], j.mean_prob[2]);
  }
}

void noise_ab() {
  std::printf("\n=== Gauss-Markov measurement whitening A/B (30 s) ===\n");
  std::printf("%-22s %7s %7s %8s %7s\n", "case", "estPx", "losHdg", "rngRMS",
              "sizeRMS");
  struct Case {
    const char* name;
    EstimatorType type;
    float corr;
    TargetManeuver man;
  };
  const Case cases[] = {
      {"EKF off, smooth", EstimatorType::Ekf, 0.0f, TargetManeuver::Smooth},
      {"EKF on,  smooth", EstimatorType::Ekf, 0.6f, TargetManeuver::Smooth},
      {"UKF off, smooth", EstimatorType::Ukf, 0.0f, TargetManeuver::Smooth},
      {"UKF on,  smooth", EstimatorType::Ukf, 0.6f, TargetManeuver::Smooth},
      {"EKF off, jink", EstimatorType::Ekf, 0.0f, TargetManeuver::Jink},
      {"EKF on,  jink", EstimatorType::Ekf, 0.6f, TargetManeuver::Jink},
      {"UKF off, jink", EstimatorType::Ukf, 0.0f, TargetManeuver::Jink},
      {"UKF on,  jink", EstimatorType::Ukf, 0.6f, TargetManeuver::Jink},
  };
  for (const Case& c : cases) {
    TrackerConfig tc;
    tc.meas_corr = c.corr;
    const Result r = run(c.type, 0.5f, 3000, c.man, &tc);
    std::printf("%-22s %7.2f %7.3f %8.2f %7.3f\n", c.name, r.est_px, r.los_head,
                r.range_rms, r.size_rms);
  }
}

}  // namespace

int main() {
  std::printf(
      "sim 100 Hz, detector 10 Hz + 50 ms latency, jitter on\n"
      "estPx : delay-removed box centre vs truth box centre (px RMS)\n"
      "losHdg: estimated LOS heading vs true LOS heading (deg RMS)\n"
      "rng*  : range estimate error (m); size: box-spanning extent error (m)\n");

  noise_ab();
  print_table("smooth", 0.5f, 3000, TargetManeuver::Smooth);
  print_table("jinking", 0.5f, 3000, TargetManeuver::Jink);
  return 0;
}
