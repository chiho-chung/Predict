#include "app/bench.hpp"

#include "io/csv_log.hpp"
#include "estimator/predictor.hpp"
#include "sim/sim.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

namespace {

bool mkdir_one(const char* p) {
#ifdef _WIN32
  return _mkdir(p) == 0 || errno == EEXIST;
#else
  return mkdir(p, 0755) == 0 || errno == EEXIST;
#endif
}

std::string stamp_now() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_MSC_VER)
  localtime_s(&tm, &t);
#elif defined(_WIN32)
  if (const std::tm* p = std::localtime(&t)) tm = *p;
#else
  localtime_r(&t, &tm);
#endif
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900,
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

void sanitize(char* s) {
  for (; *s; ++s) {
    if (*s == ' ' || *s == '/' || *s == '\\' || *s == ',' || *s == ':') *s = '_';
  }
}

float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

struct Acc {
  double sum_sq = 0;
  double sum = 0;
  double peak = 0;
  int n = 0;
  void add(double v) {
    if (!std::isfinite(v)) return;
    sum_sq += v * v;
    sum += v;
    if (std::fabs(v) > peak) peak = std::fabs(v);
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
  double mean() const { return n ? sum / n : 0.0; }
};

struct Scenario {
  const char* id;
  const char* name;
  TargetManeuver man;
  float detect_hz;
  float latency_s;
  bool jitter;
  float center_px;
  float size_px;
  Vec3 drone_pos;
  Vec3 target_pos;
  Vec3 target_vel;
  uint32_t seed;
  bool timing = true;
  float period_jit = 0.20f;
  float lat_jit = 0.010f;
  float stamp_jit = 0.002f;
  bool zoom_cycle = false;
};

struct RunScore {
  EstimatorType type = EstimatorType::ExportEkf;
  Acc est_px, held_px, los_h, los_a, range, size, speed;
  int rejects = 0;
  int updates = 0;
  int steps = 0;
  std::string ts_path;
};

const EstimatorType kAllEst[] = {
    EstimatorType::CvPixel,   EstimatorType::ExportEkf,  EstimatorType::ExportImmEkf,
    EstimatorType::Ekf,       EstimatorType::Ukf,        EstimatorType::ImmEkf,
    EstimatorType::ImmUkf,    EstimatorType::EkfLos,     EstimatorType::UkfLos,
    EstimatorType::ImmEkfLos, EstimatorType::ImmUkfLos,
};

const char* meas_tag(EstimatorType t) {
  if (!estimator_uses_filter(t)) return "px";
  return estimator_uses_bbox(t) ? "bbox" : "los";
}

const char* family_name(EstimatorType t) {
  switch (t) {
    case EstimatorType::ExportEkf:
      return "Export-EKF";
    case EstimatorType::ExportImmEkf:
      return "Export-IMM";
    case EstimatorType::Ekf:
    case EstimatorType::EkfLos:
      return "EKF";
    case EstimatorType::Ukf:
    case EstimatorType::UkfLos:
      return "UKF";
    case EstimatorType::ImmEkf:
    case EstimatorType::ImmEkfLos:
      return "IMM-EKF";
    case EstimatorType::ImmUkf:
    case EstimatorType::ImmUkfLos:
      return "IMM-UKF";
    default:
      return "CV";
  }
}

SimConfig make_base() {
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
  cfg.timing.period_jitter_frac = 0.20f;
  cfg.timing.latency_jitter_s = 0.010f;
  cfg.timing.stamp_jitter_s = 0.002f;
  cfg.chase_enabled = true;
  // Ideal gimbal: detections do not depend on which filter is steering,
  // so every estimator on a scenario sees the same tape.
  cfg.gimbal.ideal = true;
  return cfg;
}

std::vector<Scenario> make_scenarios(bool quick) {
  const Vec3 d0{0, -12, 10};
  const Vec3 t0{6, 6, 6};
  const Vec3 v0{5, 2, 0};
  const Vec3 d_far{0, -28, 12};
  const Vec3 t_far{10, 10, 8};
  const Vec3 v_far{4, 1, 0};
  const Vec3 d_near{0, -8, 8};
  const Vec3 t_near{3, 3, 6};
  const Vec3 v_near{6, -1, 0.5f};
  const Vec3 v_alt{-3, 5, 0};

  std::vector<Scenario> s = {
      {"s01", "smooth_10hz_50ms", TargetManeuver::Smooth, 10, 0.05f, true, 4,
       4.0f, d0, t0, v0, 0xC0FFEEu},
      {"s02", "jink_10hz_50ms", TargetManeuver::Jink, 10, 0.05f, true, 4, 4.0f,
       d0, t0, v0, 0xC0FFEEu},
      {"s03", "smooth_10hz_150ms", TargetManeuver::Smooth, 10, 0.15f, true, 4,
       4.0f, d0, t0, v0, 0xC0FFEEu},
      {"s04", "jink_10hz_150ms", TargetManeuver::Jink, 10, 0.15f, true, 4, 4.0f,
       d0, t0, v0, 0xC0FFEEu},
      {"s05", "smooth_20hz_50ms", TargetManeuver::Smooth, 20, 0.05f, true, 4,
       4.0f, d0, t0, v0, 0xC0FFEEu},
      {"s06", "jink_4hz_50ms", TargetManeuver::Jink, 4, 0.05f, true, 4, 4.0f, d0,
       t0, v0, 0xC0FFEEu},
      {"s07", "smooth_10hz_nojitter", TargetManeuver::Smooth, 10, 0.05f, false, 4,
       4.0f, d0, t0, v0, 0xC0FFEEu},
      {"s08", "jink_10hz_nojitter", TargetManeuver::Jink, 10, 0.05f, false, 4,
       4.0f, d0, t0, v0, 0xC0FFEEu},
      {"s09", "far_smooth", TargetManeuver::Smooth, 10, 0.05f, true, 4, 4.0f,
       d_far, t_far, v_far, 0xA11CEu},
      {"s10", "close_jink", TargetManeuver::Jink, 10, 0.05f, true, 4, 4.0f,
       d_near, t_near, v_near, 0xBEEFu},
      {"s11", "seed2_smooth", TargetManeuver::Smooth, 10, 0.05f, true, 4, 4.0f,
       d0, t0, v_alt, 0xCAFEu},
      {"s12", "hi_jitter_jink", TargetManeuver::Jink, 10, 0.05f, true, 8, 8.0f,
       d0, t0, v0, 0xC0FFEEu},
      {"s13", "stable_timing", TargetManeuver::Smooth, 10, 0.05f, true, 4, 4.0f,
       d0, t0, v0, 0xC0FFEEu, false, 0.0f, 0.0f, 0.0f},
      {"s14", "heavy_period", TargetManeuver::Smooth, 10, 0.05f, true, 4, 4.0f,
       d0, t0, v0, 0xC0FFEEu, true, 0.40f, 0.010f, 0.002f},
      {"s15", "heavy_stamp", TargetManeuver::Jink, 10, 0.05f, true, 4, 4.0f, d0,
       t0, v0, 0xC0FFEEu, true, 0.20f, 0.015f, 0.008f},
      {"s16", "zoom_1to3x", TargetManeuver::Smooth, 10, 0.05f, true, 4, 4.0f, d0,
       t0, v0, 0xC0FFEEu, true, 0.20f, 0.010f, 0.002f, true},
  };
  if (quick) s.resize(4);
  return s;
}

SimConfig cfg_of(const Scenario& sc) {
  SimConfig cfg = make_base();
  cfg.target.maneuver = sc.man;
  cfg.rates.detect_hz = sc.detect_hz;
  cfg.rates.detect_latency_s = sc.latency_s;
  cfg.jitter.enabled = sc.jitter;
  cfg.jitter.center_px = sc.center_px;
  cfg.jitter.size_px = sc.size_px;
  cfg.spawn.drone_pos = sc.drone_pos;
  cfg.spawn.target_pos = sc.target_pos;
  cfg.spawn.target_vel = sc.target_vel;
  cfg.spawn.rng_seed = sc.seed;
  cfg.timing.enabled = sc.timing;
  cfg.timing.period_jitter_frac = sc.period_jit;
  cfg.timing.latency_jitter_s = sc.lat_jit;
  cfg.timing.stamp_jitter_s = sc.stamp_jit;
  cfg.zoom.auto_cycle = sc.zoom_cycle;
  cfg.zoom.min_zoom = 1.0f;
  cfg.zoom.max_zoom = 3.0f;
  cfg.zoom.period_s = 8.0f;
  return cfg;
}

RunScore run_one(const Scenario& sc, EstimatorType type, int steps, float dt,
                 const std::string& run_path, FILE* tape, bool write_tape) {
  SimConfig cfg = cfg_of(sc);
  cfg.tracker.type = type;

  Simulation sim(cfg);
  ExperimentLog exp;
  char label[96];
  std::snprintf(label, sizeof(label), "%s_%s", sc.id, estimator_name(type));
  sanitize(label);
  exp.begin_file(run_path.c_str(), "auto", label, cfg, false);

  if (write_tape && tape) {
    std::fprintf(tape,
                 "t_s,stamp_s,t_true,latency_s,period_s,stamp_err_s,"
                 "u,v,w,h,own_vx,own_vy,own_vz,"
                 "drone_x,drone_y,drone_z,tgt_x,tgt_y,tgt_z,"
                 "true_range_m,origin_hdg,origin_att\n");
  }

  RunScore out;
  out.type = type;
  constexpr float kWarmup = 2.0f;

  for (int i = 0; i < steps; ++i) {
    sim.step(dt);
    const auto& snap = sim.snapshot();
    exp.sample(cfg, snap);
    ++out.steps;

    if (write_tape && tape && sim.meas_delivered_this_step()) {
      const TrackerMeas& m = sim.last_push();
      const Vec2 c = m.box.center();
      std::fprintf(tape,
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f\n",
                   snap.time, m.t, m.t_true, m.latency_s, snap.last_period_s,
                   snap.stamp_err_s, c.x, c.y, m.box.width(), m.box.height(),
                   m.own_vel.x, m.own_vel.y, m.own_vel.z, snap.drone.pos.x,
                   snap.drone.pos.y, snap.drone.pos.z, snap.target.pos.x,
                   snap.target.pos.y, snap.target.pos.z, snap.true_range_m,
                   snap.los.origin.valid ? snap.los.origin.heading_deg : NAN,
                   snap.los.origin.valid ? snap.los.origin.attack_deg : NAN);
    }

    if (snap.time < kWarmup) continue;
    out.est_px.add(snap.est_err_px);
    out.held_px.add(snap.held_err_px);
    if (snap.los.origin.valid && snap.los.estimate.valid) {
      out.los_h.add(wrap180(snap.los.estimate.heading_deg -
                            snap.los.origin.heading_deg));
      out.los_a.add(wrap180(snap.los.estimate.attack_deg -
                            snap.los.origin.attack_deg));
    }
    if (estimator_uses_filter(type) && snap.track_now.valid &&
        snap.track_now.range_m > 0.1f && snap.true_range_m > 0.1f) {
      out.range.add(snap.range_err_m);
      out.size.add(snap.size_err_m);
      out.speed.add(snap.track_now.speed_mps - snap.target.vel.length());
    }
    out.rejects = snap.tracker_rejects;
    out.updates = snap.tracker_updates;
  }

  exp.end();
  out.ts_path = run_path;
  return out;
}

const RunScore* find_score(const std::vector<RunScore>& runs, EstimatorType t) {
  for (const RunScore& r : runs) {
    if (r.type == t) return &r;
  }
  return nullptr;
}

}  // namespace

int run_auto_bench(float seconds, bool quick) {
  if (seconds < 4.0f) seconds = 4.0f;
  if (seconds > 120.0f) seconds = 120.0f;

  mkdir_one("logs");
  const std::string root = std::string("logs/auto_") + stamp_now();
  mkdir_one(root.c_str());
  const std::string runs_dir = root + "/runs";
  const std::string tapes_dir = root + "/tapes";
  mkdir_one(runs_dir.c_str());
  mkdir_one(tapes_dir.c_str());

  const std::vector<Scenario> scenarios = make_scenarios(quick);
  const float dt = 0.01f;
  const int steps = static_cast<int>(seconds / dt + 0.5f);

  FILE* sc_f = std::fopen((root + "/scenarios.csv").c_str(), "w");
  FILE* sum_f = std::fopen((root + "/summary.csv").c_str(), "w");
  FILE* cmp_f = std::fopen((root + "/compare.csv").c_str(), "w");
  FILE* chosen_f = std::fopen((root + "/chosen_ekf.csv").c_str(), "w");
  if (!sc_f || !sum_f || !cmp_f || !chosen_f) {
    std::fprintf(stderr, "auto: could not create CSVs under %s\n", root.c_str());
    if (sc_f) std::fclose(sc_f);
    if (sum_f) std::fclose(sum_f);
    if (cmp_f) std::fclose(cmp_f);
    if (chosen_f) std::fclose(chosen_f);
    return 1;
  }

  std::fprintf(sc_f,
               "id,name,maneuver,detect_hz,latency_s,jitter,center_px,size_px,"
               "timing,period_jit,lat_jit_s,stamp_jit_s,"
               "drone_x,drone_y,drone_z,tgt_x,tgt_y,tgt_z,tgt_vx,tgt_vy,tgt_vz,"
               "seed,duration_s,ideal_gimbal\n");
  std::fprintf(sum_f,
               "scenario,name,estimator,family,meas,maneuver,detect_hz,latency_s,"
               "jitter,timing,period_jit,lat_jit_s,stamp_jit_s,"
               "est_px_rms,held_px_rms,los_head_rms,los_att_rms,"
               "range_rms,range_bias,range_peak,size_rms,speed_rms,"
               "updates,rejects,steps,timeseries\n");
  std::fprintf(chosen_f,
               "scenario,name,estimator,est_px_rms,los_head_rms,los_att_rms,"
               "range_rms,size_rms,rejects,timing\n");
  std::fprintf(cmp_f,
               "scenario,name,family,maneuver,detect_hz,latency_s,jitter,"
               "bbox_est_px,los_est_px,d_est_px,"
               "bbox_hdg,los_hdg,d_hdg,"
               "bbox_att,los_att,d_att,"
               "bbox_range,los_range,d_range,"
               "bbox_size,los_size\n");

  std::printf("auto bench  %s  %.0fs  %d scenarios x %d estimators\n",
              root.c_str(), seconds, static_cast<int>(scenarios.size()),
              static_cast<int>(sizeof(kAllEst) / sizeof(kAllEst[0])));

  for (const Scenario& sc : scenarios) {
    std::fprintf(sc_f,
                 "%s,%s,%s,%.2f,%.3f,%d,%.1f,%.3f,"
                 "%d,%.3f,%.4f,%.4f,"
                 "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                 "%u,%.1f,1\n",
                 sc.id, sc.name,
                 sc.man == TargetManeuver::Jink ? "jink" : "smooth",
                 sc.detect_hz, sc.latency_s, sc.jitter ? 1 : 0, sc.center_px,
                 sc.size_px, sc.timing ? 1 : 0, sc.period_jit, sc.lat_jit,
                 sc.stamp_jit, sc.drone_pos.x, sc.drone_pos.y, sc.drone_pos.z,
                 sc.target_pos.x, sc.target_pos.y, sc.target_pos.z,
                 sc.target_vel.x, sc.target_vel.y, sc.target_vel.z, sc.seed,
                 seconds);

    std::vector<RunScore> scores;
    scores.reserve(sizeof(kAllEst) / sizeof(kAllEst[0]));

    bool first = true;
    for (EstimatorType type : kAllEst) {
      char est_safe[64];
      std::snprintf(est_safe, sizeof(est_safe), "%s", estimator_name(type));
      sanitize(est_safe);
      const std::string run_path =
          runs_dir + "/" + sc.id + "_" + est_safe + ".csv";
      const std::string tape_path = tapes_dir + "/" + sc.id + "_meas.csv";

      FILE* tape = nullptr;
      if (first) {
        tape = std::fopen(tape_path.c_str(), "w");
        if (!tape) {
          std::fprintf(stderr, "auto: could not write %s\n", tape_path.c_str());
        }
      }

      std::printf("  %s  %-12s ...", sc.id, estimator_name(type));
      std::fflush(stdout);
      RunScore sc_run =
          run_one(sc, type, steps, dt, run_path, tape, first && tape);
      if (tape) std::fclose(tape);
      first = false;

      std::printf("  est=%.2fpx  hdg=%.3fdeg  att=%.3fdeg  rng=%.2fm\n",
                  sc_run.est_px.rms(), sc_run.los_h.rms(), sc_run.los_a.rms(),
                  sc_run.range.rms());

      if (type == EstimatorType::ExportEkf ||
          type == EstimatorType::ExportImmEkf || type == EstimatorType::Ekf ||
          type == EstimatorType::ImmEkf) {
        std::fprintf(chosen_f, "%s,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n",
                     sc.id, sc.name, estimator_name(type), sc_run.est_px.rms(),
                     sc_run.los_h.rms(), sc_run.los_a.rms(), sc_run.range.rms(),
                     sc_run.size.rms(), sc_run.rejects, sc.timing ? 1 : 0);
      }

      std::fprintf(sum_f,
                   "%s,%s,%s,%s,%s,%s,%.2f,%.3f,%d,%d,%.3f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,%.4f,"
                   "%d,%d,%d,%s\n",
                   sc.id, sc.name, estimator_name(type), family_name(type),
                   meas_tag(type),
                   sc.man == TargetManeuver::Jink ? "jink" : "smooth",
                   sc.detect_hz, sc.latency_s, sc.jitter ? 1 : 0,
                   sc.timing ? 1 : 0, sc.period_jit, sc.lat_jit, sc.stamp_jit,
                   sc_run.est_px.rms(), sc_run.held_px.rms(),
                   sc_run.los_h.rms(), sc_run.los_a.rms(), sc_run.range.rms(),
                   sc_run.range.mean(), sc_run.range.peak, sc_run.size.rms(),
                   sc_run.speed.rms(), sc_run.updates, sc_run.rejects,
                   sc_run.steps, sc_run.ts_path.c_str());
      scores.push_back(std::move(sc_run));
    }

    struct Pair {
      EstimatorType bbox;
      EstimatorType los;
    };
    const Pair pairs[] = {
        {EstimatorType::Ekf, EstimatorType::EkfLos},
        {EstimatorType::Ukf, EstimatorType::UkfLos},
        {EstimatorType::ImmEkf, EstimatorType::ImmEkfLos},
        {EstimatorType::ImmUkf, EstimatorType::ImmUkfLos},
    };
    for (const Pair& p : pairs) {
      const RunScore* b = find_score(scores, p.bbox);
      const RunScore* l = find_score(scores, p.los);
      if (!b || !l) continue;
      std::fprintf(cmp_f,
                   "%s,%s,%s,%s,%.2f,%.3f,%d,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f\n",
                   sc.id, sc.name, family_name(p.bbox),
                   sc.man == TargetManeuver::Jink ? "jink" : "smooth",
                   sc.detect_hz, sc.latency_s, sc.jitter ? 1 : 0,
                   b->est_px.rms(), l->est_px.rms(),
                   l->est_px.rms() - b->est_px.rms(), b->los_h.rms(),
                   l->los_h.rms(), l->los_h.rms() - b->los_h.rms(),
                   b->los_a.rms(), l->los_a.rms(),
                   l->los_a.rms() - b->los_a.rms(), b->range.rms(),
                   l->range.rms(), l->range.rms() - b->range.rms(),
                   b->size.rms(), l->size.rms());
    }
    std::fflush(sc_f);
    std::fflush(sum_f);
    std::fflush(cmp_f);
    std::fflush(chosen_f);
  }

  std::fclose(sc_f);
  std::fclose(sum_f);
  std::fclose(cmp_f);
  std::fclose(chosen_f);

  std::printf("auto done. CSVs in %s\n", root.c_str());
  std::printf(
      "  scenarios.csv  summary.csv  compare.csv  chosen_ekf.csv  tapes/  runs/\n");
  return 0;
}
