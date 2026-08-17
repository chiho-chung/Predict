#include "io/csv_log.hpp"

#include "estimator/predictor.hpp"

#include <cerrno>
#include <cmath>
#include <ctime>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

namespace {

const char* maneuver_name(TargetManeuver m) {
  return m == TargetManeuver::Jink ? "jink" : "smooth";
}

void sanitize(char* s) {
  for (; *s; ++s) {
    if (*s == ' ' || *s == '/' || *s == '\\' || *s == ',' || *s == ':') *s = '_';
  }
}

}  // namespace

void ExperimentLog::Acc::add(double v) {
  sum_sq += v * v;
  sum += v;
  if (std::fabs(v) > peak) peak = std::fabs(v);
  ++n;
}

double ExperimentLog::Acc::rms() const {
  return n ? std::sqrt(sum_sq / n) : 0.0;
}

double ExperimentLog::Acc::mean() const { return n ? sum / n : 0.0; }

float ExperimentLog::wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

bool ExperimentLog::ensure_logs_dir() {
#ifdef _WIN32
  return _mkdir("logs") == 0 || errno == EEXIST;
#else
  return mkdir("logs", 0755) == 0 || errno == EEXIST;
#endif
}

std::string ExperimentLog::stamp() {
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

ExperimentLog::~ExperimentLog() { end(); }

bool ExperimentLog::begin(const char* kind, const char* label,
                          const SimConfig& cfg) {
  end();
  if (!ensure_logs_dir()) return false;

  char safe[128];
  std::snprintf(safe, sizeof(safe), "%s", label ? label : "exp");
  sanitize(safe);
  const std::string path = "logs/" + stamp() + "_" + (kind ? kind : "run") +
                           "_" + safe + ".csv";
  return begin_file(path.c_str(), kind, label, cfg, true);
}

bool ExperimentLog::begin_file(const char* path, const char* kind,
                               const char* label, const SimConfig& cfg,
                               bool write_index) {
  end();

  kind_ = kind ? kind : "run";
  label_ = label ? label : "exp";
  write_index_ = write_index;
  cfg_ = cfg;
  est_px_ = held_px_ = track_px_ = {};
  los_h_ = los_a_ = {};
  range_ = size_ = speed_ = {};
  steps_ = 0;
  rejects_end_ = 0;

  ts_path_ = path ? path : "";
  if (ts_path_.empty()) return false;
  ts_ = std::fopen(ts_path_.c_str(), "w");
  if (!ts_) {
    ts_path_.clear();
    return false;
  }
  write_ts_header();
  return true;
}

void ExperimentLog::write_ts_header() {
  std::fprintf(
      ts_,
      "t_s,estimator,maneuver,horizon_s,detect_hz,latency_s,jitter,"
      "meas_corr,predict_on,"
      "true_range_m,est_range_m,range_err_m,range_sigma_m,"
      "est_size_w_m,est_size_h_m,size_err_m,"
      "true_speed_mps,est_speed_mps,"
      "held_err_px,est_err_px,track_err_px,"
      "origin_hdg,origin_att,delayed_hdg,delayed_att,"
      "jittered_hdg,jittered_att,est_hdg,est_att,pred_hdg,pred_att,"
      "err_delayed_hdg,err_delayed_att,err_jittered_hdg,err_jittered_att,"
      "err_est_hdg,err_est_att,"
      "detect_count,detect_age_s,tracker_updates,tracker_rejects,"
      "imm_p0,imm_p1,imm_p2,"
      "chaser_vx,chaser_vy,chaser_vz,target_vx,target_vy,target_vz,"
      "timing_on,period_s,pkt_latency_s,meas_stamp_s,stamp_err_s,"
      "zoom,fov_deg,fx\n");
}

void ExperimentLog::sample(const SimConfig& cfg, const SimSnapshot& snap) {
  if (!ts_) return;
  cfg_ = cfg;
  ++steps_;
  rejects_end_ = snap.tracker_rejects;

  if (snap.detection.visible && snap.detection_gt.visible) {
    est_px_.add(snap.est_err_px);
    held_px_.add(snap.held_err_px);
    track_px_.add(snap.track_err_px);
  }
  if (snap.los.origin.valid && snap.los.estimate.valid) {
    los_h_.add(wrap180(snap.los.estimate.heading_deg - snap.los.origin.heading_deg));
    los_a_.add(wrap180(snap.los.estimate.attack_deg - snap.los.origin.attack_deg));
  }
  if (snap.track_now.valid && snap.track_now.range_m > 0) {
    range_.add(snap.track_now.range_m - snap.true_range_m);
    size_.add(snap.size_err_m);
    speed_.add(snap.track_now.speed_mps - snap.target.vel.length());
  }

  auto ang = [](const LosAngles& a, bool hdg) {
    return a.valid ? (hdg ? a.heading_deg : a.attack_deg) : NAN;
  };
  auto err_h = [&](const LosAngles& a) {
    return (a.valid && snap.los.origin.valid)
               ? wrap180(a.heading_deg - snap.los.origin.heading_deg)
               : NAN;
  };
  auto err_a = [&](const LosAngles& a) {
    return (a.valid && snap.los.origin.valid)
               ? (a.attack_deg - snap.los.origin.attack_deg)
               : NAN;
  };

  const bool has_range = snap.track_now.valid && snap.track_now.range_m > 0;
  std::fprintf(
      ts_,
      "%.4f,%s,%s,%.3f,%.2f,%.3f,%d,%.3f,%d,"
      "%.4f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,"
      "%.4f,%.4f,"
      "%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,"
      "%d,%.4f,%d,%d,"
      "%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
      "%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.2f\n",
      snap.time, estimator_name(cfg.tracker.type),
      maneuver_name(cfg.target.maneuver), cfg.predict.horizon_s,
      cfg.rates.detect_hz, cfg.rates.detect_latency_s,
      cfg.jitter.enabled ? 1 : 0, cfg.tracker.meas_corr,
      cfg.predict.enabled ? 1 : 0, snap.true_range_m,
      has_range ? snap.track_now.range_m : NAN,
      has_range ? (snap.track_now.range_m - snap.true_range_m) : NAN,
      has_range ? snap.track_now.range_sigma_m : NAN,
      has_range ? snap.track_now.size_w_m : NAN,
      has_range ? snap.track_now.size_h_m : NAN,
      has_range ? snap.size_err_m : NAN, snap.target.vel.length(),
      has_range ? snap.track_now.speed_mps : NAN, snap.held_err_px,
      snap.est_err_px, snap.track_err_px, ang(snap.los.origin, true),
      ang(snap.los.origin, false), ang(snap.los.delayed, true),
      ang(snap.los.delayed, false), ang(snap.los.jittered, true),
      ang(snap.los.jittered, false), ang(snap.los.estimate, true),
      ang(snap.los.estimate, false), ang(snap.los.predicted, true),
      ang(snap.los.predicted, false), err_h(snap.los.delayed),
      err_a(snap.los.delayed), err_h(snap.los.jittered),
      err_a(snap.los.jittered), err_h(snap.los.estimate),
      err_a(snap.los.estimate), snap.detect_count, snap.detection_age,
      snap.tracker_updates, snap.tracker_rejects,
      snap.track_now.model_prob[0], snap.track_now.model_prob[1],
      snap.track_now.model_prob[2], snap.drone.vel.x, snap.drone.vel.y,
      snap.drone.vel.z, snap.target.vel.x, snap.target.vel.y,
      snap.target.vel.z, cfg.timing.enabled ? 1 : 0, snap.last_period_s,
      snap.last_latency_s, snap.meas_stamp_s, snap.stamp_err_s, snap.zoom,
      snap.fov_deg, snap.fx);
}

void ExperimentLog::append_summary() {
  if (ts_path_.empty() || !write_index_) return;
  FILE* idx = std::fopen("logs/experiments.csv", "a+");
  if (!idx) return;

  // Write the header once if the file is empty.
  std::fseek(idx, 0, SEEK_END);
  if (std::ftell(idx) == 0) {
    std::fprintf(
        idx,
        "stamp,kind,label,estimator,meas,maneuver,horizon_s,detect_hz,latency_s,"
        "jitter,meas_corr,sigma_accel,predict_on,steps,duration_s,"
        "est_px_rms,held_px_rms,track_px_rms,los_head_rms,los_att_rms,"
        "range_rms,range_bias,range_peak,size_rms,speed_rms,rejects,"
        "timing_on,timeseries\n");
  }

  const char* meas = !estimator_uses_filter(cfg_.tracker.type)
                         ? "px"
                         : (estimator_uses_bbox(cfg_.tracker.type) ? "bbox"
                                                                  : "los");
  std::fprintf(
      idx,
      "%s,%s,%s,%s,%s,%s,%.3f,%.2f,%.3f,%d,%.3f,%.2f,%d,%d,%.3f,"
      "%.4f,%.4f,%.4f,%.4f,%.4f,"
      "%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,"
      "%s\n",
      stamp().c_str(), kind_.c_str(), label_.c_str(),
      estimator_name(cfg_.tracker.type), meas,
      maneuver_name(cfg_.target.maneuver),
      cfg_.predict.horizon_s, cfg_.rates.detect_hz, cfg_.rates.detect_latency_s,
      cfg_.jitter.enabled ? 1 : 0, cfg_.tracker.meas_corr,
      cfg_.tracker.sigma_accel, cfg_.predict.enabled ? 1 : 0, steps_,
      steps_ * cfg_.sim_dt(), est_px_.rms(), held_px_.rms(), track_px_.rms(),
      los_h_.rms(), los_a_.rms(), range_.rms(), range_.mean(), range_.peak,
      size_.rms(), speed_.rms(), rejects_end_,
      cfg_.timing.enabled ? 1 : 0, ts_path_.c_str());
  std::fclose(idx);
}

void ExperimentLog::end() {
  if (!ts_) return;
  std::fclose(ts_);
  ts_ = nullptr;
  append_summary();
}
