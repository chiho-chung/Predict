#pragma once

#include "sim/types.hpp"

#include <cstdio>
#include <string>

// One experiment = one timeseries CSV plus one row appended to
// logs/experiments.csv. Used by the live sim and by the headless checker
// so every A/B lands in the same index.
class ExperimentLog {
 public:
  ExperimentLog() = default;
  ~ExperimentLog();

  ExperimentLog(const ExperimentLog&) = delete;
  ExperimentLog& operator=(const ExperimentLog&) = delete;

  // kind: "live" or "check". label is a short tag for the filename
  // (estimator + target + any extra).
  bool begin(const char* kind, const char* label, const SimConfig& cfg);
  // Write the timeseries to an explicit path. If write_index is false the
  // row is not appended to logs/experiments.csv (auto-bench uses its own
  // summary files).
  bool begin_file(const char* path, const char* kind, const char* label,
                  const SimConfig& cfg, bool write_index);
  void sample(const SimConfig& cfg, const SimSnapshot& snap);
  void end();  // flush summary; safe to call twice

  bool active() const { return ts_ != nullptr; }
  const std::string& timeseries_path() const { return ts_path_; }

 private:
  struct Acc {
    double sum_sq = 0;
    double sum = 0;
    double peak = 0;
    int n = 0;
    void add(double v);
    double rms() const;
    double mean() const;
  };

  void write_ts_header();
  void append_summary();
  static bool ensure_logs_dir();
  static std::string stamp();
  static float wrap180(float d);

  FILE* ts_ = nullptr;
  std::string ts_path_;
  std::string kind_;
  std::string label_;
  bool write_index_ = true;
  SimConfig cfg_{};
  Acc est_px_, held_px_, track_px_;
  Acc los_h_, los_a_;
  Acc range_, size_, speed_;
  int steps_ = 0;
  int rejects_end_ = 0;
};
