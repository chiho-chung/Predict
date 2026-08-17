#pragma once

#include "ui/gfx.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Third window: browse logs/experiments.csv, inspect RMS, load a run's
// timeseries and review heading / attack / range error traces.
class HistoryWindow {
 public:
  HistoryWindow(int w = 980, int h = 640);
  ~HistoryWindow();

  HistoryWindow(const HistoryWindow&) = delete;
  HistoryWindow& operator=(const HistoryWindow&) = delete;

  bool create_window(const char* title);
  bool alive() const { return hwnd_ != nullptr; }
  void show();  // create if closed, otherwise raise

  void reload();  // re-read experiments.csv
  void draw();
  void present();

  void on_destroyed() { hwnd_ = nullptr; }
  void blit_to(void* hdc) const;

  void move_sel(int delta);
  void load_selected();

 private:
  struct ExpRow {
    std::string stamp;
    std::string kind;
    std::string label;
    std::string estimator;
    std::string meas;
    std::string maneuver;
    std::string timeseries;
    float horizon_s = 0;
    float detect_hz = 0;
    float latency_s = 0;
    int jitter = 0;
    int timing_on = 0;
    float duration_s = 0;
    float est_px_rms = 0;
    float los_head_rms = 0;
    float los_att_rms = 0;
    float range_rms = 0;
    float size_rms = 0;
    int rejects = 0;
    int steps = 0;
  };

  struct TextItem {
    int x = 0, y = 0;
    uint32_t color = kColText;
    std::string text;
  };

  Surface surface() { return Surface{pixels_.data(), win_w_, win_h_}; }
  void add_text(int x, int y, uint32_t color, const char* fmt, ...);
  void draw_spark(int x, int y, int w, int h, const std::vector<float>& yv,
                  uint32_t color, const char* title, const char* unit);

  int win_w_ = 980;
  int win_h_ = 640;
  std::vector<uint32_t> pixels_;
  std::vector<TextItem> texts_;

  std::vector<ExpRow> rows_;
  int sel_ = 0;
  int scroll_ = 0;

  std::vector<float> tr_t_;
  std::vector<float> tr_hdg_;
  std::vector<float> tr_att_;
  std::vector<float> tr_rng_;
  std::string loaded_path_;
  bool loaded_ok_ = false;
  std::string status_;

  void* hwnd_ = nullptr;
  void* hdc_mem_ = nullptr;
  void* hbmp_ = nullptr;
  void* old_bmp_ = nullptr;
};
