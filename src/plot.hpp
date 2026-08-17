#pragma once

#include "gfx.hpp"
#include "sim.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Second window: scrolling time series of LOS attack angle and heading for each
// signal path (origin, delayed, jittered+delayed, delay-removed, predicted),
// the error of each path against the origin, and estimated vs true range.
class PlotWindow {
 public:
  static constexpr int kSeries = 5;
  static constexpr int kErrSeries = kSeries - 1;  // every path except origin
  static constexpr int kRangeSeries = 2;          // true range, filter range
  static constexpr int kCap = 800;                // 8 s of history at 100 Hz

  struct SeriesStyle {
    const char* name;
    uint32_t color;
  };

  PlotWindow(int w = 900, int h = 940);
  ~PlotWindow();

  PlotWindow(const PlotWindow&) = delete;
  PlotWindow& operator=(const PlotWindow&) = delete;

  bool create_window(const char* title);
  bool alive() const { return hwnd_ != nullptr; }

  // Call once per sim step. cfg is needed because scoring the +H trace requires
  // knowing the horizon it was produced with.
  void push(const SimSnapshot& snap, const SimConfig& cfg);
  void draw(const SimConfig& cfg);
  void present();

  void on_destroyed() { hwnd_ = nullptr; }
  void blit_to(void* hdc) const;
  void toggle_series(int i) {
    if (i >= 0 && i < kSeries) show_[i] = !show_[i];
  }
  void nudge_latency(int steps) { latency_steps_ += steps; }

  // Drained by the main loop, so the delay can be set while watching the plots
  // instead of only from the sim window.
  int take_latency_steps() {
    const int s = latency_steps_;
    latency_steps_ = 0;
    return s;
  }

 private:
  struct TextItem {
    int x = 0, y = 0;
    uint32_t color = kColText;
    std::string text;
  };

  Surface surface() { return Surface{pixels_.data(), win_w_, win_h_}; }
  void draw_panel(int x, int y, int w, int h, const char* title,
                  const std::vector<float>& data,
                  const std::vector<uint8_t>& valid, int n_series,
                  const SeriesStyle* styles, const bool* show, const char* unit,
                  bool error_panel = false);
  void add_text(int x, int y, uint32_t color, const char* fmt, ...);

  int win_w_;
  int win_h_;
  std::vector<uint32_t> pixels_;
  std::vector<TextItem> texts_;

  // Ring buffers: [series][sample]
  std::vector<float> heading_;
  std::vector<float> attack_;
  std::vector<uint8_t> valid_;
  std::vector<float> err_heading_;
  std::vector<float> err_attack_;
  std::vector<uint8_t> err_valid_;
  std::vector<float> range_;
  std::vector<uint8_t> range_valid_;

  // The +H trace describes t+H, so it can only be scored once the origin at
  // t+H arrives. Its raw angles are kept so that sample can be filled in later.
  std::vector<float> pred_raw_heading_;
  std::vector<float> pred_raw_attack_;
  std::vector<uint8_t> pred_raw_valid_;

  int write_ = 0;
  long long total_ = 0;

  // Heading is unwrapped as it arrives so the trace doesn't jump at +/-180.
  float prev_heading_[kSeries]{};
  bool have_prev_[kSeries]{};

  float sample_dt_ = 0.01f;
  bool show_[kSeries] = {true, true, true, true, true};
  bool show_range_[kRangeSeries] = {true, true};
  int latency_steps_ = 0;

  void* hwnd_ = nullptr;
  void* hdc_mem_ = nullptr;
  void* hbmp_ = nullptr;
  void* old_bmp_ = nullptr;
};
