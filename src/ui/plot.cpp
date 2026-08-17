#include "ui/plot.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {

using SeriesStyle = PlotWindow::SeriesStyle;

const SeriesStyle kStyles[PlotWindow::kSeries] = {
    {"origin (true)", kColTruth},
    {"delayed", kColDelayed},
    {"jittered+delayed", kColMeas},
    {"delay removed", kColEst},
    {"predicted +H", kColPred},
};

const SeriesStyle kRangeStyles[PlotWindow::kRangeSeries] = {
    {"true range", kColTruth},
    {"filter range", kColEst},
};

// Angles: a difference must be wrapped or a crossing of the +/-180 branch cut
// shows up as a 360 deg spike.
float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

constexpr int kMarginL = 56;
constexpr int kMarginR = 12;
constexpr int kTitleH = 22;
constexpr int kLegendH = 70;

LRESULT CALLBACK PlotProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (msg == WM_NCCREATE) {
    auto* cs = reinterpret_cast<CREATESTRUCTW*>(lp);
    SetWindowLongPtrW(hwnd, GWLP_USERDATA,
                      reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    return TRUE;
  }
  auto* self =
      reinterpret_cast<PlotWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
  switch (msg) {
    case WM_KEYDOWN:
      if (self && wp >= VK_F1 && wp <= VK_F1 + PlotWindow::kSeries - 1) {
        self->toggle_series(static_cast<int>(wp - VK_F1));
      }
      // Same keys as the sim window, so the delay can be set from whichever
      // window happens to have focus.
      if (self && wp == '7') self->nudge_latency(-1);
      if (self && wp == '8') self->nudge_latency(+1);
      return 0;
    case WM_ERASEBKGND:
      return 1;
    case WM_PAINT: {
      PAINTSTRUCT ps;
      HDC hdc = BeginPaint(hwnd, &ps);
      if (self) self->blit_to(hdc);
      EndPaint(hwnd, &ps);
      return 0;
    }
    case WM_CLOSE:
      // Closing the plot must not take the whole app down.
      DestroyWindow(hwnd);
      return 0;
    case WM_DESTROY:
      if (self) self->on_destroyed();
      return 0;
    default:
      return DefWindowProcW(hwnd, msg, wp, lp);
  }
}

}  // namespace

PlotWindow::PlotWindow(int w, int h) : win_w_(w), win_h_(h) {
  pixels_.assign(static_cast<size_t>(win_w_) * static_cast<size_t>(win_h_), 0);
  heading_.assign(static_cast<size_t>(kSeries) * kCap, 0.0f);
  attack_.assign(static_cast<size_t>(kSeries) * kCap, 0.0f);
  valid_.assign(static_cast<size_t>(kSeries) * kCap, 0);
  err_heading_.assign(static_cast<size_t>(kErrSeries) * kCap, 0.0f);
  err_attack_.assign(static_cast<size_t>(kErrSeries) * kCap, 0.0f);
  err_valid_.assign(static_cast<size_t>(kErrSeries) * kCap, 0);
  range_.assign(static_cast<size_t>(kRangeSeries) * kCap, 0.0f);
  range_valid_.assign(static_cast<size_t>(kRangeSeries) * kCap, 0);
  pred_raw_heading_.assign(kCap, 0.0f);
  pred_raw_attack_.assign(kCap, 0.0f);
  pred_raw_valid_.assign(kCap, 0);
}

PlotWindow::~PlotWindow() {
  if (hdc_mem_) {
    HDC hdc = static_cast<HDC>(hdc_mem_);
    if (old_bmp_) SelectObject(hdc, static_cast<HBITMAP>(old_bmp_));
    if (hbmp_) DeleteObject(static_cast<HBITMAP>(hbmp_));
    DeleteDC(hdc);
  }
  if (hwnd_) DestroyWindow(static_cast<HWND>(hwnd_));
}

bool PlotWindow::create_window(const char* title) {
  HINSTANCE hi = GetModuleHandleW(nullptr);
  WNDCLASSW wc{};
  wc.style = CS_OWNDC;
  wc.lpfnWndProc = PlotProc;
  wc.hInstance = hi;
  wc.lpszClassName = L"DroneChasePlotWnd";
  wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
  wc.hbrBackground = nullptr;
  RegisterClassW(&wc);

  wchar_t wtitle[256];
  MultiByteToWideChar(CP_UTF8, 0, title, -1, wtitle, 256);

  constexpr DWORD kStyle = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU |
                           WS_MINIMIZEBOX | WS_VISIBLE;
  RECT rc{0, 0, win_w_, win_h_};
  AdjustWindowRect(&rc, kStyle, FALSE);
  HWND hwnd = CreateWindowW(wc.lpszClassName, wtitle, kStyle, CW_USEDEFAULT,
                            CW_USEDEFAULT, rc.right - rc.left,
                            rc.bottom - rc.top, nullptr, nullptr, hi, this);
  if (!hwnd) return false;
  hwnd_ = hwnd;

  HDC hdc_screen = GetDC(hwnd);
  HDC hdc_mem = CreateCompatibleDC(hdc_screen);
  BITMAPINFO bmi{};
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = win_w_;
  bmi.bmiHeader.biHeight = -win_h_;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 32;
  bmi.bmiHeader.biCompression = BI_RGB;
  void* bits = nullptr;
  HBITMAP hbmp =
      CreateDIBSection(hdc_mem, &bmi, DIB_RGB_COLORS, &bits, nullptr, 0);
  ReleaseDC(hwnd, hdc_screen);
  if (!hbmp) return false;

  old_bmp_ = SelectObject(hdc_mem, hbmp);
  hdc_mem_ = hdc_mem;
  hbmp_ = hbmp;
  return true;
}

void PlotWindow::push(const SimSnapshot& snap, const SimConfig& cfg) {
  sample_dt_ = cfg.sim_dt();

  const LosAngles* src[kSeries] = {&snap.los.origin, &snap.los.delayed,
                                   &snap.los.jittered, &snap.los.estimate,
                                   &snap.los.predicted};

  for (int s = 0; s < kSeries; ++s) {
    const size_t i = static_cast<size_t>(s) * kCap + static_cast<size_t>(write_);
    const LosAngles& a = *src[s];
    if (!a.valid) {
      valid_[i] = 0;
      have_prev_[s] = false;
      continue;
    }

    float hd = a.heading_deg;
    if (have_prev_[s]) {
      while (hd - prev_heading_[s] > 180.0f) hd -= 360.0f;
      while (hd - prev_heading_[s] < -180.0f) hd += 360.0f;
    }
    prev_heading_[s] = hd;
    have_prev_[s] = true;

    heading_[i] = hd;
    attack_[i] = a.attack_deg;
    valid_[i] = 1;
  }

  // Error against origin. Computed from the raw angles with a wrapped
  // difference, not from the unwrapped traces, because two series can pick up
  // different 360 deg offsets while unwrapping independently.
  for (int e = 0; e < kErrSeries; ++e) {
    err_valid_[static_cast<size_t>(e) * kCap + static_cast<size_t>(write_)] = 0;
  }
  const LosAngles& origin = snap.los.origin;
  if (origin.valid) {
    // delayed, jittered+delayed and delay-removed all describe "now", so they
    // are scored against the origin of this same sample.
    for (int e = 0; e < kErrSeries - 1; ++e) {
      const LosAngles& a = *src[e + 1];
      if (!a.valid) continue;
      const size_t i =
          static_cast<size_t>(e) * kCap + static_cast<size_t>(write_);
      err_heading_[i] = wrap180(a.heading_deg - origin.heading_deg);
      err_attack_[i] = a.attack_deg - origin.attack_deg;
      err_valid_[i] = 1;
    }

    // The +H trace claims to describe t+H, so scoring it against the origin of
    // its own sample would only measure the intended lead. Instead the origin
    // that has just arrived retires the sample from H ago.
    const int lead =
        static_cast<int>(cfg.predict.horizon_s / sample_dt_ + 0.5f);
    if (lead >= 0 && lead < kCap) {
      const int j = ((write_ - lead) % kCap + kCap) % kCap;
      if (pred_raw_valid_[static_cast<size_t>(j)]) {
        const size_t i = static_cast<size_t>(kErrSeries - 1) * kCap +
                         static_cast<size_t>(j);
        err_heading_[i] = wrap180(pred_raw_heading_[static_cast<size_t>(j)] -
                                  origin.heading_deg);
        err_attack_[i] = wrap180(
            pred_raw_attack_[static_cast<size_t>(j)] - origin.attack_deg);
        err_valid_[i] = 1;
      }
    }
  }

  pred_raw_valid_[static_cast<size_t>(write_)] =
      snap.los.predicted.valid ? 1 : 0;
  pred_raw_heading_[static_cast<size_t>(write_)] =
      snap.los.predicted.heading_deg;
  pred_raw_attack_[static_cast<size_t>(write_)] = snap.los.predicted.attack_deg;

  const size_t ri = static_cast<size_t>(write_);
  range_[ri] = snap.true_range_m;
  range_valid_[ri] = snap.true_range_m > 0 ? 1 : 0;
  const size_t ri2 = static_cast<size_t>(kCap) + ri;
  // CV-pixel reports no range at all, so leave the trace broken rather than
  // drawing a zero.
  const bool has_est = snap.track_now.valid && snap.track_now.range_m > 0;
  range_[ri2] = has_est ? snap.track_now.range_m : 0.0f;
  range_valid_[ri2] = has_est ? 1 : 0;

  write_ = (write_ + 1) % kCap;
  ++total_;
}

void PlotWindow::add_text(int x, int y, uint32_t color, const char* fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  texts_.push_back(TextItem{x, y, color, buf});
}

void PlotWindow::draw_panel(int x, int y, int w, int h, const char* title,
                            const std::vector<float>& data,
                            const std::vector<uint8_t>& valid, int n_series,
                            const SeriesStyle* styles, const bool* show,
                            const char* unit, bool error_panel) {
  const Surface s = surface();
  const int plot_x = x + kMarginL;
  const int plot_y = y + kTitleH;
  const int plot_w = w - kMarginL - kMarginR;
  const int plot_h = h - kTitleH;
  if (plot_w < 20 || plot_h < 20) return;

  surf_rect(s, plot_x, plot_y, plot_x + plot_w, plot_y + plot_h, rgb(12, 12, 15),
            true);

  const int count = static_cast<int>((std::min<long long>)(total_, kCap));
  const int start = (total_ < kCap) ? 0 : write_;

  // Autoscale over the visible series only, so hiding the wide-swinging
  // predicted trace lets the others fill the panel.
  float lo = 1e9f, hi = -1e9f;
  for (int sIdx = 0; sIdx < n_series; ++sIdx) {
    if (!show[sIdx]) continue;
    for (int i = 0; i < count; ++i) {
      const size_t idx = static_cast<size_t>(sIdx) * kCap +
                         static_cast<size_t>((start + i) % kCap);
      if (!valid[idx]) continue;
      lo = (std::min)(lo, data[idx]);
      hi = (std::max)(hi, data[idx]);
    }
  }
  if (lo > hi) {
    lo = -1.0f;
    hi = 1.0f;
  }
  // An error panel is only readable against zero, so keep it in view.
  if (error_panel) {
    lo = (std::min)(lo, 0.0f);
    hi = (std::max)(hi, 0.0f);
  }
  float pad = (hi - lo) * 0.12f;
  if (pad < 0.05f) pad = 0.05f;
  lo -= pad;
  hi += pad;

  auto to_y = [&](float v) {
    const float t = (v - lo) / (hi - lo);
    return plot_y + plot_h - 1 - static_cast<int>(t * (plot_h - 2));
  };
  auto to_x = [&](int i) {
    if (count <= 1) return plot_x;
    return plot_x + static_cast<int>(static_cast<float>(i) /
                                     static_cast<float>(count - 1) *
                                     static_cast<float>(plot_w - 1));
  };

  // Horizontal grid + value labels
  constexpr int kTicks = 5;
  for (int t = 0; t < kTicks; ++t) {
    const float v = lo + (hi - lo) * static_cast<float>(t) / (kTicks - 1);
    const int gy = to_y(v);
    surf_line(s, plot_x, gy, plot_x + plot_w - 1, gy, kColGrid);
    add_text(x + 6, gy - 8, rgb(150, 150, 160), "%7.2f", v);
  }
  if (error_panel && lo < 0.0f && hi > 0.0f) {
    const int zy = to_y(0.0f);
    surf_line(s, plot_x, zy, plot_x + plot_w - 1, zy, rgb(120, 120, 135));
  }
  surf_rect(s, plot_x, plot_y, plot_x + plot_w, plot_y + plot_h,
            rgb(80, 80, 90), false);

  // Reverse order so the reference series ends up drawn on top of the rest.
  for (int sIdx = n_series - 1; sIdx >= 0; --sIdx) {
    if (!show[sIdx]) continue;
    int prev_x = 0, prev_y = 0;
    bool has_prev = false;
    for (int i = 0; i < count; ++i) {
      const size_t idx = static_cast<size_t>(sIdx) * kCap +
                         static_cast<size_t>((start + i) % kCap);
      if (!valid[idx]) {
        has_prev = false;
        continue;
      }
      const int cx = to_x(i);
      const int cy = to_y(data[idx]);
      if (has_prev) {
        surf_line(s, prev_x, prev_y, cx, cy, styles[sIdx].color);
      }
      prev_x = cx;
      prev_y = cy;
      has_prev = true;
    }
  }

  // Time axis: oldest sample on the left, current on the right.
  const float span_s = static_cast<float>(count) * sample_dt_;
  add_text(plot_x + 4, plot_y + plot_h - 16, rgb(130, 130, 140), "-%.1f s",
           span_s);
  add_text(plot_x + plot_w - 32, plot_y + plot_h - 16, rgb(130, 130, 140),
           "now");

  add_text(x + 6, y + 3, kColText, "%s  [%s]", title, unit);

  // On an error panel the shape alone is hard to compare, so put the RMS of
  // each visible trace over the window on the title row, in its own colour.
  if (error_panel) {
    int lx = x + 300;
    add_text(lx, y + 3, rgb(150, 150, 160), "rms");
    lx += 32;
    for (int sIdx = 0; sIdx < n_series; ++sIdx) {
      if (!show[sIdx]) continue;
      double sum = 0;
      int n = 0;
      for (int i = 0; i < count; ++i) {
        const size_t idx = static_cast<size_t>(sIdx) * kCap +
                           static_cast<size_t>((start + i) % kCap);
        if (!valid[idx]) continue;
        sum += static_cast<double>(data[idx]) * data[idx];
        ++n;
      }
      if (n == 0) continue;
      add_text(lx, y + 3, styles[sIdx].color, "%.2f", std::sqrt(sum / n));
      lx += 48;
    }
  }
}

void PlotWindow::draw(const SimConfig& cfg) {
  if (!hwnd_) return;

  const Surface s = surface();
  texts_.clear();
  surf_fill(s, kColPanel);

  sample_dt_ = cfg.sim_dt();
  const int count = static_cast<int>((std::min<long long>)(total_, kCap));
  const float span_s = static_cast<float>(count) * sample_dt_;

  // Error panels reuse the parent series' styles and visibility flags, offset by
  // one, so F2-F5 hide a trace and its error together.
  const int body_h = win_h_ - kLegendH;
  const int panel_h = body_h / 5;
  draw_panel(0, 0, win_w_, panel_h, "LOS attack angle (elevation)", attack_,
             valid_, kSeries, kStyles, show_, "deg");
  draw_panel(0, panel_h, win_w_, panel_h, "attack error vs origin", err_attack_,
             err_valid_, kErrSeries, kStyles + 1, show_ + 1, "deg", true);
  draw_panel(0, 2 * panel_h, win_w_, panel_h,
             "LOS heading (azimuth, unwrapped)", heading_, valid_, kSeries,
             kStyles, show_, "deg");
  draw_panel(0, 3 * panel_h, win_w_, panel_h, "heading error vs origin",
             err_heading_, err_valid_, kErrSeries, kStyles + 1, show_ + 1, "deg",
             true);
  draw_panel(0, 4 * panel_h, win_w_, body_h - 4 * panel_h,
             "range: filter estimate vs truth", range_, range_valid_,
             kRangeSeries, kRangeStyles, show_range_, "m");

  // Legend + context. F1..F5 hide a trace, which also drops it from autoscale.
  const int ly = win_h_ - kLegendH + 4;
  int lx = 10;
  for (int i = 0; i < kSeries; ++i) {
    const uint32_t col = show_[i] ? kStyles[i].color : rgb(90, 90, 96);
    surf_rect(s, lx, ly + 4, lx + 12, ly + 12, col, true);
    if (!show_[i]) {
      surf_line(s, lx, ly + 12, lx + 12, ly + 4, rgb(200, 60, 60));
    }
    add_text(lx + 18, ly + 2, col, "F%d %s", i + 1, kStyles[i].name);
    lx += 18 + 8 * static_cast<int>(std::strlen(kStyles[i].name) + 3) + 10;
  }
  add_text(10, ly + 20, kColText,
           "%s   window %.1f s   detect %.0f Hz   delay %.0f ms (7/8)   "
           "H=%.2f s (+/-)   F1-F5 toggle",
           estimator_name(cfg.tracker.type), span_s, cfg.rates.detect_hz,
           cfg.rates.detect_latency_s * 1000.0f, cfg.predict.horizon_s);
  add_text(10, ly + 36, rgb(150, 150, 160),
           "+H error is scored against the origin at t+H, so its trace ends "
           "%.2f s before now", cfg.predict.horizon_s);
}

void PlotWindow::blit_to(void* hdc) const {
  if (!hdc || !hdc_mem_) return;
  BitBlt(static_cast<HDC>(hdc), 0, 0, win_w_, win_h_,
         static_cast<HDC>(hdc_mem_), 0, 0, SRCCOPY);
}

void PlotWindow::present() {
  if (!hwnd_ || !hdc_mem_) return;

  HWND hwnd = static_cast<HWND>(hwnd_);
  HDC hdc_mem = static_cast<HDC>(hdc_mem_);
  HBITMAP hbmp = static_cast<HBITMAP>(hbmp_);

  BITMAPINFO bmi{};
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = win_w_;
  bmi.bmiHeader.biHeight = -win_h_;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 32;
  bmi.bmiHeader.biCompression = BI_RGB;
  SetDIBits(hdc_mem, hbmp, 0, win_h_, pixels_.data(), &bmi, DIB_RGB_COLORS);

  SetBkMode(hdc_mem, TRANSPARENT);
  for (const TextItem& t : texts_) {
    SetTextColor(hdc_mem,
                 RGB(red_of(t.color), green_of(t.color), blue_of(t.color)));
    TextOutA(hdc_mem, t.x, t.y, t.text.c_str(), static_cast<int>(t.text.size()));
  }

  HDC hdc = GetDC(hwnd);
  blit_to(hdc);
  ReleaseDC(hwnd, hdc);
}
