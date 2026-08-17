#include "ui/history.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>

namespace {

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else if (c != '\r' && c != '\n') {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

std::string cell(const std::vector<std::string>& cols,
                 const std::map<std::string, int>& idx, const char* name) {
  auto it = idx.find(name);
  if (it == idx.end() || it->second < 0 ||
      it->second >= static_cast<int>(cols.size())) {
    return "";
  }
  return cols[static_cast<size_t>(it->second)];
}

float cellf(const std::vector<std::string>& cols,
            const std::map<std::string, int>& idx, const char* name) {
  const std::string s = cell(cols, idx, name);
  if (s.empty()) return 0.0f;
  return static_cast<float>(std::atof(s.c_str()));
}

int celli(const std::vector<std::string>& cols,
          const std::map<std::string, int>& idx, const char* name) {
  return static_cast<int>(cellf(cols, idx, name));
}

LRESULT CALLBACK HistProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (msg == WM_NCCREATE) {
    auto* cs = reinterpret_cast<CREATESTRUCTW*>(lp);
    SetWindowLongPtrW(hwnd, GWLP_USERDATA,
                      reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    return TRUE;
  }
  auto* self =
      reinterpret_cast<HistoryWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
  switch (msg) {
    case WM_KEYDOWN:
      if (!self) return 0;
      if (wp == VK_UP) self->move_sel(-1);
      if (wp == VK_DOWN) self->move_sel(+1);
      if (wp == VK_PRIOR) self->move_sel(-12);
      if (wp == VK_NEXT) self->move_sel(+12);
      if (wp == VK_HOME) self->move_sel(-100000);
      if (wp == VK_END) self->move_sel(+100000);
      if (wp == VK_RETURN || wp == VK_SPACE) self->load_selected();
      if (wp == VK_F5 || wp == 'R') self->reload();
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

HistoryWindow::HistoryWindow(int w, int h) : win_w_(w), win_h_(h) {
  pixels_.assign(static_cast<size_t>(win_w_) * static_cast<size_t>(win_h_), 0);
}

HistoryWindow::~HistoryWindow() {
  if (hdc_mem_) {
    HDC hdc = static_cast<HDC>(hdc_mem_);
    if (old_bmp_) SelectObject(hdc, static_cast<HBITMAP>(old_bmp_));
    if (hbmp_) DeleteObject(static_cast<HBITMAP>(hbmp_));
    DeleteDC(hdc);
  }
  if (hwnd_) DestroyWindow(static_cast<HWND>(hwnd_));
}

bool HistoryWindow::create_window(const char* title) {
  HINSTANCE hi = GetModuleHandleW(nullptr);
  WNDCLASSW wc{};
  wc.style = CS_OWNDC;
  wc.lpfnWndProc = HistProc;
  wc.hInstance = hi;
  wc.lpszClassName = L"DroneChaseHistoryWnd";
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

  if (!hdc_mem_) {
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
  }
  reload();
  if (!rows_.empty()) load_selected();
  return true;
}

void HistoryWindow::show() {
  if (!alive()) {
    create_window("History — experiment index + trace review");
    return;
  }
  ShowWindow(static_cast<HWND>(hwnd_), SW_SHOW);
  SetForegroundWindow(static_cast<HWND>(hwnd_));
}

void HistoryWindow::add_text(int x, int y, uint32_t color, const char* fmt,
                             ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  texts_.push_back(TextItem{x, y, color, buf});
}

void HistoryWindow::reload() {
  rows_.clear();
  std::ifstream in("logs/experiments.csv");
  if (!in) {
    status_ = "no logs/experiments.csv  (run the sim first)";
    return;
  }
  std::string header;
  if (!std::getline(in, header)) {
    status_ = "experiments.csv is empty";
    return;
  }
  const auto names = split_csv(header);
  std::map<std::string, int> idx;
  for (int i = 0; i < static_cast<int>(names.size()); ++i) idx[names[i]] = i;

  std::string line;
  std::vector<ExpRow> chrono;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto cols = split_csv(line);
    ExpRow r;
    r.stamp = cell(cols, idx, "stamp");
    r.kind = cell(cols, idx, "kind");
    r.label = cell(cols, idx, "label");
    r.estimator = cell(cols, idx, "estimator");
    r.meas = cell(cols, idx, "meas");
    r.maneuver = cell(cols, idx, "maneuver");
    r.timeseries = cell(cols, idx, "timeseries");
    r.horizon_s = cellf(cols, idx, "horizon_s");
    r.detect_hz = cellf(cols, idx, "detect_hz");
    r.latency_s = cellf(cols, idx, "latency_s");
    r.jitter = celli(cols, idx, "jitter");
    r.timing_on = celli(cols, idx, "timing_on");
    r.duration_s = cellf(cols, idx, "duration_s");
    r.est_px_rms = cellf(cols, idx, "est_px_rms");
    r.los_head_rms = cellf(cols, idx, "los_head_rms");
    r.los_att_rms = cellf(cols, idx, "los_att_rms");
    r.range_rms = cellf(cols, idx, "range_rms");
    r.size_rms = cellf(cols, idx, "size_rms");
    r.rejects = celli(cols, idx, "rejects");
    r.steps = celli(cols, idx, "steps");
    chrono.push_back(std::move(r));
  }
  rows_.assign(chrono.rbegin(), chrono.rend());
  if (sel_ >= static_cast<int>(rows_.size())) sel_ = 0;
  status_ = rows_.empty() ? "index has no rows" : "Up/Down select   Enter load   F5 reload";
}

void HistoryWindow::move_sel(int delta) {
  if (rows_.empty()) return;
  sel_ += delta;
  if (sel_ < 0) sel_ = 0;
  if (sel_ >= static_cast<int>(rows_.size())) {
    sel_ = static_cast<int>(rows_.size()) - 1;
  }
}

void HistoryWindow::load_selected() {
  tr_t_.clear();
  tr_hdg_.clear();
  tr_att_.clear();
  tr_rng_.clear();
  loaded_ok_ = false;
  loaded_path_.clear();
  if (sel_ < 0 || sel_ >= static_cast<int>(rows_.size())) return;
  loaded_path_ = rows_[static_cast<size_t>(sel_)].timeseries;
  std::ifstream in(loaded_path_);
  if (!in) {
    status_ = "missing " + loaded_path_;
    return;
  }
  std::string header;
  if (!std::getline(in, header)) return;
  const auto names = split_csv(header);
  std::map<std::string, int> idx;
  for (int i = 0; i < static_cast<int>(names.size()); ++i) idx[names[i]] = i;

  std::vector<float> t, h, a, r;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto cols = split_csv(line);
    const float tt = cellf(cols, idx, "t_s");
    if (tt < 2.0f) continue;
    t.push_back(tt);
    h.push_back(cellf(cols, idx, "err_est_hdg"));
    a.push_back(cellf(cols, idx, "err_est_att"));
    r.push_back(cellf(cols, idx, "range_err_m"));
  }
  constexpr int kCap = 400;
  if (static_cast<int>(t.size()) > kCap && !t.empty()) {
    const int step = static_cast<int>(t.size()) / kCap;
    for (int i = 0; i < static_cast<int>(t.size()); i += step) {
      tr_t_.push_back(t[static_cast<size_t>(i)]);
      tr_hdg_.push_back(h[static_cast<size_t>(i)]);
      tr_att_.push_back(a[static_cast<size_t>(i)]);
      tr_rng_.push_back(r[static_cast<size_t>(i)]);
    }
  } else {
    tr_t_ = std::move(t);
    tr_hdg_ = std::move(h);
    tr_att_ = std::move(a);
    tr_rng_ = std::move(r);
  }
  loaded_ok_ = !tr_t_.empty();
  status_ = loaded_ok_ ? ("loaded " + loaded_path_) : ("no samples in " + loaded_path_);
}

void HistoryWindow::draw_spark(int x, int y, int w, int h,
                               const std::vector<float>& yv, uint32_t color,
                               const char* title, const char* unit) {
  Surface s = surface();
  surf_rect(s, x, y, x + w, y + h, rgb(22, 22, 28), true);
  surf_rect(s, x, y, x + w, y + h, rgb(70, 70, 82), false);
  add_text(x + 8, y + 4, color, "%s  (%s)", title, unit);
  if (yv.size() < 2) {
    add_text(x + 8, y + 24, rgb(140, 140, 150), "Enter to load timeseries");
    return;
  }
  float lo = yv[0], hi = yv[0];
  for (float v : yv) {
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  if (lo > 0) lo = 0;
  if (hi < 0) hi = 0;
  if (hi - lo < 1e-4f) {
    hi += 1;
    lo -= 1;
  }
  const int plot_y0 = y + 22;
  const int plot_h = h - 28;
  const int z =
      plot_y0 + static_cast<int>((hi - 0) / (hi - lo) * static_cast<float>(plot_h));
  surf_line(s, x + 4, z, x + w - 4, z, rgb(70, 70, 80));
  auto mapx = [&](int i) {
    return x + 6 +
           static_cast<int>(static_cast<float>(i) / (yv.size() - 1) *
                            static_cast<float>(w - 12));
  };
  auto mapy = [&](float v) {
    return plot_y0 +
           static_cast<int>((hi - v) / (hi - lo) * static_cast<float>(plot_h));
  };
  for (int i = 1; i < static_cast<int>(yv.size()); ++i) {
    surf_line(s, mapx(i - 1), mapy(yv[static_cast<size_t>(i - 1)]), mapx(i),
              mapy(yv[static_cast<size_t>(i)]), color);
  }
  add_text(x + 8, y + h - 18, rgb(160, 160, 170), "span %.2f .. %.2f %s", lo, hi,
           unit);
}

void HistoryWindow::draw() {
  texts_.clear();
  Surface s = surface();
  surf_fill(s, rgb(12, 12, 16));

  constexpr int kListW = 420;
  surf_rect(s, 0, 0, kListW - 1, win_h_ - 1, kColPanel, true);
  add_text(12, 8, kColEst, "EXPERIMENT INDEX");
  add_text(12, 26, rgb(150, 150, 160), "%s", status_.c_str());

  constexpr int kRowH = 18;
  constexpr int kListTop = 48;
  const int vis = (win_h_ - kListTop - 12) / kRowH;
  if (sel_ < scroll_) scroll_ = sel_;
  if (sel_ >= scroll_ + vis) scroll_ = sel_ - vis + 1;
  if (scroll_ < 0) scroll_ = 0;

  for (int i = 0; i < vis; ++i) {
    const int ri = scroll_ + i;
    if (ri < 0 || ri >= static_cast<int>(rows_.size())) break;
    const ExpRow& r = rows_[static_cast<size_t>(ri)];
    const int y = kListTop + i * kRowH;
    if (ri == sel_) {
      surf_rect(s, 6, y - 2, kListW - 8, y + kRowH - 4, rgb(36, 52, 42), true);
    }
    const uint32_t col = (ri == sel_) ? kColEst : kColText;
    add_text(12, y,
             col, "%s  %s  %s  px=%.2f  h=%.2f  r=%.2f",
             r.stamp.size() > 15 ? r.stamp.c_str() + 4 : r.stamp.c_str(),
             r.estimator.c_str(), r.kind.c_str(), r.est_px_rms, r.los_head_rms,
             r.range_rms);
  }

  const int rx = kListW + 8;
  add_text(rx, 8, kColEst, "REVIEW");
  if (sel_ >= 0 && sel_ < static_cast<int>(rows_.size())) {
    const ExpRow& r = rows_[static_cast<size_t>(sel_)];
    add_text(rx, 28, kColText, "%s   %s / %s   %s", r.label.c_str(),
             r.estimator.c_str(), r.meas.empty() ? "-" : r.meas.c_str(),
             r.maneuver.c_str());
    add_text(rx, 46, kColText,
             "H=%.2fs   detect %.0fHz   delay %.0fms   jitter %s   timing %s",
             r.horizon_s, r.detect_hz, r.latency_s * 1000.0f,
             r.jitter ? "ON" : "off", r.timing_on ? "ON" : "off");
    add_text(rx, 64, kColText, "%.1fs  %d steps  rejects %d", r.duration_s,
             r.steps, r.rejects);
    add_text(rx, 86, kColMeas, "est px RMS  %.3f", r.est_px_rms);
    add_text(rx, 104, kColEst, "LOS hdg RMS %.3f deg    att RMS %.3f deg",
             r.los_head_rms, r.los_att_rms);
    add_text(rx, 122, kColPred, "range RMS  %.3f m    size RMS %.3f m",
             r.range_rms, r.size_rms);
    add_text(rx, 144, rgb(150, 150, 160), "%s", r.timeseries.c_str());
  } else {
    add_text(rx, 28, rgb(150, 150, 160), "no experiment selected");
  }

  const int plot_w = win_w_ - rx - 16;
  draw_spark(rx, 168, plot_w, 140, tr_hdg_, kColEst, "delay-removed heading error",
             "deg");
  draw_spark(rx, 316, plot_w, 140, tr_att_, kColDelayed, "delay-removed attack error",
             "deg");
  draw_spark(rx, 464, plot_w, 150, tr_rng_, kColPred, "range error", "m");

  add_text(12, win_h_ - 22, rgb(140, 140, 150),
           "%d runs   H from sim to raise this window",
           static_cast<int>(rows_.size()));
}

void HistoryWindow::blit_to(void* hdc) const {
  if (!hdc || !hdc_mem_) return;
  BitBlt(static_cast<HDC>(hdc), 0, 0, win_w_, win_h_,
         static_cast<HDC>(hdc_mem_), 0, 0, SRCCOPY);
}

void HistoryWindow::present() {
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
