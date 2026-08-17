#include "ui/render.hpp"

#include "ui/gfx.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace {

// Palette lives in gfx.hpp so the plot window uses identical colors.
constexpr uint32_t kColBoxMeas = kColMeas;
constexpr uint32_t kColBoxTrue = kColTruth;
constexpr uint32_t kColBoxEst = kColEst;
constexpr uint32_t kColBoxPred = kColPred;

// Drones are 30 cm across and the chase view sits tens of meters back, so the
// meshes are drawn exaggerated there. Camera feed always uses true scale.
constexpr float kWorldViewDroneScale = 8.0f;
constexpr float kMapDroneScale = 6.0f;
constexpr float kMapSpanM = 40.0f;  // meters across the minimap

struct HudLine {
  std::string text;
  uint32_t color = kColText;
  bool swatch = false;
  bool swatch_filled = false;
  bool section = false;
};

std::vector<HudLine> g_hud_lines;
int g_hud_x = 0;

struct RollAcc {
  double sum_sq = 0;
  int n = 0;
  void reset() {
    sum_sq = 0;
    n = 0;
  }
  void add(double v) {
    if (!std::isfinite(v)) return;
    sum_sq += v * v;
    ++n;
  }
  double rms() const { return n ? std::sqrt(sum_sq / n) : 0.0; }
};

struct LiveRms {
  RollAcc est_px, held_px, los_h, los_a, range, size;
  float last_t = -1.0f;
  void feed(const SimSnapshot& snap) {
    if (snap.time + 1e-4f < last_t) {
      est_px.reset();
      held_px.reset();
      los_h.reset();
      los_a.reset();
      range.reset();
      size.reset();
    }
    last_t = snap.time;
    if (snap.time < 2.0f) return;
    if (snap.detection.visible && snap.detection_gt.visible) {
      est_px.add(snap.est_err_px);
      held_px.add(snap.held_err_px);
    }
    if (snap.los.origin.valid && snap.los.estimate.valid) {
      float dh = snap.los.estimate.heading_deg - snap.los.origin.heading_deg;
      float da = snap.los.estimate.attack_deg - snap.los.origin.attack_deg;
      while (dh > 180) dh -= 360;
      while (dh < -180) dh += 360;
      los_h.add(dh);
      los_a.add(da);
    }
    if (snap.track_now.valid && snap.track_now.range_m > 0.1f) {
      range.add(snap.track_now.range_m - snap.true_range_m);
      size.add(snap.size_err_m);
    }
  }
};

LiveRms g_live;

// Shared layout: swatches go in the framebuffer, text goes down via GDI, so both
// need the same row geometry.
constexpr int kGap = 8;
constexpr int kPanelW = 328;
constexpr int kMapSize = 196;
constexpr int kHudTop = 228;
constexpr int kHudStep = 15;
constexpr int kHudMaxLines = 38;
constexpr int kSwatchDx = 12;
constexpr int kTextDx = 36;

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
  if (msg == WM_NCCREATE) {
    auto* cs = reinterpret_cast<CREATESTRUCTW*>(lp);
    SetWindowLongPtrW(hwnd, GWLP_USERDATA,
                      reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    return TRUE;
  }
  auto* self =
      reinterpret_cast<Renderer*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
  switch (msg) {
    case WM_ERASEBKGND:
      // We paint the whole client area; letting DefWindowProc fill it first
      // is what makes the view flash every frame.
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
      PostQuitMessage(0);
      return 0;
    default:
      return DefWindowProcW(hwnd, msg, wp, lp);
  }
}

// Third-person camera framing both drones from the side.
CameraFrame make_chase_view_camera(const SimSnapshot& snap,
                                   const CameraConfig& vcfg) {
  const Vec3 a = snap.drone.pos;
  const Vec3 b = snap.target.pos;
  const Vec3 center = (a + b) * 0.5f;

  Vec3 dir = b - a;
  dir.z = 0;
  const float sep = dir.length();
  dir = (sep < 1e-3f) ? Vec3{0, 1, 0} : dir * (1.0f / sep);
  const Vec3 side{-dir.y, dir.x, 0};

  const float d = std::max(16.0f, sep * 1.5f);
  const Vec3 cam_pos = center - dir * (d * 0.30f) + side * (d * 0.80f) +
                       Vec3{0, 0, d * 0.40f};

  const Vec3 to = center - cam_pos;
  const float horiz = std::sqrt(to.x * to.x + to.y * to.y);
  const float yaw = std::atan2(to.x, to.y);
  const float pitch = std::atan2(to.z, std::max(horiz, 1e-4f));
  return make_camera_frame(cam_pos, yaw, pitch, vcfg);
}

}  // namespace

Renderer::Renderer(int cam_w, int cam_h) : cam_w_(cam_w), cam_h_(cam_h) {
  // Camera feed on top, 3D chase view under it, HUD panel to the right.
  view_w_ = cam_w_;
  view_h_ = cam_h_;
  win_w_ = cam_w_ + kGap + kPanelW;
  win_h_ = (std::max)(cam_h_ + kGap + view_h_ + kGap,
                      kHudTop + kHudMaxLines * kHudStep + 12);
  // The HUD column is the taller of the two, so give the leftover height to the
  // chase view instead of leaving a dead band under it.
  view_h_ = win_h_ - cam_h_ - 2 * kGap;
  pixels_.assign(static_cast<size_t>(win_w_) * static_cast<size_t>(win_h_), 0);
}

void Renderer::note_sample(const SimSnapshot& snap) { g_live.feed(snap); }

Viewport Renderer::cam_vp() const { return Viewport{0, 0, cam_w_, cam_h_}; }

Viewport Renderer::world_vp() const {
  return Viewport{0, cam_h_ + kGap, view_w_, view_h_};
}

int Renderer::panel_x() const { return cam_w_ + kGap; }

Renderer::~Renderer() {
  if (hdc_mem_) {
    HDC hdc = static_cast<HDC>(hdc_mem_);
    if (old_bmp_) SelectObject(hdc, static_cast<HBITMAP>(old_bmp_));
    if (hbmp_) DeleteObject(static_cast<HBITMAP>(hbmp_));
    DeleteDC(hdc);
  }
  if (hwnd_) DestroyWindow(static_cast<HWND>(hwnd_));
}

bool Renderer::create_window(const char* title) {
  HINSTANCE hi = GetModuleHandleW(nullptr);
  WNDCLASSW wc{};
  wc.style = CS_OWNDC;
  wc.lpfnWndProc = WndProc;
  wc.hInstance = hi;
  wc.lpszClassName = L"DroneChaseSimWnd";
  wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
  wc.hbrBackground = nullptr;
  RegisterClassW(&wc);

  wchar_t wtitle[256];
  MultiByteToWideChar(CP_UTF8, 0, title, -1, wtitle, 256);

  // Fixed size: a resize invalidates the client and the DIB is not resized,
  // which is another source of flash.
  constexpr DWORD kStyle = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU |
                           WS_MINIMIZEBOX | WS_VISIBLE;
  RECT rc{0, 0, win_w_, win_h_};
  AdjustWindowRect(&rc, kStyle, FALSE);
  HWND hwnd =
      CreateWindowW(wc.lpszClassName, wtitle, kStyle, CW_USEDEFAULT,
                    CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top,
                    nullptr, nullptr, hi, this);
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

bool Renderer::process_events(Simulation& sim) {
  MSG msg;
  while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
    if (msg.message == WM_QUIT) {
      quit_ = true;
      return false;
    }
    if (msg.message == WM_KEYDOWN) {
      switch (msg.wParam) {
        case VK_ESCAPE:
          if (hwnd_) DestroyWindow(static_cast<HWND>(hwnd_));
          quit_ = true;
          return false;
        case 'C':
          sim.toggle_chase();
          break;
        case 'P':
          sim.toggle_predict();
          break;
        case 'J':
          sim.toggle_jitter();
          break;
        case 'T':
          sim.toggle_timing_jitter();
          break;
        case '9':
          sim.adjust_zoom(-0.25f);
          break;
        case '0':
          sim.adjust_zoom(+0.25f);
          break;
        case '1':
          sim.adjust_jitter_center(-1.0f);
          break;
        case '2':
          sim.adjust_jitter_center(+1.0f);
          break;
        case '3':
          sim.adjust_jitter_size(-0.01f);
          break;
        case '4':
          sim.adjust_jitter_size(+0.01f);
          break;
        case '5':
          sim.adjust_detect_hz(-1.0f);
          break;
        case '6':
          sim.adjust_detect_hz(+1.0f);
          break;
        case '7':
          sim.adjust_detect_latency(-0.01f);
          break;
        case '8':
          sim.adjust_detect_latency(+0.01f);
          break;
        case 'E':
          sim.cycle_estimator(+1);
          break;
        case 'W':
          sim.cycle_estimator(-1);
          break;
        case 'M':
          sim.cycle_target_maneuver();
          break;
        case 'G':
          sim.toggle_ideal_gimbal();
          break;
        case 'R':
          sim.reset();
          break;
        case VK_OEM_PLUS:
        case VK_ADD:
          sim.set_predict_horizon(sim.config().predict.horizon_s + 0.1f);
          break;
        case VK_OEM_MINUS:
        case VK_SUBTRACT:
          sim.set_predict_horizon(sim.config().predict.horizon_s - 0.1f);
          break;
        case VK_LEFT:
          sim.nudge_target({-2, 0, 0});
          break;
        case VK_RIGHT:
          sim.nudge_target({2, 0, 0});
          break;
        case VK_UP:
          sim.nudge_target({0, 2, 0});
          break;
        case VK_DOWN:
          sim.nudge_target({0, -2, 0});
          break;
        case 'H':
          hist_req_ = true;
          break;
        default:
          break;
      }
    }
    TranslateMessage(&msg);
    DispatchMessageW(&msg);
  }
  return !quit_;
}

Surface Renderer::surface() {
  return Surface{pixels_.data(), win_w_, win_h_};
}

void Renderer::clear(uint32_t color) { surf_fill(surface(), color); }

void Renderer::put_pixel(int x, int y, uint32_t color) {
  surf_pixel(surface(), x, y, color);
}

void Renderer::draw_line(int x0, int y0, int x1, int y1, uint32_t color) {
  surf_line(surface(), x0, y0, x1, y1, color);
}

void Renderer::draw_rect(int x0, int y0, int x1, int y1, uint32_t color,
                         bool fill) {
  surf_rect(surface(), x0, y0, x1, y1, color, fill);
}

void Renderer::put_pixel_vp(const Viewport& vp, int x, int y, uint32_t color) {
  if (x < 0 || y < 0 || x >= vp.w || y >= vp.h) return;
  put_pixel(vp.x + x, vp.y + y, color);
}

void Renderer::draw_line_vp(const Viewport& vp, int x0, int y0, int x1, int y1,
                            uint32_t color) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int guard = 4 * (vp.w + vp.h) + 16;
  for (;;) {
    put_pixel_vp(vp, x0, y0, color);
    if ((x0 == x1 && y0 == y1) || --guard <= 0) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void Renderer::draw_rect_vp(const Viewport& vp, int x0, int y0, int x1, int y1,
                            uint32_t color, bool fill) {
  if (fill) {
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    x0 = (std::max)(0, x0);
    y0 = (std::max)(0, y0);
    x1 = (std::min)(vp.w - 1, x1);
    y1 = (std::min)(vp.h - 1, y1);
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) put_pixel_vp(vp, x, y, color);
  } else {
    draw_line_vp(vp, x0, y0, x1, y0, color);
    draw_line_vp(vp, x1, y0, x1, y1, color);
    draw_line_vp(vp, x1, y1, x0, y1, color);
    draw_line_vp(vp, x0, y1, x0, y0, color);
  }
}

void Renderer::draw_filled_circle_vp(const Viewport& vp, int cx, int cy, int r,
                                     uint32_t color, float depth,
                                     std::vector<float>& zbuf) {
  if (r < 1) r = 1;
  const int r2 = r * r;
  for (int y = -r; y <= r; ++y) {
    for (int x = -r; x <= r; ++x) {
      if (x * x + y * y > r2) continue;
      const int px = cx + x;
      const int py = cy + y;
      if (px < 0 || py < 0 || px >= vp.w || py >= vp.h) continue;
      const size_t i = static_cast<size_t>(py) * static_cast<size_t>(vp.w) +
                       static_cast<size_t>(px);
      if (depth < zbuf[i]) {
        zbuf[i] = depth;
        put_pixel_vp(vp, px, py, color);
      }
    }
  }
}

void Renderer::draw_filled_tri_vp(const Viewport& vp, int x0, int y0, float z0,
                                  int x1, int y1, float z1, int x2, int y2,
                                  float z2, uint32_t color,
                                  std::vector<float>& zbuf) {
  int minx = (std::max)(0, (std::min)({x0, x1, x2}));
  int maxx = (std::min)(vp.w - 1, (std::max)({x0, x1, x2}));
  int miny = (std::max)(0, (std::min)({y0, y1, y2}));
  int maxy = (std::min)(vp.h - 1, (std::max)({y0, y1, y2}));

  const float area =
      static_cast<float>((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
  if (std::fabs(area) < 1e-3f) return;

  for (int y = miny; y <= maxy; ++y) {
    for (int x = minx; x <= maxx; ++x) {
      const float w0 =
          static_cast<float>((x1 - x) * (y2 - y) - (x2 - x) * (y1 - y)) / area;
      const float w1 =
          static_cast<float>((x2 - x) * (y0 - y) - (x0 - x) * (y2 - y)) / area;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < 0 || w1 < 0 || w2 < 0) continue;
      const float depth = w0 * z0 + w1 * z1 + w2 * z2;
      const size_t i = static_cast<size_t>(y) * static_cast<size_t>(vp.w) +
                       static_cast<size_t>(x);
      if (depth < zbuf[i]) {
        zbuf[i] = depth;
        put_pixel_vp(vp, x, y, color);
      }
    }
  }
}

void Renderer::draw_drone_3d(const Viewport& vp, const CameraFrame& cam,
                             const DroneModel::Pose& pose, uint32_t plate_color,
                             uint32_t rotor_color, std::vector<float>& zbuf) {
  // Top face of plate (two triangles)
  const auto corners = DroneModel::plate_corners_body();
  ProjPoint tl[4];
  for (int i = 0; i < 4; ++i) {
    tl[i] = project_point(cam, DroneModel::body_to_world(pose, corners[4 + i]));
  }
  if (tl[0].ok && tl[1].ok && tl[2].ok) {
    draw_filled_tri_vp(vp, static_cast<int>(tl[0].u), static_cast<int>(tl[0].v),
                       tl[0].depth, static_cast<int>(tl[1].u),
                       static_cast<int>(tl[1].v), tl[1].depth,
                       static_cast<int>(tl[2].u), static_cast<int>(tl[2].v),
                       tl[2].depth, plate_color, zbuf);
  }
  if (tl[0].ok && tl[2].ok && tl[3].ok) {
    draw_filled_tri_vp(vp, static_cast<int>(tl[0].u), static_cast<int>(tl[0].v),
                       tl[0].depth, static_cast<int>(tl[2].u),
                       static_cast<int>(tl[2].v), tl[2].depth,
                       static_cast<int>(tl[3].u), static_cast<int>(tl[3].v),
                       tl[3].depth, plate_color, zbuf);
  }

  // Four rotor disks — project center + rim to get pixel radius
  for (const Vec3& rc : DroneModel::rotor_centers_body()) {
    const Vec3 center_w = DroneModel::body_to_world(pose, rc);
    const Vec3 rim_body{rc.x + DroneModel::kRotorRadius, rc.y, rc.z};
    const Vec3 rim_w = DroneModel::body_to_world(pose, rim_body);
    const ProjPoint pc = project_point(cam, center_w);
    const ProjPoint pr = project_point(cam, rim_w);
    if (!pc.ok) continue;
    float rad = 2.0f;
    if (pr.ok) {
      const float du = pr.u - pc.u;
      const float dv = pr.v - pc.v;
      rad = std::sqrt(du * du + dv * dv);
    }
    draw_filled_circle_vp(vp, static_cast<int>(pc.u), static_cast<int>(pc.v),
                          static_cast<int>(rad + 0.5f), rotor_color, pc.depth,
                          zbuf);
  }
}

void Renderer::draw_world_line(const Viewport& vp, const CameraFrame& cam,
                               const Vec3& a, const Vec3& b, uint32_t color) {
  // Subdivide so segments crossing behind the camera are simply dropped.
  constexpr int kSeg = 16;
  ProjPoint prev = project_point(cam, a);
  for (int i = 1; i <= kSeg; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kSeg);
    const ProjPoint cur = project_point(cam, a + (b - a) * t);
    if (prev.ok && cur.ok) {
      draw_line_vp(vp, static_cast<int>(prev.u), static_cast<int>(prev.v),
                   static_cast<int>(cur.u), static_cast<int>(cur.v), color);
    }
    prev = cur;
  }
}

void Renderer::draw_ground(const Viewport& vp, const CameraFrame& cam,
                           std::vector<float>& zbuf) {
  // Per-pixel ray vs the z=0 plane: gives a correct horizon plus depth values
  // so the drone meshes occlude against the ground.
  for (int y = 0; y < vp.h; ++y) {
    const float yn = (vp.h * 0.5f - (y + 0.5f)) / cam.fy;
    for (int x = 0; x < vp.w; ++x) {
      const float xn = (x + 0.5f - vp.w * 0.5f) / cam.fx;
      const Vec3 dir = cam.forward + cam.right * xn + cam.up * yn;

      uint32_t color;
      if (dir.z > -1e-4f || cam.pos.z <= 0) {
        // Sky gradient, brighter at the top
        const float f = 1.0f - static_cast<float>(y) / static_cast<float>(vp.h);
        const uint8_t r = static_cast<uint8_t>(120 + 50 * f);
        const uint8_t g = static_cast<uint8_t>(160 + 40 * f);
        const uint8_t b = static_cast<uint8_t>(205 + 40 * f);
        color = rgb(r, g, b);
      } else {
        const float t = -cam.pos.z / dir.z;
        if (t <= 0 || t > 400.0f) {
          color = rgb(150, 175, 200);
        } else {
          const Vec3 hit = cam.pos + dir * t;
          const int cell =
              (static_cast<int>(std::floor(hit.x / 5.0f)) +
               static_cast<int>(std::floor(hit.y / 5.0f))) &
              1;
          const float fog = std::min(1.0f, t / 260.0f);
          const float base = cell ? 84.0f : 62.0f;
          const uint8_t r =
              static_cast<uint8_t>(base * 0.55f + fog * (150 - base * 0.55f));
          const uint8_t g =
              static_cast<uint8_t>(base + fog * (175 - base));
          const uint8_t b =
              static_cast<uint8_t>(base * 0.62f + fog * (200 - base * 0.62f));
          color = rgb(r, g, b);
          const size_t i = static_cast<size_t>(y) * static_cast<size_t>(vp.w) +
                           static_cast<size_t>(x);
          zbuf[i] = t;
        }
      }
      put_pixel_vp(vp, x, y, color);
    }
  }
}

void Renderer::draw_drone_topdown(int ox, int oy, int size, const Vec3& center,
                                  const Vec3& pos, float yaw, uint32_t color) {
  const float scale = size / kMapSpanM;
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  // A 30 cm mesh is sub-pixel at this map scale, so exaggerate it.
  const float body = kMapDroneScale;
  auto map = [&](const Vec3& b) {
    const Vec3 v = b * body;
    const Vec3 w = pos + Vec3{c * v.x + s * v.y, -s * v.x + c * v.y, 0} - center;
    return std::pair<int, int>{ox + size / 2 + static_cast<int>(w.x * scale),
                               oy + size / 2 - static_cast<int>(w.y * scale)};
  };

  const float hx = DroneModel::kPlateHalfX;
  const float hy = DroneModel::kPlateHalfY;
  const auto p0 = map({-hx, -hy, 0});
  const auto p1 = map({+hx, -hy, 0});
  const auto p2 = map({+hx, +hy, 0});
  const auto p3 = map({-hx, +hy, 0});
  draw_line(p0.first, p0.second, p1.first, p1.second, color);
  draw_line(p1.first, p1.second, p2.first, p2.second, color);
  draw_line(p2.first, p2.second, p3.first, p3.second, color);
  draw_line(p3.first, p3.second, p0.first, p0.second, color);

  const int rr = (std::max)(
      1, static_cast<int>(DroneModel::kRotorRadius * body * scale));
  for (const Vec3& rc : DroneModel::rotor_centers_body()) {
    const auto m = map(rc);
    draw_rect(m.first - rr, m.second - rr, m.first + rr, m.second + rr, color,
              false);
  }
}

void Renderer::render_camera_feed(const SimSnapshot& snap, const SimConfig& cfg) {
  const Viewport vp = cam_vp();

  std::vector<float> zbuf(
      static_cast<size_t>(vp.w) * static_cast<size_t>(vp.h), 1e9f);
  const Vec3 cam_pos = snap.drone.pos + Vec3{0, 0, -0.04f};
  const CameraFrame cam =
      make_camera_frame(cam_pos, snap.gimbal.yaw, snap.gimbal.pitch, cfg.camera);

  draw_ground(vp, cam, zbuf);
  const uint32_t grid = rgb(105, 125, 105);
  for (int i = -6; i <= 6; ++i) {
    const float c = i * 10.0f;
    draw_world_line(vp, cam, {c, -60, 0}, {c, 60, 0}, grid);
    draw_world_line(vp, cam, {-60, c, 0}, {60, c, 0}, grid);
  }

  DroneModel::Pose target_pose;
  target_pose.pos = snap.target.pos;
  target_pose.yaw = snap.target.yaw;
  draw_drone_3d(vp, cam, target_pose, rgb(45, 45, 52), rgb(22, 22, 26), zbuf);

  auto box_rect = [&](const BBox& b, uint32_t color) {
    draw_rect_vp(vp, static_cast<int>(b.u0), static_cast<int>(b.v0),
                 static_cast<int>(b.u1), static_cast<int>(b.v1), color, false);
  };

  if (snap.detection_gt.visible) box_rect(snap.detection_gt.box, kColBoxTrue);
  if (snap.detection.visible) box_rect(snap.detection.box, kColBoxMeas);
  if (snap.estimate_now.visible) box_rect(snap.estimate_now.box, kColBoxEst);
  if (snap.predicted.visible) {
    box_rect(snap.predicted.box, kColBoxPred);
    if (snap.estimate_now.visible) {
      const Vec2 a = snap.estimate_now.box.center();
      const Vec2 b = snap.predicted.box.center();
      draw_line_vp(vp, static_cast<int>(a.x), static_cast<int>(a.y),
                   static_cast<int>(b.x), static_cast<int>(b.y), kColBoxPred);
    }
  }

  // Detector duty bar: fills up between 10 Hz ticks, resets on each new box.
  const float frac =
      std::min(1.0f, snap.detection_age / std::max(1e-3f, cfg.detect_dt()));
  const int bar_w = 120;
  const int bx = vp.w - bar_w - 10;
  const int by = 10;
  draw_rect_vp(vp, bx, by, bx + bar_w, by + 8, rgb(20, 20, 20), true);
  draw_rect_vp(vp, bx, by, bx + static_cast<int>(bar_w * frac), by + 8,
               kColBoxMeas, true);
  draw_rect_vp(vp, bx, by, bx + bar_w, by + 8, rgb(200, 200, 200), false);

  const int cx = vp.w / 2, cy = vp.h / 2;
  draw_line_vp(vp, cx - 12, cy, cx + 12, cy, rgb(255, 255, 255));
  draw_line_vp(vp, cx, cy - 12, cx, cy + 12, rgb(255, 255, 255));
  draw_rect_vp(vp, 0, 0, vp.w - 1, vp.h - 1, rgb(90, 90, 100), false);
}

void Renderer::render_world_view(const SimSnapshot& snap, const SimConfig& cfg) {
  const Viewport vp = world_vp();

  CameraConfig vcfg;
  vcfg.width = vp.w;
  vcfg.height = vp.h;
  vcfg.fov_deg = 60.0f;
  vcfg.near_z = 0.3f;
  vcfg.far_z = 500.0f;
  const CameraFrame view = make_chase_view_camera(snap, vcfg);

  std::vector<float> zbuf(
      static_cast<size_t>(vp.w) * static_cast<size_t>(vp.h), 1e9f);
  draw_ground(vp, view, zbuf);

  // Ground grid every 10 m for scale
  const uint32_t grid = rgb(110, 130, 110);
  for (int i = -6; i <= 6; ++i) {
    const float c = i * 10.0f;
    draw_world_line(vp, view, {c, -60, 0}, {c, 60, 0}, grid);
    draw_world_line(vp, view, {-60, c, 0}, {60, c, 0}, grid);
  }

  // Altitude drop lines
  draw_world_line(vp, view, snap.drone.pos, {snap.drone.pos.x, snap.drone.pos.y, 0},
                  kColChaser);
  draw_world_line(vp, view, snap.target.pos,
                  {snap.target.pos.x, snap.target.pos.y, 0}, kColTarget);

  // Gimbal camera axis and FOV edges
  const Vec3 cam_pos = snap.drone.pos + Vec3{0, 0, -0.04f};
  const CameraFrame gim =
      make_camera_frame(cam_pos, snap.gimbal.yaw, snap.gimbal.pitch, cfg.camera);
  const float ray_len =
      std::max(6.0f, (snap.target.pos - snap.drone.pos).length() * 1.15f);
  draw_world_line(vp, view, cam_pos, cam_pos + gim.forward * ray_len,
                  kColLookRay);
  const float tx = (cfg.camera.width * 0.5f) / gim.fx;
  const float ty = (cfg.camera.height * 0.5f) / gim.fy;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      const Vec3 corner =
          (gim.forward + gim.right * (tx * sx) + gim.up * (ty * sy)).normalized();
      draw_world_line(vp, view, cam_pos, cam_pos + corner * ray_len,
                      rgb(140, 140, 80));
    }
  }

  // Line of sight
  draw_world_line(vp, view, snap.drone.pos, snap.target.pos, rgb(200, 200, 200));

  DroneModel::Pose chaser_pose;
  chaser_pose.pos = snap.drone.pos;
  chaser_pose.yaw = snap.drone.yaw;
  chaser_pose.scale = kWorldViewDroneScale;
  draw_drone_3d(vp, view, chaser_pose, kColChaser, rgb(30, 70, 110), zbuf);

  DroneModel::Pose target_pose;
  target_pose.pos = snap.target.pos;
  target_pose.yaw = snap.target.yaw;
  target_pose.scale = kWorldViewDroneScale;
  draw_drone_3d(vp, view, target_pose, kColTarget, rgb(90, 25, 25), zbuf);

  // Where the target will be after the horizon
  const ProjPoint pf = project_point(view, snap.predicted_world);
  if (pf.ok) {
    const int px = static_cast<int>(pf.u);
    const int py = static_cast<int>(pf.v);
    draw_line_vp(vp, px - 5, py, px + 5, py, kColBoxPred);
    draw_line_vp(vp, px, py - 5, px, py + 5, kColBoxPred);
    draw_world_line(vp, view, snap.target.pos, snap.predicted_world, kColBoxPred);
  }

  draw_rect_vp(vp, 0, 0, vp.w - 1, vp.h - 1, rgb(90, 90, 100), false);
}

void Renderer::render_minimap(const SimSnapshot& snap) {
  const int ox = panel_x() + 12;
  const int oy = 12;
  const int size = kMapSize;
  draw_rect(ox, oy, ox + size, oy + size, rgb(25, 25, 30), true);
  draw_rect(ox, oy, ox + size, oy + size, rgb(80, 80, 90), false);

  draw_line(ox + size / 2, oy, ox + size / 2, oy + size, rgb(50, 50, 60));
  draw_line(ox, oy + size / 2, ox + size, oy + size / 2, rgb(50, 50, 60));

  // Map follows the chaser, so the pair stays visible however far they roam.
  const Vec3 center = snap.drone.pos;
  draw_drone_topdown(ox, oy, size, center, snap.drone.pos, snap.drone.yaw,
                     kColChaser);
  draw_drone_topdown(ox, oy, size, center, snap.target.pos, snap.target.yaw,
                     kColTarget);

  const float scale = size / kMapSpanM;
  auto world_to_map = [&](const Vec3& p) {
    const Vec3 r = p - center;
    return std::pair<int, int>{ox + size / 2 + static_cast<int>(r.x * scale),
                               oy + size / 2 - static_cast<int>(r.y * scale)};
  };
  const auto pr = world_to_map(snap.predicted_world);
  draw_rect(pr.first - 2, pr.second - 2, pr.first + 2, pr.second + 2, kColBoxPred,
            true);

  const auto d = world_to_map(snap.drone.pos);
  const Vec3 tip = snap.drone.pos + snap.drone.vel * 0.6f;
  const auto dv = world_to_map(tip);
  draw_line(d.first, d.second, dv.first, dv.second, kColChaser);

  const Vec3 look_end =
      snap.drone.pos +
      Vec3{std::sin(snap.gimbal.yaw), std::cos(snap.gimbal.yaw), 0} * 15.0f;
  const auto le = world_to_map(look_end);
  draw_line(d.first, d.second, le.first, le.second, kColLookRay);
}

void Renderer::render_hud(const SimSnapshot& snap, const SimConfig& cfg) {
  const int px = panel_x();
  g_hud_x = px;
  draw_rect(px, 0, win_w_ - 1, win_h_ - 1, kColPanel, true);
  render_minimap(snap);
  // live RMS is accumulated in note_sample() each sim step

  char line[160];
  g_hud_lines.clear();

  auto add = [&](const char* text, uint32_t color = kColText) {
    g_hud_lines.push_back(HudLine{text, color, false, false, false});
  };
  auto add_legend = [&](const char* text, uint32_t color, bool filled) {
    g_hud_lines.push_back(HudLine{text, color, true, filled, false});
  };
  auto add_section = [&](const char* text) {
    g_hud_lines.push_back(HudLine{text, kColEst, false, false, true});
  };

  add_legend("measured (held)", kColBoxMeas, false);
  add_legend("true box", kColBoxTrue, false);
  add_legend("estimate now", kColBoxEst, false);
  add_legend("predicted +H", kColBoxPred, false);

  add_section("SETUP");
  const char* meas_tag = !estimator_uses_filter(cfg.tracker.type)
                             ? "px"
                             : (estimator_uses_bbox(cfg.tracker.type) ? "bbox"
                                                                     : "LOS");
  std::snprintf(line, sizeof(line), "E/W  %s  %s",
                estimator_name(cfg.tracker.type), meas_tag);
  add(line, kColBoxEst);
  std::snprintf(line, sizeof(line), "M    target  %s",
                cfg.target.maneuver == TargetManeuver::Jink ? "JINK" : "smooth");
  add(line);
  std::snprintf(line, sizeof(line), "C    chase   %s",
                cfg.chase_enabled ? "ON" : "off");
  add(line);
  std::snprintf(line, sizeof(line), "P    predict %s  H=%.1fs  (+/-)",
                cfg.predict.enabled ? "ON" : "off", cfg.predict.horizon_s);
  add(line);
  std::snprintf(line, sizeof(line), "J    jitter  %s  ctr 1/2  size 3/4",
                cfg.jitter.enabled ? "ON" : "off");
  add(line);
  std::snprintf(line, sizeof(line), "     ctr+/-%.0fpx  sz+/-%.0f%%",
                cfg.jitter.center_px, cfg.jitter.size_frac * 100.0f);
  add(line);
  std::snprintf(line, sizeof(line), "T    timing  %s",
                cfg.timing.enabled ? "ON" : "off");
  add(line);
  std::snprintf(line, sizeof(line), "G    gimbal  %s",
                cfg.gimbal.ideal ? "IDEAL" : "servo");
  add(line);
  std::snprintf(line, sizeof(line), "5/6  detect  %.0f Hz", cfg.rates.detect_hz);
  add(line);
  std::snprintf(line, sizeof(line), "7/8  delay   %.0f ms",
                cfg.rates.detect_latency_s * 1000.0f);
  add(line);
  std::snprintf(line, sizeof(line), "9/0  zoom    %.2fx  FOV %.0f  fx %.0f",
                snap.zoom, snap.fov_deg, snap.fx);
  add(line);
  add("H    history  (review logs)");
  add("R reset   Esc quit");

  add_section("ANALYSIS");
  std::snprintf(line, sizeof(line), "t=%.1fs  det #%d  age=%.0fms  upd %d rej %d",
                snap.time, snap.detect_count, snap.detection_age * 1000.0f,
                snap.tracker_updates, snap.tracker_rejects);
  add(line);
  if (cfg.timing.enabled) {
    const float inst_hz =
        snap.last_period_s > 1e-4f ? 1.0f / snap.last_period_s : 0.0f;
    std::snprintf(line, sizeof(line), "inst %.1fHz  lat %.0fms  stamp%+.1fms",
                  inst_hz, snap.last_latency_s * 1000.0f,
                  snap.stamp_err_s * 1000.0f);
    add(line);
  } else {
    std::snprintf(line, sizeof(line), "timing off  stable %.0fHz",
                  cfg.rates.detect_hz);
    add(line);
  }
  std::snprintf(line, sizeof(line), "range true %.1f m   chaser %.1f  tgt %.1f m/s",
                snap.true_range_m, snap.drone.vel.length(),
                snap.target.vel.length());
  add(line);
  if (snap.detection.visible) {
    std::snprintf(line, sizeof(line), "box %.0fx%.0f px   held %.1f  est %.1f  off %.0f",
                  snap.detection.box.width(), snap.detection.box.height(),
                  snap.held_err_px, snap.est_err_px, snap.track_err_px);
    add(line, kColBoxMeas);
  } else {
    add("box: lost", kColTarget);
  }
  if (snap.track_now.valid && snap.track_now.range_m > 0) {
    std::snprintf(line, sizeof(line), "est range %.1f +/-%.1f m  (err %.2f)",
                  snap.track_now.range_m, snap.track_now.range_sigma_m,
                  snap.range_err_m);
    add(line, kColBoxEst);
    std::snprintf(line, sizeof(line), "extent %.2fx%.2f m  (err %.2f)",
                  snap.track_now.size_w_m, snap.track_now.size_h_m,
                  snap.size_err_m);
    add(line);
    std::snprintf(line, sizeof(line), "tgt speed %.1f m/s  (true %.1f)",
                  snap.track_now.speed_mps, snap.target.vel.length());
    add(line);
    if (snap.track_now.model_count > 1) {
      std::snprintf(line, sizeof(line), "IMM q%.2f m%.2f h%.2f",
                    snap.track_now.model_prob[0], snap.track_now.model_prob[1],
                    snap.track_now.model_prob[2]);
      add(line);
    }
  } else {
    add("filter: image-space only (no range/size)");
  }
  add("this-run RMS after 2s:", rgb(170, 170, 180));
  std::snprintf(line, sizeof(line), "px held %.2f  est %.2f", g_live.held_px.rms(),
                g_live.est_px.rms());
  add(line, kColBoxEst);
  std::snprintf(line, sizeof(line), "LOS hdg %.3f deg   att %.3f deg",
                g_live.los_h.rms(), g_live.los_a.rms());
  add(line);
  std::snprintf(line, sizeof(line), "range %.2f m   size %.3f m",
                g_live.range.rms(), g_live.size.rms());
  add(line, kColBoxPred);

  const int sx = px + kSwatchDx;
  for (size_t i = 0; i < g_hud_lines.size(); ++i) {
    const HudLine& hl = g_hud_lines[i];
    const int y = kHudTop + static_cast<int>(i) * kHudStep;
    if (hl.section) {
      draw_rect(px + 4, y - 1, win_w_ - 5, y + kHudStep - 3, rgb(28, 36, 32),
                true);
    }
    if (!hl.swatch) continue;
    draw_rect(sx, y + 3, sx + 12, y + 12, hl.color, hl.swatch_filled);
  }
}

void Renderer::draw(const SimSnapshot& snap, const SimConfig& cfg) {
  clear(rgb(8, 8, 10));
  render_camera_feed(snap, cfg);
  render_world_view(snap, cfg);
  render_hud(snap, cfg);
}

void Renderer::blit_to(void* hdc) const {
  if (!hdc || !hdc_mem_) return;
  BitBlt(static_cast<HDC>(hdc), 0, 0, win_w_, win_h_,
         static_cast<HDC>(hdc_mem_), 0, 0, SRCCOPY);
}

void Renderer::present() {
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

  // Text goes on the backbuffer so the window sees one finished frame, not
  // "bitmap then labels" which reads as blink.
  SetBkMode(hdc_mem, TRANSPARENT);
  auto text_at = [&](int x, int y, uint32_t color, const char* s) {
    SetTextColor(hdc_mem, RGB(red_of(color), green_of(color), blue_of(color)));
    TextOutA(hdc_mem, x, y, s, static_cast<int>(std::strlen(s)));
  };

  const int ox = g_hud_x + kTextDx;
  for (size_t i = 0; i < g_hud_lines.size(); ++i) {
    const HudLine& hl = g_hud_lines[i];
    if (hl.text.empty()) continue;
    text_at(ox, kHudTop + static_cast<int>(i) * kHudStep, hl.color,
            hl.text.c_str());
  }

  text_at(8, 6, kColText, "GIMBAL CAMERA FEED - true scale, bbox @10Hz");
  text_at(8, cam_h_ + kGap + 6, kColText,
          "3D CHASE VIEW - drones drawn oversized");

  HDC hdc = GetDC(hwnd);
  blit_to(hdc);
  ReleaseDC(hwnd, hdc);
}
