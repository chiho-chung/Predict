#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

// 0x00RRGGBB, matching the BI_RGB 32bpp DIB byte order (B,G,R,X).
constexpr uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}

constexpr uint8_t red_of(uint32_t c) { return static_cast<uint8_t>(c >> 16); }
constexpr uint8_t green_of(uint32_t c) { return static_cast<uint8_t>(c >> 8); }
constexpr uint8_t blue_of(uint32_t c) { return static_cast<uint8_t>(c); }

// Semantic palette shared by the camera/3D window and the plot window, so a
// color means the same thing everywhere.
constexpr uint32_t kColTruth = rgb(150, 150, 150);   // ground truth
constexpr uint32_t kColDelayed = rgb(255, 150, 60);  // clean box, held @detect
constexpr uint32_t kColMeas = rgb(0, 255, 255);      // jittered box, held
constexpr uint32_t kColEst = rgb(90, 255, 120);      // predictor at now
constexpr uint32_t kColPred = rgb(255, 220, 0);      // predictor at now+H
constexpr uint32_t kColChaser = rgb(80, 180, 255);
constexpr uint32_t kColTarget = rgb(220, 70, 70);
constexpr uint32_t kColLookRay = rgb(180, 180, 100);
constexpr uint32_t kColText = rgb(230, 230, 230);
constexpr uint32_t kColPanel = rgb(18, 18, 22);
constexpr uint32_t kColGrid = rgb(52, 52, 60);

// Plain 32-bit pixel buffer view.
struct Surface {
  uint32_t* px = nullptr;
  int w = 0;
  int h = 0;
};

inline void surf_fill(const Surface& s, uint32_t color) {
  std::fill(s.px, s.px + static_cast<size_t>(s.w) * static_cast<size_t>(s.h),
            color);
}

inline void surf_pixel(const Surface& s, int x, int y, uint32_t color) {
  if (x < 0 || y < 0 || x >= s.w || y >= s.h) return;
  s.px[static_cast<size_t>(y) * static_cast<size_t>(s.w) +
       static_cast<size_t>(x)] = color;
}

inline void surf_line(const Surface& s, int x0, int y0, int x1, int y1,
                      uint32_t color) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int guard = 4 * (s.w + s.h) + 16;  // stop runaway lines from wild coords
  for (;;) {
    surf_pixel(s, x0, y0, color);
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

inline void surf_rect(const Surface& s, int x0, int y0, int x1, int y1,
                      uint32_t color, bool fill) {
  if (fill) {
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    x0 = (std::max)(0, x0);
    y0 = (std::max)(0, y0);
    x1 = (std::min)(s.w - 1, x1);
    y1 = (std::min)(s.h - 1, y1);
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x) surf_pixel(s, x, y, color);
  } else {
    surf_line(s, x0, y0, x1, y0, color);
    surf_line(s, x1, y0, x1, y1, color);
    surf_line(s, x1, y1, x0, y1, color);
    surf_line(s, x0, y1, x0, y0, color);
  }
}
