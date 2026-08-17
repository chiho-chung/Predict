#pragma once

#include "drone_model.hpp"
#include "gfx.hpp"
#include "sim.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Rectangular region of the framebuffer. Drawing helpers that take a Viewport
// use coordinates local to it and clip to its bounds.
struct Viewport {
  int x = 0, y = 0, w = 0, h = 0;
};

// Software framebuffer + Win32 window: gimbal camera feed, 3D chase view, HUD.
class Renderer {
 public:
  Renderer(int cam_w, int cam_h);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  bool create_window(const char* title);
  bool process_events(Simulation& sim);  // returns false to quit
  void draw(const SimSnapshot& snap, const SimConfig& cfg);
  void present();
  void on_destroyed() { hwnd_ = nullptr; }
  void blit_to(void* hdc) const;  // HDC, kept untyped so the header stays free of windows.h

 private:
  Surface surface();
  void clear(uint32_t color);
  void put_pixel(int x, int y, uint32_t color);
  void draw_line(int x0, int y0, int x1, int y1, uint32_t color);
  void draw_rect(int x0, int y0, int x1, int y1, uint32_t color, bool fill);

  // Viewport-local drawing
  void put_pixel_vp(const Viewport& vp, int x, int y, uint32_t color);
  void draw_line_vp(const Viewport& vp, int x0, int y0, int x1, int y1,
                    uint32_t color);
  void draw_rect_vp(const Viewport& vp, int x0, int y0, int x1, int y1,
                    uint32_t color, bool fill);
  void draw_filled_circle_vp(const Viewport& vp, int cx, int cy, int r,
                             uint32_t color, float depth,
                             std::vector<float>& zbuf);
  void draw_filled_tri_vp(const Viewport& vp, int x0, int y0, float z0, int x1,
                          int y1, float z1, int x2, int y2, float z2,
                          uint32_t color, std::vector<float>& zbuf);
  void draw_drone_3d(const Viewport& vp, const CameraFrame& cam,
                     const DroneModel::Pose& pose, uint32_t plate_color,
                     uint32_t rotor_color, std::vector<float>& zbuf);
  void draw_world_line(const Viewport& vp, const CameraFrame& cam, const Vec3& a,
                       const Vec3& b, uint32_t color);
  void draw_ground(const Viewport& vp, const CameraFrame& cam,
                   std::vector<float>& zbuf);
  void draw_drone_topdown(int ox, int oy, int size, const Vec3& center,
                          const Vec3& pos, float yaw, uint32_t color);

  void render_camera_feed(const SimSnapshot& snap, const SimConfig& cfg);
  void render_world_view(const SimSnapshot& snap, const SimConfig& cfg);
  void render_minimap(const SimSnapshot& snap);
  void render_hud(const SimSnapshot& snap, const SimConfig& cfg);

  Viewport cam_vp() const;
  Viewport world_vp() const;
  int panel_x() const;

  int cam_w_ = 480;
  int cam_h_ = 360;
  int view_w_ = 480;
  int view_h_ = 360;
  int win_w_ = 960;
  int win_h_ = 660;
  std::vector<uint32_t> pixels_;

  void* hwnd_ = nullptr;  // HWND without including windows.h here
  void* hdc_mem_ = nullptr;
  void* hbmp_ = nullptr;
  void* old_bmp_ = nullptr;
  bool quit_ = false;
};
