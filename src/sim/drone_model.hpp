#pragma once

#include "sim/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// Shared simple quadrotor: 1 rectangular plate + 4 rotor disks.
// Envelope: 30 cm x 30 cm x 5 cm.
struct DroneModel {
  static constexpr float kSpanX = 0.30f;  // meters
  static constexpr float kSpanY = 0.30f;
  static constexpr float kSpanZ = 0.05f;

  static constexpr float kPlateHalfX = 0.08f;   // 16 cm plate
  static constexpr float kPlateHalfY = 0.08f;
  static constexpr float kPlateHalfZ = 0.01f;   // 2 cm thick

  static constexpr float kRotorRadius = 0.055f;  // 11 cm diameter disks
  static constexpr float kRotorZ = 0.015f;       // slightly above plate mid
  static constexpr float kArm = 0.095f;  // rotor center offset → outer = 0.15 m

  struct Pose {
    Vec3 pos{};
    float yaw = 0;     // body heading (rad)
    float scale = 1;   // visual exaggeration; keep 1 for bbox fitting
  };

  // Body → world
  static Vec3 body_to_world(const Pose& pose, const Vec3& body_in) {
    const Vec3 body = body_in * pose.scale;
    const float c = std::cos(pose.yaw);
    const float s = std::sin(pose.yaw);
    return pose.pos + Vec3{c * body.x + s * body.y, -s * body.x + c * body.y,
                           body.z};
  }

  static std::array<Vec3, 4> rotor_centers_body() {
    return {
        Vec3{+kArm, +kArm, kRotorZ},
        Vec3{+kArm, -kArm, kRotorZ},
        Vec3{-kArm, +kArm, kRotorZ},
        Vec3{-kArm, -kArm, kRotorZ},
    };
  }

  // Plate corners (bottom then top), body frame
  static std::array<Vec3, 8> plate_corners_body() {
    const float hx = kPlateHalfX, hy = kPlateHalfY, hz = kPlateHalfZ;
    return {
        Vec3{-hx, -hy, -hz}, Vec3{+hx, -hy, -hz}, Vec3{+hx, +hy, -hz},
        Vec3{-hx, +hy, -hz}, Vec3{-hx, -hy, +hz}, Vec3{+hx, -hy, +hz},
        Vec3{+hx, +hy, +hz}, Vec3{-hx, +hy, +hz},
    };
  }

  // Dense sample points for tight camera bbox fitting.
  static std::vector<Vec3> sample_points_world(const Pose& pose,
                                               int rim_samples = 16) {
    std::vector<Vec3> out;
    out.reserve(8 + 4 * static_cast<size_t>(rim_samples) + 4);

    for (const Vec3& b : plate_corners_body()) {
      out.push_back(body_to_world(pose, b));
    }

    constexpr float kPi = 3.14159265358979323846f;
    for (const Vec3& rc : rotor_centers_body()) {
      out.push_back(body_to_world(pose, rc));
      for (int i = 0; i < rim_samples; ++i) {
        const float a = (2.0f * kPi * static_cast<float>(i)) /
                        static_cast<float>(rim_samples);
        const Vec3 rim{rc.x + kRotorRadius * std::cos(a),
                       rc.y + kRotorRadius * std::sin(a), rc.z};
        out.push_back(body_to_world(pose, rim));
      }
    }
    return out;
  }
};

struct CameraFrame {
  Vec3 pos{};
  Vec3 forward{};
  Vec3 right{};
  Vec3 up{};
  float fx = 1;
  float fy = 1;
  int width = 640;
  int height = 480;
  float near_z = 0.5f;
  float far_z = 200.0f;
};

inline CameraFrame make_camera_frame(const Vec3& cam_pos, float yaw, float pitch,
                                     const CameraConfig& cfg) {
  constexpr float kPi = 3.14159265358979323846f;
  CameraFrame cam;
  cam.pos = cam_pos;
  cam.width = cfg.width;
  cam.height = cfg.height;
  cam.near_z = cfg.near_z;
  cam.far_z = cfg.far_z;

  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  cam.forward = Vec3{sy * cp, cy * cp, sp}.normalized();
  const Vec3 world_up{0, 0, 1};
  cam.right = cam.forward.cross(world_up).normalized();
  if (cam.right.length() < 1e-4f) cam.right = Vec3{1, 0, 0};
  cam.up = cam.right.cross(cam.forward).normalized();

  cam.fx = (cfg.width * 0.5f) / std::tan(cfg.fov_deg * 0.5f * kPi / 180.0f);
  cam.fy = cam.fx;
  return cam;
}

struct ProjPoint {
  float u = 0, v = 0, depth = 0;
  bool ok = false;
};

inline ProjPoint project_point(const CameraFrame& cam, const Vec3& world) {
  ProjPoint p;
  const Vec3 rel = world - cam.pos;
  const float depth = rel.dot(cam.forward);
  if (depth < cam.near_z || depth > cam.far_z) return p;
  const float x_cam = rel.dot(cam.right);
  const float y_cam = rel.dot(cam.up);
  p.u = cam.width * 0.5f + cam.fx * (x_cam / depth);
  p.v = cam.height * 0.5f - cam.fy * (y_cam / depth);
  p.depth = depth;
  p.ok = true;
  return p;
}

// Pixel -> unit ray in world coordinates. Inverse of project_point.
inline Vec3 unproject_dir(const CameraFrame& cam, float u, float v) {
  const float xn = (u - cam.width * 0.5f) / cam.fx;
  const float yn = (cam.height * 0.5f - v) / cam.fy;
  return (cam.forward + cam.right * xn + cam.up * yn).normalized();
}

inline LosAngles los_from_dir(const Vec3& dir) {
  constexpr float kRadToDeg = 57.29577951308232f;
  LosAngles a;
  const float horiz = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  a.heading_deg = std::atan2(dir.x, dir.y) * kRadToDeg;
  a.attack_deg = std::atan2(dir.z, std::max(horiz, 1e-6f)) * kRadToDeg;
  a.valid = true;
  return a;
}

inline LosAngles los_from_box(const CameraFrame& cam, const BBox& box) {
  const Vec2 c = box.center();
  return los_from_dir(unproject_dir(cam, c.x, c.y));
}

// Axis-aligned bbox from projected drone mesh samples.
inline Detection fit_bbox_from_model(const CameraFrame& cam,
                                     const DroneModel::Pose& pose) {
  Detection det;
  float u0 = 1e9f, v0 = 1e9f, u1 = -1e9f, v1 = -1e9f;
  float depth_sum = 0;
  int n = 0;

  for (const Vec3& w : DroneModel::sample_points_world(pose)) {
    const ProjPoint p = project_point(cam, w);
    if (!p.ok) continue;
    u0 = std::min(u0, p.u);
    v0 = std::min(v0, p.v);
    u1 = std::max(u1, p.u);
    v1 = std::max(v1, p.v);
    depth_sum += p.depth;
    ++n;
  }

  if (n == 0) return det;

  det.box = {u0, v0, u1, v1};
  det.depth = depth_sum / static_cast<float>(n);
  det.visible = (u1 > 0 && u0 < cam.width && v1 > 0 && v0 < cam.height);
  return det;
}
