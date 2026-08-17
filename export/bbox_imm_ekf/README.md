# BBoxImmEkf — drop-in IMM-EKF for another project

Copy `bbox_imm_ekf.hpp` and `bbox_imm_ekf.cpp` into the other repo. Compile
`bbox_imm_ekf.cpp` with C++17. No other Predict files, no Eigen.

This is the 3-hypothesis IMM of the export bbox EKF (`export/bbox_ekf`): same
measurements, same camera (intrinsics only), same LOS / range bias knobs.
The bank is quiet / moderate / hard process noise (`3 / 8 / 20` m/s^2), matching
the sim IMM.

## What you must supply each frame

Same as `bbox_ekf`: `heading_deg`, `attack_deg`, `width_px`, `height_px`, `cam`
(fx / FOV / zoom, **not** yaw/pitch), `own_vel`, `t`. Optional known calib:
`heading_bias_deg`, `attack_bias_deg`, `range_bias_m`.

## Minimal loop

```cpp
#include "bbox_imm_ekf.hpp"

bbox_imm_ekf::BBoxImmEkf imm;  // same model as sim Export-IMM

bbox_imm_ekf::Meas m;
m.heading_deg = los_hdg;
m.attack_deg = los_att;
m.width_px = box_w;
m.height_px = box_h;
m.cam = bbox_imm_ekf::Camera::from_fov(640, 480, 70.0f, zoom);
m.own_vel = {vn, ve, vd};
m.t = stamp_s;
imm.push(m);

auto now = imm.predict(cam_now, vel_now, t_now);        // delay removed
auto fut = imm.predict(cam_now, vel_now, t_now + 0.5);  // +H
auto fut2 = imm.predict(cam_now, vel_now, t_now + 0.5, cam_delta);  // + IMU / plan

// now.heading_deg, now.range_m, now.box
// now.model_prob[0..2]  — quiet / moderate / hard
```

`push` uses `m.t` for `dt`. `predict(..., t_now)` coasts from the last stamp.

## Extra `Config` knobs

Everything in `bbox_ekf::Config` is here with the same defaults (LOS / range
bias estimate **off**). Plus:

```cpp
cfg.imm_sigma_accel[0] = 3.0f;   // quiet
cfg.imm_sigma_accel[1] = 8.0f;   // moderate
cfg.imm_sigma_accel[2] = 20.0f;  // hard
cfg.imm_stay_prob = 0.93f;
cfg.sigma_accel = 8.0f;          // used only when coasting the mixed estimate
```

LOS and range bias modes are identical to `export/bbox_ekf/README.md` — swap
the namespace / class name.

## Build

```bat
g++ -std=c++17 -O2 -c bbox_imm_ekf.cpp -o bbox_imm_ekf.o
```
