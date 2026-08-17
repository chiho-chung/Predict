# BBoxEkf — drop-in EKF for another project

Copy `bbox_ekf.hpp` and `bbox_ekf.cpp` into the other repo. Compile `bbox_ekf.cpp` with C++17. No other Predict files, no Eigen.

This is the same bbox EKF used in the chase sim (timestamped updates, delay removal, future LOS), with a slimmer camera input: **intrinsics only**.

## What you must supply each frame

| Input | Meaning |
|---|---|
| `heading_deg` | LOS azimuth (world / NED) |
| `attack_deg` | LOS elevation / pitch, negative = look down |
| `heading_bias_deg`, `attack_bias_deg` | Optional known/calib LOS offset. Leave 0 if unknown. |
| `range_bias_m` | Optional known/calib range offset (metres). Leave 0 if unknown. |
| `width_px`, `height_px` | Box size in pixels (angular size) |
| `cam` | **Intrinsics + zoom at capture** (`fx` / FOV / image size). Not yaw/pitch. |
| `own_vel` | Your velocity in the **same world / NED frame** as the LOS |
| `t` | Frame **timestamp in seconds** (not “frame index”, not `1/Hz`) |

You do **not** need own position or gimbal attitude. Range comes from

```text
range ≈ fx * size_m / width_px
```

LOS is already world heading/attack, so camera orientation is unused.

## Minimal loop

```cpp
#include "bbox_ekf.hpp"

bbox_ekf::BBoxEkf ekf;   // defaults match the sim

// --- slow path: each detector frame ---
bbox_ekf::Meas m;
m.heading_deg = los_hdg;   // your LOS measurement
m.attack_deg  = los_att;   // pitch / elevation
m.width_px    = box_w;     // box size, pixels
m.height_px   = box_h;
// zoom of THIS frame (1 = 70° wide, 2 = 35°, fx doubles). Required if the
// lens zooms; box size in pixels scales with zoom.
m.cam = bbox_ekf::Camera::from_fov(640, 480, 70.0f, zoom);
// or, if you already have calibrated fx (pixels):
// m.cam = bbox_ekf::Camera::from_fx(640, 480, fx);
// or: keep a Camera and call cam.set_zoom(zoom, 70.0f) each frame
m.own_vel = {vn, ve, vd};   // world / NED, m/s, at capture
m.t = stamp_s;              // capture time, not arrival time
// optional: known LOS / range calib. Leave 0 if unknown. The filter
// estimates a residual only when the matching Config sigma is > 0.
// m.heading_bias_deg = calib_hdg;
// m.attack_bias_deg  = calib_att;
// m.range_bias_m     = calib_range_m;
ekf.push(m);

// --- fast path: every control tick ---
const double t_now = clock_s;
const bbox_ekf::Camera cam_now = bbox_ekf::Camera::from_fov(640, 480, 70.0f, zoom_now);
const bbox_ekf::Vec3 vel_now = {vn, ve, vd};

auto now  = ekf.predict(cam_now, vel_now, t_now);        // delay removed
auto fut  = ekf.predict(cam_now, vel_now, t_now + 0.5);  // +H future LOS

if (now.valid) {
  // now.heading_deg, now.attack_deg  — bias removed (from p_rel)
  // now.heading_bias_deg, now.attack_bias_deg  — estimated residual
  // now.range_m, now.range_bias_m, now.box, now.vel_world
}
```

`push` uses `m.t` to get `dt` (irregular cameras are fine). `predict(..., t_now)` coasts from the last stamp to now, which **removes pipeline delay**. `t_now + H` is the future LOS.

`predict` uses `cam_query` only for `fx` / zoom (predicted box size in pixels). `now.box` is that size, placed at the image centre — there is no attitude to project a pixel centre.

## Camera (intrinsics)

- `fx, fy` — pixels; principal point is image centre
- `width, height` — image size
- `fov_wide_deg` / `zoom` — `fov = wide / zoom`, then `fx` from that FOV
- yaw / pitch / forward / right / up are **not** part of this API

If the lens zooms, pass this frame’s zoom (or this frame’s `fx`). A stale `fx` scales range wrong.

## Tuning (`bbox_ekf::Config`)

Defaults are the sim’s EKF:

```cpp
bbox_ekf::Config cfg;
cfg.sigma_px_center = 4.0f;   // LOS noise, pixels (converted to angle via fx)
cfg.sigma_px_size   = 4.0f;   // box width/height noise, pixels — not a percent
cfg.size_prior_m    = 0.36f;  // metres of target that span the box
cfg.sigma_accel     = 8.0f;   // raise if the target jinks hard
cfg.meas_corr       = 0.6f;   // 0 = disable size-bias whitening
cfg.los_bias_sigma_deg = 0.0f;  // 0 = off (default). Set ~2 to estimate residual
cfg.los_bias_walk_deg  = 0.02f; // deg/sqrt(s) when estimation is on
cfg.range_bias_sigma_m = 0.0f;  // 0 = off (default). Set ~1 to estimate residual
cfg.range_bias_walk_m  = 0.0f;  // freeze as constant when estimation is on
ekf.set_config(cfg);
```

If the target is not ~30 cm, set `size_prior_m` to the width that actually fills the box.

## LOS bias modes

Two knobs:

| Knob | Where | What it does |
|---|---|---|
| `los_bias_sigma_deg` | `Config` | `0` = do not estimate. `~2` = estimate a residual (deg 1-sigma). |
| `heading_bias_deg`, `attack_bias_deg` | `Meas` each frame | Known/calib offset (deg). Subtracted before the update. Leave `0` if unknown. |

`Estimate::heading_deg` / `attack_deg` always come from `p_rel` (bias removed). `Estimate::heading_bias_deg` / `attack_bias_deg` are the estimated residual. Call `ekf.reset()` if you change mode on a live filter.

### 1. Raw (default)

No calib, no estimation. Same as the sim EKF.

```cpp
bbox_ekf::Config cfg;
cfg.los_bias_sigma_deg = 0.0f;     // already 0
bbox_ekf::BBoxEkf ekf(cfg);

m.heading_bias_deg = 0;
m.attack_bias_deg  = 0;
ekf.push(m);
```

### 2. Estimate

Unknown offset; the filter tries to find it. Weak — a constant angle bias looks like a sideways position error when the only direction sensor is that same LOS. Use only if you expect leftover bias.

```cpp
bbox_ekf::Config cfg;
cfg.los_bias_sigma_deg = 2.0f;     // initial 1-sigma, deg
cfg.los_bias_walk_deg  = 0.02f;    // slow drift; 0 = freeze as constant
bbox_ekf::BBoxEkf ekf(cfg);

m.heading_bias_deg = 0;
m.attack_bias_deg  = 0;
ekf.push(m);
// now.heading_bias_deg / now.attack_bias_deg  — what it estimated
```

### 3. Known calib (best if you have the offset)

```cpp
bbox_ekf::Config cfg;
cfg.los_bias_sigma_deg = 0.0f;     // do not also estimate a residual
bbox_ekf::BBoxEkf ekf(cfg);

m.heading_deg = los_hdg;           // raw measured LOS
m.attack_deg  = los_att;
m.heading_bias_deg = calib_hdg;    // known offset, deg
m.attack_bias_deg  = calib_att;
ekf.push(m);
```

### 4. Known + estimate leftover

Calib first, then a small residual on top.

```cpp
bbox_ekf::Config cfg;
cfg.los_bias_sigma_deg = 2.0f;
bbox_ekf::BBoxEkf ekf(cfg);

m.heading_bias_deg = calib_hdg;
m.attack_bias_deg  = calib_att;
ekf.push(m);
```

`ekf.set_config(cfg)` after construct is the same as passing `Config` in. Do not enable estimate (mode 2) on a clean LOS — range/LOS get slightly worse.

## Range bias modes

Same four modes and the same defaults as LOS (`sigma = 0`). Size and range share one observation (`width_px ≈ fx * size / range`), so a quiet track cannot tell a first-catch box error from a size error. Estimate only when range or `fx` is changing (jink, zoom, closing). **Known calib is the reliable remove.**

| Knob | Where | What it does |
|---|---|---|
| `range_bias_sigma_m` | `Config` | `0` = do not estimate. `~1` = estimate a constant residual (m 1-sigma). |
| `range_bias_m` | `Meas` each frame | Known/calib offset (metres). Subtracted in the size model. Leave `0` if unknown. |

Angular size is read as `width_px = fx * size / (range + range_bias)`. `Estimate::range_m` is `|p_rel|` (bias removed). `Estimate::range_bias_m` is the estimated residual. Call `ekf.reset()` if you change mode on a live filter.

### 1. Raw (default)

```cpp
bbox_ekf::Config cfg;
cfg.range_bias_sigma_m = 0.0f;     // already 0
bbox_ekf::BBoxEkf ekf(cfg);
m.range_bias_m = 0;
ekf.push(m);
```

### 2. Estimate

Unknown offset; the filter tries to find it. Helps when own-motion or zoom changes range/`fx`. Can slightly worsen a quiet track — do not enable on a clean, smooth lock.

```cpp
bbox_ekf::Config cfg;
cfg.range_bias_sigma_m = 1.0f;     // initial 1-sigma, metres
cfg.range_bias_walk_m  = 0.0f;     // 0 = freeze as constant (first-catch)
bbox_ekf::BBoxEkf ekf(cfg);
m.range_bias_m = 0;                // unknown
ekf.push(m);
// now.range_m       — debiased geometry
// now.range_bias_m  — what it estimated
```

### 3. Known calib (best if you have the offset)

```cpp
bbox_ekf::Config cfg;
cfg.range_bias_sigma_m = 0.0f;     // do not also estimate a residual
bbox_ekf::BBoxEkf ekf(cfg);
m.range_bias_m = calib_range_m;    // bbox-implied range minus true, metres
ekf.push(m);
```

### 4. Known + estimate leftover

```cpp
bbox_ekf::Config cfg;
cfg.range_bias_sigma_m = 1.0f;
bbox_ekf::BBoxEkf ekf(cfg);
m.range_bias_m = calib_range_m;
ekf.push(m);
```

## Build

```bat
g++ -std=c++17 -O2 -c bbox_ekf.cpp -o bbox_ekf.o
```

Then link `bbox_ekf.o` into the other project.
