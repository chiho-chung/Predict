# Drone chase simulation (C++)

Chaser drone with a gimbal camera tracking an enemy drone, with a slow detector
and a fast predictor — the setup for designing bounding-box prediction.

## Build / run

```bat
build.bat
build\drone_chase_sim.exe
build\drone_chase_sim.exe auto
```

Requires MSYS2 MinGW (`C:\msys64\ucrt64\bin\g++`). Links statically, so the exe
runs without MSYS on `PATH`.

| Mode | How | What it does |
|---|---|---|
| **Manual** | `build\drone_chase_sim.exe` | Same interactive windows as before. `E`/`W` now cycle bbox **and** LOS-only filters. |
| **Auto** | `build\drone_chase_sim.exe auto` | Headless. Generates scenarios, replays every estimator on the **same** detections, writes `logs/auto_<stamp>/`. |
| Auto (short) | `build\drone_chase_sim.exe auto --quick` | 4 scenarios × 8 s, for a smoke check. |
| Auto (duration) | `build\drone_chase_sim.exe auto --seconds 30` | Override run length (default 20 s). |

Headless A/B of the older estimator set (no window), useful when tuning:

```bat
tools\check.bat
```

## Experiment CSVs

Every run writes under `logs/`:

| File | What it is |
|---|---|
| `logs/experiments.csv` | append-only index: one row per experiment (config + RMS) |
| `logs/<stamp>_<kind>_<label>.csv` | timeseries, one row per sim step (100 Hz) |

`kind` is `live` (manual window), `check` (`tools\check.bat`), or `auto`. A new
live file starts when you change estimator, target motion, jitter, predict,
detect rate, delay, horizon, or hit `R`. Close the sim (Esc / X) to flush the
last summary row.

### Auto-bench CSVs

`drone_chase_sim.exe auto` writes a fresh folder `logs/auto_<stamp>/`:

| File | What it is |
|---|---|
| `scenarios.csv` | generated environments (motion, Hz, delay, bbox jitter, **frame timing**, spawn, seed) |
| `summary.csv` | one row per scenario × estimator (RMS after 2 s warmup) |
| `compare.csv` | bbox vs LOS-only paired on the **same** tape |
| `chosen_ekf.csv` | Export-EKF, Export-IMM, origin EKF, and IMM-EKF |
| `tapes/sXX_meas.csv` | shared detection tape, including per-frame timestamps |
| `runs/sXX_<estimator>.csv` | 100 Hz timeseries for that replay |

Auto mode forces the **ideal gimbal** so the box does not depend on which filter
is steering. Combined with a fixed spawn seed, every estimator on a scenario
sees identical measurements. LOS-only filters use the same centre / own-velocity
/ camera pose and ignore width and height.

Need `#include <cmath>` in csv_log for NAN - already there.

localtime_s on MinGW: I'll use a safer fallback in case compile fails.

Also add logs/ to a simple note - user might want gitignore. No git repo.

Build.

## Loop rates

| Loop | Rate | What runs |
|---|---|---|
| Physics + estimator | 100 Hz | target motion, chase controller, gimbal servo, `Tracker::at()` |
| Camera + bounding box | ~10 Hz, irregular | projection, bbox jitter, stamped `Tracker::push()` |

The camera is **not** a metronome. Nominal 10 Hz, but the inter-frame period
jitters (±20% by default), pipeline delay jitters (±10 ms), and the reported
timestamp has clock noise (±2 ms). Each frame still carries a timestamp.

Every predictor (Export-EKF first, origin EKF / IMM-EKF for A/B, and the rest)
uses that stamp, not `1/Hz`:

- update: predict from the last stamp to this stamp, then incorporate the box
- delay removed: coast from the last stamp to `t_now`
- future LOS: coast from the last stamp to `t_now + H`

Press `T` to turn timing jitter off and recover a stable 10 Hz for comparison.

## Views

- **Gimbal camera feed** (top) — true scale. A 30 cm drone at ~7 m is about
  15 px wide, so the boxes are small on purpose.
- **3D chase view** (bottom) — third-person, drones drawn oversized so they're
  visible from tens of meters back. Shows gimbal axis, camera FOV edges,
  altitude drop lines and line of sight.
- **Minimap** (panel) — top-down, centered on the chaser, 40 m across.
- **SETUP** (panel) — estimator, motion, chase, predict, jitter, timing,
  gimbal, detect rate, delay, zoom. Keys are listed next to the values.
- **ANALYSIS** (panel) — live box/range/size errors plus this-run RMS after 2 s.
- **History** (third window, `H`) — `logs/experiments.csv` index. Arrow keys
  pick a run, Enter loads its timeseries, F5 reloads the index. Closing it does
  not close the sim.

## LOS angle plots (second window)

A separate window plots the two angles a guidance loop actually consumes —
**attack angle** (LOS elevation) and **heading** (LOS azimuth) — for the same
target derived through five signal paths, the **error of each path against the
origin** for both angles, and the filter's **range** estimate against truth.
Closing this window does not close the sim.

| Trace | Color | How it is derived |
|---|---|---|
| origin (true) | gray/white | true box, current camera pose |
| delayed | orange | clean box held at the detector rate |
| jittered+delayed | cyan | measured (jittered) box held at the detector rate |
| delay removed | green | `predictor.at(now, 0)` |
| predicted +H | yellow | `predictor.at(now, H)` |

Each stale box is back-projected with the camera pose **it was captured with**,
so the delayed traces are true time-shifted copies of the origin rather than old
pixels misread through a new pose. All five start from a box center, so they
differ only by delay, jitter and prediction.

Sampled once per sim step (100 Hz), so the 10 Hz staircase is visible instead of
aliased. Heading is unwrapped, so it runs continuously past ±180° instead of
jumping.

### Panels

Five panels, each spanning the same 8 s window:

1. LOS attack angle, all five paths
2. **attack error vs origin**, the four non-origin paths
3. LOS heading, all five paths
4. **heading error vs origin**, the four non-origin paths
5. range: filter estimate vs truth

The error panels keep zero in view, mark it with a brighter line, and print the
**RMS of each visible trace** on the title row in that trace's colour, so the
paths can be ranked at a glance instead of eyeballed.

`F1`–`F5` hide a trace *and its error trace together*, which also drops it from
the autoscale. That matters here: the `+H` error is several times larger than the
rest, so with `F5` on the error panels span ±35° and with `F5` off they span ±3°
and the remaining traces become readable.

The three "now" paths (delayed, jittered+delayed, delay-removed) are scored
against the origin of the same sample. The `+H` path is not: it claims to
describe `t+H`, so scoring it against its own sample would only measure the
intended lead. Instead each `+H` sample is retired later, when the origin at
`t+H` arrives, which is why its error trace stops `H` seconds short of *now*.

### What the panels show

At the default 50 ms delay, typical RMS over the window (`F5` off):

| Path | Attack | Heading |
|---|---|---|
| delayed | 0.92° | 1.83° |
| jittered+delayed | 0.93° | 1.83° |
| delay removed | **0.51°** | **0.39°** |

The delayed figure is geometry, not a defect: LOS heading slews fast enough that
tens of ms of staleness is degrees of error. Jitter adds almost nothing on top of
the delay — the delayed and jittered traces sit on top of each other — which says
the dominant error here is *lateness, not noise*.

Raising the delay to 150 ms with `8` makes that concrete: the delayed heading
error grows from 1.83° to 7.87°, while delay-removed grows only to 2.70°, still
~3x better. Attack angle is the interesting exception — delay-removed goes to
2.55° versus 1.66° for the simply-delayed trace, i.e. **worse**. When the LOS
oscillates quickly and the delay is large, extrapolating that oscillation forward
amplifies error faster than accepting the lag does. Prediction is not free.

## Colors

| Color | Meaning |
|---|---|
| Cyan | measured box, 10 Hz, held between ticks |
| Gray | true box, 100 Hz (ground truth) |
| Green | predictor at `now` — detector latency removed |
| Yellow | predictor at `now + H`, and the future point on the map |
| Blue | chaser drone |
| Red | target drone |
| Olive | gimbal axis and FOV edges |

## Model

Shared by both drones: rectangular plate + 4 rotor disks, envelope
**30 x 30 x 5 cm**. The bounding box is fitted to the projected mesh (plate
corners + rotor rims), then centre jitter of a few **pixels** (typical 3–5 px)
and size jitter of **±10% of the current box**.

## Gimbal

The gimbal is a rate-limited servo (`GimbalConfig`) steered by the *tracker's*
pixel error, not by the target's true position. This matters: with an ideal
gimbal that snaps to truth every step, the box stays pinned to the image center
(~0.6 px off, 3 px/s) and there is nothing to predict. With the servo the box
moves at ~100 px/s, which is what makes prediction worth doing. Press `G` to
compare against the ideal gimbal.

## Controls

| Key | Action |
|-----|--------|
| `E` `W` | Cycle estimator forward / back |
| `M` | Cycle target motion (smooth / jink) |
| `C` | Toggle chase |
| `P` | Toggle prediction |
| `J` | Toggle bbox jitter |
| `T` | Toggle **frame-timing jitter** (period / latency / stamp noise) |
| `G` | Toggle ideal / servo gimbal |
| `1` `2` | Center jitter -/+ 1 px |
| `3` `4` | Size jitter -/+ 2% of box |
| `5` `6` | Detector rate -/+ 1 Hz |
| `7` `8` | **Delay (detector latency) -/+ 10 ms**, works in either window |
| `9` `0` | Zoom out / in (1x–4x; FOV = 70° / zoom) |
| `+` `-` | Predict horizon +/- 0.1 s |
| `H` | Open / raise the **history** window (review past experiments) |
| Arrows | Nudge target (sim). In history: move selection |
| `R` | Reset sim. In history: reload the index |
| `Esc` | Quit |
| `F1`-`F5` | Show/hide a trace and its error trace (plot window) |
| `F5` / Enter | History: reload index / load selected timeseries |

## Source layout

| Folder | What lives there |
|---|---|
| `src/app/` | `main.cpp`, auto-bench |
| `src/sim/` | world sim, drone/gimbal, shared types (`types.hpp`) |
| `src/estimator/` | bbox / LOS filters (`predictor.*`) |
| `src/ui/` | Win32 HUD, plot, history |
| `src/io/` | experiment CSV writer |
| `src/math/` | small matrix helpers (`linalg.hpp`) |

Include path is `-Isrc`. Headers are `"sim/sim.hpp"`, `"estimator/predictor.hpp"`, and so on.

## Estimator

`src/estimator/predictor.hpp` / `src/estimator/predictor.cpp` holds eleven selectable
estimators. Cycle them at runtime with `E` / `W`. **Export-EKF** is the default
(the drop-in `export/bbox_ekf` filter: world LOS + box size, no camera attitude).
**Export-IMM** is the 3-hypothesis IMM of that same model (`export/bbox_imm_ekf`).
Origin **EKF** and **IMM-EKF** stay in the cycle for comparison. All of them
consume the per-frame timestamp.

To drop a filter into another project, copy `export/bbox_ekf/` or
`export/bbox_imm_ekf/` (hpp + cpp). See that folder’s README for the API.

| Type | Measurement | What it is |
|---|---|---|
| `CV-pixel` | box centre + size (pixels) | image-space constant velocity (baseline) |
| `Export-EKF` | LOS + box size | **default.** export `BBoxEkf`, world heading/attack + fx |
| `Export-IMM` | LOS + box size | export `BBoxImmEkf`, quiet/mod/hard accel bank |
| `EKF` | bbox | origin 12-state pose-based filter (comparison) |
| `UKF` | bbox | same origin model, Julier unscented update (`kappa = 3`) |
| `IMM-EKF` | bbox | origin 3-hypothesis IMM, EKF updates (comparison) |
| `IMM-UKF` | bbox | 3 manoeuvre hypotheses, UKF updates |
| `EKF-LOS` | centre / LOS only | origin EKF, **no** width or height |
| `UKF-LOS` | centre / LOS only | origin UKF, **no** width or height |
| `IMM-EKF-LOS` | centre / LOS only | IMM-EKF without box size |
| `IMM-UKF-LOS` | centre / LOS only | IMM-UKF without box size |

LOS-only filters share the state vector (range and extent stay in the model)
but the update is 2-D. Range starts from `los_range_prior_m` (default 12 m)
and is tightened only by own-motion parallax — never by angular size. Use
auto mode to compare bbox vs LOS on identical data.

### What is measured, and what is inferred

Sensors, matching what the airframe can actually supply:

- own velocity in the nav frame (never own position)
- camera direction on the drone (body attitude + gimbal angles)
- the detector's bounding box: centre in pixels and size as ±10% of the box,
  jittered and late

From those, range, target velocity and target extent are *inferred*.

### State

```
x = [ p_rel(3) | v_target(3) | width_m | height_m | centre_bias_px(2) | size_frac(2) ]   (12 states)
```

`p_rel` is target-minus-camera, because own velocity is measurable but own
position is not. Own velocity therefore enters as a known input:

```
d(p_rel)/dt = v_target - v_own      d(v_target)/dt = accel noise
```

This is linear, so prediction is exact and identical for every hypothesis. The
camera projection is the only nonlinear part, which is why EKF and UKF differ
*only* in the update:

```
u      = cu + fx * x_c / z_c + b_u     w_px = fx * width_m  / z_c * (1 + b_w)
v      = cv - fy * y_c / z_c + b_v     h_px = fy * height_m / z_c * (1 + b_h)
```

`b_u`, `b_v` are additive pixels; `b_w`, `b_h` are fractions of the geometric
box (the detector’s ±10% size jitter).

Because the state is relative, the measurement needs the camera's *orientation*
only — no absolute camera position appears anywhere.

The last four states are a first-order Gauss-Markov model of the detector EMA
(`b_{k+1} = ρ b_k + w`, ρ = 0.6 per 100 ms): two centre-pixel states and two
size-fraction states. They are used in the **EKF** update only: the same states
in the UKF dump size residuals into the bias and range gets worse, so the UKF
keeps a geometry-only measurement. A low-pass in
front of either filter is **not** a substitute — it colors the residuals
further and adds lag. Measured (EKF, 30 s, smooth target):

| Treatment | estPx | losHdg | range RMS |
|---|---|---|---|
| none (white R on colored jitter) | 3.75 | 0.374° | 0.94 m |
| **Gauss-Markov bias (kept)** | **3.59** | **0.316°** | **0.80 m** |
| IIR pre-filter, α=0.4 | 8.48 | 0.748° | 2.07 m |
| IIR pre-filter, α=0.6 | 16.58 | 1.662° | 2.49 m |

The pre-filter more than doubles pointing error because it injects another
100–150 ms of lag on top of the 50 ms pipeline. The bias states do the opposite:
they absorb the correlated part of the box so `R` can describe only the white
leftover, which is what the gate and the IMM likelihood already assume.
Reprojection ignores the bias — it is detector error, not geometry.

### How range becomes observable

Angular size gives `width/range`, never `width` alone, so range and extent are
coupled and neither is observable from one frame. Two things break the tie: own
motion parallax (only while manoeuvring) and a prior on target extent. The prior
is on **the extent that spans the box**, not the 30 cm envelope — a square quad
seen from a random yaw spans `0.30*(|cos|+|sin|)`, averaging ~0.38 m. Centring
the prior on 0.30 m biased range low by the same fraction, which is why
`size_prior_m` defaults to 0.36.

Initial covariance is deliberately anisotropic: tight across the line of sight
(pixel noise) and loose along it (range rests on the prior).

### Latency

Detections are captured at `t`, queued, and handed over at `t + detect_latency_s`
(50 ms default) carrying the camera pose and own velocity *from the capture
instant*. The filter timestamps at capture and predicts forward to now, so it
removes both the 100 ms detector dead time and the pipeline latency rather than
smoothing over them. `7` / `8` adjust the latency.

### Measured results

30 s runs, detector 10 Hz + 50 ms latency, jitter on, `H = 0.5 s`, from
`tools\check.bat`. `estPx` is the delay-removed box centre against the true box
centre; `losHdg` is estimated LOS heading against true.

Smooth target:

| Estimator | estPx | losHdg | range RMS | extent RMS |
|---|---|---|---|---|
| CV-pixel | 7.72 | 0.780° | n/a | n/a |
| EKF | **3.59** | **0.316°** | 0.80 m | 0.036 m |
| UKF | 4.44 | 0.579° | 0.90 m | 0.031 m |
| IMM-EKF | 3.84 | 0.362° | 0.72 m | 0.033 m |
| IMM-UKF | 5.02 | 0.705° | **0.58 m** | **0.029 m** |

Jinking target:

| Estimator | estPx | losHdg | range RMS | extent RMS |
|---|---|---|---|---|
| CV-pixel | 9.04 | 1.130° | n/a | n/a |
| EKF | **4.86** | **0.513°** | 2.76 m | 0.140 m |
| UKF | 6.25 | 0.952° | **1.21 m** | **0.068 m** |
| IMM-EKF | 5.70 | 0.652° | 2.90 m | 0.141 m |
| IMM-UKF | 7.10 | 0.992° | 2.53 m | 0.130 m |

Against the image-space baseline the (whitened) EKF more than halves both pixel
and LOS error, and adds range, target velocity and extent, which the baseline
cannot produce at all.

**EKF vs UKF is a real trade, not a wash.** The EKF is better for pointing; the
UKF is much better for range while jinking (1.21 m vs 2.77 m, bias −0.78 m vs
−2.70 m). Range is where the `1/z` nonlinearity bites, so the unscented update
earns its cost there, while bearing is nearly linear and the EKF's tighter
update wins. Pick by what you need: pointing, or range.

### On the IMM: implemented, and measured not to pay off here

The IMM is a standard bank — mix, predict, update, reweight by likelihood — over
quiet / moderate / hard acceleration hypotheses. It does not beat a well-tuned
single filter in this configuration, and the reason is quantitative rather than a
coding fault:

At 10 Hz the divergence between a quiet and a manoeuvring hypothesis over one
update interval is about `0.5 * a * dt^2 = 0.5 * 7 * 0.01 ≈ 0.036 m`, which is
1.2 px at 7 m. Detector noise is ~3 px. The manoeuvre signature is *below the
noise*, so all residuals fit every hypothesis, the likelihood collapses to the
`1/sqrt(det S)` normalisation, and that always prefers the tightest covariance —
the quietest model. The bank then behaves like a single filter using its quietest
sigma, while still paying the mixing variance.

The bank itself is correct. Forcing a quiet-first set (1.5/5/12) and giving it a
clean detector shows the weights moving exactly as they should:

| Condition | smooth mix | jinking mix |
|---|---|---|
| as configured (3 px, 10 Hz) | 0.94 / 0.04 / 0.02 | 0.92 / 0.07 / 0.01 |
| clean detector (0.5 px) | 0.94 / 0.04 / 0.02 | 0.90 / 0.09 / 0.01 |
| **clean + slow (0.5 px, 4 Hz)** | **0.77 / 0.16 / 0.06** | **0.15 / 0.77 / 0.08** |
| clean + fast (0.5 px, 25 Hz) | 0.97 / 0.02 / 0.01 | 0.97 / 0.03 / 0.01 |

At 4 Hz the interval is long enough that the manoeuvre finally exceeds the noise
and the bank switches decisively to the manoeuvre model. Note the direction:
a *faster* detector makes discrimination worse, because the divergence shrinks as
`dt^2` while the noise stays put. So IMM here needs a low-noise detector or a
long update interval; use it when the target genuinely switches between quiet
flight and hard turns, and prefer a single well-tuned filter otherwise.

There is also a tension worth knowing: a quiet model responsive enough to track
well is a quiet model that already absorbs the manoeuvre, so it can no longer be
told apart from the manoeuvre model. Tuning for tracking and tuning for
discrimination pull in opposite directions.

### Process noise

`sigma_accel` defaults to 8 m/s², well above the target's real 2–7 m/s². At a
10 Hz detector the extra responsiveness beats the extra variance, and the sweep
is flat from 8 to 12:

| `sigma_accel` | 1 | 2 | 3 | 5 | 8 | 12 | 20 |
|---|---|---|---|---|---|---|---|
| estPx, smooth | 5.67 | 4.59 | 4.22 | 3.86 | **3.75** | 3.91 | 4.49 |
| estPx, jinking | 11.90 | 8.05 | 6.71 | 5.64 | 5.10 | **4.92** | 4.91 |

Reproduce any of the tables with `tools\check.bat` (headless, no window needed).
`tools\grab.ps1 -Which plot -Out build\shot.png` screenshots a running window.

### Known limits

- Detector jitter is correlated (EMA, `smooth = 0.6`). Size jitter is ±10% of
  the current box; the EKF models that as a Gauss-Markov fraction so the leftover
  innovation is white. The UKF still treats `R` as white, which is slightly
  wrong on purpose.
- The `+H` box is reprojected through the *current* camera pose because the
  future pose is unknown.
- The chase controller still steers on truth; only the gimbal and the estimator
  run closed-loop on measurements.
- Range degrades gracefully rather than silently: at 40 m a 30 cm target is ~3 px
  wide, angular size stops carrying information, and the reported
  `range_sigma_m` widens to ±10 m to say so.
