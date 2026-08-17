#include "bench.hpp"
#include "csv_log.hpp"
#include "history.hpp"
#include "plot.hpp"
#include "predictor.hpp"
#include "render.hpp"
#include "sim.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace {

int run_manual() {
  SimConfig cfg;
  cfg.camera.width = 480;
  cfg.camera.height = 360;
  cfg.camera.fov_wide_deg = 70.0f;
  cfg.camera.set_zoom(1.0f);

  cfg.rates.sim_hz = 100.0f;           // physics + estimator
  cfg.rates.detect_hz = 10.0f;         // camera feed + bounding box
  cfg.rates.detect_latency_s = 0.05f;  // capture -> available, set with 7/8

  cfg.predict.enabled = true;
  cfg.predict.horizon_s = 0.5f;
  cfg.predict.vel_smooth = 0.5f;

  cfg.jitter.enabled = true;
  cfg.jitter.center_px = 4.0f;
  cfg.jitter.size_px = 4.0f;
  cfg.jitter.smooth = 0.6f;

  cfg.chase_enabled = true;

  Simulation sim(cfg);
  Renderer renderer(cfg.camera.width, cfg.camera.height);
  if (!renderer.create_window("Drone Chase Sim - 10Hz detect / 100Hz predict")) {
    return 1;
  }

  PlotWindow plot;
  plot.create_window("LOS angles + error vs origin + range");

  HistoryWindow hist;
  hist.create_window("History — experiment index + trace review");

  ExperimentLog exp;
  auto exp_label = [](const SimConfig& c) {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "%s_%s_H%.2f",
                  estimator_name(c.tracker.type),
                  c.target.maneuver == TargetManeuver::Jink ? "jink" : "smooth",
                  c.predict.horizon_s);
    return std::string(buf);
  };
  auto same_exp = [](const SimConfig& a, const SimConfig& b) {
    return a.tracker.type == b.tracker.type &&
           a.target.maneuver == b.target.maneuver &&
           a.jitter.enabled == b.jitter.enabled &&
           a.predict.enabled == b.predict.enabled &&
           std::fabs(a.rates.detect_hz - b.rates.detect_hz) < 0.05f &&
           std::fabs(a.rates.detect_latency_s - b.rates.detect_latency_s) <
               0.005f &&
           std::fabs(a.predict.horizon_s - b.predict.horizon_s) < 0.05f &&
           a.timing.enabled == b.timing.enabled &&
           std::fabs(a.camera.zoom - b.camera.zoom) < 0.05f;
  };
  SimConfig exp_cfg = sim.config();
  exp.begin("live", exp_label(exp_cfg).c_str(), exp_cfg);
  float last_t = 0;

  using clock = std::chrono::steady_clock;
  const float fixed_dt = cfg.sim_dt();
  auto prev = clock::now();
  float accumulator = 0.0f;

  while (renderer.process_events(sim)) {
    // The plot window has its own message queue, so its delay keys arrive here
    // rather than through the sim window.
    if (renderer.take_history()) hist.show();
    if (const int steps_lat = plot.take_latency_steps()) {
      sim.adjust_detect_latency(0.01f * static_cast<float>(steps_lat));
    }

    const auto now = clock::now();
    float frame_dt = std::chrono::duration<float>(now - prev).count();
    prev = now;
    if (frame_dt > 0.10f) frame_dt = 0.10f;  // don't chase a long stall
    accumulator += frame_dt;

    int steps = 0;
    while (accumulator >= fixed_dt && steps < 32) {
      sim.step(fixed_dt);
      const SimConfig& now_cfg = sim.config();
      const auto& snap = sim.snapshot();
      // New file when the user changes the experiment (estimator, target,
      // delay, …) or hits reset (time jumps backward).
      if (snap.time + 1e-4f < last_t || !same_exp(exp_cfg, now_cfg)) {
        exp.end();
        exp_cfg = now_cfg;
        exp.begin("live", exp_label(exp_cfg).c_str(), exp_cfg);
        if (hist.alive()) hist.reload();
      }
      last_t = snap.time;
      exp.sample(now_cfg, snap);
      // Sampled per sim step, so the 10 Hz staircase is visible rather than
      // aliased by the render rate.
      plot.push(snap, now_cfg);
      renderer.note_sample(snap);
      accumulator -= fixed_dt;
      ++steps;
    }

    renderer.draw(sim.snapshot(), sim.config());
    renderer.present();

    if (plot.alive()) {
      plot.draw(sim.config());
      plot.present();
    }
    if (hist.alive()) {
      hist.draw();
      hist.present();
    }
    // Win32 Sleep, not std::this_thread: MinGW's static winpthread teardown
    // aborts on exit and the console reports 0xFFFFFFFF.
    Sleep(2);
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  bool auto_mode = false;
  bool quick = false;
  float seconds = 20.0f;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "auto") == 0 ||
        std::strcmp(argv[i], "--auto") == 0) {
      auto_mode = true;
    } else if (std::strcmp(argv[i], "--quick") == 0) {
      quick = true;
      if (seconds > 8.0f) seconds = 8.0f;
    } else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
      seconds = static_cast<float>(std::atof(argv[++i]));
    } else if (std::strcmp(argv[i], "--help") == 0 ||
               std::strcmp(argv[i], "-h") == 0) {
      std::printf(
          "usage:\n"
          "  drone_chase_sim.exe              manual (windows)\n"
          "  drone_chase_sim.exe auto         auto-bench, write logs/auto_*\n"
          "  drone_chase_sim.exe auto --quick shorter (4 scenarios, 8 s)\n"
          "  drone_chase_sim.exe auto --seconds 30\n"
          "\n"
          "manual windows:\n"
          "  sim     camera + 3D + SETUP / ANALYSIS panel\n"
          "  plot    LOS / error / range traces\n"
          "  history experiment index (H). Up/Down, Enter load, F5 reload\n");
      return 0;
    }
  }

  if (auto_mode) return run_auto_bench(seconds, quick);
  return run_manual();
}
