#pragma once

// Headless auto-benchmark: generate scenarios, force the ideal gimbal so every
// estimator sees the same detections, replay all filters, write CSVs.
int run_auto_bench(float seconds, bool quick);
