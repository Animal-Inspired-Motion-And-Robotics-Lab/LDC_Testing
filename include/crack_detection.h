// Two-stage crack detector built on top of the rotated measurement stream.
// See crack_detection.cpp for the full algorithm description and pipeline.

#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Tuning knobs. All fields are runtime-mutable via the serial CLI.
typedef struct {
  float threshold;              // Minimum fitted parabola peak height (above baseline).
  size_t window_samples;        // Rolling window length used for fit + planarity.
  float min_parabola_r2;        // Minimum R² of the parabola fit (0..1).
  float min_planar_angle_rad;   // Min planar angle = atan2(stdL, stdRp) over window,
                                // in [0, π/2]. π/2 = pure L motion (t-L planar);
                                // 0 = pure Rp motion. Higher = stricter.
  float max_rp_l_range_ratio;   // Stage 2b cap: reject if range(rotated Rp)
                                // > this · range(rotated L) over the window.
                                // Catches material transitions that sweep Rp
                                // monotonically while L peaks symmetrically —
                                // a case the Pearson planar check misses
                                // because cov(Rp, L) ≈ 0. 0 disables the cap;
                                // higher = looser.
  float length_estimate_scale;  // Scales fit peak height into a length estimate.
} crack_detection_config_t;

// Per-tick result published by crackDetectionCheck(). Fit values are populated
// whenever a parabola fit was possible — even on rejection — so the debug
// stream can show what the fit looked like. On detection, reject_reason is
// nullptr; on rejection it points to a static reason string.
typedef struct {
  bool detected;
  float fit_peak_height;       // Vertex height above the rotated baseline.
  float fit_peak_x_samples;    // Vertex location, 0..window_samples-1 (oldest→newest).
  float fit_half_peak_height;  // = fit_peak_height / 2 (kept for debug).
  float fit_width_samples;     // Full width where the parabola crosses zero.
  float fit_r2;                // Coefficient of determination of the fit.
  // When `detected` is false and the parabola fit succeeded, this points to a
  // static string explaining which check rejected the window (e.g. "low_r2",
  // "threshold", "not_planar", "no_planar", "refractory", "held").
  // nullptr means no rejection reason (either detected, or no fit attempted).
  const char* reject_reason;
} crack_detection_result_t;

// Adopt `config` (or, if null, just re-clamp the current values) and reset the
// dedup / refractory state machine.
void crackDetectionInit(const crack_detection_config_t* config);

// Per-tick check. Returns true exactly when a new detection fires (not on a
// "held" continuation of the same event). Always populates `result` if given.
bool crackDetectionCheck(crack_detection_result_t* result);

// Runtime tuning. Each set() clamps to the valid range; each get() returns the
// post-clamp value currently in use. All paired with a `crack_*` serial command.
void crackDetectionSetWindowSamples(size_t window_samples);
size_t crackDetectionGetWindowSamples(void);

void crackDetectionSetThreshold(float threshold);
float crackDetectionGetThreshold(void);

void crackDetectionSetMinParabolaR2(float min_parabola_r2);
float crackDetectionGetMinParabolaR2(void);

// Minimum planar angle the Stage-2 check requires. The planar angle is
// atan2(stdL, stdRp) over the rotated samples in the window — a measure of how
// much of the parabola's 3D extent lies along L vs Rp. Clamped to [0, π/2].
void crackDetectionSetMinPlanarAngleRad(float min_planar_angle_rad);
float crackDetectionGetMinPlanarAngleRad(void);

// Max allowed ratio of range(rotated Rp) to range(rotated L) over the Stage-2
// window. Stage 2b reject fires when the ratio exceeds this value. Clamped
// to >= 0; 0 disables the check entirely.
void crackDetectionSetMaxRpLRangeRatio(float max_rp_l_range_ratio);
float crackDetectionGetMaxRpLRangeRatio(void);

void crackDetectionSetLengthEstimateScale(float length_estimate_scale);
float crackDetectionGetLengthEstimateScale(void);

#endif
