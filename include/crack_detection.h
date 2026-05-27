#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  float threshold;
  size_t window_samples;
  float min_parabola_r2;
  float min_phase_angle_rad;
  float max_phase_angle_rad;
  float length_estimate_scale;
} crack_detection_config_t;

typedef struct {
  bool detected;
  float fit_peak_height;
  float fit_peak_x_samples;
  float fit_half_peak_height;
  float fit_width_samples;
  float fit_r2;
  // When `detected` is false and the parabola fit succeeded, this points to a
  // static string explaining which check rejected the window (e.g. "low_r2",
  // "threshold", "phase_low", "phase_high", "no_phase", "refractory", "held").
  // nullptr means no rejection reason (either detected, or no fit attempted).
  const char* reject_reason;
} crack_detection_result_t;

void crackDetectionInit(const crack_detection_config_t* config);

// Returns true when a threshold-qualified window completes and emits a crack size.
bool crackDetectionCheck(crack_detection_result_t* result);

void crackDetectionSetWindowSamples(size_t window_samples);
size_t crackDetectionGetWindowSamples(void);

void crackDetectionSetThreshold(float threshold);
float crackDetectionGetThreshold(void);

void crackDetectionSetMinParabolaR2(float min_parabola_r2);
float crackDetectionGetMinParabolaR2(void);

void crackDetectionSetPhaseAngleRange(float min_phase_angle_rad, float max_phase_angle_rad);
float crackDetectionGetMinPhaseAngleRad(void);
float crackDetectionGetMaxPhaseAngleRad(void);

void crackDetectionSetLengthEstimateScale(float length_estimate_scale);
float crackDetectionGetLengthEstimateScale(void);

#endif
