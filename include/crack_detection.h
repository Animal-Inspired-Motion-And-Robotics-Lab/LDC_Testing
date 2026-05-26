#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  float threshold;
  size_t window_samples;
  size_t min_points;
  float length_estimate_scale;
} crack_detection_config_t;

typedef struct {
  bool detected;
  float crack_size;
  float current_window_max;
  size_t points_above_threshold;
  float total_length_estimate;
  uint32_t timestamp_ms;
} crack_detection_result_t;

void crackDetectionInit(const crack_detection_config_t* config);

// Returns true when a threshold-qualified window completes and emits a crack size.
bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result);

void crackDetectionSetWindowSamples(size_t window_samples);
size_t crackDetectionGetWindowSamples(void);

void crackDetectionSetThreshold(float threshold);
float crackDetectionGetThreshold(void);

void crackDetectionSetMinPoints(size_t min_points);
size_t crackDetectionGetMinPoints(void);

void crackDetectionSetLengthEstimateScale(float length_estimate_scale);
float crackDetectionGetLengthEstimateScale(void);

float crackDetectionGetTotalLengthEstimate(void);
void crackDetectionResetTotalLengthEstimate(void);

#endif
