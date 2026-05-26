#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  float min_vector_magnitude;
  float min_phase_angle_rad;
  float max_phase_angle_rad;
  uint32_t cooldown_ms;
  size_t window_samples;
  float length_estimate_scale;
} crack_detection_config_t;

typedef struct {
  bool detected;
  float vector_rp_ohms;
  float vector_l_uH;
  float vector_magnitude;
  float phase_angle_rad;
  float total_length_estimate;
  uint32_t timestamp_ms;
} crack_detection_result_t;

void crackDetectionInit(const crack_detection_config_t* config);

// Returns true when an up-left event is detected in rotated, filtered samples.
bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result);

void crackDetectionSetWindowSamples(size_t window_samples);
size_t crackDetectionGetWindowSamples(void);

void crackDetectionSetMinVectorMagnitude(float min_vector_magnitude);
float crackDetectionGetMinVectorMagnitude(void);

void crackDetectionSetLengthEstimateScale(float length_estimate_scale);
float crackDetectionGetLengthEstimateScale(void);

float crackDetectionGetTotalLengthEstimate(void);
void crackDetectionResetTotalLengthEstimate(void);

#endif
