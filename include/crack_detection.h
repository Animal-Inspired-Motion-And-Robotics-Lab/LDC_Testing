#ifndef CRACK_DETECTION_H
#define CRACK_DETECTION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  size_t lookback_samples;
  float min_left_rp_ohms;
  float min_up_l_uH;
  uint32_t cooldown_ms;
} crack_detection_config_t;

typedef struct {
  bool detected;
  float delta_rp_ohms;
  float delta_l_uH;
  uint32_t timestamp_ms;
} crack_detection_result_t;

void crackDetectionInit(const crack_detection_config_t* config);

// Returns true when an up-left event is detected in rotated, filtered samples.
bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result);

#endif
