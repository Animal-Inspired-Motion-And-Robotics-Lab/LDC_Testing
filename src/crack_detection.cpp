#include "crack_detection.h"

#include "measurement_arrays.h"

namespace {

static crack_detection_config_t gConfig = {5, 3.0f, 0.010f, 1000};
static bool gInitialized = false;
static uint32_t gLastDetectionMs = 0;

}  // namespace

void crackDetectionInit(const crack_detection_config_t* config) {
  if (config != nullptr) {
    gConfig = *config;
  }

  if (gConfig.lookback_samples < 1) {
    gConfig.lookback_samples = 1;
  }

  if (gConfig.min_left_rp_ohms < 0.0f) {
    gConfig.min_left_rp_ohms = 0.0f;
  }

  if (gConfig.min_up_l_uH < 0.0f) {
    gConfig.min_up_l_uH = 0.0f;
  }

  gLastDetectionMs = 0;
  gInitialized = true;
}

bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  float deltaRp = 0.0f;
  float deltaL = 0.0f;
  if (!getRecentRotatedDelta(gConfig.lookback_samples, &deltaRp, &deltaL)) {
    return false;
  }

  // Up-left means L rises while Rp falls over the lookback horizon.
  const bool isUpLeft = (deltaL >= gConfig.min_up_l_uH) &&
                        (deltaRp <= -gConfig.min_left_rp_ohms);
  if (!isUpLeft) {
    return false;
  }

  const uint32_t elapsed = timestamp_ms - gLastDetectionMs;
  if (gLastDetectionMs != 0 && elapsed < gConfig.cooldown_ms) {
    return false;
  }

  gLastDetectionMs = timestamp_ms;
  if (result != nullptr) {
    result->detected = true;
    result->delta_rp_ohms = deltaRp;
    result->delta_l_uH = deltaL;
    result->timestamp_ms = timestamp_ms;
  }

  return true;
}
