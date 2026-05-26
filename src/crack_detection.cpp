#include "crack_detection.h"

#include <math.h>

#include "measurement_arrays.h"

namespace {

static constexpr float kPi = 3.14159265358979323846f;
static crack_detection_config_t gConfig = {3.0f, (kPi * 0.5f), kPi, 1000, 50, 1.0f};
static bool gInitialized = false;
static uint32_t gLastDetectionMs = 0;
static float gTotalLengthEstimate = 0.0f;

// Peak-tracking state machine.
enum CrackTrackingState { CD_IDLE = 0, CD_TRACKING };
static CrackTrackingState gTrackingState = CD_IDLE;
static float gPeakMagnitude = 0.0f;
static float gPeakVectorRp = 0.0f;
static float gPeakVectorL = 0.0f;
static float gPeakPhaseAngle = 0.0f;

}  // namespace

void crackDetectionInit(const crack_detection_config_t* config) {
  if (config != nullptr) {
    gConfig = *config;
  }

  if (gConfig.min_vector_magnitude < 0.0f) {
    gConfig.min_vector_magnitude = 0.0f;
  }

  if (gConfig.min_phase_angle_rad > gConfig.max_phase_angle_rad) {
    float tmp = gConfig.min_phase_angle_rad;
    gConfig.min_phase_angle_rad = gConfig.max_phase_angle_rad;
    gConfig.max_phase_angle_rad = tmp;
  }

  if (gConfig.window_samples < 1) {
    gConfig.window_samples = 1;
  }

  if (gConfig.length_estimate_scale < 0.0f) {
    gConfig.length_estimate_scale = 0.0f;
  }

  gLastDetectionMs = 0;
  gTotalLengthEstimate = 0.0f;
  gTrackingState = CD_IDLE;
  gPeakMagnitude = 0.0f;
  gPeakVectorRp = 0.0f;
  gPeakVectorL = 0.0f;
  gPeakPhaseAngle = 0.0f;
  gInitialized = true;
}

bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (result != nullptr) {
    result->detected = false;
    result->vector_rp_ohms = 0.0f;
    result->vector_l_uH = 0.0f;
    result->vector_magnitude = 0.0f;
    result->phase_angle_rad = NAN;
    result->total_length_estimate = gTotalLengthEstimate;
    result->timestamp_ms = timestamp_ms;
  }

  float vectorRp = 0.0f;
  float vectorL = 0.0f;
  if (!getRecentRotatedDelta(gConfig.window_samples, &vectorRp, &vectorL)) {
    return false;
  }

  float magnitude = sqrtf(vectorRp * vectorRp + vectorL * vectorL);
  float phaseAngle = atan2f(vectorL, vectorRp);

  if (result != nullptr) {
    result->vector_rp_ohms = vectorRp;
    result->vector_l_uH = vectorL;
    result->vector_magnitude = magnitude;
    result->phase_angle_rad = phaseAngle;
  }

  if (isnan(magnitude) || isnan(phaseAngle)) {
    return false;
  }

  // Enforce post-detection cooldown; reset any in-progress tracking.
  if (gLastDetectionMs != 0) {
    const uint32_t elapsed = timestamp_ms - gLastDetectionMs;
    if (elapsed < gConfig.cooldown_ms) {
      gTrackingState = CD_IDLE;
      return false;
    }
  }

  const bool aboveMagnitudeThreshold = (magnitude >= gConfig.min_vector_magnitude);
  const bool inPhaseWindow = (phaseAngle >= gConfig.min_phase_angle_rad) &&
                             (phaseAngle <= gConfig.max_phase_angle_rad);

  if (gTrackingState == CD_IDLE) {
    // Enter tracking when the window vector first qualifies.
    if (aboveMagnitudeThreshold && inPhaseWindow) {
      gTrackingState = CD_TRACKING;
      gPeakMagnitude = magnitude;
      gPeakVectorRp = vectorRp;
      gPeakVectorL = vectorL;
      gPeakPhaseAngle = phaseAngle;
    }
    return false;
  }

  // CD_TRACKING: update peak while magnitude is still growing.
  if (magnitude > gPeakMagnitude) {
    gPeakMagnitude = magnitude;
    gPeakVectorRp = vectorRp;
    gPeakVectorL = vectorL;
    gPeakPhaseAngle = phaseAngle;
    return false;
  }

  // Magnitude has started to decrease — fire detection with the stored peak.
  gTrackingState = CD_IDLE;
  gLastDetectionMs = timestamp_ms;
  gTotalLengthEstimate += (gPeakMagnitude * gConfig.length_estimate_scale);

  if (result != nullptr) {
    result->detected = true;
    result->vector_rp_ohms = gPeakVectorRp;
    result->vector_l_uH = gPeakVectorL;
    result->vector_magnitude = gPeakMagnitude;
    result->phase_angle_rad = gPeakPhaseAngle;
    result->total_length_estimate = gTotalLengthEstimate;
    result->timestamp_ms = timestamp_ms;
  }

  return true;
}

void crackDetectionSetWindowSamples(size_t window_samples) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (window_samples < 1) {
    window_samples = 1;
  }
  gConfig.window_samples = window_samples;
}

size_t crackDetectionGetWindowSamples(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.window_samples;
}

void crackDetectionSetMinVectorMagnitude(float min_vector_magnitude) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_vector_magnitude < 0.0f) {
    min_vector_magnitude = 0.0f;
  }
  gConfig.min_vector_magnitude = min_vector_magnitude;
}

float crackDetectionGetMinVectorMagnitude(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_vector_magnitude;
}

void crackDetectionSetLengthEstimateScale(float length_estimate_scale) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (length_estimate_scale < 0.0f) {
    length_estimate_scale = 0.0f;
  }
  gConfig.length_estimate_scale = length_estimate_scale;
}

float crackDetectionGetLengthEstimateScale(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.length_estimate_scale;
}

float crackDetectionGetTotalLengthEstimate(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gTotalLengthEstimate;
}

void crackDetectionResetTotalLengthEstimate(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  gTotalLengthEstimate = 0.0f;
}
