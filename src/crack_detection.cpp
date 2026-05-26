#include "crack_detection.h"

#include "measurement_arrays.h"

namespace {

static crack_detection_config_t gConfig = {0.1f, 100, 25, 1.0f};
static bool gInitialized = false;
static float gTotalLengthEstimate = 0.0f;
static bool gWindowActive = false;
static float gTrackedMax = 0.0f;

bool getWindowStats(size_t sampleCount, float threshold,
                    size_t* pointsAboveThreshold, float* maxPointValue) {
  if (pointsAboveThreshold == nullptr || maxPointValue == nullptr) {
    return false;
  }
  if (sampleCount == 0) {
    return false;
  }
  float rotationCenterRp = 0.0f;
  float rotationCenterL = 0.0f;
  getRotationCenter(&rotationCenterRp, &rotationCenterL);

  *pointsAboveThreshold = 0;
  *maxPointValue = 0.0f;
  bool anyPoint = false;
  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float pointValue = l - rotationCenterL;
    if (!anyPoint || pointValue > *maxPointValue) {
      *maxPointValue = pointValue;
      anyPoint = true;
    }
    if (pointValue >= threshold) {
      (*pointsAboveThreshold)++;
    }
  }

  if (!anyPoint) {
    return false;
  }
  return true;
}

}  // namespace

void crackDetectionInit(const crack_detection_config_t* config) {
  if (config != nullptr) {
    gConfig = *config;
  }

  if (gConfig.threshold < 0.0f) {
    gConfig.threshold = 0.0f;
  }

  if (gConfig.window_samples < 1) {
    gConfig.window_samples = 1;
  }

  if (gConfig.min_points < 1) {
    gConfig.min_points = 1;
  }
  if (gConfig.min_points > gConfig.window_samples) {
    gConfig.min_points = gConfig.window_samples;
  }

  if (gConfig.length_estimate_scale < 0.0f) {
    gConfig.length_estimate_scale = 0.0f;
  }

  gTotalLengthEstimate = 0.0f;
  gWindowActive = false;
  gTrackedMax = 0.0f;
  gInitialized = true;
}

bool crackDetectionCheck(uint32_t timestamp_ms, crack_detection_result_t* result) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (result != nullptr) {
    result->detected = false;
    result->crack_size = 0.0f;
    result->current_window_max = 0.0f;
    result->points_above_threshold = 0;
    result->total_length_estimate = gTotalLengthEstimate;
    result->timestamp_ms = timestamp_ms;
  }

  size_t pointsAboveThreshold = 0;
  float windowMax = 0.0f;
  if (!getWindowStats(gConfig.window_samples, gConfig.threshold,
                      &pointsAboveThreshold, &windowMax)) {
    return false;
  }

  if (result != nullptr) {
    result->current_window_max = windowMax;
    result->points_above_threshold = pointsAboveThreshold;
  }

  const bool windowQualified = pointsAboveThreshold >= gConfig.min_points;
  if (windowQualified) {
    if (!gWindowActive) {
      gWindowActive = true;
      gTrackedMax = windowMax;
    } else if (windowMax > gTrackedMax) {
      gTrackedMax = windowMax;
    }
    return false;
  }

  if (!gWindowActive) {
    return false;
  }

  gWindowActive = false;
  gTotalLengthEstimate += (gTrackedMax * gConfig.length_estimate_scale);

  if (result != nullptr) {
    result->detected = true;
    result->crack_size = gTrackedMax;
    result->current_window_max = windowMax;
    result->points_above_threshold = pointsAboveThreshold;
    result->total_length_estimate = gTotalLengthEstimate;
    result->timestamp_ms = timestamp_ms;
  }

  gTrackedMax = 0.0f;

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
  if (gConfig.min_points > gConfig.window_samples) {
    gConfig.min_points = gConfig.window_samples;
  }
}

size_t crackDetectionGetWindowSamples(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.window_samples;
}

void crackDetectionSetThreshold(float threshold) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (threshold < 0.0f) {
    threshold = 0.0f;
  }
  gConfig.threshold = threshold;
}

float crackDetectionGetThreshold(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.threshold;
}

void crackDetectionSetMinPoints(size_t min_points) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_points < 1) {
    min_points = 1;
  }
  if (min_points > gConfig.window_samples) {
    min_points = gConfig.window_samples;
  }
  gConfig.min_points = min_points;
}

size_t crackDetectionGetMinPoints(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_points;
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
