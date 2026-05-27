#include "crack_detection.h"

#include <math.h>

#include "measurement_arrays.h"

namespace {

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kTwoPi = 2.0f * kPi;
static constexpr float kPeakEpsilon = 1.0e-6f;
static constexpr float kDefaultMinParabolaR2 = 0.90f;

static crack_detection_config_t gConfig = {0.1f, 100, kDefaultMinParabolaR2,
                                           (kPi * 0.25f), kPi, 1.0f};
static bool gInitialized = false;
static bool gPreviousQualified = false;
static size_t gRefractoryRemaining = 0;

size_t computeRefractorySamples(float fitWidthSamples) {
  size_t widthBased = 1;
  if (isfinite(fitWidthSamples) && fitWidthSamples > 0.0f) {
    widthBased = static_cast<size_t>(ceilf(fitWidthSamples));
    if (widthBased < 1) {
      widthBased = 1;
    }
  }

  size_t windowBased = gConfig.window_samples / 4;
  if (windowBased < 1) {
    windowBased = 1;
  }

  return (widthBased > windowBased) ? widthBased : windowBased;
}

bool getPhaseAngleForWindow(size_t sampleCount, float* phaseAngleRad) {
  if (phaseAngleRad == nullptr) {
    return false;
  }

  if (sampleCount < 2) {
    return false;
  }

  // Use the same rolling window for phase qualification: oldest->newest delta.
  float newestRp = 0.0f;
  float newestL = 0.0f;
  float oldestRp = 0.0f;
  float oldestL = 0.0f;
  if (!getRecentRotatedSample(0, &newestRp, &newestL)) {
    return false;
  }
  if (!getRecentRotatedSample(sampleCount - 1, &oldestRp, &oldestL)) {
    return false;
  }

  *phaseAngleRad = atan2f(newestL - oldestL, newestRp - oldestRp);
  return true;
}

float normalizeAngleRad(float angleRad) {
  while (angleRad <= -kPi) {
    angleRad += kTwoPi;
  }
  while (angleRad > kPi) {
    angleRad -= kTwoPi;
  }
  return angleRad;
}

bool angleInRange(float angleRad, float minAngleRad, float maxAngleRad) {
  if (minAngleRad <= maxAngleRad) {
    return (angleRad >= minAngleRad) && (angleRad <= maxAngleRad);
  }
  return (angleRad >= minAngleRad) || (angleRad <= maxAngleRad);
}

bool fitParabolaForWindow(size_t sampleCount,
                          float* peakHeight,
                          float* peakXSamples,
                          float* halfPeakHeight,
                          float* widthSamples,
                          float* fitR2) {
  if (peakHeight == nullptr || peakXSamples == nullptr ||
      halfPeakHeight == nullptr ||
      widthSamples == nullptr || fitR2 == nullptr) {
    return false;
  }

  if (sampleCount < 3) {
    return false;
  }

  float rotationCenterRp = 0.0f;
  float rotationCenterL = 0.0f;
  getRotationCenter(&rotationCenterRp, &rotationCenterL);

  float sumX = 0.0f;
  float sumX2 = 0.0f;
  float sumX3 = 0.0f;
  float sumX4 = 0.0f;
  float sumY = 0.0f;
  float sumXY = 0.0f;
  float sumX2Y = 0.0f;

  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float x = static_cast<float>(i);
    const float y = l - rotationCenterL;
    const float x2 = x * x;

    sumX += x;
    sumX2 += x2;
    sumX3 += x2 * x;
    sumX4 += x2 * x2;
    sumY += y;
    sumXY += x * y;
    sumX2Y += x2 * y;
  }

  const float n = static_cast<float>(sampleCount);

  // Solve normal equations for y = a*x^2 + b*x + c.
  float m00 = sumX4;
  float m01 = sumX3;
  float m02 = sumX2;
  float m10 = sumX3;
  float m11 = sumX2;
  float m12 = sumX;
  float m20 = sumX2;
  float m21 = sumX;
  float m22 = n;

  float v0 = sumX2Y;
  float v1 = sumXY;
  float v2 = sumY;

  // Gaussian elimination (3x3).
  if (fabsf(m00) < kPeakEpsilon) {
    return false;
  }
  const float f10 = m10 / m00;
  const float f20 = m20 / m00;
  m10 -= f10 * m00;
  m11 -= f10 * m01;
  m12 -= f10 * m02;
  v1 -= f10 * v0;
  m20 -= f20 * m00;
  m21 -= f20 * m01;
  m22 -= f20 * m02;
  v2 -= f20 * v0;

  if (fabsf(m11) < kPeakEpsilon) {
    return false;
  }
  const float f21 = m21 / m11;
  m21 -= f21 * m11;
  m22 -= f21 * m12;
  v2 -= f21 * v1;

  if (fabsf(m22) < kPeakEpsilon) {
    return false;
  }

  const float c = v2 / m22;
  const float b = (v1 - m12 * c) / m11;
  const float a = (v0 - m01 * b - m02 * c) / m00;

  if (!(a < -kPeakEpsilon) || !isfinite(a) || !isfinite(b) || !isfinite(c)) {
    return false;
  }

  const float xVertex = -b / (2.0f * a);
  if (!(xVertex >= 0.0f) || !(xVertex <= (n - 1.0f)) || !isfinite(xVertex)) {
    return false;
  }

  const float fittedPeak = c - ((b * b) / (4.0f * a));
  if (!(fittedPeak > 0.0f) || !isfinite(fittedPeak)) {
    return false;
  }

  const float halfHeight = 0.5f * fittedPeak;
  const float halfWidthSquared = -fittedPeak / (2.0f * a);
  if (!(halfWidthSquared > 0.0f) || !isfinite(halfWidthSquared)) {
    return false;
  }

  const float fittedWidthSamples = 2.0f * sqrtf(halfWidthSquared);
  if (!(fittedWidthSamples > 0.0f) || !isfinite(fittedWidthSamples)) {
    return false;
  }

  float sst = 0.0f;
  float sse = 0.0f;
  const float meanY = sumY / n;
  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float x = static_cast<float>(i);
    const float y = l - rotationCenterL;
    const float yHat = (a * x * x) + (b * x) + c;
    const float err = y - yHat;
    const float dy = y - meanY;
    sse += err * err;
    sst += dy * dy;
  }

  float r2 = 0.0f;
  if (sst <= kPeakEpsilon) {
    r2 = (sse <= kPeakEpsilon) ? 1.0f : 0.0f;
  } else {
    r2 = 1.0f - (sse / sst);
  }
  if (!isfinite(r2)) {
    return false;
  }

  *peakHeight = fittedPeak;
  *peakXSamples = xVertex;
  *halfPeakHeight = halfHeight;
  *widthSamples = fittedWidthSamples;
  *fitR2 = r2;
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

  if (gConfig.min_parabola_r2 < 0.0f) {
    gConfig.min_parabola_r2 = 0.0f;
  }
  if (gConfig.min_parabola_r2 > 1.0f) {
    gConfig.min_parabola_r2 = 1.0f;
  }

  if (gConfig.min_phase_angle_rad > gConfig.max_phase_angle_rad) {
    const float tmp = gConfig.min_phase_angle_rad;
    gConfig.min_phase_angle_rad = gConfig.max_phase_angle_rad;
    gConfig.max_phase_angle_rad = tmp;
  }

  if (gConfig.length_estimate_scale < 0.0f) {
    gConfig.length_estimate_scale = 0.0f;
  }

  gPreviousQualified = false;
  gRefractoryRemaining = 0;
  gInitialized = true;
}

bool crackDetectionCheck(crack_detection_result_t* result) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (gRefractoryRemaining > 0) {
    --gRefractoryRemaining;
  }

  if (result != nullptr) {
    result->detected = false;
    result->fit_peak_height = 0.0f;
    result->fit_peak_x_samples = 0.0f;
    result->fit_half_peak_height = 0.0f;
    result->fit_width_samples = 0.0f;
    result->fit_r2 = 0.0f;
    result->reject_reason = nullptr;
  }

  float fitPeakHeight = 0.0f;
  float fitPeakXSamples = 0.0f;
  float fitHalfPeakHeight = 0.0f;
  float fitWidthSamples = 0.0f;
  float fitR2 = 0.0f;
  if (!fitParabolaForWindow(gConfig.window_samples,
                            &fitPeakHeight, &fitPeakXSamples,
                            &fitHalfPeakHeight,
                            &fitWidthSamples, &fitR2)) {
    // No fit at all — leave reject_reason nullptr (too noisy to report).
    gPreviousQualified = false;
    return false;
  }

  if (result != nullptr) {
    result->fit_peak_height = fitPeakHeight;
    result->fit_peak_x_samples = fitPeakXSamples;
    result->fit_half_peak_height = fitHalfPeakHeight;
    result->fit_width_samples = fitWidthSamples;
    result->fit_r2 = fitR2;
  }

  if (fitR2 < gConfig.min_parabola_r2) {
    if (result != nullptr) result->reject_reason = "low_r2";
    gPreviousQualified = false;
    return false;
  }
  if (fitPeakHeight < gConfig.threshold) {
    if (result != nullptr) result->reject_reason = "threshold";
    gPreviousQualified = false;
    return false;
  }

  float phaseAngleRad = NAN;
  if (!getPhaseAngleForWindow(gConfig.window_samples, &phaseAngleRad)) {
    if (result != nullptr) result->reject_reason = "no_phase";
    gPreviousQualified = false;
    return false;
  }

  const float normalizedPhase = normalizeAngleRad(phaseAngleRad);
  const float oppositePhase = normalizeAngleRad(normalizedPhase + kPi);
  const bool phaseQualified =
      angleInRange(normalizedPhase, gConfig.min_phase_angle_rad, gConfig.max_phase_angle_rad) ||
      angleInRange(oppositePhase, gConfig.min_phase_angle_rad, gConfig.max_phase_angle_rad);
  if (!phaseQualified) {
    // Both phase representatives missed the cone. Pick the one closer to the
    // cone and report which boundary it overshot.
    const float minAngle = gConfig.min_phase_angle_rad;
    const float maxAngle = gConfig.max_phase_angle_rad;
    auto missInfo = [minAngle, maxAngle](float angle, float* distance, bool* low) {
      if (angle < minAngle) { *distance = minAngle - angle; *low = true; }
      else                  { *distance = angle - maxAngle;  *low = false; }
    };
    float distNorm = 0.0f, distOpp = 0.0f;
    bool lowNorm = false, lowOpp = false;
    missInfo(normalizedPhase, &distNorm, &lowNorm);
    missInfo(oppositePhase, &distOpp, &lowOpp);
    const bool isLow = (distNorm <= distOpp) ? lowNorm : lowOpp;
    if (result != nullptr) result->reject_reason = isLow ? "phase_low" : "phase_high";
    gPreviousQualified = false;
    return false;
  }

  if (gRefractoryRemaining > 0) {
    if (result != nullptr) result->reject_reason = "refractory";
    gPreviousQualified = true;
    return false;
  }

  if (gPreviousQualified) {
    if (result != nullptr) result->reject_reason = "held";
    return false;
  }
  gPreviousQualified = true;
  gRefractoryRemaining = computeRefractorySamples(fitWidthSamples);

  if (result != nullptr) {
    result->detected = true;
    result->fit_peak_height = fitPeakHeight;
    result->fit_peak_x_samples = fitPeakXSamples;
    result->fit_half_peak_height = fitHalfPeakHeight;
    result->fit_width_samples = fitWidthSamples;
    result->fit_r2 = fitR2;
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

void crackDetectionSetMinParabolaR2(float min_parabola_r2) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_parabola_r2 < 0.0f) {
    min_parabola_r2 = 0.0f;
  }
  if (min_parabola_r2 > 1.0f) {
    min_parabola_r2 = 1.0f;
  }
  gConfig.min_parabola_r2 = min_parabola_r2;
}

float crackDetectionGetMinParabolaR2(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_parabola_r2;
}

void crackDetectionSetPhaseAngleRange(float min_phase_angle_rad, float max_phase_angle_rad) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_phase_angle_rad > max_phase_angle_rad) {
    const float tmp = min_phase_angle_rad;
    min_phase_angle_rad = max_phase_angle_rad;
    max_phase_angle_rad = tmp;
  }

  gConfig.min_phase_angle_rad = min_phase_angle_rad;
  gConfig.max_phase_angle_rad = max_phase_angle_rad;
}

float crackDetectionGetMinPhaseAngleRad(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_phase_angle_rad;
}

float crackDetectionGetMaxPhaseAngleRad(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.max_phase_angle_rad;
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
