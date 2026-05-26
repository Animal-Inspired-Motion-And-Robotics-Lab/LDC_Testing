#include "crack_detection.h"

#include <math.h>

#include "measurement_arrays.h"

namespace {

static constexpr float kPi = 3.14159265358979323846f;
static crack_detection_config_t gConfig = {
  3.0f, 0.0f, kPi, 250, 50, 1.0f, 0.45f, 0.03f};
static bool gInitialized = false;
static uint32_t gLastDetectionMs = 0;
static float gTotalLengthEstimate = 0.0f;
static constexpr size_t kMinParabolaSamples = 7;
static constexpr float kPeakDropEpsilon = 1.0e-5f;

// Peak-tracking state machine.
enum CrackTrackingState { CD_IDLE = 0, CD_TRACKING };
static CrackTrackingState gTrackingState = CD_IDLE;
static float gPeakMagnitude = 0.0f;
static float gPeakVectorRp = 0.0f;
static float gPeakVectorL = 0.0f;
static float gPeakPhaseAngle = 0.0f;

bool getWindowVector(size_t sampleCount, float* vectorRp, float* vectorL) {
  if (vectorRp == nullptr || vectorL == nullptr || sampleCount < 2) {
    return false;
  }
  float oldestRp = 0.0f;
  float oldestL = 0.0f;
  float latestRp = 0.0f;
  float latestL = 0.0f;
  if (!getRecentRotatedSample(sampleCount - 1, &oldestRp, &oldestL)) {
    return false;
  }
  if (!getRecentRotatedSample(0, &latestRp, &latestL)) {
    return false;
  }
  *vectorRp = latestRp - oldestRp;
  *vectorL = latestL - oldestL;
  return true;
}

// Measure the strongest positive residual from the chord across the window.
// Also returns the sample location of that apex.
bool getParabolaHeightMetric(size_t sampleCount, float* height, float* fitR2,
                             float* peakRp, float* peakL, float* peakAngle) {
  if (height == nullptr || fitR2 == nullptr || peakRp == nullptr ||
      peakL == nullptr || peakAngle == nullptr ||
      sampleCount < kMinParabolaSamples) {
    return false;
  }

  float x0 = 0.0f;
  float y0 = 0.0f;
  float x1 = 0.0f;
  float y1 = 0.0f;
  if (!getRecentRotatedSample(sampleCount - 1, &x0, &y0)) {
    return false;
  }
  if (!getRecentRotatedSample(0, &x1, &y1)) {
    return false;
  }

  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float chordMag = sqrtf(dx * dx + dy * dy);
  if (!(chordMag > 0.0f)) {
    return false;
  }

  const float nx = -dy / chordMag;
  const float ny = dx / chordMag;
  const float center = 0.5f * (float)(sampleCount - 1);
  const float denom = (float)(sampleCount - 1);

  float s0 = 0.0f;
  float s1 = 0.0f;
  float s2 = 0.0f;
  float s3 = 0.0f;
  float s4 = 0.0f;
  float t0 = 0.0f;
  float t1 = 0.0f;
  float t2 = 0.0f;
  float sumResidual = 0.0f;
  float sumRp = 0.0f;
  float sumL = 0.0f;
  float maxResidual = -1.0e9f;

  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float alpha = ((float)i) / denom;
    const float baseX = x0 + alpha * dx;
    const float baseY = y0 + alpha * dy;
    const float residual = (rp - baseX) * nx + (l - baseY) * ny;
    const float x = (float)i - center;
    const float x2 = x * x;

    s0 += 1.0f;
    s1 += x;
    s2 += x2;
    s3 += x2 * x;
    s4 += x2 * x2;
    t0 += residual;
    t1 += x * residual;
    t2 += x2 * residual;
    sumResidual += residual;
    sumRp += rp;
    sumL += l;

    if (residual > maxResidual) {
      maxResidual = residual;
      *peakRp = rp;
      *peakL = l;
    }
  }

  const float det = s4 * (s2 * s0 - s1 * s1) -
                    s3 * (s3 * s0 - s1 * s2) +
                    s2 * (s3 * s1 - s2 * s2);
  if (fabsf(det) < 1.0e-9f) {
    return false;
  }

  const float detA = t2 * (s2 * s0 - s1 * s1) -
                     s3 * (t1 * s0 - s1 * t0) +
                     s2 * (t1 * s1 - s2 * t0);
  const float detB = s4 * (t1 * s0 - s1 * t0) -
                     t2 * (s3 * s0 - s1 * s2) +
                     s2 * (s3 * t0 - t1 * s2);
  const float detC = s4 * (s2 * t0 - t1 * s1) -
                     s3 * (s3 * t0 - t1 * s2) +
                     t2 * (s3 * s1 - s2 * s2);

  const float a = detA / det;
  const float b = detB / det;
  const float c = detC / det;
  const float meanResidual = sumResidual / (float)sampleCount;
  const float meanRp = sumRp / (float)sampleCount;
  const float meanL = sumL / (float)sampleCount;

  float sse = 0.0f;
  float sst = 0.0f;
  for (size_t i = 0; i < sampleCount; ++i) {
    float rp = 0.0f;
    float l = 0.0f;
    const size_t samplesAgo = sampleCount - 1 - i;
    if (!getRecentRotatedSample(samplesAgo, &rp, &l)) {
      return false;
    }

    const float alpha = ((float)i) / denom;
    const float baseX = x0 + alpha * dx;
    const float baseY = y0 + alpha * dy;
    const float residual = (rp - baseX) * nx + (l - baseY) * ny;
    const float x = (float)i - center;
    const float yhat = a * x * x + b * x + c;
    const float err = residual - yhat;
    const float dev = residual - meanResidual;

    sse += err * err;
    sst += dev * dev;
  }

  *fitR2 = (sst > 1.0e-9f) ? (1.0f - (sse / sst)) : 0.0f;
  *height = maxResidual;
  *peakAngle = atan2f(*peakL - meanL, *peakRp - meanRp);
  return true;
}

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

  if (gConfig.min_parabola_fit_r2 < 0.0f) {
    gConfig.min_parabola_fit_r2 = 0.0f;
  }
  if (gConfig.min_parabola_fit_r2 > 1.0f) {
    gConfig.min_parabola_fit_r2 = 1.0f;
  }

  if (gConfig.min_parabola_sharpness < 0.0f) {
    gConfig.min_parabola_sharpness = 0.0f;
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
    result->parabola_fit_r2 = NAN;
    result->parabola_sharpness = NAN;
    result->total_length_estimate = gTotalLengthEstimate;
    result->timestamp_ms = timestamp_ms;
  }

  size_t sampleCount = gConfig.window_samples;
  if (sampleCount < kMinParabolaSamples) {
    sampleCount = kMinParabolaSamples;
  }

  float vectorRp = 0.0f;
  float vectorL = 0.0f;
  if (!getWindowVector(sampleCount, &vectorRp, &vectorL)) {
    return false;
  }

  float parabolaHeight = 0.0f;
  float parabolaR2 = 0.0f;
  float parabolaPeakRp = 0.0f;
  float parabolaPeakL = 0.0f;
  float parabolaPeakAngle = 0.0f;
  if (!getParabolaHeightMetric(sampleCount, &parabolaHeight, &parabolaR2,
                               &parabolaPeakRp, &parabolaPeakL, &parabolaPeakAngle)) {
    return false;
  }

  float magnitude = sqrtf(vectorRp * vectorRp + vectorL * vectorL);
  float phaseAngle = parabolaPeakAngle;
  const float parabolaSharpness = parabolaHeight / (magnitude + 1.0e-6f);

  if (result != nullptr) {
    result->vector_rp_ohms = parabolaPeakRp;
    result->vector_l_uH = parabolaPeakL;
    // Expose fitted parabola height as event magnitude (crack size metric).
    result->vector_magnitude = parabolaHeight;
    result->phase_angle_rad = phaseAngle;
    result->parabola_fit_r2 = parabolaR2;
    result->parabola_sharpness = parabolaSharpness;
  }

  if (isnan(magnitude) || isnan(phaseAngle) || isnan(parabolaHeight) || isnan(parabolaR2)) {
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

  const bool aboveMagnitudeThreshold = (parabolaHeight >= gConfig.min_vector_magnitude);
  const bool inPhaseWindow = (phaseAngle >= gConfig.min_phase_angle_rad) &&
                             (phaseAngle <= gConfig.max_phase_angle_rad);
  const bool parabolaShapeOkay = (parabolaR2 >= gConfig.min_parabola_fit_r2) ||
                                 (parabolaSharpness >= gConfig.min_parabola_sharpness);

  if (gTrackingState == CD_IDLE) {
    // Enter tracking when the window vector first qualifies.
    if (aboveMagnitudeThreshold && inPhaseWindow && parabolaShapeOkay) {
      gTrackingState = CD_TRACKING;
      gPeakMagnitude = parabolaHeight;
      gPeakVectorRp = parabolaPeakRp;
      gPeakVectorL = parabolaPeakL;
      gPeakPhaseAngle = phaseAngle;
    }
    return false;
  }

  // CD_TRACKING: update peak while magnitude is still growing.
  if (parabolaHeight > (gPeakMagnitude + kPeakDropEpsilon) && inPhaseWindow && parabolaShapeOkay) {
    gPeakMagnitude = parabolaHeight;
    gPeakVectorRp = parabolaPeakRp;
    gPeakVectorL = parabolaPeakL;
    gPeakPhaseAngle = phaseAngle;
    return false;
  }

  if (parabolaHeight >= (gPeakMagnitude - kPeakDropEpsilon) && inPhaseWindow && parabolaShapeOkay) {
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

void crackDetectionSetPhaseWindow(float min_phase_angle_rad,
                                  float max_phase_angle_rad) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_phase_angle_rad > max_phase_angle_rad) {
    float tmp = min_phase_angle_rad;
    min_phase_angle_rad = max_phase_angle_rad;
    max_phase_angle_rad = tmp;
  }

  gConfig.min_phase_angle_rad = min_phase_angle_rad;
  gConfig.max_phase_angle_rad = max_phase_angle_rad;
}

void crackDetectionGetPhaseWindow(float* min_phase_angle_rad,
                                  float* max_phase_angle_rad) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_phase_angle_rad != nullptr) {
    *min_phase_angle_rad = gConfig.min_phase_angle_rad;
  }
  if (max_phase_angle_rad != nullptr) {
    *max_phase_angle_rad = gConfig.max_phase_angle_rad;
  }
}

void crackDetectionSetParabolaFitMinR2(float min_r2) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_r2 < 0.0f) {
    min_r2 = 0.0f;
  }
  if (min_r2 > 1.0f) {
    min_r2 = 1.0f;
  }
  gConfig.min_parabola_fit_r2 = min_r2;
}

float crackDetectionGetParabolaFitMinR2(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_parabola_fit_r2;
}

void crackDetectionSetParabolaSharpnessMin(float min_sharpness) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }

  if (min_sharpness < 0.0f) {
    min_sharpness = 0.0f;
  }
  gConfig.min_parabola_sharpness = min_sharpness;
}

float crackDetectionGetParabolaSharpnessMin(void) {
  if (!gInitialized) {
    crackDetectionInit(nullptr);
  }
  return gConfig.min_parabola_sharpness;
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
