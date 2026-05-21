#include "measurement_arrays.h"

#include <Arduino.h>
#include <math.h>

static constexpr size_t array_length = 1000;

static float Rp_array[array_length] = {0.0f};
static float L_array[array_length] = {0.0f};
static float rotatedRpArray[array_length] = {0.0f};
static float rotatedLArray[array_length] = {0.0f};
static size_t measurementCount = 0;

static constexpr size_t kMaxFilterWindow = 512;
static float rpRing[kMaxFilterWindow] = {0.0f};
static float lRing[kMaxFilterWindow] = {0.0f};
static size_t ringHead = 0;     // next write index
static size_t ringFilled = 0;   // valid entries, capped at kMaxFilterWindow
static size_t filterWindow = 25; // 1 = pass-through
static float latestFilteredRp = 0.0f;
static float latestFilteredL = 0.0f;
static float latestRotatedRp = 0.0f;
static float latestRotatedL = 0.0f;
static float rotationAngleRad = 0.0f;
static float rotationCenterRp = 0.0f;
static float rotationCenterL = 0.0f;
static bool rotationEnabled = false;

void setFilterWindow(size_t window) {
  if (window < 1) window = 1;
  if (window > kMaxFilterWindow) window = kMaxFilterWindow;
  filterWindow = window;
}

size_t getFilterWindow(void) { return filterWindow; }

static void filterSample(float rp_in, float l_in, float& rp_out, float& l_out) {
  rpRing[ringHead] = rp_in;
  lRing[ringHead] = l_in;
  ringHead = (ringHead + 1) % kMaxFilterWindow;
  if (ringFilled < kMaxFilterWindow) ringFilled++;

  size_t n = filterWindow < ringFilled ? filterWindow : ringFilled;
  float sumRp = 0.0f;
  float sumL = 0.0f;
  for (size_t k = 0; k < n; ++k) {
    size_t idx = (ringHead + kMaxFilterWindow - 1 - k) % kMaxFilterWindow;
    sumRp += rpRing[idx];
    sumL += lRing[idx];
  }
  rp_out = sumRp / (float)n;
  l_out = sumL / (float)n;
  latestFilteredRp = rp_out;
  latestFilteredL = l_out;
}

float getLatestFilteredRp(void) { return latestFilteredRp; }
float getLatestFilteredL(void) { return latestFilteredL; }
float getLatestRotatedRp(void) { return latestRotatedRp; }
float getLatestRotatedL(void) { return latestRotatedL; }

void rotateSample(float rp_in, float l_in, float angle_rad,
                  float& rp_out, float& l_out) {
  float c = cosf(angle_rad);
  float s = sinf(angle_rad);
  rp_out = rp_in * c - l_in * s;
  l_out = rp_in * s + l_in * c;
}

void setRotationAngle(float angle_rad) { rotationAngleRad = angle_rad; }
float getRotationAngle(void) { return rotationAngleRad; }
void setRotationCenter(float rp_center, float l_center) {
  rotationCenterRp = rp_center;
  rotationCenterL = l_center;
}
void getRotationCenter(float* rp_center, float* l_center) {
  if (rp_center != nullptr) {
    *rp_center = rotationCenterRp;
  }
  if (l_center != nullptr) {
    *l_center = rotationCenterL;
  }
}
void setRotationEnabled(bool enabled) { rotationEnabled = enabled; }
bool getRotationEnabled(void) { return rotationEnabled; }

void appendMeasurement(float rp_ohms, float l_uH) {
  float rp_f, l_f;
  filterSample(rp_ohms, l_uH, rp_f, l_f);

  if (rotationEnabled) {
    float rpShifted = rp_f - rotationCenterRp;
    float lShifted = l_f - rotationCenterL;
    rotateSample(rpShifted, lShifted, rotationAngleRad, latestRotatedRp, latestRotatedL);
    latestRotatedRp += rotationCenterRp;
    latestRotatedL += rotationCenterL;
  } else {
    latestRotatedRp = rp_f;
    latestRotatedL = l_f;
  }

  if (measurementCount < array_length) {
    Rp_array[measurementCount] = rp_f;
    L_array[measurementCount] = l_f;
    rotatedRpArray[measurementCount] = latestRotatedRp;
    rotatedLArray[measurementCount] = latestRotatedL;
    measurementCount++;
    return;
  }

  // Keep the newest array_length values by shifting left.
  for (size_t i = 1; i < array_length; ++i) {
    Rp_array[i - 1] = Rp_array[i];
    L_array[i - 1] = L_array[i];
    rotatedRpArray[i - 1] = rotatedRpArray[i];
    rotatedLArray[i - 1] = rotatedLArray[i];
  }

  Rp_array[array_length - 1] = rp_f;
  L_array[array_length - 1] = l_f;
  rotatedRpArray[array_length - 1] = latestRotatedRp;
  rotatedLArray[array_length - 1] = latestRotatedL;
}

void printMeasurementArrays(void) {
  Serial.print("Rp_array:");
  for (size_t i = 0; i < array_length; ++i) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(Rp_array[i], 3);
  }
  Serial.println();

  Serial.print("L_array:");
  for (size_t i = 0; i < array_length; ++i) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print(L_array[i], 6);
  }
  Serial.println();
}

size_t getMeasurementCount(void) {
  return measurementCount;
}

bool getRecentMeasurementMean(size_t requested_samples, float* mean_rp,
                              float* mean_l, size_t* used_samples) {
  if (measurementCount == 0 || mean_rp == nullptr || mean_l == nullptr) {
    if (used_samples != nullptr) {
      *used_samples = 0;
    }
    return false;
  }

  size_t sampleCount = requested_samples;
  if (sampleCount == 0 || sampleCount > measurementCount) {
    sampleCount = measurementCount;
  }

  size_t start = measurementCount - sampleCount;
  float sumRp = 0.0f;
  float sumL = 0.0f;
  for (size_t i = start; i < measurementCount; ++i) {
    sumRp += Rp_array[i];
    sumL += L_array[i];
  }

  *mean_rp = sumRp / (float)sampleCount;
  *mean_l = sumL / (float)sampleCount;
  if (used_samples != nullptr) {
    *used_samples = sampleCount;
  }
  return true;
}

bool getRecentRotatedDelta(size_t lookback_samples, float* delta_rp,
                           float* delta_l) {
  if (delta_rp == nullptr || delta_l == nullptr) {
    return false;
  }
  if (lookback_samples == 0) {
    return false;
  }
  if (measurementCount <= lookback_samples) {
    return false;
  }

  const size_t latestIdx = measurementCount - 1;
  const size_t olderIdx = latestIdx - lookback_samples;
  *delta_rp = rotatedRpArray[latestIdx] - rotatedRpArray[olderIdx];
  *delta_l = rotatedLArray[latestIdx] - rotatedLArray[olderIdx];
  return true;
}

float calculateDominantAngleRecent(size_t requested_samples, size_t* used_samples) {
  if (measurementCount == 0) {
    if (used_samples != nullptr) {
      *used_samples = 0;
    }
    return NAN;
  }

  size_t sampleCount = requested_samples;
  if (sampleCount == 0 || sampleCount > measurementCount) {
    sampleCount = measurementCount;
  }

  if (used_samples != nullptr) {
    *used_samples = sampleCount;
  }

  size_t start = measurementCount - sampleCount;

  // Compute means.
  float meanRp = 0.0f;
  float meanL = 0.0f;

  for (size_t i = start; i < measurementCount; i++) {
    meanRp += Rp_array[i];
    meanL += L_array[i];
  }

  meanRp /= (float)sampleCount;
  meanL /= (float)sampleCount;

  // Compute covariance terms.
  float covRpL = 0.0f;
  float varRp = 0.0f;

  for (size_t i = start; i < measurementCount; i++) {
    float dx = Rp_array[i] - meanRp;
    float dy = L_array[i] - meanL;

    varRp += dx * dx;
    covRpL += dx * dy;
  }

  if (varRp <= 0.0f) {
    return NAN;
  }

  // Trend angle of L as a function of Rp.
  float slope = covRpL / varRp;
  float theta = atanf(slope);

  return theta; // radians
}

float calculateDominantAngle(void) {
  return calculateDominantAngleRecent(0, nullptr);
}