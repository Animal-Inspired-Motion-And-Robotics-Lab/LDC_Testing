#include "measurement_arrays.h"

#include <Arduino.h>

static constexpr size_t array_length = 500;

static float Rp_array[array_length] = {0.0f};
static float L_array[array_length] = {0.0f};
static size_t measurementCount = 0;

static constexpr size_t kMaxFilterWindow = 64;
static float rpRing[kMaxFilterWindow] = {0.0f};
static float lRing[kMaxFilterWindow] = {0.0f};
static size_t ringHead = 0;     // next write index
static size_t ringFilled = 0;   // valid entries, capped at kMaxFilterWindow
static size_t filterWindow = 1; // 1 = pass-through
static float latestFilteredRp = 0.0f;
static float latestFilteredL = 0.0f;

void setFilterWindow(size_t window) {
  if (window < 1) window = 1;
  if (window > kMaxFilterWindow) window = kMaxFilterWindow;
  filterWindow = window;
}

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

void appendMeasurement(float rp_ohms, float l_uH) {
  float rp_f, l_f;
  filterSample(rp_ohms, l_uH, rp_f, l_f);

  if (measurementCount < array_length) {
    Rp_array[measurementCount] = rp_f;
    L_array[measurementCount] = l_f;
    measurementCount++;
    return;
  }

  // Keep the newest array_length values by shifting left.
  for (size_t i = 1; i < array_length; ++i) {
    Rp_array[i - 1] = Rp_array[i];
    L_array[i - 1] = L_array[i];
  }

  Rp_array[array_length - 1] = rp_f;
  L_array[array_length - 1] = l_f;
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

float calculateDominantAngle(void) {
  // Compute means.
  float meanRp = 0.0f;
  float meanL = 0.0f;

  for (size_t i = 0; i < array_length; i++) {
    meanRp += Rp_array[i];
    meanL += L_array[i];
  }

  meanRp /= (float)array_length;
  meanL /= (float)array_length;

  // Compute covariance terms.
  float Sxx = 0.0f;
  float Syy = 0.0f;
  float Sxy = 0.0f;

  for (size_t i = 0; i < array_length; i++) {
    float dx = Rp_array[i] - meanRp;
    float dy = L_array[i] - meanL;

    Sxx += dx * dx;
    Syy += dy * dy;
    Sxy += dx * dy;
  }

  // Principal-axis angle.
  float theta = 0.5f * atan2f(2.0f * Sxy, Sxx - Syy);

  return theta; // radians
}