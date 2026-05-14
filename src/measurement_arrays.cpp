#include "measurement_arrays.h"

#include <Arduino.h>

static constexpr size_t array_length = 500;

static float Rp_array[array_length] = {0.0f};
static float L_array[array_length] = {0.0f};
static size_t measurementCount = 0;

void appendMeasurement(float rp_ohms, float l_uH) {
  if (measurementCount < array_length) {
    Rp_array[measurementCount] = rp_ohms;
    L_array[measurementCount] = l_uH;
    measurementCount++;
    return;
  }

  // Keep the newest array_length values by shifting left.
  for (size_t i = 1; i < array_length; ++i) {
    Rp_array[i - 1] = Rp_array[i];
    L_array[i - 1] = L_array[i];
  }

  Rp_array[array_length - 1] = rp_ohms;
  L_array[array_length - 1] = l_uH;
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