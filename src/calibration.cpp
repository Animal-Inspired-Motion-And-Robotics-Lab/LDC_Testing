#include "calibration.h"
#include <Arduino.h>
#include <math.h>

#include "ldc1101.h"
#include "measurement_arrays.h"

calibration_result_t calibrationRun(float sensor_c_f, size_t requested_samples) {
  (void)sensor_c_f;

  const size_t sampleTarget = requested_samples == 0 ? 40 : requested_samples;
  size_t accepted = 0;
  float theta = calculateDominantAngleRecent(sampleTarget, &accepted);
  if (!isnan(theta)) {
    float centerRp = 0.0f;
    float centerL = 0.0f;
    if (getRecentMeasurementMean(sampleTarget, &centerRp, &centerL, nullptr)) {
      setRotationCenter(centerRp, centerL);
    }
    setRotationAngle(-theta);
    setRotationEnabled(true);
  }

  calibration_result_t result;
  result.dominant_angle_rad = theta;
  result.dominant_angle_deg = theta * (180.0f / (float)M_PI);
  result.samples_requested = sampleTarget;
  result.samples_accepted = accepted;

  return result;
}

void calibrationPrintResult(const calibration_result_t* result) {
  if (result == nullptr) {
    Serial.println("ERR calibration result is null");
    return;
  }

  Serial.print("calibrate done samples=");
  Serial.print((unsigned int)result->samples_requested);
  Serial.print(" accepted=");
  Serial.println((unsigned int)result->samples_accepted);

  Serial.print("dominant_angle_rad=");
  Serial.println(result->dominant_angle_rad, 6);
  Serial.print("dominant_angle_deg=");
  Serial.println(result->dominant_angle_deg, 3);
}
