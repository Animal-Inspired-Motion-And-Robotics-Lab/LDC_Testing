// Per-substrate calibration. Invoked by the `calibrate [N]` serial command.
// Captures the substrate's "no-crack" trend so subsequent samples can be
// rotated into a frame where crack excursions live on the L axis.

#include "calibration.h"
#include <Arduino.h>
#include <math.h>

#include "ldc1101.h"
#include "measurement_arrays.h"

// Run a one-shot calibration over the most recent `requested_samples` filtered
// samples (or 40 if 0 is passed):
//   1. Fit the dominant trend angle θ via least-squares slope of L vs Rp.
//   2. Compute the cloud's centroid (mean Rp, mean L) as the rotation center.
//   3. Store (−θ) as the rotation angle so the trend rotates onto the new x.
//   4. Enable rotation.
//
// `sensor_c_f` is accepted for forward-compatibility (e.g. if a future variant
// triggers a fresh ldc1101_read() during calibration) but is currently unused.
calibration_result_t calibrationRun(float sensor_c_f, size_t requested_samples) {
  (void)sensor_c_f;

  const size_t sampleTarget = requested_samples == 0 ? 40 : requested_samples;
  size_t accepted = 0;
  float theta = calculateDominantAngleRecent(sampleTarget, &accepted);
  if (!isnan(theta)) {
    // Center first so rotation about (mean_Rp, mean_L) puts the trend along
    // the new x-axis and crack excursions on the new L axis.
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

// Dump a human-readable summary of a calibration run to Serial. Called after
// `calibrate` so the operator sees both the angle and the sample counts.
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
