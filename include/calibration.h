// One-shot, per-substrate rotation calibration. Captures the trend direction
// of the (Rp, L) cloud and installs a rotation that lays it flat along the
// x-axis so crack excursions live on the rotated L axis.

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stddef.h>

// Report returned by calibrationRun() and printed by calibrationPrintResult().
typedef struct {
  float dominant_angle_rad;   // Trend angle θ (NaN on degenerate cloud).
  float dominant_angle_deg;   // Same value in degrees, for human readability.
  size_t samples_requested;   // How many samples the caller asked for.
  size_t samples_accepted;    // How many were actually available / used.
} calibration_result_t;

// Run a calibration and (on success) install the resulting rotation
// parameters into measurement_arrays. Returns the report; does not print.
calibration_result_t calibrationRun(float sensor_c_f, size_t requested_samples);

// Pretty-print a calibration report over Serial.
void calibrationPrintResult(const calibration_result_t* result);

#endif
