#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stddef.h>

typedef struct {
  float dominant_angle_rad;
  float dominant_angle_deg;
  size_t samples_requested;
  size_t samples_accepted;
} calibration_result_t;

calibration_result_t calibrationRun(float sensor_c_f, size_t requested_samples);

void calibrationPrintResult(const calibration_result_t* result);

#endif
