#ifndef MEASUREMENT_ARRAYS_H
#define MEASUREMENT_ARRAYS_H

#include <stddef.h>

void appendMeasurement(float rp_ohms, float l_uH);
void printMeasurementArrays(void);
float calculateDominantAngle(void);

// Set the moving-average window applied to incoming samples before they are
// stored. window=1 disables smoothing. Values are clamped to [1, max].
void setFilterWindow(size_t window);

// Most recent filtered values written by appendMeasurement. Both return 0.0f
// before the first sample is appended.
float getLatestFilteredRp(void);
float getLatestFilteredL(void);

#endif