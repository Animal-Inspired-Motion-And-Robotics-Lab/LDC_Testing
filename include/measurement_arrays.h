#ifndef MEASUREMENT_ARRAYS_H
#define MEASUREMENT_ARRAYS_H

#include <stddef.h>

void appendMeasurement(float rp_ohms, float l_uH);
void printMeasurementArrays(void);
float calculateDominantAngle(void);

#endif