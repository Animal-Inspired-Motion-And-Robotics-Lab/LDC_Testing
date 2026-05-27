#ifndef MEASUREMENT_ARRAYS_H
#define MEASUREMENT_ARRAYS_H

#include <stdbool.h>
#include <stddef.h>

void appendMeasurement(float rp_ohms, float l_uH);
float calculateDominantAngleRecent(size_t requested_samples, size_t* used_samples);
bool getRecentMeasurementMean(size_t requested_samples, float* mean_rp,
							  float* mean_l, size_t* used_samples);

// Set the moving-average window applied to incoming samples before they are
// stored. window=1 disables smoothing. Values are clamped to [1, max].
void setFilterWindow(size_t window);
size_t getFilterWindow(void);

// Most recent filtered values written by appendMeasurement. Both return 0.0f
// before the first sample is appended.
float getLatestFilteredRp(void);
float getLatestFilteredL(void);

// Rotate an Rp/L sample using a 2D rotation matrix.
void rotateSample(float rp_in, float l_in, float angle_rad,
				  float& rp_out, float& l_out);

// Runtime rotation controls applied to filtered samples.
void setRotationAngle(float angle_rad);
float getRotationAngle(void);
void setRotationCenter(float rp_center, float l_center);
void getRotationCenter(float* rp_center, float* l_center);
void setRotationEnabled(bool enabled);
bool getRotationEnabled(void);

// Most recent rotated values derived from the latest filtered sample.
float getLatestRotatedRp(void);
float getLatestRotatedL(void);

// Get a rotated sample by age: 0 = latest, 1 = previous, etc.
// Returns false when samples_ago is outside available history.
bool getRecentRotatedSample(size_t samples_ago, float* rp, float* l);

#endif