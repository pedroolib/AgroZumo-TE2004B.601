#pragma once

#include <stdint.h>

#define ALPHA_N_SENSORS   5
#define SENSOR_SPACING_M  0.015f   // distance between neighbor line sensors, measure it on your Zumo

// Calibration point: raw reading of one line sensor at a known distance from the white corner.
typedef struct { uint16_t reading; float radius_m; } gray_cal_t;

// Converts a raw reading to the distance from the white corner (m).
// cal[] must be sorted by reading. Interpolates linearly and clamps at both ends.
float gray_to_radius(uint16_t reading, const gray_cal_t *cal, int n_cal);

// Angle alpha (deg) between the sensor line S1..S5 and the tangent to the terrain arc at S3.
// r[i] = distance from the white corner to sensor S(i+1), in m.
// Positive when S5 is farther from the corner than S1.
float alpha_from_radii(const float r[ALPHA_N_SENSORS], float spacing_m);
