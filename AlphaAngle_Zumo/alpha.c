#include <math.h>
#include "alpha.h"

float gray_to_radius(uint16_t reading, const gray_cal_t *cal, int n_cal) {
    if (reading <= cal[0].reading) return cal[0].radius_m;
    for (int i = 1; i < n_cal; i++) {
        if (reading <= cal[i].reading) {
            float f = (float)(reading - cal[i - 1].reading) / (float)(cal[i].reading - cal[i - 1].reading);
            return cal[i - 1].radius_m + f * (cal[i].radius_m - cal[i - 1].radius_m);
        }
    }
    return cal[n_cal - 1].radius_m;
}

// Sensor i sits at s_i = (i - 2) * spacing along the sensor line, with S3 at s = 0.
// With O the white corner, u the unit vector along the line and beta the angle
// between u and the radial direction at S3:
//   r_i^2 = r_3^2 + 2 s_i r_3 cos(beta) + s_i^2          (law of cosines)
// The tangent is 90 deg from the radial, so sin(alpha) = cos(beta).
// Least squares over the five sensors:
//   sin(alpha) = sum s_i (r_i^2 - r_3^2 - s_i^2) / (2 r_3 sum s_i^2)
float alpha_from_radii(const float r[ALPHA_N_SENSORS], float spacing_m) {
    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < ALPHA_N_SENSORS; i++) {
        float s = (float)(i - 2) * spacing_m;
        num += s * (r[i] * r[i] - r[2] * r[2] - s * s);
        den += s * s;
    }
    float sin_a = num / (2.0f * r[2] * den);
    if (sin_a > 1.0f) sin_a = 1.0f;
    if (sin_a < -1.0f) sin_a = -1.0f;
    return asinf(sin_a) * 180.0f / 3.14159265f;
}
