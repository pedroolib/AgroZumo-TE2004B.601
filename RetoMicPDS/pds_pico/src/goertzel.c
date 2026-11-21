#include "goertzel.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void goertzel_init(goertzel_t *g, float freq_hz, float fs_hz) {
    g->freq_hz = freq_hz;
    g->coeff = 2.0f * cosf(2.0f * (float)M_PI * freq_hz / fs_hz);
    goertzel_reset(g);
}

void goertzel_reset(goertzel_t *g) {
    g->s1 = 0.0f;
    g->s2 = 0.0f;
}

float goertzel_power(const goertzel_t *g) {
    float p = g->s1 * g->s1 + g->s2 * g->s2 - g->coeff * g->s1 * g->s2;
    return p > 0.0f ? p : 0.0f;   /* evita negativos por redondeo */
}

float goertzel_amplitude(const goertzel_t *g, int n) {
    return 2.0f * sqrtf(goertzel_power(g)) / (float)n;
}

float goertzel_block(goertzel_t *g, const float *x, int n) {
    goertzel_reset(g);
    for (int i = 0; i < n; i++) goertzel_step(g, x[i]);
    return goertzel_power(g);
}
