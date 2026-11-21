/*
 * goertzel.h — Algoritmo de Goertzel (REQ-AUD-07)
 *
 * Referencia: Li Tan & Jean Jiang, "Digital Signal Processing: Fundamentals and
 * Applications", sección "Goertzel algorithm" (cap. de DFT/FFT).
 *
 * Para una frecuencia f, con w0 = 2*pi*f/Fs:
 *     v[n] = 2cos(w0) * v[n-1] - v[n-2] + x[n]          (filtro IIR de 2º orden)
 *     |X(k)|^2 = v[N-1]^2 + v[N-2]^2 - 2cos(w0) v[N-1] v[N-2]
 * Costo: 1 MAC por muestra por tono (vs. N log2 N de una FFT completa).
 */
#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <stdint.h>

typedef struct {
    float freq_hz;   /* frecuencia objetivo */
    float coeff;     /* 2*cos(2*pi*f/Fs) */
    float s1, s2;    /* estados v[n-1], v[n-2] */
} goertzel_t;

void  goertzel_init(goertzel_t *g, float freq_hz, float fs_hz);
void  goertzel_reset(goertzel_t *g);

/* Procesa una muestra (O(1)). */
static inline void goertzel_step(goertzel_t *g, float x) {
    float s0 = x + g->coeff * g->s1 - g->s2;
    g->s2 = g->s1;
    g->s1 = s0;
}

/* Potencia |X(k)|^2 al terminar un bloque de N muestras. */
float goertzel_power(const goertzel_t *g);

/* Amplitud estimada de la senoide (mismas unidades que x): A = 2|X(k)|/N */
float goertzel_amplitude(const goertzel_t *g, int n);

/* Procesa un bloque completo de muestras float (resetea antes). */
float goertzel_block(goertzel_t *g, const float *x, int n);

#endif
