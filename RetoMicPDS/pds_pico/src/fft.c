#include "fft.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void fft_init(fft_t *f, int n, int log2n, float *tw_re, float *tw_im, float *win) {
    f->n = n;
    f->log2n = log2n;
    f->tw_re = tw_re;
    f->tw_im = tw_im;
    f->win = win;
    for (int k = 0; k < n / 2; k++) {
        double a = 2.0 * M_PI * k / n;
        tw_re[k] = (float)cos(a);
        tw_im[k] = (float)-sin(a);
    }
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        win[i] = (float)(0.5 - 0.5 * cos(2.0 * M_PI * i / (n - 1)));   /* Hann */
        sum += win[i];
    }
    f->win_gain = (float)(sum / n);   /* ~0.5 para Hann */
}

static unsigned bit_reverse(unsigned x, int bits) {
    unsigned r = 0;
    for (int i = 0; i < bits; i++) { r = (r << 1) | (x & 1u); x >>= 1; }
    return r;
}

void fft_run(const fft_t *f, float *re, float *im) {
    const int n = f->n;

    /* 1) Reordenamiento bit-reverso */
    for (int i = 0; i < n; i++) {
        unsigned j = bit_reverse((unsigned)i, f->log2n);
        if (j > (unsigned)i) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    /* 2) log2(N) etapas de mariposas */
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = n / len;               /* salto en la tabla de twiddles */
        for (int start = 0; start < n; start += len) {
            for (int k = 0; k < half; k++) {
                float wr = f->tw_re[k * step];
                float wi = f->tw_im[k * step];
                int a = start + k, b = a + half;
                float tr = wr * re[b] - wi * im[b];
                float ti = wr * im[b] + wi * re[b];
                re[b] = re[a] - tr;  im[b] = im[a] - ti;
                re[a] += tr;         im[a] += ti;
            }
        }
    }
}

void fft_spectrum_db(const fft_t *f, const float *x, float *re, float *im, float *mag_db) {
    const int n = f->n;
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += x[i];
    mean /= (float)n;

    for (int i = 0; i < n; i++) {
        re[i] = (x[i] - mean) * f->win[i];
        im[i] = 0.0f;
    }
    fft_run(f, re, im);

    /* Amplitud de senoide: A = 2|X|/(N * ganancia_ventana) */
    const float scale = 2.0f / ((float)n * f->win_gain);
    for (int k = 0; k < n / 2; k++) {
        float m = sqrtf(re[k] * re[k] + im[k] * im[k]) * scale;
        mag_db[k] = 20.0f * log10f(m + 1e-6f);
    }
}
