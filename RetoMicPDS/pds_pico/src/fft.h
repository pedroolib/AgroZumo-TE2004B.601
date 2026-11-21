/*
 * fft.h — FFT radix-2 iterativa (decimación en tiempo), implementada desde cero
 * para el diagnóstico espectral (REQ-AUD-09). Sin librerías externas.
 */
#ifndef FFT_H
#define FFT_H

typedef struct {
    int    n;        /* tamaño (potencia de 2) */
    int    log2n;
    float *tw_re;    /* twiddles cos(2*pi*k/N), k = 0..N/2-1 */
    float *tw_im;    /* twiddles -sin(2*pi*k/N) */
    float *win;      /* ventana de Hann de N puntos */
    float  win_gain; /* suma(w)/N, para corregir la amplitud */
} fft_t;

/* Las tablas las aporta el llamador (evita malloc en el micro). */
void fft_init(fft_t *f, int n, int log2n, float *tw_re, float *tw_im, float *win);

/* FFT in-place sobre re[] / im[] de longitud n. */
void fft_run(const fft_t *f, float *re, float *im);

/* Pipeline de diagnóstico: quita la media, aplica Hann, FFT y devuelve
 * mag_db[k] = 20 log10(A_k) para k = 0..n/2-1, donde A_k es la amplitud
 * estimada de una senoide en ese bin (mismas unidades que x).
 * re[] / im[] son buffers de trabajo de n floats. */
void fft_spectrum_db(const fft_t *f, const float *x, float *re, float *im, float *mag_db);

#endif
