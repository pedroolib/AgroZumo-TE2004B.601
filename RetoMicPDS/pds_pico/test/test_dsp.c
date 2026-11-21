/*
 * test_dsp.c — Verificación en PC (sin hardware) de Goertzel, FFT y detector.
 * Compilar:  gcc -O2 -Isrc test/test_dsp.c src/goertzel.c src/fft.c src/tone_detect.c -lm -o test_dsp
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"
#include "goertzel.h"
#include "fft.h"
#include "tone_detect.h"

#define PI 3.14159265358979323846
static int fails = 0;
#define CHECK(cond, ...) do { if (cond) printf("  OK   "); else { printf("  FALLA "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static void tone(float *x, int n, float f, float a, int off) {
    for (int i = 0; i < n; i++) x[i] = a * (float)sin(2 * PI * f * (i + off) / FS_HZ);
}

int main(void) {
    float x[FFT_N];

    printf("[1] Goertzel: amplitud en la frecuencia del tono\n");
    goertzel_t g;
    tone(x, BLOCK_N, TONE_A_HZ, 100.0f, 0);
    goertzel_init(&g, TONE_A_HZ, FS_HZ);
    goertzel_block(&g, x, BLOCK_N);
    float a = goertzel_amplitude(&g, BLOCK_N);
    CHECK(fabsf(a - 100.0f) < 1.0f, "A(1000 Hz) = %.2f (esperado 100)", a);
    goertzel_init(&g, TONE_B_HZ, FS_HZ);
    goertzel_block(&g, x, BLOCK_N);
    a = goertzel_amplitude(&g, BLOCK_N);
    CHECK(a < 1.0f, "A(1500 Hz) con tono de 1000 Hz = %.3f (esperado ~0)", a);

    printf("[2] Goertzel vs DFT directa en el bin k = f*N/Fs\n");
    int k = (int)(TONE_B_HZ * BLOCK_N / FS_HZ);
    for (int i = 0; i < BLOCK_N; i++) x[i] = (float)(rand() % 2001 - 1000);
    double re = 0, im = 0;
    for (int i = 0; i < BLOCK_N; i++) { re += x[i] * cos(2 * PI * k * i / BLOCK_N); im -= x[i] * sin(2 * PI * k * i / BLOCK_N); }
    goertzel_init(&g, TONE_B_HZ, FS_HZ);
    float pg = goertzel_block(&g, x, BLOCK_N);
    double pd = re * re + im * im;
    CHECK(fabs(pg - pd) / pd < 1e-3, "|X(%d)|^2 Goertzel=%.4g DFT=%.4g", k, pg, pd);

    printf("[3] FFT: pico en el bin correcto y amplitud\n");
    static float tr[FFT_N / 2], ti[FFT_N / 2], w[FFT_N], re2[FFT_N], im2[FFT_N], db[FFT_N / 2];
    fft_t f;
    fft_init(&f, FFT_N, FFT_LOG2N, tr, ti, w);
    for (int i = 0; i < FFT_N; i++)
        x[i] = 2048.0f + 200.0f * (float)sin(2 * PI * 1500.0 * i / FS_HZ);
    fft_spectrum_db(&f, x, re2, im2, db);
    int kmax = 1;
    for (int j = 1; j < FFT_N / 2; j++) if (db[j] > db[kmax]) kmax = j;
    float fpk = (float)kmax * FS_HZ / FFT_N;
    CHECK(fabsf(fpk - 1500.0f) <= (float)FS_HZ / FFT_N, "pico en %.1f Hz (esperado 1500)", fpk);
    CHECK(fabsf(db[kmax] - 20.0f * log10f(200.0f)) < 1.5f, "nivel pico %.1f dB (esperado %.1f, Hann scalloping < 1.5 dB)", db[kmax], 20 * log10f(200.0f));
    CHECK(db[0] < 0.0f, "DC removido: %.1f dB", db[0]);

    printf("[4] Detector: mapeo tono -> comando con duración mínima\n");
    tone_detector_t d;
    tone_detector_init(&d, FS_HZ);
    command_t c = CMD_NONE;
    int off = 0;
    for (int b = 0; b < DET_MIN_BLOCKS - 1; b++, off += BLOCK_N) { tone(x, BLOCK_N, TONE_B_HZ, 50, off); c = tone_detector_process(&d, x, BLOCK_N); }
    CHECK(c == CMD_NONE, "antes de %d bloques: sin comando", DET_MIN_BLOCKS);
    tone(x, BLOCK_N, TONE_B_HZ, 50, off); off += BLOCK_N;
    c = tone_detector_process(&d, x, BLOCK_N);
    CHECK(c == CMD_CAMPO_B, "1500 Hz -> CAMPO_B (A=%.1f, B=%.1f)", d.ampl[0], d.ampl[1]);
    for (int b = 0; b < DET_MIN_BLOCKS; b++, off += BLOCK_N) { tone(x, BLOCK_N, TONE_A_HZ, 50, off); c = tone_detector_process(&d, x, BLOCK_N); }
    CHECK(c == CMD_CAMPO_A, "1000 Hz -> CAMPO_A");
    for (int b = 0; b < DET_MIN_BLOCKS; b++, off += BLOCK_N) {
        for (int i = 0; i < BLOCK_N; i++)
            x[i] = 50 * (float)(sin(2 * PI * TONE_A_HZ * (i + off) / FS_HZ) + sin(2 * PI * TONE_B_HZ * (i + off) / FS_HZ));
        c = tone_detector_process(&d, x, BLOCK_N);
    }
    CHECK(c == CMD_NONE, "ambos tonos iguales -> rechazado (ambiguo)");
    for (int b = 0; b < DET_MIN_BLOCKS; b++) { for (int i = 0; i < BLOCK_N; i++) x[i] = 0; c = tone_detector_process(&d, x, BLOCK_N); }
    CHECK(c == CMD_NONE, "silencio -> NINGUNO");
    for (int b = 0; b < DET_MIN_BLOCKS; b++, off += BLOCK_N) { tone(x, BLOCK_N, 1250.0f, 50, off); c = tone_detector_process(&d, x, BLOCK_N); }
    CHECK(c == CMD_NONE, "1250 Hz (fuera de tonos) -> NINGUNO (A=%.2f, B=%.2f)", d.ampl[0], d.ampl[1]);

    printf("\n%s (%d fallas)\n", fails ? "HAY FALLAS" : "TODAS LAS PRUEBAS PASARON", fails);
    return fails != 0;
}
