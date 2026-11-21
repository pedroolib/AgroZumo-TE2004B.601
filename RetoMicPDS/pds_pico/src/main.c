/*
 * main.c — Reto TE2004B, módulo PDS. Fase 1: adquisición limpia + Goertzel + FFT.
 *
 * Flujo por bloque (cada 20 ms):
 *   DMA (ADC @16 kHz) -> bloque uint16 -> quitar DC -> banco Goertzel -> decisión
 *                                      \-> acumulador FFT 1024 (diagnóstico)
 *
 * Protocolo por USB-serial (115200, el valor no importa en CDC):
 *   Comandos (1 carácter): h ayuda | g Goertzel on/off | f FFT on/off |
 *                          r volcado crudo | t tiempos | c calibrar silencio
 *   Salida (líneas CSV con prefijo):
 *     DET,<comando>                         cambio de comando detectado
 *     G,<bloque>,<A_1000>,<A_1500>,<rms>,<cmd>   amplitudes por bloque
 *     F,<fs>,<n>,<us>,<dB*10 x n/2>          espectro FFT (amplitud en cuentas)
 *     R,<fs>,<n>,<muestras x n>              muestras crudas de 12 bits
 *     S,...                                 estadísticas de tiempo real
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "config.h"
#include "adc_capture.h"
#include "tone_detect.h"
#include "fft.h"

/* ---- Estado global ---- */
static tone_detector_t det;
static float xblk[BLOCK_N];

static fft_t fft;
static float fft_tw_re[FFT_N / 2], fft_tw_im[FFT_N / 2], fft_win[FFT_N];
static float fft_in[FFT_N], fft_re[FFT_N], fft_im[FFT_N], fft_db[FFT_N / 2];
static uint16_t raw_in[FFT_N];
static int fft_fill = 0;

static bool stream_goertzel = false;
static bool stream_fft = false;
static bool want_raw = false;
static int  calib_blocks = 0;
static float calib_max = 0.0f;

static uint32_t proc_us_max = 0, proc_us_sum = 0, proc_count = 0;
static uint32_t fft_us_last = 0;
static uint32_t last_fft_ms = 0;
#define FFT_PERIOD_MS 200

static void print_help(void) {
    printf("\n=== PDS Pico 2W | Fs=%u Hz | N=%d (%.0f ms) | tonos: %.0f Hz (A), %.0f Hz (B) ===\n",
           FS_HZ, BLOCK_N, 1000.0f * BLOCK_N / FS_HZ, TONE_A_HZ, TONE_B_HZ);
    printf("h ayuda | g Goertzel on/off | f FFT on/off | r volcado crudo | t tiempos | c calibrar silencio\n");
}

static void print_stats(void) {
    adc_stats_t s;
    adc_capture_get_stats(&s);
    float avg = proc_count ? (float)proc_us_sum / proc_count : 0.0f;
    printf("S,bloques=%lu,overruns=%lu,periodo_us=[%lu..%lu] (ideal %u),"
           "goertzel_us_prom=%.1f,max=%lu,us_por_muestra=%.3f,presupuesto_us=%u,"
           "carga_cpu=%.2f%%,fft_us=%lu,umbral=%.1f\n",
           (unsigned long)s.blocks, (unsigned long)s.overruns,
           (unsigned long)s.period_min_us, (unsigned long)s.period_max_us,
           (unsigned)(1000000u * BLOCK_N / FS_HZ),
           avg, (unsigned long)proc_us_max, avg / BLOCK_N,
           (unsigned)(1000000u * BLOCK_N / FS_HZ),
           100.0f * avg / (1000000.0f * BLOCK_N / FS_HZ),
           (unsigned long)fft_us_last, det.min_ampl);
    adc_capture_reset_stats();
    proc_us_max = proc_us_sum = proc_count = 0;
}

static void handle_input(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;
    switch (c) {
        case 'h': print_help(); break;
        case 'g': stream_goertzel = !stream_goertzel; break;
        case 'f': stream_fft = !stream_fft; fft_fill = 0; break;
        case 'r': want_raw = true; fft_fill = 0; break;
        case 't': print_stats(); break;
        case 'c':
            calib_blocks = 50; calib_max = 0.0f;   /* 1 s de silencio */
            printf("CAL,inicio (mantener silencio 1 s)\n");
            break;
        default: break;
    }
}

static void emit_fft(void) {
    uint32_t t0 = time_us_32();
    fft_spectrum_db(&fft, fft_in, fft_re, fft_im, fft_db);
    fft_us_last = time_us_32() - t0;

    printf("F,%u,%d,%lu", FS_HZ, FFT_N, (unsigned long)fft_us_last);
    for (int k = 0; k < FFT_N / 2; k++) printf(",%d", (int)(fft_db[k] * 10.0f));
    printf("\n");
}

static void emit_raw(void) {
    printf("R,%u,%d", FS_HZ, FFT_N);
    for (int i = 0; i < FFT_N; i++) printf(",%u", raw_in[i]);
    printf("\n");
}

static void process_block(const uint16_t *b) {
    /* Quitar DC (el MAX4466 está polarizado a VCC/2 ~ 2048 cuentas) */
    float mean = 0.0f;
    for (int i = 0; i < BLOCK_N; i++) mean += b[i];
    mean /= BLOCK_N;
    for (int i = 0; i < BLOCK_N; i++) xblk[i] = (float)b[i] - mean;

    /* Goertzel + decisión, cronometrado */
    uint32_t t0 = time_us_32();
    command_t prev = det.current;
    command_t cmd = tone_detector_process(&det, xblk, BLOCK_N);
    uint32_t dt = time_us_32() - t0;
    if (dt > proc_us_max) proc_us_max = dt;
    proc_us_sum += dt;
    proc_count++;

    if (cmd != prev)
        printf("DET,%s\n", cmd == CMD_NONE ? "NINGUNO" : TONE_NAMES[cmd]);

    if (stream_goertzel)
        printf("G,%lu,%.2f,%.2f,%.2f,%d\n", (unsigned long)proc_count,
               det.ampl[0], det.ampl[1], det.rms, (int)cmd);

    /* Calibración del umbral con silencio: umbral = 3x el máximo observado */
    if (calib_blocks > 0) {
        for (int k = 0; k < NUM_TONES; k++)
            if (det.ampl[k] > calib_max) calib_max = det.ampl[k];
        if (--calib_blocks == 0) {
            det.min_ampl = 3.0f * calib_max;
            if (det.min_ampl < 2.0f) det.min_ampl = 2.0f;
            printf("CAL,ruido_max=%.2f,umbral=%.2f\n", calib_max, det.min_ampl);
        }
    }

    /* Acumulador para FFT / volcado crudo */
    bool fft_due = stream_fft && (to_ms_since_boot(get_absolute_time()) - last_fft_ms >= FFT_PERIOD_MS);
    if (fft_due || want_raw) {
        for (int i = 0; i < BLOCK_N && fft_fill < FFT_N; i++, fft_fill++) {
            raw_in[fft_fill] = b[i];
            fft_in[fft_fill] = (float)b[i];
        }
        if (fft_fill == FFT_N) {
            if (want_raw) { emit_raw(); want_raw = false; }
            if (fft_due)  { emit_fft(); last_fft_ms = to_ms_since_boot(get_absolute_time()); }
            fft_fill = 0;
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);                    /* tiempo para que el host abra el puerto */

    tone_detector_init(&det, (float)FS_HZ);
    fft_init(&fft, FFT_N, FFT_LOG2N, fft_tw_re, fft_tw_im, fft_win);

    adc_capture_init();
    adc_capture_start();
    print_help();

    while (true) {
        handle_input();
        const uint16_t *b = adc_capture_get_block();
        if (b) process_block(b);
        else   tight_loop_contents();
    }
}
