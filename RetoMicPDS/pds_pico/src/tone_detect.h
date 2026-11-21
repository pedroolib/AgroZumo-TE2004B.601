/*
 * tone_detect.h — Banco de Goertzel + mapeo tono -> comando (REQ-AUD-01, 07)
 *
 * Por ahora la decisión es básica (umbral + dominancia + duración mínima).
 * La validación completa con SNR estimada y rechazo de ambigüedad es REQ-AUD-08.
 */
#ifndef TONE_DETECT_H
#define TONE_DETECT_H

#include "goertzel.h"
#include "config.h"

typedef enum {
    CMD_NONE    = -1,
    CMD_CAMPO_A = 0,   /* TONE_A_HZ */
    CMD_CAMPO_B = 1,   /* TONE_B_HZ */
} command_t;

typedef struct {
    goertzel_t bank[NUM_TONES];
    float ampl[NUM_TONES];   /* amplitud del último bloque (cuentas ADC) */
    float rms;               /* RMS del bloque sin DC (referencia de energía total) */
    float min_ampl;          /* umbral de amplitud */
    int   candidate;         /* tono ganador en bloques consecutivos */
    int   run;               /* cuántos bloques seguidos lleva */
    command_t current;       /* comando confirmado */
} tone_detector_t;

extern const float TONE_FREQS[NUM_TONES];
extern const char *const TONE_NAMES[NUM_TONES];

void tone_detector_init(tone_detector_t *d, float fs_hz);

/* Procesa un bloque de n muestras (ya sin DC). Devuelve el comando confirmado
 * actual (CMD_NONE si no hay tono válido). */
command_t tone_detector_process(tone_detector_t *d, const float *x, int n);

#endif
