#include "tone_detect.h"
#include <math.h>

const float TONE_FREQS[NUM_TONES] = { TONE_A_HZ, TONE_B_HZ };
const char *const TONE_NAMES[NUM_TONES] = { "CAMPO_A", "CAMPO_B" };

void tone_detector_init(tone_detector_t *d, float fs_hz) {
    for (int i = 0; i < NUM_TONES; i++) {
        goertzel_init(&d->bank[i], TONE_FREQS[i], fs_hz);
        d->ampl[i] = 0.0f;
    }
    d->rms = 0.0f;
    d->min_ampl = DET_MIN_AMPL;
    d->candidate = CMD_NONE;
    d->run = 0;
    d->current = CMD_NONE;
}

command_t tone_detector_process(tone_detector_t *d, const float *x, int n) {
    /* 1) Goertzel: un MAC por muestra por tono */
    for (int k = 0; k < NUM_TONES; k++) goertzel_reset(&d->bank[k]);
    float energy = 0.0f;
    for (int i = 0; i < n; i++) {
        float s = x[i];
        energy += s * s;
        for (int k = 0; k < NUM_TONES; k++) goertzel_step(&d->bank[k], s);
    }
    d->rms = sqrtf(energy / (float)n);

    /* 2) Tono con mayor amplitud */
    int best = 0;
    for (int k = 0; k < NUM_TONES; k++) {
        d->ampl[k] = goertzel_amplitude(&d->bank[k], n);
        if (d->ampl[k] > d->ampl[best]) best = k;
    }
    float second = 0.0f;
    for (int k = 0; k < NUM_TONES; k++)
        if (k != best && d->ampl[k] > second) second = d->ampl[k];

    /* 3) Decisión básica: umbral + dominancia sobre el otro tono */
    int winner = CMD_NONE;
    if (d->ampl[best] >= d->min_ampl && d->ampl[best] >= DET_DOMINANCE * second)
        winner = best;

    /* 4) Duración mínima: DET_MIN_BLOCKS bloques consecutivos iguales */
    if (winner == d->candidate) {
        if (d->run < 1000) d->run++;
    } else {
        d->candidate = winner;
        d->run = 1;
    }
    if (d->run >= DET_MIN_BLOCKS) d->current = (command_t)d->candidate;

    return d->current;
}
