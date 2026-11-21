/*
 * adc_capture.h — Muestreo determinista del micrófono (REQ-AUD-02)
 *
 * El ADC del RP2350 se dispara por hardware con su divisor de reloj
 * (Fs = 48 MHz / (1 + DIV)), y dos canales DMA encadenados en ping-pong mueven
 * las muestras a dos buffers de BLOCK_N. La CPU nunca participa en el instante
 * de muestreo, así que Ts es constante (sin jitter por software ni por USB).
 */
#ifndef ADC_CAPTURE_H
#define ADC_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t blocks;         /* bloques completados */
    uint32_t overruns;       /* bloques que la CPU no alcanzó a procesar */
    uint32_t period_min_us;  /* periodo medido entre bloques */
    uint32_t period_max_us;
} adc_stats_t;

void adc_capture_init(void);
void adc_capture_start(void);

/* Si hay un bloque listo devuelve un puntero a él (BLOCK_N muestras de 12 bits),
 * si no, NULL. El bloque es válido hasta la siguiente llamada. */
const uint16_t *adc_capture_get_block(void);

void adc_capture_get_stats(adc_stats_t *s);
void adc_capture_reset_stats(void);

#endif
