/*
 * config.h — Parámetros globales del sistema PDS (Reto TE2004B)
 *
 * Hardware (PCBTractor):
 *   - Raspberry Pi Pico 2W (RP2350, Cortex-M33 @150 MHz con FPU simple precisión)
 *   - Micrófono MAX4466: J1.1 = 3V3, J1.2 = AGND, J1.3 = GPIO26 / ADC0
 *   - MCP2515 (CAN Pico): SPI0 -> MISO GP16, CS GP17, SCK GP18, MOSI GP19, INT GP20
 *     (reservado para REQ-AUD-10; no se usa en esta fase)
 *   - Zumo 2040 (RP2040) en el otro nodo CAN
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---------------- REQ-AUD-02: Adquisición y muestreo ---------------- */
#define MIC_GPIO            26          /* GPIO26 = ADC0 */
#define MIC_ADC_CH          0

#define ADC_CLK_HZ          48000000u   /* clk_adc = 48 MHz (USB PLL) */
#define FS_HZ               16000u      /* Frecuencia de muestreo */
/* El ADC toma una muestra cada (1 + DIV) ciclos de clk_adc:
 *   DIV = 48e6 / 16000 - 1 = 2999  ->  Fs = 16000.000 Hz exacto, Ts = 62.5 us */
#define ADC_CLKDIV          ((float)(ADC_CLK_HZ / FS_HZ) - 1.0f)

/* Tamaño de bloque: N = 320 muestras = 20 ms. Resolución Goertzel Fs/N = 50 Hz,
 * así los tonos de comando caen exactamente en un bin entero (k = f/50). */
#define BLOCK_N             320

/* ---------------- REQ-AUD-01: Tonos de comando ----------------------- */
/* Separación 500 Hz (>= 150 Hz), > 100 Hz, muy por debajo de Fs/2 = 8 kHz.
 * Segundo y tercer armónico (2/3/4.5 kHz) también quedan < Fs/2. */
#define NUM_TONES           2
#define TONE_A_HZ           1000.0f     /* Campo de cultivo A -> Zumo dirección 1 */
#define TONE_B_HZ           1500.0f     /* Campo de cultivo B -> Zumo dirección 2 */

/* ---------------- Decisión básica (se refina en REQ-AUD-08) ---------- */
#define DET_MIN_AMPL        8.0f        /* amplitud mínima (cuentas ADC) por defecto */
#define DET_DOMINANCE       4.0f        /* tono ganador >= 4x el otro (~12 dB) */
#define DET_MIN_BLOCKS      3           /* 3 bloques seguidos = 60 ms */

/* ---------------- REQ-AUD-09: FFT de diagnóstico --------------------- */
#define FFT_N               1024        /* 64 ms, resolución 15.625 Hz */
#define FFT_LOG2N           10

#endif
