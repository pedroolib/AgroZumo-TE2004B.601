#include "adc_capture.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

static uint16_t buf[2][BLOCK_N] __attribute__((aligned(4)));
static int dma_ch[2];

static volatile int      ready_idx = -1;    /* buffer listo para la CPU */
static volatile uint32_t blocks = 0, overruns = 0;
static volatile uint32_t last_us = 0, pmin = 0xFFFFFFFFu, pmax = 0;

static void dma_irq_handler(void) {
    uint32_t now = time_us_32();
    for (int i = 0; i < 2; i++) {
        if (dma_channel_get_irq0_status(dma_ch[i])) {
            dma_channel_acknowledge_irq0(dma_ch[i]);
            /* Rearmar la dirección destino; el contador se recarga solo
             * (TRANS_COUNT conserva el valor escrito como RELOAD). */
            dma_channel_set_write_addr(dma_ch[i], buf[i], false);

            if (ready_idx != -1) overruns++;   /* el anterior no se consumió */
            ready_idx = i;

            if (blocks > 0) {
                uint32_t p = now - last_us;
                if (p < pmin) pmin = p;
                if (p > pmax) pmax = p;
            }
            last_us = now;
            blocks++;
        }
    }
}

static void setup_channel(int i, int chain_to) {
    dma_channel_config c = dma_channel_get_default_config(dma_ch[i]);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);    /* siempre ADC FIFO */
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);           /* pacing por el ADC */
    channel_config_set_chain_to(&c, dma_ch[chain_to]);
    dma_channel_configure(dma_ch[i], &c, buf[i], &adc_hw->fifo, BLOCK_N, false);
    dma_channel_set_irq0_enabled(dma_ch[i], true);
}

void adc_capture_init(void) {
    adc_init();
    adc_gpio_init(MIC_GPIO);          /* pin en alta impedancia, sin buffer digital */
    adc_select_input(MIC_ADC_CH);
    adc_fifo_setup(true,   /* FIFO activo */
                   true,   /* genera DREQ */
                   1,      /* DREQ con 1 muestra */
                   false,  /* sin bit de error en la muestra */
                   false); /* 12 bits completos */
    adc_set_clkdiv(ADC_CLKDIV);       /* Fs = 48 MHz / (1 + 2999) = 16 kHz */

    dma_ch[0] = dma_claim_unused_channel(true);
    dma_ch[1] = dma_claim_unused_channel(true);
    setup_channel(0, 1);
    setup_channel(1, 0);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void adc_capture_start(void) {
    adc_fifo_drain();
    dma_channel_start(dma_ch[0]);
    adc_run(true);                    /* modo free-running a Fs */
}

const uint16_t *adc_capture_get_block(void) {
    uint32_t irq = save_and_disable_interrupts();
    int idx = ready_idx;
    ready_idx = -1;
    restore_interrupts(irq);
    return idx < 0 ? NULL : buf[idx];
}

void adc_capture_get_stats(adc_stats_t *s) {
    uint32_t irq = save_and_disable_interrupts();
    s->blocks = blocks;
    s->overruns = overruns;
    s->period_min_us = pmin;
    s->period_max_us = pmax;
    restore_interrupts(irq);
}

void adc_capture_reset_stats(void) {
    uint32_t irq = save_and_disable_interrupts();
    overruns = 0;
    pmin = 0xFFFFFFFFu;
    pmax = 0;
    restore_interrupts(irq);
}
