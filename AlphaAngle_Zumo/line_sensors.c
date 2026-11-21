#include "pico/stdlib.h"
#include "line_sensors.h"

static const uint LS_PINS[5] = { 22, 21, 20, 19, 18 };

void line_sensors_init(void) {
    for (int i = 0; i < 5; i++) gpio_init(LS_PINS[i]);
    gpio_init(LS_EMITTER_PIN);
}

// RC method: charge the sensor capacitor, then time how long the phototransistor
// takes to pull the pin low. More reflected IR light = faster discharge.
void line_sensors_read(uint16_t out[5]) {
    gpio_set_dir(LS_EMITTER_PIN, GPIO_OUT);
    gpio_put(LS_EMITTER_PIN, 1);
    sleep_us(200);   // let the emitters reach full brightness

    for (int i = 0; i < 5; i++) {
        gpio_set_dir(LS_PINS[i], GPIO_OUT);
        gpio_put(LS_PINS[i], 1);
        out[i] = LS_TIMEOUT_US;
    }
    sleep_us(10);

    uint32_t start = time_us_32();
    for (int i = 0; i < 5; i++) gpio_set_dir(LS_PINS[i], GPIO_IN);
    int pending = 5;
    uint32_t t;
    while (pending > 0 && (t = time_us_32() - start) < LS_TIMEOUT_US) {
        for (int i = 0; i < 5; i++) {
            if (out[i] == LS_TIMEOUT_US && !gpio_get(LS_PINS[i])) {
                out[i] = (uint16_t)t;
                pending--;
            }
        }
    }

    // GP26 is also the battery level input, so release it after the reading.
    gpio_set_dir(LS_EMITTER_PIN, GPIO_IN);
}
