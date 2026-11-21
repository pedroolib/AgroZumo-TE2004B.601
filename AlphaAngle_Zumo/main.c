#include <stdio.h>
#include "pico/stdlib.h"
#include "line_sensors.h"
#include "alpha.h"

// Gray level -> distance from the white corner.
// PLACEHOLDER values. To calibrate: put S3 (DN3) on the terrain at known distances
// from the white corner, write down the raw S3 value that this program prints,
// and fill the table sorted by reading.
static const gray_cal_t GRAY_CAL[] = {
    {  200, 0.050f },
    {  600, 0.100f },
    { 1000, 0.150f },
    { 1400, 0.200f },
};
#define N_CAL ((int)(sizeof GRAY_CAL / sizeof GRAY_CAL[0]))

int main(void) {
    stdio_init_all();
    sleep_ms(3000);   // give the USB serial connection time to enumerate
    printf("\n--- Zumo: alpha angle from the five line sensors ---\n");
    line_sensors_init();

    while (true) {
        uint16_t raw[ALPHA_N_SENSORS];
        float r[ALPHA_N_SENSORS];
        line_sensors_read(raw);
        for (int i = 0; i < ALPHA_N_SENSORS; i++) r[i] = gray_to_radius(raw[i], GRAY_CAL, N_CAL);
        float alpha = alpha_from_radii(r, SENSOR_SPACING_M);

        printf("raw");
        for (int i = 0; i < ALPHA_N_SENSORS; i++) printf(" %4u", raw[i]);
        printf(" | r (m)");
        for (int i = 0; i < ALPHA_N_SENSORS; i++) printf(" %.4f", r[i]);
        printf(" | alpha = %.1f deg\n", alpha);
        sleep_ms(200);
    }
}
