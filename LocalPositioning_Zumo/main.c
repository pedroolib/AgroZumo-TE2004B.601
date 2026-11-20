#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "mcp2515.h"
#include "motors.h"

// CAN-ID database, message 1: local positioning data (LPdata) from the Pico 2 W.
//   ID  = 0x100, DLC = 6
//   D0-D1 = x in mm (int16, big-endian)
//   D2-D3 = y in mm (int16, big-endian)
//   D4-D5 = angle in degrees (int16, big-endian)
#define CAN_ID_LPDATA   0x100
#define CAN_DLC_LPDATA  6

#define TARGET_MM       500    // destination waypoint: 50 cm from the start
#define SLOW_ZONE_MM    100    // drive slower in the last 10 cm to stop closer to the target
#define SPEED_FAST      1500   // motor speed, range -6000..6000
#define SPEED_SLOW      700
#define LP_TIMEOUT_MS   500    // stop if no LPdata arrives for this long (marker lost or Pico off)

static int16_t get_int16(const uint8_t *src) {
    return (int16_t)(((uint16_t)src[0] << 8) | src[1]);
}

int main(void) {
    stdio_init_all();
    sleep_ms(3000); // give the USB serial connection time to enumerate

    printf("\n--- Zumo: move 50 cm in y with local positioning ---\n");
    motors_init();
    motors_set_speeds(0, 0);

    mcp2515_init();
    // After a reset the MCP2515 is in config mode, so CANSTAT reads 0x80.
    // 0x00 or 0xFF means the Zumo is not talking to the chip over SPI.
    uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
    printf("MCP2515 CANSTAT after reset = 0x%02X (expected 0x80)\n", canstat);
    while (!mcp2515_set_bitrate_500kbps_8mhz() || !mcp2515_set_mode(MCP2515_MODE_NORMAL)) {
        printf("MCP2515 not responding. Check: Pico powered (J5.8 in, switch ON), "
               "SPI wiring on GP0-GP3 and GP28. Retrying...\n");
        sleep_ms(1000);
        mcp2515_reset();
    }
    printf("MCP2515 ready (500 kbps). Waiting for LPdata on ID 0x%03X...\n", CAN_ID_LPDATA);

    bool started = false;      // true after the first LPdata sets the starting waypoint
    bool arrived = false;      // true after the Zumo reaches the destination waypoint
    int16_t y_start_mm = 0;
    absolute_time_t last_rx = get_absolute_time();

    while (true) {
        if (mcp2515_message_available()) {
            can_frame_t rx;
            if (mcp2515_read_message(&rx) && rx.id == CAN_ID_LPDATA && rx.dlc == CAN_DLC_LPDATA) {
                int16_t x_mm  = get_int16(&rx.data[0]);
                int16_t y_mm  = get_int16(&rx.data[2]);
                int16_t angle = get_int16(&rx.data[4]);
                last_rx = get_absolute_time();

                printf("RX  ID=0x%03X  DLC=%d  Data=", (unsigned)rx.id, rx.dlc);
                for (int i = 0; i < rx.dlc; i++) printf("%02X ", rx.data[i]);
                printf(" -> x=%d mm  y=%d mm  angle=%d deg\n", x_mm, y_mm, angle);

                if (!started) {
                    started = true;
                    y_start_mm = y_mm;
                    printf("Start waypoint: x=%d mm  y=%d mm  angle=%d deg. Destination: %d mm away in y\n",
                           x_mm, y_mm, angle, TARGET_MM);
                }

                if (!arrived) {
                    // abs(): y can grow or shrink depending on how the Zumo faces the track
                    int traveled = abs(y_mm - y_start_mm);
                    int remaining = TARGET_MM - traveled;
                    printf("Check waypoint: traveled=%d mm  remaining=%d mm\n", traveled, remaining);

                    if (remaining <= 0) {
                        motors_set_speeds(0, 0);
                        arrived = true;
                        printf("Destination waypoint reached: y=%d mm (traveled %d mm). Motors stopped.\n",
                               y_mm, traveled);
                    } else {
                        int speed = (remaining < SLOW_ZONE_MM) ? SPEED_SLOW : SPEED_FAST;
                        motors_set_speeds(speed, speed);
                        printf("Moving forward: speed=%d\n", speed);
                    }
                }
            }
        }

        // Safety stop: without fresh LPdata the Zumo cannot know where it is.
        if (started && !arrived &&
            absolute_time_diff_us(last_rx, get_absolute_time()) > LP_TIMEOUT_MS * 1000) {
            motors_set_speeds(0, 0);
            printf("No LPdata for %d ms. Motors stopped, waiting for data...\n", LP_TIMEOUT_MS);
            last_rx = get_absolute_time();   // print once per timeout, not on every loop
        }
    }
}
