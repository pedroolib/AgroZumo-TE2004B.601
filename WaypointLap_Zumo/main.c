#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "mcp2515.h"
#include "motors.h"
#include "follow.h"
#include "waypoints.h"

// CAN-ID database, message 1: local positioning data (LPdata) from the Pico 2 W.
//   ID  = 0x100, DLC = 6
//   D0-D1 = x in mm, D2-D3 = y in mm, D4-D5 = angle in degrees (int16, big-endian)
#define CAN_ID_LPDATA   0x100
#define CAN_DLC_LPDATA  6

#define BUTTON_A_PIN    25     // shared with the yellow LED, reads low when pressed
#define LP_TIMEOUT_MS   500    // stop if no LPdata arrives for this long (marker lost or Pico off)

static const float DEG = 3.14159265f / 180.0f;

typedef struct { float x, y, heading; int16_t x_mm, y_mm, angle; } pose_t;

static int16_t get_int16(const uint8_t *src) {
    return (int16_t)(((uint16_t)src[0] << 8) | src[1]);
}

// Reads one LPdata frame. Returns false if no new frame is waiting.
static bool read_pose(pose_t *p) {
    if (!mcp2515_message_available()) return false;
    can_frame_t rx;
    if (!mcp2515_read_message(&rx) || rx.id != CAN_ID_LPDATA || rx.dlc != CAN_DLC_LPDATA) return false;
    p->x_mm = get_int16(&rx.data[0]);
    p->y_mm = get_int16(&rx.data[2]);
    p->angle = get_int16(&rx.data[4]);
    p->heading = (ANGLE_SIGN * p->angle + ANGLE_OFFSET_DEG) * DEG;
    // The waypoints hold turning-center positions, so move the marker position to the center.
    p->x = p->x_mm / 1000.0f + MARKER_OFFSET_M * cosf(p->heading);
    p->y = p->y_mm / 1000.0f + MARKER_OFFSET_M * sinf(p->heading);
    return true;
}

static bool button_a_pressed(void) {
    return !gpio_get(BUTTON_A_PIN);
}

static float seconds_since(absolute_time_t t0) {
    return absolute_time_diff_us(t0, get_absolute_time()) / 1e6f;
}

// Follows the route once. Returns the lap time in s.
static float run_lap(void) {
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t last_rx = t0;
    bool waiting = false;
    while (true) {
        pose_t p;
        if (read_pose(&p)) {
            last_rx = get_absolute_time();
            waiting = false;
            follow_out_t o;
            follow_step(p.x, p.y, p.heading, &o);
            float t = seconds_since(t0);
            if (o.reached >= 0)
                printf("Reached waypoint P%d at %.2f s\n", ROUTE[o.reached] + 1, t);
            if (o.lost) {
                motors_set_speeds(0, 0);
                printf("Lost the route: %.0f mm from the path at %.2f s. Lap stopped.\n",
                       o.cross_track_m * 1000.0f, t);
                return -1.0f;
            }
            if (o.done) {
                motors_set_speeds(0, 0);
                printf("Final waypoint P%d reached. LAP TIME %.2f s\n", ROUTE[ROUTE_LEN - 1] + 1, t);
                return t;
            }
            motors_set_speeds(o.left, o.right);
            printf("t=%.2f x=%d y=%d a=%d | P%d->P%d err=%.0f mm | L=%ld R=%ld\n", t,
                   p.x_mm, p.y_mm, p.angle, ROUTE[o.section] + 1, ROUTE[o.section + 1] + 1,
                   o.cross_track_m * 1000.0f, (long)o.left, (long)o.right);
        }

        // Safety stop: without fresh LPdata the Zumo cannot know where it is.
        if (!waiting && absolute_time_diff_us(last_rx, get_absolute_time()) > LP_TIMEOUT_MS * 1000) {
            motors_set_speeds(0, 0);
            printf("No LPdata for %d ms. Motors stopped, waiting for data...\n", LP_TIMEOUT_MS);
            waiting = true;
        }
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(3000);   // give the USB serial connection time to enumerate

    printf("\n--- Zumo: waypoint lap with local positioning ---\n");
    motors_init();
    motors_set_speeds(0, 0);
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);

    mcp2515_init();
    while (!mcp2515_set_bitrate_500kbps_8mhz() || !mcp2515_set_mode(MCP2515_MODE_NORMAL)) {
        printf("MCP2515 not responding. Check: Pico powered (J5.8 in, switch ON), "
               "SPI wiring on GP0-GP3 and GP28. Retrying...\n");
        sleep_ms(1000);
        mcp2515_reset();
    }

    float last_lap = 0.0f;
    while (true) {
        int n = follow_init(WAYPOINTS, ROUTE, ROUTE_LEN);
        if (n < 0) {
            printf("Waypoint table is not valid (first section must be straight, max %d points)\n",
                   PATH_MAX_PTS);
            sleep_ms(2000);
            continue;
        }

        // Wait for button A. Print the pose so you can place the Zumo on the start waypoint.
        const waypoint_t *start = &WAYPOINTS[ROUTE[0]];
        absolute_time_t next_print = get_absolute_time();
        while (!button_a_pressed()) {
            pose_t p;
            if (read_pose(&p) && absolute_time_diff_us(next_print, get_absolute_time()) > 0) {
                next_print = make_timeout_time_ms(500);
                printf("Ready (%d path points). Pose x=%d y=%d a=%d, %.0f mm from start P%d. "
                       "Last lap %.2f s. Press A to start.\n", n, p.x_mm, p.y_mm, p.angle,
                       hypotf(p.x - start->x, p.y - start->y) * 1000.0f, ROUTE[0] + 1, last_lap);
            }
        }
        while (button_a_pressed()) tight_loop_contents();
        sleep_ms(300);   // let the hand leave the robot
        while (read_pose(&(pose_t){0})) {}   // drop a frame that is older than the start
        last_lap = run_lap();
    }
}
