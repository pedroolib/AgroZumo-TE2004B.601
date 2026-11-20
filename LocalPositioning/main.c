#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "mcp2515.h"

static const char *WIFI_SSID = "embedded";
static const char *WIFI_PASSWORD = "12345678";
static const uint16_t TELEMETRY_PORT = 5000;

// Our team's ArUco marker. The camera PC sends every team's markers.
#define MARKER_ID 3

// CAN-ID database, message 1: local positioning data (LPdata).
//   ID  = 0x100, DLC = 6
//   D0-D1 = x in mm (int16, big-endian)
//   D2-D3 = y in mm (int16, big-endian)
//   D4-D5 = angle in degrees (int16, big-endian)
#define CAN_ID_LPDATA   0x100
#define CAN_DLC_LPDATA  6
#define CAN_PERIOD_MS   100

static float lp_x, lp_y, lp_angle;
static bool lp_new = false;   // true when a fresh position arrived since the last CAN send

// Reads the number after "key": inside str. Returns false if the key is missing.
static bool read_number(const char *str, const char *key, float *out) {
    const char *p = strstr(str, key);
    if (p == NULL) return false;
    *out = strtof(p + strlen(key), NULL);
    return true;
}

// Finds our marker in one JSON line such as:
// {..."markers":[{"id":1,"x":-0.333,"y":-0.74,"angle":-78.0}],...}
static bool parse_marker(const char *json, float *x, float *y, float *angle) {
    char needle[16];
    snprintf(needle, sizeof needle, "{\"id\":%d,", MARKER_ID);   // the brace and comma stop "id":1 matching "id":12
    const char *m = strstr(json, needle);
    if (m == NULL) return false;   // our marker is not in view
    return read_number(m, "\"x\":", x) &&
           read_number(m, "\"y\":", y) &&
           read_number(m, "\"angle\":", angle);
}

static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg;
    (void)pcb;
    (void)addr;
    (void)port;
    if (p == NULL) return;

    // The payload is not null-terminated, so copy it into a string first.
    char json[512];
    u16_t n = pbuf_copy_partial(p, json, sizeof json - 1, 0);
    json[n] = '\0';
    pbuf_free(p);

    float x, y, angle;
    if (parse_marker(json, &x, &y, &angle)) {
        lp_x = x;
        lp_y = y;
        lp_angle = angle;
        lp_new = true;
    }
}

static void put_int16(uint8_t *dst, int16_t v) {
    dst[0] = (uint8_t)((uint16_t)v >> 8);
    dst[1] = (uint8_t)((uint16_t)v & 0xFF);
}

static void send_lpdata(void) {
    int16_t x_mm = (int16_t)(lp_x * 1000.0f);
    int16_t y_mm = (int16_t)(lp_y * 1000.0f);
    int16_t ang = (int16_t)lp_angle;

    can_frame_t frame = { .id = CAN_ID_LPDATA, .dlc = CAN_DLC_LPDATA };
    put_int16(&frame.data[0], x_mm);
    put_int16(&frame.data[2], y_mm);
    put_int16(&frame.data[4], ang);

    mcp2515_send_message(&frame);
    // The class driver returns true even on timeout, so read TXB0CTRL instead:
    // TXREQ (bit 3) still set means no node ACKed the frame.
    uint8_t txctrl = mcp2515_read_register(MCP2515_TXB0CTRL);
    bool ok = (txctrl & 0x08) == 0;
    printf("LP x=%.3f y=%.3f angle=%.1f | TX ID=0x%03X DLC=%d Data=",
           lp_x, lp_y, lp_angle, (unsigned)frame.id, frame.dlc);
    for (int i = 0; i < frame.dlc; i++) printf("%02X ", frame.data[i]);
    if (ok) {
        printf("OK\n");
    } else {
        printf("NO ACK | TXB0CTRL=0x%02X TEC=%u REC=%u EFLG=0x%02X\n", txctrl,
               mcp2515_read_register(0x1C), mcp2515_read_register(0x1D),
               mcp2515_read_register(0x2D));
        mcp2515_modify_register(MCP2515_TXB0CTRL, 0x08, 0x00);   // drop the stale frame so the next one carries fresh data
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);   // time to open the serial monitor

    mcp2515_init();
    mcp2515_set_bitrate_500kbps_8mhz();
    mcp2515_set_mode(MCP2515_MODE_NORMAL);
    printf("MCP2515 ready (500 kbps)\n");

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                              CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Wi-Fi connect failed, retrying...\n");
    }
    printf("Connected\n");

    // Without this, the radio delays broadcast packets by several seconds.
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
    udp_recv(pcb, on_udp_packet, NULL);

    absolute_time_t next_send = make_timeout_time_ms(CAN_PERIOD_MS);
    while (true) {
        cyw43_arch_poll();

        // Send from the main loop, not the UDP callback: a CAN send can block
        // for up to 1 s when no node ACKs, and that would stall Wi-Fi.
        if (absolute_time_diff_us(get_absolute_time(), next_send) <= 0) {
            next_send = make_timeout_time_ms(CAN_PERIOD_MS);
            if (lp_new) {   // send only fresh data, so the Zumo can tell when the marker is lost
                lp_new = false;
                send_lpdata();
            }
        }

        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
    }
}
