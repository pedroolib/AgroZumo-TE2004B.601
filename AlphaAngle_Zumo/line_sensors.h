#pragma once

#include <stdint.h>

// Zumo 2040 front sensor array (Pololu Zumo 2040 User's Guide, section 6.10):
// DN1 (leftmost) = GP22 ... DN5 (rightmost) = GP18, emitter control DNE = GP26.
#define LS_EMITTER_PIN  26
#define LS_TIMEOUT_US   2500   // a reading equal to this value means very dark

void line_sensors_init(void);

// Reads the five sensors, S1 = DN1 ... S5 = DN5. Each value is the discharge
// time in us: low on white, high on dark.
void line_sensors_read(uint16_t out[5]);
