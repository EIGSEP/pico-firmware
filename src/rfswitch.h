#ifndef RFSWITCH_H
#define RFSWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "eigsep_command.h"

// define rfswitch pins
#define RFSWITCH0_PIN   8
#define RFSWITCH1_PIN   9
#define RFSWITCH2_PIN  10
#define RFSWITCH3_PIN  11
#define RFSWITCH4_PIN  12
#define RFSWITCH5_PIN  13
#define RFSWITCH6_PIN  14
#define RFSWITCH7_PIN  15

// Sentinel sw_state value reported while the physical switch is still
// settling. Downstream consumers treat this as "state unknown".
#define SW_STATE_UNKNOWN (-1)

// Time the firmware waits after driving new GPIO levels before it
// trusts the physical RF switch to have settled. Measured estimate is
// ~200 ms; revise once a bench measurement tightens the number.
#define SWITCH_SETTLE_MS 200


typedef struct {
    int commanded_state;        // state driven to GPIOs right now
    int reported_state;         // last state the firmware trusts as settled
    bool in_transition;         // true while waiting for settle timer
    absolute_time_t transition_end;
    uint pins[8];
} RFSwitch;

// report rfswitch status
void rfswitch_init(uint8_t);
void rfswitch_server(uint8_t, const char *);
void rfswitch_op(uint8_t);
void rfswitch_status(uint8_t);

#endif // RFSWITCH_H
