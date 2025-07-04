#ifndef RFSWITCH_H
#define RFSWITCH_H

#include <stdint.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"

// define rfswitch pins
#define RFSWITCH4_PIN   5
#define RFSWITCH2_PIN   6
#define RFSWITCH3_PIN   7
#define RFSWITCH1_PIN   8
#define RFSWITCH0_PIN   9
#define RFSWITCH6_PIN  10
#define RFSWITCH7_PIN  11
#define RFSWITCH5_PIN  12


typedef struct {
    int sw_state;
    uint pins[8];
} RFSwitch;

// report rfswitch status
void rfswitch_init(uint8_t);
void rfswitch_server(uint8_t, const char *);
void rfswitch_op(uint8_t);
void rfswitch_status(uint8_t);

#endif // RFSWITCH_H

