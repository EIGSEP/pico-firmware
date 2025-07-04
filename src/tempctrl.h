#ifndef TEMPCTRL_H
#define TEMPCTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"
#include "tempmon.h"

// Temperature Control 1 configuration
#define PELTIER1_PWM_PIN    16
#define PELTIER1_DIR_PIN1   18
#define PELTIER1_DIR_PIN2   19

// Temperature Control 2 configuration
#define PELTIER2_PWM_PIN    15
#define PELTIER2_DIR_PIN3   13
#define PELTIER2_DIR_PIN4   12

// Temperature sensor pins
#define TEMP_SENSOR1_PIN    21
#define TEMP_SENSOR2_PIN    22

// PWM configuration
#define PWM_WRAP            1000

// Temperature control structure
typedef struct {
    uint pwm_slice;
    float T_now;
    float T_target;
    float drive;
    float gain;
    float hysteresis;
    bool active;
    bool enabled;
    int channel;
} TempControl;

// Standard app interface functions
void tempctrl_init(uint8_t app_id);
void tempctrl_server(uint8_t app_id, const char *json_str);
void tempctrl_op(uint8_t app_id);
void tempctrl_status(uint8_t app_id);

#endif // TEMPCTRL_H
