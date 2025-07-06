#ifndef TEMPCTRL_H
#define TEMPCTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"
#include "tempmon.h"

// Temperature Control 1 configuration
#define TEMP_SENSOR1_PIN    21  // thermistor data pin
#define PELTIER1_PWM_PIN    8  // enable1
#define PELTIER1_DIR_PIN1   10  // in1
#define PELTIER1_DIR_PIN2   12  // in2

// Temperature Control 2 configuration
#define TEMP_SENSOR2_PIN    22
#define PELTIER2_PWM_PIN    9  // enable2
#define PELTIER2_DIR_PIN3   11  // in3
#define PELTIER2_DIR_PIN4   13  // in4

// PWM configuration
#define PWM_WRAP            1000

// Error handling configuration
#define ERROR_COUNT_THRESHOLD   5     // Number of errors before permanent disable
#define ERROR_TIME_WINDOW_MS    10000 // Time window for error counting (10 seconds)

// Temperature control structure
typedef struct {
    uint pwm_slice;
    float T_now;
    float T_target;
    float drive;
    float gain;
    float hysteresis;
    float clamp;
    bool active;
    bool enabled;
    bool permanently_disabled;
    int channel;
    uint32_t error_count;
    uint32_t last_error_time;
} TempControl;

// Standard app interface functions
void tempctrl_init(uint8_t app_id);
void tempctrl_server(uint8_t app_id, const char *json_str);
void tempctrl_op(uint8_t app_id);
void tempctrl_status(uint8_t app_id);

#endif // TEMPCTRL_H
