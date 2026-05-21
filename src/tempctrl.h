#ifndef TEMPCTRL_H
#define TEMPCTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "eigsep_command.h"
#include "temp_simple.h"

// LNA Temperature Control configuration
#define TEMP_SENSOR_LNA_PIN     27  // thermistor data pin
#define PELTIER_LNA_PWM_PIN     8   // enable1
#define PELTIER_LNA_DIR_PIN1    10  // in1
#define PELTIER_LNA_DIR_PIN2    12  // in2

// LOAD Temperature Control configuration
#define TEMP_SENSOR_LOAD_PIN    26
#define PELTIER_LOAD_PWM_PIN    9   // enable2
#define PELTIER_LOAD_DIR_PIN3   11  // in3
#define PELTIER_LOAD_DIR_PIN4   13  // in4

// PWM configuration
#define PWM_WRAP            1000

// Stall detection: if the channel is actively driving (drive!=0) but T_now
// fails to move by at least TEMPCTRL_STALL_MIN_DELTA over a
// TEMPCTRL_STALL_WINDOW_MS window, the sensor or Peltier is stuck and we
// trip the channel. The threshold is far above the DS18B20 quantization
// (1/16 = 0.0625 C) and a healthy half-power Peltier moves the load
// several C/min, so a healthy run rolls the window forward long before
// reaching the trip threshold.
#define TEMPCTRL_STALL_WINDOW_MS   60000
#define TEMPCTRL_STALL_MIN_DELTA   0.5f

// Temperature control structure
typedef struct {
    uint dir_pin1;
    uint dir_pin2;
    uint pwm_pin;
    uint pwm_slice;
    TempSensor temp_sensor;
    float T_now;
    float T_target;
    float drive;
    float gain;
    float baseline;
    float hysteresis;
    float clamp;
    bool active;
    // `enabled` is host intent only — firmware never mutates it. A channel
    // drives only when enabled && none of the trip flags below are set; the
    // host distinguishes "asked off" from "blocked by trip" by reading
    // enabled together with the trip flags.
    bool enabled;
    bool internally_disabled;  // sensor read error (auto-derived each cycle)
    // Stall guard: sticky fault tripped when an active drive fails to move
    // T_now. Cleared by an explicit *_enable=true command from the host
    // (mirrors the watchdog ack pattern).
    bool stall_tripped;
    bool stall_window_active;
    float stall_check_T;
    absolute_time_t stall_check_time;
} TempControl;

// Standard app interface functions
void tempctrl_init(uint8_t app_id);
void tempctrl_server(uint8_t app_id, const char *json_str);
void tempctrl_op(uint8_t app_id);
void tempctrl_status(uint8_t app_id);

#endif // TEMPCTRL_H
