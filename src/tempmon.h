#ifndef TEMPMON_H
#define TEMPMON_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"

// Temperature sensor pins
#define TEMPMON_SENSOR1_PIN         19
#define TEMPMON_SENSOR2_PIN         21

// Temperature monitoring structure
typedef struct {
    float temperature1;
    float temperature2;
    bool sensor1_valid;
    bool sensor2_valid;
    bool initialized;
    time_t last_read;
} TempMonitor;

// Standard app interface functions
void tempmon_init(uint8_t app_id);
void tempmon_server(uint8_t app_id, const char *json_str);
void tempmon_op(uint8_t app_id);
void tempmon_status(uint8_t app_id);

// Temperature monitoring functions
bool tempmon_read_sensors(void);
float tempmon_get_temperature1(void);
float tempmon_get_temperature2(void);

#endif // TEMPMON_H
