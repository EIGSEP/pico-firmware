#ifndef TEMPMON_H
#define TEMPMON_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"

// Temperature sensor pins (LNA = sensor 1, LOAD = sensor 2)
#define TEMPMON_SENSOR1_PIN         26
#define TEMPMON_SENSOR2_PIN         27

// Standard app interface functions
void tempmon_init(uint8_t app_id);
void tempmon_server(uint8_t app_id, const char *json_str);
void tempmon_op(uint8_t app_id);
void tempmon_status(uint8_t app_id);

#endif // TEMPMON_H
