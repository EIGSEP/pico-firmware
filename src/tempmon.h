#ifndef TEMPMON_H
#define TEMPMON_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "hardware/gpio.h"
#include "eigsep_command.h"

// OneWire DS18B20 temperature sensor
#define TEMPMON_DS_PIN              22

// Temperature sensor structure
typedef struct {
    uint64_t rom_code;
    float temperature;
    time_t last_read;
    bool valid;
    char sensor_id[17];  // 16 hex chars + null terminator
} TempSensor;

// Temperature monitoring structure
typedef struct {
    TempSensor sensors[8];  // Support up to 8 sensors
    int sensor_count;
    bool initialized;
    uint32_t read_interval_ms;
    uint32_t last_read_time;
} TempMonitor;

// Standard app interface functions
void tempmon_init(uint8_t app_id);
void tempmon_server(uint8_t app_id, const char *json_str);
void tempmon_op(uint8_t app_id);
void tempmon_status(uint8_t app_id);

// Temperature monitoring functions
bool tempmon_read_sensors(void);
float tempmon_get_temperature(uint64_t rom_code);
int tempmon_get_sensor_count(void);
bool tempmon_get_sensor_by_index(int index, TempSensor *sensor);
bool tempmon_search_sensors(void);

#endif // TEMPMON_H