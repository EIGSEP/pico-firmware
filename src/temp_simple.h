#ifndef TEMP_SIMPLE_H
#define TEMP_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "onewire_library.h"
#include "ds18b20.h"

// Temperature sensor structure for direct GPIO connection
typedef struct {
    OW ow;
    uint gpio_pin;
    float temperature;
    bool valid;
    uint32_t last_conversion_time;
    bool conversion_started;
} TempSensor;

// Initialize a temperature sensor on a specific GPIO pin
bool temp_sensor_init(TempSensor *sensor, uint gpio_pin, PIO pio, uint sm_offset);

// Start temperature conversion
void temp_sensor_start_conversion(TempSensor *sensor);

// Read temperature (returns true if successful)
bool temp_sensor_read(TempSensor *sensor);

// Get current temperature value
float temp_sensor_get_temp(TempSensor *sensor);

// Check if sensor is valid
bool temp_sensor_is_valid(TempSensor *sensor);

#endif // TEMP_SIMPLE_H