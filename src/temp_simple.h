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
    uint32_t last_conversion_time;
    bool conversion_started;
    bool read_error;
} TempSensor;

// Initialize a temperature sensor on a specific GPIO pin
void temp_sensor_init(TempSensor *sensor, uint gpio_pin, PIO pio, uint sm_offset);

// Start temperature conversion
void temp_sensor_start_conversion(TempSensor *sensor);

// Read temperature (returns true if successful)
void temp_sensor_read(TempSensor *sensor);

// Get current temperature value
float temp_sensor_get_temp(TempSensor *sensor);
float temp_sensor_get_conversion_time(TempSensor *sensor);

// Get error status
bool temp_sensor_has_error(TempSensor *sensor);

#endif // TEMP_SIMPLE_H
