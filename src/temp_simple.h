#ifndef TEMP_SIMPLE_H
#define TEMP_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// ADC thermistor helper for the tempctrl app.  The hardware keeps the existing
// sensor GPIOs (GP26/ADC0 and GP27/ADC1) and uses an external fixed resistor
// from 3V3 to the ADC node, with the thermistor from the ADC node to AGND.
#define THERMISTOR_SAMPLE_INTERVAL_MS 250
#define THERMISTOR_ADC_MAX_COUNTS     4095.0f
#define THERMISTOR_SUPPLY_VOLTS       3.3f
#define THERMISTOR_FIXED_OHMS         10680.0f

// Steinhart-Hart coefficients for resistance in ohms:
// 95339.0 ohms at 0 C, 16212.0 ohms at 40 C, 5387.4 ohms at 70 C.
#define THERMISTOR_SH_A               9.2463455e-4f
#define THERMISTOR_SH_B               2.2246310e-4f
#define THERMISTOR_SH_C               1.2326590e-7f

// Temperature sensor structure for direct ADC connection.
typedef struct {
    uint gpio_pin;
    uint adc_input;
    float temperature;
    float voltage;
    float resistance;
    uint32_t last_sample_time;
    bool adc_configured;
    bool conversion_started;
    bool read_error;
} TempSensor;

// Initialize a temperature sensor on a specific GPIO pin. The PIO arguments
// remain in the API so tempctrl can keep its existing initialization shape.
void temp_sensor_init(TempSensor *sensor, uint gpio_pin, PIO pio, uint sm_offset);

// Start a timed sampling window.
void temp_sensor_start_conversion(TempSensor *sensor);

// Attempt to read a new temperature sample. Returns true exactly when a fresh
// ADC sample was just decoded this call, so callers gating on new data can
// avoid integrating on stale ticks.
bool temp_sensor_read(TempSensor *sensor);

// Get current temperature value
float temp_sensor_get_temp(TempSensor *sensor);

// Returns the absolute time (ms since boot) of the last accepted ADC sample.
uint32_t temp_sensor_get_conversion_time(TempSensor *sensor);

// Get error status
bool temp_sensor_has_error(TempSensor *sensor);

#endif // TEMP_SIMPLE_H
