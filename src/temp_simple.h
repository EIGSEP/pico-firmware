#ifndef TEMP_SIMPLE_H
#define TEMP_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

// ADC thermistor helper for the tempctrl app. The existing tempctrl app shape
// is preserved; only the private TempSensor backend reads an ADC divider.
#define THERMISTOR_SAMPLE_INTERVAL_MS 750
#define THERMISTOR_ADC_MAX_COUNTS     4095.0f
#define THERMISTOR_SUPPLY_VOLTS       3.3f
#define THERMISTOR_FIXED_OHMS         10680.0f
#define THERMISTOR_BOARD_PULLUP_OHMS  4700.0f
#define THERMISTOR_TOP_OHMS           \
    ((THERMISTOR_FIXED_OHMS * THERMISTOR_BOARD_PULLUP_OHMS) / \
     (THERMISTOR_FIXED_OHMS + THERMISTOR_BOARD_PULLUP_OHMS))

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
    uint32_t last_conversion_time;
    bool adc_configured;
    bool conversion_started;
    bool read_error;
} TempSensor;

// Initialize a temperature sensor on a specific ADC-capable GPIO pin.
void temp_sensor_init(TempSensor *sensor, uint gpio_pin);

// Start a timed sampling window.
void temp_sensor_start_conversion(TempSensor *sensor);

// Attempt to read a new temperature sample. Returns true exactly when
// a fresh sample was just decoded this call (so callers gating on new
// data — e.g. a PI controller — can avoid integrating on stale ticks).
// Returns false when the sample interval has not elapsed, or when the read
// failed (see temp_sensor_has_error()).
bool temp_sensor_read(TempSensor *sensor);

// Get current temperature value
float temp_sensor_get_temp(TempSensor *sensor);

// Returns the absolute time (ms since boot) of the last accepted ADC sample.
uint32_t temp_sensor_get_conversion_time(TempSensor *sensor);

// Get error status
bool temp_sensor_has_error(TempSensor *sensor);

#endif // TEMP_SIMPLE_H
