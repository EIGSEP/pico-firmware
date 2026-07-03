#ifndef TEMP_SIMPLE_H
#define TEMP_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

// ADC thermistor helper for the tempctrl app. The existing tempctrl app shape
// is preserved; only the private TempSensor backend reads an ADC divider.
// The ADC conversion is effectively instantaneous, so every read takes a
// fresh sample; the caller owns the sampling cadence (tempctrl samples on a
// fixed TEMPCTRL_SAMPLE_MS timer).
#define THERMISTOR_ADC_MAX_COUNTS     4095.0f
#define THERMISTOR_SUPPLY_VOLTS       3.3f
#define THERMISTOR_FIXED_OHMS         10680.0f
#define THERMISTOR_BOARD_PULLUP_OHMS  4700.0f
#define THERMISTOR_TOP_OHMS           \
    ((THERMISTOR_FIXED_OHMS * THERMISTOR_BOARD_PULLUP_OHMS) / \
     (THERMISTOR_FIXED_OHMS + THERMISTOR_BOARD_PULLUP_OHMS))

// Vishay NTCLE100E3103 NTC, 10 kOhm at 25 C, B25/85 = 3977 K.
// Extended Steinhart-Hart fit from the datasheet (document 29049,
// "Mat A" coefficient row), specified for -40..+125 C:
//   1/T = A1 + B1 ln(R/Rref) + C1 ln^2(R/Rref) + D1 ln^3(R/Rref)
// with T in kelvin and Rref the 25 C resistance.
#define THERMISTOR_REF_OHMS           10000.0f
#define THERMISTOR_SH_A1              3.354016e-3f
#define THERMISTOR_SH_B1              2.569850e-4f
#define THERMISTOR_SH_C1              2.620131e-6f
#define THERMISTOR_SH_D1              6.383091e-8f

// Shared ADC sampling helpers (also used by rfswitch for its PCB
// thermistors, which report raw volts — conversion happens host-side).
// adc_channel_init validates the pin is ADC-capable (GPIO 26-29), maps
// it to the ADC input index, and performs one-time ADC + pin setup.
bool adc_channel_init(uint gpio_pin, uint *adc_input);
// Average THERMISTOR_ADC_SAMPLES conversions (discarding the first
// sample after the mux switch) and return the pin voltage in volts.
float adc_read_avg_voltage(uint adc_input);

// Temperature sensor structure for direct ADC connection.
typedef struct {
    uint gpio_pin;
    uint adc_input;
    float temperature;
    float voltage;
    float resistance;
    uint32_t last_sample_time;
    bool adc_configured;
    bool read_error;
} TempSensor;

// Initialize a temperature sensor on a specific ADC-capable GPIO pin.
void temp_sensor_init(TempSensor *sensor, uint gpio_pin);

// Read a fresh temperature sample from the ADC. Returns true when a sample
// was decoded this call (so callers gating on new data — e.g. a PI
// controller — can skip ticks with no valid sample). Returns false when the
// plausibility conversion failed (see temp_sensor_has_error()); `voltage` is
// still updated with the measured value in that case — only `temperature`,
// `resistance`, and `last_sample_time` hold their last-good values.
bool temp_sensor_read(TempSensor *sensor);

// Get current temperature value
float temp_sensor_get_temp(TempSensor *sensor);

// Returns the absolute time (ms since boot) of the last accepted ADC sample.
uint32_t temp_sensor_get_sample_time(TempSensor *sensor);

// Get error status
bool temp_sensor_has_error(TempSensor *sensor);

#endif // TEMP_SIMPLE_H
