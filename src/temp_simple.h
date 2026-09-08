#ifndef TEMP_SIMPLE_H
#define TEMP_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

// Shared ADC sampling helpers, used by tempctrl's LOAD channel and by
// rfswitch for its PCB thermistors (which report raw volts — conversion
// happens host-side for those). The ADC conversion is effectively
// instantaneous, so every read takes a fresh sample; the caller owns the
// sampling cadence.
#define THERMISTOR_ADC_MAX_COUNTS     4095.0f
#define THERMISTOR_SUPPLY_VOLTS       3.3f

// adc_channel_init validates the pin is ADC-capable (GPIO 26-29), maps
// it to the ADC input index, and performs one-time ADC + pin setup.
bool adc_channel_init(uint gpio_pin, uint *adc_input);
// Average THERMISTOR_ADC_SAMPLES conversions (discarding the first
// sample after the mux switch) and return the pin voltage in volts.
float adc_read_avg_voltage(uint adc_input);

#endif // TEMP_SIMPLE_H
