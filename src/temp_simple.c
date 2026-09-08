#include "temp_simple.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

#define THERMISTOR_ADC_SAMPLES 16

static bool adc_ready = false;

bool adc_channel_init(uint gpio_pin, uint *adc_input) {
    if (gpio_pin < 26 || gpio_pin > 29) {
        return false;
    }
    *adc_input = gpio_pin - 26;

    if (!adc_ready) {
        adc_init();
        adc_ready = true;
    }
    adc_gpio_init(gpio_pin);
    return true;
}

float adc_read_avg_voltage(uint adc_input) {
    uint32_t total = 0;

    adc_select_input(adc_input);
    (void)adc_read();  // discard first sample after mux switch

    for (uint i = 0; i < THERMISTOR_ADC_SAMPLES; i++) {
        total += adc_read();
    }

    float counts = (float)total / (float)THERMISTOR_ADC_SAMPLES;
    return counts * THERMISTOR_SUPPLY_VOLTS / THERMISTOR_ADC_MAX_COUNTS;
}
