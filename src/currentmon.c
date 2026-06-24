#include "currentmon.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

#define CURRENTMON_GPIO         28
#define CURRENTMON_ADC_CH       2
#define CURRENTMON_ADC_SAMPLES  16
#define CURRENTMON_ADC_MAX      4095.0f
#define CURRENTMON_VREF         3.3f

static float current_voltage = 0.0f;

void currentmon_init(void) {
    adc_init();
    adc_gpio_init(CURRENTMON_GPIO);
}

void currentmon_op(void) {
    adc_select_input(CURRENTMON_ADC_CH);
    (void)adc_read();  // discard first conversion after selecting the input

    uint32_t total = 0;
    for (uint i = 0; i < CURRENTMON_ADC_SAMPLES; i++) {
        total += adc_read();
    }
    float counts = (float)total / (float)CURRENTMON_ADC_SAMPLES;
    current_voltage = counts * CURRENTMON_VREF / CURRENTMON_ADC_MAX;
}

float currentmon_voltage(void) {
    return current_voltage;
}
