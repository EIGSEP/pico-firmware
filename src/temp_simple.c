#include "temp_simple.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include <math.h>

#define THERMISTOR_ADC_SAMPLES 16

static bool adc_ready = false;

static bool adc_input_from_gpio(uint gpio_pin, uint *adc_input) {
    if (gpio_pin < 26 || gpio_pin > 29) {
        return false;
    }
    *adc_input = gpio_pin - 26;
    return true;
}

static float temp_sensor_read_voltage(const TempSensor *sensor) {
    uint32_t total = 0;

    adc_select_input(sensor->adc_input);
    (void)adc_read();  // discard first sample after mux switch

    for (uint i = 0; i < THERMISTOR_ADC_SAMPLES; i++) {
        total += adc_read();
    }

    float counts = (float)total / (float)THERMISTOR_ADC_SAMPLES;
    return counts * THERMISTOR_SUPPLY_VOLTS / THERMISTOR_ADC_MAX_COUNTS;
}

static bool temp_sensor_voltage_to_temperature(float voltage,
                                               float *resistance,
                                               float *temperature) {
    if (!isfinite(voltage) ||
        voltage <= 0.0f ||
        voltage >= THERMISTOR_SUPPLY_VOLTS) {
        return false;
    }

    float r_thermistor = THERMISTOR_TOP_OHMS
        * voltage / (THERMISTOR_SUPPLY_VOLTS - voltage);
    if (!isfinite(r_thermistor) || r_thermistor <= 0.0f) {
        return false;
    }

    float log_r = logf(r_thermistor);
    float inverse_kelvin = THERMISTOR_SH_A
        + THERMISTOR_SH_B * log_r
        + THERMISTOR_SH_C * log_r * log_r * log_r;
    if (!isfinite(inverse_kelvin) || inverse_kelvin <= 0.0f) {
        return false;
    }

    float temp_c = 1.0f / inverse_kelvin - 273.15f;
    if (!isfinite(temp_c) || temp_c < -55.0f || temp_c > 125.0f) {
        return false;
    }

    *resistance = r_thermistor;
    *temperature = temp_c;
    return true;
}

void temp_sensor_init(TempSensor *sensor, uint gpio_pin) {
    sensor->gpio_pin = gpio_pin;
    sensor->adc_input = 0;
    sensor->temperature = 0.0f;
    sensor->voltage = 0.0f;
    sensor->resistance = 0.0f;
    sensor->last_conversion_time = 0;
    sensor->adc_configured = false;
    sensor->conversion_started = false;
    sensor->read_error = false;

    if (!adc_input_from_gpio(gpio_pin, &sensor->adc_input)) {
        sensor->read_error = true;
        return;
    }

    if (!adc_ready) {
        adc_init();
        adc_ready = true;
    }

    adc_gpio_init(gpio_pin);
    sensor->adc_configured = true;

    temp_sensor_start_conversion(sensor);
}

void temp_sensor_start_conversion(TempSensor *sensor) {
    if (!sensor->adc_configured) {
        sensor->read_error = true;
        return;
    }

    sensor->last_conversion_time = to_ms_since_boot(get_absolute_time());
    sensor->conversion_started = true;
}

bool temp_sensor_read(TempSensor *sensor) {
    if (!sensor->adc_configured) {
        sensor->read_error = true;
        return false;
    }

    if (!sensor->conversion_started) {
        return false;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - sensor->last_conversion_time) < THERMISTOR_SAMPLE_INTERVAL_MS) {
        return false;
    }

    float voltage = temp_sensor_read_voltage(sensor);
    float resistance = 0.0f;
    float temperature = 0.0f;
    if (!temp_sensor_voltage_to_temperature(voltage, &resistance, &temperature)) {
        sensor->read_error = true;
        sensor->conversion_started = false;
        return false;
    }

    sensor->voltage = voltage;
    sensor->resistance = resistance;
    sensor->temperature = temperature;
    sensor->last_conversion_time = now;
    sensor->conversion_started = false;
    sensor->read_error = false;
    return true;
}

float temp_sensor_get_temp(TempSensor *sensor) {
    return sensor->temperature;
}

uint32_t temp_sensor_get_conversion_time(TempSensor *sensor) {
    return sensor->last_conversion_time;
}

bool temp_sensor_has_error(TempSensor *sensor) {
    return sensor->read_error;
}
